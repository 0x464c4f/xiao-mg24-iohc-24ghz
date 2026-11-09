/***************************************************************************//**
 * @file app.c
 * @brief Standalone 2.4 GHz io-homecontrol 1-Way Dual-Device Controller
 *        for Seeed Studio XIAO MG24 (EFR32MG24B220F1536IM48)
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Credits & Attributions:
 * - Signal discovery, EFR32MG24 Z2xZ8 DSSS correlator configuration, two-stage
 *   LPM wakeup framing, Flash NVM wear-leveling, and CLI architecture developed
 *   with Google Antigravity (AGY) powered by Gemini.
 * - 1-Way MAC transformation (`iohc_checksum_step` / `iohc_create_1w_hmac`)
 *   adapted from `rspaargaren/iohomecontrol` (Apache-2.0).
 ******************************************************************************/
#include "app.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "em_device.h"
#include "em_core.h"
#include "sl_gpio.h"
#include "sl_cli.h"
#include "rail.h"
#include "app_common.h"
#include "response_print.h"

#if defined(__has_include)
  #if __has_include("velux_secrets.h")
    #include "velux_secrets.h"
  #endif
#endif

static void velux_on_rx_valid_frame(const uint8_t *psdu, uint16_t len);

// -----------------------------------------------------------------------------
// Constants and Configuration
// -----------------------------------------------------------------------------
#define BURST_BUFFER_SIZE           (64 * 1024)   // 64 KB RAM buffer
#define BURST_IDLE_TIMEOUT_US       120000        // 120 ms radio silence marks burst end
#define FIFO_STREAM_CAPTURE_BYTES   12000         // 12,000 bytes = 384 ms continuous capture

// -----------------------------------------------------------------------------
// Packet Storage Header
// -----------------------------------------------------------------------------
typedef struct __attribute__((packed)) {
  uint32_t timestamp_us;
  int8_t   rssi;
  uint8_t  lqi;
  uint16_t length;
  uint8_t  packet_status;
  uint8_t  sub_phy_id;
  int16_t  freq_offset;
  uint16_t channel;
} BurstPacketHeader_t;

// -----------------------------------------------------------------------------
// Velux Direct Hardware Burst Control Struct
// -----------------------------------------------------------------------------
typedef struct {
  volatile uint32_t trigger;        // +0x00: 1=dual, 2=replay, 3=LPM, 4=short, 5=rapid 333us, 6=1ms hop, 7=self-test
  volatile uint32_t num_bursts;     // +0x04: default 1 (or e.g. 2 back-to-back bursts)
  uint8_t           psdu[28];       // +0x08: bytes 0..27 of PSDU
  uint8_t           last_rx_psdu[28]; // +0x24: auto-populated whenever RX decodes a valid frame!
  volatile uint32_t last_rx_valid;  // +0x40
  volatile uint32_t tx_done_count;  // +0x44
  volatile uint32_t psdu_len;       // +0x48: 0 or 28 = 28B, 23 = 23B (pair), 34 = 34B (add)
  uint8_t           psdu_ext[12];   // +0x4C: bytes 28..39 when psdu_len > 28
  volatile uint32_t last_rx_len;    // +0x58
  uint8_t           last_rx_ext[12];// +0x5C
} VeluxTxControl_t;

__attribute__((used, aligned(4))) VeluxTxControl_t g_velux_tx = {0};

// -----------------------------------------------------------------------------
// Static Variables in RAM
// -----------------------------------------------------------------------------
static uint8_t  g_burst_buffer[BURST_BUFFER_SIZE];
static uint32_t g_burst_write_idx = 0;
static uint32_t g_burst_packet_count = 0;
static bool     g_burst_in_progress = false;
static bool     g_burst_overflow = false;
static uint32_t g_last_rx_time = 0;

// -----------------------------------------------------------------------------
// Kermit CRC-16 (poly 0x8408 reflected, init 0x0000)
// -----------------------------------------------------------------------------
static uint16_t kermit_crc16(const uint8_t *data, uint16_t len)
{
  uint16_t crc = 0x0000;
  for (uint16_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (int b = 0; b < 8; b++) {
      uint16_t rem = (crc & 1) ? 0x8408 : 0;
      crc = (uint16_t)((crc >> 1) ^ rem);
    }
  }
  return crc;
}

// -----------------------------------------------------------------------------
// Group Inversion helper on Z2 x Z8 symbol space
// -----------------------------------------------------------------------------
static inline uint8_t inv_nibble(uint8_t n, uint8_t P)
{
  return (uint8_t)((((n + 8 - (P & 7)) & 7) | ((n ^ P) & 8)));
}

static inline uint8_t get_nibble(const uint8_t *payload, uint32_t nib_idx)
{
  return (nib_idx & 1) ? ((payload[nib_idx >> 1] >> 4) & 0x0f)
                       : (payload[nib_idx >> 1] & 0x0f);
}

// -----------------------------------------------------------------------------
// Dump Captured Burst cleanly over serial
// -----------------------------------------------------------------------------
static void dump_burst_over_serial(void)
{
  if (g_burst_packet_count == 0) {
    return;
  }

  printf("\n--- BURST CAPTURE DUMP START (count=%lu, bytes=%lu, ovf=%d) ---\n",
         (unsigned long)g_burst_packet_count,
         (unsigned long)g_burst_write_idx,
         g_burst_overflow ? 1 : 0);

  uint32_t offset = 0;
  uint32_t first_t = 0;

  for (uint32_t i = 0; i < g_burst_packet_count && offset < g_burst_write_idx; i++) {
    const BurstPacketHeader_t *hdr = (const BurstPacketHeader_t *)&g_burst_buffer[offset];
    offset += sizeof(BurstPacketHeader_t);

    if (i == 0) {
      first_t = hdr->timestamp_us;
    }
    int32_t dt_ms = (int32_t)((hdr->timestamp_us - first_t) / 1000);

    const uint8_t *payload = &g_burst_buffer[offset];
    offset += hdr->length;

    // 1. Print raw packet line
    printf("[%03lu] +%4ld ms | ch:%2u | rssi:%4d | lqi:%3u | st:%02x | len:%2u | ",
           (unsigned long)i,
           (long)dt_ms,
           hdr->channel,
           hdr->rssi,
           hdr->lqi,
           hdr->packet_status,
           hdr->length);

    for (uint16_t b = 0; b < hdr->length; b++) {
      printf("%02x ", payload[b]);
    }
    printf("\n");

    // 2. Case 1: direct SFD=0x56 lock at byte 0
    bool case1_found = false;
    uint8_t case1_phr = 0;
    if (hdr->length >= 4) {
      uint8_t phr = payload[0];
      if (phr >= 3 && phr <= 120 && (uint16_t)(1 + phr) <= hdr->length) {
        if (kermit_crc16(&payload[1], phr) == 0) {
          case1_found = true;
          case1_phr = phr;
          printf("  ==> [CRC-OK] off=0 align=0 P=0 PHR=%02x PSDU: ", phr);
          for (uint16_t k = 0; k < phr; k++) {
            printf("%02x ", payload[1 + k]);
          }
          printf("\n");
          if (phr == 0x1c) {
            printf("  *** NEW COMMAND FRAME DETECTED! ***\n");
            memcpy((void *)g_velux_tx.last_rx_psdu, &payload[1], 28);
            g_velux_tx.last_rx_valid = 1;
          } else if (phr != 5 || (payload[1] != 0x00 || payload[2] != 0x00 || payload[3] != 0x68)) {
            printf("  *** NEW COMMAND FRAME DETECTED! ***\n");
          }
        }
      }
    }

    // 3. Case 2: Group inversion search across both even and odd nibble alignments
    uint16_t total_nibs = hdr->length * 2;
    uint16_t nib_idx = case1_found ? (2 * (1 + case1_phr) + 4) : 4;

    while (nib_idx + 1 < total_nibs) {
      uint8_t P = get_nibble(payload, nib_idx - 1);
      uint16_t pre_count = 0;
      while ((nib_idx - 1 >= pre_count) && (get_nibble(payload, nib_idx - 1 - pre_count) == P)) {
        pre_count++;
      }

      if (pre_count >= 4 &&
          inv_nibble(get_nibble(payload, nib_idx), P) == 6 &&
          inv_nibble(get_nibble(payload, nib_idx + 1), P) == 5) {

        uint16_t off = nib_idx / 2;
        uint8_t align = nib_idx % 2;
        uint16_t avail_bytes = (total_nibs - (nib_idx + 2)) / 2;

        if (avail_bytes >= 1) {
          uint8_t phr = inv_nibble(get_nibble(payload, nib_idx + 2), P) |
                        (inv_nibble(get_nibble(payload, nib_idx + 3), P) << 4);
          uint16_t remaining = avail_bytes - 1;
          uint8_t psdu[128];
          uint16_t copy_len = (remaining < phr) ? remaining : phr;
          if (copy_len > sizeof(psdu)) {
            copy_len = sizeof(psdu);
          }
          for (uint16_t b = 0; b < copy_len; b++) {
            uint8_t n0 = get_nibble(payload, nib_idx + 4 + 2 * b);
            uint8_t n1 = get_nibble(payload, nib_idx + 5 + 2 * b);
            psdu[b] = inv_nibble(n0, P) | (inv_nibble(n1, P) << 4);
          }

          bool crc_ok = false;
          if (remaining >= phr && phr >= 3 && phr <= sizeof(psdu)) {
            if (kermit_crc16(psdu, phr) == 0) {
              crc_ok = true;
            }
          }

          if (crc_ok) {
            printf("  ==> [CRC-OK] off=%u align=%u P=%u PHR=%02x PSDU: ", off, align, P, phr);
            for (uint16_t b = 0; b < phr; b++) {
              printf("%02x ", psdu[b]);
            }
            printf("\n");
            if (phr == 0x1c) {
              printf("  *** NEW COMMAND FRAME DETECTED! ***\n");
              memcpy((void *)g_velux_tx.last_rx_psdu, psdu, 28);
              g_velux_tx.last_rx_valid = 1;
              velux_on_rx_valid_frame(psdu, phr);
            } else if (phr != 5 || (psdu[0] != 0x00 || psdu[1] != 0x00 || psdu[2] != 0x68)) {
              printf("  *** NEW COMMAND FRAME DETECTED! ***\n");
              if (phr >= 0x0A && phr <= 0x24) {
                velux_on_rx_valid_frame(psdu, phr);
              }
            }
            nib_idx += 2 + 2 * (1 + phr);
            continue;
          } else if (pre_count >= 6) {
            uint16_t print_len = (phr > 0 && phr <= remaining) ? phr : remaining;
            if (print_len > sizeof(psdu)) {
              print_len = sizeof(psdu);
            }
            printf("  --> [SFD-LOCK] off=%u align=%u P=%u PHR=%02x (avail=%u) RAW_INV: ",
                   off, align, P, phr, (unsigned int)remaining);
            for (uint16_t b = 0; b < print_len; b++) {
              uint8_t n0 = get_nibble(payload, nib_idx + 4 + 2 * b);
              uint8_t n1 = get_nibble(payload, nib_idx + 5 + 2 * b);
              printf("%02x ", (unsigned int)(inv_nibble(n0, P) | (inv_nibble(n1, P) << 4)));
            }
            printf("\n");
            nib_idx += 2;
            continue;
          }
        }
      }
      nib_idx++;
    }
  }

  printf("--- BURST CAPTURE DUMP END ---\n\n");
}

// -----------------------------------------------------------------------------
// Dump Continuous FIFO Stream Capture over serial
// -----------------------------------------------------------------------------
static void dump_fifo_stream_over_serial(void)
{
  if (g_burst_write_idx == 0) {
    return;
  }

  uint32_t duration_ms = (g_burst_write_idx * 32UL) / 1000UL;
  printf("\n--- FIFO STREAM CAPTURE DUMP START (bytes=%lu, duration_ms=%lu, ovf=%d) ---\n",
         (unsigned long)g_burst_write_idx,
         (unsigned long)duration_ms,
         g_burst_overflow ? 1 : 0);

  uint32_t total_crc_ok = 0;
  uint32_t total_new_cmd = 0;
  bool printed_first_beacon = false;

  // Scan full captured stream (up to 40,000 bytes = 1,280 ms)
  uint32_t scan_bytes = g_burst_write_idx;
  uint32_t total_nibs = scan_bytes * 2;

  // Pass 1: Strobe & Preamble-Locked Frames
  uint32_t i = 4;
  while (i + 1 < total_nibs) {
    uint8_t P = get_nibble(g_burst_buffer, i - 1);
    uint32_t pre_count = 0;
    while ((i - 1 >= pre_count) && (get_nibble(g_burst_buffer, i - 1 - pre_count) == P)) {
      pre_count++;
    }

    if (pre_count >= 4 &&
        inv_nibble(get_nibble(g_burst_buffer, i), P) == 6 &&
        inv_nibble(get_nibble(g_burst_buffer, i + 1), P) == 5) {

      uint32_t off = i / 2;
      uint8_t align = (uint8_t)(i % 2);
      uint32_t t_ms = (off * 32UL) / 1000UL;
      uint32_t t_us_rem = ((off * 32UL) % 1000UL) / 100UL;
      uint32_t avail_nibbles = total_nibs - (i + 2);
      uint32_t avail_bytes = avail_nibbles / 2;

      if (avail_bytes >= 1) {
        uint8_t phr = inv_nibble(get_nibble(g_burst_buffer, i + 2), P) |
                      (inv_nibble(get_nibble(g_burst_buffer, i + 3), P) << 4);
        uint32_t remaining = avail_bytes - 1;

        if (remaining >= phr && phr >= 3 && phr <= 36) {
          uint8_t psdu[36];
          for (uint32_t b = 0; b < phr; b++) {
            uint8_t n0 = get_nibble(g_burst_buffer, i + 4 + 2 * b);
            uint8_t n1 = get_nibble(g_burst_buffer, i + 5 + 2 * b);
            psdu[b] = inv_nibble(n0, P) | (inv_nibble(n1, P) << 4);
          }

          if (kermit_crc16(psdu, phr) == 0) {
            bool print_it = false;
            if (phr >= 0x0A && phr <= 0x24) {
              print_it = true;
              total_new_cmd++;
            } else if (phr == 0x05 && psdu[0] == 0x00 && psdu[1] == 0x00 && psdu[2] <= 0x6A) {
              uint8_t count8 = psdu[2];
              if (!printed_first_beacon || count8 >= 0x68 || count8 <= 0x08) {
                print_it = true;
                printed_first_beacon = true;
              }
            }

            if (print_it) {
              total_crc_ok++;
              printf("[HOP] +%4lu.%01lu ms (off=%5lu align=%u P=%2u) | [CRC-OK] PHR=%02x PSDU: ",
                     (unsigned long)t_ms, (unsigned long)t_us_rem, (unsigned long)off, align, P, phr);
              for (uint32_t b = 0; b < phr; b++) {
                printf("%02x ", psdu[b]);
              }
              printf("\n");
              if (phr >= 0x0A && phr <= 0x24) {
                printf("  *** NEW COMMAND FRAME DETECTED! ***\n");
                uint32_t c0 = (phr <= 28) ? phr : 28;
                memcpy((void *)g_velux_tx.last_rx_psdu, psdu, c0);
                if (phr > 28) {
                  memcpy((void *)g_velux_tx.last_rx_ext, &psdu[28], phr - 28);
                }
                g_velux_tx.last_rx_len = phr;
                g_velux_tx.last_rx_valid = 1;
                velux_on_rx_valid_frame(psdu, phr);
              }
            }
            i += 2 + 2 * (1 + phr);
            continue;
          }
        }
      }
    }
    i++;
  }

  // Pass 2: Exhaustive Command/Pair/Add Search (10B..36B) across the entire scan range across all 16 phases
  uint32_t start_nib = 4;
  uint32_t end_nib = (total_nibs > (2 + 2 * 35)) ? (total_nibs - (2 + 2 * 35)) : 0;

  uint32_t p2 = start_nib;
  while (p2 < end_nib) {
    bool found_p2 = false;
    for (uint8_t P = 0; P < 16; P++) {
      if (inv_nibble(get_nibble(g_burst_buffer, p2), P) == 6 &&
          inv_nibble(get_nibble(g_burst_buffer, p2 + 1), P) == 5) {
        uint8_t phr = inv_nibble(get_nibble(g_burst_buffer, p2 + 2), P) |
                      (inv_nibble(get_nibble(g_burst_buffer, p2 + 3), P) << 4);
        if ((phr >= 0x0A && phr <= 0x24) &&
            (inv_nibble(get_nibble(g_burst_buffer, p2 + 4), P) | (inv_nibble(get_nibble(g_burst_buffer, p2 + 5), P) << 4)) == 0x01) {
          uint8_t psdu[36];
          for (uint32_t b = 0; b < phr; b++) {
            uint8_t n0 = get_nibble(g_burst_buffer, p2 + 4 + 2 * b);
            uint8_t n1 = get_nibble(g_burst_buffer, p2 + 5 + 2 * b);
            psdu[b] = inv_nibble(n0, P) | (inv_nibble(n1, P) << 4);
          }
          if (kermit_crc16(psdu, phr) == 0) {
            uint32_t off = p2 / 2;
            uint8_t align = (uint8_t)(p2 % 2);
            uint32_t t_ms = (off * 32UL) / 1000UL;
            uint32_t t_us_rem = ((off * 32UL) % 1000UL) / 100UL;
            printf("[EXHAUSTIVE 28B] +%4lu.%01lu ms (off=%5lu align=%u P=%2u) | [CRC-OK] PSDU: ",
                   (unsigned long)t_ms, (unsigned long)t_us_rem, (unsigned long)off, align, P);
            for (uint32_t b = 0; b < phr; b++) {
              printf("%02x ", psdu[b]);
            }
            printf("\n");
            printf("  *** EXHAUSTIVE COMMAND FRAME DETECTED! ***\n");
            uint32_t c0 = (phr <= 28) ? phr : 28;
            memcpy((void *)g_velux_tx.last_rx_psdu, psdu, c0);
            if (phr > 28) {
              memcpy((void *)g_velux_tx.last_rx_ext, &psdu[28], phr - 28);
            }
            g_velux_tx.last_rx_len = phr;
            g_velux_tx.last_rx_valid = 1;
            velux_on_rx_valid_frame(psdu, phr);
            total_new_cmd++;
            total_crc_ok++;
            p2 += 2 + 2 * (1 + phr);
            found_p2 = true;
            break;
          }
        }
      }
    }
    if (!found_p2) {
      p2++;
    }
  }

  printf("--- SUMMARY: CRC-OK=%lu, NEW-COMMAND=%lu ---\n",
         (unsigned long)total_crc_ok,
         (unsigned long)total_new_cmd);
  printf("--- BURST CAPTURE DUMP END ---\n\n");
}

// -----------------------------------------------------------------------------
// Live In-Flight TX Self-Test Telemetry
// -----------------------------------------------------------------------------
typedef struct {
  uint32_t ok_count;
  uint32_t err_count;
  uint32_t beacon_dur_us;
  uint32_t cmd_dur_us;
  uint32_t active_sync0;
  uint32_t active_fcd0;
  uint32_t active_fcd1;
  uint32_t active_wcntcmp0;
  uint32_t active_status;
} TxDiagTelemetry_t;

static TxDiagTelemetry_t g_tx_diag = { .ok_count = 1 }; // In .data to preserve .bss layout

// -----------------------------------------------------------------------------
// Direct Hardware Velux 2.4 GHz Burst Transmitter (Exact 1000.0 us Grid)
// -----------------------------------------------------------------------------
static void tx_at_grid(RAIL_Handle_t railHandle, uint16_t channel, const uint8_t *pkt, uint16_t len, uint32_t t_target_us)
{
  RAIL_Idle(railHandle, RAIL_IDLE_ABORT, true);
  *(volatile uint32_t *)0xB8014078 = 0x56;
  *(volatile uint32_t *)0xB8004108 = 0x000040FF;
  *(volatile uint32_t *)0xB800410C = 0x000040FF;
  *(volatile uint32_t *)0xB8004110 = 0x000040FF;
  *(volatile uint32_t *)0xB8004114 = 0x000040FF;
  RAIL_SetFixedLength(railHandle, len);
  *(volatile uint32_t *)0xB8014078 = 0x56;
  *(volatile uint32_t *)0xB8004108 = 0x000040FF;
  *(volatile uint32_t *)0xB800410C = 0x000040FF;
  *(volatile uint32_t *)0xB8004110 = 0x000040FF;
  *(volatile uint32_t *)0xB8004114 = 0x000040FF;
  RAIL_ResetFifo(railHandle, true, false);
  RAIL_WriteTxFifo(railHandle, pkt, len, true);

  // Spin until exact microsecond t_target_us
  while ((int32_t)(t_target_us - RAIL_GetTime()) > 0) {
  }
  *(volatile uint32_t *)0xB8014078 = 0x56;
  *(volatile uint32_t *)0xB8004108 = 0x000040FF;
  *(volatile uint32_t *)0xB800410C = 0x000040FF;
  *(volatile uint32_t *)0xB8004110 = 0x000040FF;
  *(volatile uint32_t *)0xB8004114 = 0x000040FF;
  uint32_t t0 = RAIL_GetTime();
  RAIL_StartTx(railHandle, channel, RAIL_TX_OPTION_REMOVE_CRC, NULL);

  // 1. Wait until radio actually enters TX state (or 500 us timeout)
  while (!(RAIL_GetRadioState(railHandle) & RAIL_RF_STATE_TX) &&
         ((uint32_t)(RAIL_GetTime() - t0) < 500UL)) {
  }
  if (RAIL_GetRadioState(railHandle) & RAIL_RF_STATE_TX) {
    g_tx_diag.ok_count++;
    g_tx_diag.active_sync0 = *(volatile uint32_t *)0xB8014078;
    g_tx_diag.active_fcd0 = *(volatile uint32_t *)0xB8004108;
    g_tx_diag.active_fcd1 = *(volatile uint32_t *)0xB800410C;
    g_tx_diag.active_wcntcmp0 = *(volatile uint32_t *)0xB8004020;
    g_tx_diag.active_status = *(volatile uint32_t *)0xB8004004;
  } else {
    g_tx_diag.err_count++;
  }
  // 2. Wait until packet finishes transmitting on air (or max 3 ms timeout)
  while ((RAIL_GetRadioState(railHandle) & RAIL_RF_STATE_TX) &&
         ((uint32_t)(RAIL_GetTime() - t0) < 3000UL)) {
  }
  uint32_t dur_us = RAIL_GetTime() - t0;
  if (len == 6) {
    g_tx_diag.beacon_dur_us = dur_us;
  } else if (len > 6) {
    g_tx_diag.cmd_dur_us = dur_us;
  }
}

static void tx_beacon_tick(RAIL_Handle_t railHandle, uint16_t channel, int cnt, uint32_t t_target_us)
{
  uint8_t pkt6[6];
  pkt6[0] = 0x05;
  pkt6[1] = 0x00;
  pkt6[2] = (uint8_t)((cnt >> 8) & 0xFF);
  pkt6[3] = (uint8_t)(cnt & 0xFF);
  if (cnt == 0) {
    pkt6[4] = 0x00;
    pkt6[5] = 0x00;
  } else {
    uint16_t crc = kermit_crc16(&pkt6[1], 3);
    pkt6[4] = (uint8_t)(crc & 0xFF);
    pkt6[5] = (uint8_t)((crc >> 8) & 0xFF);
  }
  tx_at_grid(railHandle, channel, pkt6, 6, t_target_us);
}

// -----------------------------------------------------------------------------
// Legacy 333 us Rapid 3-Channel Slot Transmitter (Trigger 5)
// -----------------------------------------------------------------------------
static void tx_channel_slot(RAIL_Handle_t railHandle, uint16_t channel, const uint8_t *pkt, uint16_t len, uint32_t slot_us)
{
  uint32_t t0 = RAIL_GetTime();
  *(volatile uint32_t *)0xB8014078 = 0x56;
  *(volatile uint32_t *)0xB8004108 = 0x000040FF;
  *(volatile uint32_t *)0xB800410C = 0x000040FF;
  *(volatile uint32_t *)0xB8004110 = 0x000040FF;
  *(volatile uint32_t *)0xB8004114 = 0x000040FF;
  RAIL_SetFixedLength(railHandle, len);
  *(volatile uint32_t *)0xB8014078 = 0x56;
  *(volatile uint32_t *)0xB8004108 = 0x000040FF;
  *(volatile uint32_t *)0xB800410C = 0x000040FF;
  *(volatile uint32_t *)0xB8004110 = 0x000040FF;
  *(volatile uint32_t *)0xB8004114 = 0x000040FF;
  RAIL_ResetFifo(railHandle, true, false);
  RAIL_WriteTxFifo(railHandle, pkt, len, true);
  *(volatile uint32_t *)0xB8014078 = 0x56;
  *(volatile uint32_t *)0xB8004108 = 0x000040FF;
  *(volatile uint32_t *)0xB800410C = 0x000040FF;
  *(volatile uint32_t *)0xB8004110 = 0x000040FF;
  *(volatile uint32_t *)0xB8004114 = 0x000040FF;
  RAIL_StartTx(railHandle, channel, RAIL_TX_OPTION_REMOVE_CRC, NULL);
  while (!(RAIL_GetRadioState(railHandle) & RAIL_RF_STATE_TX) &&
         ((uint32_t)(RAIL_GetTime() - t0) < 500UL)) {}
  if (RAIL_GetRadioState(railHandle) & RAIL_RF_STATE_TX) {
    g_tx_diag.ok_count++;
    g_tx_diag.active_sync0 = *(volatile uint32_t *)0xB8014078;
    g_tx_diag.active_fcd0 = *(volatile uint32_t *)0xB8004108;
    g_tx_diag.active_fcd1 = *(volatile uint32_t *)0xB800410C;
    g_tx_diag.active_wcntcmp0 = *(volatile uint32_t *)0xB8004020;
    g_tx_diag.active_status = *(volatile uint32_t *)0xB8004004;
  } else {
    g_tx_diag.err_count++;
  }
  while ((RAIL_GetRadioState(railHandle) & RAIL_RF_STATE_TX) &&
         ((uint32_t)(RAIL_GetTime() - t0) < 3000UL)) {}
  uint32_t dur_us = RAIL_GetTime() - t0;
  if (len == 6) {
    g_tx_diag.beacon_dur_us = dur_us;
  } else if (len > 6) {
    g_tx_diag.cmd_dur_us = dur_us;
  }
  while ((uint32_t)(RAIL_GetTime() - t0) < slot_us) {}
}

static void tx_packet_on_channels(RAIL_Handle_t railHandle, const uint8_t *pkt, uint16_t len, uint32_t slot_us)
{
  const uint16_t channels[3] = {10, 0, 5};
  RAIL_SetFixedLength(railHandle, len);
  for (int c = 0; c < 3; c++) {
    tx_channel_slot(railHandle, channels[c], pkt, len, slot_us);
  }
}

static void tx_wakeup_until(RAIL_Handle_t railHandle, uint32_t t_target_us, int max_cnt)
{
  const uint16_t channels[3] = {10, 0, 5};
  uint8_t pkt6[6];
  pkt6[0] = 0x05;
  pkt6[1] = 0x00;

  RAIL_SetFixedLength(railHandle, 6);
  int ch_idx = 0;
  while (1) {
    int32_t rem_us = (int32_t)(t_target_us - RAIL_GetTime());
    if (rem_us <= 1500) {
      break;
    }
    int cnt = (int)(rem_us / 1000);
    if (cnt > max_cnt) cnt = max_cnt;
    if (cnt < 1) cnt = 1;
    pkt6[2] = (uint8_t)((cnt >> 8) & 0xFF);
    pkt6[3] = (uint8_t)(cnt & 0xFF);
    uint16_t crc = kermit_crc16(&pkt6[1], 3);
    pkt6[4] = (uint8_t)(crc & 0xFF);
    pkt6[5] = (uint8_t)((crc >> 8) & 0xFF);
    tx_channel_slot(railHandle, channels[ch_idx], pkt6, 6, 333);
    ch_idx = (ch_idx + 1) % 3;
  }

  pkt6[2] = 0x00;
  pkt6[3] = 0x00;
  pkt6[4] = 0x00;
  pkt6[5] = 0x00;
  for (int c = 0; c < 3; c++) {
    tx_channel_slot(railHandle, channels[c], pkt6, 6, 333);
  }
}

static void transmit_single_subburst(RAIL_Handle_t railHandle, uint16_t ch, const uint8_t *w1_pkt, const uint8_t *w2_pkt, uint16_t pkt_len, bool lpm_wakeup)
{
  uint32_t t_origin = RAIL_GetTime() + 500UL;

  if (lpm_wakeup) {
    // Step A: Main Wake-Up Countdown (tick = 0..106, cnt = 106 down to 0, 1000.0 us per tick)
    for (int tick = 0; tick <= 106; tick++) {
      int cnt = 106 - tick;
      tx_beacon_tick(railHandle, ch, cnt, t_origin + (uint32_t)tick * 1000UL);
    }

    // Step B: 4 Command Windows (w = 0..3 at w_tick = 107 + w * 13, i.e., tick = 107, 120, 133, 146)
    for (int w = 0; w < 4; w++) {
      uint32_t w_tick = 107UL + (uint32_t)w * 13UL;
      const uint8_t *pkt = (w == 0) ? w1_pkt : w2_pkt;
      tx_at_grid(railHandle, ch, pkt, pkt_len, t_origin + w_tick * 1000UL);

      // If w < 3, transmit Short Countdown cnt = 7 down to 0 at w_tick + 5 .. w_tick + 12
      if (w < 3) {
        for (int s = 0; s <= 7; s++) {
          int cnt = 7 - s;
          uint32_t s_tick = w_tick + 5UL + (uint32_t)s;
          tx_beacon_tick(railHandle, ch, cnt, t_origin + s_tick * 1000UL);
        }
      }
    }
  } else {
    // Step A: Initial Short Countdown cnt = 7 down to 0 (tick = 0..7)
    for (int tick = 0; tick <= 7; tick++) {
      int cnt = 7 - tick;
      tx_beacon_tick(railHandle, ch, cnt, t_origin + (uint32_t)tick * 1000UL);
    }

    // Step B: 4 Command Windows (w = 0..3 at w_tick = 8 + w * 13, i.e., tick = 8, 21, 34, 47)
    for (int w = 0; w < 4; w++) {
      uint32_t w_tick = 8UL + (uint32_t)w * 13UL;
      tx_at_grid(railHandle, ch, w2_pkt, pkt_len, t_origin + w_tick * 1000UL);

      if (w < 3) {
        for (int s = 0; s <= 7; s++) {
          int cnt = 7 - s;
          uint32_t s_tick = w_tick + 5UL + (uint32_t)s;
          tx_beacon_tick(railHandle, ch, cnt, t_origin + s_tick * 1000UL);
        }
      }
    }
  }
}

static void transmit_hopping_subburst(RAIL_Handle_t railHandle, uint32_t b, const uint8_t *w1_pkt, const uint8_t *w2_pkt, uint16_t pkt_len)
{
  const uint16_t channels_fwd[3] = {0, 5, 10};
  uint32_t t_origin = RAIL_GetTime() + 500UL;

  for (int tick = 0; tick <= 106; tick++) {
    int cnt = 106 - tick;
    uint16_t ch = channels_fwd[(tick + b) % 3];
    tx_beacon_tick(railHandle, ch, cnt, t_origin + (uint32_t)tick * 1000UL);
  }

  for (int w = 0; w < 4; w++) {
    uint32_t w_tick = 107UL + (uint32_t)w * 13UL;
    uint16_t w_ch = channels_fwd[(w_tick + b) % 3];
    const uint8_t *pkt = (w == 0) ? w1_pkt : w2_pkt;
    tx_at_grid(railHandle, w_ch, pkt, pkt_len, t_origin + w_tick * 1000UL);

    if (w < 3) {
      for (int s = 0; s <= 7; s++) {
        int cnt = 7 - s;
        uint32_t s_tick = w_tick + 5UL + (uint32_t)s;
        uint16_t s_ch = channels_fwd[(s_tick + b) % 3];
        tx_beacon_tick(railHandle, s_ch, cnt, t_origin + s_tick * 1000UL);
      }
    }
  }
}

static void transmit_velux_burst(RAIL_Handle_t railHandle)
{
  uint8_t base_psdu[36];
  uint16_t psdu_len = 28;
  uint32_t trig = g_velux_tx.trigger;

  if (trig == 2) {
    if (!g_velux_tx.last_rx_valid) {
      printf("[VELUX-TX] Error: No valid last_rx_psdu to replay\n");
      g_velux_tx.trigger = 0;
      return;
    }
    psdu_len = (g_velux_tx.last_rx_len >= 10 && g_velux_tx.last_rx_len <= 36)
               ? (uint16_t)g_velux_tx.last_rx_len : 28;
    uint16_t c0 = (psdu_len <= 28) ? psdu_len : 28;
    memcpy(base_psdu, (const void *)g_velux_tx.last_rx_psdu, c0);
    if (psdu_len > 28) {
      memcpy(&base_psdu[28], (const void *)g_velux_tx.last_rx_ext, psdu_len - 28);
    }
  } else {
    psdu_len = (g_velux_tx.psdu_len >= 10 && g_velux_tx.psdu_len <= 36)
               ? (uint16_t)g_velux_tx.psdu_len : 28;
    uint16_t c0 = (psdu_len <= 28) ? psdu_len : 28;
    memcpy(base_psdu, (const void *)g_velux_tx.psdu, c0);
    if (psdu_len > 28) {
      memcpy(&base_psdu[28], (const void *)g_velux_tx.psdu_ext, psdu_len - 28);
    }
  }

  uint32_t bursts = (g_velux_tx.num_bursts > 0) ? g_velux_tx.num_bursts : 1;

  printf("\n[VELUX-TX] Starting %lu burst(s) (trigger=%lu, psdu_len=%u)...\n",
         (unsigned long)bursts, (unsigned long)trig, (unsigned int)psdu_len);

  // 1. Ensure RF Switch: PB05=1, PB04=0
  sl_gpio_t rf_sw_pw = { .port = SL_GPIO_PORT_B, .pin = 5 };
  sl_gpio_set_pin_mode(&rf_sw_pw, SL_GPIO_MODE_PUSH_PULL, 1);
  sl_gpio_t rf_sw_sel = { .port = SL_GPIO_PORT_B, .pin = 4 };
  sl_gpio_set_pin_mode(&rf_sw_sel, SL_GPIO_MODE_PUSH_PULL, 0);

  // 2. Set MODEM_NS.SYNC0 to 0x56 (direct on-air SFD=0x56), disable DUALSYNC,
  //    and clear INCLUDECRC/CALCCRC in FRC_NS.FCD0..FCD3 so hardware
  //    does NOT overwrite our software Kermit CRC-16 bytes at the end of the TX FIFO!
  RAIL_Idle(railHandle, RAIL_IDLE_ABORT, true);

  extern RAIL_DataConfig_t railDataConfig;
  extern bool rxFifoManual;
  railDataConfig.txMethod = PACKET_MODE;
  railDataConfig.rxMethod = PACKET_MODE;
  RAIL_ConfigData(railHandle, &railDataConfig);
  rxFifoManual = false;
  RAIL_ResetFifo(railHandle, true, true);

  RAIL_SyncWordConfig_t txSyncConfig = { .syncWordBits = 8U, .syncWord1 = 0x56UL, .syncWord2 = 0x6AUL };
  RAIL_ConfigSyncWords(railHandle, &txSyncConfig);

  memset(&g_tx_diag, 0, sizeof(g_tx_diag));

  *(volatile uint32_t *)0xB8014050 = 0x0052C007;
  *(volatile uint32_t *)0xB8014078 = 0x56;
  *(volatile uint32_t *)0xB8004108 = 0x000040FF;
  *(volatile uint32_t *)0xB800410C = 0x000040FF;
  *(volatile uint32_t *)0xB8004110 = 0x000040FF;
  *(volatile uint32_t *)0xB8004114 = 0x000040FF;

  // 3. Set TX Power to +10 dBm (100 deci-dBm, official Seeed XIAO MG24 PA config)
  RAIL_SetTxPowerDbm(railHandle, 100);

  // 4. Configure TX transitions to return to IDLE after transmit
  RAIL_StateTransitions_t tx_trans = {
    .success = RAIL_RF_STATE_IDLE,
    .error   = RAIL_RF_STATE_IDLE
  };
  RAIL_SetTxTransitions(railHandle, &tx_trans);

  // 5. Build Window 1 (CB2=0x20) and Window 2..4 (CB2=0x00) packets (1 + psdu_len bytes each)
  uint16_t pkt_len = 1 + psdu_len;
  uint8_t w1_pkt[37];
  w1_pkt[0] = (uint8_t)psdu_len;
  memcpy(&w1_pkt[1], base_psdu, psdu_len);
  w1_pkt[1 + 4] = 0x20; // CB2 = 0x20 (LPM active)
  uint16_t c1 = kermit_crc16(&w1_pkt[1], psdu_len - 2);
  w1_pkt[1 + psdu_len - 2] = (uint8_t)(c1 & 0xFF);
  w1_pkt[1 + psdu_len - 1] = (uint8_t)((c1 >> 8) & 0xFF);

  uint8_t w2_pkt[37];
  w2_pkt[0] = (uint8_t)psdu_len;
  memcpy(&w2_pkt[1], base_psdu, psdu_len);
  w2_pkt[1 + 4] = 0x00; // CB2 = 0x00
  uint16_t c2 = kermit_crc16(&w2_pkt[1], psdu_len - 2);
  w2_pkt[1 + psdu_len - 2] = (uint8_t)(c2 & 0xFF);
  w2_pkt[1 + psdu_len - 1] = (uint8_t)((c2 >> 8) & 0xFF);

  uint16_t s1_psdu_len = (g_velux_tx.last_rx_len >= 10 && g_velux_tx.last_rx_len <= 36)
                         ? (uint16_t)g_velux_tx.last_rx_len : 28;
  uint16_t s1_pkt_len = 1 + s1_psdu_len;
  uint8_t s1_w1_pkt[37];
  uint8_t s1_w2_pkt[37];
  {
    uint8_t s1_base[36];
    uint16_t c0 = (s1_psdu_len <= 28) ? s1_psdu_len : 28;
    memcpy(s1_base, (const void *)g_velux_tx.last_rx_psdu, c0);
    if (s1_psdu_len > 28) {
      memcpy(&s1_base[28], (const void *)g_velux_tx.last_rx_ext, s1_psdu_len - 28);
    }
    s1_w1_pkt[0] = (uint8_t)s1_psdu_len;
    memcpy(&s1_w1_pkt[1], s1_base, s1_psdu_len);
    s1_w1_pkt[1 + 4] = 0x20;
    uint16_t sc1 = kermit_crc16(&s1_w1_pkt[1], s1_psdu_len - 2);
    s1_w1_pkt[1 + s1_psdu_len - 2] = (uint8_t)(sc1 & 0xFF);
    s1_w1_pkt[1 + s1_psdu_len - 1] = (uint8_t)((sc1 >> 8) & 0xFF);

    s1_w2_pkt[0] = (uint8_t)s1_psdu_len;
    memcpy(&s1_w2_pkt[1], s1_base, s1_psdu_len);
    s1_w2_pkt[1 + 4] = 0x00;
    uint16_t sc2 = kermit_crc16(&s1_w2_pkt[1], s1_psdu_len - 2);
    s1_w2_pkt[1 + s1_psdu_len - 2] = (uint8_t)(sc2 & 0xFF);
    s1_w2_pkt[1 + s1_psdu_len - 1] = (uint8_t)((sc2 >> 8) & 0xFF);
  }

  printf("[VELUX-TX] W1 PSDU (%uB): ", (unsigned int)psdu_len);
  for (uint16_t b = 0; b < psdu_len; b++) printf("%02x ", w1_pkt[1 + b]);
  printf("\n[VELUX-TX] W2 PSDU (%uB): ", (unsigned int)psdu_len);
  for (uint16_t b = 0; b < psdu_len; b++) printf("%02x ", w2_pkt[1 + b]);
  printf("\n");

  const uint16_t channels_fwd[3] = {0, 5, 10};
  uint32_t t_all_start = RAIL_GetTime();

  if (trig == 9) {
    for (uint32_t b = 0; b < bursts; b++) {
      uint16_t ch = channels_fwd[b % 3];
      transmit_single_subburst(railHandle, ch, s1_w1_pkt, s1_w2_pkt, s1_pkt_len, true);
      uint32_t pause_t = RAIL_GetTime();
      while ((uint32_t)(RAIL_GetTime() - pause_t) < 5000UL) {}
    }
    for (uint32_t b = 0; b < bursts; b++) {
      uint16_t ch = channels_fwd[b % 3];
      transmit_single_subburst(railHandle, ch, w1_pkt, w2_pkt, pkt_len, false);
      if (b + 1 < bursts) {
        uint32_t pause_t = RAIL_GetTime();
        while ((uint32_t)(RAIL_GetTime() - pause_t) < 5000UL) {}
      }
    }
  } else {
    for (uint32_t b = 0; b < bursts; b++) {
      uint16_t ch = channels_fwd[b % 3];

      if (trig == 1 || trig == 2) {
        transmit_single_subburst(railHandle, ch, w1_pkt, w2_pkt, pkt_len, true);
        uint32_t pause_t = RAIL_GetTime();
        while ((uint32_t)(RAIL_GetTime() - pause_t) < 20000UL) {}
        transmit_single_subburst(railHandle, ch, w1_pkt, w2_pkt, pkt_len, false);
      } else if (trig == 3) {
        transmit_single_subburst(railHandle, ch, w1_pkt, w2_pkt, pkt_len, true);
      } else if (trig == 4) {
        transmit_single_subburst(railHandle, ch, w1_pkt, w2_pkt, pkt_len, false);
      } else if (trig == 5) {
        uint32_t t_burst_start = RAIL_GetTime();
        tx_wakeup_until(railHandle, t_burst_start + 106000UL, 0x006A);

        uint32_t t_w1 = RAIL_GetTime();
        tx_packet_on_channels(railHandle, w1_pkt, pkt_len, 1100);

        tx_wakeup_until(railHandle, t_w1 + 13000UL, 7);
        tx_packet_on_channels(railHandle, w2_pkt, pkt_len, 1100);

        tx_wakeup_until(railHandle, t_w1 + 26000UL, 7);
        tx_packet_on_channels(railHandle, w2_pkt, pkt_len, 1100);

        tx_wakeup_until(railHandle, t_w1 + 39000UL, 7);
        tx_packet_on_channels(railHandle, w2_pkt, pkt_len, 1100);
      } else if (trig == 6) {
        transmit_hopping_subburst(railHandle, b, w1_pkt, w2_pkt, pkt_len);
      } else if (trig == 7) {
        uint32_t t_test = RAIL_GetTime() + 1000UL;
        tx_beacon_tick(railHandle, 0, 0x006A, t_test);
        tx_at_grid(railHandle, 0, w1_pkt, pkt_len, t_test + 10000UL);
      } else if (trig == 8) {
        transmit_single_subburst(railHandle, ch, s1_w1_pkt, s1_w2_pkt, s1_pkt_len, true);
        uint32_t pause_t = RAIL_GetTime();
        while ((uint32_t)(RAIL_GetTime() - pause_t) < 15000UL) {}
        transmit_single_subburst(railHandle, ch, w1_pkt, w2_pkt, pkt_len, false);
      }

      if (b + 1 < bursts) {
        uint32_t pause_t = RAIL_GetTime();
        while ((uint32_t)(RAIL_GetTime() - pause_t) < 15000UL) {}
      }
    }
  }

  uint32_t total_ms = (RAIL_GetTime() - t_all_start) / 1000UL;

  // Restore the RX FCD registers and SYNC0/SYNC1
  RAIL_Idle(railHandle, RAIL_IDLE_ABORT, true);
  *(volatile uint32_t *)0xB8004108 = 0x00004000;
  *(volatile uint32_t *)0xB800410C = 0x00004CFF;
  *(volatile uint32_t *)0xB8004110 = 0x00004100;
  *(volatile uint32_t *)0xB8004114 = 0x00004DFF;
  *(volatile uint32_t *)0xB8014078 = 0x6a;
  *(volatile uint32_t *)0xB801407C = 0x56;
  RAIL_SetFixedLength(railHandle, 0);

  RAIL_SyncWordConfig_t rxSyncConfig = { .syncWordBits = 8U, .syncWord1 = 0x6AUL, .syncWord2 = 0x56UL };
  RAIL_ConfigSyncWords(railHandle, &rxSyncConfig);

  g_velux_tx.tx_done_count++;
  g_velux_tx.trigger = 0;
  printf("[VELUX-TX] Burst complete (count=%lu, total_ms=%lu)\n",
         (unsigned long)g_velux_tx.tx_done_count, (unsigned long)total_ms);
  printf("[TX_DIAG] trig=%lu ok=%lu err=%lu bcn_us=%lu cmd_us=%lu SYNC0=0x%08lX FCD0=0x%08lX FCD1=0x%08lX WCNT0=0x%08lX\n",
         (unsigned long)trig,
         (unsigned long)g_tx_diag.ok_count,
         (unsigned long)g_tx_diag.err_count,
         (unsigned long)g_tx_diag.beacon_dur_us,
         (unsigned long)g_tx_diag.cmd_dur_us,
         (unsigned long)g_tx_diag.active_sync0,
         (unsigned long)g_tx_diag.active_fcd0,
         (unsigned long)g_tx_diag.active_fcd1,
         (unsigned long)g_tx_diag.active_wcntcmp0);
}

// =============================================================================
// Self-Contained Dual-Device (Blind + Skylight) Crypto, Flash NVM & CLI Module
// =============================================================================

#if defined(VELUX_HAS_COMPILE_TIME_TRANSFER_KEY) && (VELUX_HAS_COMPILE_TIME_TRANSFER_KEY == 1)
static uint8_t g_velux_transfer_key[16] = VELUX_COMPILE_TIME_TRANSFER_KEY;
static bool    g_velux_transfer_key_valid = true;
#else
static uint8_t g_velux_transfer_key[16] = {0};
static bool    g_velux_transfer_key_valid = false;
#endif

static const uint8_t aes_sbox[256] = {
  0x63,0x7c,0x77,0x7b,0xf2,0x6b,0x6f,0xc5,0x30,0x01,0x67,0x2b,0xfe,0xd7,0xab,0x76,
  0xca,0x82,0xc9,0x7d,0xfa,0x59,0x47,0xf0,0xad,0xd4,0xa2,0xaf,0x9c,0xa4,0x72,0xc0,
  0xb7,0xfd,0x93,0x26,0x36,0x3f,0xf7,0xcc,0x34,0xa5,0xe5,0xf1,0x71,0xd8,0x31,0x15,
  0x04,0xc7,0x23,0xc3,0x18,0x96,0x05,0x9a,0x07,0x12,0x80,0xe2,0xeb,0x27,0xb2,0x75,
  0x09,0x83,0x2c,0x1a,0x1b,0x6e,0x5a,0xa0,0x52,0x3b,0xd6,0xb3,0x29,0xe3,0x2f,0x84,
  0x53,0xd1,0x00,0xed,0x20,0xfc,0xb1,0x5b,0x6a,0xcb,0xbe,0x39,0x4a,0x4c,0x58,0xcf,
  0xd0,0xef,0xaa,0xfb,0x43,0x4d,0x33,0x85,0x45,0xf9,0x02,0x7f,0x50,0x3c,0x9f,0xa8,
  0x51,0xa3,0x40,0x8f,0x92,0x9d,0x38,0xf5,0xbc,0xb6,0xda,0x21,0x10,0xff,0xf3,0xd2,
  0xcd,0x0c,0x13,0xec,0x5f,0x97,0x44,0x17,0xc4,0xa7,0x7e,0x3d,0x64,0x5d,0x19,0x73,
  0x60,0x81,0x4f,0xdc,0x22,0x2a,0x90,0x88,0x46,0xee,0xb8,0x14,0xde,0x5e,0x0b,0xdb,
  0xe0,0x32,0x3a,0x0a,0x49,0x06,0x24,0x5c,0xc2,0xd3,0xac,0x62,0x91,0x95,0xe4,0x79,
  0xe7,0xc8,0x37,0x6d,0x8d,0xd5,0x4e,0xa9,0x6c,0x56,0xf4,0xea,0x65,0x7a,0xae,0x08,
  0xba,0x78,0x25,0x2e,0x1c,0xa6,0xb4,0xc6,0xe8,0xdd,0x74,0x1f,0x4b,0xbd,0x8b,0x8a,
  0x70,0x3e,0xb5,0x66,0x48,0x03,0xf6,0x0e,0x61,0x35,0x57,0xb9,0x86,0xc1,0x1d,0x9e,
  0xe1,0xf8,0x98,0x11,0x69,0xd9,0x8e,0x94,0x9b,0x1e,0x87,0xe9,0xce,0x55,0x28,0xdf,
  0x8c,0xa1,0x89,0x0d,0xbf,0xe6,0x42,0x68,0x41,0x99,0x2d,0x0f,0xb0,0x54,0xbb,0x16
};

static inline uint8_t aes_xtime(uint8_t x)
{
  return (uint8_t)((x << 1) ^ ((x >> 7) * 0x1b));
}

static void aes128_ecb_encrypt(const uint8_t key[16], const uint8_t in[16], uint8_t out[16])
{
  static const uint8_t rcon[11] = {0x00,0x01,0x02,0x04,0x08,0x10,0x20,0x40,0x80,0x1b,0x36};
  uint8_t rk[176];
  memcpy(rk, key, 16);
  for (int i = 4; i < 44; i++) {
    uint8_t t0 = rk[(i - 1) * 4 + 0];
    uint8_t t1 = rk[(i - 1) * 4 + 1];
    uint8_t t2 = rk[(i - 1) * 4 + 2];
    uint8_t t3 = rk[(i - 1) * 4 + 3];
    if ((i & 3) == 0) {
      uint8_t a = aes_sbox[t1], b = aes_sbox[t2], c = aes_sbox[t3], d = aes_sbox[t0];
      t0 = (uint8_t)(a ^ rcon[i >> 2]);
      t1 = b;
      t2 = c;
      t3 = d;
    }
    rk[i * 4 + 0] = rk[(i - 4) * 4 + 0] ^ t0;
    rk[i * 4 + 1] = rk[(i - 4) * 4 + 1] ^ t1;
    rk[i * 4 + 2] = rk[(i - 4) * 4 + 2] ^ t2;
    rk[i * 4 + 3] = rk[(i - 4) * 4 + 3] ^ t3;
  }

  uint8_t s[16];
  memcpy(s, in, 16);
  for (int i = 0; i < 16; i++) s[i] ^= rk[i];
  for (int round = 1; round <= 10; round++) {
    for (int i = 0; i < 16; i++) s[i] = aes_sbox[s[i]];
    uint8_t t[16];
    for (int r = 0; r < 4; r++) {
      for (int c = 0; c < 4; c++) {
        t[r + 4 * c] = s[r + 4 * ((c + r) & 3)];
      }
    }
    memcpy(s, t, 16);
    if (round != 10) {
      for (int c = 0; c < 4; c++) {
        uint8_t *col = &s[4 * c];
        uint8_t a0 = col[0], a1 = col[1], a2 = col[2], a3 = col[3];
        col[0] = (uint8_t)(aes_xtime(a0) ^ (aes_xtime(a1) ^ a1) ^ a2 ^ a3);
        col[1] = (uint8_t)(a0 ^ aes_xtime(a1) ^ (aes_xtime(a2) ^ a2) ^ a3);
        col[2] = (uint8_t)(a0 ^ a1 ^ aes_xtime(a2) ^ (aes_xtime(a3) ^ a3));
        col[3] = (uint8_t)((aes_xtime(a0) ^ a0) ^ a1 ^ a2 ^ aes_xtime(a3));
      }
    }
    for (int i = 0; i < 16; i++) s[i] ^= rk[round * 16 + i];
  }
  memcpy(out, s, 16);
}

static void iohc_checksum_step(uint8_t frame_byte, uint8_t *chk1, uint8_t *chk2)
{
  uint8_t tmp = (uint8_t)(frame_byte ^ *chk2);
  uint8_t loc = (uint8_t)(((*chk1 & 0x7f) << 1) & 0xff);
  if (tmp >= 0x80) loc |= 1;
  if ((*chk1 & 0x80) == 0) {
    *chk1 = loc;
    *chk2 = (uint8_t)((tmp << 1) & 0xff);
  } else {
    *chk1 = (uint8_t)(loc ^ 0x55);
    *chk2 = (uint8_t)(((tmp << 1) ^ 0x5b) & 0xff);
  }
}

static void iohc_create_1w_hmac(uint8_t hmac[6], uint16_t seq_num, const uint8_t key[16],
                                const uint8_t *frame_data, uint8_t frame_len)
{
  uint8_t iv[16], out[16];
  memset(iv, 0, 16);
  uint8_t c1 = 0, c2 = 0;
  for (uint8_t i = 0; i < frame_len; i++) {
    iohc_checksum_step(frame_data[i], &c1, &c2);
    if (i < 8) iv[i] = frame_data[i];
  }
  for (uint8_t j = frame_len; j < 8; j++) {
    iv[j] = 0x55;
  }
  iv[8]  = c1;
  iv[9]  = c2;
  iv[10] = (uint8_t)((seq_num >> 8) & 0xFF);
  iv[11] = (uint8_t)(seq_num & 0xFF);
  iv[12] = iv[13] = iv[14] = iv[15] = 0x55;
  aes128_ecb_encrypt(key, iv, out);
  memcpy(hmac, out, 6);
}

static void iohc_decrypt_1w_key(const uint8_t node_addr[3], const uint8_t enc_key[16], uint8_t out_key[16])
{
  uint8_t iv[16], ks[16];
  for (int i = 0; i < 15; i += 3) {
    iv[i + 0] = node_addr[0];
    iv[i + 1] = node_addr[1];
    iv[i + 2] = node_addr[2];
  }
  iv[15] = node_addr[0];
  aes128_ecb_encrypt(g_velux_transfer_key, iv, ks);
  for (int i = 0; i < 16; i++) {
    out_key[i] = (uint8_t)(enc_key[i] ^ ks[i]);
  }
}

// -----------------------------------------------------------------------------
// Non-Volatile Flash Storage (Page at 0x0817C000: 127 wear-leveled device slots
// + Slot 127 at 0x0817FFC0 reserved for optional runtime 1-Way transfer key)
// -----------------------------------------------------------------------------
#define VELUX_NVM_FLASH_PAGE_ADDR   0x0817C000UL
#define VELUX_NVM_SLOT_COUNT        127U          // Slots 0..126 for wear-leveled state
#define VELUX_NVM_TK_SLOT_ADDR      (0x0817C000UL + 127U * 64U)
#define VELUX_NVM_MAGIC             0x564C5832UL  // "VLX2"
#define VELUX_TK_MAGIC              0x564C584BUL  // "VLXK"

typedef struct __attribute__((aligned(4))) {
  uint32_t magic;            // +0x00: 0x564C5832 ("VLX2")
  uint32_t generation;       // +0x04: 1, 2, 3...
  uint8_t  blind_addr[3];    // +0x08: 3-byte blind actuator/remote address
  uint8_t  blind_pos;        // +0x0B: 0..100 (HA %: 0=closed, 100=open)
  uint8_t  blind_key[16];    // +0x0C: 16-byte AES-128 key
  uint16_t blind_seq;        // +0x1C: rolling counter
  uint16_t blind_valid;      // +0x1E: 1 = paired/valid
  uint8_t  sky_addr[3];      // +0x20: 3-byte skylight actuator/remote address
  uint8_t  sky_pos;          // +0x23: 0..100
  uint8_t  sky_key[16];      // +0x24: 16-byte AES-128 key
  uint16_t sky_seq;          // +0x34: rolling counter
  uint16_t sky_valid;        // +0x36: 1 = paired/valid
  uint32_t reserved;         // +0x38
  uint32_t crc32;            // +0x3C: kermit_crc16 of bytes 0..59
} VeluxNvmSlot_t;            // 64 bytes total

typedef struct __attribute__((aligned(4))) {
  uint32_t magic;            // +0x00: 0x564C584B ("VLXK")
  uint8_t  transfer_key[16]; // +0x04: 16-byte 1-Way transfer key
  uint8_t  pad[40];          // +0x14..+0x3B
  uint32_t crc32;            // +0x3C: kermit_crc16 of bytes 0..59
} VeluxTkSlot_t;             // 64 bytes total

static VeluxNvmSlot_t g_velux_nvm;
static uint32_t       g_velux_nvm_next_slot = 0;
static bool           g_velux_pair_mode = false;
static int            g_velux_pair_target = 1; // 0=blind, 1=skylight

static void velux_flash_wait_ready(void)
{
  uint32_t timeout = 2000000UL;
  while ((MSC->STATUS & (MSC_STATUS_BUSY | MSC_STATUS_PENDING)) && timeout > 0) {
    timeout--;
  }
}

static void velux_flash_erase_page(uint32_t page_addr)
{
  CORE_DECLARE_IRQ_STATE;
  CORE_ENTER_CRITICAL();
  CMU->CLKEN1_SET = CMU_CLKEN1_MSC;
  MSC->LOCK = MSC_LOCK_LOCKKEY_UNLOCK;
  MSC->WRITECTRL_SET = MSC_WRITECTRL_WREN;
  MSC->ADDRB = page_addr;
  MSC->WRITECMD = MSC_WRITECMD_ERASEPAGE;
  velux_flash_wait_ready();
  velux_flash_wait_ready();
  MSC->WRITECTRL_CLR = MSC_WRITECTRL_WREN;
  MSC->LOCK = MSC_LOCK_LOCKKEY_LOCK;
  CORE_EXIT_CRITICAL();
}

static void velux_flash_write_64b(uint32_t slot_addr, const void *slot64)
{
  const uint32_t *words = (const uint32_t *)slot64;
  uint32_t num_words = 64U / 4U;

  CORE_DECLARE_IRQ_STATE;
  CORE_ENTER_CRITICAL();
  CMU->CLKEN1_SET = CMU_CLKEN1_MSC;
  MSC->LOCK = MSC_LOCK_LOCKKEY_UNLOCK;
  MSC->WRITECTRL_SET = MSC_WRITECTRL_WREN;

  for (uint32_t i = 0; i < num_words; i++) {
    velux_flash_wait_ready();
    MSC->ADDRB = slot_addr + (i * 4U);
    MSC->WDATA = words[i];
    velux_flash_wait_ready();
  }

  velux_flash_wait_ready();
  MSC->WRITECTRL_CLR = MSC_WRITECTRL_WREN;
  MSC->LOCK = MSC_LOCK_LOCKKEY_LOCK;
  CORE_EXIT_CRITICAL();
}

static void velux_tk_write_if_needed(void)
{
  if (!g_velux_transfer_key_valid) {
    return;
  }
  const VeluxTkSlot_t *tk_flash = (const VeluxTkSlot_t *)VELUX_NVM_TK_SLOT_ADDR;
  if (tk_flash->magic == 0xFFFFFFFFUL) {
    VeluxTkSlot_t tk;
    memset(&tk, 0, sizeof(tk));
    tk.magic = VELUX_TK_MAGIC;
    memcpy(tk.transfer_key, g_velux_transfer_key, 16);
    tk.crc32 = (uint32_t)kermit_crc16((const uint8_t *)&tk, 60);
    velux_flash_write_64b(VELUX_NVM_TK_SLOT_ADDR, &tk);
  }
}

static void velux_nvm_save(void)
{
  g_velux_nvm.magic = VELUX_NVM_MAGIC;
  g_velux_nvm.generation++;
  g_velux_nvm.crc32 = (uint32_t)kermit_crc16((const uint8_t *)&g_velux_nvm, 60);

  if (g_velux_nvm_next_slot >= VELUX_NVM_SLOT_COUNT) {
    velux_flash_erase_page(VELUX_NVM_FLASH_PAGE_ADDR);
    g_velux_nvm_next_slot = 0;
    velux_tk_write_if_needed();
  }

  uint32_t slot_addr = VELUX_NVM_FLASH_PAGE_ADDR + g_velux_nvm_next_slot * sizeof(VeluxNvmSlot_t);
  velux_flash_write_64b(slot_addr, &g_velux_nvm);
  g_velux_nvm_next_slot++;
}

static void velux_nvm_init(void)
{
  // 1. Check if a runtime 1-Way transfer key is saved in Slot 127
  const VeluxTkSlot_t *tk_flash = (const VeluxTkSlot_t *)VELUX_NVM_TK_SLOT_ADDR;
  if (tk_flash->magic == VELUX_TK_MAGIC) {
    uint32_t tk_crc = (uint32_t)kermit_crc16((const uint8_t *)tk_flash, 60);
    if (tk_crc == tk_flash->crc32) {
      memcpy(g_velux_transfer_key, tk_flash->transfer_key, 16);
      g_velux_transfer_key_valid = true;
    }
  }

  // 2. Scan Slots 0..126 for the most recent valid device state slot
  const VeluxNvmSlot_t *flash_slots = (const VeluxNvmSlot_t *)VELUX_NVM_FLASH_PAGE_ADDR;
  int last_valid_idx = -1;

  for (uint32_t i = 0; i < VELUX_NVM_SLOT_COUNT; i++) {
    if (flash_slots[i].magic == 0xFFFFFFFFUL) {
      g_velux_nvm_next_slot = i;
      break;
    }
    if (flash_slots[i].magic == VELUX_NVM_MAGIC) {
      uint32_t crc = (uint32_t)kermit_crc16((const uint8_t *)&flash_slots[i], 60);
      if (crc == flash_slots[i].crc32) {
        last_valid_idx = (int)i;
        g_velux_nvm_next_slot = i + 1;
      }
    }
  }

  if (last_valid_idx >= 0) {
    memcpy(&g_velux_nvm, &flash_slots[last_valid_idx], sizeof(VeluxNvmSlot_t));
  } else {
    // Clean factory-blank state (zero personal keys or addresses)
    memset(&g_velux_nvm, 0, sizeof(VeluxNvmSlot_t));
    g_velux_nvm.magic = VELUX_NVM_MAGIC;
    g_velux_nvm.generation = 0;
    g_velux_nvm.blind_seq = 1;
    g_velux_nvm.blind_valid = 0;
    g_velux_nvm.sky_seq = 1;
    g_velux_nvm.sky_valid = 0;
    g_velux_nvm_next_slot = VELUX_NVM_SLOT_COUNT; // Force erase on first save
    velux_nvm_save();
  }
}

static void velux_print_device_json(const char *event, int dev_idx, const char *action)
{
  const char *name = (dev_idx == 0) ? "blind" : "skylight";
  const uint8_t *addr = (dev_idx == 0) ? g_velux_nvm.blind_addr : g_velux_nvm.sky_addr;
  const uint8_t *key  = (dev_idx == 0) ? g_velux_nvm.blind_key  : g_velux_nvm.sky_key;
  uint16_t seq        = (dev_idx == 0) ? g_velux_nvm.blind_seq  : g_velux_nvm.sky_seq;
  uint8_t  pos        = (dev_idx == 0) ? g_velux_nvm.blind_pos  : g_velux_nvm.sky_pos;
  bool     paired     = (dev_idx == 0) ? (g_velux_nvm.blind_valid != 0) : (g_velux_nvm.sky_valid != 0);

  printf("{\"event\":\"%s\",\"device\":\"%s\",\"addr\":\"%02x%02x%02x\",\"action\":\"%s\","
         "\"position\":%u,\"seq\":%u,\"seq_hex\":\"0x%04X\",\"paired\":%s,"
         "\"transfer_key_set\":%s,\"key\":\"",
         event, name, addr[0], addr[1], addr[2], action,
         (unsigned int)pos, (unsigned int)seq, (unsigned int)seq,
         paired ? "true" : "false",
         g_velux_transfer_key_valid ? "true" : "false");
  for (int i = 0; i < 16; i++) {
    printf("%02x", key[i]);
  }
  printf("\"}\n");
}

static void velux_build_30b_frame(int dev_idx, const uint8_t addr[3], const uint8_t key[16],
                                  uint16_t seq_num, uint8_t fp_val, uint8_t bcast_hi, uint8_t bcast_lo,
                                  uint8_t out_psdu[30])
{
  uint8_t appli[9] = { 0x00, 0x01, 0x61, 0xd4, 0x00, 0x80, fp_val, 0x00, 0x00 };
  (void)dev_idx;
  out_psdu[0]  = 0x01;
  out_psdu[1]  = 0x69;
  out_psdu[2]  = 0x6f;
  out_psdu[3]  = 0xf8;
  out_psdu[4]  = 0x20;
  out_psdu[5]  = 0x00;
  out_psdu[6]  = bcast_hi;
  out_psdu[7]  = bcast_lo;
  out_psdu[8]  = addr[0];
  out_psdu[9]  = addr[1];
  out_psdu[10] = addr[2];
  memcpy(&out_psdu[11], appli, 9);
  out_psdu[20] = (uint8_t)((seq_num >> 8) & 0xFF);
  out_psdu[21] = (uint8_t)(seq_num & 0xFF);
  iohc_create_1w_hmac(&out_psdu[22], seq_num, key, appli, 9);
  uint16_t crc = kermit_crc16(out_psdu, 28);
  out_psdu[28] = (uint8_t)(crc & 0xFF);
  out_psdu[29] = (uint8_t)((crc >> 8) & 0xFF);
}

static void velux_build_28b_frame(const uint8_t addr[3], const uint8_t key[16],
                                  uint16_t seq_num, uint8_t main0, uint8_t out_psdu[28])
{
  uint8_t appli[7] = { 0x00, 0x01, 0x61, main0, 0x00, 0x00, 0x00 };
  out_psdu[0]  = 0x01;
  out_psdu[1]  = 0x69;
  out_psdu[2]  = 0x6f;
  out_psdu[3]  = 0xf6;
  out_psdu[4]  = 0x20;
  out_psdu[5]  = 0x00;
  out_psdu[6]  = 0x00;
  out_psdu[7]  = 0x3f;
  out_psdu[8]  = addr[0];
  out_psdu[9]  = addr[1];
  out_psdu[10] = addr[2];
  memcpy(&out_psdu[11], appli, 7);
  out_psdu[18] = (uint8_t)((seq_num >> 8) & 0xFF);
  out_psdu[19] = (uint8_t)(seq_num & 0xFF);
  iohc_create_1w_hmac(&out_psdu[20], seq_num, key, appli, 7);
  uint16_t crc = kermit_crc16(out_psdu, 26);
  out_psdu[26] = (uint8_t)(crc & 0xFF);
  out_psdu[27] = (uint8_t)((crc >> 8) & 0xFF);
}

static void velux_exec_command(int dev_idx, uint8_t main0, const char *action_name, uint32_t num_bursts)
{
  bool valid = (dev_idx == 0) ? (g_velux_nvm.blind_valid != 0) : (g_velux_nvm.sky_valid != 0);
  if (!valid) {
    printf("{\"event\":\"velux_error\",\"device\":\"%s\",\"error\":\"Device key not paired yet. Run 'velux pair %s' and press rear pair button.\"}\n",
           (dev_idx == 0) ? "blind" : "skylight",
           (dev_idx == 0) ? "blind" : "skylight");
    return;
  }

  const uint8_t *addr = (dev_idx == 0) ? g_velux_nvm.blind_addr : g_velux_nvm.sky_addr;
  const uint8_t *key  = (dev_idx == 0) ? g_velux_nvm.blind_key  : g_velux_nvm.sky_key;
  uint16_t seq_n      = (dev_idx == 0) ? g_velux_nvm.blind_seq  : g_velux_nvm.sky_seq;
  uint16_t seq_n1     = (uint16_t)((seq_n + 1U) & 0xFFFFU);

  // Update position & advance rolling sequence counter (+2 for single pair) in Flash before TX
  if (main0 <= 0xC8) {
    uint8_t ha_pos = (uint8_t)(100U - ((uint32_t)main0 * 100U) / 200U);
    if (dev_idx == 0) {
      g_velux_nvm.blind_pos = ha_pos;
    } else {
      g_velux_nvm.sky_pos = ha_pos;
    }
  }
  if (dev_idx == 0) {
    g_velux_nvm.blind_seq = (uint16_t)((seq_n + 2U) & 0xFFFFU);
  } else {
    g_velux_nvm.sky_seq = (uint16_t)((seq_n + 2U) & 0xFFFFU);
  }
  velux_nvm_save();

  if (num_bursts == 0) {
    num_bursts = 4;
  }

  uint8_t f30[30], f28[28];
  uint8_t bcast_hi = (dev_idx == 0) ? 0x00 : 0x01;
  uint8_t bcast_lo = (dev_idx == 0) ? 0x7f : 0x3f;
  velux_build_30b_frame(dev_idx, addr, key, seq_n, 0xC8, bcast_hi, bcast_lo, f30);
  velux_build_28b_frame(addr, key, seq_n1, main0, f28);

  // Stage 1: 30B Non-LPM (Trigger 4) @ seq = N
  memcpy((void *)g_velux_tx.psdu, f30, 28);
  memcpy((void *)g_velux_tx.psdu_ext, &f30[28], 2);
  g_velux_tx.psdu_len   = 30;
  g_velux_tx.num_bursts = num_bursts;
  g_velux_tx.trigger    = 4;
  transmit_velux_burst(railHandle);

  // Stage 2: 28B LPM (Trigger 3) @ seq = N + 1
  memcpy((void *)g_velux_tx.psdu, f28, 28);
  g_velux_tx.psdu_len   = 28;
  g_velux_tx.num_bursts = num_bursts;
  g_velux_tx.trigger    = 3;
  transmit_velux_burst(railHandle);

  velux_print_device_json("velux_state", dev_idx, action_name);
}

static void velux_start_pair_capture(int target_dev_idx)
{
  if (!g_velux_transfer_key_valid) {
    printf("{\"event\":\"velux_error\",\"error\":\"1-Way transfer key not set. Run 'velux set_transfer_key <32-hex>' first (or use 'velux setkey').\"}\n");
    return;
  }

  extern RAIL_DataConfig_t railDataConfig;
  extern bool rxFifoManual;
  g_velux_pair_mode = true;
  g_velux_pair_target = target_dev_idx;

  RAIL_Idle(railHandle, RAIL_IDLE_ABORT, true);
  railDataConfig.txMethod = PACKET_MODE;
  railDataConfig.rxMethod = FIFO_MODE;
  RAIL_ConfigData(railHandle, &railDataConfig);
  rxFifoManual = true;
  channel = 0;

  *(volatile uint32_t *)0xB8004108 = 0x00004000;
  *(volatile uint32_t *)0xB800410C = 0x00004CFF;
  *(volatile uint32_t *)0xB8004110 = 0x00004100;
  *(volatile uint32_t *)0xB8004114 = 0x00004DFF;
  RAIL_SetFixedLength(railHandle, 4095);
  RAIL_ResetFifo(railHandle, true, true);
  g_burst_write_idx = 0;
  g_burst_in_progress = false;
  g_burst_overflow = false;
  RAIL_StartRx(railHandle, 0, NULL);
  *(volatile uint32_t *)0xB8014050 = 0x0052C207;
  *(volatile uint32_t *)0xB8014078 = 0x6a;
  *(volatile uint32_t *)0xB801407C = 0x56;
  *(volatile uint32_t *)0xB8014080 = 0x08A01145;

  printf("{\"event\":\"velux_pairing_armed\",\"device\":\"%s\",\"info\":\"Press the rear PAIR/RESET button on the remote now...\"}\n",
         (target_dev_idx == 0) ? "blind" : "skylight");
}

static void velux_stop_pair_capture(void)
{
  extern RAIL_DataConfig_t railDataConfig;
  extern bool rxFifoManual;
  g_velux_pair_mode = false;
  RAIL_Idle(railHandle, RAIL_IDLE_ABORT, true);
  railDataConfig.txMethod = PACKET_MODE;
  railDataConfig.rxMethod = PACKET_MODE;
  RAIL_ConfigData(railHandle, &railDataConfig);
  rxFifoManual = false;
  RAIL_SetFixedLength(railHandle, 0);
  RAIL_ResetFifo(railHandle, true, true);
}

static void velux_on_rx_valid_frame(const uint8_t *psdu, uint16_t len)
{
  if (len < 12 || psdu[0] != 0x01 || psdu[1] != 0x69 || psdu[2] != 0x6f) {
    return;
  }

  const uint8_t *src = &psdu[8];

  // 1. Key Transfer Frame (PHR=0x22 / 34 bytes, Cmd=0x30 from rear PAIR button)
  if (len == 34 && psdu[11] == 0x30) {
    if (!g_velux_transfer_key_valid) {
      printf("{\"event\":\"velux_error\",\"error\":\"Captured 0x30 key frame, but 1-Way transfer key is not set! Run 'velux set_transfer_key <32-hex>' first.\"}\n");
      return;
    }
    uint8_t master_key[16];
    iohc_decrypt_1w_key(src, &psdu[12], master_key);
    uint16_t rx_seq = (uint16_t)(((uint16_t)psdu[30] << 8) | psdu[31]);

    int dev_idx = g_velux_pair_target;
    if (!g_velux_pair_mode) {
      if (memcmp(src, g_velux_nvm.blind_addr, 3) == 0) {
        dev_idx = 0;
      } else {
        dev_idx = 1;
      }
    }

    if (dev_idx == 0) {
      memcpy(g_velux_nvm.blind_addr, src, 3);
      memcpy(g_velux_nvm.blind_key, master_key, 16);
      g_velux_nvm.blind_seq = (uint16_t)((rx_seq + 1U) & 0xFFFFU);
      g_velux_nvm.blind_valid = 1;
    } else {
      memcpy(g_velux_nvm.sky_addr, src, 3);
      memcpy(g_velux_nvm.sky_key, master_key, 16);
      g_velux_nvm.sky_seq = (uint16_t)((rx_seq + 1U) & 0xFFFFU);
      g_velux_nvm.sky_valid = 1;
    }
    velux_nvm_save();

    if (g_velux_pair_mode) {
      velux_stop_pair_capture();
    }
    velux_print_device_json("velux_paired", dev_idx, "paired");
    return;
  }

  // 2. Movement / Status Frame (28B or 30B) -> Sync rolling counter & position!
  int matched_dev = -1;
  if (g_velux_nvm.blind_valid && memcmp(src, g_velux_nvm.blind_addr, 3) == 0) {
    matched_dev = 0;
  } else if (g_velux_nvm.sky_valid && memcmp(src, g_velux_nvm.sky_addr, 3) == 0) {
    matched_dev = 1;
  }

  if (matched_dev >= 0 && (len == 28 || len == 30)) {
    uint16_t rx_seq = (len == 28)
                      ? (uint16_t)(((uint16_t)psdu[18] << 8) | psdu[19])
                      : (uint16_t)(((uint16_t)psdu[20] << 8) | psdu[21]);
    const char *act = "sync";
    if (len == 28 && psdu[13] == 0x61) {
      uint8_t main0 = psdu[14];
      if (main0 == 0x00) {
        act = "open";
      } else if (main0 == 0xC8) {
        act = "close";
      } else if (main0 == 0xD2) {
        act = "stop";
      } else {
        act = "pos";
      }
      if (main0 <= 0xC8) {
        uint8_t ha_pos = (uint8_t)(100U - ((uint32_t)main0 * 100U) / 200U);
        if (matched_dev == 0) {
          g_velux_nvm.blind_pos = ha_pos;
        } else {
          g_velux_nvm.sky_pos = ha_pos;
        }
      }
    }
    if (matched_dev == 0) {
      g_velux_nvm.blind_seq = (uint16_t)((rx_seq + 1U) & 0xFFFFU);
    } else {
      g_velux_nvm.sky_seq = (uint16_t)((rx_seq + 1U) & 0xFFFFU);
    }
    velux_nvm_save();
    velux_print_device_json("velux_remote", matched_dev, act);
  }
}

static int parse_hex_bytes(const char *hex_str, uint8_t *out, int max_bytes)
{
  int count = 0;
  while (*hex_str && count < max_bytes) {
    while (*hex_str == ' ' || *hex_str == ':') hex_str++;
    if (!hex_str[0] || !hex_str[1]) break;
    char buf[3] = { hex_str[0], hex_str[1], 0 };
    out[count++] = (uint8_t)strtoul(buf, NULL, 16);
    hex_str += 2;
  }
  return count;
}

void cli_velux(sl_cli_command_arg_t *args)
{
  int argc = sl_cli_get_argument_count(args);
  if (argc == 0) {
    velux_print_device_json("velux_status", 0, "status");
    velux_print_device_json("velux_status", 1, "status");
    return;
  }

  const char *a0 = sl_cli_get_argument_string(args, 0);
  const char *a1 = (argc > 1) ? sl_cli_get_argument_string(args, 1) : NULL;
  const char *a2 = (argc > 2) ? sl_cli_get_argument_string(args, 2) : NULL;
  const char *a3 = (argc > 3) ? sl_cli_get_argument_string(args, 3) : NULL;

  if (strcmp(a0, "status") == 0 || strcmp(a0, "info") == 0) {
    velux_print_device_json("velux_status", 0, "status");
    velux_print_device_json("velux_status", 1, "status");
    return;
  }

  if (strcmp(a0, "set_transfer_key") == 0 && a1) {
    uint8_t tk[16];
    if (parse_hex_bytes(a1, tk, 16) == 16) {
      memcpy(g_velux_transfer_key, tk, 16);
      g_velux_transfer_key_valid = true;
      // Rewrite Flash page if Slot 127 was already written with an older key
      const VeluxTkSlot_t *tk_flash = (const VeluxTkSlot_t *)VELUX_NVM_TK_SLOT_ADDR;
      if (tk_flash->magic != 0xFFFFFFFFUL) {
        g_velux_nvm_next_slot = VELUX_NVM_SLOT_COUNT;
        velux_nvm_save();
      } else {
        velux_tk_write_if_needed();
      }
      printf("{\"event\":\"velux_transfer_key\",\"transfer_key_set\":true}\n");
    } else {
      printf("{\"event\":\"velux_error\",\"error\":\"Expected 32 hex chars for 16-byte transfer key\"}\n");
    }
    return;
  }

  if (strcmp(a0, "reset_nvm") == 0) {
    velux_flash_erase_page(VELUX_NVM_FLASH_PAGE_ADDR);
    memset(&g_velux_nvm, 0, sizeof(VeluxNvmSlot_t));
    memset(g_velux_transfer_key, 0, sizeof(g_velux_transfer_key));
    g_velux_transfer_key_valid = false;
    g_velux_nvm.magic = VELUX_NVM_MAGIC;
    g_velux_nvm.blind_seq = 1;
    g_velux_nvm.sky_seq = 1;
    g_velux_nvm_next_slot = 0;
    velux_nvm_save();
    printf("{\"event\":\"velux_nvm_reset\",\"status\":\"ok\"}\n");
    return;
  }

  if (strcmp(a0, "pair") == 0) {
    int target = 1; // default skylight
    if (a1 && (strcmp(a1, "blind") == 0 || strcmp(a1, "0") == 0)) {
      target = 0;
    }
    velux_start_pair_capture(target);
    return;
  }

  if (strcmp(a0, "stop_pair") == 0) {
    velux_stop_pair_capture();
    printf("{\"event\":\"velux_pairing_stopped\"}\n");
    return;
  }

  if (strcmp(a0, "setseq") == 0 && a1 && a2) {
    int dev = (strcmp(a1, "skylight") == 0 || strcmp(a1, "sky") == 0 || strcmp(a1, "1") == 0) ? 1 : 0;
    uint16_t seq = (uint16_t)strtoul(a2, NULL, 0);
    if (dev == 0) {
      g_velux_nvm.blind_seq = seq;
    } else {
      g_velux_nvm.sky_seq = seq;
    }
    velux_nvm_save();
    velux_print_device_json("velux_status", dev, "setseq");
    return;
  }

  if (strcmp(a0, "setkey") == 0 && a1 && a2) {
    int dev = (strcmp(a1, "skylight") == 0 || strcmp(a1, "sky") == 0 || strcmp(a1, "1") == 0) ? 1 : 0;
    uint8_t key[16];
    if (parse_hex_bytes(a2, key, 16) == 16) {
      if (dev == 0) {
        memcpy(g_velux_nvm.blind_key, key, 16);
        g_velux_nvm.blind_valid = 1;
        if (a3) parse_hex_bytes(a3, g_velux_nvm.blind_addr, 3);
      } else {
        memcpy(g_velux_nvm.sky_key, key, 16);
        g_velux_nvm.sky_valid = 1;
        if (a3) parse_hex_bytes(a3, g_velux_nvm.sky_addr, 3);
      }
      velux_nvm_save();
      velux_print_device_json("velux_status", dev, "setkey");
    } else {
      printf("{\"event\":\"velux_error\",\"error\":\"Expected 32 hex chars for 16-byte key\"}\n");
    }
    return;
  }

  int dev_idx = 0; // default blind
  const char *cmd_str = a0;
  const char *val_str = a1;

  if (strcmp(a0, "blind") == 0 || strcmp(a0, "0") == 0) {
    dev_idx = 0;
    cmd_str = a1 ? a1 : "status";
    val_str = a2;
  } else if (strcmp(a0, "skylight") == 0 || strcmp(a0, "sky") == 0 || strcmp(a0, "window") == 0 || strcmp(a0, "1") == 0) {
    dev_idx = 1;
    cmd_str = a1 ? a1 : "status";
    val_str = a2;
  } else if (a1 && (strcmp(a1, "blind") == 0 || strcmp(a1, "0") == 0)) {
    dev_idx = 0;
    val_str = a2;
  } else if (a1 && (strcmp(a1, "skylight") == 0 || strcmp(a1, "sky") == 0 || strcmp(a1, "window") == 0 || strcmp(a1, "1") == 0)) {
    dev_idx = 1;
    val_str = a2;
  }

  if (strcmp(cmd_str, "status") == 0) {
    velux_print_device_json("velux_status", dev_idx, "status");
  } else if (strcmp(cmd_str, "open") == 0 || strcmp(cmd_str, "up") == 0) {
    velux_exec_command(dev_idx, 0x00, "open", 3);
  } else if (strcmp(cmd_str, "close") == 0 || strcmp(cmd_str, "down") == 0) {
    velux_exec_command(dev_idx, 0xC8, "close", 3);
  } else if (strcmp(cmd_str, "stop") == 0 || strcmp(cmd_str, "my") == 0) {
    velux_exec_command(dev_idx, 0xD2, "stop", 3);
  } else if (strcmp(cmd_str, "vent") == 0 || strcmp(cmd_str, "half") == 0) {
    velux_exec_command(dev_idx, 0x64, "pos", 3);
  } else if (strcmp(cmd_str, "pos") == 0 || strcmp(cmd_str, "position") == 0) {
    uint32_t ha_pct = val_str ? strtoul(val_str, NULL, 10) : 50;
    if (ha_pct > 100) ha_pct = 100;
    // HA: 100% = fully open (0x00), 0% = fully closed (0xC8 = 200)
    uint8_t main0 = (uint8_t)(200U - (ha_pct * 2U));
    velux_exec_command(dev_idx, main0, "pos", 3);
  } else if (strcmp(cmd_str, "pair") == 0) {
    velux_start_pair_capture(dev_idx);
  } else {
    printf("{\"event\":\"velux_error\",\"error\":\"Unknown command '%s'. Use open|close|stop|pos|status|pair|setseq|setkey|set_transfer_key|reset_nvm\"}\n", cmd_str);
  }
}

// -----------------------------------------------------------------------------
// Application Init
// -----------------------------------------------------------------------------
void app_init(void)
{
  // 1. Initialize XIAO MG24 RF Antenna Switch:
  //    PB05 = HIGH (1): Power onboard RF antenna switch IC
  sl_gpio_t rf_sw_pw = { .port = SL_GPIO_PORT_B, .pin = 5 };
  sl_gpio_set_pin_mode(&rf_sw_pw, SL_GPIO_MODE_PUSH_PULL, 1);

  //    PB04 = LOW (0): Select onboard ceramic chip antenna (0=onboard, 1=U.FL)
  sl_gpio_t rf_sw_sel = { .port = SL_GPIO_PORT_B, .pin = 4 };
  sl_gpio_set_pin_mode(&rf_sw_sel, SL_GPIO_MODE_PUSH_PULL, 0);

  // 2. Configure zero-UART-stall capture:
  rxHeld = true;
  rxProcessHeld = false;
  logLevel &= ~ASYNC_RESPONSE;

  // 3. Clear RAM burst buffer
  g_burst_write_idx = 0;
  g_burst_packet_count = 0;
  g_burst_in_progress = false;
  g_burst_overflow = false;
  g_last_rx_time = 0;

  // 4. Initialize Non-Volatile Velux Profiles (Blind + Skylight) in Flash
  velux_nvm_init();

  printf("[VELUX-TX] Control block at 0x%08lX (size=%u)\n",
         (unsigned long)&g_velux_tx, (unsigned int)sizeof(g_velux_tx));
  velux_print_device_json("velux_boot", 0, "boot");
  velux_print_device_json("velux_boot", 1, "boot");
}

// -----------------------------------------------------------------------------
// Application Process Action (super loop)
// -----------------------------------------------------------------------------
void app_process_action(void)
{
  // 0. Direct Hardware Burst Transmitter Trigger Check
  if (g_velux_tx.trigger != 0) {
    transmit_velux_burst(railHandle);
  }

  rxProcessHeld = false;

  // 1. Packet Mode Burst Capture:
  while (packetsHeld > 0) {
    rxProcessHeld = false;
    RAIL_RxPacketInfo_t packetInfo;
    RAIL_RxPacketHandle_t handle =
      RAIL_GetRxPacketInfo(railHandle, RAIL_RX_PACKET_HANDLE_OLDEST_COMPLETE, &packetInfo);

    if (handle == RAIL_RX_PACKET_HANDLE_INVALID) {
      break;
    }

    RAIL_RxPacketDetails_t details;
    RAIL_Status_t det_status = RAIL_GetRxPacketDetailsAlt(railHandle, handle, &details);

    uint16_t pkt_len = packetInfo.packetBytes;
    uint32_t total_record_len = sizeof(BurstPacketHeader_t) + pkt_len;

    if (g_burst_write_idx + total_record_len <= BURST_BUFFER_SIZE) {
      BurstPacketHeader_t *hdr = (BurstPacketHeader_t *)&g_burst_buffer[g_burst_write_idx];

      hdr->timestamp_us  = (det_status == RAIL_STATUS_NO_ERROR) ? details.timeReceived.packetTime : RAIL_GetTime();
      hdr->rssi          = (det_status == RAIL_STATUS_NO_ERROR) ? details.rssi : 0;
      hdr->lqi           = (det_status == RAIL_STATUS_NO_ERROR) ? details.lqi : 0;
      hdr->length        = pkt_len;
      hdr->packet_status = (uint8_t)packetInfo.packetStatus;
      hdr->sub_phy_id    = (det_status == RAIL_STATUS_NO_ERROR) ? details.subPhyId : 0;
      hdr->freq_offset   = (int16_t)rxFreqOffset;
      hdr->channel       = channel;

      RAIL_CopyRxPacket(&g_burst_buffer[g_burst_write_idx + sizeof(BurstPacketHeader_t)], &packetInfo);

      g_burst_write_idx += total_record_len;
      g_burst_packet_count++;
    } else {
      g_burst_overflow = true;
    }

    RAIL_ReleaseRxPacket(railHandle, handle);

    CORE_DECLARE_IRQ_STATE;
    CORE_ENTER_CRITICAL();
    if (packetsHeld > 0) {
      packetsHeld--;
    }
    CORE_EXIT_CRITICAL();

    g_last_rx_time = RAIL_GetTime();
    g_burst_in_progress = true;
  }

  // 2. FIFO Stream Mode Burst Capture:
  if (railDataConfig.rxMethod == FIFO_MODE && rxFifoManual) {
    static bool s_resynced_stage2 = false;
    if (!g_burst_in_progress) {
      s_resynced_stage2 = false;
    }

    uint16_t avail = RAIL_GetRxFifoBytesAvailable(railHandle);
    if (avail > 0) {
      uint32_t space = (g_burst_write_idx < FIFO_STREAM_CAPTURE_BYTES)
                       ? (FIFO_STREAM_CAPTURE_BYTES - g_burst_write_idx) : 0;
      uint16_t to_read = (avail > space) ? (uint16_t)space : avail;
      if (to_read > 0) {
        uint16_t read_bytes = RAIL_ReadRxFifo(railHandle, &g_burst_buffer[g_burst_write_idx], to_read);
        g_burst_write_idx += read_bytes;
        g_last_rx_time = RAIL_GetTime();
        g_burst_in_progress = true;
      }
      if (avail > space) {
        g_burst_overflow = true;
      }
    }

    if (g_burst_in_progress && !s_resynced_stage2 && g_burst_write_idx >= 5500) {
      s_resynced_stage2 = true;
      RAIL_Idle(railHandle, RAIL_IDLE_ABORT, true);
      RAIL_ResetFifo(railHandle, false, true);
      RAIL_StartRx(railHandle, channel, NULL);
      *(volatile uint32_t *)0xB8014050 = 0x0052C007;
      *(volatile uint32_t *)0xB8014078 = 0x56;
      *(volatile uint32_t *)0xB8014080 = 0x08A01145;
      g_last_rx_time = RAIL_GetTime();
    }

    uint32_t timeout_us = s_resynced_stage2 ? 500000UL : BURST_IDLE_TIMEOUT_US;
    if (g_burst_in_progress &&
        ((g_burst_write_idx >= FIFO_STREAM_CAPTURE_BYTES) ||
         (RAIL_GetTime() - g_last_rx_time > timeout_us))) {

      RAIL_Idle(railHandle, RAIL_IDLE_ABORT, true);
      RAIL_ResetFifo(railHandle, false, true);

      dump_fifo_stream_over_serial();

      g_burst_write_idx = 0;
      g_burst_in_progress = false;
      g_burst_overflow = false;
      s_resynced_stage2 = false;
      RAIL_StartRx(railHandle, channel, NULL);
      *(volatile uint32_t *)0xB8014050 = 0x0052C207;
      *(volatile uint32_t *)0xB8014078 = 0x6a;
      *(volatile uint32_t *)0xB801407C = 0x56;
      *(volatile uint32_t *)0xB8014080 = 0x08A01145;
    }
    return;
  }

  // 3. Burst Timeout Detection:
  if (g_burst_in_progress && (RAIL_GetTime() - g_last_rx_time > BURST_IDLE_TIMEOUT_US)) {
    g_burst_in_progress = false;

    dump_burst_over_serial();

    g_burst_write_idx = 0;
    g_burst_packet_count = 0;
    g_burst_overflow = false;
  }
}
