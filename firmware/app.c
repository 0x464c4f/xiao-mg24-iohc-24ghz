/***************************************************************************//**
 * @file app.c
 * @brief 2.4 GHz io-homecontrol 1-Way controller (blind + skylight) for the
 *        Seeed Studio XIAO MG24 (EFR32MG24B220F1536IM48), built on RAILtest.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Credits:
 * - Developed with Google Antigravity (AGY) powered by Gemini.
 * - 1-Way MAC transformation (iohc_checksum_step / iohc_create_1w_hmac)
 *   adapted from rspaargaren/iohomecontrol (Apache-2.0).
 *
 * Layout:
 *   1. Helpers ............ CRC-16/KERMIT, Z2xZ8 nibble inversion
 *   2. RX capture ......... packet-mode burst dump + FIFO stream decoder
 *   3. TX engine .......... 1000 us beacon grid, LPM / non-LPM sub-bursts
 *   4. Crypto ............. AES-128, 1-Way MAC, key-transfer unwrap
 *   5. Flash NVM .......... wear-leveled device state + transfer key slot
 *   6. Device logic / CLI . `velux ...` command
 *   7. app_init / app_process_action
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

// RAILtest globals used by this module
extern RAIL_DataConfig_t railDataConfig;
extern bool rxFifoManual;

static void velux_on_rx_valid_frame(const uint8_t *psdu, uint16_t len);

// -----------------------------------------------------------------------------
// Configuration
// -----------------------------------------------------------------------------
#define BURST_BUFFER_SIZE           (64 * 1024)   // RX capture RAM buffer
#define BURST_IDLE_TIMEOUT_US       120000UL      // 120 ms silence = end of burst
#define FIFO_STREAM_CAPTURE_BYTES   12000UL       // ~384 ms continuous capture
#define FIFO_STAGE2_RESYNC_BYTES    5500UL        // re-arm RX after Stage 1

#define VELUX_TX_POWER_DDBM         100           // +10 dBm
#define VELUX_DEFAULT_BURSTS        3U            // channel repetitions per stage

// Channels: base 2425 MHz, 5 MHz spacing -> 2425 / 2450 / 2475 MHz
static const uint16_t VELUX_CHANNELS[3] = { 0, 5, 10 };

// Direct register addresses (EFR32xG24 non-secure aliases)
#define REG_FRC_FCD0      (*(volatile uint32_t *)0xB8004108UL)
#define REG_FRC_FCD1      (*(volatile uint32_t *)0xB800410CUL)
#define REG_FRC_FCD2      (*(volatile uint32_t *)0xB8004110UL)
#define REG_FRC_FCD3      (*(volatile uint32_t *)0xB8004114UL)
#define REG_MODEM_CTRL1   (*(volatile uint32_t *)0xB8014050UL)
#define REG_MODEM_SYNC0   (*(volatile uint32_t *)0xB8014078UL)
#define REG_MODEM_SYNC1   (*(volatile uint32_t *)0xB801407CUL)
#define REG_MODEM_TIMING  (*(volatile uint32_t *)0xB8014080UL)

// -----------------------------------------------------------------------------
// RX capture state
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

static uint8_t  g_burst_buffer[BURST_BUFFER_SIZE];
static uint32_t g_burst_write_idx = 0;
static uint32_t g_burst_packet_count = 0;
static bool     g_burst_in_progress = false;
static bool     g_burst_overflow = false;
static uint32_t g_last_rx_time = 0;

// =============================================================================
// 1. Helpers
// =============================================================================

// CRC-16/KERMIT (poly 0x8408 reflected, init 0x0000)
static uint16_t kermit_crc16(const uint8_t *data, uint16_t len)
{
  uint16_t crc = 0x0000;
  for (uint16_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (int b = 0; b < 8; b++) {
      crc = (uint16_t)((crc >> 1) ^ ((crc & 1) ? 0x8408 : 0));
    }
  }
  return crc;
}

// Undo the Z2 x Z8 symbol permutation for a receiver locked at phase P
static inline uint8_t inv_nibble(uint8_t n, uint8_t P)
{
  return (uint8_t)((((n + 8 - (P & 7)) & 7) | ((n ^ P) & 8)));
}

static inline uint8_t get_nibble(const uint8_t *buf, uint32_t nib_idx)
{
  return (nib_idx & 1) ? ((buf[nib_idx >> 1] >> 4) & 0x0f)
                       : (buf[nib_idx >> 1] & 0x0f);
}

static void decode_bytes(const uint8_t *buf, uint32_t first_nib, uint8_t P,
                         uint8_t *out, uint16_t len)
{
  for (uint16_t b = 0; b < len; b++) {
    uint8_t n0 = get_nibble(buf, first_nib + 2U * b);
    uint8_t n1 = get_nibble(buf, first_nib + 2U * b + 1U);
    out[b] = (uint8_t)(inv_nibble(n0, P) | (inv_nibble(n1, P) << 4));
  }
}

static void print_hex_line(const uint8_t *data, uint16_t len)
{
  for (uint16_t i = 0; i < len; i++) {
    printf("%02x ", data[i]);
  }
  printf("\n");
}

// Command / pairing frames are 10..36 bytes; 5-byte frames are wake-up beacons
static inline bool is_command_len(uint8_t phr)
{
  return phr >= 0x0A && phr <= 0x24;
}

static inline bool sfd_at(const uint8_t *buf, uint32_t nib, uint8_t P)
{
  return inv_nibble(get_nibble(buf, nib), P) == 6
         && inv_nibble(get_nibble(buf, nib + 1), P) == 5;
}

static uint32_t preamble_run(const uint8_t *buf, uint32_t nib, uint8_t P)
{
  uint32_t n = 0;
  while ((nib - 1 >= n) && (get_nibble(buf, nib - 1 - n) == P)) {
    n++;
  }
  return n;
}

// =============================================================================
// 2. RX capture
// =============================================================================

// Packet-mode burst dump (diagnostic): prints every captured packet with
// timing / RSSI / LQI, plus any frame decodable via a preamble+SFD search.
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
    const uint8_t *payload = &g_burst_buffer[offset];
    offset += hdr->length;

    printf("[%03lu] +%4ld ms | ch:%2u | rssi:%4d | lqi:%3u | st:%02x | len:%2u | ",
           (unsigned long)i, (long)((hdr->timestamp_us - first_t) / 1000),
           hdr->channel, hdr->rssi, hdr->lqi, hdr->packet_status, hdr->length);
    print_hex_line(payload, hdr->length);

    // Case 1: hardware locked directly on SFD 0x56 (PHR at byte 0)
    uint32_t nib = 4;
    if (hdr->length >= 4) {
      uint8_t phr = payload[0];
      if (phr >= 3 && phr <= 120 && (uint16_t)(1 + phr) <= hdr->length
          && kermit_crc16(&payload[1], phr) == 0) {
        printf("  ==> [CRC-OK] off=0 P=0 PHR=%02x PSDU: ", phr);
        print_hex_line(&payload[1], phr);
        if (is_command_len(phr)) {
          velux_on_rx_valid_frame(&payload[1], phr);
        }
        nib = 2U * (1U + phr) + 4U;
      }
    }

    // Case 2: preamble + inverted SFD at any nibble alignment / phase
    uint32_t total_nibs = (uint32_t)hdr->length * 2U;
    while (nib + 1 < total_nibs) {
      uint8_t P = get_nibble(payload, nib - 1);
      if (preamble_run(payload, nib, P) >= 4 && sfd_at(payload, nib, P)
          && nib + 4 <= total_nibs) {
        uint8_t phr;
        decode_bytes(payload, nib + 2, P, &phr, 1);
        uint32_t remaining = (total_nibs - (nib + 4)) / 2;
        if (phr >= 3 && phr <= 128 && remaining >= phr) {
          uint8_t psdu[128];
          decode_bytes(payload, nib + 4, P, psdu, phr);
          if (kermit_crc16(psdu, phr) == 0) {
            printf("  ==> [CRC-OK] off=%lu align=%lu P=%u PHR=%02x PSDU: ",
                   (unsigned long)(nib / 2), (unsigned long)(nib % 2), P, phr);
            print_hex_line(psdu, phr);
            if (is_command_len(phr)) {
              velux_on_rx_valid_frame(psdu, phr);
            }
            nib += 2U + 2U * (1U + phr);
            continue;
          }
        }
      }
      nib++;
    }
  }

  printf("--- BURST CAPTURE DUMP END ---\n\n");
}

// FIFO stream decoder (raw demodulator stream, 32 us per byte). Used while
// pairing and to follow wall-switch presses for rolling-counter sync.
static void dump_fifo_stream_over_serial(void)
{
  if (g_burst_write_idx == 0) {
    return;
  }

  printf("\n--- FIFO STREAM CAPTURE DUMP START (bytes=%lu, duration_ms=%lu, ovf=%d) ---\n",
         (unsigned long)g_burst_write_idx,
         (unsigned long)((g_burst_write_idx * 32UL) / 1000UL),
         g_burst_overflow ? 1 : 0);

  const uint8_t *buf = g_burst_buffer;
  uint32_t total_nibs = g_burst_write_idx * 2U;
  uint32_t total_crc_ok = 0;
  uint32_t total_cmd = 0;
  bool printed_first_beacon = false;

  // Pass 1: preamble-locked frames (beacons + commands)
  uint32_t nib = 4;
  while (nib + 1 < total_nibs) {
    uint8_t P = get_nibble(buf, nib - 1);
    if (preamble_run(buf, nib, P) >= 4 && sfd_at(buf, nib, P) && nib + 4 <= total_nibs) {
      uint8_t phr;
      decode_bytes(buf, nib + 2, P, &phr, 1);
      uint32_t remaining = (total_nibs - (nib + 4)) / 2;
      if (phr >= 3 && phr <= 36 && remaining >= phr) {
        uint8_t psdu[36];
        decode_bytes(buf, nib + 4, P, psdu, phr);
        if (kermit_crc16(psdu, phr) == 0) {
          bool is_cmd = is_command_len(phr);
          bool print_it = is_cmd;
          if (!is_cmd && phr == 0x05 && psdu[0] == 0x00 && psdu[1] == 0x00 && psdu[2] <= 0x6A) {
            // Only print first / last few wake-up beacons to keep output short
            if (!printed_first_beacon || psdu[2] >= 0x68 || psdu[2] <= 0x08) {
              print_it = true;
              printed_first_beacon = true;
            }
          }
          if (print_it) {
            uint32_t off = nib / 2;
            total_crc_ok++;
            printf("[HOP] +%4lu.%01lu ms (off=%5lu align=%lu P=%2u) | [CRC-OK] PHR=%02x PSDU: ",
                   (unsigned long)((off * 32UL) / 1000UL),
                   (unsigned long)(((off * 32UL) % 1000UL) / 100UL),
                   (unsigned long)off, (unsigned long)(nib % 2), P, phr);
            print_hex_line(psdu, phr);
          }
          if (is_cmd) {
            total_cmd++;
            printf("  *** NEW COMMAND FRAME DETECTED! ***\n");
            velux_on_rx_valid_frame(psdu, phr);
          }
          nib += 2U + 2U * (1U + phr);
          continue;
        }
      }
    }
    nib++;
  }

  // Pass 2: exhaustive search for command frames without a clean preamble
  // (all 16 phases, PSDU must start with 0x01)
  uint32_t end_nib = (total_nibs > (2 + 2 * 35)) ? (total_nibs - (2 + 2 * 35)) : 0;
  nib = 4;
  while (nib < end_nib) {
    bool found = false;
    for (uint8_t P = 0; P < 16 && !found; P++) {
      if (!sfd_at(buf, nib, P)) {
        continue;
      }
      uint8_t hdr2[2];
      decode_bytes(buf, nib + 2, P, hdr2, 2);   // PHR + first PSDU byte
      uint8_t phr = hdr2[0];
      if (!is_command_len(phr) || hdr2[1] != 0x01) {
        continue;
      }
      uint8_t psdu[36];
      decode_bytes(buf, nib + 4, P, psdu, phr);
      if (kermit_crc16(psdu, phr) != 0) {
        continue;
      }
      uint32_t off = nib / 2;
      printf("[EXHAUSTIVE] +%4lu.%01lu ms (off=%5lu align=%lu P=%2u) | [CRC-OK] PSDU: ",
             (unsigned long)((off * 32UL) / 1000UL),
             (unsigned long)(((off * 32UL) % 1000UL) / 100UL),
             (unsigned long)off, (unsigned long)(nib % 2), P);
      print_hex_line(psdu, phr);
      velux_on_rx_valid_frame(psdu, phr);
      total_cmd++;
      total_crc_ok++;
      nib += 2U + 2U * (1U + phr);
      found = true;
    }
    if (!found) {
      nib++;
    }
  }

  printf("--- SUMMARY: CRC-OK=%lu, NEW-COMMAND=%lu ---\n",
         (unsigned long)total_crc_ok, (unsigned long)total_cmd);
  printf("--- BURST CAPTURE DUMP END ---\n\n");
}

// Re-arm FIFO stream RX with dual sync words (0x6A raw / 0x56 on-air)
static void fifo_rx_rearm(void)
{
  RAIL_StartRx(railHandle, channel, NULL);
  REG_MODEM_CTRL1  = 0x0052C207;   // DUALSYNC enabled
  REG_MODEM_SYNC0  = 0x6A;
  REG_MODEM_SYNC1  = 0x56;
  REG_MODEM_TIMING = 0x08A01145;
}

// =============================================================================
// 3. TX engine
// =============================================================================
// Every transmission on a channel is a countdown beacon train followed by
// 4 command windows, all on an exact 1000 us grid:
//   LPM     (147 ms): beacons cnt=106..0, windows at tick 107/120/133/146
//   non-LPM ( 47 ms): beacons cnt=7..0,   windows at tick 8/21/34/47
// Between windows: 4 ms silence, then a short cnt=7..0 countdown.

typedef struct {
  uint32_t ok_count;
  uint32_t err_count;
  uint32_t beacon_dur_us;
  uint32_t cmd_dur_us;
} TxDiag_t;

static TxDiag_t g_tx_diag;
static uint32_t g_tx_done_count = 0;

// TX overrides: on-air SFD 0x56, and FCD0..3 without CRC generation so the
// software CRC-16/KERMIT at the end of the PSDU is sent unmodified. Some RAIL
// calls rewrite these registers, so they are re-applied before each step.
static inline void apply_tx_overrides(void)
{
  REG_MODEM_SYNC0 = 0x56;
  REG_FRC_FCD0 = 0x000040FF;
  REG_FRC_FCD1 = 0x000040FF;
  REG_FRC_FCD2 = 0x000040FF;
  REG_FRC_FCD3 = 0x000040FF;
}

static inline void restore_rx_fcd(void)
{
  REG_FRC_FCD0 = 0x00004000;
  REG_FRC_FCD1 = 0x00004CFF;
  REG_FRC_FCD2 = 0x00004100;
  REG_FRC_FCD3 = 0x00004DFF;
}

// Transmit one frame (PHR + PSDU) starting exactly at t_target_us
static void tx_at_grid(uint16_t ch, const uint8_t *pkt, uint16_t len, uint32_t t_target_us)
{
  RAIL_Idle(railHandle, RAIL_IDLE_ABORT, true);
  apply_tx_overrides();
  RAIL_SetFixedLength(railHandle, len);
  apply_tx_overrides();
  RAIL_ResetFifo(railHandle, true, false);
  RAIL_WriteTxFifo(railHandle, pkt, len, true);

  while ((int32_t)(t_target_us - RAIL_GetTime()) > 0) {
  }
  apply_tx_overrides();
  uint32_t t0 = RAIL_GetTime();
  RAIL_StartTx(railHandle, ch, RAIL_TX_OPTION_REMOVE_CRC, NULL);

  // Wait for TX to start (max 500 us), then to finish (max 3 ms)
  while (!(RAIL_GetRadioState(railHandle) & RAIL_RF_STATE_TX)
         && ((uint32_t)(RAIL_GetTime() - t0) < 500UL)) {
  }
  if (RAIL_GetRadioState(railHandle) & RAIL_RF_STATE_TX) {
    g_tx_diag.ok_count++;
  } else {
    g_tx_diag.err_count++;
  }
  while ((RAIL_GetRadioState(railHandle) & RAIL_RF_STATE_TX)
         && ((uint32_t)(RAIL_GetTime() - t0) < 3000UL)) {
  }
  uint32_t dur_us = RAIL_GetTime() - t0;
  if (len == 6) {
    g_tx_diag.beacon_dur_us = dur_us;
  } else {
    g_tx_diag.cmd_dur_us = dur_us;
  }
}

// 6-byte wake-up beacon: 05 00 cnt_hi cnt_lo crc_lo crc_hi (all zero at cnt=0)
static void tx_beacon_tick(uint16_t ch, int cnt, uint32_t t_target_us)
{
  uint8_t pkt[6] = { 0x05, 0x00, (uint8_t)((cnt >> 8) & 0xFF), (uint8_t)(cnt & 0xFF), 0x00, 0x00 };
  if (cnt != 0) {
    uint16_t crc = kermit_crc16(&pkt[1], 3);
    pkt[4] = (uint8_t)(crc & 0xFF);
    pkt[5] = (uint8_t)(crc >> 8);
  }
  tx_at_grid(ch, pkt, 6, t_target_us);
}

static void tx_short_countdown(uint16_t ch, uint32_t t_origin, uint32_t first_tick)
{
  for (int s = 0; s <= 7; s++) {
    tx_beacon_tick(ch, 7 - s, t_origin + (first_tick + (uint32_t)s) * 1000UL);
  }
}

// One sub-burst on one channel. w_first is used for window 0 in LPM mode
// (CB2=0x20); w_rest (CB2=0x00) for all other windows.
static void tx_subburst(uint16_t ch, const uint8_t *w_first, const uint8_t *w_rest,
                        uint16_t pkt_len, bool lpm)
{
  uint32_t t_origin = RAIL_GetTime() + 500UL;
  uint32_t first_window;

  if (lpm) {
    for (int tick = 0; tick <= 106; tick++) {
      tx_beacon_tick(ch, 106 - tick, t_origin + (uint32_t)tick * 1000UL);
    }
    first_window = 107;
  } else {
    tx_short_countdown(ch, t_origin, 0);
    first_window = 8;
  }

  for (uint32_t w = 0; w < 4; w++) {
    uint32_t w_tick = first_window + w * 13UL;
    const uint8_t *pkt = (lpm && w == 0) ? w_first : w_rest;
    tx_at_grid(ch, pkt, pkt_len, t_origin + w_tick * 1000UL);
    if (w < 3) {
      tx_short_countdown(ch, t_origin, w_tick + 5UL);
    }
  }
}

static void build_window_pkt(uint8_t *pkt, const uint8_t *psdu, uint16_t psdu_len, uint8_t cb2)
{
  pkt[0] = (uint8_t)psdu_len;
  memcpy(&pkt[1], psdu, psdu_len);
  pkt[1 + 4] = cb2;
  uint16_t crc = kermit_crc16(&pkt[1], (uint16_t)(psdu_len - 2));
  pkt[1 + psdu_len - 2] = (uint8_t)(crc & 0xFF);
  pkt[1 + psdu_len - 1] = (uint8_t)(crc >> 8);
}

// Transmit a PSDU (10..36 bytes incl. CRC) across 2425/2450/2475 MHz,
// `bursts` times, as LPM or non-LPM sub-bursts.
static void velux_transmit(const uint8_t *psdu, uint16_t psdu_len, bool lpm, uint32_t bursts)
{
  if (psdu_len < 10 || psdu_len > 36) {
    return;
  }
  if (bursts == 0) {
    bursts = 1;
  }

  // RF switch: PB05=1 (power), PB04=0 (onboard antenna)
  sl_gpio_t rf_sw_pw  = { .port = SL_GPIO_PORT_B, .pin = 5 };
  sl_gpio_t rf_sw_sel = { .port = SL_GPIO_PORT_B, .pin = 4 };
  sl_gpio_set_pin_mode(&rf_sw_pw, SL_GPIO_MODE_PUSH_PULL, 1);
  sl_gpio_set_pin_mode(&rf_sw_sel, SL_GPIO_MODE_PUSH_PULL, 0);

  RAIL_Idle(railHandle, RAIL_IDLE_ABORT, true);
  railDataConfig.txMethod = PACKET_MODE;
  railDataConfig.rxMethod = PACKET_MODE;
  RAIL_ConfigData(railHandle, &railDataConfig);
  rxFifoManual = false;
  RAIL_ResetFifo(railHandle, true, true);

  RAIL_SyncWordConfig_t tx_sync = { .syncWordBits = 8U, .syncWord1 = 0x56UL, .syncWord2 = 0x6AUL };
  RAIL_ConfigSyncWords(railHandle, &tx_sync);

  memset(&g_tx_diag, 0, sizeof(g_tx_diag));
  REG_MODEM_CTRL1 = 0x0052C007;   // single sync word
  apply_tx_overrides();

  RAIL_SetTxPowerDbm(railHandle, VELUX_TX_POWER_DDBM);
  RAIL_StateTransitions_t tx_trans = { .success = RAIL_RF_STATE_IDLE, .error = RAIL_RF_STATE_IDLE };
  RAIL_SetTxTransitions(railHandle, &tx_trans);

  uint16_t pkt_len = (uint16_t)(1 + psdu_len);
  uint8_t w_first[37], w_rest[37];
  build_window_pkt(w_first, psdu, psdu_len, 0x20);
  build_window_pkt(w_rest, psdu, psdu_len, 0x00);

  printf("\n[VELUX-TX] %s x%lu PSDU (%uB): ", lpm ? "LPM" : "non-LPM",
         (unsigned long)bursts, (unsigned int)psdu_len);
  print_hex_line(&w_rest[1], psdu_len);

  uint32_t t_start = RAIL_GetTime();
  for (uint32_t b = 0; b < bursts; b++) {
    tx_subburst(VELUX_CHANNELS[b % 3], w_first, w_rest, pkt_len, lpm);
    if (b + 1 < bursts) {
      uint32_t t_pause = RAIL_GetTime();
      while ((uint32_t)(RAIL_GetTime() - t_pause) < 15000UL) {
      }
    }
  }
  uint32_t total_ms = (RAIL_GetTime() - t_start) / 1000UL;

  // Restore RX configuration
  RAIL_Idle(railHandle, RAIL_IDLE_ABORT, true);
  restore_rx_fcd();
  REG_MODEM_SYNC0 = 0x6A;
  REG_MODEM_SYNC1 = 0x56;
  RAIL_SetFixedLength(railHandle, 0);
  RAIL_SyncWordConfig_t rx_sync = { .syncWordBits = 8U, .syncWord1 = 0x6AUL, .syncWord2 = 0x56UL };
  RAIL_ConfigSyncWords(railHandle, &rx_sync);

  g_tx_done_count++;
  printf("[VELUX-TX] done (count=%lu, total_ms=%lu, ok=%lu, err=%lu, bcn_us=%lu, cmd_us=%lu)\n",
         (unsigned long)g_tx_done_count, (unsigned long)total_ms,
         (unsigned long)g_tx_diag.ok_count, (unsigned long)g_tx_diag.err_count,
         (unsigned long)g_tx_diag.beacon_dur_us, (unsigned long)g_tx_diag.cmd_dur_us);
}

// =============================================================================
// 4. Crypto  /  5. Flash NVM
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

// =============================================================================
// 6. Device logic / CLI
// =============================================================================
// Device index: 0 = blind (KLI 312), 1 = skylight (KLI 311)

static inline const char *dev_name(int dev)
{
  return (dev == 0) ? "blind" : "skylight";
}

static bool parse_device(const char *s, int *dev)
{
  if (s == NULL) {
    return false;
  }
  if (strcmp(s, "blind") == 0 || strcmp(s, "0") == 0) {
    *dev = 0;
    return true;
  }
  if (strcmp(s, "skylight") == 0 || strcmp(s, "sky") == 0
      || strcmp(s, "window") == 0 || strcmp(s, "1") == 0) {
    *dev = 1;
    return true;
  }
  return false;
}

// Home Assistant position (0 = closed, 100 = open) <-> io-homecontrol main
// parameter (0x00 = open, 0xC8 = closed)
static inline uint8_t main0_to_ha_pct(uint8_t main0)
{
  return (uint8_t)(100U - ((uint32_t)main0 * 100U) / 200U);
}

static void velux_print_device_json(const char *event, int dev, const char *action)
{
  const uint8_t *addr = (dev == 0) ? g_velux_nvm.blind_addr : g_velux_nvm.sky_addr;
  const uint8_t *key  = (dev == 0) ? g_velux_nvm.blind_key  : g_velux_nvm.sky_key;
  uint16_t seq        = (dev == 0) ? g_velux_nvm.blind_seq  : g_velux_nvm.sky_seq;
  uint8_t  pos        = (dev == 0) ? g_velux_nvm.blind_pos  : g_velux_nvm.sky_pos;
  bool     paired     = (dev == 0) ? (g_velux_nvm.blind_valid != 0) : (g_velux_nvm.sky_valid != 0);

  printf("{\"event\":\"%s\",\"device\":\"%s\",\"addr\":\"%02x%02x%02x\",\"action\":\"%s\","
         "\"position\":%u,\"seq\":%u,\"seq_hex\":\"0x%04X\",\"paired\":%s,"
         "\"transfer_key_set\":%s,\"key\":\"",
         event, dev_name(dev), addr[0], addr[1], addr[2], action,
         (unsigned int)pos, (unsigned int)seq, (unsigned int)seq,
         paired ? "true" : "false",
         g_velux_transfer_key_valid ? "true" : "false");
  for (int i = 0; i < 16; i++) {
    printf("%02x", key[i]);
  }
  printf("\"}\n");
}

static void velux_print_error(const char *msg)
{
  printf("{\"event\":\"velux_error\",\"error\":\"%s\"}\n", msg);
}

// Common 1-Way frame header: 01 69 6f <ctrl1> 20 00 <bcast 2B> <src 3B>
static void build_header(uint8_t *out, uint8_t ctrl1, uint8_t bcast_hi, uint8_t bcast_lo,
                         const uint8_t addr[3])
{
  out[0] = 0x01;
  out[1] = 0x69;
  out[2] = 0x6f;
  out[3] = ctrl1;
  out[4] = 0x20;
  out[5] = 0x00;
  out[6] = bcast_hi;
  out[7] = bcast_lo;
  memcpy(&out[8], addr, 3);
}

// Append appli payload, sequence, 6-byte MAC and CRC; returns total length
static uint16_t finish_frame(uint8_t *out, const uint8_t *appli, uint8_t appli_len,
                             uint16_t seq, const uint8_t key[16])
{
  uint16_t i = 11;
  memcpy(&out[i], appli, appli_len);
  i += appli_len;
  out[i++] = (uint8_t)(seq >> 8);
  out[i++] = (uint8_t)(seq & 0xFF);
  iohc_create_1w_hmac(&out[i], seq, key, appli, appli_len);
  i += 6;
  uint16_t crc = kermit_crc16(out, i);
  out[i++] = (uint8_t)(crc & 0xFF);
  out[i++] = (uint8_t)(crc >> 8);
  return i;
}

// Stage 1: 30-byte non-LPM pre-command (byte 17 is always 0xC8)
static void velux_build_30b_frame(int dev, const uint8_t addr[3], const uint8_t key[16],
                                  uint16_t seq, uint8_t out[30])
{
  static const uint8_t appli[9] = { 0x00, 0x01, 0x61, 0xd4, 0x00, 0x80, 0xc8, 0x00, 0x00 };
  // Broadcast class: blind 00 7f, skylight (window opener) 01 3f
  build_header(out, 0xf8, (dev == 0) ? 0x00 : 0x01, (dev == 0) ? 0x7f : 0x3f, addr);
  finish_frame(out, appli, sizeof(appli), seq, key);
}

// Stage 2: 28-byte LPM command carrying the target position / action
static void velux_build_28b_frame(const uint8_t addr[3], const uint8_t key[16],
                                  uint16_t seq, uint8_t main0, uint8_t out[28])
{
  const uint8_t appli[7] = { 0x00, 0x01, 0x61, main0, 0x00, 0x00, 0x00 };
  build_header(out, 0xf6, 0x00, 0x3f, addr);
  finish_frame(out, appli, sizeof(appli), seq, key);
}

// Send open (0x00) / close (0xC8) / stop (0xD2) / position (0x01..0xC7).
// One button press = Stage 1 (seq N) + Stage 2 (seq N+1); every channel
// repetition reuses the same pair, otherwise a moving motor will STOP.
static void velux_exec_command(int dev, uint8_t main0, const char *action_name)
{
  bool valid = (dev == 0) ? (g_velux_nvm.blind_valid != 0) : (g_velux_nvm.sky_valid != 0);
  if (!valid) {
    printf("{\"event\":\"velux_error\",\"device\":\"%s\",\"error\":\"Device not paired yet. "
           "Run 'velux pair %s' and press the rear PAIR button on the remote.\"}\n",
           dev_name(dev), dev_name(dev));
    return;
  }

  const uint8_t *addr = (dev == 0) ? g_velux_nvm.blind_addr : g_velux_nvm.sky_addr;
  const uint8_t *key  = (dev == 0) ? g_velux_nvm.blind_key  : g_velux_nvm.sky_key;
  uint16_t *seq_p     = (dev == 0) ? &g_velux_nvm.blind_seq : &g_velux_nvm.sky_seq;
  uint8_t  *pos_p     = (dev == 0) ? &g_velux_nvm.blind_pos : &g_velux_nvm.sky_pos;
  uint16_t seq_n      = *seq_p;

  // Persist new counter + position before transmitting
  if (main0 <= 0xC8) {
    *pos_p = main0_to_ha_pct(main0);
  }
  *seq_p = (uint16_t)(seq_n + 2U);
  velux_nvm_save();

  uint8_t f30[30], f28[28];
  velux_build_30b_frame(dev, addr, key, seq_n, f30);
  velux_build_28b_frame(addr, key, (uint16_t)(seq_n + 1U), main0, f28);

  velux_transmit(f30, sizeof(f30), false, VELUX_DEFAULT_BURSTS);
  velux_transmit(f28, sizeof(f28), true, VELUX_DEFAULT_BURSTS);

  velux_print_device_json("velux_state", dev, action_name);
}

static void velux_start_pair_capture(int dev)
{
  if (!g_velux_transfer_key_valid) {
    velux_print_error("1-Way transfer key not set. Run 'velux set_transfer_key <32-hex>' first (or use 'velux setkey').");
    return;
  }

  g_velux_pair_mode = true;
  g_velux_pair_target = dev;

  RAIL_Idle(railHandle, RAIL_IDLE_ABORT, true);
  railDataConfig.txMethod = PACKET_MODE;
  railDataConfig.rxMethod = FIFO_MODE;
  RAIL_ConfigData(railHandle, &railDataConfig);
  rxFifoManual = true;
  channel = 0;

  restore_rx_fcd();
  RAIL_SetFixedLength(railHandle, 4095);   // infinite-length raw stream
  RAIL_ResetFifo(railHandle, true, true);
  g_burst_write_idx = 0;
  g_burst_in_progress = false;
  g_burst_overflow = false;
  fifo_rx_rearm();

  printf("{\"event\":\"velux_pairing_armed\",\"device\":\"%s\","
         "\"info\":\"Press the rear PAIR/RESET button on the remote now...\"}\n", dev_name(dev));
}

static void velux_stop_pair_capture(void)
{
  g_velux_pair_mode = false;
  RAIL_Idle(railHandle, RAIL_IDLE_ABORT, true);
  railDataConfig.txMethod = PACKET_MODE;
  railDataConfig.rxMethod = PACKET_MODE;
  RAIL_ConfigData(railHandle, &railDataConfig);
  rxFifoManual = false;
  RAIL_SetFixedLength(railHandle, 0);
  RAIL_ResetFifo(railHandle, true, true);
}

// Handle a decoded, CRC-valid frame
static void velux_on_rx_valid_frame(const uint8_t *psdu, uint16_t len)
{
  if (len < 12 || psdu[0] != 0x01 || psdu[1] != 0x69 || psdu[2] != 0x6f) {
    return;
  }
  const uint8_t *src = &psdu[8];

  // 1. Key-transfer frame (34 bytes, cmd 0x30) from the rear PAIR button
  if (len == 34 && psdu[11] == 0x30) {
    if (!g_velux_transfer_key_valid) {
      velux_print_error("Captured key-transfer frame, but the 1-Way transfer key is not set. Run 'velux set_transfer_key <32-hex>' first.");
      return;
    }
    uint8_t master_key[16];
    iohc_decrypt_1w_key(src, &psdu[12], master_key);
    uint16_t rx_seq = (uint16_t)(((uint16_t)psdu[30] << 8) | psdu[31]);

    int dev = g_velux_pair_target;
    if (!g_velux_pair_mode) {
      dev = (memcmp(src, g_velux_nvm.blind_addr, 3) == 0) ? 0 : 1;
    }
    if (dev == 0) {
      memcpy(g_velux_nvm.blind_addr, src, 3);
      memcpy(g_velux_nvm.blind_key, master_key, 16);
      g_velux_nvm.blind_seq = (uint16_t)(rx_seq + 1U);
      g_velux_nvm.blind_valid = 1;
    } else {
      memcpy(g_velux_nvm.sky_addr, src, 3);
      memcpy(g_velux_nvm.sky_key, master_key, 16);
      g_velux_nvm.sky_seq = (uint16_t)(rx_seq + 1U);
      g_velux_nvm.sky_valid = 1;
    }
    velux_nvm_save();

    if (g_velux_pair_mode) {
      velux_stop_pair_capture();
    }
    velux_print_device_json("velux_paired", dev, "paired");
    return;
  }

  // 2. Command from a paired wall switch: follow its rolling counter/position
  int dev = -1;
  if (g_velux_nvm.blind_valid && memcmp(src, g_velux_nvm.blind_addr, 3) == 0) {
    dev = 0;
  } else if (g_velux_nvm.sky_valid && memcmp(src, g_velux_nvm.sky_addr, 3) == 0) {
    dev = 1;
  }
  if (dev < 0 || (len != 28 && len != 30)) {
    return;
  }

  uint16_t rx_seq = (len == 28)
                    ? (uint16_t)(((uint16_t)psdu[18] << 8) | psdu[19])
                    : (uint16_t)(((uint16_t)psdu[20] << 8) | psdu[21]);
  const char *act = "sync";
  if (len == 28 && psdu[13] == 0x61) {
    uint8_t main0 = psdu[14];
    act = (main0 == 0x00) ? "open" : (main0 == 0xC8) ? "close" : (main0 == 0xD2) ? "stop" : "pos";
    if (main0 <= 0xC8) {
      if (dev == 0) {
        g_velux_nvm.blind_pos = main0_to_ha_pct(main0);
      } else {
        g_velux_nvm.sky_pos = main0_to_ha_pct(main0);
      }
    }
  }
  if (dev == 0) {
    g_velux_nvm.blind_seq = (uint16_t)(rx_seq + 1U);
  } else {
    g_velux_nvm.sky_seq = (uint16_t)(rx_seq + 1U);
  }
  velux_nvm_save();
  velux_print_device_json("velux_remote", dev, act);
}

static int parse_hex_bytes(const char *hex_str, uint8_t *out, int max_bytes)
{
  int count = 0;
  while (*hex_str && count < max_bytes) {
    while (*hex_str == ' ' || *hex_str == ':') {
      hex_str++;
    }
    if (!hex_str[0] || !hex_str[1]) {
      break;
    }
    char buf[3] = { hex_str[0], hex_str[1], 0 };
    out[count++] = (uint8_t)strtoul(buf, NULL, 16);
    hex_str += 2;
  }
  return count;
}

static void cli_print_status_all(void)
{
  velux_print_device_json("velux_status", 0, "status");
  velux_print_device_json("velux_status", 1, "status");
}

/*
 * velux [status]
 * velux set_transfer_key <32-hex>
 * velux pair <blind|skylight>      velux stop_pair
 * velux <blind|skylight> <open|close|stop|vent|pos N|status|pair>
 * velux setkey <blind|skylight> <32-hex-key> [6-hex-addr]
 * velux setseq <blind|skylight> <seq>
 * velux reset_nvm
 */
void cli_velux(sl_cli_command_arg_t *args)
{
  int argc = sl_cli_get_argument_count(args);
  const char *a0 = (argc > 0) ? sl_cli_get_argument_string(args, 0) : "status";
  const char *a1 = (argc > 1) ? sl_cli_get_argument_string(args, 1) : NULL;
  const char *a2 = (argc > 2) ? sl_cli_get_argument_string(args, 2) : NULL;
  const char *a3 = (argc > 3) ? sl_cli_get_argument_string(args, 3) : NULL;
  int dev = 1;

  if (strcmp(a0, "status") == 0 || strcmp(a0, "info") == 0) {
    cli_print_status_all();
    return;
  }

  if (strcmp(a0, "set_transfer_key") == 0) {
    uint8_t tk[16];
    if (a1 == NULL || parse_hex_bytes(a1, tk, 16) != 16) {
      velux_print_error("Expected 32 hex chars for 16-byte transfer key");
      return;
    }
    memcpy(g_velux_transfer_key, tk, 16);
    g_velux_transfer_key_valid = true;
    const VeluxTkSlot_t *tk_flash = (const VeluxTkSlot_t *)VELUX_NVM_TK_SLOT_ADDR;
    if (tk_flash->magic != 0xFFFFFFFFUL) {
      // Slot already programmed: force a page rewrite (state + new key)
      g_velux_nvm_next_slot = VELUX_NVM_SLOT_COUNT;
      velux_nvm_save();
    } else {
      velux_tk_write_if_needed();
    }
    printf("{\"event\":\"velux_transfer_key\",\"transfer_key_set\":true}\n");
    return;
  }

  if (strcmp(a0, "reset_nvm") == 0) {
    velux_flash_erase_page(VELUX_NVM_FLASH_PAGE_ADDR);
    memset(&g_velux_nvm, 0, sizeof(VeluxNvmSlot_t));
    memset(g_velux_transfer_key, 0, sizeof(g_velux_transfer_key));
    g_velux_transfer_key_valid = false;
    g_velux_nvm.blind_seq = 1;
    g_velux_nvm.sky_seq = 1;
    g_velux_nvm_next_slot = 0;
    velux_nvm_save();
    printf("{\"event\":\"velux_nvm_reset\",\"status\":\"ok\"}\n");
    return;
  }

  if (strcmp(a0, "pair") == 0) {
    if (a1 != NULL && !parse_device(a1, &dev)) {
      velux_print_error("Usage: velux pair <blind|skylight>");
      return;
    }
    velux_start_pair_capture(dev);
    return;
  }

  if (strcmp(a0, "stop_pair") == 0) {
    velux_stop_pair_capture();
    printf("{\"event\":\"velux_pairing_stopped\"}\n");
    return;
  }

  if (strcmp(a0, "setseq") == 0) {
    if (!parse_device(a1, &dev) || a2 == NULL) {
      velux_print_error("Usage: velux setseq <blind|skylight> <seq>");
      return;
    }
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

  if (strcmp(a0, "setkey") == 0) {
    uint8_t key[16];
    if (!parse_device(a1, &dev) || a2 == NULL || parse_hex_bytes(a2, key, 16) != 16) {
      velux_print_error("Usage: velux setkey <blind|skylight> <32-hex-key> [6-hex-addr]");
      return;
    }
    if (dev == 0) {
      memcpy(g_velux_nvm.blind_key, key, 16);
      g_velux_nvm.blind_valid = 1;
      if (a3) {
        parse_hex_bytes(a3, g_velux_nvm.blind_addr, 3);
      }
    } else {
      memcpy(g_velux_nvm.sky_key, key, 16);
      g_velux_nvm.sky_valid = 1;
      if (a3) {
        parse_hex_bytes(a3, g_velux_nvm.sky_addr, 3);
      }
    }
    velux_nvm_save();
    velux_print_device_json("velux_status", dev, "setkey");
    return;
  }

  // Device commands: "velux <device> <action> [value]" or "velux <action> <device> [value]"
  const char *cmd;
  const char *val;
  if (parse_device(a0, &dev)) {
    cmd = a1 ? a1 : "status";
    val = a2;
  } else if (parse_device(a1, &dev)) {
    cmd = a0;
    val = a2;
  } else {
    velux_print_error("Missing device. Use: velux <blind|skylight> <open|close|stop|vent|pos N|status|pair>");
    return;
  }

  if (strcmp(cmd, "status") == 0) {
    velux_print_device_json("velux_status", dev, "status");
  } else if (strcmp(cmd, "open") == 0 || strcmp(cmd, "up") == 0) {
    velux_exec_command(dev, 0x00, "open");
  } else if (strcmp(cmd, "close") == 0 || strcmp(cmd, "down") == 0) {
    velux_exec_command(dev, 0xC8, "close");
  } else if (strcmp(cmd, "stop") == 0 || strcmp(cmd, "my") == 0) {
    velux_exec_command(dev, 0xD2, "stop");
  } else if (strcmp(cmd, "vent") == 0 || strcmp(cmd, "half") == 0) {
    velux_exec_command(dev, 0x64, "pos");
  } else if (strcmp(cmd, "pos") == 0 || strcmp(cmd, "position") == 0) {
    uint32_t ha_pct = val ? strtoul(val, NULL, 10) : 50;
    if (ha_pct > 100) {
      ha_pct = 100;
    }
    velux_exec_command(dev, (uint8_t)(200U - ha_pct * 2U), "pos");
  } else if (strcmp(cmd, "pair") == 0) {
    velux_start_pair_capture(dev);
  } else {
    velux_print_error("Unknown command. Use open|close|stop|vent|pos|status|pair|setseq|setkey|set_transfer_key|reset_nvm|stop_pair");
  }
}

// =============================================================================
// 7. Application entry points
// =============================================================================
void app_init(void)
{
  // XIAO MG24 RF switch: PB05=1 powers the switch, PB04=0 selects the
  // onboard ceramic antenna (1 = U.FL)
  sl_gpio_t rf_sw_pw  = { .port = SL_GPIO_PORT_B, .pin = 5 };
  sl_gpio_t rf_sw_sel = { .port = SL_GPIO_PORT_B, .pin = 4 };
  sl_gpio_set_pin_mode(&rf_sw_pw, SL_GPIO_MODE_PUSH_PULL, 1);
  sl_gpio_set_pin_mode(&rf_sw_sel, SL_GPIO_MODE_PUSH_PULL, 0);

  // Hold received packets for our own drain loop and keep RAILtest's async
  // printing off so the UART never stalls RX
  rxHeld = true;
  rxProcessHeld = false;
  logLevel &= ~ASYNC_RESPONSE;

  velux_nvm_init();

  velux_print_device_json("velux_boot", 0, "boot");
  velux_print_device_json("velux_boot", 1, "boot");
}

void app_process_action(void)
{
  // Prevent RAILtest's heldRxProcess() from consuming our packets
  rxProcessHeld = false;

  // 1. Packet mode: drain held packets into the RAM burst buffer
  while (packetsHeld > 0) {
    RAIL_RxPacketInfo_t info;
    RAIL_RxPacketHandle_t handle =
      RAIL_GetRxPacketInfo(railHandle, RAIL_RX_PACKET_HANDLE_OLDEST_COMPLETE, &info);
    if (handle == RAIL_RX_PACKET_HANDLE_INVALID) {
      break;
    }

    RAIL_RxPacketDetails_t details;
    bool have_details = (RAIL_GetRxPacketDetailsAlt(railHandle, handle, &details) == RAIL_STATUS_NO_ERROR);
    uint16_t pkt_len = info.packetBytes;
    uint32_t record_len = sizeof(BurstPacketHeader_t) + pkt_len;

    if (g_burst_write_idx + record_len <= BURST_BUFFER_SIZE) {
      BurstPacketHeader_t *hdr = (BurstPacketHeader_t *)&g_burst_buffer[g_burst_write_idx];
      hdr->timestamp_us  = have_details ? details.timeReceived.packetTime : RAIL_GetTime();
      hdr->rssi          = have_details ? details.rssi : 0;
      hdr->lqi           = have_details ? details.lqi : 0;
      hdr->length        = pkt_len;
      hdr->packet_status = (uint8_t)info.packetStatus;
      hdr->sub_phy_id    = have_details ? details.subPhyId : 0;
      hdr->freq_offset   = (int16_t)rxFreqOffset;
      hdr->channel       = channel;
      RAIL_CopyRxPacket(&g_burst_buffer[g_burst_write_idx + sizeof(BurstPacketHeader_t)], &info);
      g_burst_write_idx += record_len;
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

  // 2. FIFO stream mode (pairing): read raw demodulator bytes
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
        g_burst_write_idx += RAIL_ReadRxFifo(railHandle, &g_burst_buffer[g_burst_write_idx], to_read);
        g_last_rx_time = RAIL_GetTime();
        g_burst_in_progress = true;
      }
      if (avail > space) {
        g_burst_overflow = true;
      }
    }

    // After ~176 ms (Stage 1 captured) re-arm RX with SYNC0=0x56 so the
    // correlator locks freshly onto Stage 2
    if (g_burst_in_progress && !s_resynced_stage2 && g_burst_write_idx >= FIFO_STAGE2_RESYNC_BYTES) {
      s_resynced_stage2 = true;
      RAIL_Idle(railHandle, RAIL_IDLE_ABORT, true);
      RAIL_ResetFifo(railHandle, false, true);
      RAIL_StartRx(railHandle, channel, NULL);
      REG_MODEM_CTRL1  = 0x0052C007;
      REG_MODEM_SYNC0  = 0x56;
      REG_MODEM_TIMING = 0x08A01145;
      g_last_rx_time = RAIL_GetTime();
    }

    uint32_t timeout_us = s_resynced_stage2 ? 500000UL : BURST_IDLE_TIMEOUT_US;
    if (g_burst_in_progress
        && ((g_burst_write_idx >= FIFO_STREAM_CAPTURE_BYTES)
            || (RAIL_GetTime() - g_last_rx_time > timeout_us))) {
      RAIL_Idle(railHandle, RAIL_IDLE_ABORT, true);
      RAIL_ResetFifo(railHandle, false, true);

      dump_fifo_stream_over_serial();

      g_burst_write_idx = 0;
      g_burst_in_progress = false;
      g_burst_overflow = false;
      s_resynced_stage2 = false;
      fifo_rx_rearm();
    }
    return;
  }

  // 3. Packet mode: dump the burst after 120 ms of silence
  if (g_burst_in_progress && (RAIL_GetTime() - g_last_rx_time > BURST_IDLE_TIMEOUT_US)) {
    g_burst_in_progress = false;
    dump_burst_over_serial();
    g_burst_write_idx = 0;
    g_burst_packet_count = 0;
    g_burst_overflow = false;
  }
}
