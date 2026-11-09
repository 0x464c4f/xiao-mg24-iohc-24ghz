/*
 * Fake platform for host tests: RAILtest globals, registers, NOR flash,
 * virtual clock, TX recorder, RX FIFO feed and packet queue.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "fake_platform.h"

RAIL_Handle_t railHandle = (RAIL_Handle_t)1;
uint16_t channel;
bool rxHeld, rxProcessHeld, rxFifoManual;
volatile uint32_t packetsHeld;
uint32_t logLevel = 0xFFFFFFFFU;
int32_t rxFreqOffset;
RAIL_DataConfig_t railDataConfig;
volatile uint32_t fake_reg_fcd[4];
volatile uint32_t fake_reg_modem_ctrl1, fake_reg_sync0, fake_reg_sync1, fake_reg_timing;

uint8_t fake_flash[FAKE_FLASH_PAGE_SIZE] __attribute__((aligned(64)));
unsigned fake_flash_erase_count;

uint32_t g_now;
TxRec g_tx[FAKE_TX_MAX];
int g_tx_n;
uint8_t g_feed[FAKE_FEED_MAX];
size_t g_feed_len, g_feed_pos;
uint8_t g_pkt[256];
uint16_t g_pkt_len;
bool g_pkt_pending;

static uint8_t g_txfifo[64];
static uint16_t g_txfifo_len;
static int g_tx_state_calls;

void fake_reset(void)
{
  memset(fake_flash, 0xFF, sizeof(fake_flash));
  fake_flash_erase_count = 0;
  g_tx_n = 0;
  g_feed_len = g_feed_pos = 0;
  g_pkt_pending = false;
  packetsHeld = 0;
  railDataConfig.txMethod = PACKET_MODE;
  railDataConfig.rxMethod = PACKET_MODE;
  rxFifoManual = false;
}

// ---- flash (NOR semantics: erase -> 0xFF, programming only clears bits)
void velux_flash_erase_page(uintptr_t page_addr)
{
  memset((void *)page_addr, 0xFF, FAKE_FLASH_PAGE_SIZE);
  fake_flash_erase_count++;
}

void velux_flash_write_64b(uintptr_t slot_addr, const void *slot64)
{
  uint8_t *dst = (uint8_t *)slot_addr;
  const uint8_t *src = (const uint8_t *)slot64;
  if (slot_addr < (uintptr_t)fake_flash
      || slot_addr + 64 > (uintptr_t)fake_flash + FAKE_FLASH_PAGE_SIZE) {
    fprintf(stderr, "flash write out of range\n");
    abort();
  }
  for (int i = 0; i < 64; i++) {
    dst[i] &= src[i];
  }
}

// ---- misc SDK
int sl_gpio_set_pin_mode(const sl_gpio_t *g, sl_gpio_mode_t m, bool o)
{
  (void)g; (void)m; (void)o;
  return 0;
}
int sl_cli_get_argument_count(sl_cli_command_arg_t *a) { return a->argc; }
const char *sl_cli_get_argument_string(sl_cli_command_arg_t *a, int i) { return a->argv[i]; }

// ---- clock: each call advances 3 us
uint32_t RAIL_GetTime(void) { g_now += 3; return g_now; }

// ---- radio control (no-ops)
RAIL_Status_t RAIL_Idle(RAIL_Handle_t h, RAIL_IdleMode_t m, bool w) { (void)h; (void)m; (void)w; return 0; }
uint16_t RAIL_SetFixedLength(RAIL_Handle_t h, uint16_t l) { (void)h; return l; }
void RAIL_ResetFifo(RAIL_Handle_t h, bool t, bool r) { (void)h; (void)t; (void)r; }
RAIL_Status_t RAIL_ConfigData(RAIL_Handle_t h, const RAIL_DataConfig_t *c) { (void)h; (void)c; return 0; }
RAIL_Status_t RAIL_ConfigSyncWords(RAIL_Handle_t h, const RAIL_SyncWordConfig_t *c) { (void)h; (void)c; return 0; }
RAIL_Status_t RAIL_SetTxPowerDbm(RAIL_Handle_t h, int16_t p) { (void)h; (void)p; return 0; }
RAIL_Status_t RAIL_SetTxTransitions(RAIL_Handle_t h, const RAIL_StateTransitions_t *t) { (void)h; (void)t; return 0; }
RAIL_Status_t RAIL_StartRx(RAIL_Handle_t h, uint16_t c, const void *s) { (void)h; (void)c; (void)s; return 0; }

// ---- TX recorder
uint16_t RAIL_WriteTxFifo(RAIL_Handle_t h, const uint8_t *d, uint16_t len, bool reset)
{
  (void)h; (void)reset;
  memcpy(g_txfifo, d, len);
  g_txfifo_len = len;
  return len;
}

RAIL_Status_t RAIL_StartTx(RAIL_Handle_t h, uint16_t ch, uint32_t opts, const void *s)
{
  (void)h; (void)s;
  if (!(opts & RAIL_TX_OPTION_REMOVE_CRC) || g_tx_n >= FAKE_TX_MAX) {
    abort();
  }
  TxRec *r = &g_tx[g_tx_n++];
  r->ch = ch;
  r->t = g_now;
  r->len = g_txfifo_len;
  memcpy(r->data, g_txfifo, g_txfifo_len);
  r->sync0 = fake_reg_sync0;
  r->fcd0 = fake_reg_fcd[0];
  g_tx_state_calls = 2;   // report "TX" for the next two state queries
  return 0;
}

RAIL_RadioState_t RAIL_GetRadioState(RAIL_Handle_t h)
{
  (void)h;
  if (g_tx_state_calls > 0) {
    g_tx_state_calls--;
    return RAIL_RF_STATE_TX;
  }
  return RAIL_RF_STATE_IDLE;
}

// ---- RX FIFO feed
uint16_t RAIL_GetRxFifoBytesAvailable(RAIL_Handle_t h)
{
  (void)h;
  size_t rem = g_feed_len - g_feed_pos;
  return (uint16_t)(rem > 256 ? 256 : rem);
}

uint16_t RAIL_ReadRxFifo(RAIL_Handle_t h, uint8_t *dst, uint16_t len)
{
  (void)h;
  memcpy(dst, &g_feed[g_feed_pos], len);
  g_feed_pos += len;
  return len;
}

// ---- packet-mode queue (single packet)
RAIL_RxPacketHandle_t RAIL_GetRxPacketInfo(RAIL_Handle_t h, RAIL_RxPacketHandle_t w, RAIL_RxPacketInfo_t *info)
{
  (void)h; (void)w;
  if (!g_pkt_pending) {
    return RAIL_RX_PACKET_HANDLE_INVALID;
  }
  info->packetBytes = g_pkt_len;
  info->packetStatus = 0;
  return (RAIL_RxPacketHandle_t)7;
}

RAIL_Status_t RAIL_GetRxPacketDetailsAlt(RAIL_Handle_t h, RAIL_RxPacketHandle_t p, RAIL_RxPacketDetails_t *d)
{
  (void)h; (void)p;
  d->timeReceived.packetTime = g_now;
  d->rssi = -60;
  d->lqi = 200;
  d->subPhyId = 0;
  return RAIL_STATUS_NO_ERROR;
}

void RAIL_CopyRxPacket(uint8_t *dst, const RAIL_RxPacketInfo_t *info)
{
  memcpy(dst, g_pkt, info->packetBytes);
}

RAIL_Status_t RAIL_ReleaseRxPacket(RAIL_Handle_t h, RAIL_RxPacketHandle_t p)
{
  (void)h; (void)p;
  g_pkt_pending = false;
  return 0;
}
