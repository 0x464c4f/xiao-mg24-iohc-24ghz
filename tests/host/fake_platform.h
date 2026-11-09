/* Fake platform interface for host tests. SPDX-License-Identifier: Apache-2.0 */
#ifndef FAKE_PLATFORM_H
#define FAKE_PLATFORM_H
#include "velux_host_stubs.h"

#define FAKE_TX_MAX   4096
#define FAKE_FEED_MAX 16384

typedef struct {
  uint16_t ch;
  uint32_t t;
  uint16_t len;
  uint8_t  data[40];
  uint32_t sync0;   // MODEM SYNC0 at StartTx
  uint32_t fcd0;    // FRC FCD0 at StartTx
} TxRec;

extern uint32_t g_now;
extern TxRec g_tx[FAKE_TX_MAX];
extern int g_tx_n;
extern uint8_t g_feed[FAKE_FEED_MAX];
extern size_t g_feed_len, g_feed_pos;
extern uint8_t g_pkt[256];
extern uint16_t g_pkt_len;
extern bool g_pkt_pending;

void fake_reset(void);
#endif
