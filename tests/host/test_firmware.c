/*
 * Host unit tests for firmware/app.c
 *
 * Builds the real firmware source against a fake radio / flash / CLI layer
 * (stubs/velux_host_stubs.h). All keys and addresses are synthetic.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <stdlib.h>

#include "velux_host_stubs.h"

// Capture firmware output
static char g_out[1 << 20];
static size_t g_out_len;
static int test_printf(const char *fmt, ...)
{
  va_list ap;
  va_start(ap, fmt);
  int n = vsnprintf(g_out + g_out_len, sizeof(g_out) - g_out_len, fmt, ap);
  va_end(ap);
  if (n > 0) {
    g_out_len += (size_t)n;
    if (g_out_len >= sizeof(g_out)) {
      g_out_len = sizeof(g_out) - 1;
    }
  }
  return n;
}
#define printf test_printf
#include "app.c"
#undef printf

#include "fake_platform.h"

// =============================================================================
// Mini test framework
// =============================================================================
static int g_checks, g_failures;
static const char *g_test = "";

#define CHECK(cond) do { g_checks++; if (!(cond)) { g_failures++; \
  fprintf(stderr, "  FAIL [%s] %s:%d: %s\n", g_test, __FILE__, __LINE__, #cond); } } while (0)
#define CHECK_EQ(a, b) do { long long _a = (long long)(a), _b = (long long)(b); g_checks++; \
  if (_a != _b) { g_failures++; fprintf(stderr, "  FAIL [%s] %s:%d: %s == %s (0x%llx != 0x%llx)\n", \
  g_test, __FILE__, __LINE__, #a, #b, _a, _b); } } while (0)
#define CHECK_MEM(a, b, n) do { g_checks++; if (memcmp((a), (b), (n)) != 0) { g_failures++; \
  fprintf(stderr, "  FAIL [%s] %s:%d: memcmp(%s, %s)\n", g_test, __FILE__, __LINE__, #a, #b); } } while (0)
#define CHECK_OUT(substr) do { g_checks++; if (strstr(g_out, (substr)) == NULL) { g_failures++; \
  fprintf(stderr, "  FAIL [%s] %s:%d: output lacks \"%s\"\n", g_test, __FILE__, __LINE__, (substr)); } } while (0)
#define CHECK_NOT_OUT(substr) do { g_checks++; if (strstr(g_out, (substr)) != NULL) { g_failures++; \
  fprintf(stderr, "  FAIL [%s] %s:%d: output unexpectedly has \"%s\"\n", g_test, __FILE__, __LINE__, (substr)); } } while (0)

static void out_clear(void)
{
  g_out_len = 0;
  g_out[0] = 0;
}

// =============================================================================
// Helpers
// =============================================================================
static const uint8_t TEST_KEY[16]  = { 0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77,
                                       0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff };
static const uint8_t TEST_ADDR[3]  = { 0xaa, 0xbb, 0xcc };
static const uint8_t TEST_TK[16]   = { 0x0f, 0x1e, 0x2d, 0x3c, 0x4b, 0x5a, 0x69, 0x78,
                                       0x87, 0x96, 0xa5, 0xb4, 0xc3, 0xd2, 0xe1, 0xf0 };
static const char   *TEST_TK_HEX   = "0f1e2d3c4b5a69788796a5b4c3d2e1f0";

static void hex2bin(const char *hex, uint8_t *out, size_t n)
{
  for (size_t i = 0; i < n; i++) {
    unsigned v;
    sscanf(hex + 2 * i, "%2x", &v);
    out[i] = (uint8_t)v;
  }
}

// Run "velux <args...>"
static void cli(int argc, ...)
{
  const char *argv[8];
  va_list ap;
  va_start(ap, argc);
  for (int i = 0; i < argc; i++) {
    argv[i] = va_arg(ap, const char *);
  }
  va_end(ap);
  sl_cli_command_arg_t a = { argc, argv };
  cli_velux(&a);
}

// Simulate a reboot: forget RAM state, reload from (fake) flash
static void reboot(void)
{
  memset(&g_velux_nvm, 0, sizeof(g_velux_nvm));
  memset(g_velux_transfer_key, 0, sizeof(g_velux_transfer_key));
  g_velux_transfer_key_valid = false;
  g_velux_nvm_next_slot = 0;
  g_velux_pair_mode = false;
  app_init();
}

static void fresh_device(void)
{
  fake_reset();
  reboot();
  out_clear();
}

// Spin the main loop for `us` microseconds of virtual time
static void run_loop(uint32_t us)
{
  uint32_t end = g_now + us;
  while ((int32_t)(end - g_now) > 0) {
    app_process_action();
    g_now += 50;
  }
}

static void append_crc(uint8_t *psdu, uint16_t len_with_crc)
{
  uint16_t crc = kermit_crc16(psdu, (uint16_t)(len_with_crc - 2));
  psdu[len_with_crc - 2] = (uint8_t)(crc & 0xFF);
  psdu[len_with_crc - 1] = (uint8_t)(crc >> 8);
}

// ---- raw demodulator stream encoder (inverse of inv_nibble for phase P)
static uint8_t enc_nibble(uint8_t m, uint8_t P)
{
  for (uint8_t n = 0; n < 16; n++) {
    if (inv_nibble(n, P) == m) {
      return n;
    }
  }
  abort();
}

static uint8_t g_nibs[FAKE_FEED_MAX * 2];
static size_t g_nib_n;

static void nib_push(uint8_t n) { g_nibs[g_nib_n++] = (uint8_t)(n & 0xF); }

// noise + preamble + SFD + PHR + PSDU, as seen by a receiver locked at phase P
static void stream_add_frame(const uint8_t *psdu, uint8_t len, uint8_t P, unsigned noise_nibs)
{
  for (unsigned i = 0; i < noise_nibs; i++) {
    nib_push((uint8_t)((P + 5 + i * 7) & 0xF) == P ? (uint8_t)(P ^ 1) : (uint8_t)((P + 5 + i * 7) & 0xF));
  }
  for (int i = 0; i < 8; i++) {
    nib_push(P);                       // preamble symbol 0 -> raw nibble P
  }
  nib_push(enc_nibble(6, P));          // SFD 0x56, low nibble first
  nib_push(enc_nibble(5, P));
  uint8_t phr = len;
  nib_push(enc_nibble(phr & 0xF, P));
  nib_push(enc_nibble(phr >> 4, P));
  for (uint8_t i = 0; i < len; i++) {
    nib_push(enc_nibble(psdu[i] & 0xF, P));
    nib_push(enc_nibble(psdu[i] >> 4, P));
  }
}

static void stream_commit(void)
{
  for (int i = 0; i < 64; i++) {
    nib_push((uint8_t)(i & 0xF));      // trailing noise
  }
  if (g_nib_n & 1) {
    nib_push(0x3);
  }
  for (size_t i = 0; i < g_nib_n / 2; i++) {
    g_feed[i] = (uint8_t)(g_nibs[2 * i] | (g_nibs[2 * i + 1] << 4));
  }
  g_feed_len = g_nib_n / 2;
  g_feed_pos = 0;
  g_nib_n = 0;
}

// 34-byte key-transfer frame as broadcast by the rear PAIR button
static void make_key_transfer_frame(uint8_t out[34], const uint8_t addr[3],
                                    const uint8_t master_key[16], uint16_t seq)
{
  uint8_t iv[16], ks[16];
  for (int i = 0; i < 15; i += 3) {
    memcpy(&iv[i], addr, 3);
  }
  iv[15] = addr[0];
  aes128_ecb_encrypt(TEST_TK, iv, ks);
  const uint8_t hdr[8] = { 0x01, 0x69, 0x6f, 0xfc, 0x00, 0x00, 0x01, 0x3f };
  memcpy(out, hdr, 8);
  memcpy(&out[8], addr, 3);
  out[11] = 0x30;
  for (int i = 0; i < 16; i++) {
    out[12 + i] = (uint8_t)(master_key[i] ^ ks[i]);
  }
  out[28] = 0x01;
  out[29] = 0x01;
  out[30] = (uint8_t)(seq >> 8);
  out[31] = (uint8_t)(seq & 0xFF);
  append_crc(out, 34);
}

// =============================================================================
// Tests: checksum
// =============================================================================
static void test_crc(void)
{
  g_test = "crc16";
  CHECK_EQ(kermit_crc16((const uint8_t *)"123456789", 9), 0x2189);
  uint8_t buf[11] = "123456789";
  append_crc(buf, 11);
  CHECK_EQ(kermit_crc16(buf, 11), 0);
}

static void test_nibble_map(void)
{
  g_test = "nibble_map";
  for (uint8_t P = 0; P < 16; P++) {
    uint16_t seen = 0;
    for (uint8_t n = 0; n < 16; n++) {
      seen |= (uint16_t)(1U << inv_nibble(n, P));
    }
    CHECK_EQ(seen, 0xFFFF);              // bijective for every phase
    CHECK_EQ(inv_nibble(P, P), 0);       // preamble nibble decodes to symbol 0
  }
  for (uint8_t n = 0; n < 16; n++) {
    CHECK_EQ(inv_nibble(n, 0), n);       // phase 0 is the identity
  }
}

// =============================================================================
// Tests: frame builders vs. frames from the hardware-verified firmware
// =============================================================================
static void test_frames_golden(void)
{
  g_test = "frames_golden";
  uint8_t exp[30], f30[30], f28[28];

  velux_build_30b_frame(0, TEST_ADDR, TEST_KEY, 0x1234, f30);
  hex2bin("01696ff82000007faabbcc000161d40080c8000012347e42e89e5d7d37a0", exp, 30);
  CHECK_MEM(f30, exp, 30);

  velux_build_30b_frame(1, TEST_ADDR, TEST_KEY, 0x0241, f30);
  hex2bin("01696ff82000013faabbcc000161d40080c800000241588419fb594e4dde", exp, 30);
  CHECK_MEM(f30, exp, 30);

  static const struct { uint8_t main0; const char *hex; } v28[] = {
    { 0x00, "01696ff62000003faabbcc000161000000001235882fd7ed2f5f0528" },
    { 0xC8, "01696ff62000003faabbcc000161c800000012356b9beee3e2b7ad92" },
    { 0xD2, "01696ff62000003faabbcc000161d200000012355558e9c1d4084769" },
    { 0x64, "01696ff62000003faabbcc000161640000001235efb7f6735758c7ee" },
    { 0x8C, "01696ff62000003faabbcc0001618c0000001235fcc6c0313a908e4d" },
  };
  for (size_t i = 0; i < sizeof(v28) / sizeof(v28[0]); i++) {
    velux_build_28b_frame(TEST_ADDR, TEST_KEY, 0x1235, v28[i].main0, f28);
    hex2bin(v28[i].hex, exp, 28);
    CHECK_MEM(f28, exp, 28);
    CHECK_EQ(kermit_crc16(f28, 28), 0);
  }
}

// =============================================================================
// Tests: transmit schedule
// =============================================================================
typedef struct { uint32_t tick; int cnt; int window; } Slot;

// Expected slot list for one sub-burst (cnt >= 0: beacon, window >= 0: command)
static int expected_slots(bool lpm, Slot *s)
{
  int n = 0;
  uint32_t first_window;
  if (lpm) {
    for (int t = 0; t <= 106; t++) { s[n++] = (Slot){ (uint32_t)t, 106 - t, -1 }; }
    first_window = 107;
  } else {
    for (int t = 0; t <= 7; t++) { s[n++] = (Slot){ (uint32_t)t, 7 - t, -1 }; }
    first_window = 8;
  }
  for (int w = 0; w < 4; w++) {
    uint32_t wt = first_window + 13U * (uint32_t)w;
    s[n++] = (Slot){ wt, -1, w };
    if (w < 3) {
      for (int k = 0; k <= 7; k++) { s[n++] = (Slot){ wt + 5U + (uint32_t)k, 7 - k, -1 }; }
    }
  }
  return n;
}

// Verify one sub-burst starting at g_tx[idx]; returns index after it
static int check_subburst(int idx, bool lpm, uint16_t ch, const uint8_t *psdu, uint8_t psdu_len)
{
  Slot s[160];
  int n = expected_slots(lpm, s);
  CHECK(idx + n <= g_tx_n);
  if (idx + n > g_tx_n) {
    return g_tx_n;
  }
  uint32_t t0 = g_tx[idx].t;
  for (int i = 0; i < n; i++) {
    const TxRec *r = &g_tx[idx + i];
    int32_t dt = (int32_t)(r->t - t0) - (int32_t)(s[i].tick * 1000U);
    CHECK(dt >= -20 && dt <= 20);                 // 1000 us grid
    CHECK_EQ(r->ch, ch);
    CHECK_EQ(r->sync0, 0x56);                     // on-air SFD
    CHECK_EQ(r->fcd0, 0x000040FF);                // hardware CRC disabled
    if (s[i].cnt >= 0) {
      uint8_t b[6] = { 0x05, 0x00, (uint8_t)(s[i].cnt >> 8), (uint8_t)s[i].cnt, 0, 0 };
      if (s[i].cnt != 0) {
        append_crc(&b[1], 5);
      }
      CHECK_EQ(r->len, 6);
      CHECK_MEM(r->data, b, 6);
    } else {
      uint8_t w[37];
      w[0] = psdu_len;
      memcpy(&w[1], psdu, psdu_len);
      w[1 + 4] = (lpm && s[i].window == 0) ? 0x20 : 0x00;   // CB2
      append_crc(&w[1], psdu_len);
      CHECK_EQ(r->len, psdu_len + 1);
      CHECK_MEM(r->data, w, (size_t)psdu_len + 1);
    }
  }
  return idx + n;
}

static void pair_blind_via_setkey(void)
{
  char keyhex[33];
  for (int i = 0; i < 16; i++) {
    snprintf(&keyhex[2 * i], 3, "%02x", TEST_KEY[i]);
  }
  cli(4, "setkey", "blind", keyhex, "aabbcc");
  cli(3, "setseq", "blind", "0x1234");
}

static void test_tx_close_blind(void)
{
  g_test = "tx_close_blind";
  fresh_device();
  pair_blind_via_setkey();
  g_tx_n = 0;
  out_clear();

  cli(2, "blind", "close");

  uint8_t s1[30], s2[28];
  velux_build_30b_frame(0, TEST_ADDR, TEST_KEY, 0x1234, s1);
  velux_build_28b_frame(TEST_ADDR, TEST_KEY, 0x1235, 0xC8, s2);

  // Stage 1: 3 non-LPM sub-bursts (36 frames each) on 2425/2450/2475 MHz
  // Stage 2: 3 LPM sub-bursts (135 frames each)
  CHECK_EQ(g_tx_n, 3 * 36 + 3 * 135);
  // Channel numbers on the 2425 MHz + 5 MHz grid: 2425 / 2450 / 2475 MHz
  static const uint16_t expected_ch[3] = { 0, 5, 10 };
  int idx = 0;
  for (int b = 0; b < 3; b++) {
    idx = check_subburst(idx, false, expected_ch[b], s1, 30);
  }
  for (int b = 0; b < 3; b++) {
    idx = check_subburst(idx, true, expected_ch[b], s2, 28);
  }

  CHECK_EQ(g_velux_nvm.blind_seq, 0x1236);   // N, N+1 used
  CHECK_EQ(g_velux_nvm.blind_pos, 0);        // closed
  CHECK_OUT("\"event\":\"velux_state\",\"device\":\"blind\"");
  CHECK_OUT("\"action\":\"close\"");
  // RX configuration restored afterwards
  CHECK_EQ(fake_reg_fcd[0], 0x00004000);
  CHECK_EQ(fake_reg_sync0, 0x6A);
  CHECK_EQ(fake_reg_sync1, 0x56);
}

static void test_tx_positions(void)
{
  g_test = "tx_positions";
  static const struct { const char *a1; const char *a2; uint8_t main0; uint8_t pos; } v[] = {
    { "open",  NULL,  0x00, 100 },
    { "stop",  NULL,  0xD2, 100 },   // stop keeps last position
    { "pos",   "30",  0x8C, 30  },
    { "pos",   "150", 0x00, 100 },   // clamped
    { "vent",  NULL,  0x64, 50  },
    { "pos",   "0",   0xC8, 0   },
  };
  fresh_device();
  pair_blind_via_setkey();
  for (size_t i = 0; i < sizeof(v) / sizeof(v[0]); i++) {
    uint16_t seq = g_velux_nvm.blind_seq;
    g_tx_n = 0;
    if (v[i].a2) {
      cli(3, "blind", v[i].a1, v[i].a2);
    } else {
      cli(2, "blind", v[i].a1);
    }
    // Last frame of the command is LPM window 3 of the last sub-burst
    const TxRec *last = &g_tx[g_tx_n - 1];
    CHECK_EQ(last->len, 29);
    CHECK_EQ(last->data[1 + 14], v[i].main0);
    CHECK_EQ(((last->data[1 + 18] << 8) | last->data[1 + 19]), (uint16_t)(seq + 1));
    CHECK_EQ(g_velux_nvm.blind_pos, v[i].pos);
    CHECK_EQ(g_velux_nvm.blind_seq, (uint16_t)(seq + 2));
  }
}

static void test_skylight_broadcast_class(void)
{
  g_test = "skylight_class";
  fresh_device();
  char keyhex[33];
  for (int i = 0; i < 16; i++) {
    snprintf(&keyhex[2 * i], 3, "%02x", TEST_KEY[i]);
  }
  cli(4, "setkey", "skylight", keyhex, "aabbcc");
  cli(3, "setseq", "skylight", "0x0241");
  g_tx_n = 0;
  cli(2, "skylight", "open");
  uint8_t exp[30];
  hex2bin("01696ff82000013faabbcc000161d40080c800000241588419fb594e4dde", exp, 30);
  exp[4] = 0x00;
  append_crc(exp, 30);
  CHECK_EQ(g_tx[8].len, 31);                 // first stage-1 window
  CHECK_MEM(&g_tx[8].data[1], exp, 30);
}

// =============================================================================
// Tests: over-the-air pairing through the raw FIFO decoder
// =============================================================================
static const uint8_t PAIR_MASTER[16] = { 0xde, 0xad, 0xbe, 0xef, 0x01, 0x23, 0x45, 0x67,
                                         0x89, 0xab, 0xcd, 0xef, 0x10, 0x32, 0x54, 0x76 };
static const uint8_t PAIR_ADDR[3] = { 0x12, 0x34, 0x56 };

static void test_pair_requires_transfer_key(void)
{
  g_test = "pair_needs_tk";
  fresh_device();
  cli(2, "pair", "skylight");
  CHECK_OUT("transfer key not set");
  CHECK(!g_velux_pair_mode);
  CHECK_EQ(railDataConfig.rxMethod, PACKET_MODE);
}

static void test_pair_all_phases(void)
{
  for (uint8_t P = 0; P < 16; P++) {
    for (unsigned align = 0; align < 2; align++) {
      g_test = "pair_phase";
      fresh_device();
      cli(2, "set_transfer_key", TEST_TK_HEX);
      cli(2, "pair", "skylight");
      CHECK(g_velux_pair_mode);
      CHECK_EQ(railDataConfig.rxMethod, FIFO_MODE);

      uint8_t f[34];
      make_key_transfer_frame(f, PAIR_ADDR, PAIR_MASTER, 0x0500);
      stream_add_frame(f, 34, P, 40 + align);
      stream_commit();
      out_clear();
      run_loop(400000);

      if (!strstr(g_out, "velux_paired")) {
        fprintf(stderr, "  (phase P=%u align=%u)\n", P, align);
      }
      CHECK_OUT("\"event\":\"velux_paired\",\"device\":\"skylight\",\"addr\":\"123456\"");
      CHECK_MEM(g_velux_nvm.sky_key, PAIR_MASTER, 16);
      CHECK_MEM(g_velux_nvm.sky_addr, PAIR_ADDR, 3);
      CHECK_EQ(g_velux_nvm.sky_seq, 0x0501);
      CHECK_EQ(g_velux_nvm.sky_valid, 1);
      CHECK_EQ(g_velux_nvm.blind_valid, 0);
      CHECK(!g_velux_pair_mode);
      CHECK_EQ(railDataConfig.rxMethod, PACKET_MODE);
    }
  }
}

static void test_pair_blind_and_persist(void)
{
  g_test = "pair_blind_persist";
  fresh_device();
  cli(2, "set_transfer_key", TEST_TK_HEX);
  cli(2, "pair", "blind");
  uint8_t f[34];
  make_key_transfer_frame(f, TEST_ADDR, TEST_KEY, 0x00FF);
  stream_add_frame(f, 34, 9, 33);
  stream_commit();
  run_loop(400000);
  CHECK_EQ(g_velux_nvm.blind_valid, 1);

  reboot();   // everything must survive a power cycle
  CHECK(g_velux_transfer_key_valid);
  CHECK_MEM(g_velux_transfer_key, TEST_TK, 16);
  CHECK_EQ(g_velux_nvm.blind_valid, 1);
  CHECK_MEM(g_velux_nvm.blind_key, TEST_KEY, 16);
  CHECK_MEM(g_velux_nvm.blind_addr, TEST_ADDR, 3);
  CHECK_EQ(g_velux_nvm.blind_seq, 0x0100);
  CHECK_EQ(g_velux_nvm.sky_valid, 0);
}

static void test_pair_corrupted_frame_ignored(void)
{
  g_test = "pair_bad_crc";
  fresh_device();
  cli(2, "set_transfer_key", TEST_TK_HEX);
  cli(2, "pair", "skylight");
  uint8_t f[34];
  make_key_transfer_frame(f, PAIR_ADDR, PAIR_MASTER, 0x0500);
  f[20] ^= 0x01;   // flip one bit after CRC was computed
  stream_add_frame(f, 34, 4, 40);
  stream_commit();
  out_clear();
  run_loop(400000);
  CHECK_NOT_OUT("velux_paired");
  CHECK_EQ(g_velux_nvm.sky_valid, 0);
  CHECK(g_velux_pair_mode);           // still waiting
  cli(1, "stop_pair");
  CHECK(!g_velux_pair_mode);
  CHECK_OUT("velux_pairing_stopped");
}

// =============================================================================
// Tests: wall-switch sync (packet-mode RX path)
// =============================================================================
static void queue_packet_direct(const uint8_t *psdu, uint8_t len)
{
  g_pkt[0] = len;                      // hardware locked on SFD: PHR at byte 0
  memcpy(&g_pkt[1], psdu, len);
  g_pkt_len = (uint16_t)(len + 1);
  g_pkt_pending = true;
  packetsHeld = 1;
}

static void test_wall_switch_sync(void)
{
  g_test = "wall_switch_sync";
  fresh_device();
  pair_blind_via_setkey();

  // Wall switch sends "open" with a counter well ahead of ours
  uint8_t f[28];
  velux_build_28b_frame(TEST_ADDR, TEST_KEY, 0x2000, 0x00, f);
  queue_packet_direct(f, 28);
  out_clear();
  run_loop(300000);
  CHECK_OUT("\"event\":\"velux_remote\",\"device\":\"blind\"");
  CHECK_OUT("\"action\":\"open\"");
  CHECK_EQ(g_velux_nvm.blind_seq, 0x2001);
  CHECK_EQ(g_velux_nvm.blind_pos, 100);

  // Our next command must continue from the synced counter
  g_tx_n = 0;
  cli(2, "blind", "close");
  const TxRec *last = &g_tx[g_tx_n - 1];
  CHECK_EQ(((last->data[1 + 18] << 8) | last->data[1 + 19]), 0x2002);

  // Unknown remote: ignored
  const uint8_t other[3] = { 0x01, 0x02, 0x03 };
  velux_build_28b_frame(other, TEST_KEY, 0x7000, 0xC8, f);
  queue_packet_direct(f, 28);
  out_clear();
  run_loop(300000);
  CHECK_NOT_OUT("velux_remote");
  CHECK_EQ(g_velux_nvm.blind_seq, 0x2003);

  // Corrupted frame: ignored
  velux_build_28b_frame(TEST_ADDR, TEST_KEY, 0x7000, 0xC8, f);
  f[15] ^= 0x40;
  queue_packet_direct(f, 28);
  out_clear();
  run_loop(300000);
  CHECK_NOT_OUT("velux_remote");
  CHECK_EQ(g_velux_nvm.blind_seq, 0x2003);
}

// =============================================================================
// Tests: flash persistence
// =============================================================================
static void test_nvm_wear_leveling(void)
{
  g_test = "nvm_wear_leveling";
  fresh_device();
  cli(2, "set_transfer_key", TEST_TK_HEX);
  pair_blind_via_setkey();
  unsigned erases0 = fake_flash_erase_count;

  // 400 saves -> must wrap the 127-slot log several times
  for (int i = 0; i < 400; i++) {
    char s[16];
    snprintf(s, sizeof(s), "%d", 1000 + i);
    cli(3, "setseq", "blind", s);
  }
  CHECK(fake_flash_erase_count >= erases0 + 3);

  reboot();
  CHECK_EQ(g_velux_nvm.blind_seq, 1399);
  CHECK_EQ(g_velux_nvm.blind_valid, 1);
  CHECK_MEM(g_velux_nvm.blind_key, TEST_KEY, 16);
  CHECK(g_velux_transfer_key_valid);         // survives page erases
  CHECK_MEM(g_velux_transfer_key, TEST_TK, 16);
}

static void test_transfer_key_change(void)
{
  g_test = "transfer_key_change";
  fresh_device();
  cli(2, "set_transfer_key", TEST_TK_HEX);
  pair_blind_via_setkey();
  cli(2, "set_transfer_key", "ffeeddccbbaa99887766554433221100");
  reboot();
  uint8_t exp[16];
  hex2bin("ffeeddccbbaa99887766554433221100", exp, 16);
  CHECK_MEM(g_velux_transfer_key, exp, 16);
  CHECK_EQ(g_velux_nvm.blind_valid, 1);      // device state kept
  CHECK_EQ(g_velux_nvm.blind_seq, 0x1234);
}

static void test_reset_nvm(void)
{
  g_test = "reset_nvm";
  fresh_device();
  cli(2, "set_transfer_key", TEST_TK_HEX);
  pair_blind_via_setkey();
  out_clear();
  cli(1, "reset_nvm");
  CHECK_OUT("velux_nvm_reset");
  reboot();
  CHECK(!g_velux_transfer_key_valid);
  CHECK_EQ(g_velux_nvm.blind_valid, 0);
  CHECK_EQ(g_velux_nvm.sky_valid, 0);
}

static void test_blank_device(void)
{
  g_test = "blank_device";
  fresh_device();
  CHECK_EQ(g_velux_nvm.blind_valid, 0);
  CHECK_EQ(g_velux_nvm.sky_valid, 0);
  CHECK(!g_velux_transfer_key_valid);
  static const uint8_t zero[16] = { 0 };
  CHECK_MEM(g_velux_nvm.blind_key, zero, 16);
  CHECK_MEM(g_velux_nvm.sky_key, zero, 16);
}

// =============================================================================
// Tests: CLI
// =============================================================================
static void test_cli(void)
{
  g_test = "cli";
  fresh_device();

  // Unpaired device: refuses to transmit
  g_tx_n = 0;
  cli(2, "blind", "open");
  CHECK_EQ(g_tx_n, 0);
  CHECK_OUT("Device not paired yet");

  // Status: both devices with no arguments, one device with a name
  out_clear();
  cli(0);
  CHECK_OUT("\"event\":\"velux_status\",\"device\":\"blind\"");
  CHECK_OUT("\"event\":\"velux_status\",\"device\":\"skylight\"");
  out_clear();
  cli(2, "skylight", "status");
  CHECK_OUT("\"device\":\"skylight\"");
  CHECK_NOT_OUT("\"device\":\"blind\"");

  // Argument errors
  out_clear();
  cli(3, "setkey", "blind", "0011");
  CHECK_OUT("Usage: velux setkey");
  CHECK_EQ(g_velux_nvm.blind_valid, 0);
  out_clear();
  cli(2, "set_transfer_key", "abcd");
  CHECK_OUT("Expected 32 hex chars");
  CHECK(!g_velux_transfer_key_valid);
  out_clear();
  cli(1, "open");
  CHECK_OUT("Missing device");
  out_clear();
  cli(2, "pair", "garage");
  CHECK_OUT("Usage: velux pair");
  out_clear();
  cli(2, "blind", "dance");
  CHECK_OUT("Unknown command");

  // Device/action order is flexible
  pair_blind_via_setkey();
  g_tx_n = 0;
  cli(2, "close", "blind");
  CHECK(g_tx_n > 0);

  // Aliases
  out_clear();
  g_tx_n = 0;
  cli(2, "sky", "status");
  CHECK_OUT("\"device\":\"skylight\"");
}

// =============================================================================
int main(void)
{
  test_crc();
  test_nibble_map();
  test_frames_golden();
  test_blank_device();
  test_tx_close_blind();
  test_tx_positions();
  test_skylight_broadcast_class();
  test_pair_requires_transfer_key();
  test_pair_all_phases();
  test_pair_blind_and_persist();
  test_pair_corrupted_frame_ignored();
  test_wall_switch_sync();
  test_nvm_wear_leveling();
  test_transfer_key_change();
  test_reset_nvm();
  test_cli();

  printf("%d checks, %d failures\n", g_checks, g_failures);
  return g_failures ? 1 : 0;
}
