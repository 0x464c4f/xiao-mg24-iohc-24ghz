# Technical Specification: 2.4 GHz `io-homecontrol` (AU / NZ / US / CA) PHY & Two-Stage LPM Protocol

> **Clean-Room Research Note:** All physical-layer and framing specifications documented below were determined through publicly available regulatory filings (**FCC ID: `XSG831641`**) and clean-room over-the-air RF observation of user-owned `KLI 311` and `KLI 312` wall switches using a Seeed Studio XIAO MG24 (`EFR32MG24B220F1536IM48`) in collaboration with **Google Antigravity (AGY) / Gemini**. Zero vendor keys or personal device keys are included in this document (all examples use synthetic addresses `AA BB CC` and dummy keys).

---

## 1. Physical Layer (PHY) Overview

Unlike European `io-homecontrol` devices (which operate on three 868 MHz channels using `38.4 kbps` 2-FSK and UART start/stop bit framing), North American and Asia-Pacific (`AU / NZ / US / CA`) Velux devices operate in the **2.4 GHz ISM band** using a modified **IEEE 802.15.4 O-QPSK DSSS** physical layer:

| Parameter | 2.4 GHz `io-homecontrol` Value | Standard IEEE 802.15.4 (Zigbee/Thread) |
| :--- | :--- | :--- |
| **Carrier Frequencies** | **`2425.0 MHz` (Ch 0), `2450.0 MHz` (Ch 5), `2475.0 MHz` (Ch 10)** | `2405..2480 MHz` (5 MHz grid, Ch 11..26) |
| **Modulation** | O-QPSK with half-sine chip shaping | O-QPSK with half-sine chip shaping |
| **Chip Rate / Bit Rate** | `2.0 Mcps` / `250 kbps` (`62.5 ksym/s`, 4 bits/symbol) | `2.0 Mcps` / `250 kbps` |
| **DSSS Chip Polarity** | **Inverted / `Z2 × Z8` Phase-Permuted** | Standard 32-chip IEEE 802.15.4 table |
| **Preamble** | `4 bytes` (`8 nibbles` of `0x0`) | `4 bytes` (`0x00000000`) |
| **Start-of-Frame Delimiter (SFD)** | **`0x56` (`1 byte`: nibble `6` then nibble `5`)** | `0xA7` (`1 byte`: nibble `7` then nibble `A`) |
| **Frame Length Header (`PHR`)** | `1 byte` (`3..36` bytes total PSDU length including CRC) | `1 byte` (`0..127` bytes) |
| **CRC-16** | **CRC-16/KERMIT** (`poly = 0x8408` reflected, `init = 0x0000`, little-endian) | IEEE 802.15.4 ITU-T CRC-16 |

---

## 2. Why Standard 802.15.4 Radios Are Deaf (The `Z2 × Z8` Group Inversion)

When an `EFR32MG24` receiver is configured with the standard IEEE 802.15.4 32-chip table (`dsss_chipping_code = 0x744AC39B`), incoming 2.4 GHz Velux packets lock at one of 16 carrier/timing phase states $P \in \{0..15\}$. Because the 32-chip cyclic shift + I/Q inversion forms the mathematical group $\mathbb{Z}_2 \times \mathbb{Z}_8$, each received 4-bit symbol nibble $n$ is transformed relative to the preamble symbol $P$ by:

$$\text{inv\_nibble}(n, P) = \bigl((n + 8 - (P \bmod 8)) \bmod 8\bigr) \;\Big|\; \bigl((n \oplus P) \ \& \ 8\bigr)$$

- **During RX (`dump_fifo_stream_over_serial` in [`firmware/app.c`](../firmware/app.c)):**
  - When $P = 0$, the on-air SFD byte `0x56` appears in the raw demodulator stream as `0x6A` (since $\text{inv\_nibble}(0\text{xA}, 0) = 6$ and $\text{inv\_nibble}(0\text{x6}, 0) = 5$, yielding byte `0x56`).
  - By configuring `MODEM_NS->SYNC0 = 0x6A` and `SYNC1 = 0x56` with `DUALSYNC` enabled in FIFO stream mode, and scanning across all $P \in \{0..15\}$, the XIAO MG24 achieves 100% packet reception across all phase locks.
- **During TX (`transmit_velux_burst` in [`firmware/app.c`](../firmware/app.c)):**
  - Setting `MODEM_NS->SYNC0 = 0x56` (`0xB8014078 = 0x56`) transmits the exact `0x00 00 00 00 56` preamble + SFD expected by Velux actuators.
  - Overriding the Frame Controller descriptors `FRC_NS->FCD0..FCD3` (`0xB8004108..0xB8004114 = 0x000040FF`) disables the hardware CRC generator so our software-computed CRC-16/KERMIT at the end of the PSDU is transmitted unmodified.

---

## 3. Low-Power Mode (LPM) Wake-Up Grid (`1000.0 µs` Tick Period)

Because Velux Solar Skylights and Solar Blinds run on batteries charged by small solar panels, their 2.4 GHz receivers sleep most of the time and only sample the RF channel briefly for wake-up beacons.

Every transmission on a channel consists of a **Countdown Beacon Train** followed by **4 Command Windows (`W0..W3`)**, all locked to a strict **`1000.0 µs` (`1.000 ms`) time grid**:

### 3.1. Wake-Up Countdown Beacon (`6 bytes` on air: `PHR = 0x05`)
```text
PHR   Payload (3 bytes)      CRC-16/KERMIT (2 bytes)
05    00 [cnt_hi] [cnt_lo]   [crc_lo] [crc_hi]
```
- `cnt` counts down by 1 every `1000.0 µs` (`1.000 ms`) down to `0x0000`.
- Special case at `cnt == 0`: `05 00 00 00 00 00` (all zeros for CRC on the final `cnt = 0` tick).
- When a sleeping actuator wakes up and hears a `6-byte` beacon with `cnt = K`, it knows the first Command Window (`W0`) will arrive in exactly `K + 1` milliseconds!

### 3.2. Two Sub-Burst Types (`147 ms` LPM vs. `47 ms` Non-LPM)
1. **LPM Sub-Burst (`147 ms`, `lpm_wakeup = true`, `Trigger 3`):**
   - **Main Countdown (`tick = 0..106`):** `107` beacons with `cnt = 106` (`0x006A`) down to `0`.
   - **4 Command Windows (`W0..W3` at `tick = 107, 120, 133, 146`):**
     - `W0` (`tick = 107`): Transmits the command PSDU with `CB2 = 0x20` (LPM flag set).
     - `tick = 108..111`: `4 ms` of radio silence.
     - `tick = 112..119`: Short countdown `cnt = 7` down to `0`.
     - `W1` (`tick = 120`), `W2` (`tick = 133`), `W3` (`tick = 146`): Transmit the command PSDU with `CB2 = 0x00`, separated by the same `4 ms` silence + `cnt = 7..0` short countdowns.
2. **Non-LPM Sub-Burst (`47 ms`, `lpm_wakeup = false`, `Trigger 4`):**
   - **Initial Short Countdown (`tick = 0..7`):** `8` beacons with `cnt = 7` down to `0`.
   - **4 Command Windows (`W0..W3` at `tick = 8, 21, 34, 47`):** All 4 windows transmit with `CB2 = 0x00`, separated by `cnt = 7..0` short countdowns.

---

## 4. Two-Stage Command Framing (`30B Stage 1` + `28B Stage 2`)

When you press Open, Close, or Stop on a `KLI 311` (Skylight) or `KLI 312` (Blind) wall switch, the remote transmits **two distinct command stages** with consecutive 16-bit rolling sequence numbers (`seq = N` and `seq = N + 1`):

### Stage 1: `30-Byte` Pre-Command (`PHR = 0x1E`, `47 ms` Non-LPM, `seq = N`)
```text
Byte:  00 01 02 03  04    05 06 07     08 09 10  11 12 13 14 15 16 17 18 19  20 21    22..27        28 29
Hex:   01 69 6f f8  [CB2] 00 [BCAST]   [SRC_3B]  00 01 61 d4 00 80 c8 00 00  [SEQ_N]  [6B_1W_MAC]   [CRC16]
```
- `01 69 6f`: Fixed 2.4 GHz `io-homecontrol` prefix (`"io"` in ASCII at bytes `1..2`!).
- `f8`: Control Byte 1 for a `30-byte` (`22-byte` payload + `6-byte` MAC + `2-byte` CRC) 1-Way broadcast frame.
- `00 [BCAST]`: Actuator type broadcast address:
  - **Solar Blind (`KLI 312`):** `00 00 7f` (Roller Shutter / Blind class)
  - **Solar Skylight (`KLI 311`):** `00 01 3f` (Window Opener class `(4 << 6) | 0x3f`)
- `00 01 61 d4 00 80 c8 00 00`: 9-byte application payload (`Cmd = 0x01`, `0x61 d4`). Note that byte 17 is fixed at `0xC8` (`200`) in Stage 1 for both Open and Close.

### Stage 2: `28-Byte` LPM Execution Command (`PHR = 0x1C`, `147 ms` LPM, `seq = N + 1`)
```text
Byte:  00 01 02 03  04    05 06 07  08 09 10  11 12 13 14    15 16 17  18 19      20..25        26 27
Hex:   01 69 6f f6  [CB2] 00 00 3f  [SRC_3B]  00 01 61 [POS] 00 00 00  [SEQ_N+1]  [6B_1W_MAC]   [CRC16]
```
- `f6`: Control Byte 1 for a `28-byte` 1-Way broadcast frame.
- `00 00 3f`: Universal broadcast target.
- `00 01 61 [POS] 00 00 00`: 7-byte application payload where `[POS]` (`main0`) encodes the target position or action:
  - `0x00` (`0`): **Open (`100%` in Home Assistant)**
  - `0xC8` (`200`): **Close (`0%` in Home Assistant)**
  - `0xD2` (`210`): **Stop / My Position**
  - `0x01 .. 0xC7` (`1 .. 199`): **Exact intermediate position** ($\text{main0} = 200 - 2 \times \text{HA\_pct}$)

> **Why Sequence Reuse Across Multi-Channel Bursts Is Critical:**
> Each Stage (`30B` then `28B`) is transmitted across the 2.4 GHz channels (`2425 MHz`, `2450 MHz`, `2475 MHz`). All channel repetitions within a single button press **must reuse the exact same `(seq = N, seq = N + 1)` pair**. If a controller increments the sequence number to `(N + 2, N + 3)` on a retry ~1 second later while the skylight motor is already moving, the actuator interprets the new sequence number as a *second button press while in motion*—which triggers an immediate **STOP**!

---

## 5. Over-the-Air Key Transfer Frame (`34 Bytes`, `PHR = 0x22`, `Cmd = 0x30`)

When the recessed **PAIR / Config** button on the back of a `KLI 311` or `KLI 312` remote is pressed briefly (~1 second), the remote transmits a `34-byte` (`PHR = 0x22`) LPM burst containing the device's encrypted 16-byte Master Key:

```text
Byte:  00 01 02 03  04    05 06 07  08 09 10  11  12..27             28 29  30 31    32 33
Hex:   01 69 6f fc  [CB2] 00 01 3f  [SRC_3B]  30  [16B_ENC_KEY]      01 01  [SEQ_N]  [CRC16]
```
The 16-byte Master Key is unwrapped on-chip (`iohc_decrypt_1w_key` in [`firmware/app.c`](../firmware/app.c)) by repeating the 3-byte source address `SRC_3B` five times plus `SRC_3B[0]` to form a 16-byte IV, encrypting that IV with AES-128-ECB under the 1-Way system transfer key, and XORing the resulting 16-byte keystream with `16B_ENC_KEY`.
