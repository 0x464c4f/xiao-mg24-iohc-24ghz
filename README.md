# \$10 2.4 GHz Velux Skylight & Blind Controller (AU / NZ / US / CA) for Home Assistant (`Seeed Studio XIAO MG24`)

> **Disclaimer:** This is an independent, clean-room interoperability project and is **not** affiliated with, endorsed by, or supported by VELUX A/S, Somfy, or io-homecontrol. All trademarks (`VELUX®`, `io-homecontrol®`, `KLI 310/311/312`) belong to their respective owners and are used strictly for nominative fair-use identification of compatible hardware. **Motorized skylights can cause injury or severe rain/water damage if operated unattended—use at your own risk.** See [DISCLAIMER.md](DISCLAIMER.md) for full legal, statutory interoperability (AU Copyright Act s 47D / US DMCA § 1201(f) / EU Dir. 2009/24 Art. 6), and safety notices.

---

## Overview

If you live in **Australia, New Zealand, the United States, or Canada** and own **Velux Solar Skylights or Solar Blinds** controlled by `KLI 310`, `KLI 311`, or `KLI 312` wall switches (`FCC ID: XSG831641`), you may have discovered a frustrating surprise:

1. **Every existing open-source `io-homecontrol` GitHub project only supports European 868 MHz (`2-FSK`).** In AU/NZ/US/CA, Velux uses a completely different **2.4 GHz physical layer** (`2425 MHz`, `2450 MHz`, `2475 MHz` O-QPSK) that `SX1276` / `CC1101` transceivers cannot tune to.
2. **Standard Zigbee / Thread 2.4 GHz dongles are deaf to it** because the 2.4 GHz `io-homecontrol` PHY uses **inverted DSSS chip polarity**, a **non-standard 1-byte Start-of-Frame Delimiter (`0x56` instead of `0xA7`)**, and a **two-stage Low-Power Mode (LPM) wake-up beacon train**.
3. Until now, the only ways to integrate AU/US Velux skylights and blinds into Home Assistant were buying a **\$250–\$350+ KLF 200 / KIG 300 gateway** or **soldering optocouplers/relays onto a sacrificial \$80+ KLI wall switch PCB** for every single blind or skylight (which also loses `0–100%` position control).

This project turns a single **~\$10 Seeed Studio XIAO MG24** (`Silicon Labs EFR32MG24B220F1536IM48`) thumb-sized USB-C board into a **self-contained, multi-device 2.4 GHz `io-homecontrol` 1-Way transceiver** for Home Assistant—**100% in software over USB-C, with zero soldering**.

### How This Compares to Existing Solutions

| Solution / Project | Frequency & Region | Hardware Cost | Soldering Required? | Exact `0–100%` Position? | Multi-Device on 1 Board? |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **[`rspaargaren/iohomecontrol`](https://github.com/rspaargaren/iohomecontrol)** / **[`iown-homecontrol`](https://github.com/rspaargaren/iown-homecontrol-esp32sx1276)** | **868 MHz (EU only)** (`2-FSK`, `38.4 kbps`) | ~\$15 (`ESP32` + `SX1276`) | Yes (SPI wiring) | Yes (EU 868 MHz only) | **Incompatible with AU/US 2.4 GHz Velux** |
| **ESPHome / Optocoupler Relay Hack** | Any (wires soldered to remote buttons) | \$15 + **\$80+ per sacrificial KLI remote** | **Yes** (fine-pitch PCB soldering) | **No** (Open / Close / Stop timing only) | **No** (1 sacrificial remote per device) |
| **Official Gateway (`KLF 200` / `KIG 300`)** | 868 MHz or 2.4 GHz | **\$200 – \$350+** | No | Yes | Yes |
| **This Project (`xiao-mg24-velux-iohc`)** | **2.4 GHz (AU / NZ / US / CA)** (`O-QPSK` `2425/2450/2475 MHz`) | **~\$10** (Plug-and-play USB-C `XIAO MG24`) | **No** (100% USB-C) | **Yes (`0–100%`)** | **Yes** (Skylight + Blind on 1 dongle) |

---

## Key Features

- **100% Self-Contained On-Chip Firmware:**
  - Runs directly on the `EFR32MG24B220F1536IM48` on a Seeed Studio XIAO MG24.
  - Performs AES-128 ECB encryption, 1-Way `io-homecontrol` MAC generation, software CRC-16/KERMIT (`0x8408` reflected), and microsecond-accurate `1000.0 µs` LPM wake-up beacon trains (`147 ms` LPM + `47 ms` Non-LPM) across all three 2.4 GHz channels (`2425 / 2450 / 2475 MHz`).
- **One-Command Over-the-Air Remote Pairing (`velux pair`):**
  - Run `velux pair blind` or `velux pair skylight` and press the recessed **PAIR** button on the back of your `KLI 311` / `KLI 312` remote for 1 second.
  - The XIAO MG24 captures the `34-byte` (`0x30`) key-transfer broadcast over the air, unwraps the device's 16-byte Master Key on-chip, learns its 3-byte source address and rolling counter, and saves everything to internal Flash NVM.
  - Because the XIAO MG24 clones your remote's address and syncs rolling counters when it hears your wall switch transmit, **your physical wall switches continue working alongside Home Assistant**.
- **Wear-Leveled Internal Flash Persistence (`0x0817C000`):**
  - Paired keys, device addresses, rolling sequence numbers, and last-known `0–100%` positions survive power loss and reboots using a 127-slot wear-leveled log in internal Flash.
- **Zero-Dependency Home Assistant Bridge ([`homeassistant/velux_ha_ctl.py`](homeassistant/velux_ha_ctl.py)):**
  - Uses only the Python 3 standard library (`os`, `termios`, `select`, `glob`)—no `pip install pyserial` or external packages needed inside Home Assistant OS!

---

## Hardware Required

1. **Seeed Studio XIAO MG24** (or XIAO MG24 Sense) — based on the Silicon Labs `EFR32MG24B220F1536IM48` SoC (~\$10 USD).
   - Uses the onboard ceramic antenna by default (`PB05 = HIGH`, `PB04 = LOW`), or an external 2.4 GHz U.FL antenna.
2. **USB-C Data Cable** connected to your Home Assistant host (or Proxmox server with USB passthrough).
3. **Your existing 2.4 GHz Velux Remote (`KLI 310 / 311 / 312`)** to pair from.

---

## Step 1: Build & Flash the Firmware

### 1.1. Prerequisites
- Install **[Simplicity Studio v5](https://www.silabs.com/developers/simplicity-studio)** (free from Silicon Labs) with the **Simplicity SDK (`2025.6.x` or newer)** and the **GNU ARM Toolchain**.

### 1.2. Create or Import the Project in Simplicity Studio v5
1. Plug your **Seeed Studio XIAO MG24** into your computer via USB-C.
2. In Simplicity Studio v5, create a new project for target part **`EFR32MG24B220F1536IM48`** using the **`RAIL - SoC RAILtest`** (`rail_soc_railtest`) example template (or import [`firmware/velux.slcp`](firmware/velux.slcp)).
3. Copy the files from this repository's [`firmware/`](firmware/) folder into your Simplicity Studio project:
   - [`firmware/app.c`](firmware/app.c) → `<project>/app.c`
   - [`firmware/app.h`](firmware/app.h) → `<project>/app.h`
   - [`firmware/main.c`](firmware/main.c) → `<project>/main.c` (unchanged Silicon Labs template, only needed if you import `velux.slcp`)
   - [`firmware/config/sl_device_init_hfxo_config.h`](firmware/config/sl_device_init_hfxo_config.h) → `<project>/config/sl_device_init_hfxo_config.h`
   - [`firmware/config/rail/radio_settings.radioconf`](firmware/config/rail/radio_settings.radioconf) → `<project>/config/rail/radio_settings.radioconf`
   - [`firmware/autogen_patches/rail_config.c`](firmware/autogen_patches/rail_config.c) → `<project>/autogen/rail_config.c`
4. Ensure the `velux` CLI command is registered in `<project>/autogen/sl_cli_command_table.c` (see [`firmware/autogen_patches/sl_cli_command_table_velux.inc`](firmware/autogen_patches/sl_cli_command_table_velux.inc)).

> [!IMPORTANT]
> **Why `sl_device_init_hfxo_config.h` (`CTUNE = 140`) and `rail_config.c` Matter:**
> - Silicon Labs' default `CTUNE` targets a radio board with different crystal load capacitors, causing a `-65 kHz` carrier frequency offset at 2.4 GHz on the Seeed XIAO MG24. Setting `SL_DEVICE_INIT_HFXO_CTUNE` to `120–140` centers the 39.0 MHz crystal within `< 2 kHz`.
> - If Simplicity Studio's Radio Configurator regenerates `autogen/rail_config.c`, re-copy [`firmware/autogen_patches/rail_config.c`](firmware/autogen_patches/rail_config.c) before compiling so the `Z2 × Z8` DSSS phase inversion and `FCD0` CRC overrides remain in place.

5. Click **Build** (hammer icon) in Simplicity Studio, then flash the resulting `.hex` / `.s37` binary to the XIAO MG24 using Simplicity Studio's **Flash Programmer** or OpenOCD (via the XIAO MG24's built-in CMSIS-DAP interface).

---

## Step 2: Configure the 1-Way Transfer Key & Pair Your Remotes

To keep this repository 100% free of third-party cryptographic keys, a newly flashed XIAO MG24 starts with blank NVM slots. You only need to configure the transfer key and pair your remotes **once**—everything is saved permanently to the MG24's internal Flash (`0x0817C000`).

### 2.1. Set the 1-Way System Transfer Key (One-Time Setup)
When a 1-Way `io-homecontrol` remote (`KLI 311` / `KLI 312`) broadcasts its device key via the rear **PAIR** button (`Command 0x30`), the key is XOR-encrypted with `AES-128-ECB(transfer_key, addr_iv)`. The 16-byte 1-Way transfer key is publicly documented in community projects such as [`rspaargaren/iohomecontrol`](https://github.com/rspaargaren/iohomecontrol).

Run the following command once using [`velux_ha_ctl.py`](homeassistant/velux_ha_ctl.py) (or type `velux set_transfer_key <32-hex-chars>` in any `115200 8N1` serial terminal):

```bash
python3 homeassistant/velux_ha_ctl.py set_transfer_key <32-HEX-CHARS-FROM-IOHOMECONTROL-DOCS>
```
 Expected response:
```json
{"event":"velux_transfer_key","transfer_key_set":true}
```

*(Alternatively, if you already know your device's 16-byte key and 3-byte address, you can load it directly via `velux setkey blind <32-hex-key> <6-hex-addr>` without needing the transfer key).*

### 2.2. Pair Your Blind & Skylight Remotes Over-the-Air
1. Put the XIAO MG24 into pairing listen mode for `blind` or `skylight`:
   ```bash
   python3 homeassistant/velux_ha_ctl.py pair skylight
   ```
   Output:
   ```json
   {"event":"velux_pairing_armed","device":"skylight","info":"Press the rear PAIR/RESET button on the remote now..."}
   Waiting for rear PAIR/RESET button press on skylight remote...
   ```
2. Turn your `KLI 311` (Skylight) or `KLI 312` (Blind) wall switch over and press the recessed **PAIR / Config** button on the back (next to the gear icon) briefly (~1 second) with a paperclip.
3. Within ~1 second, the XIAO MG24 captures the `34-byte` (`PHR = 0x22`, `Cmd = 0x30`) frame, decrypts the Master Key, saves it to Flash NVM, and prints:
   ```json
   {"event":"velux_paired","device":"skylight","addr":"aabbcc","action":"paired","position":0,"seq":5,"seq_hex":"0x0005","paired":true,"transfer_key_set":true,"key":"..."}
   ```
4. Repeat for your blind (`python3 homeassistant/velux_ha_ctl.py pair blind`).

### 2.3. Test Controlling Your Skylight & Blind
```bash
# Check status of both devices
python3 homeassistant/velux_ha_ctl.py status

# Control the Solar Blind
python3 homeassistant/velux_ha_ctl.py blind open
python3 homeassistant/velux_ha_ctl.py blind pos 50
python3 homeassistant/velux_ha_ctl.py blind stop
python3 homeassistant/velux_ha_ctl.py blind close

# Control the Solar Skylight
python3 homeassistant/velux_ha_ctl.py skylight open
python3 homeassistant/velux_ha_ctl.py skylight stop
python3 homeassistant/velux_ha_ctl.py skylight close
```

---

## Step 3: Proxmox USB Passthrough & Home Assistant Setup

Because all keys and rolling sequence counters live in the XIAO MG24's internal Flash, you can unplug it from your laptop and plug it straight into your Home Assistant server.

### 3.1. Proxmox VE USB Passthrough (If Running HAOS in a VM)
1. Plug the XIAO MG24 into a USB port on your Proxmox host.
2. In the Proxmox Web UI, select your **Home Assistant VM** → **Hardware** → **Add** → **USB Device**.
3. Choose **Use USB Vendor/Device ID** and select **`Seeed Studio XIAO MG24 (2886:0062)`** (or CMSIS-DAP).
4. Click **Add** (Proxmox hot-plugs USB devices into Home Assistant OS immediately—no VM reboot required).
5. In Home Assistant, go to **Settings → System → Hardware → All Hardware** and search for `ttyACM`—you will see `/dev/ttyACM0` (or `/dev/ttyACM1`) and `/dev/serial/by-id/usb-Seeed_Studio_XIAO_MG24_...`.

### 3.2. Install `velux_ha_ctl.py` in Home Assistant
1. Copy [`homeassistant/velux_ha_ctl.py`](homeassistant/velux_ha_ctl.py) into your Home Assistant `/config/` directory (`/config/velux_ha_ctl.py`).
2. Make it executable:
   ```bash
   chmod +x /config/velux_ha_ctl.py
   ```
3. Test it from the Home Assistant Terminal:
   > **Note for the Home Assistant "Terminal & SSH" Add-on:** The Terminal add-on runs inside a minimal Alpine container that doesn't install `python3` by default (even though the main Home Assistant Core container *does* have Python 3). To test from the Terminal add-on, run `apk add python3` first (or run `ha core exec python3 /config/velux_ha_ctl.py status`).
   ```bash
   python3 /config/velux_ha_ctl.py status
   ```

### 3.3. Add Cover Entities to `/config/configuration.yaml`
Copy the contents of [`homeassistant/configuration.yaml.example`](homeassistant/configuration.yaml.example) into your `/config/configuration.yaml` (merge with any existing `command_line:`, `shell_command:` or `template:` sections) and restart Home Assistant.

It creates:

| Entity | Purpose |
| :--- | :--- |
| `cover.velux_solar_blind` | Open / close / stop + 0–100 % position slider |
| `cover.velux_solar_skylight` | Open / close / stop + 0–100 % position slider |
| `sensor.velux_blind_position`, `sensor.velux_skylight_position` | Last known position reported by the XIAO MG24 (also updated when you use the wall switch) |

The covers are [template covers](https://www.home-assistant.io/integrations/template/) that call `velux_ha_ctl.py` via `shell_command`, because the `command_line` cover integration has no position support. `velux_ha_ctl.py` locks the serial port, so overlapping commands from automations are queued instead of colliding.

---

## Serial CLI Reference (`115200 8N1`)

You can also interact with the XIAO MG24 directly over any serial terminal (`screen`, `minicom`, PuTTY):

| Command | Description |
| :--- | :--- |
| `velux status` | Print JSON state for both `blind` and `skylight` |
| `velux set_transfer_key <32-hex>` | Save the 16-byte 1-Way transfer key to Flash NVM (`0x0817FFC0`) |
| `velux pair blind` | Arm over-the-air pairing for `blind` (press rear PAIR button on remote) |
| `velux pair skylight` | Arm over-the-air pairing for `skylight` (press rear PAIR button on remote) |
| `velux stop_pair` | Cancel pairing mode |
| `velux <blind\|skylight> open` | Open (`0x00` / HA `100%`) |
| `velux <blind\|skylight> close` | Close (`0xC8` / HA `0%`) |
| `velux <blind\|skylight> stop` | Stop (`0xD2`) |
| `velux <blind\|skylight> vent` | Move to 50 % |
| `velux <blind\|skylight> pos <0..100>` | Move to exact percentage (`0` = closed, `100` = open) |
| `velux <blind\|skylight> status` | Print JSON state for one device (used by Home Assistant) |
| `velux setkey <blind\|skylight> <32-hex> [6-hex]` | Manually set 16-byte device key and (optionally) 3-byte address |
| `velux setseq <blind\|skylight> <seq>` | Manually set 16-bit rolling sequence counter |
| `velux reset_nvm` | Erase Flash NVM (`0x0817C000`) and clear all paired keys |

---

## Running the Tests

The firmware logic is covered by host-side unit tests that compile the real `firmware/app.c` against a fake radio, flash and CLI. You don't need the board or the Silicon Labs SDK, just a C compiler:

```bash
cd tests/host && make test
```

They check frame bytes against frames from the hardware-verified firmware, the full transmit schedule (channels, 1000 µs beacon grid, command windows), over-the-air pairing at all 16 receiver phases, wall-switch counter sync, flash persistence and wear leveling, and CLI error handling.

---

## Technical Deep-Dive: How the 2.4 GHz PHY & Protocol Work

Curious how Velux's 2.4 GHz O-QPSK physical layer, `Z2 × Z8` DSSS chip inversion, `0x56` sync word, and two-stage `30B Non-LPM` + `28B LPM` wake-up sequence work under the hood?
👉 Read **[`docs/PROTOCOL_24GHZ.md`](docs/PROTOCOL_24GHZ.md)**.

---

## Credits & Acknowledgments

- **Google Antigravity (AGY) & Gemini:** Over-the-air RF signal analysis, `EFR32MG24` hardware register discovery (`Z2 × Z8` DSSS phase group permutation, `0x56` SFD correlation, `FCD0` CRC override), two-stage LPM wakeup protocol timing, embedded C firmware, and Home Assistant integration were developed in pair-programming collaboration with **Google Antigravity (AGY)** powered by **Gemini**.
- **[`rspaargaren/iohomecontrol`](https://github.com/rspaargaren/iohomecontrol) & [`rspaargaren/iown-homecontrol-esp32sx1276`](https://github.com/rspaargaren/iown-homecontrol-esp32sx1276):** Community documentation of the `io-homecontrol` application-layer framing and 1-Way MAC transformation (`iohc_checksum_step` / `iohc_create_1w_hmac`), licensed under Apache-2.0.

## License

Licensed under the **[Apache License, Version 2.0](LICENSE)**. See [NOTICE](NOTICE) and [DISCLAIMER.md](DISCLAIMER.md) for attributions and legal/safety notices.
