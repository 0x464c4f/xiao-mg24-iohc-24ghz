# Legal, Trademark, Interoperability & Safety Disclaimer

## 1. Non-Affiliation & Nominative Trademark Fair Use
This project is an independent, community-developed hardware and software interoperability bridge. It is **not** affiliated with, endorsed by, sponsored by, authorized by, or supported by:
- **VELUX A/S** or **VKR Holding A/S**
- **Somfy Activities SA** or **Overkiz**
- **Silicon Laboratories Inc.**
- **Seeed Technology Co., Ltd.**

All product, brand, and company names mentioned in this repository—including **`VELUX®`**, **`io-homecontrol®`**, **`KLI 310`**, **`KLI 311`**, **`KLI 312`**, **`KLF 200`**, **`EFR32MG24`**, and **`Seeed Studio XIAO MG24`**—are trademarks or registered trademarks of their respective owners. Any use of these marks in this repository and accompanying documentation is strictly for **nominative fair-use identification** of compatible third-party hardware owned by the end user.

---

## 2. Clean-Room Interoperability Research (Statutory Protections)
All 2.4 GHz physical-layer parameters (`2425 / 2450 / 2475 MHz` carrier frequencies, `250 kbps` O-QPSK symbol mapping, `0x56` Start-of-Frame Delimiter, Low-Power Mode wake-up beacon timing, and two-stage command framing) documented and implemented in this repository were obtained exclusively through:
1. Publicly available regulatory test reports and filings (including **FCC ID: `XSG831641`**), and
2. Clean-room, over-the-air radio frequency (RF) signal observation of hardware lawfully owned by the author.

**No proprietary vendor SDKs, leaked confidential documentation, Non-Disclosure Agreements (NDAs), or firmware extracted from locked/readout-protected microcontrollers were used.**

This project is published in good faith solely to enable lawful owners of 2.4 GHz `io-homecontrol` actuators to achieve local smart-home interoperability (e.g., with Home Assistant), pursuant to statutory interoperability exemptions worldwide:
- **Australia:** Section 47D of the *Copyright Act 1968* (Cth) (Reproduction or adaptation of computer programs for interoperability)
- **United States:** 17 U.S.C. § 1201(f) (*Digital Millennium Copyright Act* — Reverse Engineering for Interoperability)
- **European Union:** Article 6 of Directive 2009/24/EC on the legal protection of computer programs (Decompilation/observation for interoperability)

---

## 3. Zero Cryptographic Keys Included
To respect intellectual property boundaries, **this repository contains zero third-party or device-specific cryptographic keys**:
- No personal actuator keys or device addresses are stored in the source code or commit history.
- No vendor system or key-transfer keys are hardcoded in the firmware source. Users must pair their own physical remotes that they lawfully own (`velux pair`) after configuring their own key material in Flash NVM.

---

## 4. Physical Safety & Property Damage Warning (Motorized Skylights & Blinds)
> **WARNING — OPERATE AT YOUR OWN RISK:**
> Motorized skylights, roof windows, and blinds exert mechanical force and expose the interior of a building to outdoor weather conditions.
>
> 1. **Pinch / Crush Hazard:** Never operate motorized skylights or windows remotely when people, children, pets, or objects could be in the path of the moving sash.
> 2. **Water / Weather Damage Hazard:** Opening a skylight remotely or via an unattended automation during rain, hail, snow, or high winds can cause severe interior water or structural damage. Always ensure hardware rain sensors remain functional and never rely solely on software automations for weather protection.
> 3. **Radio Interference & Regulatory Compliance:** Users are solely responsible for ensuring that operating a 2.4 GHz transceiver on `2425 MHz`, `2450 MHz`, and `2475 MHz` at `+10 dBm` complies with their local radio spectrum regulations (e.g., ACMA LIPD Class Licence in Australia, FCC Part 15 in the US).

As stated in Section 7 and Section 8 of the [Apache License 2.0](LICENSE), this software is provided **"AS IS", WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND**, and in no event shall the authors or contributors be liable for any direct, indirect, incidental, or consequential damages arising out of the use of this software or hardware.
