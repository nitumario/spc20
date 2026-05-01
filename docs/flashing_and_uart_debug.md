# Flashing & UART Debug Guide — SPC_20 via LP-MSPM0L1306 XDS110

This guide explains how to use a **LP-MSPM0L1306** LaunchPad as a standalone
XDS110 debug probe to program the SPC_20 board (MSPM0G3507) and how to read
the firmware's UART debug output through the same USB cable using CCS.

The LP-MSPM0L1306 has an onboard **XDS110-ET** debug probe and a **backchannel
UART** (virtual COM port). Both ride on the single USB connection to the PC.

---

## 1. Use the XDS110 OUT debug connector

The LP-MSPM0L1306 exposes the probe on a **10-pin 0.05" Cortex Debug
connector** labelled **XDS110 OUT** (sometimes silkscreened `J101`/`J102`
depending on board revision; it's the small fine-pitch box header near the
USB end of the board, not the 0.1" pin headers used by BoosterPacks).

This connector is the supported way to drive an external target. It carries
SWDIO, SWDCLK, nRST, target VCC sense, and GND on the standard
ARM 10-pin pinout — no jumper surgery required.

> The `XDS110 IN` connector is the reverse: it lets an **external** XDS110
> drive the on-board L1306. You don't need it for this workflow.

### Cable

Use a standard **ARM 10-pin (0.05" / 1.27 mm) Cortex Debug ribbon cable**.
TI ships one with the XDS110 standalone probe; any 10-pin SWD ribbon works.

### Cortex Debug 10-pin pinout (for reference)

```
       ┌──────────┐
   VCC │ 1     2 │ SWDIO
   GND │ 3     4 │ SWDCLK
   GND │ 5     6 │ SWO   (not used on MSPM0)
   KEY │ 7     8 │ NC
GNDDet │ 9    10 │ nRST
       └──────────┘
```

If your SPC_20 board has the same 10-pin Cortex Debug header, plug the
ribbon in and you're done. If it only exposes loose SWD pads (SWDIO,
SWCLK, GND, nRST, 3V3), use a 10-pin-to-flying-leads adapter and match
those pins.

### Power

- **Preferred**: power the SPC_20 from its own supply (battery / panel),
  do **not** also drive `VCC` from the probe.
- **Bench-only**: if the SPC_20 is unpowered, the XDS110 OUT `VCC` pin can
  source 3.3 V to the MCU rail for flashing — fine for the logic side but
  don't expect it to drive buck/LEDs/USB.
- The probe uses `VCC` only as a **target voltage reference** — it must see
  the target rail to translate logic levels, so always have GND + VCC
  connected somehow (either probe-supplied or target-supplied).

---

## 2. Install / verify CCS toolchain

1. Code Composer Studio **v12.4 or newer**.
2. **MSPM0 SDK v2.09.00.01** installed and registered in
   *Window → Preferences → Code Composer Studio → Products*.
3. **TI Arm Clang (TICLANG)** compiler installed.
4. **XDS110 Emulation drivers** (bundled with CCS). Confirm under
   *Help → About → Installation Details → Installed Software* — look for
   "TI Emulators".

Plug the LaunchPad into USB. On Linux, `lsusb` should list
`Texas Instruments XDS110 (...) Embed with CMSIS-DAP`. On Windows,
*Device Manager* shows two ports under "XDS110 Class":

- **XDS110 Class Application/User UART** — the backchannel UART (Section 6).
- **XDS110 Class Auxiliary Data Port** — used internally by the debugger.

---

## 3. Build the SPC_20 firmware

1. *File → Open Projects from File System… → Directory…* and select
   `/home/inka/Desktop/spc20`.
2. Right-click the project → *Build Project*.
3. Output: `Debug/SPC20-Nitu Mario.elf`.

Build errors are almost always a missing SDK path — check
`${COM_TI_MSPM0_SDK_INSTALL_DIR}` in *Project Properties → Resource → Linked
Resources*.

---

## 4. Flash the SPC_20 over XDS110

1. Confirm the active target configuration is **MSPM0G3507**, not the
   on-board L1306. If it doesn't exist:
   *File → New → Target Configuration File*, connection
   *Texas Instruments XDS110 USB Debug Probe*, board *MSPM0G3507*.
2. *Right-click the .ccxml → Set as Active Target Configuration*.
3. *Run → Debug* (green bug icon). CCS will:
   - connect through XDS110 OUT,
   - halt the core,
   - erase + program flash,
   - load symbols and stop at the reset vector.
4. Press **F8** (Resume) to run.
5. To flash without entering a debug session: *Run → Load → Load Program…*,
   pick the .elf, then *Run → Free Run* and *Terminate*.

If CCS reports `Error connecting to the target (-1170 / -151 / -2131)`:

- Verify the **XDS110 OUT** connector is the one you cabled, not XDS110 IN.
- Verify target `VCC` is present at the probe pin (probe needs to see it).
- Try lowering the SWD clock in the .ccxml *Advanced Setup → Target
  Configuration → Connection properties* (e.g. 2 MHz instead of default).
- Hold-then-release nRST while clicking *Connect Target*.

---

## 5. Read UART debug output

The firmware uses **UART0 at 115200 8N1** (from `SPC_20.syscfg`):

- `PA10` — TX (target → probe)
- `PA11` — RX (probe → target; not driven by app today)

`SPCBoardAPI.c::printToUART()` is the blocking transmit helper; the main
loop pushes log lines on the 1 s tick (`TICK_LOG_MS` in `hw_config.h`),
plus the boot banner / fault history added in the diagnostics commit.

### 5a. Wire the backchannel UART to PA10/PA11

The XDS110 OUT 10-pin Cortex connector does **not** carry the backchannel
UART — that's a separate path. The LaunchPad exposes the backchannel UART
on a small 2-pin header (sometimes silkscreened `J22` / `BACKCHANNEL UART`,
varies by revision) on the XDS110 side of the board, with pins labelled
`TXD` and `RXD`.

Run three flying leads from there to the SPC_20:

| LP-MSPM0L1306 (probe side) | SPC_20            |
|----------------------------|-------------------|
| `TXD` (probe TX)           | `PA11` (MCU RX)   |
| `RXD` (probe RX)           | `PA10` (MCU TX)   |
| `GND`                      | any GND pin       |

> Probe `RXD` is what the probe **reads** from the target, so it goes to
> the target's TX (`PA10`). Cross-wire if you only see garbage or silence.

If your LaunchPad revision routes the backchannel UART through the J101
isolation block instead of a dedicated header, the same signals are
available on the probe side of the `RXD`/`TXD` jumpers there — pull those
two jumpers off and tap their probe-side pins.

### 5b. Open the serial terminal in CCS

CCS ships with a built-in terminal:

1. *Window → Show View → Other… → Terminal → Terminal*.
2. Click the monitor icon ("Open a Terminal") in the new view.
3. Choose **Serial Terminal**.
4. Configure:
   - **Serial port**: the XDS110 *Application/User UART* port
     (`/dev/ttyACM0` on Linux, `COMx` on Windows — the two XDS110 ports
     come up as a pair; pick the **Application/User UART**, not the
     Auxiliary Data Port).
   - **Baud Rate**: `115200`
   - **Data**: `8`, **Parity**: `None`, **Stop**: `1`, **Flow control**:
     `None`
5. *OK* — log lines should appear at the 1 s cadence, with a boot banner
   on reset.

You can leave the terminal open while a debug session is active — the
backchannel UART is independent of the SWD link, so breakpoints don't
interrupt the serial stream (though the firmware halt does, since that
stops `printToUART` from running).

### 5c. Alternatives outside CCS

Any serial monitor on the same COM port works:

- Linux: `picocom -b 115200 /dev/ttyACM0`  or  `screen /dev/ttyACM0 115200`.
- Windows: PuTTY, Tera Term.
- VS Code: *Serial Monitor* extension.

---

## 6. Quick checklist

- [ ] LP-MSPM0L1306 connected to PC over USB; `XDS110 Class` ports enumerated.
- [ ] 10-pin Cortex Debug ribbon from **XDS110 OUT** to SPC_20 SWD header.
- [ ] SPC_20 powered (own supply preferred, probe `VCC` only as fallback).
- [ ] CCS active target configuration is **MSPM0G3507**, not L1306.
- [ ] Three flying leads for backchannel UART:
      probe `TXD`→`PA11`, probe `RXD`→`PA10`, GND common.
- [ ] CCS Serial Terminal open on the XDS110 *Application/User UART* port
      at **115200 8N1, no flow control**.
