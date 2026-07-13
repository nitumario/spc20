# Bug: Battery hot-plug locks at approximately 0.8 V and cannot recover from solar

## Summary

When the battery is removed while the MCU remains powered through the debug
interface, `Vbat` falls from the real cell voltage and settles near 0.8 V. If the
battery is then reconnected, `Vbat` remains near 0.8 V. Connecting a valid solar
panel does not recover the system: the panel is detected, but the charger and
MPPT stay off indefinitely.

This is a firmware-visible consequence of the battery protection circuit, not a
stalled ADC or UART logger. Q416/Q417 can disconnect the cell negative terminal
from system ground. While they are open, the single-ended `V_BATM` ADC channel
does not measure cell voltage; it measures an undefined/protection-biased node
relative to system ground. The observed approximately 0.8 V is therefore not
evidence that the cell itself is at 0.8 V.

Firmware interprets that invalid measurement as a genuine battery undervoltage,
latches `FAULT_BAT_UNDERVOLT`, enters `EM_SAFE_MODE`, and disables the only
charging path capable of waking the protection IC. The result is a persistent
self-lock.

**Severity:** High

**Impact:** A healthy battery hot-plugged into a debug-powered controller may be
unusable until reset or until an external charger independently releases the
protection circuit. Solar input alone cannot recover it with the current
firmware.

**Hardware constraint:** The PCB cannot be changed. Recovery must be implemented
in firmware using the existing buck, charge switch, current sensing, and ADC
channels.

## Evidence

### Battery removal

The captured log is in
[`serial_20260712_215354.log`](../serial_20260712_215354.log). Immediately before
removal, battery and output voltage are valid:

```text
ms:3427267 Vbat:3324 Vout:3320 ... EM:IDLE CHG:OFF ... fault:0000
```

After removal, the output rails collapse, the battery reading settles near
0.8 V, and the undervoltage fault latches:

```text
EM: IDLE -> SAFE @ 3427667 ms
ms:3428274 Vbat:744 Vout:476 ... EM:SAFE CHG:OFF ... fault:0010
ms:3429267 Vbat:834 Vout:144 ... EM:SAFE CHG:OFF ... fault:0010
ms:3430271 Vbat:830 Vout:50  ... EM:SAFE CHG:OFF ... fault:0010
ms:3431274 Vbat:852 Vout:22  ... EM:SAFE CHG:OFF ... fault:0010
```

The changing `Vbat` values and decaying output rails show that conversions and
logging continue. This is not a cached 3.3 V sample.

### Panel connection during the lockout

After the battery is reconnected and a panel is applied, the panel is correctly
detected but no power is transferred:

```text
ms:3694084 Vbat:896 Vpanel:12956 Ipanel:4 Ppanel:51
           has_sun:1 allowed_chg:200
           EM:SAFE CHG:OFF MPPT:OFF pwm:399 fault:0010
```

`Vpanel` near 13 V and `has_sun:1` prove that panel detection works. `Ipanel`
near zero, `CHG:OFF`, `MPPT:OFF`, and `pwm:399` prove that the buck is not being
used. Connecting the panel is therefore not equivalent to applying a charger to
the protected battery terminals.

## Electrical cause

On Charger schematic sheet 4 in
[`Copy of SPC_20 - R2 Production - Schematic Prints.PDF`](Copy%20of%20SPC_20%20-%20R2%20Production%20-%20Schematic%20Prints.PDF):

- U46 is `S-8240ADC-I6T1U`.
- Q416 and Q417 are back-to-back protection MOSFETs in the battery-negative
  path.
- MCU/system ground is on the protected pack side of those MOSFETs.
- `V_BATM` measures battery positive relative to MCU/system ground through Q412
  and the R451/R452 divider.

When Q416/Q417 are open, cell negative is no longer the ADC reference. The MCU
cannot infer true cell voltage from `V_BATM` in this state.

The selected S-8240 variant has the power-down function available and uses
charger connection as the release condition for discharge-overcurrent status.
The manufacturer's operating description also cautions that immediately after
battery connection, discharge may not be possible and normal status may require
connecting a charger. See the
[official S-8240A datasheet](https://www.ablic.com/en/doc/datasheet/battery_protection/S8240A_E.pdf),
particularly the product-options table and the Normal/Overdischarge operation
sections.

The approximately 0.8 V reading is consistent with this protection-open state;
it must not be treated as a trustworthy cell-terminal measurement.

## Firmware causes that make the condition permanent

### 1. The protection-open signature is classified as real undervoltage

`fault_mgr.c` raises `FAULT_BAT_UNDERVOLT` for every reading between 500 mV and
2000 mV. The approximately 800 to 900 mV protection-open signature is therefore
guaranteed to latch the fault.

The existing 500 mV disconnected-battery guard assumes a missing or isolated
battery reads near zero. Bench evidence disproves that assumption.

### 2. SAFE mode removes the charger needed for protection release

On entry to `EM_SAFE_MODE`, firmware disables the buck and charge switch. The
panel can subsequently reach `has_sun:1`, but there is no state transition or
in-state action that applies charging voltage while `Vbat` remains below the
rescue floor.

### 3. Existing supervised rescue excludes the observed voltage

`safe_mode_rescue_active()` requires:

```text
Vbat >= BAT_RESCUE_MIN_MV (1500 mV)
```

The observed stable range is approximately 830 to 900 mV, so the rescue cannot
start.

### 4. Lowering the rescue floor alone would still not wake the battery

`activate_charger_region()` initializes the buck from:

```text
measured Vbat + CHG_ACTIVATION_HEADROOM_MV
```

With a false 0.8 to 0.9 V measurement, `set_charging_voltage()` selects the
lowest available calibrated LUT voltage, approximately 2.786 V. A healthy
LiFePO4 cell near 3.3 V will not see forward charger bias at that target, so the
S-8240 may remain in its protected state.

Consequently, changing `BAT_RESCUE_MIN_MV` from 1500 mV to 500 or 800 mV is not
a complete fix and would also remove the intentional safety floor for a truly
deeply discharged cell.

### 5. Periodically pulsing `VBATM_EN` addresses the wrong mechanism

Q412/Q415 form a level-controlled measurement switch. They do not contain a
latch. Pulsing `VBATM_EN` cannot reconnect cell negative through Q416/Q417 and
cannot make a single-ended ADC measurement valid while the protection FETs are
open.

## Recommended firmware-only fix

Add a short, energy-limited **protection wake probe** that is separate from the
normal charger FSM and from deep-cell rescue.

This is preferable to weakening the undervoltage rules because firmware cannot
distinguish a missing battery, an open protection circuit, and a genuinely
damaged low-voltage cell from the initial `V_BATM` reading alone.

### Candidate detection

Treat the following as `BAT_PROTECTION_UNKNOWN`, not confirmed cell
undervoltage:

- `Vbat` remains in a bench-derived signature window around 0.8 V for multiple
  consecutive samples. An initial candidate window of 600 to 1100 mV is
  reasonable but must be validated across boards and temperature.
- The output rail has collapsed and no load is enabled.
- The only live battery-related fault is `FAULT_BAT_UNDERVOLT`; no overtemperature,
  charge-overcurrent, battery-overvoltage, precharge-timeout, or temperature
  charge-block fault is present.
- A panel is present at a valid open-circuit voltage.

A disconnected battery can produce the same signature, so candidate detection
must not by itself declare a battery present or clear any fault.

### Wake-probe sequence

1. Keep output, USB, and LED loads disabled.
2. Keep the battery power-path switch enabled.
3. Start from buck-off and enable the charge path under a dedicated probe state.
4. Use a fixed, bounded charger target independent of the invalid `Vbat`
   measurement. A target no higher than the normal 3650 mV LiFePO4 CV limit is
   appropriate; do not use `Vbat + headroom` for this probe.
5. Enforce the existing precharge current ceiling of 200 mA, with a lower probe
   ceiling if regulation and current-sensor resolution permit it.
6. Limit the first probe to a few seconds. A healthy cell whose protection IC
   merely needs charger detection should reconnect promptly. Do not use this
   probe to raise a genuinely deeply discharged cell through the S-8240 release
   threshold.
7. Abort immediately on panel collapse, overcurrent, overtemperature, battery
   overvoltage, invalid current direction, or loss of the input source.

The probe needs its own fixed initial PWM derived from the 3.65 V LUT entry. It
must not call the normal activation path until the battery measurement has been
validated.

### Presence and success validation

Rising `Vbat` while the buck is on is not proof that a battery exists: with no
battery, the buck can charge the open terminal and the ADC can report the
commanded voltage.

Validate success using an off-state persistence test:

1. During the probe, require either evidence of bounded positive charge current
   or a rapid transition from the 0.8 V signature to a plausible cell voltage.
2. Turn the buck and charge switch off.
3. Allow the analog path and moving average to settle, or use fresh raw samples
   explicitly collected after switch-off.
4. Confirm that `Vbat` remains at a plausible cell voltage while unpowered by
   the buck.

If voltage collapses back to the 0.8 V signature, classify the result as no
battery or protection still open. Keep loads off, retain the fault, and schedule
only a rate-limited retry while the panel remains available.

If voltage persists:

- The protection path is connected and `Vbat` is valid again.
- If `Vbat >= 3200 mV`, allow the existing fault recovery and SAFE-mode exit
  conditions to run.
- If `1500 mV <= Vbat < 3200 mV`, hand off to the existing supervised rescue and
  normal precharge timeout.
- If the validated off-state voltage is below the existing hard safety floor,
  stop and retain a terminal/manual-intervention fault. Do not continue an
  unattended charge.

### Retry policy

- Retry only while a valid panel source is present.
- Use a bounded attempt count and a long retry interval, such as one attempt per
  panel relock interval, to prevent continuous pulsing into an empty connector.
- Reset the attempt budget after a confirmed battery connection, panel removal,
  or system reset.
- Emit explicit UART events for candidate detection, probe start, validation,
  success, no-battery collapse, and terminal failure.

## Why this is the safest available software-only approach

- It does not redefine 0.8 V as a safe cell voltage.
- It does not continuously charge a cell whose true voltage is unknowable.
- Voltage and energy are bounded by the existing LiFePO4 charge limits.
- Loads remain shed throughout recovery.
- A no-battery condition is detected by removing the stimulus and observing
  voltage collapse.
- Once the protection path reconnects, all existing ADC-based safety and charger
  logic become valid again.

## Rejected fixes

### Lower `BAT_RESCUE_MIN_MV` to approximately 800 mV

Rejected as a standalone fix. It conflates a protected/missing battery with a
genuinely damaged cell, and normal charger activation would still select an
insufficient approximately 2.786 V initial buck target.

### Clear `FAULT_BAT_UNDERVOLT` when the panel appears

Rejected. Clearing the fault does not establish a valid battery reference and
could restore loads onto an absent or isolated battery.

### Keep the buck enabled continuously in SAFE mode

Rejected. This can energize an empty battery connector indefinitely and removes
the bounded-energy property required when true cell voltage is unknown.

### Continue pulsing `VBATM_EN`

Rejected as a recovery mechanism. It only controls the measurement divider and
does not affect the S-8240 protection MOSFETs.

### Trust charge-on voltage as proof of a battery

Rejected. The buck itself can create that voltage with no battery. Off-state
persistence is required.

## Acceptance criteria

The fix is complete only when all of the following pass on hardware:

1. **Hot-plug recovery with debug power:** Boot from the debug interface without
   a battery, connect a healthy battery, then connect the panel. The system
   performs one bounded wake probe, validates the battery with the buck off, and
   reports the real cell voltage.
2. **Battery-only cold boot:** A healthy connected battery boots without a false
   permanent undervoltage lockout.
3. **Removal detection:** Removing a previously valid battery causes loads to be
   shed and does not retain a false valid-battery state.
4. **No-battery probe:** With the panel present and no battery installed, the
   connector is energized only for the bounded probe, off-state voltage
   collapses, loads remain disabled, and retries are rate-limited.
5. **Deeply discharged-cell safety:** A test source representing a cell below the
   hard rescue floor does not receive sustained unattended charging after the
   short wake probe.
6. **Normal charging regression:** Charging, MPPT, CV taper, and existing fault
   containment remain unchanged after a validated battery connection.
7. **Weak-panel behavior:** A weak panel that cannot support the probe causes an
   immediate safe abort without repeated rapid cycling.
8. **Fault interaction:** Temperature, overvoltage, charge-overcurrent, and
   precharge-timeout faults always block or terminate the probe.
9. **Telemetry:** Logs distinguish `PROTECTION_UNKNOWN`, `WAKE_PROBE`,
   `VALIDATED`, `NO_BATTERY`, and `WAKE_FAILED`; they never label the initial
   approximately 0.8 V value as confirmed cell voltage.

## Files expected to change when implementing

- `system_types.h`: protection-wake state and timing fields.
- `hw_config.h`: candidate band, fixed probe target, current/timeout/retry
  limits.
- `fault_mgr.c`: distinguish untrusted protection-open measurement from
  validated battery undervoltage.
- `energy_mode.c` / `energy_mode.h`: own the SAFE-mode wake-probe sequence and
  hardware enables.
- `charger.c`: prevent the normal `Vbat + headroom` activation path from being
  used until battery voltage is validated; hand off after validation.
- `measurements.c` or the HAL measurement layer: provide fresh post-switch-off
  samples or reset/qualify the moving-average window for validation.
- `main.c`: diagnostic events and possibly sequencing support.
- `docs/fault_recovery.md`: document protection-unknown recovery separately
  from genuine undervoltage rescue.
