# Fault Recovery Matrix

`fault_mgr_update()` runs once per 50 ms main tick after measurements,
flags, and power budgeting. Faults latch in `ctx->fault.code`; recovery is
attempted no faster than `FAULT_RECOVER_WAIT_MS` (10 s). Clearing a fault bit
does not re-enable hardware directly. When all live fault bits clear,
`energy_mode_update()` detects the `fault.code != 0 -> 0` falling edge and
re-applies the current energy-mode entry action to re-arm the outputs that are
valid for that state.

`ctx->fault.history` is sticky for the full boot and records every fault that
was ever raised, even after live recovery.

| Fault | Immediate containment | Recovery condition | Recovery behavior |
| --- | --- | --- | --- |
| `FAULT_OVERTEMP` | Disable buck, charge switch, output switch, USB boost, and LED boost. | Battery temperature is below `BAT_TEMP_MAX_DISCHARGE_C - TEMP_HYSTERESIS_C` and board temperature is below `BOARD_TEMP_MAX_C - TEMP_HYSTERESIS_C`. | Auto-clear after the next recovery window once both sensors cool. The current energy state re-arms only the rails it normally owns. |
| `FAULT_BAT_OVERVOLT` | Disable buck and charge switch. | `V_bat < BAT_OVERVOLT_RECOVER_MV`. | Auto-clear after the cell relaxes below the recovery threshold. Charger may restart only if energy mode still calls for charging and no other charge-blocking fault remains. |
| `FAULT_OVERCURRENT_CHG` | Disable buck and charge switch. | `I_charge < BAT_CC_MAX_MA`. | Timed auto-retry. With the charger disabled, current should fall near zero; if the short or control fault remains, the next re-arm trips the fault again and history keeps the trace. |
| `FAULT_OVERCURRENT_DSG` | Disable output switch, USB boost, and LED boost. | `I_discharge < BAT_CC_MAX_MA`. | Timed auto-retry of the load rails through the energy-mode re-arm path. A persistent overload re-latches the fault on the next tick. |
| `FAULT_BAT_UNDERVOLT` | Disable loads, USB boost, LED boost, buck, and charge switch. | `V_bat > BAT_UNDERVOLT_RECOVER_MV` (3200 mV), and the protection wake probe must not be busy (a probe-driven reading is not the cell — see below). | Auto-clear only after the battery is genuinely recovered. Energy mode is **forced** to `EM_SAFE_MODE` while this fault is latched (regardless of sun — the eval guards alone would park a sunlit system in CHARGE_ONLY with dead hardware), and SAFE_MODE's own exit also requires `V_bat > BAT_SAFE_RECOVER_MV` (3200 mV) — so the fault and the mode recover together. The immediate containment disables the buck, so this threshold is only reachable via the **supervised rescue** (`safe_mode_rescue_active()` in `energy_mode.c`): while this is the sole latched fault, there is usable sun, `V_bat >= BAT_RESCUE_MIN_MV` (1500 mV), and no wake probe is busy, SAFE_MODE re-arms the charge path (loads stay shed) and trickle-charges at the precharge rate until the cell climbs to 3200 mV. Below 1500 mV it stays hard-latched — unless the reading is the ≈0.8 V protection-open signature, which the **wake probe** (below) handles. If the cell will not climb past 3000 mV within `BAT_RESCUE_TIMEOUT_MS` (30 min), the charger escalates to `FAULT_PRECHARGE_TIMEOUT` (the damaged-cell terminal state). |
| `FAULT_USB_OVERVOLT` | Disable USB boost. | Both USB output readings are below `FAULT_USB_OVERVOLT_MV`. | Timed auto-retry of USB boost. If the boost remains bad, USB overvoltage immediately re-latches. |
| `FAULT_PRECHARGE_TIMEOUT` | Disable buck and charge switch. | `V_bat > BAT_PRECHARGE_MV`. | External-intervention recovery. The firmware should not keep trickle-charging a cell that stayed below 3.0 V for 15 minutes; the bit clears only if the battery is replaced or externally brought above precharge voltage. |
| `FAULT_TEMP_CHARGE_BLOCK` | Disable buck and charge switch; loads stay available. | `ctx->temp_charge_ok == true` after the charge-temperature hysteresis in `flags_update()`. | Soft charge-only recovery. Charging resumes when the cell returns to the allowed charge-temperature window; discharge/load operation is not blocked by this fault. |
| `FAULT_REVERSE_PUMP` | Open Q49, then disable the buck (that order — isolate the cell before the converter coasts down). | `I_charge > -CHG_REVERSE_CURRENT_MA`, which is trivially true once Q49 is open; `FAULT_RECOVER_WAIT_MS` is the real debounce. | Timed auto-retry, same shape as `FAULT_OVERCURRENT_CHG`. Raised by `charger_fast_guard` **only when the input is still live** — the cell is being pushed back into a working panel, which can drive `V_panel` above open circuit toward the TPS564247 input ceiling. Losing the input is **not** this fault: `charger_input_guard` stands the charger down with no latch (see below). |

### Input loss is not a fault

Until v0.26 `charger_fast_guard` raised `FAULT_OVERCURRENT_CHG` for any reverse
current, so switching the source off mid-charge latched a code reading "charge
over-current" at 664 mA of *forward* current (bench 2026-07-30). Two changes
separate the cases:

* `charger_input_guard` (`charger.c`, foreground) trips when the instantaneous
  `V_panel` falls below `V_bat + CHG_INPUT_LOST_MARGIN_MV` (400 mV) — just above
  the `V_panel ≈ V_bat` crossover where back-feeding begins. The margin is held
  below `PANEL_SAFETY_MV` (4800 mV) on purpose so a sagging-but-present panel
  stays `panel_safety_backoff`'s business and does not get answered with a full
  teardown. It opens Q49, stops the buck, and sets
  `charger.input_lost_pending`; `energy_mode_update` performs the FSM
  transition on the next tick so the `CHG: … -> OFF` line still reaches the log.
  Nothing latches.
* `charger_fast_guard` remains the backstop for the case where the input dies
  faster than one 10 ms conversion. It re-tests input presence and routes a
  dead-input trip to the same clean stand-down; only a live-input trip latches
  `FAULT_REVERSE_PUMP`.

Because the latched fault previously doubled as the charger's only restart
timer, `charger_rearm_due()` (`energy_mode.c`) replaces it: with the mode still
calling for charging, the charger `CHG_INACTIVE`, `fault.code` clean, sun
present, and `CHG_INPUT_REARM_MS` elapsed, the current state's entry actions are
re-applied. Without it a dropout too short to clear `has_sun` (30 ticks / 1.5 s)
would strand the charger inactive indefinitely.

## Protection-Unknown Recovery (Wake Probe)

Distinct from the genuine-undervolt rescue above. When the S-8240 battery
protection IC (U46) opens Q416/Q417 in the battery-negative path (battery
hot-unplug under debug power, deep overdischarge cutout), cell negative is no
longer the ADC ground reference and `V_BATM` reads a protection-biased node at
a stable **≈0.8 V** — identically for "battery absent" and "battery present but
protection open". That signature latches `FAULT_BAT_UNDERVOLT` on an *invalid*
measurement: the fault's own containment then removes the charger stimulus that
is the S-8240's release condition, and the lockout survives even MCU reset.
See `docs/bug_battery_hotplug_800mv_lockout.md` for the full bench analysis.

`bat_wake_tick()` (SAFE_MODE in-state, `energy_mode.c`) recovers this without
ever redefining 0.8 V as a safe cell voltage:

1. **Candidate** (`BATWAKE: MON -> WAKE_PROBE` on UART): `V_bat` stable inside
   `BAT_PROT_SIG_MIN/MAX_MV` (600–1100 mV) for 2 s, undervolt the *sole* fault,
   usable sun, no load, output rail collapsed. Candidacy clears nothing.
2. **Staged wake probe** (≤ 3 s): Q49 stays open while the buck reaches its
   fixed target; only then is the battery connected. The first attempt uses
   3000 mV for a genuinely depleted pack and later attempts use 3650 mV for a
   healthy protection-open pack. A foreground raw-current guard cuts at
   200 mA or on reverse current; abort on sun loss, panel collapse, or any
   additional fault.
3. **Off-state validation** (1.5 s settle + 1 s observe, buck OFF): an on-state
   voltage proves nothing (the buck drives an empty connector to the commanded
   voltage). The off-state minimum decides: **VALIDATED** (≥ 1500 mV — the
   measurement is real again; at/above 2000 mV it clears the synthetic
   undervoltage latch and hands off to normal staged PRECHARGE, while
   1500–1999 mV retains the supervised SAFE rescue), **NO_BATTERY** (collapsed
   back to the signature — rate-limited retry,
   max 3 attempts / 60 s apart, then terminal), or **WAKE_FAILED** (persisted
   below the rescue floor — a genuinely deep cell; fault retained, no
   unattended charging).

While the probe is busy, the `V_bat` reading is untrusted by construction, so
the undervolt recovery row above, the SAFE_MODE exit comparison, and the rescue
gate are all held (`bat_wake_probe_busy()`).

## Non-recoverable By Fault Manager

CPU HardFaults, default interrupt-vector traps, corrupt control flow, and
hardware failures that keep measurements outside their recovery thresholds are
not recoverable by `fault_mgr_update()`. The existing HardFault handler logs a
post-mortem over UART; returning to normal operation after that requires reset
or a firmware-level watchdog policy outside this fault bitmask.
