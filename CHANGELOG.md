# [v0.05] - 15.04.26
## Implement fault manager module (pipeline step 4)

New files: `fault_mgr.h`, `fault_mgr.c`

### NEW: fault_mgr_update() — pipeline step 4
- Runs every TICK_MAIN_MS between `power_budget_update()` and `energy_mode_update()`
- Two-phase: detection pass raises/latches faults; recovery pass throttled to `FAULT_RECOVER_WAIT_MS` (10 s) clears latched bits whose recovery condition holds
- Detection covers all 8 fault bits already defined in `system_types.h`:
  - `FAULT_OVERTEMP`: `bat_temp > 60°C` or `board_temp > 60°C`
  - `FAULT_BAT_OVERVOLT`: `V_bat > 3700 mV`
  - `FAULT_BAT_UNDERVOLT`: `V_bat < 2000 mV` — gated by a 500 mV floor so a disconnected battery on boot doesn't false-trip
  - `FAULT_OVERCURRENT_CHG`: `I_chg > 2200 mA`
  - `FAULT_OVERCURRENT_DSG`: `I_dsg > 5000 mA`
  - `FAULT_USB_OVERVOLT`: either USB output > 6000 mV
  - `FAULT_TEMP_CHARGE_BLOCK`: mirror of `!temp_charge_ok` (hysteresis already applied in `flags_update()`)
- `FAULT_PRECHARGE_TIMEOUT` is **not** detected here — it is raised externally by `charger_update()` via `fault_raise()`
- Recovery thresholds use the `*_RECOVER_MV` constants from `hw_config.h` (e.g., overvolt trips at 3700, clears at 3400 — 300 mV hysteresis)

### NEW: fault_raise() helper — public
- Idempotent: re-raising a latched fault is a no-op
- Applies immediate protective hardware action on first raise, scoped per fault class:
  - Charge-side faults (overvolt, chg overcurrent, precharge timeout, temp-charge-block) → disable buck + charge switch
  - Discharge-side (dsg overcurrent) → disable output switch + USB boost
  - Overtemp → disable buck + charger + output + USB (full shutdown)
  - USB overvolt → disable USB boost only
  - Undervolt → disable loads AND charger (prevents further drain + prevents re-triggering overvolt during recovery ramp)
- Exposed in the header so `charger_update()` can signal `FAULT_PRECHARGE_TIMEOUT` when the 15 min precharge timer expires

### Design note
- `energy_mode_update()` does not yet consult `ctx->fault.code`. The immediate hardware shutdown in `fault_take_action()` contains the fault for the current tick, but energy_mode's next-tick entry actions may re-enable switches. A follow-up change should gate energy_mode transitions on `ctx->fault.active` (likely forcing `EM_IDLE` or `EM_SAFE_MODE` when a hard fault is latched).

---

# [v0.04] - 27.03.26
## Implement power budget module (pipeline step 3)

New files: `power_budget.h`, `power_budget.c`

### NEW: power_budget_update() — pipeline step 3
- Computes `i_buck_max = MIN(BUCK_MAX_CURRENT_MA, mppt_limit_ma)`
- Computes `allowed_chg = i_buck_max - I_load`, clamped to `[0, bat_limit]`
- Three battery voltage zones for `bat_limit`:
  - V_bat < 3000 mV → 200 mA (precharge, gentle trickle)
  - V_bat < 3650 mV → 2000 mA (CC, normal bulk charge)
  - V_bat >= 3650 mV → 200 mA (near-full safety cap, defense-in-depth against one-tick CC overshoot before charger transitions to CV)
- Signed arithmetic prevents unsigned underflow when loads exceed buck capacity

---

# [v0.03] - 20.03.26
## Implement measurements module (pipeline steps 1 & 2)

New files: `measurements.h`, `measurements.c`

### NEW: measurements_update() — pipeline step 1
- Reads all ADC channels via HAL `get_*()` functions into `ctx->meas`
- Clamps `panel_current` to 0 if HAL returns negative (sensor noise protection)
- Computes derived values: `panel_power` (mW), `i_bat_net` (signed mA)

### NEW: flags_update() — pipeline step 2
- `bat_low`: debounced (3-count) + hysteresis (2800/2900 mV) — unchanged from prior design
- `has_sun`: debounced (3-count) + hysteresis (9000/8000 mV) — **upgraded from plain bool** to `debounce_flag_t` to filter cloud transients
- `has_load`: hysteresis only (50/30 mA) — load events are clean, no debounce needed
- `panel_limited`: guards against unsigned underflow when `allowed_chg < PANEL_LIMITED_MARGIN_MA`
- `temp_charge_ok`: 10°C hysteresis on recovery (blocked at 0/45°C, resumes at 10/35°C)
- `bat_full`: not touched — owned by charger module

### NEW: debounce_update() helper — static in measurements.c
- Generic debounce logic for any `debounce_flag_t`: counts consecutive set-condition readings, clears on clear-condition

### Changed: system_types.h
- `has_sun` (plain `bool`) → `flag_has_sun` (`debounce_flag_t`) — state machines will read `ctx->flag_has_sun.value`
- `ctx_init()`: added `flag_has_sun` debounce configuration (count_threshold = 3)

### Changed: hw_config.h
- Added `HAS_SUN_DEBOUNCE_COUNT` (3) — consecutive readings before `has_sun` sets (×50 ms = 150 ms)

---

# [v0.02] - 20.03.26
## SPCBoardAPI refactor — HAL/application split

Stripped application logic from SPCBoardAPI so it is a pure HAL.
All removed items are either replaced by the new module architecture
(energy_mode, charger, mppt, UI_MGR) or are pending move there.

### Removed — application logic that doesn't belong in HAL
- `power_manager()` — replaced by energy_mode + charger + power_budget
- `apply_mppt_perturb_observe_step()` and `MpptStep` enum — replaced by mppt module
- `handle_button_input()` — button reading stays in HAL; action policy moves to UI_MGR
- `handle_ir()`, `receive_command()`, `get_command()` — IR command interpretation is application logic
- `receive_command_sw()`, `on_received_edge()`, `get_decoded_value()`, `get_msg_led()`, `l_sw_init()`, `reset_sw_receive()` — entire light switch decoder removed
- `handle_uart()`, `UARTReceive()`, `get_UART_buffer()` — UART receive/parse is application logic; `printToUART()` kept
- `turn_on_outputs()`, `turn_off_outputs()` — energy_mode calls individual enable/disable functions directly
- `startup_safe_connect()` — replaced by new init sequence in main
- `get_system_state()` — replaced by ctx_print() style logging from system_ctx_t

### Removed — toggle functions
- `toggle_charger_switch()`, `toggle_battery_switch()`, `toggle_output_switch()`, `toggle_usb()`, `toggle_buck()`, `toggle_led()` — dangerous in a state machine; state machine uses explicit enable/disable so switch state is always known

### Removed — _log measurement variants
- `get_charge_current_log()`, `get_battery_voltage_log()`, and all other `_log` getters — logging reads `ctx->meas` directly; no need for a parallel getter set
- Internal `buffer_log`, `sum_log`, `avg_readings_log`, `add_sample_log()`, `get_average_log()` also removed

### Removed — display policy functions
- `display_time()`, `display_error_fault()`, `display_ovp_fault()`, `display_ocp_fault()`, `displayCurrentPower()`, `displayChargeStorage()` — what to show is UI_MGR policy; HAL keeps `update_led_bar()` and `update_seven_segment_display()` as raw primitives
- `update_led_display()` mux timing kept; policy calls replaced with `/* UI_MGR sets content here */` comment

### Changed — return types
- `get_temperature()`: `float` → `int16_t` (°C truncated). Thermistor math unchanged internally.
- `get_charge_current()`: `int32_t` → `int16_t`. Max range is ~±2500 mA; int16_t (±32767) is sufficient and avoids wasting a register pair on M0+.

---

# [v0.01] - 18.03.26
## format change for tables

 - added priority column: integer per source state, evaluated as strict if/else chains (lower - first checked, higher - checked later)
 - added NOTES column for implementation instructions
 - added explicit '<default>' stay-in-state row per source state to make it clear that no unhandeled condition exists

 ---

 ### BUG FIX: DISCHARGE_ONLY → SAFE_MODE (critical)
- **Old:** `bat_low AND !has_load AND !has_sun → SAFE_MODE`
- **Problem:** The most dangerous case — `bat_low AND has_load AND !has_sun` — had NO transition to SAFE_MODE. Battery drains to damage under load.
- **New:** `bat_low AND has_load → SAFE_MODE` (priority 1). Added `bat_low AND !has_load → IDLE` (priority 2) since with no load there's nothing to shed.


### BUG FIX: SAFE_MODE missing exit for !has_sun AND has_load AND V_bat recovered (critical)
- **Old:** No transition existed for `V_bat > SAFE_RECOVER_MV AND !has_sun AND has_load`.
- **Problem:** If battery recovers above 3200mV overnight (no sun, load still physically connected), there was no exit from SAFE_MODE. System would stay locked in SAFE_MODE with loads shed indefinitely even though the battery is healthy and able to supply the load.
- **New:** Added `V_bat > SAFE_RECOVER_MV AND !has_sun AND has_load → DISCHARGE_ONLY` (priority 3). Existing `!has_sun AND !has_load → IDLE` shifted to priority 4; `<default>` shifted to priority 5.

### BUG FIX: SAFE_MODE oscillation (medium)
- **Old:** `!has_load → IDLE` was an exit from SAFE_MODE.
- **Problem:** SAFE_MODE sheds loads (disables OUTPUT_EN/USB_EN) → has_load drops to false → exits to IDLE → load device still physically connected → re-enables outputs → current flows → bat_low → SAFE_MODE → repeat. Oscillation.
- **New:** Removed `!has_load → IDLE`. ALL SAFE_MODE exits now require `V_bat > SAFE_RECOVER_MV (3200mV)`. System stays in SAFE_MODE with loads shed until battery genuinely recovers (e.g., sun returns and charges it above 3200mV).

### FIX: CHARGE_ONLY overlapping guards
- **Old:** `!has_sun OR bat_full → IDLE` was a single disjunctive guard.
- **Problem:** Evaluated as one transition, but masks the priority between "sun lost" (urgent, stop buck) vs "battery full" (graceful, just stop pushing current). Different exit actions may be needed.
- **New:** Split into two separate rows with explicit priority. `!has_sun → IDLE` (priority 2), `bat_full → IDLE` (priority 3). `has_load → CHARGE_AND_LOAD` remains priority 1.
 
### FIX: IDLE missing path for has_load AND bat_low AND !has_sun
- **Old:** `!has_sun AND has_load AND !bat_low → DISCHARGE_ONLY`. If bat_low was true, no transition matched.
- **New:** Added `!has_sun AND has_load AND bat_low → SAFE_MODE` (priority 4).
 
### FIX: CHARGE_AND_LOAD guard priority
- **Old:** Five exit guards with no defined order. `!has_sun AND bat_low` and `bat_full AND has_load` could overlap in edge cases.
- **New:** Explicit priority ordering. Safety-critical `!has_sun AND bat_low → SAFE_MODE` is priority 1.



---
## MPPT transition table
 
### BUG FIX: HOLD → DISABLED guard (critical typo)
- **Old:** `!panel_limited AND panel_limited → DISABLED` — logical contradiction, always false. This transition could never fire.
- **New:** `!panel_limited → DISABLED`.
 
### FIX: HOLD → TRACKING missing has_sun guard
- **Old:** `hold_time expired AND panel_limited → TRACKING`
- **Problem:** Could re-enter TRACKING after sun dropped if panel_limited flag was stale.
- **New:** `hold_time expired AND panel_limited AND has_sun → TRACKING`
 
### FIX: HOLD missing !has_sun → DISABLED
- **Old:** No explicit sun-loss transition from HOLD.
- **New:** `!has_sun → DISABLED` (priority 3). Consistent with TRACKING behavior.
 
---
 
## CHARGER transition table
 
### NEW: Explicit INACTIVE state and deactivation transitions
- **Old:** "Activated/deactivated by ENERGY MODE" with no defined behavior.
- **New:** Added INACTIVE state. Every active state (PRECHARGE, CC, CV) has a `deactivated → INACTIVE` transition with mandatory exit action: `pwm = PWM_MIN_DUTY (399), assert BUCK_DIS, reset charger state`.
- This prevents the buck from free-running after charger deactivation.
 
### NEW: PRECHARGE timeout → FAULT
- **Old:** "15min timeout failure" — destination unspecified.
- **New:** Explicit `timeout → FAULT` transition. A battery that can't exit precharge in 15 minutes is likely damaged or disconnected.

