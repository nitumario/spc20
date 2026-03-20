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

