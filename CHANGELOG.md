# [v0.11] - 17.04.26
## Stub unimplemented HAL button/display functions

Changed files: `SPCBoardAPI.c`

### FIX: link errors on buttons_init / led_display_init / get_button_state
- `SPCBoardAPI.h` declared these plus `extern volatile bool check_buttons`, but none were defined in `SPCBoardAPI.c`. `main.c:171` calls `buttons_init()` → unresolved symbol at link time, no `.elf` produced.
- Added empty stubs for `buttons_init()` and `led_display_init()`, a `get_button_state()` that returns `false`, and a definition `volatile bool check_buttons = false`.
- Buttons/display are intentionally inert for now; `update_buttons()` and `update_led_display()` were already implemented and remain functional if called.

---

# [v0.10] - 16.04.26
## Implement full main loop with deterministic pipeline

Changed files: `main.c` (rewritten), `SPCBoardAPI.h`, `SPCBoardAPI.c`

### REWRITE: main.c — bringup stub → production pipeline
- Replaces the v0.05 bringup version (measurements + UART only) with the full 8-step deterministic pipeline
- All modules wired in strict order: `measurements → flags → power_budget → fault_mgr → energy_mode → mppt → charger → apply_pwm`
- Three independent tick rates in the super-loop:
  - 20 ms: button polling (`update_buttons()`)
  - 50 ms: deterministic pipeline (steps 1–8)
  - 1000 ms: UART diagnostic logging

### NEW: apply_pwm() — pipeline step 8
- The **only** function that touches the buck timer register
- Reads `ctx->pwm` (written by charger or MPPT in steps 6/7) and commits to hardware via `set_buck_pwm()`
- Defense-in-depth clamping to `[PWM_MAX_DUTY=1, PWM_MIN_DUTY=399]` — prevents inductor saturation (0) or counter overflow (400) even if upstream has a bug

### NEW: SysTick_Handler — 1 ms ISR
- Increments millisecond timestamp via `update_timestamp()`
- Kicks ADC conversions every `TICK_ADC_MS` (10 ms): `read_adc_values()` harvests completed conversions, `adc_read_step()` starts next pair

### NEW: SYS_INIT → SYS_RUN sequence
- `system_init()` → `timer_init()` → `buttons_init()` → `ctx_init()`
- Enables battery switch early so V_bat is available from the first ADC sample
- Transitions to `SYS_RUN`, sends UART header, enters super-loop

### NEW: idle sleep support
- When `energy_mode` signals `idle_sleep_pending`, main loop disables LED bar and enters `__WFI()` low-power stop mode
- On wake (GPIO or RTC interrupt), clears pending flag, resets idle timer, and resumes pipeline

### UPGRADED: UART logging
- Now logs full system state: all measurements, all flags, energy mode / charger / MPPT state names, power budget outputs, PWM, and fault code
- Tab-separated format preserved for spreadsheet compatibility

### NEW: set_buck_pwm() — HAL function (SPCBoardAPI)
- Thin wrapper: writes a raw PWM value `[1..399]` to the buck converter channel via `set_pwm_duty_cycle(&_pwm_outputs[0], ...)`
- Single point of contact between the pipeline and the buck timer — `apply_pwm()` calls only this function

---

# [v0.09] - 15.04.26
## Implement charger module (pipeline step 7)

New files: `charger.h`, `charger.c`

### NEW: charger_update() — pipeline step 7
- Runs after `mppt_update()` and before `apply_pwm()` so the charger can yield PWM control when MPPT is TRACKING, and its regulation output is what ends up in the timer
- Four-state machine per `docs/charger_states.csv`: `CHG_INACTIVE`, `CHG_PRECHARGE`, `CHG_CC`, `CHG_CV`
- Activation gated by `energy_mode ∈ {EM_CHARGE_ONLY, EM_CHARGE_AND_LOAD}` AND no charge-blocking fault latched; otherwise forced to `CHG_INACTIVE` with `pwm = PWM_MIN_DUTY`

### Self-activation from INACTIVE
- `energy_mode` enables buck + charge switch and leaves `state = CHG_INACTIVE`. The charger picks:
  - `V_bat < 3000 mV` → `CHG_PRECHARGE`
  - `V_bat ≥ 3000 mV` → `CHG_CC`
- Falls through to run a regulation tick in the new state — no wasted tick
- PWM is **not** reset on state entry (per README contract): PRECHARGE → CC → CV hand off the PWM value

### Transitions (per charger_states.csv)
- `PRECHARGE → CC` on `V_bat ≥ BAT_PRECHARGE_MV` (3000 mV)
- `PRECHARGE → FAULT` on 15-minute timeout (`BAT_PRECHARGE_TIMEOUT_MS`) — raises `FAULT_PRECHARGE_TIMEOUT` via `fault_raise()` and parks pwm at `PWM_MIN_DUTY`
- `CC → CV` on `V_bat ≥ BAT_CV_VOLTAGE_MV` (3650 mV) — falls through to CV regulation on the same tick
- `CV → bat_full` when `I_charge < BAT_CV_TAPER_MA` (200 mA) held **continuously** for `BAT_FULL_HOLD_MS` (30 s). Any sample above taper resets the window. Sets `ctx->bat_full = true` which `energy_mode` consumes to exit charging

### Regulation (bang-bang with deadband, identical order across all active states)
1. **Panel safety** (runs first): `V_panel < PANEL_SAFETY_MV` (10 V) → `pwm += PANEL_BACKOFF_STEP` (5), skip remaining regulation. Runs **before** the MPPT gate so the panel is protected even during MPPT perturbations that caused the collapse
2. **MPPT gate**: if `ctx->mppt.state == MPPT_TRACKING`, skip regulation. Transition guards and CV taper tracking still run — they observe measurements regardless of who owns PWM
3. **Bang-bang regulator**:
   - CC/PRECHARGE: target = `ctx->allowed_chg`, deadband `±CC_DEADBAND_MA` (25 mA). Outside deadband, step pwm by `CC_PWM_STEP` (1)
   - CV: target window `[BAT_CV_VOLTAGE_MV, BAT_CV_VOLTAGE_MV + CV_DEADBAND_MV]` (3650..3655 mV), step pwm by `CV_PWM_STEP` (1)

### PRECHARGE target derivation
- No separate 200 mA clamp in the charger — `power_budget_update()` already clamps `allowed_chg` to `BAT_PRECHARGE_MAX_MA` (200 mA) when `V_bat < 3000 mV`. CC regulation is therefore reused directly; "precharge-ness" is enforced upstream

### Fault interaction
- Charge-blocking fault mask: `FAULT_OVERTEMP | FAULT_BAT_OVERVOLT | FAULT_OVERCURRENT_CHG | FAULT_PRECHARGE_TIMEOUT | FAULT_TEMP_CHARGE_BLOCK`
- When any is latched, the charger deactivates but does **not** touch GPIO switches — `fault_mgr` already took the hardware action, `energy_mode` owns the enables. Avoids double-ownership of hardware state
- `FAULT_BAT_UNDERVOLT` deliberately excluded: by the time it trips, `energy_mode` has already moved to `EM_SAFE_MODE` which deactivates the charger path

### PWM sign convention (codified in helpers)
- `pwm_clamp()` enforces range `[PWM_MAX_DUTY=1, PWM_MIN_DUTY=399]`
- `pwm_step(delta)` is the only mutation path: `delta < 0` → higher duty → more current; `delta > 0` → lower duty → less current
- Comments repeat the convention on every regulation step so a future edit can't silently flip a sign

---

# [v0.08] - 15.04.26
## Implement MPPT module (pipeline step 6)

New files: `mppt.h`, `mppt.c`

### NEW: mppt_update() — pipeline step 6
- Runs after `energy_mode_update()` and before `charger_update()` so the charger can see whether MPPT currently owns PWM
- Three-state machine per `docs/MPPT_transition_table.csv`: `DISABLED`, `TRACKING`, `HOLD`
- Activation gated by the charger region being active (`EM_CHARGE_ONLY` or `EM_CHARGE_AND_LOAD`); otherwise forced to `DISABLED` with `mppt_limit_ma = BUCK_MAX_CURRENT_MA` (no panel constraint)

### Algorithm: incremental conductance, integer math only
- Decision quantity: `X = dI·V + I·dV`, signed by `dV` to recover `sign(dP/dV)`:
  - `X > 0` → left of MPP → raise V → `pwm += step` (lower duty)
  - `X < 0` → right of MPP → lower V → `pwm -= step` (higher duty)
  - `X == 0` or `dV == 0 && dI == 0` → hold last direction
- Edge case `dV == 0, dI != 0`: direction chosen from sign of `dI` (V stuck, I moving means we're on a flat stretch)
- Overflow safe on int32: max term `15000 mV × 2000 mA = 3e7` ≪ 2.1e9
- PWM convention explicit in code comments: lower pwm value = higher duty = more current drawn = V_panel falls

### Adaptive step size
- Starts at `MPPT_MAX_STEP_SIZE` (8), halved on every direction reversal down to `MPPT_MIN_STEP_SIZE` (1)
- Convergence: `step_size == 1 AND reversals >= MPPT_CONVERGE_REVERSALS` (6) → transition to HOLD
- Runtime safety: `tracking_start_ms` timer forces HOLD after `MPPT_RUNTIME_MS` (300 ms) even without convergence — handles fast-changing irradiance

### TRACKING entry actions (matches README spec)
- `V_prev = V_panel`, `I_prev = I_panel`, `step_size = MAX`, `reversals = 0`, `max_power = 0`, `last_direction = -1`
- Forces first perturbation `pwm -= step_size` so next tick has a valid `dV`/`dI`
- Records `tracking_start_ms`

### HOLD entry actions
- Parks `ctx->pwm = max_power_pwm` (best operating point seen during the session)
- Publishes `mppt_limit_ma = (max_power_mW × 1000) / V_bat_mV`, clamped to `[0, BUCK_MAX_CURRENT_MA]`. This is what `power_budget_update()` consumes next tick to cap `i_buck_max`
- `V_bat` floored at 1000 mV to avoid a divide-by-near-zero on a missing/disconnected battery

### Transitions implemented (per MPPT_transition_table.csv)
- `DISABLED → TRACKING` on `panel_limited AND has_sun`
- `TRACKING → DISABLED` on `!has_sun` (safety-first, priority 1)
- `TRACKING → HOLD` on convergence or runtime timeout
- `HOLD → DISABLED` on `!panel_limited` or `!has_sun`
- `HOLD → TRACKING` on `hold_time expired AND panel_limited AND has_sun` (`MPPT_HOLD_TIME_MS` = 30 s)

### Contract with charger module
- While `ctx->mppt.state == MPPT_TRACKING`, the charger must skip its CC regulation step — MPPT writes `ctx->pwm`. Outside of TRACKING, CC/CV owns the PWM
- `enter_disabled()` resets `step_size`, `reversals`, `last_direction` so a subsequent reactivation starts from a clean state

---

# [v0.07] - 15.04.26
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

# [v0.06] - 03.04.26
## Implement energy mode finite state machine (pipeline step 5)

New files: `energy_mode.h`, `energy_mode.c`

### NEW: energy_mode_update() — pipeline step 5
- Runs after `fault_mgr_update()` and before `mppt_update()` — the traffic controller that decides which hardware power paths are active
- Five-state machine per `docs/transition_table.csv`: `EM_IDLE`, `EM_CHARGE_ONLY`, `EM_CHARGE_AND_LOAD`, `EM_DISCHARGE_ONLY`, `EM_SAFE_MODE`
- Guard inputs are all read-only fields set by earlier pipeline steps: `flag_has_sun.value`, `flag_bat_low.value`, `has_load`, `bat_full`, `meas.bat_voltage`

### Hardware enable matrix (set on entry)
| State            | CHARGER_EN | BATTERY_EN | OUTPUT_EN | USB_EN | BUCK_DIS |
|------------------|------------|------------|-----------|--------|----------|
| IDLE             | off        | off        | off       | off    | asserted |
| CHARGE_ONLY      | on         | on         | off       | off    | released |
| CHARGE_AND_LOAD  | on         | on         | on        | on     | released |
| DISCHARGE_ONLY   | off        | on         | on        | on     | asserted |
| SAFE_MODE        | off        | on         | off       | off    | asserted |

### Charger/MPPT region control
- `activate_charger_region()`: on entry to CHARGE_ONLY or CHARGE_AND_LOAD — enables buck + charge switch; charger self-determines PRECHARGE vs CC on its next tick. Guarded by `charger.state == CHG_INACTIVE` to avoid resetting a running charge cycle (e.g., CHARGE_ONLY → CHARGE_AND_LOAD keeps charger state intact)
- `deactivate_charger_region()`: on exit from a charging state to a non-charging state — parks PWM at `PWM_MIN_DUTY`, asserts `BUCK_DIS`, resets charger → `CHG_INACTIVE`, MPPT → `MPPT_DISABLED`, clears `bat_full`

### Transition guards — priority-ordered per state
- Each state has a dedicated `eval_*()` function with guards checked in strict if/else priority order (lower number = higher priority = checked first)
- Safety-critical transitions (→ SAFE_MODE) are always highest priority where applicable
- SAFE_MODE recovery requires `V_bat > BAT_SAFE_RECOVER_MV` (3200 mV) — raw voltage, intentionally not debounced. All exits require genuine battery recovery; `!has_load` alone is not an exit (prevents oscillation — see CHANGELOG v0.01)

### Idle sleep timeout
- In IDLE with no transition, tracks time since `idle_start_ms`. After `IDLE_SLEEP_TIMEOUT_MS` (2 min), sets `ctx->idle_sleep_pending = true` for main loop to enter low-power mode

---

# [v0.05] - 01.04.26
## Implement main system entry point with measurements and UART logging

New files: `main.c` (bringup stub)

### NEW: main.c — bringup version
- Minimal entry point for hardware validation: initialises system, runs measurements + flags on a 50 ms tick, logs all ADC values over UART on a 1 s tick
- No state machines, no charger, no MPPT, no fault management — purely for verifying ADC readings and flag behaviour on physical hardware
- SysTick ISR at 1 ms: increments timestamp, kicks ADC conversions every 10 ms
- Tab-separated UART output for spreadsheet import

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

