# SPC_20 — Constants, Thresholds & Faults Reference

Generated from `hw_config.h`, `system_types.h`, `fault_mgr.c`, `charger.c`, `power_budget.c`
and `SPCBoardAPI.c` at commit `a59be85` (v0.25 + uncommitted working tree, 2026-09-08).

`hw_config.h` is the single source of truth: **no magic number lives anywhere else**.
Values marked *derived* are computed by the preprocessor from other constants.

---

## 1. Battery limits (LiFePO4, single cell)

| Constant | Value | Unit | Meaning / why |
|---|---|---|---|
| `BAT_PRECHARGE_MV` | 3000 | mV | Below this the charger enters PRECHARGE (trickle only) |
| `BAT_PRECHARGE_MAX_MA` | 200 | mA | Trickle ceiling during precharge |
| `BAT_PRECHARGE_TIMEOUT_MS` | 900 000 | ms | 15 min stuck < 3.0 V → `FAULT_PRECHARGE_TIMEOUT` |
| `BAT_CC_MAX_MA` | 2000 | mA | Max charge current in the CC zone |
| `BAT_CV_VOLTAGE_MV` | 3650 | mV | CV target voltage (never exceed) |
| `BAT_CV_TOLERANCE_MV` | 10 | mV | Regulate between 3650 and 3660 mV |
| `BAT_CV_TAPER_MA` | 200 | mA | I_chg below this in CV → battery considered full |
| `BAT_FULL_HOLD_MS` | 30 000 | ms | Must hold below taper current 30 s to confirm `bat_full` |
| `BAT_LOW_MV` | 2800 | mV | `bat_low` flag sets below this |
| `BAT_LOW_CLEAR_MV` | 2900 | mV | `bat_low` clears above this (100 mV hysteresis) |
| `BAT_LOW_DEBOUNCE_COUNT` | 3 | ticks | ×50 ms consecutive readings before `bat_low` sets |
| `BAT_FULL_MV` | 3650 | mV | Used in flag evaluation (with a current check) |
| `BAT_SAFE_RECOVER_MV` | 3200 | mV | Raw V_bat needed to exit `EM_SAFE_MODE` (400 mV gap vs `BAT_LOW_MV`) |
| `BAT_OVERVOLT_MV` | 3700 | mV | Hard fault trip: disconnect charger immediately |
| `BAT_OVERVOLT_RECOVER_MV` | 3400 | mV | Overvolt fault clears below this (300 mV hysteresis) |
| `BAT_UNDERVOLT_MV` | 2000 | mV | Hard fault trip: shed loads + charge path |
| `BAT_UNDERVOLT_RECOVER_MV` | 3200 | mV | Undervolt fault clears above this (aligned with SAFE_MODE exit) |
| *(literal)* undervolt sense guard | 500 | mV | Hard-coded in `fault_mgr.c`: below this V_bat is "no cell", no undervolt trip |

**Battery-limit SoC gate** (`power_budget.c`, sets `battery_limit`):

| V_bat range | Limit | Phase |
|---|---|---|
| < 3000 mV | 200 mA | Precharge trickle |
| 3000 – 3650 mV | 2000 mA | CC, full rate |
| ≥ 3650 mV | 200 mA | CV safety cap |

---

## 2. Supervised undervolt rescue (v0.22)

| Constant | Value | Unit | Meaning / why |
|---|---|---|---|
| `BAT_RESCUE_MIN_MV` | 1500 | mV | Hard floor — below this stay latched (copper-dissolution territory). 500–1500 mV is the hard-latched band |
| `BAT_RESCUE_TIMEOUT_MS` | 1 800 000 | ms | 30 min deep-discharge precharge window before escalating to `FAULT_PRECHARGE_TIMEOUT` |

Rescue fires only when undervolt is the **sole** latched fault, sun is usable, and V_bat ≥ 1500 mV.

---

## 3. Battery protection wake probe (v0.23, S-8240 lockout escape)

| Constant | Value | Unit | Meaning / why |
|---|---|---|---|
| `BAT_PROT_SIG_MIN_MV` | 600 | mV | Protection-open bias-node signature window, low edge (bench: 744–896 mV) |
| `BAT_PROT_SIG_MAX_MV` | 1100 | mV | Signature window, high edge |
| `BAT_PROT_SIG_COLLAPSED_MAX_MV` | 300 | mV | Collapsed variant (panel absent, bus dead → node reads ~0 mV) |
| `BAT_WAKE_DETECT_TICKS` | 40 | ticks | 2 s of consecutive candidate readings before probing (outlasts the 640 ms MA) |
| `BAT_WAKE_OUT_COLLAPSED_MV` | 1000 | mV | 3VOUT must be below this (loads shed) to qualify as the quiescent lockout |
| `BAT_WAKE_LOW_PROBE_TARGET_MV` | 3000 *(= `BAT_PRECHARGE_MV`)* | mV | First probe target — gentler step into a low cell |
| `BAT_WAKE_PROBE_TARGET_MV` | 3650 *(= `BAT_CV_VOLTAGE_MV`)* | mV | Full probe target; **fixed**, never V_bat + headroom |
| `BAT_WAKE_BUCK_SETTLE_MS` | 50 | ms | Min buck soft-start delay with Q49 open |
| `BAT_WAKE_BUCK_SETTLE_MAX_MS` | 1000 | ms | Buck-ready timeout → rate-limited retry |
| `BAT_WAKE_BUCK_READY_TOL_MV` | 75 | mV | VCHG must be within this of target before connecting the cell |
| `BAT_WAKE_PROBE_MS` | 3000 | ms | Stimulus duration cap (this is *not* a charging window) |
| `BAT_WAKE_PROBE_CUT_MA` | 200 *(= `BAT_PRECHARGE_MAX_MA`)* | mA | End the stimulus early once this much current flows |
| `BAT_WAKE_SETTLE_MS` | 1500 | ms | Buck-off settle (≈2.3 × the 640 ms MA window) before the persistence test |
| `BAT_WAKE_VALIDATE_MS` | 1000 | ms | Persistence window — V_bat must hold plausibly for the whole time, buck off |
| `BAT_WAKE_VALID_MIN_MV` | 1500 *(= `BAT_RESCUE_MIN_MV`)* | mV | Min off-state voltage accepted as "a real cell is connected" |
| `BAT_WAKE_RETRY_MS` | 60 000 *(= `HAS_SUN_RELOCK_MS`)* | ms | Retry pacing |
| `BAT_WAKE_MAX_ATTEMPTS` | 3 | — | Attempt budget, then terminal |

---

## 4. Solar panel / `has_sun`

| Constant | Value | Unit | Meaning / why |
|---|---|---|---|
| `PANEL_MIN_MV` | 6000 | mV | `has_sun` sets above this (unloaded panel ≈ Voc in IDLE) |
| `PANEL_MIN_CLEAR_MV` | 4200 | mV | `has_sun` clear threshold; sits below `PANEL_SAFETY_MV` and above the ~3.6 V collapsed reading |
| `HAS_SUN_DEBOUNCE_COUNT` | 3 | ticks | ×50 ms before `has_sun` sets |
| `HAS_SUN_CLEAR_COUNT` | 30 | ticks | 1.5 s of sub-threshold readings before `has_sun` clears (rides out activation transients) |
| `PANEL_USABLE_MIN_MW` | 200 | mW | Dusk floor: below this while charging, `has_sun` clears even at Voc |
| `HAS_SUN_DUSK_CLEAR_COUNT` | 80 | ticks | 4 s — the dusk detector's **own** counter (v0.21 fix: must not chain with the voltage clear) |
| `HAS_SUN_RELOCK_MS` | 60 000 | ms | After a power-based clear, suppress voltage re-set for 60 s |
| `PANEL_SAFETY_MV` | 4800 | mV | Emergency floor — regulation backs off hard below this |
| `PANEL_OVP_CLAMP_MV` | *(15000, commented out)* | mV | Approximate hardware OVP clamp; informational only |

Floor chain (must stay ordered): vreg band low 5300 > `PANEL_SAFETY_MV` 4800 > `PANEL_MIN_CLEAR_MV` 4200 > collapsed panel ≈ 3600.

---

## 5. Input-voltage regulation ("constant-voltage MPPT")

| Constant | Value | Unit | Meaning / why |
|---|---|---|---|
| **`CHARGER_INPUT_VREG`** | **1** | — | **Master switch.** 1 = regulate panel input voltage + setpoint-P&O MPPT; 0 = legacy fixed-current CC + PWM-perturbing MPPT |
| `PANEL_VREG_SETPOINT_MV` | 6500 | mV | **Cold-boot fallback only** — the live setpoint is `ctx->mppt.vreg_setpoint_mv` |
| `PANEL_VREG_DEADBAND_MV` | 1200 | mV | Half-width of the regulation band (6500 ± 1200 → hold 5.3–7.7 V); wide because 1 PWM count ≈ 25 mA panel-side |
| `PANEL_VREG_STEP` | 1 | counts | PWM counts per regulation step (~2.9 mV rail, ~45 mA charge-side) |
| `PANEL_VREG_INTERVAL_MS` | 400 | ms | Min spacing between steps — **must** exceed the ~320 ms ADC MA group delay |
| `LOAD_REACQUIRE_MA` | 60 | mA | In-band but delivering less than this → step toward more current (v0.21 backoff-overshoot escape) |

---

## 6. Buck converter (TPS564247) & PWM

| Constant | Value | Unit | Meaning / why |
|---|---|---|---|
| `PWM_PERIOD` | 400 | counts | Timer period; compare written = `400 − pwm` |
| `PWM_MAX_DUTY` | 1 | counts | Lowest `pwm` → 99.75 % duty → **maximum** current |
| `PWM_MIN_DUTY` | 399 | counts | Highest `pwm` → 0.25 % duty → minimum current / off (boot + every shutdown park here) |
| `BUCK_MAX_CURRENT_MA` | 2000 | mA | Hardware inductor/FET current limit |

**Sign convention:** lower `pwm` = higher duty = **more** current. Legal range `[1, 399]`; **0 and 400 are forbidden** (0 = inductor saturation, 400 = counter overflow). `apply_pwm()` is the only writer and clamps as defense-in-depth.

---

## 7. LED boost & lamps

| Constant | Value | Unit | Meaning / why |
|---|---|---|---|
| `LED_BOOST_TARGET_MV` | 11 500 | mV | LED rail target (saturates to duty=1, rail clamps ~11.3 V) |
| `LAMP_ON_CURRENT_MA` | 150 | mA | Per-lamp drive when switched on (LUT bins 145/155; HW max 350 mA/channel) |
| `LAMP_DIM_LEVELS` | 40 | levels | Brightness levels; index 40 = full, index 0 = off (tap only) |
| `LAMP_DIM_STEP_MS` | 83 | ms | One dim level per hold interval → full → min in ≈3.2 s |
| `LAMP_DIM_MIN_LEVEL` | 1 | level | A hold stops here; it never switches the lamp off |

---

## 8. Load detection

| Constant | Value | Unit | Meaning / why |
|---|---|---|---|
| `LOAD_DETECT_MA` | 120 | mA | I_load above this → `has_load = true` |
| `LOAD_DETECT_CLEAR_MA` | 80 | mA | I_load below this → `has_load = false` |

Both sit clear of the R432 discharge shunt's ~45 mA zero-offset (the old 50/30 window straddled it and latched phantom loads).

---

## 9. Charger tuning

| Constant | Value | Unit | Meaning / why |
|---|---|---|---|
| `CC_DEADBAND_MA` | 25 | mA | ±25 mA around target before adjusting |
| `CC_PWM_STEP` | 1 | counts | PWM adjustment per regulation cycle |
| `CHG_ACTIVATION_HEADROOM_MV` | 50 | mV | Pre-position to V_bat + this on activation. Not 0 (sync FETs reverse-pump), not 200 (commanded ~2.7 A and collapsed the panel — a real bug) |
| `CHG_BUCK_READY_MARGIN_MV` | 20 | mV | Staged activation: VCHG must exceed V_bat by this before Q49 closes |
| `CHG_BUCK_SETTLE_MS` | 50 | ms | Min unloaded buck soft-start dwell |
| `CHG_BUCK_SETTLE_RAMP_MS` | 50 *(= `TICK_MAIN_MS`)* | ms | Trim cadence while Q49 is open |
| `CHG_BUCK_SETTLE_PWM_STEP` | 2 | counts | Trim step (~6 mV) toward readiness |
| `CC_DOWNSTEP_INTERVAL_MS` | 300 | ms | Max one CC down-step (more current) per interval; up-steps stay every tick |
| `CV_DEADBAND_MV` | 5 | mV | Regulate between `BAT_CV_VOLTAGE_MV` and +5 mV |
| `CV_PWM_STEP` | 1 | counts | PWM step in CV |
| `PANEL_BACKOFF_STEP` | 5 | counts | Panel-safety backoff step (~−215 mA demand), max once per `PANEL_VREG_INTERVAL_MS` |
| `CHG_REVERSE_CURRENT_MA` | 100 | mA | More negative than this → reverse-current escape (lift the rail fast) |
| `CHG_INPUT_LOST_MARGIN_MV` | 400 | mV | Input-loss trip: instantaneous V_panel < V_bat + this. **Sized to stay below `PANEL_SAFETY_MV`** |
| `CHG_INPUT_LOST_SAMPLES` | 3 | — | Observed 10 ms conversions below that margin before standing down. Each one first backs the draw off (v0.32); a live panel over its knee recovers on the first, a removed source never does. Gated on `adc_sample_seq()`, not on calls |
| `CHG_INPUT_RECOVER_STEP` | 10 | counts | PWM backoff per counted sub-margin conversion (≈450 mA less). Applied on samples 1 and 2; sample 3 stands down (v0.32) |
| `CHG_REVERSE_BLANK_MS` | 200 | ms | After the input guard's last backoff, reverse current with a live input is read as the rescue's tail, not reverse pumping: correct the target, then stand down cleanly — never latch `0x0100` (v0.34) |
| `CHG_CONNECT_BLANK_MS` | 150 | ms | After Q49 closes, `charger_fast_guard` yields instead of judging reverse current. The buck hands off from SETL delivering ~nothing, so cell current starts at 0 where one raw sample straddles the threshold on noise; the pathology is excluded meanwhile by SETL's VCHG > V_bat interlock (v0.36) |
| `CHG_REVERSE_SAMPLES` | 3 | — | Distinct 10 ms conversions of reverse current on a LIVE input before latching `0x0100`. Real reverse pumping is sustained (−1010 mA on the May trace); a single noise sample about zero is not, and a latch costs 10 s. Gated on `adc_sample_seq()`. The dead-input stand-down stays undebounced (v0.36) |
| `CHG_INPUT_REARM_MS` | 2000 | ms | Wait before energy_mode may re-arm after an input-loss stand-down |
| `CHG_CLIFF_PWM_MARGIN` | 6 | counts | Learned PWM ceiling: every collapse fences `cliff_pwm_min` this far above the count that fell over (~270 mA of demand). `cc_regulate`/`cv_regulate` may not draw below it (v0.33) |
| `CHG_CLIFF_PWM_RELAX_MS` | 180000 | ms | One count of that ceiling released per this much collapse-free charging. Slower than the fading-panel drift it fences, or it cancels the ratchet (v0.33) |
| `CHG_RESUME_MAX_ADVANCE` | 10 | counts | How far a warm resume may advance the activation pre-position toward the learned ceiling. Bounds the rail above the cell at Q49 close (~1.2 A first connection) (v0.33) |
| `MPPT_RESEED_GAP_MS` | 60000 | ms | Charger-region downtime after which MPPT re-seeds from FOCV. Under it (and with `has_sun` still set) a resume keeps the learned Voc/setpoint (v0.29) |
| `ADC_PANEL_TRACE_DEPTH` | 16 | — | Raw V_panel ring dumped as an `INTRACE` line when the input guard trips. Diagnostic only (v0.29) |

`cc_regulate` priority order each tick: **1.** panel safety backoff → **2.** reverse-current escape → **3.** battery-intake clamp (over-current always wins) → **4.** panel-voltage loop.

Every branch that draws **more** current (the panel-voltage loop, the in-band re-acquire, CV's pull-up) goes through `pwm_draw_more` and is bounded by `cliff_pwm_min`. Backoffs, the reverse-current escape and the `CHG_BUCK_SETTLE` acquisition ramp are not: they reduce current or are interlocks that must not be blockable.

---

## 10. MPPT — setpoint-P&O (`CHARGER_INPUT_VREG = 1`, live)

| Constant | Value | Unit | Meaning / why |
|---|---|---|---|
| `MPPT_SP_FRACTION_PCT` | 87 | % | FOCV seed: setpoint = 87 % × Voc − deadband. 76 was measured wrong on this panel class (bench MPP/Voc 0.87 and 0.89) and drove the loop past the knee — v0.29 |
| `MPPT_SP_STEP_MV` | 1000 | mV | Initial setpoint perturbation per dwell |
| `MPPT_SP_STEP_MIN_MV` | 250 | mV | Step floor after halving on reversals |
| `MPPT_SP_SETTLE_MS` | 2000 | ms | Settle phase of each dwell |
| `MPPT_SP_MEASURE_MS` | 1000 | ms | Measure phase (≈3 s per probe) |
| `MPPT_SP_MIN_DELTA_MA` | 15 | mA | Noise gate: >+15 accept & grow; <−15 revert & reverse; \|Δ\| ≤ 15 → flat → reverse |
| `MPPT_SP_CONVERGE_REVERSALS` | 3 | — | Reversals before declaring convergence → HOLD |
| `MPPT_SP_RUNTIME_MS` | 45 000 | ms | Hard cap on one TRACKING session |
| `MPPT_SP_MIN_MV` | 6500 *(derived: 4800 + 1200 + 500)* | mV | Setpoint floor — keeps the band's low edge above `PANEL_SAFETY_MV` |
| `MPPT_SP_FLOOR_PCT` | 80 | % | Voc-relative setpoint floor: realised operating point cannot be commanded below 0.80·Voc. `MPPT_SP_MIN_MV` alone let the tracker reach 6.5 V on a panel with MPP at 11.85 V (v0.30) |
| `MPPT_SP_CLIFF_MARGIN_MV` | 250 | mV | On an input-loss teardown the floor is set so the band top lands this far above the last clean averaged V_panel (`sp_learn_cliff`, v0.30) |
| `MPPT_SP_VPANEL_DROP_MV` | 300 | mV | Largest tick-to-tick fall of averaged V_panel still trusted for cliff learning; larger falls already contain the collapse (v0.30) |
| `MPPT_SP_VOC_GUARD_MV` | 1500 *(derived: 1200 + 300)* | mV | Ceiling guard below captured Voc |
| `MPPT_SP_COLLAPSE_BLANK_MS` | 1200 | ms | Collapse blanking from dwell start |
| `MPPT_SP_HOLD_TIME_MS` | 60 000 | ms | HOLD duration before a periodic re-probe |

In this mode MPPT **never writes `ctx->pwm`** and **never updates `mppt_limit_ma`**.

## 11. MPPT — legacy inc-conductance (`CHARGER_INPUT_VREG = 0`)

| Constant | Value | Unit | Meaning / why |
|---|---|---|---|
| `MPPT_MAX_STEP_SIZE` | 8 | counts | Initial PWM step per perturbation |
| `MPPT_MIN_STEP_SIZE` | 1 | counts | Smallest step (convergence threshold) |
| `MPPT_CONVERGE_REVERSALS` | 6 | — | Reversals at min step → converged |
| `MPPT_STEP_INTERVAL_MS` | 400 | ms | Perturb/observe pacing (≥ ADC MA group delay) |
| `MPPT_RUNTIME_MS` | 6000 | ms | Max time in TRACKING (~15 paced steps) |
| `MPPT_HOLD_TIME_MS` | 30 000 | ms | Wait in HOLD before re-entering TRACKING |
| `MPPT_STUCK_TICK_LIMIT` | 8 | ticks | Stiff-source escape: 8 same-direction ticks at max step → HOLD |

## 12. MPPT — shared

| Constant | Value | Unit | Meaning / why |
|---|---|---|---|
| `CHARGER_MPPT_SETTLE_MS` | 250 | ms | Charger settle after activation before MPPT may engage (must beat CC's first 300 ms down-step) |
| `PANEL_LIMITED_MARGIN_MA` | 100 | mA | `I_chg < allowed_chg − this` with sun → `panel_limited` |
| `MPPT_LIMIT_DEFAULT_MA` | 2000 *(= `BUCK_MAX_CURRENT_MA`)* | mA | Cold-boot `mppt_limit_ma`; under vreg mode it never changes, so the *battery* limits bound `allowed_chg` |

---

## 13. Fault thresholds

| Constant | Value | Unit | Meaning / why |
|---|---|---|---|
| `FAULT_OVERCURRENT_CHG_MA` | 2200 | mA | Charge over-current trip (BUCK_MAX + 200 mA margin) |
| `FAULT_OVERCURRENT_DSG_MA` | 5000 | mA | Discharge over-current trip |
| `FAULT_SYSTEM_CURRENT_MA` | 6000 | mA | Total system current fault — **defined but not referenced by any code today** |
| `FAULT_USB_OVERVOLT_MV` | 6000 | mV | USB output over-voltage trip |
| `FAULT_RECOVER_WAIT_MS` | 10 000 | ms | Recovery pass cadence — min dwell before any latched fault may clear |

---

## 14. Temperature limits

| Constant | Value | Unit | Meaning / why |
|---|---|---|---|
| `BAT_TEMP_MAX_CHARGE_C` | 45 | °C | Stop charging above this (soft, `TEMP_CHARGE_BLOCK`) |
| `BAT_TEMP_MIN_CHARGE_C` | 0 | °C | Stop charging below this (soft) |
| `BAT_TEMP_MAX_DISCHARGE_C` | 60 | °C | Hard `OVERTEMP` fault on battery |
| `BOARD_TEMP_MAX_C` | 60 | °C | Hard `OVERTEMP` fault on board |
| `TEMP_HYSTERESIS_C` | 10 | °C | Must drop this far below the limit to recover |

---

## 15. System timing & deep sleep

| Constant | Value | Unit | Meaning / why |
|---|---|---|---|
| `TICK_ADC_MS` | 10 | ms | ADC raw sampling rate (SysTick harvests here) |
| `TICK_BUTTON_MS` | 20 | ms | Button debounce/polling rate |
| `TICK_MAIN_MS` | 50 | ms | State machine + regulation tick (the 8-step pipeline) |
| `LOG_MODE_DEFAULT` | `LOG_MODE_FAST` | — | Boot telemetry rate seeded into `ctx.log_mode` (0 off / 1 = 1 Hz / 2 = 5 Hz, live-writable at a breakpoint) |
| `TICK_LOG_1HZ_MS` | 1000 | ms | UART telemetry interval in `LOG_MODE_1HZ` |
| `TICK_LOG_FAST_MS` | 200 | ms | UART telemetry interval in `LOG_MODE_FAST` (5 lines/s) |
| `VBATM_REFRESH_MS` | 1000 | ms | Re-pulse VBATM_EN so a hot-plugged cell appears |
| `IDLE_SLEEP_TIMEOUT_MS` | 120 000 | ms | 2 min in IDLE/SAFE_MODE with no activity → arm STANDBY0 |
| `SLEEP_WAKE_INTERVAL_MS` | 10 000 | ms | Periodic wake-check period (~0.6 % MCU duty cycle) |
| `SLEEP_TIMER_TICK_HZ` | 16 | Hz | TIMG8 rate — **must match** SysConfig: LFCLK 32768 / 8 / 256 |
| `SLEEP_CHECK_SETTLE_MS` | 60 | ms | Awake window per wake-check before sampling |
| `SLEEP_TIMER_LOAD_COUNTS` | 159 *(derived)* | counts | `interval × 16 / 1000 − 1` |

Sleep entry is gated on: no latched fault, and (in IDLE) all lamps off. Wake sources: button edges (PB6/PB7, GROUP1) and the TIMG8 periodic timer. The four PD1 PWM timers are saved/restored around STANDBY (they lose registers), and the LED boost rail is forced off across sleep.

---

## 16. Battery capacity (coulomb counter / SoC)

| Constant | Value | Unit |
|---|---|---|
| `BAT_NOMINAL_CAPACITY_MAH` | 20 000 | mAh |
| `BAT_CHARGE_EFFICIENCY_PCT` | 94 | % |

---

## 17. UI — LED bar displays

| Constant | Value | Unit | Meaning |
|---|---|---|---|
| `UI_BAT_PRESENT_MV` | 500 | mV | Below this the battery bar stays dark (mirrors the undervolt sense guard) |
| `UI_BAT_SEG1_MV` | 3020 | mV | Battery bar segment 1 |
| `UI_BAT_SEG2_MV` | 3120 | mV | Segment 2 |
| `UI_BAT_SEG3_MV` | 3240 | mV | Segment 3 |
| `UI_BAT_SEG4_MV` | 3360 | mV | Segment 4 |
| `UI_BAT_SEG5_MV` | 3480 | mV | Segment 5 |
| `UI_PANEL_SEG1_MW` | 200 | mW | Panel-power bar segment 1 |
| `UI_PANEL_SEG2_MW` | 750 | mW | Segment 2 |
| `UI_PANEL_SEG3_MW` | 1500 | mW | Segment 3 |
| `UI_PANEL_SEG4_MW` | 3000 | mW | Segment 4 |
| `UI_PANEL_SEG5_MW` | 5000 | mW | Segment 5 |
| `UI_BOOT_ANIM_STEP_MS` | 120 | ms | Power-on sweep step (~0.6 s end to end) |

---

## 18. Hardware abstraction constants (`SPCBoardAPI.c`)

Not tuning knobs — board/sensor calibration and PWM geometry.

| Constant | Value | Meaning |
|---|---|---|
| `CPU_CLK` | 32 000 000 | MCU clock (Hz) |
| `BUCK_PWM_PERIOD` | 400 | Buck FB PWM period (matches `PWM_PERIOD`) |
| `AUX_PWM_PERIOD` | 100 | Aux (boost) PWM period |
| `LED_CURRENT_PWM_PERIOD` | 800 | LED current-reference PWM period (8× the LUT grid) |
| `LED_CURRENT_LUT_PERIOD` | 100 | Calibration LUT grid the 800-count period interpolates from |
| `LED_CURRENT_DUTY_SCALE` | 8 *(derived)* | 800 / 100 |
| `MAX_DUTY_CYCLES_BUCK` / `_LED` | 99 | Max LUT duty index (compare==period is forbidden/runaway) |
| `MAX_DUTY_CYCLES_BOOST` | 35 | Max boost duty index |
| `CHG_CUR_GAIN` / `IN_CUR_GAIN` / `OUT_CUR_GAIN` | 50.0 | Current-sense amplifier gain (V/V) |
| `CHG_CUR_RES` / `IN_CUR_RES` / `OUT_CUR_RES` | 0.005 | Shunt resistance (Ω) — R440 / R4xx / R432 |
| `VBATM_DIV_RATIO` | 0.53 (`__SPC_50_R1__`) / 0.5 | V_BATM divider |
| `VCHGM_DIV_RATIO` | 0.5 | VCHG divider |
| `VOUTM_DIV_RATIO` | 0.5 | 3VOUT divider |
| `VLED_DIV_RATIO` | 0.091 | V_LED divider |
| `VPANEL_DIV_RATIO` | 0.091 | V_panel divider |
| `VUSB_DIV_RATIO` | 0.091 | USB divider |
| `WINDOW_SIZE` | 64 | ADC moving-average depth → **~320 ms group delay at 10 ms/tick** (the reason for most pacing constants) |
| `NUM_VARS` | 15 | Averaged ADC channels |
| `DEADBAND_THRESHOLD_ADC` | 1 | ADC deadband (counts) |
| `INVALID_RESULT` | `INT16_MAX` | Sentinel for a failed conversion |
| `LED_DISPLAY_UPDATE_PERIOD_MS` | 3 | Bar-graph multiplex period |
| `UART_BUF_SIZE` (`main.c`) | 512 | Telemetry TX buffer (bytes) |

---

## 19. Faults

Nine latched bits in `ctx->fault.code` (`uint16_t`). `fault.history` is a sticky OR of everything raised this boot. `fault.active` mirrors `code != FAULT_NONE`.

| Bit | Name | Mask | Raised by | Trip condition | Immediate protective action | Recovery condition |
|---|---|---|---|---|---|---|
| 0 | `FAULT_OVERTEMP` | `0x0001` | `fault_detect` | `bat_temp > 60 °C` **or** `board_temp > 60 °C` | Disable charge switch, buck, output switch, USB boost, LED boost (everything) | Both sensors < limit − 10 °C |
| 1 | `FAULT_BAT_OVERVOLT` | `0x0002` | `fault_detect` | `V_bat > 3700 mV` | Disable charge switch + buck | `V_bat < 3400 mV` |
| 2 | `FAULT_OVERCURRENT_CHG` | `0x0004` | `fault_detect` | `chg_current > 2200 mA` | Disable charge switch + buck | `chg_current < 2000 mA` (the 10 s wait is the real debounce) |
| 3 | `FAULT_OVERCURRENT_DSG` | `0x0008` | `fault_detect` | `dsg_current > 5000 mA` | Disable output switch, USB boost, LED boost | `dsg_current < 2000 mA` |
| 4 | `FAULT_BAT_UNDERVOLT` | `0x0010` | `fault_detect` | `500 mV < V_bat < 2000 mV` | Shed loads (output, USB, LED) **and** charge path (Q49 + buck) | `V_bat > 3200 mV` **and** wake probe not busy |
| 5 | `FAULT_USB_OVERVOLT` | `0x0020` | `fault_detect` | `usb1` or `usb2 > 6000 mV` | Disable USB boost | Both USB rails < 6000 mV |
| 6 | `FAULT_PRECHARGE_TIMEOUT` | `0x0040` | `charger.c` (`tick_precharge`) | 15 min in PRECHARGE (30 min after a deep-discharge rescue) | Disable charge switch + buck | `V_bat > 3000 mV` (i.e. cell replaced / visibly climbing) |
| 7 | `FAULT_TEMP_CHARGE_BLOCK` | `0x0080` | `fault_detect` | `!ctx->temp_charge_ok` (outside 0–45 °C, 10 °C hysteresis applied in `flags_update`) | Disable charge switch + buck | `ctx->temp_charge_ok` is true again |
| 8 | `FAULT_REVERSE_PUMP` | `0x0100` | `charger.c` (`charger_fast_guard`) | `I_buck = chg + dsg < −100 mA` **while the input is still LIVE** | Disable charge switch (Q49 first) then buck; `pwm = 399` | `chg_current > −100 mA` (10 s wait is the debounce) |

### Fault contracts

- **Latching** — a raised bit stays set until its recovery condition is met on a recovery pass.
- **Recovery cadence** — the whole recovery pass is throttled to one attempt per `FAULT_RECOVER_WAIT_MS` (10 s). The dwell timer restarts when a fault is first latched.
- **Re-arm handshake** — recovery clears the bit but does **not** re-enable GPIOs. `energy_mode` detects the `fault.code` falling edge via `fault.prev_code` and re-applies the current state's entry actions.
- **No `SYS_FAULT` state** — faults are handled inside `SYS_RUN`; the mode-selection guards never branch on `fault.code`, which is why protection is taken as direct hardware action here.
- **A backoff never commands a rail below the cell** — every foreground backoff clamps at `charger.zero_draw_pwm` (the LUT count for V_bat + `CHG_ACTIVATION_HEADROOM_MV`, refreshed each charger tick). Past it the buck sinks instead of delivering, which *is* the reverse-pump condition. v0.32's blind backoff manufactured the fault `charger_fast_guard` latched on (v0.34).
- **Input loss is *not* a fault** — `charger_input_guard` stands the charger down (`input_lost_pending`, re-arm after `CHG_INPUT_REARM_MS`) without latching anything. Only reverse pumping into a *live* input latches.
- **Sleep gate** — deep sleep is blocked while any fault is latched (recovery needs pipeline ticks).

### Charger blocking mask

```
CHG_FAULT_BLOCK_MASK = OVERTEMP | BAT_OVERVOLT | OVERCURRENT_CHG
                     | REVERSE_PUMP | PRECHARGE_TIMEOUT | TEMP_CHARGE_BLOCK
                     = 0x01C7
```

`FAULT_BAT_UNDERVOLT` is deliberately **excluded** — the supervised SAFE_MODE rescue needs the charger to run with undervolt latched. `safe_mode_rescue_active()` only fires when undervolt is the sole latched fault, so no mask bit is ever bypassed.

---

## 20. Boot defaults (`ctx_init`, `system_types.h`)

| Field | Value |
|---|---|
| `system_state` | `SYS_INIT` |
| `pwm` | 399 (`PWM_MIN_DUTY`, buck off) |
| `energy_mode` | `EM_IDLE` |
| `charger.state` | `CHG_INACTIVE` |
| `mppt.state` | `MPPT_DISABLED` |
| `mppt.mppt_limit_ma` | 2000 (`MPPT_LIMIT_DEFAULT_MA`) |
| `mppt.step_size` | 8 (`MPPT_MAX_STEP_SIZE`) |
| `mppt.vreg_setpoint_mv` / `prev_sp_mv` | 6500 (`PANEL_VREG_SETPOINT_MV`) |
| `mppt.sp_direction` | +1 |
| `fault.code` / `prev_code` / `history` | `FAULT_NONE` (0) |
| `bat_wake.phase` | `BAT_WAKE_MONITOR` |
| `flag_bat_low` | set threshold 3, **clear threshold 1** (immediate clear) |
| `flag_has_sun` | set threshold 3, clear threshold 30 |
| `has_sun_relock_ms` / `has_sun_dusk_count` | 0 (probe the panel immediately at boot) |
| `temp_charge_ok` | `true` (assume OK until first measurement) |
| `lamp_level[0..3]` | 0 (all lamps off) |
