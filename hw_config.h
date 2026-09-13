/*
 * hw_config.h — Hardware Constants and Thresholds
 * ================================================
 *
 * Every magic number in the system lives here. Nothing else
 * defines thresholds, limits, or timing constants.
 *
 * Organisation:
 *   1. Battery limits        — voltage/current safety for LiFePO4 cell
 *   2. Solar/panel limits    — panel operating boundaries
 *   3. Buck converter        — PWM range and current capacity
 *   4. Load detection        — how we decide "there's a load"
 *   5. Charger tuning        — regulation deadbands and timing
 *   6. MPPT tuning           — algorithm parameters
 *   7. Fault thresholds      — over-temp, over-current, etc.
 *   8. System timing         — tick rates, debounce, sleep
 *   9. Temperature           — charge/discharge thermal limits
 */

#ifndef HW_CONFIG_H
#define HW_CONFIG_H

#include <stdint.h>   /* INT16_MAX — TEMP_INVALID_C sentinel (section 8) */

/* =========================================================================
 * 1. BATTERY LIMITS (LiFePO4 single cell)
 * =========================================================================
 *
 * LiFePO4 nominal:  3.2 V
 * Full charge:      3.65 V  (never exceed)
 * Empty:            2.5 V   (damage below this)
 *
 * The voltages below are in millivolts as read by the ADC path:
 *   V_BAT_M = V_BAT / 2  (resistor divider, Q6 enabled)
 *   The get_battery_voltage() function returns mV after scaling.
 */

/* Precharge zone: battery is deeply discharged, trickle-charge only */
#define BAT_PRECHARGE_MV          3000    /* below this → precharge at reduced current           */
#define BAT_PRECHARGE_MAX_MA      200     /* max current during precharge                        */
#define BAT_PRECHARGE_TIMEOUT_MS  900000UL /* 15 minutes — if still below 3V, battery is damaged */

/* Constant-current zone */
#define BAT_CC_MAX_MA             2000    /* max charge current in CC phase                      */

/* Constant-voltage zone */
#define BAT_CV_VOLTAGE_MV         3650    /* target voltage in CV phase                          */
#define BAT_CV_TOLERANCE_MV       10      /* regulate between 3650 and 3660                      */
#define BAT_CV_TAPER_MA           200     /* when I_charge drops below this in CV → battery full */
#define BAT_FULL_HOLD_MS          30000UL /* must stay below taper current for 30 s to confirm   */

/* Voltage thresholds for state machine flags */
#define BAT_LOW_MV                2800    /* bat_low flag sets below this                        */
#define BAT_LOW_CLEAR_MV          2900    /* bat_low flag clears above this (100 mV hysteresis)  */
#define BAT_LOW_DEBOUNCE_COUNT    3       /* consecutive readings before flag sets (×50 ms)      */
#define BAT_FULL_MV               3650    /* used in flag evaluation (requires current check too) */

/* Safe mode recovery — battery must reach this before re-enabling loads */
#define BAT_SAFE_RECOVER_MV       3200

/* Absolute limits (fault thresholds) */
#define BAT_OVERVOLT_MV           3700    /* hard fault: disconnect charger immediately           */
#define BAT_OVERVOLT_RECOVER_MV   3400    /* clear overvolt fault below this                     */
#define BAT_UNDERVOLT_MV          2000    /* hard fault: disconnect loads immediately             */
#define BAT_UNDERVOLT_RECOVER_MV  3200    /* clear undervolt fault above this                    */

/* ── Supervised undervolt rescue (see energy_mode.c safe_mode_rescue_*) ──
 *
 * BAT_UNDERVOLT latches at 2000 mV and only clears at 3200 mV, but its
 * protective action (and SAFE_MODE) disable the buck — so nothing could
 * raise the cell back to 3200 and the fault could never clear itself. The
 * rescue re-permits the charge path (loads still shed) while the fault is
 * the SOLE latched fault, there is usable sun, and V_bat is above the hard
 * floor below. power_budget already SoC-gates battery_limit to 200 mA below
 * 3000 mV, so the rescue is inherently a precharge-rate trickle.
 *
 * Hard floor: below this, stay latched — a LiFePO4 this deep is in
 * copper-dissolution territory and must not be recharged unattended. Sits
 * above the BAT_UNDERVOLT detection floor (2000) but well above the 500 mV
 * disconnected-cell sense guard, so 500..1500 mV is the hard-latched band. */
#define BAT_RESCUE_MIN_MV         1500

/* Deep-discharge precharge timeout (ms). A cell that hit BAT_UNDERVOLT this
 * boot enters the charger's PRECHARGE state (V_bat < 3000 on entry) from a much
 * deeper start than a normal precharge — whether via the supervised rescue or
 * after the wake probe cleared the latch — so it gets a longer window before
 * the cell is declared damaged. Expiry escalates to FAULT_PRECHARGE_TIMEOUT
 * (the correct terminal, user-assisted "cell is dead" state) exactly like a
 * normal precharge timeout. 30 min at the 200 mA trickle is gentle enough
 * for a genuinely deep cell yet still gives up on one that won't climb.
 *
 * Selected on fault.history (not the live fault.code) in tick_precharge — see
 * the rationale there. */
#define BAT_RESCUE_TIMEOUT_MS     1800000UL

/* ── Battery protection wake probe (see bat_wake_tick in energy_mode.c) ──
 *
 * When the S-8240 protection IC (U46) opens Q416/Q417 in the battery-NEGATIVE
 * path (battery hot-unplug under debug power, deep overdischarge cutout),
 * cell negative is no longer the ADC ground reference and the single-ended
 * V_BATM channel reads a protection-biased node at ≈0.8 V — NOT the cell
 * voltage. Firmware previously classified that signature as genuine
 * undervoltage and latched a self-blocking lockout: SAFE_MODE sheds the buck,
 * the supervised rescue can't start (reading < BAT_RESCUE_MIN_MV), and the
 * S-8240 never sees the charger connection it needs to close the FETs again.
 * See docs/bug_battery_hotplug_800mv_lockout.md and the 2026-07-12 bench log.
 *
 * The wake probe is a short, energy-limited charger stimulus at a FIXED
 * target (never V_bat + headroom — V_bat is untrusted here), followed by a
 * buck-OFF persistence test: only a real, reconnected cell holds a plausible
 * voltage with the stimulus removed (the buck can drive an EMPTY connector to
 * the commanded voltage, so an on-state reading proves nothing). */

/* Protection-open signature window (mV). Bench 2026-07-12: the node reads
 * 744–896 mV stable, identically for "battery absent" and "battery present
 * but protection open" — the window cannot distinguish the two; only the
 * off-state persistence test can. Board/temperature spread untested, hence
 * the generous margins. Sits inside the 500..2000 undervolt band, above the
 * 500 mV disconnected-sense guard. */
#define BAT_PROT_SIG_MIN_MV       600
#define BAT_PROT_SIG_MAX_MV       1100

/* Collapsed variant of the same signature (mV). Bench 2026-07-29: with the
 * panel absent and the bus fully discharged (board running on XDS110 power
 * alone), the protection-open node has nothing biasing it and reads a hard
 * 0 mV instead of the 744–896 mV above — V_BATM 0 while the cell itself sat
 * at 2.7 V. That reading is BELOW fault_mgr's 500 mV disconnected-cell sense
 * guard, so FAULT_BAT_UNDERVOLT never latches and every recovery path keyed
 * on that fault stays disarmed. Treat it as the same lockout.
 *
 * The gap between this ceiling and BAT_PROT_SIG_MIN_MV is deliberate: a node
 * persistently sitting in 300..600 mV is neither bias-node signature nor a
 * plausible cell, and gets no probe. Like the window above, this cannot
 * distinguish "protection open" from "battery absent" — only the off-state
 * persistence test in BAT_WAKE_VALIDATE can, and it treats both identically
 * (BAT_WAKE_RES_NO_BATTERY, bounded retries, then TERMINAL). */
#define BAT_PROT_SIG_COLLAPSED_MAX_MV 300

/* Consecutive 50 ms ticks the candidate condition must hold before a probe
 * (2 s). Outlasts the 640 ms V_bat moving-average window with margin, so a
 * cell sliding through the window during removal can't start a probe. */
#define BAT_WAKE_DETECT_TICKS     40

/* Output rail must have collapsed (loads shed) for a candidate — a live
 * 3VOUT means loads are (or were just) energized and this is not the
 * quiescent lockout state. */
#define BAT_WAKE_OUT_COLLAPSED_MV 1000

/* First probe target: enough to release a genuinely overdischarged S-8240
 * pack while reducing the voltage step into the low cell. If that does not
 * release a healthy protection-open pack, later attempts use the full CV
 * target below. Both targets are fixed, never derived from the invalid
 * protection-open V_bat reading. */
#define BAT_WAKE_LOW_PROBE_TARGET_MV  BAT_PRECHARGE_MV

/* Full probe target (mV): the LiFePO4 CV limit. Must exceed any healthy
 * cell's resting voltage (≤ ~3.4 V full) so the S-8240's VM pin sees a
 * charger-connection differential and releases; capped at the CV limit so
 * the driven node can never exceed normal charge voltage. NEVER derived
 * from the measured V_bat, which is invalid in the protection-open state
 * (V_bat + headroom would command ≈2.79 V, below the cell — no release). */
#define BAT_WAKE_PROBE_TARGET_MV  BAT_CV_VOLTAGE_MV

/* The charge switch stays open while the buck soft-starts. Require both a
 * minimum delay and an instantaneous VCHG reading close to the selected
 * target before connecting the battery. The timeout converts a failed buck
 * start into a rate-limited retry without ever exposing the battery to a
 * below-Vbat FCCM rail. */
#define BAT_WAKE_BUCK_SETTLE_MS       50UL
#define BAT_WAKE_BUCK_SETTLE_MAX_MS   1000UL
#define BAT_WAKE_BUCK_READY_TOL_MV    75U

/* Probe stimulus duration cap (ms). A protection IC that merely needs
 * charger detection releases promptly; this is NOT a charging window. */
#define BAT_WAKE_PROBE_MS         3000UL

/* End the stimulus early once this much charge current flows (mA) — the
 * charger connection has demonstrably been delivered (body-diode current
 * into a protected cell, or inrush into a just-released one). Reuses the
 * precharge ceiling. The 64-sample MA lags ~320 ms, so a reconnection
 * transient can briefly overshoot this; the panel's power limit and
 * FAULT_OVERCURRENT_CHG bound that overshoot. */
#define BAT_WAKE_PROBE_CUT_MA     BAT_PRECHARGE_MAX_MA

/* Buck-off settle before the persistence test (ms). Must flush the full
 * 640 ms V_bat moving-average window so no probe-driven samples remain;
 * 1500 ms ≈ 2.3 windows. */
#define BAT_WAKE_SETTLE_MS        1500UL

/* Persistence observation window (ms, ≈20 ticks). V_bat must hold a
 * plausible cell voltage for the WHOLE window with the buck off. */
#define BAT_WAKE_VALIDATE_MS      1000UL

/* Minimum off-state voltage accepted as "a real cell is connected" (mV).
 * Equal to the rescue hard floor so a validated cell hands off seamlessly:
 * ≥3200 recovers via the normal fault/SAFE exits, 1500..3200 via the
 * supervised rescue, and a persistent reading below this floor is the
 * terminal deep-cell case (retain the fault, no unattended charging). */
#define BAT_WAKE_VALID_MIN_MV     BAT_RESCUE_MIN_MV

/* Retry pacing and budget. One attempt per panel-relock-style interval,
 * bounded attempts, then terminal until the panel is removed or the fault
 * state changes — never continuous pulsing into an empty connector. */
#define BAT_WAKE_RETRY_MS         HAS_SUN_RELOCK_MS
#define BAT_WAKE_MAX_ATTEMPTS     3

/* =========================================================================
 * 2. SOLAR PANEL / BUCK INPUT LIMITS
 * =========================================================================
 *
 */
// #define PANEL_OVP_CLAMP_MV        15000   /* approximate OVP clamp voltage (from zener + MOSFET)  */

/* Retuned for the 4x-parallel (1s) panel array under real sun: OC ≈ 14 V,
 * MPP measured at/below 7.3 V (panel power still climbing as V drops to
 * 7.3 V — see bench log). The previous 8-10 V band sat ABOVE the MPP, so
 * has_sun cleared (panel < 8 V) within ~400 ms of charge start — faster
 * than CHARGER_MPPT_SETTLE_MS (1 s) — and MPPT never engaged to bring
 * allowed_chg down to the panel's real capability. Charger bounced
 * IDLE<->CHG_ONLY forever. These thresholds now sit below the MPP so the
 * charger stays active long enough for MPPT to TRACK and park at the MPP. */
#define PANEL_MIN_MV              6000    /* has_sun flag sets above this (unloaded panel ≈ OC in IDLE) */
/* has_sun CLEAR threshold. Must sit BELOW the charging V_panel floor chain:
 *   vreg band low (5300) > PANEL_SAFETY_MV (4800) > this (4200) >
 *   collapsed-panel reading ≈ 3.6 V (buck in dropout: V_panel ≈ V_bat + drop).
 * A collapsed panel reads ~3.6 V, NOT 0 V, so only the clear COUNT below
 * separates "transient collapse during regulation" from "sunset". */
#define PANEL_MIN_CLEAR_MV        4200
#define HAS_SUN_DEBOUNCE_COUNT    3       /* consecutive readings before flag sets (×50 ms)       */
/* Consecutive sub-PANEL_MIN_CLEAR_MV readings before has_sun clears (×50 ms).
 * Must outlast the worst-case activation transient: the pre-position
 * over-demand can collapse the panel for ~2 paced backoff intervals plus
 * ~320 ms of ADC filter lag on each edge ≈ 1.2 s of sub-threshold readings.
 * 30 ticks = 1.5 s rides that out; a real sunset still tears the charger
 * down within ~1.5 s, which is plenty fast. */
#define HAS_SUN_CLEAR_COUNT       30
/* "Usable sun" power floor. While the charger is actively loading the panel
 * (energy mode CHARGE_ONLY / CHARGE_AND_LOAD), if panel output power stays
 * below this for HAS_SUN_CLEAR_COUNT ticks, has_sun clears even though
 * V_panel may still float near Voc. Catches the dead-but-floating panel at
 * dusk: the charger backs all the way off (PWM rails at PWM_MIN_DUTY), the
 * unloaded panel sits at ~Voc (> PANEL_MIN_MV) so the voltage-only clear
 * never fires, yet P_panel ≈ 0 and the battery bleeds out through the
 * housekeeping boosts. Must be well below any real charging power (a few
 * hundred mW even during CV taper / precharge trickle) so it never
 * false-trips a healthy charge — the 1.5 s clear COUNT rides out transients. */
#define PANEL_USABLE_MIN_MW       200
/* After a panel-power clear of has_sun, suppress the V_panel-based re-set for
 * this long so we re-probe the panel roughly once a minute instead of
 * hammering the charger on/off every ~1.6 s — the unloaded panel floats back
 * above PANEL_MIN_MV the instant we stop loading it, which would otherwise
 * re-set has_sun immediately and re-activate the buck. Mirrors MPPT HOLD. */
#define HAS_SUN_RELOCK_MS         60000UL
/* Dusk detector debounce (×50 ms), SEPARATE from HAS_SUN_CLEAR_COUNT. The
 * "panel floats near Voc yet delivers < PANEL_USABLE_MIN_MW" test runs on its
 * OWN counter (ctx->has_sun_dusk_count) so a transient regulation collapse
 * can't chain its voltage-collapse ticks and its near-OC recovery-tail ticks
 * onto one 30-tick clear — the failure that turned a ~1 s MPPT collapse into a
 * full charger teardown + fresh re-climb (the observed charge-on/off cycle).
 * Must outlast the inner loop's worst-case near-OC re-acquire after a
 * panel_safety_backoff overshoot (LOAD_REACQUIRE_MA walks the operating point
 * back onto load in ~1–2 s), with margin. 80 ticks = 4 s rides that out yet
 * still calls a genuine dusk within a few seconds (the 60 s relock throttles
 * re-probing regardless). */
#define HAS_SUN_DUSK_CLEAR_COUNT  80
/* Emergency floor: regulation backs off hard below this. Must be BELOW the
 * vreg band low edge (setpoint − deadband = 5300 mV) so it never fires while
 * the loop holds the panel on its power plateau, and ABOVE the has_sun clear
 * threshold so backoff acts before the FSM gives up on the sun. */
#define PANEL_SAFETY_MV           4800

/* =========================================================================
 * INPUT-VOLTAGE REGULATION ("constant-voltage MPPT")
 * =========================================================================
 *
 * Master switch for the charger's bulk-charge control law. When 1, the
 * bulk regulator (charger.c cc_regulate, used in PRECHARGE and CC) holds
 * V_panel at PANEL_VREG_SETPOINT_MV instead of chasing a fixed charge-
 * current target, and the perturb/observe MPPT region (mppt.c) is kept
 * DISABLED so it does not grab PWM and fight the voltage loop.
 *
 * Why: fixed-current CC is open-loop UNSTABLE on a soft PV source — any
 * target above the panel's MPP current drags V_panel past the I-V knee
 * and collapses it, which (a) clears has_sun and tears the charger down,
 * and (b) cannot be rescued by perturb-MPPT because arming it (via the
 * panel_limited margin) requires commanding well past the MPP in the
 * first place. Regulating the INPUT voltage instead is negative feedback
 * on a monotonic plant (more current → lower V_panel), so it converges
 * to the MPP from either side and cannot walk off the cliff. It also
 * degrades gracefully: on a stiff source V_panel never reaches the
 * setpoint, so the battery-current clamp takes over and it behaves like
 * classic CC. See the bench-log saga in git history for the full path
 * that led here.
 *
 * MPPT under this mode: mppt.c runs as an OUTER loop that owns the
 * SETPOINT (ctx->mppt.vreg_setpoint_mv) and hill-climbs it to maximise
 * delivered charge current — it never touches ctx->pwm. The legacy
 * PWM-perturbing inc-conductance tracker is compiled only when this
 * switch is 0. See section 6 (MPPT TUNING). */
#define CHARGER_INPUT_VREG        1

/* MPP voltage setpoint COLD-BOOT FALLBACK (mV). The LIVE setpoint is
 * ctx->mppt.vreg_setpoint_mv: seeded at charger activation from the
 * captured open-circuit voltage (MPPT_SP_FRACTION_PCT of Voc) and then
 * hill-climbed by the outer MPPT loop (mppt.c). This constant only
 * matters until the first activation, or if the Voc capture is invalid.
 *
 * Why no fixed setpoint: 6500 mV was the measured MPP of the 4x-parallel
 * array (flat top, ≥97 % of peak from ~5.0–7.8 V). On a 13 V OC panel
 * (bench log 1206_sp.log, 2026-06-12) the same value put the regulation
 * band top (7.7 V) BELOW that panel's I-V knee (~10.9 V), so the loop
 * had no reachable operating point, walked the panel over the knee,
 * collapsed it to 3.4 V, and sawtoothed collapse/recover every ~9 s.
 * The setpoint is a per-panel quantity and must follow the panel. */
#define PANEL_VREG_SETPOINT_MV    6500U

/* Half-width of the regulation deadband around the setpoint (mV). The
 * panel's power top is FLAT (≥97 % of peak from ~5.0–7.8 V), so the band
 * is deliberately wide: 6500 ± 1200 → hold anywhere in 5.3–7.7 V.
 *
 * Why this wide: PWM is COARSE on this plant. One count moves the buck
 * rail ~2.9 mV (LUT span 3772→2786 mV over 343 counts), which across the
 * ~65 mΩ charge path is ~45 mA of battery current ≈ ~25 mA of panel
 * current at 6.5 V (measured, May bench log: pwm 143→140 moved Ichg
 * −180→−38 mA). The band must span MORE panel current than one step
 * quantum, or there may be NO reachable operating point inside it — the
 * loop then hops across the band every interval and whipsaws the panel
 * over its I-V knee (the residual bounce seen after the pacing fix).
 * 5.3–7.7 V on the flat top spans ~47 mA panel-side ≈ 2 PWM counts, so a
 * landing point always exists, and any in-band point is ≥97 % of MPP. */
#define PANEL_VREG_DEADBAND_MV    1200U

/* PWM counts per regulation step. ONE count (~45 mA charge-side, ~25 mA
 * panel-side — see deadband note) is already ~20 % of this array's MPP
 * current. The previous 2-count step out-jumped the entire usable
 * current window of the old ±400 mV band and could leap from "barely
 * loaded" straight past the knee. Finer is not available; coarser
 * cannot park. */
#define PANEL_VREG_STEP           1

/* Minimum spacing between regulation steps (ms). MUST be ≥ the panel-ADC
 * moving-average group delay (~320 ms) or the loop accumulates dead-time
 * overshoot and oscillates the panel across its I-V knee (the bench-log
 * bounce — happens in EITHER sign direction). At 400 ms each step settles
 * through the filter before the next observation. Convergence from V_oc
 * to the setpoint is a handful of steps (≈1-2 s), then it holds. */
#define PANEL_VREG_INTERVAL_MS    400UL

/* Minimum-load re-acquire threshold (mA, charge-side). If cc_regulate finds
 * V_panel INSIDE the regulation band yet chg_current below this, the buck is
 * parked near open circuit delivering ~nothing — the tail of a
 * panel_safety_backoff overshoot that the wide deadband would otherwise hold
 * indefinitely (near-OC sits inside setpoint ± PANEL_VREG_DEADBAND_MV, so
 * neither regulation branch corrects it). The loop then steps toward more
 * current (pwm DOWN, one paced step) to re-acquire the MPP instead of holding.
 * In real sun this self-terminates the instant delivered current returns above
 * the threshold; on a dead panel at dusk it loads the panel down until the
 * voltage-collapse has_sun clear fires. ~60 mA ≈ PANEL_USABLE_MIN_MW (200 mW)
 * at V_bat, so the re-acquire keeps delivered power above the dusk floor. */
#define LOAD_REACQUIRE_MA         60

/* =========================================================================
 * 3. BUCK CONVERTER (TPS56347)
 * =========================================================================
 *
 * The buck is controlled via PWM on the FB pin.
 *   Timer period = 400 counts
 *   Compare value written = (400 - pwm)
 *   Duty cycle = (400 - pwm) / 400
 *
 *   pwm = 1   → duty = 399/400 = 99.75%  (maximum current)
 *   pwm = 399 → duty = 1/400   = 0.25%   (minimum current, effectively off)
 *
 * IMPORTANT: lower pwm value = higher duty = more current.
 */
#define PWM_PERIOD                400
#define PWM_MAX_DUTY              1       /* lowest pwm value → highest duty cycle                */
#define PWM_MIN_DUTY              399     /* highest pwm value → lowest duty cycle (off)          */
#define BUCK_MAX_CURRENT_MA       2000    /* hardware current limit of the inductor/FET           */

/* Nominal MCU supply. The FB-injection network converts the PWM pin's AVERAGE
 * voltage (VDD x duty) into a buck setpoint, and the LED channels drive a
 * transconductance source the same way, so every duty in this firmware is
 * really a request for a fraction of VDD. scale_duty_cycle() divides the
 * requested duty by the measured rail to hold that fraction constant when the
 * rail drifts; this is the value it scales back to, and the fallback get_vdd()
 * reports before the ADC average exists. */
#define VDD_NOMINAL_MV            3300U

/* LED boost (TPS61088) rail target. Needs enough headroom for the LED string
 * Vf plus Vce_sat of the PNP current source; below ~10 V the per-channel CC
 * loop falls out of regulation and the lamps only weakly glow at <<150 mA.
 * Matches V2.5.5: set_led_voltage()'s linear formula saturates this request
 * to duty=1 (rail clamps to its top of ~11.3 V), which is the validated
 * operating point. */
#define LED_BOOST_TARGET_MV       11500

/* Drive current when a button-controlled lamp (any of LED1–LED4 / LEDCTRL1–4)
 * is switched ON. Lands on the 145/155 mA LUT bins (output_currents_led_mA), well under
 * the 350 mA per-channel hardware max. Switching a lamp OFF commands 0 mA,
 * which set_led_current() drives to compare 99 (~15 mA, the dimmest in-range
 * value on the inverted-polarity channel). compare==period is forbidden and
 * runs away, so "off" is a faint floor, not fully dark — see that fn. */
#define LAMP_ON_CURRENT_MA        150

/* Lamp press/hold brightness UI (lamp_buttons_update() in main.c). A short tap
 * toggles the lamp full-on / off; press-and-hold dims one level every
 * LAMP_DIM_STEP_MS and STOPS at LAMP_DIM_MIN_LEVEL — a hold only ever changes
 * brightness, it never switches the lamp off (holding past the floor just parks
 * at the dimmest on-level; use a tap to turn the lamp off). There are
 * LAMP_DIM_LEVELS - LAMP_DIM_MIN_LEVEL steps between full and the floor, so a
 * continuous hold from full reaches minimum in about 3.2 s. Per-level LED
 * currents live in lamp_level_ma[] in main.c (index LAMP_DIM_LEVELS = full =
 * LAMP_ON_CURRENT_MA, index 0 = off, reachable by tap only). The dim cadence is
 * paced off the button's pressStartTime via time_now() with a catch-up loop, so
 * LAMP_DIM_STEP_MS need not be an exact multiple of TICK_BUTTON_MS (steps just
 * land on the nearest button poll).
 * The LED-current timers use an 800-count period. set_led_current() linearly
 * interpolates the existing 100-count calibration LUT onto that finer PWM grid,
 * giving every one of these 40 levels a distinct current-reference duty. */
#define LAMP_DIM_LEVELS           40
#define LAMP_DIM_STEP_MS          83
#define LAMP_DIM_MIN_LEVEL         1

/* =========================================================================
 * 4. LOAD DETECTION
 * =========================================================================
 *
 * "has_load" is based on measured discharge current.
 * The discharge shunt (R432) has a ~45 mA zero-offset that wanders up to
 * ~50 mA with noise, so BOTH thresholds must sit clear of that floor — the
 * old 50/30 window straddled the offset, so a noise spike latched has_load
 * and the offset then held it above the 30 mA clear point forever (phantom
 * CHG+LOAD with nothing plugged in). Any real output/USB load is >=100 mA.
 */
#define LOAD_DETECT_MA            120     /* I_load above this → has_load = true                  */
#define LOAD_DETECT_CLEAR_MA      80      /* I_load below this → has_load = false (hysteresis)    */

/* =========================================================================
 * 5. CHARGER TUNING
 * =========================================================================
 *
 * CC regulation with deadband.
 *   Every tick, compare I_charge to target (allowed_chg).
 *   If outside deadband → adjust PWM by 1 step.
 *   If inside deadband → do nothing (stable).
 *
 * CV regulation: same approach but on voltage.
 */
#define CC_DEADBAND_MA            25      /* ±25 mA around target before adjusting                */
#define CC_PWM_STEP               1       /* PWM adjustment per regulation cycle                  */

/* Charger activation headroom (mV added to V_bat when pre-positioning PWM
 * on entry from CHG_INACTIVE). The buck-output LUT
 * (output_voltages_buck_mV[]) targets the buck rail node, NOT the VCHG
 * pin at the battery side of Q49 (CHG-switch P-FET, ~50 mΩ Rds_on).
 * Pre-positioning to exactly V_bat leaves zero forward bias: the
 * TPS564247 sees FB above its 0.6 V ref, drops duty, and its sync FETs
 * reverse-pump inductor current into V_panel.
 *
 * But the headroom is also an instant CURRENT command: the charge path
 * is only ~65 mΩ end-to-end, and the LUT-vs-reality zero-current offset
 * measured +24 mV (May bench log: Ichg crossed 0 with the LUT target
 * 24 mV above V_bat; slope ~45 mA per 2.9 mV count). The previous
 * 200 mV pre-positioned a ~2.7 A demand — a guaranteed instant collapse
 * of a 0.8 W panel at EVERY charge activation, which is what kept the
 * IDLE↔CHG_ONLY bounce alive even after the vreg loop was paced. 50 mV
 * clears the measured offset with ~2× margin (forward bias guaranteed)
 * while commanding only ~400 mA: one paced panel-safety backoff step
 * recovers that on a small panel, and on a stiff source it is simply a
 * gentle starting current for the loop to ramp from. */
#define CHG_ACTIVATION_HEADROOM_MV 50U

/* Staged normal activation. The buck runs unloaded with Q49 open until its
 * instantaneous VCHG reading is at least this far above both the activation-
 * time and live instantaneous V_bat readings. The LUT is only a starting
 * estimate: VDD, board, and load-condition error can move the actual rail by
 * hundreds of millivolts. While Q49 is open, trim toward readiness slowly
 * using raw VCHG feedback. Two PWM counts are about 6 mV around the battery
 * range, so the final step cannot turn the readiness margin into a large
 * connection-current command. */
#define CHG_BUCK_READY_MARGIN_MV    20U
#define CHG_BUCK_SETTLE_MS          50UL
#define CHG_BUCK_SETTLE_RAMP_MS     TICK_MAIN_MS
#define CHG_BUCK_SETTLE_PWM_STEP    2

/* CC down-step rate limit. The chg_current ADC uses a 64-sample moving
 * average at 10 ms tick → ~320 ms group delay. If CC steps pwm DOWN
 * (more current) every 50 ms tick, it will walk 6+ counts past the
 * point of regulation before the filtered measurement catches up —
 * fine for a current-limited PV panel where V_panel collapse triggers
 * panel_safety_backoff, but on a stiff source the buck will overshoot
 * past FAULT_OVERCURRENT_CHG_MA before any feedback arrives. Allow at
 * most one down-step per CC_DOWNSTEP_INTERVAL_MS so the regulator
 * never gets ahead of the measurement. UP-steps (back off) remain
 * every tick — over-current reaction must stay fast. */
#define CC_DOWNSTEP_INTERVAL_MS   300UL

#define CV_DEADBAND_MV            5       /* regulate between CV_VOLTAGE and CV_VOLTAGE + this    */
#define CV_PWM_STEP               1

/* Panel safety: if V_panel drops below PANEL_SAFETY_MV during charging,
 * increase PWM (decrease duty) by this many steps to back off quickly.
 * Under CHARGER_INPUT_VREG this fires at most once per
 * PANEL_VREG_INTERVAL_MS (see panel_safety_backoff in charger.c);
 * 5 counts ≈ −215 mA of demand per step on the ~65 mΩ charge path. */
#define PANEL_BACKOFF_STEP        5

/* Reverse-current escape threshold (mA). chg_current more negative than
 * −this means the buck rail is parked below V_bat and the sync FETs are
 * pumping battery charge into the panel; cc_regulate then walks pwm DOWN
 * by PANEL_BACKOFF_STEP per paced interval to lift the rail back above
 * V_bat quickly instead of waiting on the 1-count V_panel loop. */
#define CHG_REVERSE_CURRENT_MA    100

/* Input-loss detection (charger_input_guard, charger.c). Trips when the
 * INSTANTANEOUS V_panel falls below V_bat + this margin.
 *
 * A buck only steps down: it holds VCHG at the target while V_panel stays
 * above it, and once V_panel falls to ≈V_bat the FCCM converter can no longer
 * support its output and the sync FETs start pushing cell charge back into the
 * dying input. That crossover is what this margin sits above.
 *
 * Note the crossover tracks V_BAT, not the FB pwm value — pwm sets the buck's
 * TARGET VOLTAGE through the FB-injection network, it is not the converter's
 * switching duty (which is internally D = VCHG/V_panel ≈ 0.28 while charging).
 * So the threshold must be relative to V_bat, and it stays correct across the
 * whole cell range including a 1.5 V rescue precharge.
 *
 * Why panel voltage and not VCHG: while charging at 2 A the whole VCHG-to-V_bat
 * separation is ~44 mV (bench: Vbat 3570 / Vchg 3614), far too tight to judge
 * from a single unaveraged conversion. V_panel sits ~9 V clear of V_bat over
 * the same interval, so this test has volts of noise margin and can act on one
 * 10 ms sample.
 *
 * SIZED TO STAY BELOW PANEL_SAFETY_MV (4800). A sagging-but-present panel is
 * panel_safety_backoff's regime — it steps demand down and recovers. If this
 * guard could fire up there it would answer an ordinary over-draw with a full
 * charger teardown and re-arm, reintroducing exactly the charge-on/off limit
 * cycle v0.21 fixed. 400 mV keeps the trip at 1.9–4.05 V across the cell range,
 * always clear of the 4800 mV backoff floor.
 *
 * The cost of staying under that floor is that the trip lands close to the
 * crossover rather than well above it, so on a hard collapse this pre-empts the
 * reversal only sometimes — it always bounds the exposure to one 10 ms sample,
 * and charger_fast_guard remains the guaranteed backstop. */
#define CHG_INPUT_LOST_MARGIN_MV  400U

/* Input-collapse handling window (charger_input_guard, charger.c).
 *
 * v0.32 changed what a sub-margin reading MEANS. Bench 10.09.26 (v0.29 and
 * v0.30, 18 INTRACE dumps): every trip is a live panel pushed over its knee
 * — steady 11-12 V, one sample of descent, pinned at V_bat + ~190 mV, and
 * back at Voc within 400 ms of the stand-down. Not one removal. A panel
 * sitting at V_bat + 190 mV is still pushing Isc through the buck (in
 * dropout) into the cell: it is forward current, not a reverse-pump
 * condition, and it recovers the instant the draw drops below Isc
 * (input cap recharges in ~ms). Standing the region down for that cost
 * ~15 s of full sun per event (2 s re-arm + settle + a 25-count inner-
 * loop walk back).
 *
 * So the guard now backs the draw off by CHG_INPUT_RECOVER_STEP on EVERY
 * counted sub-margin conversion and stands down only when this many have
 * gone by with the input still under the crossover. A live collapse comes
 * back after the first or second step and charging continues at the
 * backed-off point (cc_regulate walks it back, MPPT learns the level via
 * input_dip_events); a removed source never comes back and is isolated on
 * the third.
 *
 * Counts OBSERVED conversions (gated on adc_sample_seq), and the blocking
 * 5 Hz telemetry TX hides whole conversions from the foreground, so the
 * window is 30-70 ms in practice (v0.29 measured 3-7 conversions, median
 * 4). Counting elapsed conversions would be worse — after a TX block one
 * reading would carry several samples' credit. Under-counting is the safe
 * direction.
 *
 * Exposure on a GENUINE removal: this guard no longer pre-empts at the
 * voltage crossover; the source falls through V_bat during the window and
 * the reverse current that follows is caught by charger_fast_guard on its
 * first ≥100 mA sample, which classifies a dead input as the same clean
 * stand-down (v0.26). That is the "narrowly lost the race" path the design
 * already tolerated; it is now the only path. If FAULT_REVERSE_PUMP starts
 * appearing on unplugs, that is the regression to report. */
#define CHG_INPUT_LOST_SAMPLES    3U

/* PWM counts of backoff per counted sub-margin conversion (~45 mA/count
 * of battery current: 10 counts ≈ 450 mA, roughly half of a 1 A session).
 * Enough to drop the draw under Isc for an ordinary transient shadow or a
 * knee-instability dip; too little for a heavy shadow, which then falls
 * through to the stand-down as before. The resume cost is the walk back,
 * one count per PANEL_VREG_INTERVAL_MS, so 10 counts ≈ 4 s at reduced
 * current — against ~15 s at zero. Applied twice at most (samples 1 and 2)
 * before the stand-down on sample 3. */
#define CHG_INPUT_RECOVER_STEP    10U

/* How long after charger_input_guard's last backoff that reverse current is
 * read as the rescue's own tail rather than as reverse pumping
 * (charger_fast_guard).
 *
 * A rescued collapse ends with the panel back at full voltage while the buck
 * still carries the backed-off target, so for a few conversions "reverse
 * current with a live input" — the exact signature the fault exists for — is
 * what SUCCESS looks like. Bench 10.09.26 (v0.33): four collapses, four
 * INTRACE RECOVERED lines, and FAULT_REVERSE_PUMP (0x0100) latched beside
 * every one, costing FAULT_RECOVER_WAIT_MS (10 s) each.
 *
 * 200 ms covers the collapse-to-recovery transient (~40 ms) plus the 64-sample
 * current average settling behind it, and is far shorter than any genuine
 * reverse-pump event, which is a sustained state, not a transient. Inside the
 * window the guard corrects the cause and then stands down cleanly if that
 * did not take; it never latches. */
#define CHG_REVERSE_BLANK_MS      200UL

/* ── Learned PWM ceiling (the current-domain cliff memory) ──────────────
 *
 * sp_session_floor_mv learns a collapse in the SETPOINT domain, and on a
 * fading panel that memory runs out of authority. Bench 10.09.26 (v0.30,
 * 36 min of late-afternoon sun, 13 teardowns): the last SEVEN all fired at
 * pwm 106 with V_panel 12736 and the setpoint pinned at 11576 — its
 * MPPT_SP_VOC_GUARD_MV ceiling. Inside the ±PANEL_VREG_DEADBAND_MV band
 * cc_regulate holds PWM, and the setpoint can never legally rise to
 * V_panel + deadband (that is above Voc), so raising the floor further
 * commands no backoff at all. Meanwhile Isc keeps falling under a frozen
 * draw, and the panel goes over its knee again at exactly the same count.
 *
 * So learn the cliff where the actuator actually lives. Every collapse —
 * rescued by charger_input_guard or torn down — raises cliff_pwm_min to
 * CHG_CLIFF_PWM_MARGIN counts above the PWM that fell over, and cc_regulate
 * may not step below it. One margin is ~270 mA of demand: a real notch at
 * the 300-900 mA the bench session was delivering, and small enough that
 * the ratchet lands near the knee instead of parking well under it (the
 * observed drift was 13 counts before the draw stopped moving — 2-3
 * events' worth). Backoffs,
 * the reverse-current escape and the CHG_BUCK_SETTLE ramp all ignore the
 * floor: it bounds current, and every one of those REDUCES current or is
 * an interlock that must not be blockable. */
#define CHG_CLIFF_PWM_MARGIN      6U

/* The ratchet is one-way per event, so it needs a way back down as the sun
 * comes back: release one count per this interval of collapse-free charging
 * (mppt.c).
 *
 * It must be slower than the drift it is fencing or it just cancels the
 * ratchet. Replaying the 13 teardowns of the 10.09.26 session (pwm at
 * collapse 93 → 106 as the sun went down) through the ratchet: at one count
 * per 60 s the fence is eaten between events and catches 9 of them; at 180 s
 * it catches 12 — the first is unlearnable by construction, so that is all
 * of them. Slower buys nothing more, and 180 s still returns 20 counts
 * (~900 mA) per hour, a passing cloud's worth of fence in a few minutes.
 *
 * A new day never inherits yesterday's fence regardless: the overnight IDLE
 * clears has_sun, which invalidates the seed, and enter_tracking_fresh
 * releases the ceiling with the rest of the learned panel. The relax only
 * covers within-session recovery that never drops has_sun — a thin haze
 * clearing, not a sunrise. */
#define CHG_CLIFF_PWM_RELAX_MS    180000UL

/* Ceiling on how far a warm resume may advance the activation pre-position
 * (energy_mode.c). The staged start normally enters at the LUT count for
 * V_bat + CHG_ACTIVATION_HEADROOM_MV and lets the inner loop walk down to
 * the operating point at one count per PANEL_VREG_INTERVAL_MS — ~2.5 s of
 * the ~4.5 s an input-loss bounce costs, spent re-deriving a number the
 * previous second already knew. Resuming at the learned ceiling skips that
 * walk, but the pre-position also bounds the inrush when CHG_BUCK_SETTLE
 * closes Q49: at ~2.87 mV/count and ~65 mΩ of path, 10 counts of advance
 * puts the rail ~79 mV over the cell instead of 50, i.e. ~1.2 A of first
 * connection against the 2 A budget. That is the whole allowance. */
#define CHG_RESUME_MAX_ADVANCE    10U

/* Depth of the raw-V_panel ring buffer captured for post-mortem when the
 * guard trips (SPCBoardAPI.c). 16 conversions = 160 ms of history at
 * TICK_ADC_MS, enough to see whether a trip was one bad conversion
 * (isolated outlier between healthy neighbours) or a real electrical dip
 * (smooth descent over 3-4 samples). Diagnostic only — nothing regulates
 * on it. */
#define ADC_PANEL_TRACE_DEPTH     16U

/* How long after an input-loss stand-down before energy_mode may re-arm the
 * charger region (charger_rearm_due, energy_mode.c).
 *
 * Long enough that a genuine disconnect is instead resolved the normal way —
 * has_sun clears after HAS_SUN_CLEAR_COUNT (30 ticks = 1.5 s) and the mode
 * leaves CHARGE_ONLY on its own. Short enough that a brief dropout, which
 * never holds V_panel down long enough to clear has_sun, costs ~2 s of charging
 * instead of the 10 s FAULT_RECOVER_WAIT_MS the old latched path imposed. */
#define CHG_INPUT_REARM_MS        2000UL

/* How long the charger region may stay inactive before MPPT treats the
 * learned Voc and MPP setpoint as stale and re-seeds from FOCV
 * (mppt.c, MPPT_DISABLED entry).
 *
 * Before v0.28 EVERY resumption re-seeded, which silently undid
 * enter_disabled()'s deliberate preservation of the learned point: bench
 * 10.09.26 shows 30 of 51 input-loss bounces dropping the setpoint from
 * ~11.5 V straight back to ~8.65 V (= 0.76·Voc − deadband), then taking a
 * median 12.4 s to climb back to full current — six times the 2 s the
 * re-arm block itself costs. Worse, it self-perpetuates: while the
 * setpoint sits 3 V below the knee the inner loop walks the panel off the
 * cliff again, which is why those teardowns arrive in tight bursts.
 *
 * Sized well above CHG_INPUT_REARM_MS (2 s) so a bounce resumes from the
 * converged point, and above FAULT_RECOVER_WAIT_MS (10 s) so a fault
 * round-trip does too — but short enough that a panel swap or a sunrise
 * relearns. The other, faster invalidator is has_sun clearing, which is
 * what a genuine disconnect trips (HAS_SUN_CLEAR_COUNT = 1.5 s). */
#define MPPT_RESEED_GAP_MS        60000UL

/* =========================================================================
 * 6. MPPT TUNING
 * =========================================================================
 *
 * Two trackers exist; CHARGER_INPUT_VREG selects which one is compiled:
 *
 *   =1 (current): SETPOINT-P&O. mppt.c perturbs the input-vreg setpoint
 *      (ctx->mppt.vreg_setpoint_mv) and observes averaged delivered
 *      charge current. The inner voltage loop (charger.c cc_regulate)
 *      keeps exclusive PWM ownership and realises each setpoint — all
 *      of its protections (pacing, panel backoff, reverse escape,
 *      allowed_chg clamp) stay active while MPPT probes. Constants:
 *      the MPPT_SP_* block below.
 *
 *   =0 (legacy): PWM-perturbing incremental conductance with adaptive
 *      step. Kept for stiff-source bring-up / regression comparison.
 *      Constants: MPPT_MAX_STEP_SIZE .. MPPT_STUCK_TICK_LIMIT below.
 *      Known-unworkable on soft panels: one PWM count is ~20 % of a
 *      small panel's MPP current, and the ~320 ms ADC lag makes dV/dI
 *      attribution unreliable (see CHANGELOG / git history).
 */

/* ── Setpoint-P&O (CHARGER_INPUT_VREG=1) ────────────────────────────────── */

/* FOCV seed: Vmpp ≈ this % of Voc. The seed only needs to land in the
 * basin — the hill-climb refines it from there — but it must not land
 * PAST the knee, because the inner loop parks just under the band TOP
 * (= seed + PANEL_VREG_DEADBAND_MV = k·Voc), so k is the voltage the
 * plant is actually driven to.
 *
 * 76 was wrong on this panel class and was the whole of the 09.09.26
 * failure. Measured MPP/Voc on the bench panel: 0.87 (09.09.26) and
 * 0.89 (10.09.26, Voc 13307 mV / best point 11846 mV @ 4134 mW) — two
 * different irradiance conditions, same ratio. 0.76·Voc lands ~1.5 V
 * past the knee in the constant-current region, where the buck is a
 * constant-power load and the operating point cannot be held: the loop
 * stepped pwm down one count per PANEL_VREG_INTERVAL_MS until the panel
 * fell over, 2030 times in 100 minutes.
 *
 * 87 sits just under both measurements (textbook k for crystalline
 * silicon is 0.71–0.82; this panel is stiffer than the textbook). It is
 * only the STARTING point — P&O still hill-climbs from here, and since
 * v0.28 the seed only runs at cold boot or after MPPT_RESEED_GAP_MS, so
 * a wrong value costs one climb rather than one per teardown. */
#define MPPT_SP_FRACTION_PCT      87

/* INITIAL setpoint perturbation per dwell (mV). Must move the parked
 * operating point by clearly more than one PWM count so each probe
 * produces a measurable current change: near the 13 V panel's knee one
 * count moves V_panel ~400 mV (log: pwm 135→132 moved 12.22→10.91 V),
 * and on the 4x array's flat top one count is ~1.2 V. 1000 mV ≈ 1–2.5
 * counts of operating-point movement on both known plants.
 *
 * The step HALVES on each direction reversal down to MPPT_SP_STEP_MIN_MV
 * (adaptive step, like the legacy tracker): coarse steps find the
 * parkable region fast, fine steps then bracket the knee at sub-PWM-count
 * resolution — a fixed 1 V step can over-jump the single best parkable
 * count near a steep knee (simulation: cost ~10 % of MPP). */
#define MPPT_SP_STEP_MV           1000U
#define MPPT_SP_STEP_MIN_MV       250U

/* Dwell timing. After moving the setpoint, the inner loop needs up to
 * ~3 steps × PANEL_VREG_INTERVAL_MS (400 ms) to walk there, plus the
 * 640 ms ADC moving-average window to settle — so 2 s of settle before
 * the observation window opens. Then average chg_current over 1 s
 * (20 ticks of the 64-sample MA) for the fitness comparison. Total
 * cost: 3 s per probe, ~6–8 probes per tracking session. */
#define MPPT_SP_SETTLE_MS         2000UL
#define MPPT_SP_MEASURE_MS        1000UL

/* Improvement threshold (mA) between dwell averages. Below this the two
 * setpoints are considered equal-power: the tracker reverses instead of
 * walking on noise — this is what makes it converge (not random-walk)
 * on a flat power top, and freeze when the allowed_chg clamp (not the
 * panel) is what bounds the current. ~1/3 of one PWM count's worth of
 * battery-side current (45 mA/count). */
#define MPPT_SP_MIN_DELTA_MA      15

/* Direction reversals before declaring convergence → HOLD. Each flat or
 * worse dwell counts one; with 3-s dwells, convergence from a good seed
 * is ~3–5 dwells (~10–15 s). */
#define MPPT_SP_CONVERGE_REVERSALS 3

/* Hard runtime cap on one TRACKING session → HOLD with whatever we
 * have. Generous: seed + full walk across the clamped setpoint range
 * (≈5 steps) + reversals at 3 s each fits comfortably. */
#define MPPT_SP_RUNTIME_MS        45000UL

/* Setpoint floor (mV): keeps the regulation band's LOW edge
 * (setpoint − PANEL_VREG_DEADBAND_MV) at least 500 mV above the
 * PANEL_SAFETY_MV emergency backoff floor, preserving the original
 * vreg-band ordering (band low 5300 > safety 4800). Evaluates to 6500. */
#define MPPT_SP_MIN_MV            (PANEL_SAFETY_MV + PANEL_VREG_DEADBAND_MV + 500U)

/* Voc-relative setpoint floor (% of the learned Voc). MPPT_SP_MIN_MV above
 * is a SAFETY-ordering floor, not a plausibility one: on the 13.3 V bench
 * panel it lets the tracker probe down to 6.5 V, 5.3 V under the measured
 * MPP, and bench 10.09.26 v0.29 shows it doing exactly that (sp range
 * 6500..11554 in one session). No crystalline-silicon MPP sits below
 * ~0.7·Voc, so anything under this is a guaranteed walk over the knee.
 *
 * Applied to the band TOP like the seed (floor_sp = k·Voc − deadband, so
 * the realised operating point cannot be commanded below k·Voc). 80 fences
 * a textbook 0.76 panel ~4 % above its MPP (a few % of power, stable) and
 * lifts this panel's floor from 6500 to ~9260 mV. Only active once a
 * credible Voc exists; MPPT_SP_MIN_MV still applies underneath. */
#define MPPT_SP_FLOOR_PCT         80

/* Cliff learning from an input-loss teardown (mppt.c sp_learn_cliff).
 *
 * The collapse that ends a charging session takes 10-20 ms to reach V_bat
 * and charger_input_guard tears the region down ~40 ms in — the 640 ms
 * panel average never gets anywhere near PANEL_SAFETY_MV, so neither the
 * collapse branch nor the dip classifier ever fires, and until v0.30 the
 * tracker resumed with no memory that the level it was at had just failed.
 * Bench 10.09.26 v0.29: 13 of 15 teardowns followed a downward probe within
 * 6 s; zero collapse-branch corrections in 14 minutes.
 *
 * On such a teardown the floor is set from the last CLEAN averaged V_panel
 * (see MPPT_SP_VPANEL_DROP_MV) so the band top lands this much above the
 * voltage the plant was at when it fell over. One fine step: the average
 * lags the real voltage by ~one PWM count near the knee (~300-400 mV of
 * walking), so the true collapse point is already a little below the
 * recorded one and this margin lands the band top a few hundred mV clear
 * of it. If that is still under the knee the next teardown ratchets it
 * again — converges in one or two instead of never. */
#define MPPT_SP_CLIFF_MARGIN_MV   250U

/* Largest tick-to-tick FALL of the averaged V_panel that still counts as
 * a clean reading for cliff learning. A collapsed 10 ms sample pulls the
 * 64-deep average down by ~V_panel/64 ≈ 125-190 mV; normal regulation
 * near the knee moves it ~50 mV per 50 ms tick. Above this the reading
 * already contains the collapse and would place the floor too low. */
#define MPPT_SP_VPANEL_DROP_MV    300U

/* Setpoint ceiling guard (mV below captured Voc): the band TOP
 * (setpoint + deadband) must stay meaningfully below Voc, or the inner
 * loop can "park" at open circuit drawing zero current. Ceiling =
 * Voc − this. 300 mV of real load below OC plus the band half-width. */
#define MPPT_SP_VOC_GUARD_MV      (PANEL_VREG_DEADBAND_MV + 300U)

/* Collapse blanking (ms) from dwell start. A probe below the panel's
 * knee collapses V_panel (< PANEL_SAFETY_MV); the tracker must react
 * (step the setpoint back up) but only ONCE per event — the collapsed
 * reading persists through the 64-sample MA and the inner loop's paced
 * backoff for ~1 s after the cause is removed, and re-acting on that
 * stale tail would ratchet the setpoint up several bogus steps. 1200 ms
 * ≈ 3 backoff intervals + filter delay. */
#define MPPT_SP_COLLAPSE_BLANK_MS 1200UL

/* HOLD duration before a periodic re-probe (longer than the legacy
 * MPPT_HOLD_TIME_MS): under input-vreg an irradiance change mostly
 * changes the CURRENT drawn at the held voltage, which the inner loop
 * absorbs with no MPPT involvement — only slow Vmpp drift (temperature)
 * and panel swaps need re-tracking. Each re-probe costs ~15 s at
 * slightly suboptimal points and typically one bounded knee-test dip,
 * so don't pay that every 30 s. Fast irradiance RISES are handled out
 * of band: a collapse while parked (knee rose above the held band)
 * exits HOLD immediately — see the HOLD collapse escape in mppt.c. */
#define MPPT_SP_HOLD_TIME_MS      60000UL

/* ── Legacy PWM-perturbing inc-conductance (CHARGER_INPUT_VREG=0) ──────── */

#define MPPT_MAX_STEP_SIZE        8       /* initial PWM step per perturbation                    */
#define MPPT_MIN_STEP_SIZE        1       /* smallest step (convergence threshold)                */
#define MPPT_CONVERGE_REVERSALS   6       /* reversals at min step → declare converged            */

/* Perturb/observe pacing. The panel ADCs (V, I) are 64-sample moving
 * averages at the 10 ms ADC tick → ~640 ms window, ~320 ms group delay.
 * If MPPT perturbs every 50 ms state-machine tick (as it did originally),
 * it steps PWM ~6 times before the filter reflects even the first step —
 * dV/dI are pure transient noise, and tracking parks at a garbage point
 * (observed: mppt_limit collapsed to 47 mA against a ~126 mA MPP panel).
 * Hold each perturbation at least one filter-settle window before the
 * next observation. 400 ms > 320 ms group delay, with margin. */
#define MPPT_STEP_INTERVAL_MS     400UL

/* Max time in TRACKING before forced exit to HOLD. Must allow enough
 * PACED steps to converge: descend MAX_STEP_SIZE→MIN (3 halvings) plus
 * MPPT_CONVERGE_REVERSALS reversals ≈ 10-15 steps × MPPT_STEP_INTERVAL_MS.
 * Was 300 ms (fine when perturbing every tick, far too short once paced). */
#define MPPT_RUNTIME_MS           6000UL  /* ~15 paced steps                                     */
#define MPPT_HOLD_TIME_MS         30000UL /* wait time in HOLD before re-entering TRACKING       */

/* Charger settle window: after the charger activates from CHG_INACTIVE,
 * MPPT is blocked from entering TRACKING for this long.
 *
 * Was 1000 ms: the old rationale was to let CC (CC_PWM_STEP=1) walk PWM
 * down gradually from PWM_MIN_DUTY (399, off) before MPPT took over,
 * since starting MPPT from "off" could slam the buck high-duty before a
 * current reading returned. That rationale is now obsolete:
 * activate_charger_region() PRE-POSITIONS pwm to a conducting value
 * (V_bat + CHG_ACTIVATION_HEADROOM_MV), so the buck is already in the
 * responsive range on tick 1 — MPPT does not have to wait for CC to
 * descend.
 *
 * Lowered to 250 ms because on a soft (PV) source the long wait was
 * fatal: CC chasing allowed_chg walks the panel past its MPP knee in
 * ~2 down-steps (2 × CC_DOWNSTEP_INTERVAL_MS = 600 ms) and collapses
 * V_panel before MPPT's settle window ever expires, so MPPT never
 * engaged. 250 ms (5 ticks) lets MPPT grab PWM control BEFORE CC's
 * first rate-limited down-step (at 300 ms) drags the panel down.
 * HAS_SUN_CLEAR_COUNT covers any residual sag during this window. */
#define CHARGER_MPPT_SETTLE_MS    250UL

/* Stiff-source escape: while in TRACKING at MAX step, count how many
 * consecutive ticks have walked PWM in the same direction with no
 * reversal. If this exceeds the limit, the source is stiff (dV ≈ dI ≈
 * 0 ⇒ direction stuck at last_direction) and the inc-conductance loop
 * will march monotonically into a high-duty zone. Force exit to HOLD
 * with PWM parked at the entry value. 8 ticks × MAX_STEP = 64 PWM
 * counts walked — well past any real panel's MPP search range. */
#define MPPT_STUCK_TICK_LIMIT     8

/* Panel-limited detection:
 * If I_charge < allowed_chg - this margin, and has_sun,
 * the panel can't deliver what the budget allows → MPPT needed. */
#define PANEL_LIMITED_MARGIN_MA   100

/* MPPT limit cold-boot default. mppt_limit_ma constrains allowed_chg
 * (see power_budget.c).
 *
 * With CHARGER_INPUT_VREG=1 the perturb/observe MPPT region is disabled,
 * so this value is never updated by a TRACKING session — it is simply the
 * static panel-side ceiling on allowed_chg. Set it to the buck hardware
 * limit (BUCK_MAX_CURRENT_MA) so the *battery* intake limits in
 * power_budget (precharge 200 mA, CV taper 200 mA, CC-zone 2 A) are what
 * actually bound the current. The input-voltage loop in cc_regulate keeps
 * the real drawn current at the panel's MPP, well under this ceiling on a
 * soft source; on a stiff source the ceiling (via the battery limit) is
 * what the current clamp enforces. */
#define MPPT_LIMIT_DEFAULT_MA     BUCK_MAX_CURRENT_MA

/* =========================================================================
 * 7. FAULT THRESHOLDS
 * =========================================================================
 */
#define FAULT_OVERCURRENT_CHG_MA  2200    /* charge current fault (above BUCK_MAX + margin)       */
#define FAULT_OVERCURRENT_DSG_MA  5000    /* discharge current fault                              */
#define FAULT_SYSTEM_CURRENT_MA   6000    /* total system current fault                           */
#define FAULT_USB_OVERVOLT_MV     6000    /* USB output over-voltage                              */
#define FAULT_RECOVER_WAIT_MS     10000UL /* minimum time before attempting fault recovery         */

/* =========================================================================
 * 8. TEMPERATURE LIMITS
 * =========================================================================
 *
 * LiFePO4 charge: 0°C to 45°C
 * LiFePO4 discharge: -20°C to 60°C
 * Board: up to 60°C
 *
 * Temperatures are in °C as returned by get_temperature().
 */
#define BAT_TEMP_MAX_CHARGE_C     45      /* stop charging above this                             */
#define BAT_TEMP_MIN_CHARGE_C     0       /* stop charging below this                             */
#define BAT_TEMP_MAX_DISCHARGE_C  60      /* fault: stop discharge above this                     */
#define BOARD_TEMP_MAX_C          60      /* fault: board overtemp                                */

/* Charge-window recovery hysteresis, asymmetric on purpose.
 *
 * COLD keeps the original 10 degC: a cell warming up from below 0 degC is
 * being warmed by ambient, so a wide band costs nothing and the charger's own
 * heat genuinely helps.
 *
 * HOT is 5 degC. The old symmetric 10 degC meant one 46 degC sample locked
 * charging out until the pack fell to 35 degC — in a closed enclosure on a
 * sunny day that is the rest of the afternoon. Bench capture
 * serial_20260910_155320.log shows exactly this: TEMP_CHARGE_BLOCK latched at
 * t=2658 s and never cleared for the remaining 11 minutes of the session.
 * 5 degC is still far wider than the sensor noise floor (the reading is a
 * 64-sample average, see get_temperature) and wide enough that the charger's
 * own dissipation cannot re-trip it immediately. */
#define TEMP_HYSTERESIS_C         10      /* COLD-side recovery band (0 degC -> resume at 10)     */
#define TEMP_HYSTERESIS_HOT_C     5       /* HOT-side recovery band  (45 degC -> resume at 40)    */

/* ── Thermistor front-end (R51/R52 + R55, schematic sheet "OUTPUTS_CH") ──
 *
 * BOTH NTC dividers are biased from the 2.5 V_VREF rail, NOT from 3V3/VDD:
 *
 *     2.5V_VREF ──[ 10 K ]──┬── TEMPn (ADC) ──[200R]── TP_T5n
 *                           │
 *                         [ NTC 10K ]
 *                           │
 *                          GND
 *
 * get_temperature() used to pass the *measured VDD* (~3300 mV, via get_vdd())
 * as the divider supply, which made every reading 10-15 degC too hot — the
 * error that made the 60 degC limits look wrong. Reconstructed from the bench
 * logs: a reported 37 degC was really 23 degC, a reported 70 degC really
 * 58 degC, a reported 34 degC on the pack really 22 degC.
 *
 * Because the bias rail IS the ADC reference rail, the conversion is
 * ratiometric — Rt = R_SERIES * code / (FULL_SCALE - code) — so any real VREF
 * error cancels out instead of being injected as a temperature offset. That
 * cancellation is the whole reason the divider was designed off VREF, and
 * feeding it VDD threw it away. */
#define THERMISTOR_BIAS_MV        2500    /* R51/R52 top-of-divider rail = 2.5V_VREF              */
#define THERMISTOR_SERIES_OHM     10000UL /* R51 / R52, 10K 0603                                  */

/* Plausibility band on the raw averaged ADC count, used to catch a dead
 * sensor before it is converted. Outside this band the divider is not looking
 * at a thermistor at all: a SHORTED NTC pulls the node to 0 (which the table
 * clamp would silently report as the hot end of the table -> instant, and
 * permanent, OVERTEMP), and an OPEN NTC pulls it to the rail (reported as the
 * cold end -> permanent TEMP_CHARGE_BLOCK). Both failures used to be
 * indistinguishable from a real reading.
 *
 * The band is deliberately generous: at -40 degC the NCP18X sits near 3900
 * counts and at +125 degC near 195, so [64, 4032] rejects only hard faults. */
#define THERMISTOR_ADC_MIN        64      /* below this the NTC is shorted / node at GND          */
#define THERMISTOR_ADC_MAX        4032    /* above this the NTC is open / node at the bias rail   */
#define TEMP_INVALID_C            INT16_MAX /* sentinel returned by get_temperature() on failure  */

/* Consecutive invalid samples before the sensor is declared failed. At the
 * 50 ms pipeline tick this is 3 s — long enough to ride out a harvest that
 * lands mid-conversion, short enough to react well inside any thermal event. */
#define TEMP_SENSOR_FAIL_TICKS    60

/* ── Thermal foldback (thermal.c) ──
 *
 * The hard OVERTEMP fault is a cliff: it cuts the lamps, USB and charging
 * outright and needs a 10 degC recovery before anything comes back. Foldback
 * is the graceful stage underneath it — trade brightness for temperature and
 * hold the board just below the cliff instead of falling off it.
 *
 * Sized from the bench data (serial_20260910_155320.log, corrected for the
 * bias-rail error above): a sustained 2.7 A load walks the board from 33 degC
 * to ~58 degC over ~30 min and then sits there. It never equilibrates lower,
 * so at full brightness this load lives permanently within 2 degC of the
 * 60 degC fault. Starting foldback at 55 degC engages before that and holds.
 *
 * Pacing: the board's thermal time constant is minutes, so the loop only has
 * to be faster than the plant, not fast. 5 %/4 s covers the full range in
 * ~60 s. Same principle as the MPPT dwell constants — do not outrun the
 * measurement (the reading is a 64-sample / ~640 ms average). */
#define THERMAL_FOLDBACK_START_C     55   /* start derating lamp current at/above this            */
#define THERMAL_FOLDBACK_RESUME_C    50   /* recover derate below this (5 degC deadband)          */
#define THERMAL_FOLDBACK_STEP_PCT    5    /* derate change per step                               */
#define THERMAL_FOLDBACK_INTERVAL_MS 4000UL /* minimum time between derate steps                  */
#define THERMAL_FOLDBACK_MIN_PCT     25   /* never derate below this (usable light, not darkness) */

/* =========================================================================
 * 9. SYSTEM TIMING
 * =========================================================================
 *
 * SysTick fires at 1 ms (1 kHz).
 * Main loop checks intervals against time_now() (milliseconds).
 */
#define TICK_ADC_MS               10      /* ADC raw sampling rate                                */
#define TICK_BUTTON_MS            20      /* button debounce/polling rate                         */
#define TICK_MAIN_MS              50      /* state machine + regulation tick                      */
/* ── UART telemetry rate ──
 *
 * ctx->log_mode selects the rate; LOG_MODE_DEFAULT is what ctx_init() seeds
 * at boot. The field is live-writable from a CCS breakpoint (Expressions ->
 * ctx.log_mode) so a bench session can go quiet or go fast without a reflash.
 *
 *   LOG_MODE_OFF  (0) — UART silent: no telemetry lines AND no state-transition
 *                       lines. The boot banner and the HardFault dump are
 *                       unconditional and still print. This is the only mode
 *                       that takes the blocking TX out of the super-loop
 *                       entirely (see the cost note below).
 *   LOG_MODE_1HZ  (1) — one line per second. The long-session / overnight
 *                       setting: ~30 KB per hour of capture.
 *   LOG_MODE_FAST (2) — five lines per second. Resolves events shorter than a
 *                       second (a ~2 s CHG: CC -> OFF teardown dwell, a single
 *                       PANEL_VREG_INTERVAL_MS step) that 1 Hz aliases away.
 *
 * Cost: a line is ~300 chars and printToUART() blocks, so at 115200 8N1 each
 * line parks the super-loop ~27 ms — ~14 % of wall time at FAST, ~3 % at 1HZ.
 * That delays (never bunches) the 50 ms pipeline tick and the fast guards at
 * the top of the loop; worst-case guard latency is the same ~27 ms in every
 * non-OFF mode, it just occurs more often. Do not add a faster mode at this
 * baud: below ~150 ms the TX stops fitting between lines and the loop lives
 * inside printToUART(). That needs UART1.targetBaudRate raised in SPC_20.syscfg
 * (460800 -> ~7 ms/line) and the terminal changed to match.
 */
#define LOG_MODE_OFF              0
#define LOG_MODE_1HZ              1
#define LOG_MODE_FAST             2
#define LOG_MODE_DEFAULT          LOG_MODE_FAST
#define TICK_LOG_1HZ_MS           1000    /* LOG_MODE_1HZ  interval               */
#define TICK_LOG_FAST_MS          200     /* LOG_MODE_FAST interval (5 lines/s)   */

#define VBATM_REFRESH_MS          1000    /* re-pulse VBATM_EN so a hot-plugged cell shows up     */

/* Inactivity window before arming deep sleep. Applies to EM_IDLE (nothing
 * to do) and EM_SAFE_MODE (loads shed, waiting on battery recovery — the
 * state where MCU draw matters most, since it is bleeding a depleted cell).
 * Sleep entry is additionally gated on: no latched fault (fault recovery
 * needs pipeline ticks) and, in IDLE, all lamps off (never turn off a
 * light the user is using to save power). */
#define IDLE_SLEEP_TIMEOUT_MS     120000UL /* 2 min in IDLE/SAFE with no activity → enter sleep   */

/* ── Deep sleep (STANDBY0) wake scheduling ──
 *
 * In sleep the MCU sits in STANDBY0 (LFCLK only — SysTick, ADC, PWM timers
 * and UART are all clock-gated; GPIO levels and all peripheral registers
 * are retained). Wake sources, both armed only for the duration of sleep:
 *
 *   1. Button edges (PB6/PB7, GROUP1 NVIC) — immediate full wake.
 *   2. The ADC_LOW_POWER timer (TIMG8, one of the two STANDBY-capable
 *      timers, clocked LFCLK/8/256 = 16 Hz) — periodic wake-check: the
 *      firmware runs for ~SLEEP_CHECK_SETTLE_MS, refreshes the ADC, and
 *      inspects raw V_panel / I_dsg / V_bat to decide "full wake" vs
 *      "back to STANDBY".
 *
 * The interval trades wake latency for average power: the check costs
 * ~60 ms awake, so at 10 s the MCU duty cycle is ~0.6 %. Loads plugged
 * into the USB rail are POWERED immediately regardless (the boost + load
 * switches stay on through sleep — the AP2151s cannot restart into a
 * plugged load, see the 2026-07 idle-gating revert); the interval only
 * bounds how long until the firmware *notices* and leaves IDLE. */
#define SLEEP_WAKE_INTERVAL_MS    10000UL

/* Tick rate of the wake timer. MUST match the ADC_LOW_POWER instance in
 * SPC_20.syscfg: LFCLK 32768 Hz / divider 8 / prescale 256 = 16 Hz
 * (62.5 ms per count; 16-bit counter → max interval ~68 min). The
 * generated 1 s LOAD_VALUE of 15 in ti_msp_dl_config.h confirms it. */
#define SLEEP_TIMER_TICK_HZ       16U

/* Awake window per periodic wake-check before sampling the sensors. Must
 * cover the sense-rail settle (µs — same enable edge refresh_vbatm_sense
 * relies on) plus at least one full ADC harvest: SysTick resumes on wake
 * and re-kicks conversions every TICK_ADC_MS, and a sequence takes ~a few
 * ms, so 60 ms guarantees several fresh Adc*Result sets even if the
 * conversion that was frozen mid-flight at STANDBY entry glitches. */
#define SLEEP_CHECK_SETTLE_MS     60UL

/* Wake-timer LOAD value for one SLEEP_WAKE_INTERVAL_MS period (N+1 counts
 * per period, hence the −1 — matches the generated 1 s LOAD_VALUE of 15). */
#define SLEEP_TIMER_LOAD_COUNTS \
    ((uint32_t)(SLEEP_WAKE_INTERVAL_MS * SLEEP_TIMER_TICK_HZ / 1000UL) - 1UL)

/* =========================================================================
 * 10. BATTERY CAPACITY (for coulomb counter / SOC)
 * =========================================================================
 */
#define BAT_NOMINAL_CAPACITY_MAH  20000
#define BAT_CHARGE_EFFICIENCY_PCT 94

/* =========================================================================
 * 11. USER INTERFACE — LED BAR DISPLAY
 * =========================================================================
 *
 * Two 5-segment bar graphs on the front panel (multiplexed in SPCBoardAPI.c):
 *   - LED_BAR_1 → battery state-of-charge fuel gauge
 *   - LED_BAR_2 → solar panel output power
 * Each bar fills 0..5 segments by the thresholds below; a bar stays dark when
 * its source is absent (no cell / no usable sun). Policy lives in main.c
 * (ui_display_update / led_boot_animation).
 */

/* "Battery present" floor (mV). A disconnected cell floats near 0 mV; below
 * this the battery bar stays dark instead of showing a level. Mirrors the
 * BAT_UNDERVOLT 500 mV sense guard. */
#define UI_BAT_PRESENT_MV         500

/* Battery fuel-gauge segment thresholds (mV, LiFePO4 discharge curve). V_bat
 * at or above level N lights N segments; below SEG1 → 0 segments (present but
 * empty). Bench-derived working band 3.0–3.65 V. */
#define UI_BAT_SEG1_MV            3020
#define UI_BAT_SEG2_MV            3120
#define UI_BAT_SEG3_MV            3240
#define UI_BAT_SEG4_MV            3360
#define UI_BAT_SEG5_MV            3480

/* Panel-power segment thresholds (mW). panel_power at or above level N lights
 * N segments. Display-only — tune to the deployed panel's wattage. The bar
 * stays dark (no usable sun) when has_sun is clear, independent of these. */
#define UI_PANEL_SEG1_MW          200
#define UI_PANEL_SEG2_MW          750
#define UI_PANEL_SEG3_MW          1500
#define UI_PANEL_SEG4_MW          3000
#define UI_PANEL_SEG5_MW          5000

/* Power-on sweep: light one more segment on each bar every this many ms until
 * all five are on (~0.6 s end to end). Cosmetic "firmware alive" cue. */
#define UI_BOOT_ANIM_STEP_MS      120UL

#endif /* HW_CONFIG_H */
