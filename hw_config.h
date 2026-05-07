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

/* Constant-current zone.
 *
 * Target sits well below BUCK_MAX_CURRENT_MA (2000) and the chopper
 * threshold FAULT_OVERCURRENT_CHG_MA (2500) so the cc_regulate ±25 mA
 * deadband, 64-sample-average ripple, and stiff-PSU step response have
 * room to oscillate without nicking the chopper line. Mirrors V2.5.5's
 * validated bench envelope (chg_ctx.charge_current = 1000 in utils.c).
 * The hardware ceiling stays at BUCK_MAX_CURRENT_MA; this is the
 * regulator setpoint, not a hardware limit. */
#define BAT_CC_MAX_MA             1000    /* max charge current in CC phase                      */

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

/* =========================================================================
 * 2. SOLAR PANEL / BUCK INPUT LIMITS
 * =========================================================================
 *
 */
// #define PANEL_OVP_CLAMP_MV        15000   /* approximate OVP clamp voltage (from zener + MOSFET)  */

#define PANEL_MIN_MV              9000    /* has_sun flag sets above this                         */
#define PANEL_MIN_CLEAR_MV        8000    /* has_sun clears below this (1 V hysteresis)           */
#define HAS_SUN_DEBOUNCE_COUNT    3       /* consecutive readings before flag sets (×50 ms)       */
#define PANEL_SAFETY_MV           10000   /* CC backs off below this to avoid collapsing panel    */

/* =========================================================================
 * 3. BUCK CONVERTER (TPS56347)
 * =========================================================================
 *
 * The buck is controlled by injecting a PWM signal into its FB pin through
 * an RC filter. Higher average PWM voltage at FB → buck regulator commands
 * lower V_out. The application-level pwm count maps to the timer compare
 * register as `CC = PWM_PERIOD - pwm`, so the convention from the
 * application's point of view is:
 *
 *   lower pwm value → higher V_buck → more charge current
 *   higher pwm value → lower V_buck → less charge current
 *
 * The voltage→pwm LUT in SPCBoardAPI.c (`output_voltages_buck_mV[]`)
 * covers pwm 1-344:
 *
 *   pwm = 1   → V_buck ≈ 3772 mV  (maximum output, maximum charge current)
 *   pwm = 344 → V_buck ≈ 2786 mV  (lowest LUT entry — already below any
 *                                  healthy V_bat in principle)
 *
 * Empirically on a stiff input source (bench PSU), pwm = 344 is NOT
 * actually low enough to push V_buck below V_bat — the buck still
 * delivers several watts into the cell at this duty. Matching the
 * reference firmware V2.5.5, the legal range extends to pwm = 399; at
 * that duty the buck is genuinely off, the body diode of the charge
 * MOSFET is reverse-biased, and no current flows. Pwm 345-399 is past
 * the end of the voltage LUT, but the LUT is only consulted by
 * `set_charging_voltage()` / `pwm_for_charging_voltage()` for
 * voltage-target lookups; CC and the off-state path command pwm
 * directly via `set_buck_pwm()`, which writes the timer with no LUT
 * involvement. The 345-399 range is therefore the deliberate "off"
 * zone — beyond LUT calibration, but still controlled by the same
 * monotonic FB-injection circuit.
 */
#define PWM_PERIOD                400
#define PWM_MAX_DUTY              1       /* lowest pwm value → highest V_buck (max charge)        */
#define PWM_MIN_DUTY              399     /* highest pwm value → buck genuinely off (V_buck < V_bat)*/
#define BUCK_MAX_CURRENT_MA       2000    /* hardware current limit of the inductor/FET           */

/* =========================================================================
 * 4. LOAD DETECTION
 * =========================================================================
 *
 * "has_load" is based on measured discharge current.
 * The discharge current sensor has some offset/noise,
 * so we need a threshold above zero.
 */
#define LOAD_DETECT_MA            50      /* I_load above this → has_load = true                  */
#define LOAD_DETECT_CLEAR_MA      30      /* I_load below this → has_load = false (hysteresis)    */

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

/* fast_chop_if_overcurrent reads the unfiltered ADC sample to detect a
 * sustained over-current that the 64-sample average would only see hundreds
 * of ms later. With PWM pre-seeded near V_bat the *only* remaining instant
 * spikes are cap-rebalance / inductor inrush at MOSFET close — short and
 * self-correcting. Ignore the chopper for this long after MOSFET close so
 * those transients can not re-trigger it. Real sustained over-current is
 * still caught by fault_mgr's averaged FAULT_OVERCURRENT_CHG_MA path. */
#define CC_INRUSH_IGNORE_MS       150UL

/* Single ADC samples can transiently exceed FAULT_OVERCURRENT_CHG_MA from
 * switching ripple, especially with a stiff input source where V_buck >>
 * V_bat at the LUT-seeded duty. Require this many CONSECUTIVE instant
 * samples above threshold before reacting. At a 50 ms tick this is
 * N × 50 ms of sustained over-current — long enough to dismiss ripple
 * peaks, short enough to react well before the 640 ms moving-average
 * fault path latches. */
#define CC_FAST_CHOP_CONSEC       3

/* On sustained over-current, increase PWM (decrease duty) by this many
 * counts in one tick, while keeping the charge MOSFET CLOSED and staying
 * in CC. cc_regulate's per-tick +CC_PWM_STEP=1 cannot keep up with a
 * stiff source. Stepping up by several counts at once moves the
 * operating point clear of FAULT_OVERCURRENT_CHG_MA before the
 * regulator's slow descent walks back into it. There is no point
 * re-entering CHG_BUCK_SETTLE on every over-current event — the buck IC
 * is already running steadily, only its commanded duty needs to change. */
#define CC_FAST_BACKOFF_STEP      10


#define CV_DEADBAND_MV            5       /* regulate between CV_VOLTAGE and CV_VOLTAGE + this    */
#define CV_PWM_STEP               1

/* Panel safety: if V_panel drops below PANEL_SAFETY_MV during charging,
 * increase PWM (decrease duty) by this many steps to back off quickly. */
#define PANEL_BACKOFF_STEP        5

/* =========================================================================
 * 6. MPPT TUNING (Incremental Conductance)
 * =========================================================================
 *
 * MPPT perturbs the PWM to find the panel's maximum power point.
 * It uses adaptive step size: starts large, halves on direction reversals.
 *
 * Convergence = step_size has reached 1 AND we've seen enough reversals.
 * At that point, we're oscillating around MPP with minimum perturbation.
 */
#define MPPT_MAX_STEP_SIZE        8       /* initial PWM step per perturbation                    */
#define MPPT_MIN_STEP_SIZE        1       /* smallest step (convergence threshold)                */
#define MPPT_CONVERGE_REVERSALS   6       /* reversals at min step → declare converged            */
#define MPPT_RUNTIME_MS           300     /* max time in TRACKING before forced exit to HOLD      */
#define MPPT_HOLD_TIME_MS         30000UL /* wait time in HOLD before re-entering TRACKING        */

/* Charger settle window: after the charger activates from CHG_INACTIVE,
 * MPPT is blocked from entering TRACKING for this long. Lets the slow
 * CC regulator (CC_PWM_STEP=1) walk PWM down gradually from PWM_MIN_DUTY
 * before MPPT takes over with MAX_STEP_SIZE=8 — otherwise the first
 * MPPT step from "off" can slam the buck into a high-duty zone before
 * any current measurement comes back, especially with a stiff source. */
#define CHARGER_MPPT_SETTLE_MS    1000UL

/* Buck soft-start settle: when the buck IC is brought out of disable
 * (BUCK_DIS deasserted), its internal soft-start ramps V_out from 0 up
 * to the FB-injection setpoint over a few ms. During the ramp the
 * regulator can briefly overshoot the setpoint before the loop settles,
 * and any moment where V_out > V_bat dumps inrush into the battery via
 * the charge MOSFET body diode and cap-charging path.
 *
 * To avoid that, the charger keeps the charge MOSFET OPEN for this
 * long after enabling the buck, so the buck output cap settles to its
 * off-zone level (PWM_MIN_DUTY commands V_buck < V_bat) before the
 * battery is connected. 250 ms is a comfortable margin over a typical
 * TPS56347 soft-start (~2-5 ms) plus FB-filter RC settle on a stiff
 * input source. */
#define BUCK_SETTLE_MS            250UL

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

/* MPPT chopper lockout window.
 *
 * After any tick on which the soft-chopper (fault_mgr) opened the
 * charge MOSFET, MPPT_DISABLED → TRACKING entry is blocked for this
 * long. Without it, the chopper-induced dips in the 64-sample
 * chg_current moving average defeat MPPT_ENTRY_CURRENT_GUARD_MA
 * during the brief windows when the MOSFET is open and the average
 * is still decaying — letting MPPT slip in and walk PWM down while
 * CC is fighting overcurrent.
 *
 * Mirrors the structural exclusion V2.5.5 gets for free from its
 * 10-second RECOVER_WAIT_TIME on cond_overcur_chg: while the fault is
 * latched, charging state ≠ MPPT, so MPPT cannot run. We don't latch
 * a fault (the chopper is non-latching by design), but we get the
 * same effect by timing out MPPT entry after each chop event.
 *
 * 5 s is comfortable — well past the 640 ms moving-average window so
 * the gate sees the post-event steady-state value, and short enough
 * that real solar conditions (where the chopper should never fire)
 * are unaffected. */
#define MPPT_CHOPPER_LOCKOUT_MS   5000UL

/* MPPT entry current gate.
 *
 * Block MPPT_DISABLED → TRACKING (and the periodic HOLD → TRACKING
 * re-test) whenever chg_current is already at or above this threshold.
 *
 * Rationale: MPPT only makes sense when the panel is the bottleneck.
 * If the buck is already pushing this much current into the cell, the
 * panel clearly has plenty of headroom and CC is the right loop to be
 * running. On a stiff source (bench PSU, or a panel well above its
 * knee) the inc-conductance loop has no real signal — dV ≈ dI ≈ 0 —
 * and walks PWM monotonically into the danger zone before the
 * MPPT_STUCK_TICK_LIMIT escape kicks in.
 *
 * Mirrors V2.5.5 utils.c:257 (`if (m1.chg_current < 1800)` in
 * handle_cc — only switches to MPPT when CC clearly cannot reach
 * target). With BAT_CC_MAX_MA=1000 and BUCK_MAX_CURRENT_MA=2000, a
 * threshold near 1500 sits comfortably above the CC target deadband
 * yet far below the chopper line (2500), so normal regulation ripple
 * cannot kick MPPT in or out. */
#define MPPT_ENTRY_CURRENT_GUARD_MA  1500

/* =========================================================================
 * 7. FAULT THRESHOLDS
 * =========================================================================
 */
/* Charge current SOFT-CHOPPER threshold (NON-LATCHING).
 *
 * Mirrors V2.5.5: when m->chg_current crosses this line, fault_mgr opens
 * the charge MOSFET; when it drops back below, the MOSFET re-closes on
 * the very next tick. The buck is never killed, no fault bit is latched,
 * no recovery wait. The chopper acts as a hard ceiling on average
 * battery current — by construction the moving-average chg_current
 * cannot sustainedly exceed this value.
 *
 * Set well above BAT_CC_MAX_MA (1000) and BUCK_MAX_CURRENT_MA (2000)
 * so the cc_regulate ±25 mA deadband, 64-sample-average ripple, and
 * stiff-PSU step response do not trigger continuous chops during
 * normal operation. Empirically the bench PSU operating envelope sits
 * at 2070-2260 mA on V2.5.5 with target=1000, so 2500 lands just above
 * the natural ceiling — chopper engages on real overcurrent, not on
 * regulation ripple.
 *
 * Same macro is reused by fast_backoff_if_overcurrent for its instant-
 * sample over-current trip. Both react to the same threshold; the
 * chopper acts on the averaged reading (slower, more stable), the
 * fast-backoff acts on raw 10 ms samples (faster, ripple-tolerant via
 * CC_FAST_CHOP_CONSEC). They are layered defenses, not redundant. */
#define FAULT_OVERCURRENT_CHG_MA  2500
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
#define TEMP_HYSTERESIS_C         10      /* resume after temp drops by this much                 */

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
#define TICK_LOG_MS               1000    /* UART logging interval                                */

#define IDLE_SLEEP_TIMEOUT_MS     120000UL /* 2 min in IDLE with no activity → enter sleep        */

/* =========================================================================
 * 10. BATTERY CAPACITY (for coulomb counter / SOC)
 * =========================================================================
 */
#define BAT_NOMINAL_CAPACITY_MAH  20000
#define BAT_CHARGE_EFFICIENCY_PCT 94

#endif /* HW_CONFIG_H */