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

/* LED boost (TPS61088) rail target. Needs enough headroom for the LED string
 * Vf plus Vce_sat of the PNP current source; below ~10 V the per-channel CC
 * loop falls out of regulation and the lamps only weakly glow at <<150 mA.
 * Matches V2.5.5: set_led_voltage()'s linear formula saturates this request
 * to duty=1 (rail clamps to its top of ~11.3 V), which is the validated
 * operating point. */
#define LED_BOOST_TARGET_MV       11500

/* Drive current when a button-controlled lamp (LEDCTRL1/LEDCTRL2) is switched
 * ON. Lands on the 145/155 mA LUT bins (output_currents_led_mA), well under
 * the 350 mA per-channel hardware max. Switching a lamp OFF commands 0 mA,
 * which set_led_current() drives to compare 99 (~15 mA, the dimmest in-range
 * value on the inverted-polarity channel). compare==period is forbidden and
 * runs away, so "off" is a faint floor, not fully dark — see that fn. */
#define LAMP_ON_CURRENT_MA        150

/* Lamp press/hold brightness UI (lamp_buttons_update() in main.c). A short tap
 * toggles the lamp full-on / off; press-and-hold dims one level every
 * LAMP_DIM_STEP_MS until it reaches off. There are LAMP_DIM_LEVELS steps
 * between full and off, so a continuous hold from full goes dark in
 * LAMP_DIM_LEVELS * LAMP_DIM_STEP_MS (= 5 s). Per-level LED currents live in
 * lamp_level_ma[] in main.c (index LAMP_DIM_LEVELS = full = LAMP_ON_CURRENT_MA,
 * index 0 = off). The dim cadence is paced off the button's pressStartTime via
 * time_now() with a catch-up loop, so LAMP_DIM_STEP_MS need not be an exact
 * multiple of TICK_BUTTON_MS (steps just land on the nearest button poll).
 * Note: the LED current LUT (output_currents_led_mA) has only ~15 usable bins
 * between the off-floor and LAMP_ON_CURRENT_MA, so a few adjacent levels share
 * a current — the hold still dims smoothly over the full 5 s. */
#define LAMP_DIM_LEVELS           20
#define LAMP_DIM_STEP_MS          250

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

/* FOCV seed: Vmpp ≈ this % of Voc for crystalline silicon (textbook
 * fractional-open-circuit-voltage constant, k ≈ 0.71–0.82; the 13 V
 * bench panel measured ~0.82 at its knee). The seed only needs to be
 * in the basin — the hill-climb refines it. Voc is captured from the
 * unloaded panel reading on the activation tick (the panel idles
 * unloaded at pwm=399 in IDLE, so that reading IS open-circuit). */
#define MPPT_SP_FRACTION_PCT      76

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
 * Each bar fills 0..5 segments by the thresholds below; a bar flashes all
 * five segments when its source is absent (no cell / no usable sun). Policy
 * lives in main.c (ui_display_update / led_boot_animation).
 */

/* "Battery present" floor (mV). A disconnected cell floats near 0 mV; below
 * this the battery bar blinks instead of showing a level. Mirrors the
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
 * blinks (no usable sun) when has_sun is clear, independent of these. */
#define UI_PANEL_SEG1_MW          200
#define UI_PANEL_SEG2_MW          750
#define UI_PANEL_SEG3_MW          1500
#define UI_PANEL_SEG4_MW          3000
#define UI_PANEL_SEG5_MW          5000

/* Blink half-period (ms): a "no source" bar shows all 5 segments for this long,
 * then dark for this long (≈1.25 Hz flash). */
#define UI_BLINK_PERIOD_MS        400UL

/* Power-on sweep: light one more segment on each bar every this many ms until
 * all five are on (~0.6 s end to end). Cosmetic "firmware alive" cue. */
#define UI_BOOT_ANIM_STEP_MS      120UL

#endif /* HW_CONFIG_H */