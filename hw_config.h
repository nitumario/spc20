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

#define PANEL_MIN_MV              9000    /* has_sun flag sets above this                         */
#define PANEL_MIN_CLEAR_MV        8000    /* has_sun clears below this (1 V hysteresis)           */
#define PANEL_SAFETY_MV           10000   /* CC backs off below this to avoid collapsing panel    */

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

/* Panel-limited detection:
 * If I_charge < allowed_chg - this margin, and has_sun,
 * the panel can't deliver what the budget allows → MPPT needed. */
#define PANEL_LIMITED_MARGIN_MA   100

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

#define IDLE_SLEEP_TIMEOUT_MS     120000UL /* 2 min in IDLE with no activity → enter sleep        */

/* =========================================================================
 * 10. BATTERY CAPACITY (for coulomb counter / SOC)
 * =========================================================================
 */
#define BAT_NOMINAL_CAPACITY_MAH  20000
#define BAT_CHARGE_EFFICIENCY_PCT 94

#endif /* HW_CONFIG_H */