/*
 * system_types.h — Type Definitions for SPC_20
 * ==============================================
 *
 * This file defines the entire system's data model.
 * No logic, no functions.
 *
 * The central idea:
 *   There is ONE struct (system_ctx_t) that holds the complete system state.
 *   Every function in the system takes a pointer to it.
 *   If you can print this struct, you can see everything the firmware knows.
 *
 * System hierarchy:
 *
 *   SYS_INIT  →  hardware setup, then transition to SYS_RUN
 *
 *   SYS_RUN   →  three child HSMs execute every tick IN PARALLEL:
 *
 *     ┌─ ENERGY_MGMT ──────────────────────────────────────────┐
 *     │  Makes all power decisions.                             │
 *     │  Contains 3 orthogonal regions that run together:       │
 *     │                                                         │
 *     │  [region 1: ENERGY MODE]                                │
 *     │    EM_IDLE → EM_CHARGE_ONLY → EM_CHARGE_AND_LOAD       │
 *     │    EM_DISCHARGE_ONLY → EM_SAFE_MODE                    │
 *     │    Controls: CHARGER_EN, BATTERY_EN, OUTPUT_EN, USB_EN  │
 *     │    Activates/deactivates the charger region             │
 *     │    Sleep is entered from EM_IDLE after timeout          │
 *     │                                                         │
 *     │  [region 2: CHARGER]                                    │
 *     │    CHG_INACTIVE → CHG_PRECHARGE → CHG_CC → CHG_CV      │
 *     │    Controls: buck PWM (when MPPT is not tracking)       │
 *     │    Reads: allowed_chg from power budget                 │
 *     │                                                         │
 *     │  [region 3: MPPT]                                       │
 *     │    MPPT_DISABLED → MPPT_TRACKING → MPPT_HOLD           │
 *     │    Controls: buck PWM (during TRACKING only)            │
 *     │    Writes: mppt_limit back to power budget              │
 *     └────────────────────────────────────────────────────────-┘
 *
 *     ┌─ FAULT_MGR ────────────────────────────────────────────┐
 *     │  Monitors limits (voltage, current, temperature).       │
 *     │  Runs in parallel with ENERGY_MGMT every tick.          │
 *     │  Does NOT pull the system out of RUN.                   │
 *     │  Instead: latches fault bits, takes protective actions  │
 *     │  (disable switches), and ENERGY_MGMT sees the fault    │
 *     │  flags and reacts accordingly.                          │
 *     │  Periodically attempts recovery for latched faults.     │
 *     └────────────────────────────────────────────────────────┘
 *
 *     ┌─ UI_MGR (Placeholder) ─────────────────────────────────┐
 *     │  Handles buttons + LED bar battery level display.      │
 *     │  Runs in parallel. Does not affect power decisions.    │
 *     └────────────────────────────────────────────────────────┘
 *
 * Reading guide:
 *   1. State enums       — what states each machine can be in
 *   2. Measurements      — what the ADC tells us
 *   3. Flag debounce     — how we avoid reacting to noise
 *   4. Charger context   — internal state of the charge controller
 *   5. MPPT context      — internal state of the tracker
 *   6. Fault context     — which faults are active
 *   7. system_ctx_t      — the one struct that holds everything
 */

#ifndef SYSTEM_TYPES_H
#define SYSTEM_TYPES_H

#include <stdint.h>
#include <stdbool.h>
#include "hw_config.h"

/* =========================================================================
 * 1. STATE ENUMERATIONS
 * =========================================================================
 *
 * Each state machine gets its own enum.
 * Prefixed to avoid collisions (EM_ for energy mode, CHG_ for charger, etc.)
 */

/*
 * Top-level system state.  Only two states.
 *
 *   SYS_INIT  → hardware is initializing, all outputs disabled,
 *               transitions to SYS_RUN when setup is complete.
 *
 *   SYS_RUN   → normal operation. The three child HSMs
 *               (ENERGY_MGMT, FAULT_MGR, UI_MGR) all execute
 *               every tick inside this state.
 *
 * There is no SYS_FAULT state. Faults are handled by FAULT_MGR
 * which is a parallel region INSIDE RUN. It sets flags and takes
 * protective actions; ENERGY_MGMT sees those flags and reacts.
 *
 * There is no SYS_SLEEP state. Sleep is an action taken from
 * EM_IDLE (inside ENERGY_MGMT) after a timeout. The MCU enters
 * low-power mode and wakes back into SYS_RUN via interrupt.
 */
typedef enum {
    SYS_INIT,
    SYS_RUN
} system_state_t;

/*
 * Energy mode (orthogonal region 1 inside ENERGY_MGMT)
 * Decides WHAT the system does.
 *
 * This is the "traffic controller." It looks at flags (has_sun, has_load,
 * bat_low, bat_full) and decides which hardware paths are enabled.
 * It activates/deactivates the charger region.
 *
 *   EM_IDLE             — no sun, no load. Everything off. After timeout → sleep.
 *   EM_CHARGE_ONLY      — sun present, no load. Buck on, charge battery.
 *   EM_CHARGE_AND_LOAD  — sun + load. Buck on, feeds both. Includes the case
 *                         where allowed_chg == 0 (buck feeds loads only,
 *                         battery floats or discharges as buffer).
 *   EM_DISCHARGE_ONLY   — no sun, load present. Battery feeds load alone.
 *   EM_SAFE_MODE        — battery low. Loads shed to protect battery.
 *                         Recovery requires V_bat > BAT_SAFE_RECOVER_MV.
 */
typedef enum {
    EM_IDLE,
    EM_CHARGE_ONLY,
    EM_CHARGE_AND_LOAD,
    EM_DISCHARGE_ONLY,
    EM_SAFE_MODE
} energy_mode_state_t;

/*
 * Charger state (orthogonal region 2 inside ENERGY_MGMT)
 * Decides HOW to charge.
 *
 * Activated/deactivated by energy mode. When inactive, the charger
 * does nothing and the buck is disabled.
 *
 *   CHG_INACTIVE   — charger region not running (deactivated by energy mode)
 *   CHG_PRECHARGE  — V_bat < 3.0V, trickle charge at ≤200 mA
 *   CHG_CC         — constant current, target = allowed_chg from power budget
 *   CHG_CV         — constant voltage at 3.65V, current tapers until bat_full
 */
typedef enum {
    CHG_INACTIVE,
    CHG_PRECHARGE,
    CHG_CC,
    CHG_CV
} charger_state_t;

/*
 * MPPT state (orthogonal region 3 inside ENERGY_MGMT)
 * Optimises WHERE on the panel's I-V curve we operate.
 *
 *   MPPT_DISABLED  — panel is not the bottleneck, mppt_limit = BUCK_MAX
 *   MPPT_TRACKING  — actively perturbing PWM, owns PWM control.
 *                    CC skips regulation but still runs safety checks.
 *   MPPT_HOLD      — converged or timed out. CC owns PWM again.
 *                    mppt_limit computed from best power found.
 *                    Waits MPPT_HOLD_TIME before re-entering TRACKING.
 */
typedef enum {
    MPPT_DISABLED,
    MPPT_TRACKING,
    MPPT_HOLD
} mppt_state_t;

/* =========================================================================
 * 2. MEASUREMENTS
 * =========================================================================
 *
 * Readings from ADC, converted to millivolts and milliamps.
 * Updated by measurements_update() every TICK_MAIN_MS.
 *
 * All values are positive-as-measured except:
 *   - chg_current: can read negative if current flows backward
 *     (shouldn't happen in normal operation, but the sensor can see it)
 *   - i_bat_net: signed. positive = battery charging, negative = discharging.
 *     This is derived: chg_current − dsg_current.
 */
typedef struct {
    /* Voltages (mV) */
    uint16_t bat_voltage;       /* battery terminal voltage                    */
    uint16_t chg_voltage;       /* buck output voltage (charger side of MOSFET)*/
    uint16_t out_voltage;       /* output rail voltage (after output switch)   */
    uint16_t panel_voltage;     /* buck input voltage (after OVP clamp)        */
    uint16_t usb1_voltage;      /* USB port 1 output voltage                  */
    uint16_t usb2_voltage;      /* USB port 2 output voltage                  */

    /* Currents (mA) */
    uint16_t panel_current;     /* current from panel into buck                */
    int16_t  chg_current;       /* current through charge path (into battery)  */
    uint16_t dsg_current;       /* current through discharge path (to loads)   */

    /* Derived */
    int32_t  panel_power;       /* panel_voltage × panel_current / 1000  (mW) */
    int32_t  i_bat_net;         /* chg_current − dsg_current (signed, mA)     */

    /* Temperature (°C, from thermistor lookup) */
    int16_t  bat_temp;          /* battery temperature sensor                  */
    int16_t  board_temp;        /* board temperature sensor                    */

} measurements_t;

/* =========================================================================
 * 3. FLAG DEBOUNCE HELPER
 * =========================================================================
 *
 * Some flags need hysteresis so they don't flicker on noisy signals
 * or brief transients (e.g., a load step causing a momentary V_bat dip).
 *
 * How it works:
 *   - The flag has different set and clear thresholds (defined in hw_config.h)
 *   - count tracks consecutive readings past the set threshold
 *   - When count reaches count_threshold, the flag latches true
 *   - The flag only clears when the reading crosses the clear threshold
 *     (which is offset from the set threshold by hysteresis)
 *
 * Example for bat_low:
 *   set threshold:   2800 mV  (BAT_LOW_MV)
 *   clear threshold: 2900 mV  (BAT_LOW_CLEAR_MV)
 *   count_threshold: 3        (3 × 50ms tick = 150ms debounce)
 *
 *   V_bat drops to 2750 mV:
 *     tick 1: count=1, flag=false
 *     tick 2: count=2, flag=false
 *     tick 3: count=3, flag=TRUE  ← bat_low now set
 *
 *   V_bat rises to 2850 mV:
 *     flag stays TRUE (below clear threshold of 2900)
 *
 *   V_bat rises to 2950 mV:
 *     count=0, flag=FALSE         ← bat_low cleared
 */
typedef struct {
    bool     value;             /* current debounced flag state                 */
    uint8_t  count;             /* consecutive readings past set threshold      */
    uint8_t  count_threshold;   /* how many to trigger (configured at init)     */
} debounce_flag_t;

/* =========================================================================
 * 4. CHARGER CONTEXT
 * =========================================================================
 *
 * Internal state for the CHG_INACTIVE → PRECHARGE → CC → CV machine.
 * Only meaningful when state != CHG_INACTIVE.
 *
 * The charger does not know about loads. It receives allowed_chg
 * from the power budget as its CC target. The power budget already
 * subtracted load current — the charger just regulates to the number.
 */
typedef struct {
    charger_state_t state;

    /* Precharge timing — start timestamp for timeout detection */
    uint32_t precharge_start_ms;

    /* CV taper detection:
     * Battery is full when I_charge stays below BAT_CV_TAPER_MA
     * for BAT_FULL_HOLD_MS continuously.
     *
     * bat_full_timing:   true = timer is running (current is below taper)
     * bat_full_timer_start: timestamp when current first dropped below taper
     * bat_full_signaled: true = the 30s hold completed, battery is full
     *
     * If current rises above taper while timing, reset:
     *   bat_full_timing = false, timer resets on next drop.
     */
    uint32_t bat_full_timer_start;
    bool     bat_full_timing;
    bool     bat_full_signaled;

} charger_ctx_t;

/* =========================================================================
 * 5. MPPT CONTEXT
 * =========================================================================
 *
 * Internal state for MPPT_DISABLED → MPPT_TRACKING → MPPT_HOLD.
 *
 * The algorithm is incremental conductance with adaptive step size.
 * It needs to remember the previous panel voltage and current to
 * compute dV and dI between samples.
 *
 * INTEGER MATH (no float):
 *   The conductance comparison  dI/dV  vs  -I/V  is rewritten as:
 *     dI × V  vs  -I × dV      (cross-multiply to eliminate division)
 *   All values in mV and mA, so the products are mV×mA = µW scale.
 *   int32_t is sufficient: 15000 mV × 2000 mA = 30,000,000 — fits.
 */
typedef struct {
    mppt_state_t state;

    /* Previous sample for dV, dI computation */
    int32_t v_prev;             /* panel voltage last sample (mV)              */
    int32_t i_prev;             /* panel current last sample (mA)              */

    /* Adaptive step control */
    uint8_t step_size;          /* current PWM perturbation magnitude          */
    uint8_t reversals;          /* direction changes at current step_size      */
    int8_t  last_direction;     /* +1 or -1: which way we perturbed last       */

    /* Best operating point found during this TRACKING session */
    int32_t  max_power;         /* best panel power (mW) seen this session     */
    uint16_t max_power_pwm;     /* PWM value that produced max_power           */

    /* Timing */
    uint32_t tracking_start_ms; /* timestamp when TRACKING was entered         */
    uint32_t hold_start_ms;     /* timestamp when HOLD was entered             */

    /* Output to power budget:
     * The maximum current the panel can deliver through the buck
     * at the current operating point. Computed when entering HOLD
     * from the best power point found during TRACKING.
     *
     * When MPPT is DISABLED, this is set to BUCK_MAX_CURRENT_MA
     * (no panel constraint — the panel can deliver more than the buck max).
     */
    uint16_t mppt_limit_ma;

} mppt_ctx_t;

/* =========================================================================
 * 6. FAULT CONTEXT
 * =========================================================================
 *
 * FAULT_MGR is a parallel child HSM inside SYS_RUN.
 * It runs every tick alongside ENERGY_MGMT.
 *
 * It does NOT have its own state machine with named states.
 * Instead, it maintains a bitmask of active faults.
 * Each fault type has:
 *   - A detection condition (checked every tick)
 *   - An immediate protective action (disable a switch)
 *   - A recovery condition (checked periodically)
 *
 * Faults LATCH: once a bit is set, it stays until recovery clears it.
 * This prevents the scenario where a transient fault sets and clears
 * within one tick, and the system never notices.
 *
 * How ENERGY_MGMT sees faults:
 *   ENERGY_MGMT reads ctx->fault.code to check for specific faults,
 *   and ctx->fault.active to check "is anything faulted."
 *   FAULT_MGR may have already disabled a switch (e.g., battery switch
 *   on overvoltage), so ENERGY_MGMT must not re-enable it while the
 *   fault is active.
 */

/* Fault ID bits — each fault gets one bit in the bitmask */
#define FAULT_NONE              0
#define FAULT_OVERTEMP          (1U << 0)   /* battery or board over temperature  */
#define FAULT_BAT_OVERVOLT      (1U << 1)   /* V_bat > BAT_OVERVOLT_MV            */
#define FAULT_OVERCURRENT_CHG   (1U << 2)   /* I_charge > FAULT_OVERCURRENT_CHG_MA*/
#define FAULT_OVERCURRENT_DSG   (1U << 3)   /* I_discharge > FAULT_OVERCURRENT_DSG_MA */
#define FAULT_BAT_UNDERVOLT     (1U << 4)   /* V_bat < BAT_UNDERVOLT_MV           */
#define FAULT_USB_OVERVOLT      (1U << 5)   /* USB output over-voltage            */
#define FAULT_PRECHARGE_TIMEOUT (1U << 6)   /* precharge exceeded 15 min          */
#define FAULT_TEMP_CHARGE_BLOCK (1U << 7)   /* too cold or hot to charge          */

typedef struct {
    uint16_t code;              /* bitmask of active faults                    */
    bool     active;            /* shorthand: (code != FAULT_NONE)             */
    uint32_t last_recovery_ms;  /* timestamp of last recovery attempt          */
} fault_ctx_t;

/* =========================================================================
 * 7. THE SYSTEM CONTEXT — one struct to hold everything
 * =========================================================================
 *
 * This is the single source of truth for the entire system.
 *
 * Rules:
 *   - Every function takes  system_ctx_t *ctx  as its first argument.
 *   - No global variables. If a function needs data, it's in ctx.
 *   - The main loop pipeline determines who writes what and when.
 *
 * Pipeline order inside SYS_RUN (every TICK_MAIN_MS):
 *
 *   1. measurements_update(ctx)  → writes ctx->meas
 *   2. flags_update(ctx)         → writes ctx->flags (has_sun, bat_low, etc.)
 *   3. power_budget_update(ctx)  → writes ctx->allowed_chg, ctx->i_buck_max
 *   4. fault_check(ctx)          → writes ctx->fault, may disable switches
 *   5. energy_mode_update(ctx)   → writes ctx->energy_mode, enables/disables HW
 *   6. mppt_update(ctx)          → writes ctx->mppt, may write ctx->pwm
 *   7. charger_update(ctx)       → writes ctx->charger, may write ctx->pwm
 *   8. apply_pwm(ctx)            → reads ctx->pwm, writes to timer hardware
 *
 * This order is the contract. Each step reads fields written by
 * earlier steps and writes fields read by later steps.
 * Within one tick there are no races because execution is sequential.
 *
 * Note: UI_MGR (buttons + LED bar display) will be added later
 * as a parallel step that reads ctx for display but never writes
 * power-related fields.
 */
typedef struct {

    /* ── System level ── */
    system_state_t system_state;

    /* ── Measurements (written by measurements_update, step 1) ── */
    measurements_t meas;

    /* ── Flags (written by flags_update, step 2) ──
     *
     * These debounced booleans drive all state machine transitions.
     * State machines read these; they never read raw meas values
     * for transition decisions.
     */
    debounce_flag_t flag_bat_low;    /* V_bat < BAT_LOW_MV                  */
    bool has_sun;                    /* V_panel > PANEL_MIN_MV              */
    bool has_load;                   /* I_load > LOAD_DETECT_MA             */
    bool bat_full;                   /* charger signaled taper complete     */
    bool panel_limited;              /* I_chg < allowed_chg - margin AND sun*/
    bool temp_charge_ok;             /* battery temp within charge range    */

    /* ── Power budget (written by power_budget_update, step 3) ── */
    uint16_t i_buck_max;             /* MIN(BUCK_MAX, mppt_limit)           */
    uint16_t allowed_chg;            /* CC target after subtracting load    */

    /* ── PWM (written by charger or MPPT, read by apply_pwm) ──
     *
     * The ONLY variable that maps to the hardware timer.
     * Range: PWM_MAX_DUTY (1) to PWM_MIN_DUTY (399).
     * apply_pwm() is the only function that touches the timer register.
     */
    uint16_t pwm;

    /* ── ENERGY_MGMT child HSM ── */

    /* Region 1: energy mode */
    energy_mode_state_t energy_mode;
    uint32_t idle_start_ms;          /* when EM_IDLE was entered (sleep timer)  */
    bool     idle_sleep_pending;     /* true = idle timeout reached, enter sleep*/

    /* Region 2: charger */
    charger_ctx_t charger;

    /* Region 3: MPPT */
    mppt_ctx_t mppt;

    /* ── FAULT_MGR child HSM (parallel to ENERGY_MGMT) ── */
    fault_ctx_t fault;

    /* ── UI_MGR child HSM (parallel, added later) ──
     * Placeholder. Will contain:
     *   - button state (short press, long press)
     *   - LED bar display mode
     *   - battery percentage display state
     */

} system_ctx_t;

/* =========================================================================
 * 8. CONTEXT INITIALISER
 * =========================================================================
 *
 * Call once at startup to set safe defaults.
 * After this, the system is in SYS_INIT with all outputs off.
 * main() calls ctx_init(), runs hardware setup, then sets
 * ctx->system_state = SYS_RUN to start the pipeline.
 */
static inline void ctx_init(system_ctx_t *ctx)
{
    /* Zero everything — all bools false, all ints 0, all enums 0 */
    *ctx = (system_ctx_t){0};

    /* System starts in INIT */
    ctx->system_state = SYS_INIT;

    /* PWM starts at minimum duty (buck effectively off) */
    ctx->pwm = PWM_MIN_DUTY;

    /* Energy mode starts in IDLE */
    ctx->energy_mode = EM_IDLE;

    /* Charger starts inactive */
    ctx->charger.state = CHG_INACTIVE;

    /* MPPT starts disabled — no panel constraint assumed */
    ctx->mppt.state = MPPT_DISABLED;
    ctx->mppt.mppt_limit_ma = BUCK_MAX_CURRENT_MA;
    ctx->mppt.step_size = MPPT_MAX_STEP_SIZE;

    /* No faults */
    ctx->fault.code = FAULT_NONE;
    ctx->fault.active = false;

    /* bat_low debounce configuration */
    ctx->flag_bat_low.value = false;
    ctx->flag_bat_low.count = 0;
    ctx->flag_bat_low.count_threshold = BAT_LOW_DEBOUNCE_COUNT;

    /* Assume temperature OK until first measurement */
    ctx->temp_charge_ok = true;
}

/* =========================================================================
 * 9. DEBUG — state name strings for UART logging
 * =========================================================================
 *
 * Usage:
 *   printf("SYS:%s EM:%s CHG:%s MPPT:%s FLT:0x%04X\n",
 *          system_state_name(ctx->system_state),
 *          em_state_name(ctx->energy_mode),
 *          chg_state_name(ctx->charger.state),
 *          mppt_state_name(ctx->mppt.state),
 *          ctx->fault.code);
 *
 * String literals live in flash (const), cost zero RAM.
 */
static inline const char* system_state_name(system_state_t s)
{
    switch (s) {
        case SYS_INIT: return "INIT";
        case SYS_RUN:  return "RUN";
        default:       return "???";
    }
}

static inline const char* em_state_name(energy_mode_state_t s)
{
    switch (s) {
        case EM_IDLE:            return "IDLE";
        case EM_CHARGE_ONLY:     return "CHG_ONLY";
        case EM_CHARGE_AND_LOAD: return "CHG+LOAD";
        case EM_DISCHARGE_ONLY:  return "DSG_ONLY";
        case EM_SAFE_MODE:       return "SAFE";
        default:                 return "???";
    }
}

static inline const char* chg_state_name(charger_state_t s)
{
    switch (s) {
        case CHG_INACTIVE:  return "OFF";
        case CHG_PRECHARGE: return "PRE";
        case CHG_CC:        return "CC";
        case CHG_CV:        return "CV";
        default:            return "???";
    }
}

static inline const char* mppt_state_name(mppt_state_t s)
{
    switch (s) {
        case MPPT_DISABLED: return "OFF";
        case MPPT_TRACKING: return "TRK";
        case MPPT_HOLD:     return "HLD";
        default:            return "???";
    }
}

#endif /* SYSTEM_TYPES_H */