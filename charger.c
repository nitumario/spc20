/*
 * charger.c — Charge Controller (Pipeline Step 7)
 *
 * Sign convention: lower pwm = higher duty = more current.
 *   pwm -= step → more current;  pwm += step → less current.
 * Units: mV, mA, ms.
 */

#include "charger.h"
#include "fault_mgr.h"
#include "SPCBoardAPI.h"

/* Faults that block charging. FAULT_OVERCURRENT_CHG excluded: it's a
 * non-latching per-tick chopper in fault_mgr, never appears in fault.code. */
#define CHG_FAULT_BLOCK_MASK   \
    (FAULT_OVERTEMP            \
   | FAULT_BAT_OVERVOLT        \
   | FAULT_PRECHARGE_TIMEOUT   \
   | FAULT_TEMP_CHARGE_BLOCK)

/* =========================================================================
 * PWM helpers
 * ========================================================================= */

static inline uint16_t pwm_clamp(int32_t p)
{
    if (p < PWM_MAX_DUTY) return PWM_MAX_DUTY;   /* highest V_buck */
    if (p > PWM_MIN_DUTY) return PWM_MIN_DUTY;   /* off (V_buck < V_bat) */
    return (uint16_t)p;
}

static inline void pwm_step(system_ctx_t *ctx, int32_t delta)
{
    ctx->pwm = pwm_clamp((int32_t)ctx->pwm + delta);
}

/* =========================================================================
 * STATE ENTRY ACTIONS
 * ========================================================================= */

static void enter_buck_settle(system_ctx_t *ctx)
{
    ctx->charger.state = CHG_BUCK_SETTLE;
    ctx->charger.active_start_ms = time_now();

    /* MOSFET open during settle; pre-seed PWM to V_bat-matched duty. */
    disable_charge_switch();
    ctx->pwm = pwm_for_charging_voltage(ctx->meas.bat_voltage);

    ctx->charger.bat_full_timing    = false;
    ctx->charger.bat_full_signaled  = false;
    ctx->charger.fast_chop_consec   = 0;
}

static void enter_precharge(system_ctx_t *ctx)
{
    ctx->charger.state = CHG_PRECHARGE;
    ctx->charger.precharge_start_ms = time_now();

    /* Taper timing is only meaningful in CV. Reset defensively. */
    ctx->charger.bat_full_timing    = false;
    ctx->charger.bat_full_signaled  = false;
}

static void enter_cc(system_ctx_t *ctx)
{
    ctx->charger.state = CHG_CC;
    ctx->charger.precharge_start_ms = 0;

    ctx->charger.bat_full_timing    = false;
    ctx->charger.bat_full_signaled  = false;
}

static void enter_cv(system_ctx_t *ctx)
{
    ctx->charger.state = CHG_CV;

    /* Start taper watch fresh on CV entry. */
    ctx->charger.bat_full_timing    = false;
    ctx->charger.bat_full_signaled  = false;
}

/* =========================================================================
 * SHARED REGULATION HELPERS
 * ========================================================================= */

/* Back off if V_panel < PANEL_SAFETY_MV. Runs before MPPT check. */
static bool panel_safety_backoff(system_ctx_t *ctx)
{
    if (ctx->meas.panel_voltage < PANEL_SAFETY_MV) {
        pwm_step(ctx, PANEL_BACKOFF_STEP);  /* reduce current aggressively */
        return true;
    }
    return false;
}

/* Charger yields PWM while MPPT is actively perturbing. */
static inline bool mppt_owns_pwm(const system_ctx_t *ctx)
{
    return (ctx->mppt.state == MPPT_TRACKING);
}

/*
 * Bang-bang regulator on chg_current toward allowed_chg.
 * Down-steps rate-limited to CC_DOWNSTEP_INTERVAL_MS: the 64-sample
 * moving average has ~320 ms lag, so stepping every 50 ms overshoots
 * the target before feedback arrives. Up-steps are not throttled.
 */
static void cc_regulate(system_ctx_t *ctx)
{
    /* Chopper holds MOSFET open; chg_current=0 is not an under-current signal. */
    if (ctx->charger.chopper_active) return;

    int32_t target = (int32_t)ctx->allowed_chg;
    int32_t actual = (int32_t)ctx->meas.chg_current;
    int32_t err    = actual - target;

    if (err < -CC_DEADBAND_MA) {
        uint32_t now = time_now();
        if ((now - ctx->charger.cc_last_downstep_ms) >= CC_DOWNSTEP_INTERVAL_MS) {
            pwm_step(ctx, -CC_PWM_STEP);
            ctx->charger.cc_last_downstep_ms = now;
        }
    } else if (err > CC_DEADBAND_MA) {
        pwm_step(ctx, +CC_PWM_STEP);
    }
}

/* Regulate V_bat around BAT_CV_VOLTAGE_MV ± CV_DEADBAND_MV. */
static void cv_regulate(system_ctx_t *ctx)
{
    uint16_t v_bat = ctx->meas.bat_voltage;

    if (v_bat < BAT_CV_VOLTAGE_MV) {
        pwm_step(ctx, -CV_PWM_STEP);
    } else if (v_bat > (BAT_CV_VOLTAGE_MV + CV_DEADBAND_MV)) {
        pwm_step(ctx, +CV_PWM_STEP);
    }
}

/* Set bat_full when I < BAT_CV_TAPER_MA continuously for BAT_FULL_HOLD_MS.
 * Timer resets if current rises back above taper. */
static void cv_taper_track(system_ctx_t *ctx)
{
    charger_ctx_t *c = &ctx->charger;
    uint32_t now = time_now();

    bool under_taper = (ctx->meas.chg_current < (int16_t)BAT_CV_TAPER_MA);

    if (under_taper) {
        if (!c->bat_full_timing) {
            c->bat_full_timing    = true;
            c->bat_full_timer_start = now;
        } else if (!c->bat_full_signaled &&
                   (now - c->bat_full_timer_start) >= BAT_FULL_HOLD_MS) {
            c->bat_full_signaled = true;
            ctx->bat_full        = true;  /* energy_mode reads this flag */
        }
    } else {
        c->bat_full_timing   = false;
        c->bat_full_signaled = false;
    }
}

/*
 * Stiff-source safety guard: when CC_FAST_CHOP_CONSEC consecutive raw
 * samples exceed FAULT_OVERCURRENT_CHG_MA, jump PWM up by CC_FAST_BACKOFF_STEP
 * in one tick. The 64-sample avg (640 ms window) is too slow to prevent
 * the fault latch; this reacts before the filtered value trips fault_mgr.
 */
static bool fast_backoff_if_overcurrent(system_ctx_t *ctx)
{
    /* Ignore raw samples briefly after MOSFET close: inrush peaks exceed
     * FAULT_OVERCURRENT_CHG_MA for 1-2 samples even at V_buck = V_bat. */
    if ((time_now() - ctx->charger.mosfet_close_ms) < CC_INRUSH_IGNORE_MS) {
        ctx->charger.fast_chop_consec = 0;
        return false;
    }

    int16_t i_instant = get_charge_current_instant();
    if (i_instant <= (int16_t)FAULT_OVERCURRENT_CHG_MA) {
        ctx->charger.fast_chop_consec = 0;
        return false;
    }

    if (++ctx->charger.fast_chop_consec < CC_FAST_CHOP_CONSEC) {
        return false;
    }

    /* Sustained overcurrent — back off hard. MOSFET stays closed, state stays CC. */
    pwm_step(ctx, +CC_FAST_BACKOFF_STEP);
    ctx->charger.fast_chop_consec = 0;
    return true;
}

/* =========================================================================
 * PER-STATE TICK LOGIC
 * ========================================================================= */

/* Hold MOSFET open for BUCK_SETTLE_MS, then close and enter PRECHARGE or CC. */
static void tick_buck_settle(system_ctx_t *ctx)
{
    if ((time_now() - ctx->charger.active_start_ms) < BUCK_SETTLE_MS) {
        return;
    }

    /* Settle window elapsed — connect the battery. */
    enable_charge_switch();
    ctx->charger.mosfet_close_ms = time_now();

    /* Arm CC down-step throttle so regulation doesn't fire before current sense settles. */
    ctx->charger.cc_last_downstep_ms = time_now();

    if (ctx->meas.bat_voltage < BAT_PRECHARGE_MV) {
        enter_precharge(ctx);
    } else {
        enter_cc(ctx);
    }
    /* No regulator step this tick — wait for ADC to ingest at least one
     * post-close sample before acting on chg_current. */
}

static void tick_precharge(system_ctx_t *ctx)
{
    charger_ctx_t *c = &ctx->charger;

    if (fast_backoff_if_overcurrent(ctx)) return;

    /* P1: V_bat ≥ 3 V → CC */
    if (ctx->meas.bat_voltage >= BAT_PRECHARGE_MV) {
        enter_cc(ctx);
        /* Fall through to CC regulation this tick. */
    }
    /* P2: precharge timeout → fault */
    else if ((time_now() - c->precharge_start_ms) >= BAT_PRECHARGE_TIMEOUT_MS) {
        fault_raise(ctx, FAULT_PRECHARGE_TIMEOUT);
        ctx->pwm = PWM_MIN_DUTY;
        return;
    }

    /* Panel safety + MPPT gate, then regulate. */
    if (panel_safety_backoff(ctx)) return;
    if (mppt_owns_pwm(ctx))        return;

    cc_regulate(ctx);
}

static void tick_cc(system_ctx_t *ctx)
{
    /* Fast-backoff guard must run before any PWM regulation. */
    if (fast_backoff_if_overcurrent(ctx)) return;

    /* P1: V_bat ≥ 3.65 V → CV. */
    if (ctx->meas.bat_voltage >= BAT_CV_VOLTAGE_MV) {
        enter_cv(ctx);
        /* Fall through to CV regulation on the same tick. */
        if (panel_safety_backoff(ctx)) return;
        if (mppt_owns_pwm(ctx)) {
            cv_taper_track(ctx);   /* transition guard still runs */
            return;
        }
        cv_regulate(ctx);
        cv_taper_track(ctx);
        return;
    }

    if (panel_safety_backoff(ctx)) return;
    if (mppt_owns_pwm(ctx))        return;

    cc_regulate(ctx);
}

static void tick_cv(system_ctx_t *ctx)
{
    /* Taper timer runs regardless of who owns PWM. */
    if (panel_safety_backoff(ctx)) {
        cv_taper_track(ctx);
        return;
    }

    if (mppt_owns_pwm(ctx)) {
        cv_taper_track(ctx);
        return;
    }

    cv_regulate(ctx);
    cv_taper_track(ctx);
}

/* =========================================================================
 * PUBLIC: pipeline step 7 entry
 * ========================================================================= */
void charger_update(system_ctx_t *ctx)
{
    charger_ctx_t *c = &ctx->charger;

    /* ── Gate: is the region eligible to run at all? ── */
    bool em_charging = (ctx->energy_mode == EM_CHARGE_ONLY) ||
                       (ctx->energy_mode == EM_CHARGE_AND_LOAD);
    bool fault_block = (ctx->fault.code & CHG_FAULT_BLOCK_MASK) != 0;

    if (!em_charging || fault_block) {
        /* Fault may be newly latched this tick; park PWM and mark INACTIVE.
         * GPIO switches are owned by fault_mgr/energy_mode. */
        if (c->state != CHG_INACTIVE) {
            c->state = CHG_INACTIVE;
            c->precharge_start_ms = 0;
            c->bat_full_timing    = false;
            c->bat_full_signaled  = false;
            c->fast_chop_consec   = 0;
            ctx->pwm = PWM_MIN_DUTY;
        }
        return;
    }

    /* ── Self-activation from INACTIVE ── */
    if (c->state == CHG_INACTIVE) {
        enter_buck_settle(ctx);
    }

    /* ── Per-state tick ── */
    switch (c->state) {
        case CHG_BUCK_SETTLE: tick_buck_settle(ctx); break;
        case CHG_PRECHARGE:   tick_precharge(ctx);   break;
        case CHG_CC:          tick_cc(ctx);          break;
        case CHG_CV:          tick_cv(ctx);          break;

        /* unreachable — handled by gate above */
        case CHG_INACTIVE:
        default:
            break;
    }
}
