/*
 * mppt.c — Incremental Conductance MPPT (Pipeline Step 6)
 *
 * At MPP: X = dI·V + I·dV = 0.
 *   X > 0 (dV > 0): left of MPP → raise V → pwm += step.
 *   X < 0 (dV > 0): right of MPP → lower V → pwm -= step.
 * Division-free (avoids Cortex-M0+ 32-bit divider).
 *
 * Adaptive step: halve on each reversal down to MPPT_MIN_STEP_SIZE.
 * Converged: step==1 and reversals >= MPPT_CONVERGE_REVERSALS → HOLD.
 * HOLD parks at max_power_pwm; mppt_limit_ma = P_max / V_bat.
 */

#include "mppt.h"
#include "SPCBoardAPI.h"

/* PWM_MAX_DUTY = smallest value = highest V_buck. Legal range: [MAX, MIN]. */
static inline uint16_t pwm_clamp(int32_t p)
{
    if (p < PWM_MAX_DUTY) return PWM_MAX_DUTY;
    if (p > PWM_MIN_DUTY) return PWM_MIN_DUTY;
    return (uint16_t)p;
}

/* =========================================================================
 * STATE ENTRY ACTIONS
 * ========================================================================= */

/* Initialize TRACKING state; apply first perturbation so next tick has valid dV/dI. */
static void enter_tracking(system_ctx_t *ctx)
{
    mppt_ctx_t *m = &ctx->mppt;

    m->state          = MPPT_TRACKING;
    m->v_prev         = (int32_t)ctx->meas.panel_voltage;
    m->i_prev         = (int32_t)ctx->meas.panel_current;
    m->step_size      = MPPT_MAX_STEP_SIZE;
    m->reversals      = 0;
    m->stuck_ticks    = 0;
    m->max_power      = 0;
    m->max_power_pwm  = ctx->pwm;
    m->last_direction = -1;  /* first perturbation is pwm -= step (V falls) */
    m->tracking_start_ms = time_now();

    /* Force first perturbation so the next tick has a valid dV/dI. */
    ctx->pwm = pwm_clamp((int32_t)ctx->pwm - (int32_t)m->step_size);
}

/* Park at max_power_pwm and publish mppt_limit_ma = P_max / V_bat. */
static void enter_hold(system_ctx_t *ctx)
{
    mppt_ctx_t *m = &ctx->mppt;

    m->state = MPPT_HOLD;
    m->hold_start_ms = time_now();

    /* max_power_pwm is seeded with entry pwm, so it's a safe fallback
     * when nothing was learned (stiff source: dV=dI=0, max_power never updates). */
    ctx->pwm = m->max_power_pwm;

    /* Derive I_bus max from P_max / V_bat. Guard against tiny V_bat. */
    uint16_t v_bat = ctx->meas.bat_voltage;
    if (v_bat < 1000) v_bat = 1000;  /* guard against missing battery */

    int32_t i_bus_ma = (m->max_power * 1000) / (int32_t)v_bat;
    if (i_bus_ma < 0) i_bus_ma = 0;
    if (i_bus_ma > BUCK_MAX_CURRENT_MA) i_bus_ma = BUCK_MAX_CURRENT_MA;

    m->mppt_limit_ma = (uint16_t)i_bus_ma;
}

/* Drop panel constraint; CC/CV resumes PWM control. */
static void enter_disabled(system_ctx_t *ctx)
{
    ctx->mppt.state = MPPT_DISABLED;
    ctx->mppt.mppt_limit_ma = BUCK_MAX_CURRENT_MA;
    ctx->mppt.step_size = MPPT_MAX_STEP_SIZE;
    ctx->mppt.reversals = 0;
    ctx->mppt.stuck_ticks = 0;
    ctx->mppt.last_direction = 0;
}

/* =========================================================================
 * TRACKING STEP — one incremental-conductance iteration.
 * ========================================================================= */
static void tracking_step(system_ctx_t *ctx)
{
    mppt_ctx_t *m = &ctx->mppt;

    int32_t v_now = (int32_t)ctx->meas.panel_voltage;
    int32_t i_now = (int32_t)ctx->meas.panel_current;
    int32_t p_now = (int32_t)ctx->meas.panel_power;  /* mW, already computed */

    /* ── Track best operating point this session ── */
    if (p_now > m->max_power) {
        m->max_power     = p_now;
        m->max_power_pwm = ctx->pwm;
    }

    /* ── Compute incremental conductance decision ── */
    int32_t dV = v_now - m->v_prev;
    int32_t dI = i_now - m->i_prev;

    int8_t direction;
    if (dV == 0) {
        /* V stuck — let dI alone pick direction. */
        if      (dI > 0) direction = +1;   /* I rising at stuck V → want more V */
        else if (dI < 0) direction = -1;   /* I falling at stuck V → want less V */
        else             direction = m->last_direction;  /* no info, keep moving */
    } else {
        /* X = dI·V + I·dV; sign(dP/dV) = sign(X) when dV > 0, else -sign(X). */
        int32_t X = dI * v_now + i_now * dV;
        if (dV < 0) X = -X;

        if      (X > 0) direction = +1;    /* dP/dV > 0 → left of MPP → raise V */
        else if (X < 0) direction = -1;    /* dP/dV < 0 → right of MPP → lower V */
        else            direction = m->last_direction;  /* exactly at MPP */
    }

    /* ── Adaptive step: halve on direction reversal ── */
    bool reversed = (direction != 0 &&
                     m->last_direction != 0 &&
                     direction != m->last_direction);
    if (reversed) {
        if (m->reversals < 255) m->reversals++;
        if (m->step_size > MPPT_MIN_STEP_SIZE) {
            m->step_size = (uint8_t)(m->step_size >> 1);  /* halve */
            if (m->step_size < MPPT_MIN_STEP_SIZE) m->step_size = MPPT_MIN_STEP_SIZE;
        }
    }

    /* ── Stiff-source detector ──
     * Stiff PSU has dV≈dI≈0, so no reversals occur and direction stays
     * pinned — the loop marches into the danger zone. Bail to HOLD first. */
    if (m->step_size >= MPPT_MAX_STEP_SIZE && !reversed) {
        if (m->stuck_ticks < 255) m->stuck_ticks++;
    } else {
        m->stuck_ticks = 0;
    }

    /* +1 → raise V_panel (pwm += step);  -1 → lower V_panel (pwm -= step) */
    if (direction != 0) {
        int32_t next = (int32_t)ctx->pwm + direction * (int32_t)m->step_size;
        ctx->pwm = pwm_clamp(next);
    }

    /* ── Snapshot for next tick ── */
    m->v_prev = v_now;
    m->i_prev = i_now;
    if (direction != 0) m->last_direction = direction;
}

/* =========================================================================
 * TRANSITION EVALUATION
 * ========================================================================= */

static bool tracking_converged(const mppt_ctx_t *m)
{
    return (m->step_size <= MPPT_MIN_STEP_SIZE) &&
           (m->reversals >= MPPT_CONVERGE_REVERSALS);
}

static bool tracking_timed_out(const mppt_ctx_t *m)
{
    return (time_now() - m->tracking_start_ms) >= MPPT_RUNTIME_MS;
}

static bool hold_expired(const mppt_ctx_t *m)
{
    return (time_now() - m->hold_start_ms) >= MPPT_HOLD_TIME_MS;
}

/* =========================================================================
 * PUBLIC: step 6 entry point
 * ========================================================================= */
void mppt_update(system_ctx_t *ctx)
{
    mppt_ctx_t *m = &ctx->mppt;
    const bool has_sun       = ctx->flag_has_sun.value;
    const bool panel_limited = ctx->panel_limited;
    const bool charging      = (ctx->charger.state != CHG_INACTIVE) ||
                               (ctx->energy_mode == EM_CHARGE_ONLY) ||
                               (ctx->energy_mode == EM_CHARGE_AND_LOAD);

    /* No charger → MPPT has no role. */
    if (!charging) {
        if (m->state != MPPT_DISABLED) enter_disabled(ctx);
        return;
    }

    switch (m->state) {

    case MPPT_DISABLED:
        /* P1: panel_limited + has_sun + settled + current_below_gate + chopper_quiet → TRACKING.
         *
         * settled: avoids MPPT stealing PWM on the very first tick when
         *   chg_current=0 trivially triggers panel_limited.
         * current_below_gate: panel is only the bottleneck when current
         *   is below capacity; above it CC owns the loop.
         * chopper_quiet: MPPT_CHOPPER_LOCKOUT_MS after last chop — the
         *   64-sample avg decays toward 0 during chopping, briefly faking
         *   a below-gate reading even while overcurrent-saturated. */
        {
            uint32_t since_active = time_now() - ctx->charger.active_start_ms;
            bool settled = (ctx->charger.state != CHG_INACTIVE) &&
                           (since_active >= CHARGER_MPPT_SETTLE_MS);
            bool current_below_gate =
                (ctx->meas.chg_current < (int16_t)MPPT_ENTRY_CURRENT_GUARD_MA);
            bool chopper_quiet =
                (ctx->charger.chopper_last_active_ms == 0) ||
                ((time_now() - ctx->charger.chopper_last_active_ms)
                    >= MPPT_CHOPPER_LOCKOUT_MS);
            if (panel_limited && has_sun && settled &&
                current_below_gate && chopper_quiet) {
                enter_tracking(ctx);
            }
        }
        /* P2: stay DISABLED */
        break;

    case MPPT_TRACKING:
        /* P1: !has_sun → DISABLED (safety first) */
        if (!has_sun) {
            enter_disabled(ctx);
            break;
        }
        /* Convergence/timeout evaluated after the step. */
        tracking_step(ctx);

        /* P2: converged → HOLD */
        if (tracking_converged(m)) {
            enter_hold(ctx);
            break;
        }
        /* P3: runtime timeout → HOLD */
        if (tracking_timed_out(m)) {
            enter_hold(ctx);
            break;
        }
        /* P4: stiff source → HOLD */
        if (m->stuck_ticks >= MPPT_STUCK_TICK_LIMIT) {
            enter_hold(ctx);
            break;
        }
        /* P5: continue tracking (already stepped above) */
        break;

    case MPPT_HOLD:
        /* P3 (checked first for safety): !has_sun → DISABLED */
        if (!has_sun) {
            enter_disabled(ctx);
            break;
        }
        /* P1: !panel_limited → DISABLED, but skip if mppt_limit itself
         *     is causing panel_limited=false — releasing it would trigger
         *     panel_limited again, bouncing immediately back to TRACKING. */
        if (!panel_limited && m->mppt_limit_ma > PANEL_LIMITED_MARGIN_MA) {
            enter_disabled(ctx);
            break;
        }
        /* P2: hold expired + same gates as DISABLED→TRACKING → re-enter TRACKING.
         *     Gate failure defers hold timer by MPPT_HOLD_TIME_MS. */
        if (hold_expired(m)) {
            bool current_below_gate =
                (ctx->meas.chg_current < (int16_t)MPPT_ENTRY_CURRENT_GUARD_MA);
            bool chopper_quiet =
                (ctx->charger.chopper_last_active_ms == 0) ||
                ((time_now() - ctx->charger.chopper_last_active_ms)
                    >= MPPT_CHOPPER_LOCKOUT_MS);
            if (current_below_gate && chopper_quiet) {
                enter_tracking(ctx);
            } else {
                m->hold_start_ms = time_now();
            }
            break;
        }
        /* P4: stay in HOLD */
        break;

    default:
        /* unknown state → DISABLED */
        enter_disabled(ctx);
        break;
    }
}
