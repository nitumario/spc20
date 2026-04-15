/*
 * mppt.c — Incremental Conductance MPPT (Pipeline Step 6)
 * =========================================================================
 *
 * Algorithm
 * ---------
 * Incremental conductance with adaptive step size, implemented in pure
 * integer math.
 *
 * At the maximum power point of a PV panel:
 *   dP/dV = 0   where P = V * I
 *   d(V*I)/dV = I + V·(dI/dV) = 0
 *   →  dI/dV = -I/V
 *
 * Left of MPP (V too low):   dI/dV > -I/V  →  dP/dV > 0  →  increase V
 * Right of MPP (V too high): dI/dV < -I/V  →  dP/dV < 0  →  decrease V
 *
 * To avoid division (and a 32-bit divider on a Cortex-M0+), multiply
 * both sides by V·dV. Since V_panel > 0 always, only sign(dV) matters:
 *
 *   dI/dV  vs  -I/V
 *   dI·V   vs  -I·dV
 *   X = dI·V + I·dV   vs   0
 *
 *   sign(dP/dV) = sign(X)           when dV > 0
 *   sign(dP/dV) = -sign(X)          when dV < 0
 *
 * Overflow: V ≤ 15000 mV, I ≤ 2000 mA → dI·V ≤ 3e7, fits int32_t.
 *
 * PWM ↔ panel voltage
 * -------------------
 * The buck draws current from the panel. Higher duty cycle draws more
 * current, pulling V_panel down along the I-V curve.
 *
 * In this firmware, lower pwm value = higher duty cycle:
 *   pwm -= step  → higher duty → more current drawn → V_panel FALLS
 *   pwm += step  → lower duty  → less current drawn → V_panel RISES
 *
 * So "direction = +1" (want V to rise) means pwm += step_size.
 *
 * Adaptive step
 * -------------
 * On every direction reversal, halve the step size (down to MPPT_MIN_STEP_SIZE).
 * Convergence: step_size == 1 AND reversals ≥ MPPT_CONVERGE_REVERSALS.
 * At that point we are oscillating around MPP with the smallest possible
 * perturbation — enter HOLD for MPPT_HOLD_TIME_MS.
 *
 * TRACKING is also force-exited after MPPT_RUNTIME_MS as a safety net
 * (e.g., rapidly-changing irradiance prevents convergence).
 *
 * HOLD behaviour
 * --------------
 * Entry sets ctx->pwm = max_power_pwm (the best point seen this session)
 * and computes mppt_limit_ma from the best power found:
 *
 *   I_bus_max (mA) = P_max (mW) * 1000 / V_bat (mV)
 *
 * Clamped to [0, BUCK_MAX_CURRENT_MA]. This tells power_budget to cap
 * i_buck_max at what the panel can actually sustain.
 */

#include "mppt.h"
#include "SPCBoardAPI.h"

/* ── PWM bound helpers ──
 *
 * PWM_MAX_DUTY (1) is the SMALLEST pwm value — it corresponds to the
 * HIGHEST duty cycle. PWM_MIN_DUTY (399) is the LARGEST pwm value. The
 * legal range is therefore [PWM_MAX_DUTY, PWM_MIN_DUTY]. */
static inline uint16_t pwm_clamp(int32_t p)
{
    if (p < PWM_MAX_DUTY) return PWM_MAX_DUTY;
    if (p > PWM_MIN_DUTY) return PWM_MIN_DUTY;
    return (uint16_t)p;
}

/* =========================================================================
 * STATE ENTRY ACTIONS
 * ========================================================================= */

/*
 * Enter TRACKING — per README "TRACKING Entry Actions":
 *   V_prev = V_panel, I_prev = I_panel, step_size = MAX, reversals = 0,
 *   max_power = 0, force pwm -= step_size, record tracking_start_ms.
 */
static void enter_tracking(system_ctx_t *ctx)
{
    mppt_ctx_t *m = &ctx->mppt;

    m->state          = MPPT_TRACKING;
    m->v_prev         = (int32_t)ctx->meas.panel_voltage;
    m->i_prev         = (int32_t)ctx->meas.panel_current;
    m->step_size      = MPPT_MAX_STEP_SIZE;
    m->reversals      = 0;
    m->max_power      = 0;
    m->max_power_pwm  = ctx->pwm;
    m->last_direction = -1;  /* first perturbation is pwm -= step (V falls) */
    m->tracking_start_ms = time_now();

    /* Force first perturbation so the next tick has a valid dV/dI. */
    ctx->pwm = pwm_clamp((int32_t)ctx->pwm - (int32_t)m->step_size);
}

/*
 * Enter HOLD — park PWM at the best point and publish mppt_limit_ma.
 *
 * mppt_limit_ma is the max current the buck can deliver to the bus
 * given the panel's MPP power. At buck output:
 *   I_bus ≈ P_panel / V_bus  (lossless approximation; V_bus ≈ V_bat)
 */
static void enter_hold(system_ctx_t *ctx)
{
    mppt_ctx_t *m = &ctx->mppt;

    m->state = MPPT_HOLD;
    m->hold_start_ms = time_now();

    /* Park at best operating point. If we never improved on the entry
     * point (max_power == 0), leave pwm where it is. */
    if (m->max_power > 0) {
        ctx->pwm = m->max_power_pwm;
    }

    /* Derive I_bus max from P_max / V_bat. Guard against tiny V_bat. */
    uint16_t v_bat = ctx->meas.bat_voltage;
    if (v_bat < 1000) v_bat = 1000;  /* avoid divide-by-tiny on missing battery */

    int32_t i_bus_ma = (m->max_power * 1000) / (int32_t)v_bat;
    if (i_bus_ma < 0) i_bus_ma = 0;
    if (i_bus_ma > BUCK_MAX_CURRENT_MA) i_bus_ma = BUCK_MAX_CURRENT_MA;

    m->mppt_limit_ma = (uint16_t)i_bus_ma;
}

/*
 * Enter DISABLED — drop panel constraint. CC/CV resumes PWM control.
 */
static void enter_disabled(system_ctx_t *ctx)
{
    ctx->mppt.state = MPPT_DISABLED;
    ctx->mppt.mppt_limit_ma = BUCK_MAX_CURRENT_MA;
    ctx->mppt.step_size = MPPT_MAX_STEP_SIZE;
    ctx->mppt.reversals = 0;
    ctx->mppt.last_direction = 0;
}

/* =========================================================================
 * TRACKING STEP — one iteration of incremental conductance.
 * =========================================================================
 *
 * Must be called each tick while in MPPT_TRACKING. Assumes ctx->pwm
 * already reflects the previous perturbation (set either by
 * enter_tracking or by the previous tick's tracking step).
 */
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
        /* X = dI*V + I*dV. Its sign equals sign(dP/dV) when dV > 0,
         * and is flipped when dV < 0. */
        int32_t X = dI * v_now + i_now * dV;
        if (dV < 0) X = -X;

        if      (X > 0) direction = +1;    /* dP/dV > 0 → left of MPP → raise V */
        else if (X < 0) direction = -1;    /* dP/dV < 0 → right of MPP → lower V */
        else            direction = m->last_direction;  /* exactly at MPP */
    }

    /* ── Adaptive step: halve on direction reversal ── */
    if (direction != 0 &&
        m->last_direction != 0 &&
        direction != m->last_direction) {

        if (m->reversals < 255) m->reversals++;
        if (m->step_size > MPPT_MIN_STEP_SIZE) {
            m->step_size = (uint8_t)(m->step_size >> 1);  /* halve */
            if (m->step_size < MPPT_MIN_STEP_SIZE) m->step_size = MPPT_MIN_STEP_SIZE;
        }
    }

    /* ── Apply perturbation to PWM ──
     * direction = +1  → raise V_panel → pwm += step
     * direction = -1  → lower V_panel → pwm -= step
     */
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
 * TRANSITION EVALUATION — per docs/MPPT_transition_table.csv
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
 * =========================================================================
 *
 * Runs after energy_mode. If the charger is inactive, MPPT stays
 * DISABLED (energy_mode already forces this on deactivation, but we
 * guard here too so re-activation is symmetric).
 */
void mppt_update(system_ctx_t *ctx)
{
    mppt_ctx_t *m = &ctx->mppt;
    const bool has_sun       = ctx->flag_has_sun.value;
    const bool panel_limited = ctx->panel_limited;
    const bool charging      = (ctx->charger.state != CHG_INACTIVE) ||
                               (ctx->energy_mode == EM_CHARGE_ONLY) ||
                               (ctx->energy_mode == EM_CHARGE_AND_LOAD);

    /* If the charger region is not active, MPPT has no role. Ensure
     * we're DISABLED and the panel constraint is released. */
    if (!charging) {
        if (m->state != MPPT_DISABLED) enter_disabled(ctx);
        return;
    }

    switch (m->state) {

    case MPPT_DISABLED:
        /* P1: panel_limited AND has_sun → TRACKING */
        if (panel_limited && has_sun) {
            enter_tracking(ctx);
        }
        /* P2: stay DISABLED */
        break;

    case MPPT_TRACKING:
        /* P1: !has_sun → DISABLED (safety first) */
        if (!has_sun) {
            enter_disabled(ctx);
            break;
        }
        /* Run one perturb/observe iteration. Convergence/timeout are
         * evaluated after the step so the latest reversals/time count. */
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
        /* P4: continue tracking (already stepped above) */
        break;

    case MPPT_HOLD:
        /* P1: !panel_limited → DISABLED */
        if (!panel_limited) {
            enter_disabled(ctx);
            break;
        }
        /* P3 (checked before P2 for safety): !has_sun → DISABLED */
        if (!has_sun) {
            enter_disabled(ctx);
            break;
        }
        /* P2: hold time expired AND panel_limited AND has_sun → TRACKING */
        if (hold_expired(m)) {
            enter_tracking(ctx);
            break;
        }
        /* P4: stay in HOLD (PWM parked, mppt_limit_ma already published) */
        break;

    default:
        /* Defensive: unknown state → drop to DISABLED. */
        enter_disabled(ctx);
        break;
    }
}
