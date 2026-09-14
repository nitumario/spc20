/*
 * mppt.c — Maximum Power Point Tracker (Pipeline Step 6)
 * =========================================================================
 *
 * Two implementations, selected by CHARGER_INPUT_VREG (hw_config.h):
 *
 * ── CHARGER_INPUT_VREG=1 (current): SETPOINT-P&O ───────────────────────
 *
 * Perturb & observe on the INPUT-VREG SETPOINT, not on the PWM.
 *
 * The charger's inner voltage loop (charger.c cc_regulate) holds V_panel
 * at ctx->mppt.vreg_setpoint_mv and owns ctx->pwm exclusively — pacing,
 * panel-safety backoff, reverse-current escape and the allowed_chg clamp
 * all stay active at every instant. This region is the OUTER loop: it
 * moves the setpoint one MPPT_SP_STEP_MV at a time and measures whether
 * the system delivers more or less charge current there.
 *
 * Why this survives where PWM-perturbing P&O failed on this plant:
 *   - Actuation granularity: one PWM count is ~45 mA battery-side, ~20 %
 *     of a small panel's full MPP current — a "perturbation" that big is
 *     a sledgehammer. The setpoint is continuous in mV; the inner loop
 *     realises it with its own paced 1-count steps.
 *   - Observation: each probe dwells MPPT_SP_SETTLE_MS (inner loop walk
 *     + 640 ms ADC window) before averaging chg_current over
 *     MPPT_SP_MEASURE_MS. No decisions on transient/lagged readings —
 *     the failure mode that parked the legacy tracker at garbage points.
 *   - Fitness = averaged DELIVERED charge current (chg_current is the
 *     cleanest MA-filtered signal we have, and V_bat is constant over a
 *     3 s dwell, so max current ⇔ max delivered power — converter
 *     losses included, which is what we actually want to maximise).
 *   - Safety: a probe below the panel's knee collapses V_panel; the
 *     inner backoff rescues the rail (as bench-verified 2026-06-12) and
 *     the collapse branch here steps the setpoint back up — one bounded
 *     ~1 s dip per bad probe, no teardown (has_sun clear debounce rides
 *     through it).
 *
 * Seeding (per-panel, replaces the fixed PANEL_VREG_SETPOINT_MV):
 *   On activation the panel reading still shows the unloaded IDLE value
 *   (= open circuit; the MA filter sees the pre-activation seconds), so
 *   we capture Voc for free and seed:
 *
 *     setpoint = (MPPT_SP_FRACTION_PCT/100)·Voc − PANEL_VREG_DEADBAND_MV
 *
 *   The deadband subtraction: the inner loop approaches from open
 *   circuit and parks just under the band TOP (setpoint + deadband), so
 *   the realised operating voltage ≈ setpoint + deadband ≈ k·Voc.
 *
 *   Seeding is for a FRESH start only. A resumption after an input-loss
 *   bounce keeps the converged setpoint and Voc (enter_tracking_reprobe);
 *   re-seeding on every bounce is what turned a 2 s stand-down into a
 *   ~12 s outage and fed the teardown burst. MPPT_RESEED_GAP_MS and
 *   has_sun decide which path an activation takes.
 *
 * Convergence: a probe that measures within ±MPPT_SP_MIN_DELTA_MA of
 * the baseline counts as a reversal (don't walk on noise / on a flat
 * power top / when the allowed_chg clamp, not the panel, bounds the
 * current). MPPT_SP_CONVERGE_REVERSALS reversals → HOLD for
 * MPPT_SP_HOLD_TIME_MS, then re-probe IF the panel is still the bottleneck
 * (panel_limited) — this is the upward re-probe path that recovers full
 * capability when the sun comes back after a cloudy spell.
 *
 * This region NEVER writes ctx->pwm and NEVER updates mppt_limit_ma in
 * this mode (it stays at MPPT_LIMIT_DEFAULT_MA = BUCK_MAX; the panel is
 * protected by the voltage loop, not by the current budget).
 *
 * ── CHARGER_INPUT_VREG=0 (legacy): incremental conductance on PWM ──────
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
 *   X = dI·V + I·dV   vs   0
 *   sign(dP/dV) = sign(X)·sign(dV)
 *
 * Adaptive step halves on each direction reversal; convergence when
 * step_size == 1 and reversals ≥ MPPT_CONVERGE_REVERSALS, then HOLD
 * publishes mppt_limit_ma from the best power found. Retained for
 * stiff-source bring-up / regression comparison only.
 */

#include "mppt.h"
#include "charger.h"     /* charger_vreg_deadband_mv: the live band */
#include "SPCBoardAPI.h"

#if CHARGER_INPUT_VREG
/* =========================================================================
 * SETPOINT-P&O IMPLEMENTATION
 * ========================================================================= */

/*
 * sp_clamp — bound a candidate setpoint to the legal probe range.
 *
 * Floor, highest of three:
 *   - MPPT_SP_MIN_MV keeps the regulation band's low edge above the
 *     PANEL_SAFETY_MV emergency floor (preserves the band ordering the
 *     backoff design assumes);
 *   - MPPT_SP_FLOOR_PCT of the learned Voc — no real MPP lives below it,
 *     so probing there only walks the panel over the knee (bench
 *     10.09.26: the tracker reached 6500 on a panel whose MPP is 11850);
 *   - the learned cliff (sp_session_floor_mv), raised by every collapse /
 *     dip correction and by an input-loss teardown.
 *
 * Ceiling: learned Voc minus MPPT_SP_VOC_GUARD_MV, so the band top
 * stays below open circuit — otherwise the inner loop could "park"
 * unloaded, drawing nothing. Applied only once a credible Voc estimate
 * exists (see the comment at the ceiling check below).
 */
static uint16_t sp_clamp(const system_ctx_t *ctx, int32_t sp)
{
    const mppt_ctx_t *m = &ctx->mppt;
    int32_t band = (int32_t)charger_vreg_deadband_mv(ctx);
    int32_t lo   = (int32_t)MPPT_SP_MIN_MV;
    if (m->panel_voc_mv >= PANEL_MIN_MV) {
        int32_t voc_lo = ((int32_t)m->panel_voc_mv * MPPT_SP_FLOOR_PCT) / 100
                       - band;
        if (voc_lo > lo) lo = voc_lo;
    }
    if ((int32_t)m->sp_session_floor_mv > lo)
        lo = (int32_t)m->sp_session_floor_mv;   /* don't re-test the cliff */
    if (sp < lo) return (uint16_t)lo;

    /* No ceiling until a credible Voc estimate exists (fresh activation
     * resets it; the estimate builds as a running max — see
     * mppt_update). An over-generous ceiling is search-safe: points
     * near OC just measure poorly and are never accepted. A too-LOW
     * ceiling is not safe — it can fence the search out of the only
     * parkable region (simulation: load-biased capture + guard left
     * every reachable band hopping the knee). */
    if (m->panel_voc_mv >= PANEL_MIN_MV) {
        int32_t hi = (int32_t)m->panel_voc_mv -
                     (band + (int32_t)MPPT_SP_VOC_GUARD_MARGIN_MV);
        if (hi < lo) hi = lo;
        if (sp > hi) return (uint16_t)hi;
    }
    return (uint16_t)sp;
}

static inline int32_t abs_diff_i32(int32_t a, int32_t b)
{
    return (a > b) ? (a - b) : (b - a);
}

/* Begin a new dwell at the CURRENT setpoint: settle, then measure. */
static void start_dwell(system_ctx_t *ctx)
{
    mppt_ctx_t *m   = &ctx->mppt;
    m->dwell_start_ms   = time_now();
    m->measure_start_ms = m->dwell_start_ms;
    m->dwell_phase      = 0;        /* settling */
    m->ichg_acc       = 0;
    m->ichg_acc_cnt   = 0;
    m->dwell_dipped   = false;
}

/*
 * sp_push_up — shared "this setpoint is too low" correction, used by
 * both the collapse branch (V_panel below the emergency floor) and the
 * dip classifier (V_panel below the band: band-hop whipsaw). One step
 * toward Voc, search pointed up, fresh baseline (samples from a
 * collapsing/hopping plant are garbage).
 *
 * Deliberately does NOT count toward convergence: pushes are an escape
 * ramp out of unparkable territory, not oscillation around an optimum —
 * counting them could converge HOLD inside a whipsaw. Instead the
 * learned floor rises to the post-push level, so down-probes can never
 * re-test the cliff while this panel is trusted; they clamp against the
 * floor and the resulting boundary flips (sp_probe_step) drive
 * convergence.
 */
static void sp_push_up(system_ctx_t *ctx)
{
    mppt_ctx_t *m = &ctx->mppt;
    /* Push by the CURRENT step, not the coarse initial one: after the
     * adaptive step has refined, a fine down-probe that dips must step
     * back exactly to the known-good level — a coarse push would raise
     * the session floor PAST the accepted optimum and fence it out. */
    uint16_t up = sp_clamp(ctx, (int32_t)m->vreg_setpoint_mv +
                              (int32_t)m->sp_step_mv);
    m->vreg_setpoint_mv     = up;
    m->prev_sp_mv           = up;
    m->sp_session_floor_mv  = up;
    m->sp_direction         = +1;
    m->prev_avg_valid       = false;
}

/*
 * sp_note_reversal — bookkeeping shared by every "the other direction
 * looks better / nothing here" event: count toward convergence and
 * halve the probe step (adaptive step: coarse to find the region, fine
 * to bracket the knee below one PWM count of granularity).
 */
static void sp_note_reversal(mppt_ctx_t *m)
{
    if (m->reversals < 255) m->reversals++;
    m->sp_step_mv /= 2;
    if (m->sp_step_mv < MPPT_SP_STEP_MIN_MV)
        m->sp_step_mv = MPPT_SP_STEP_MIN_MV;
}

/*
 * sp_probe_step — move the setpoint one step in the probe direction,
 * remembering the current (accepted) setpoint as the revert target.
 *
 * If the step clamps against the floor/ceiling there is nothing to
 * probe that way: flip direction (counts toward convergence) and try
 * the other side. If BOTH sides clamp (degenerate range), the setpoint
 * stays put and the reversal counter walks us into HOLD.
 */
static void sp_probe_step(system_ctx_t *ctx)
{
    mppt_ctx_t *m = &ctx->mppt;
    uint16_t cur  = m->vreg_setpoint_mv;
    uint16_t next = sp_clamp(ctx, (int32_t)cur +
                      (int32_t)m->sp_direction * (int32_t)m->sp_step_mv);
    if (next == cur) {
        m->sp_direction = (int8_t)-m->sp_direction;
        sp_note_reversal(m);
        next = sp_clamp(ctx, (int32_t)cur +
                  (int32_t)m->sp_direction * (int32_t)m->sp_step_mv);
    }
    m->prev_sp_mv       = cur;
    m->vreg_setpoint_mv = next;
}

/*
 * sp_learn_cliff — the panel fell over at the operating point the tracker
 * had it at: either the guard rescued it (v0.32, input_dip_events ticked)
 * or the region was torn down (rearm_block_ms stamped). Nothing else
 * in this file can see that event — the collapse reaches V_bat in
 * 10-20 ms and charger_input_guard stands the region down ~40 ms in, so
 * the 640 ms average never crosses PANEL_SAFETY_MV and neither the
 * collapse branch nor the dip classifier fires. Until v0.30 the tracker
 * resumed with no memory of it and probed straight back down (bench
 * 10.09.26: 13 of 15 teardowns within 6 s of a downward probe).
 *
 * Learn the cliff from the plant, not the setpoint: with a ±1200 mV band
 * the setpoint says little about where the plant actually sat, but the
 * last clean averaged V_panel does. Put the band top MPPT_SP_CLIFF_MARGIN_MV
 * above it and make that the floor — same shape as sp_push_up, sourced
 * from a measurement instead of a step. Capped at the Voc ceiling: a
 * collapse from up there (sun dimmed under a parked point) is not a
 * setpoint problem, and a floor above the ceiling would park the loop
 * at open circuit drawing nothing.
 *
 * The floor persists through the bounce and through HOLD; only a fresh
 * entry (new Voc: sun lost, or MPPT_RESEED_GAP_MS) releases it. Known
 * trade-off: after a light cloud lowers the knee, the tracker is fenced
 * a few hundred mV above the new MPP until the next fresh seed. That
 * costs a few percent under a cloud; the alternative cost 12 s of full
 * sun per teardown.
 */
/*
 * learn_cliff_pwm — the same cliff, recorded in the domain that can act on it.
 *
 * sp_learn_cliff below teaches the setpoint, which only produces a backoff
 * once V_panel falls more than PANEL_VREG_DEADBAND_MV under it. On a panel
 * whose MPP has walked up near Voc — every low-irradiance panel, so every
 * late afternoon — the setpoint is fenced MPPT_SP_VOC_GUARD_MV below Voc and
 * V_panel sits INSIDE the band, so no reachable setpoint commands anything.
 * Bench 10.09.26: three consecutive teardowns at pwm 106 with the setpoint
 * already pinned at its ceiling, nothing left to learn, nothing to change.
 *
 * The PWM that fell over is not ambiguous the way the setpoint is: it is the
 * exact draw the panel could not hold. Fence one margin above it. The
 * ratchet is what tracks a fading Isc — the voltage loop cannot, because at
 * a near-Voc operating point the collapse is a current event with no
 * precursor in V_panel (the trace before every teardown is flat to within
 * 50 mV, then one sample of descent).
 *
 * Called before sp_learn_cliff's early returns: those guard the setpoint
 * estimate (needs a credible V_panel, and gives up once the floor is at its
 * ceiling) and neither condition says anything about the PWM.
 */
static void learn_cliff_pwm(system_ctx_t *ctx)
{
    mppt_ctx_t *m = &ctx->mppt;

    /* input_trace_pwm_from is the count in force when the guard first saw
     * the input under margin — before its own backoff moved it, and before
     * a stand-down parked it at PWM_MIN_DUTY. */
    uint32_t fell_at = (uint32_t)ctx->charger.input_trace_pwm_from;
    if (fell_at == 0 || fell_at >= PWM_MIN_DUTY)
        return;                    /* no trace, or it fell over while off */

    uint32_t floor_pwm = fell_at + CHG_CLIFF_PWM_MARGIN;
    if (floor_pwm > PWM_MIN_DUTY)
        floor_pwm = PWM_MIN_DUTY;  /* a ceiling at "off" is still a ceiling */

    if (floor_pwm <= (uint32_t)m->cliff_pwm_min)
        return;                    /* already fenced at or above this draw */

    m->cliff_pwm_min      = (uint16_t)floor_pwm;
    m->cliff_pwm_relax_ms = time_now();
}

static void sp_learn_cliff(system_ctx_t *ctx)
{
    mppt_ctx_t *m = &ctx->mppt;

    learn_cliff_pwm(ctx);

    if (m->last_good_vpanel_mv < PANEL_MIN_MV)
        return;                                  /* nothing credible learned */

    int32_t band     = (int32_t)charger_vreg_deadband_mv(ctx);
    int32_t floor_sp = (int32_t)m->last_good_vpanel_mv
                     - band
                     + (int32_t)MPPT_SP_CLIFF_MARGIN_MV;
    if (m->panel_voc_mv >= PANEL_MIN_MV) {
        int32_t hi = (int32_t)m->panel_voc_mv -
                     (band + (int32_t)MPPT_SP_VOC_GUARD_MARGIN_MV);
        if (floor_sp > hi) floor_sp = hi;
    }
    if (floor_sp <= (int32_t)m->sp_session_floor_mv)
        return;                                  /* already know this cliff */

    m->sp_session_floor_mv = (uint16_t)floor_sp;
    /* Resume above the cliff, probing away from it. */
    uint16_t up = sp_clamp(ctx, (int32_t)m->vreg_setpoint_mv);
    m->vreg_setpoint_mv = up;
    m->prev_sp_mv       = up;
    m->sp_direction     = +1;
    /* Called mid-session for a RECOVERED collapse (v0.32): the setpoint
     * just moved under a running dwell, so its measurement is garbage —
     * restart it with a fresh baseline. HOLD has no dwell to restart; the
     * lifted setpoint takes effect in cc_regulate on the same tick. */
    if (m->state == MPPT_TRACKING) {
        m->prev_avg_valid = false;
        start_dwell(ctx);
    }
}

/* =========================================================================
 * STATE ENTRY ACTIONS
 * ========================================================================= */

/* Shared TRACKING entry: fresh session counters + first (baseline) dwell
 * at the current setpoint. The learned floor (sp_session_floor_mv) is
 * deliberately NOT reset here — it belongs to the learned panel, not to
 * the session, and enter_tracking_fresh is the one place that clears it.
 * (Until v0.30 every entry reset it, so a teardown erased the cliff the
 * moment it was learned.) */
static void tracking_common(system_ctx_t *ctx)
{
    mppt_ctx_t *m = &ctx->mppt;
    m->state               = MPPT_TRACKING;
    m->reversals           = 0;
    m->sp_direction        = +1;    /* first probe toward Voc — away from the knee */
    m->sp_step_mv          = MPPT_SP_STEP_MV;
    m->prev_avg_valid      = false; /* dwell 1 establishes the baseline */
    m->seen_dip_events     = ctx->charger.input_dip_events;
    m->tracking_start_ms   = time_now();
    start_dwell(ctx);
}

/*
 * Enter TRACKING from DISABLED — charger just activated.
 *
 * Voc capture is DEFERRED to the end of the first settled dwell (see
 * voc_pending in tracking_tick). Capturing here looks tempting — the
 * panel idled unloaded until this tick — but the panel ADC is a 640 ms
 * moving average while has_sun debounces in only 150 ms: on a fresh
 * plug-in this tick's reading is HALF-SETTLED (a mix of pre-plug zeros
 * and the real voltage). Seeding from it poisons both the FOCV estimate
 * and the setpoint ceiling (simulation: captured 8.5 V on a 13.3 V
 * panel and trapped the tracker at a whipsaw point). Until the capture
 * lands, the inner loop regulates to the preserved/fallback setpoint.
 */
static void enter_tracking_fresh(system_ctx_t *ctx)
{
    ctx->mppt.voc_pending         = true;
    ctx->mppt.panel_voc_mv        = 0;   /* maybe a different panel — relearn
                                          * Voc from scratch via the running
                                          * max                            */
    ctx->mppt.sp_session_floor_mv = MPPT_SP_MIN_MV;  /* ...and its cliff  */
    ctx->mppt.cliff_pwm_min       = 0;               /* ...in both domains */
    ctx->mppt.last_good_vpanel_mv = 0;
    ctx->charger.plant_mv_per_count = 0;  /* ...and its gain: the regulation
                                           * band goes back to the static
                                           * PANEL_VREG_DEADBAND_MV until the
                                           * voltage loop has measured this
                                           * panel (hw_config.h) */
    ctx->charger.vloop_measure_armed = false;
    tracking_common(ctx);
}

/*
 * Enter TRACKING from HOLD (periodic re-probe) or from DISABLED after an
 * input-loss bounce (v0.28). Keep the setpoint, the learned floor and
 * the Voc captured at activation (the panel is loaded now, so a
 * re-capture would read low); the baseline dwell re-measures the kept
 * point first so the comparison is against CURRENT conditions, not
 * against a minute-old number.
 */
static void enter_tracking_reprobe(system_ctx_t *ctx)
{
    ctx->mppt.voc_pending = false;
    tracking_common(ctx);
    /* Start FINE: we re-probe from a converged point, so the optimum is
     * almost certainly within a step or two — a coarse first probe
     * over-jumps the best parkable PWM count and its dip floors the
     * session above it. If conditions really moved, the accept-doubling
     * in the better-branch recovers coarse range within a few dwells. */
    ctx->mppt.sp_step_mv = MPPT_SP_STEP_MIN_MV;
}

/*
 * Enter HOLD — freeze the setpoint where the climb converged. The inner
 * loop keeps regulating to it; nothing else to do. mppt_limit_ma is
 * deliberately NOT touched in this mode (see file header).
 */
static void enter_hold(system_ctx_t *ctx)
{
    ctx->mppt.state         = MPPT_HOLD;
    ctx->mppt.hold_start_ms = time_now();
}

/*
 * Enter DISABLED — charger region went inactive. The setpoint and the
 * captured Voc are PRESERVED (same rationale as mppt_limit_ma in the
 * legacy path): a brief EM bounce must not forget the learned MPP, and
 * cc_regulate keeps consuming the setpoint the moment charging resumes.
 *
 * Until v0.28 that preservation was pointless, because the resume path
 * went through enter_tracking_fresh() unconditionally and wiped both.
 * Now the gap is timed instead: the learned point stays trusted until
 * has_sun drops or MPPT_RESEED_GAP_MS elapses (see mppt_update), and
 * only then does the next activation re-seed from FOCV.
 */
static void enter_disabled(system_ctx_t *ctx)
{
    ctx->mppt.state             = MPPT_DISABLED;
    ctx->mppt.disabled_since_ms = time_now();
    ctx->mppt.seed_invalidated  = false;
}

/* =========================================================================
 * TRACKING TICK — dwell sequencing + P&O decision
 * ========================================================================= */

static void tracking_tick(system_ctx_t *ctx)
{
    mppt_ctx_t *m   = &ctx->mppt;
    uint32_t now    = time_now();
    uint32_t dwelt  = now - m->dwell_start_ms;

    /* ── Collapse branch ──
     * The probe drove the panel over its I-V knee (V_panel below the
     * emergency floor — the inner backoff is already rescuing the
     * rail). The setpoint is too low: push it back up toward Voc
     * (raising the session floor) and restart with a fresh baseline —
     * any samples spanning a collapse are garbage.
     *
     * Blanked for MPPT_SP_COLLAPSE_BLANK_MS after each dwell start so
     * ONE collapse event produces ONE correction: the collapsed reading
     * persists through the MA filter + paced backoff for ~1 s after the
     * cause is removed, and re-acting on that stale tail would ratchet
     * the setpoint up several bogus steps (the same dead-time trap that
     * broke the original every-tick backoff). */
    if (ctx->meas.panel_voltage < PANEL_SAFETY_MV) {
        if (dwelt >= MPPT_SP_COLLAPSE_BLANK_MS) {
            sp_push_up(ctx);
            start_dwell(ctx);
        }
        return;   /* collapsed reading: never measure, let the rail recover */
    }

    /* ── Phase 0: settle ──
     *
     * "Settled" means the INNER loop has arrived, not that a timer expired.
     * cc_regulate realises a setpoint change at a few PWM counts per
     * PANEL_VREG_INTERVAL_MS, so a coarse probe takes seconds to reach the
     * plant — bench 14.09.26, the activation walk pwm 106→84 took 9 s, and
     * the tracker ran three whole dwells against a plant still in transit
     * and then compared their averages as though they were fitness.
     *
     * So wait for both: the timer AND V_panel inside the band. Capped at
     * MPPT_SP_SETTLE_MAX_MS for the setpoint that is never reached at all
     * (a stiff source, or one fenced out of reach by cliff_pwm_min) — that
     * case measures a not-quite-arrived plant, which is exactly what every
     * dwell did before. */
    if (m->dwell_phase == 0) {
        bool arrived =
            (uint32_t)abs_diff_i32((int32_t)ctx->meas.panel_voltage,
                                   (int32_t)m->vreg_setpoint_mv)
            <= (uint32_t)charger_vreg_deadband_mv(ctx);

        if (dwelt >= MPPT_SP_SETTLE_MS &&
            (arrived || dwelt >= MPPT_SP_SETTLE_MAX_MS)) {
            /* Deferred FOCV seed (fresh activation only), now that the
             * MA has fully settled past the plug-in transient:
             *   sp = k·Voc − deadband
             * (the inner loop parks just under the band TOP = sp +
             * deadband, so the realised voltage ≈ k·Voc). Then restart
             * the dwell: the baseline must be measured AT the seed.
             * If no credible Voc accumulated (panel died during the
             * settle) keep the previous seed. */
            if (m->voc_pending) {
                m->voc_pending = false;
                /* panel_voc_mv is the running MAX of the filtered panel
                 * voltage since activation (see mppt_update) — it caught
                 * the unloaded pre-pull-down peak even though the
                 * reading RIGHT NOW is under load. It keeps improving
                 * after this seed (every backoff recovery samples
                 * near-OC); the ceiling rises with it. */
                if (m->panel_voc_mv >= PANEL_MIN_MV) {
                    int32_t seed = ((int32_t)m->panel_voc_mv *
                                    MPPT_SP_FRACTION_PCT) / 100
                                 - (int32_t)charger_vreg_deadband_mv(ctx);
                    m->vreg_setpoint_mv = sp_clamp(ctx, seed);
                    m->prev_sp_mv       = m->vreg_setpoint_mv;
                }
                start_dwell(ctx);
                return;
            }
            m->dwell_phase      = 1;
            m->measure_start_ms = now;
        }
        return;
    }

    /* ── Phase 1: measure ── */
    m->ichg_acc += (int32_t)ctx->meas.chg_current;
    m->ichg_acc_cnt++;

    /* Dip classifier: a parked plant never reads below the band's low
     * edge. If V_panel does, the band contains no reachable operating
     * point at this setpoint (one PWM count hops clear across it — the
     * band-hop whipsaw on the panel's current-source side) and the
     * measured average is oscillation-phase noise, not fitness. The
     * 300 mV margin ignores parked readings grazing the edge. */
    if ((int32_t)ctx->meas.panel_voltage <
        (int32_t)m->vreg_setpoint_mv -
        (int32_t)charger_vreg_deadband_mv(ctx) - 300)
        m->dwell_dipped = true;

    if ((now - m->measure_start_ms) < MPPT_SP_MEASURE_MS)
        return;

    /* ── Whipsaw dwell: the setpoint is too low to park — push up.
     * Deterministic, average ignored: dwell-to-dwell comparisons inside
     * a whipsaw region random-walk (simulation: converged at a hopping
     * point delivering ~70 % of MPP and never climbed out). ── */
    if (m->dwell_dipped) {
        sp_push_up(ctx);
        start_dwell(ctx);
        return;
    }

    int16_t avg = (int16_t)(m->ichg_acc / (int32_t)m->ichg_acc_cnt);

    /* ── Baseline dwell: record fitness, take the first probe step ── */
    if (!m->prev_avg_valid) {
        m->prev_avg_ichg  = avg;
        m->prev_avg_valid = true;
        sp_probe_step(ctx);
        if (m->reversals >= MPPT_SP_CONVERGE_REVERSALS) {
            enter_hold(ctx);    /* degenerate range: both directions clamped */
            return;
        }
        start_dwell(ctx);
        return;
    }

    /* ── Comparison dwell: P&O decision with noise gate ── */
    int32_t delta = (int32_t)avg - (int32_t)m->prev_avg_ichg;

    if (delta > (int32_t)MPPT_SP_MIN_DELTA_MA) {
        /* Better → accept this setpoint, keep climbing the same way,
         * and re-grow the step (capped at the initial size): consecutive
         * accepts mean there's real distance to cover. */
        m->prev_avg_ichg = avg;
        m->sp_step_mv *= 2;
        if (m->sp_step_mv > MPPT_SP_STEP_MV)
            m->sp_step_mv = MPPT_SP_STEP_MV;
        sp_probe_step(ctx);
    } else if (delta < -(int32_t)MPPT_SP_MIN_DELTA_MA) {
        /* Worse → revert to the accepted setpoint and flip. Baseline is
         * invalidated so the next dwell RE-MEASURES the reverted point
         * under current conditions (irradiance may have moved between
         * the two dwells — never compare against a stale number). */
        m->vreg_setpoint_mv = m->prev_sp_mv;
        m->sp_direction     = (int8_t)-m->sp_direction;
        sp_note_reversal(m);
        m->prev_avg_valid   = false;
    } else {
        /* Flat: noise floor, flat power top, or the allowed_chg clamp
         * (not the panel) bounds the current. Don't walk — flip and
         * count toward convergence. Baseline follows the fresh reading
         * so slow drift doesn't accumulate into a fake delta. */
        m->prev_avg_ichg = avg;
        m->sp_direction  = (int8_t)-m->sp_direction;
        sp_note_reversal(m);
        sp_probe_step(ctx);
    }

    if (m->reversals >= MPPT_SP_CONVERGE_REVERSALS) {
        enter_hold(ctx);
        return;
    }
    start_dwell(ctx);
}

/* =========================================================================
 * PUBLIC: step 6 entry point (setpoint-P&O)
 * ========================================================================= */

void mppt_update(system_ctx_t *ctx)
{
    mppt_ctx_t *m = &ctx->mppt;
    const bool has_sun  = ctx->flag_has_sun.value;
    const bool charging = (ctx->charger.state == CHG_PRECHARGE) ||
                          (ctx->charger.state == CHG_CC) ||
                          (ctx->charger.state == CHG_CV);

    /* Nothing to optimise until Q49 is connected. CHG_BUCK_SETTLE runs the
     * buck intentionally unloaded and owns PWM acquisition; starting a P&O
     * dwell there would measure zero charge current, pollute its baseline,
     * and report MPPT tracking before charging has actually begun. */
    if (!charging) {
        if (m->state != MPPT_DISABLED) {
            /* Why did the region go down? energy_mode (step 5, this same
             * tick) stamps rearm_block_ms only for an input-loss stand-
             * down, so a block still in the future means the panel just
             * collapsed under us: learn the level before forgetting the
             * session. Any other exit (CV, fault, sun lost) teaches
             * nothing about the knee. */
            if ((int32_t)(ctx->charger.rearm_block_ms - time_now()) > 0)
                sp_learn_cliff(ctx);
            enter_disabled(ctx);
        }

        /* While the region is down, decide whether the learned Voc and
         * setpoint survive the gap. Both signals mean "this may not be the
         * same panel, or not the same day":
         *   - has_sun cleared: the debounced source-present flag went away,
         *     which is what a GENUINE disconnect trips (1.5 s) and what an
         *     input-loss bounce never manages (it re-arms in 2 s with the
         *     averaged panel voltage still sitting at 11 V).
         *   - the gap ran past MPPT_RESEED_GAP_MS: a long stand-down, a
         *     fault round-trip, or an overnight idle.
         * Latched, never cleared here — enter_disabled() re-arms it on the
         * next teardown and the MPPT_DISABLED entry path consumes it. */
        if (!has_sun ||
            (time_now() - m->disabled_since_ms) >= MPPT_RESEED_GAP_MS)
            m->seed_invalidated = true;

        return;
    }

    /* Voc estimate: running max of the filtered panel voltage while the
     * region is active. V_panel never exceeds open circuit (barring the
     * reverse-pump pathology, where an inflated estimate only loosens
     * the search ceiling — safe), and it touches near-OC repeatedly:
     * the unloaded stretch right after activation, every backoff
     * recovery, light-load moments. This self-corrects the load bias of
     * the activation-time capture. */
    if (m->state != MPPT_DISABLED &&
        ctx->meas.panel_voltage > m->panel_voc_mv)
        m->panel_voc_mv = ctx->meas.panel_voltage;

    /* A collapse the guard rescued (charger_input_guard backed the draw off
     * and the panel came back) is a cliff sighting exactly like a teardown,
     * minus the teardown. Learn it BEFORE this tick's V_panel is recorded
     * below — the reading that matters is the one from before the dip. */
    if (m->state != MPPT_DISABLED &&
        ctx->charger.input_dip_events != m->seen_dip_events) {
        m->seen_dip_events = ctx->charger.input_dip_events;
        sp_learn_cliff(ctx);
    }

    /* Relax the learned PWM ceiling while nothing is falling over. The
     * ratchet above only ever fences MORE conservatively, so a haze that
     * lifts without ever dropping has_sun (no fresh seed, no release) would
     * otherwise keep the draw capped at the level the haze imposed. One
     * count per CHG_CLIFF_PWM_RELAX_MS, and only while actually charging —
     * an inactive region proves nothing about the knee. The stamp is shared
     * with the ratchet, so a fresh collapse also restarts the clock. */
    if (m->cliff_pwm_min != 0 &&
        (time_now() - m->cliff_pwm_relax_ms) >= CHG_CLIFF_PWM_RELAX_MS) {
        m->cliff_pwm_relax_ms = time_now();
        m->cliff_pwm_min--;
        if (m->cliff_pwm_min <= PWM_MAX_DUTY)
            m->cliff_pwm_min = 0;   /* down to the hardware limit: no fence */
    }

    /* Cliff memory for sp_learn_cliff: the last averaged V_panel that was
     * not already dragged down by a collapse. A collapsed 10 ms sample
     * costs the 64-deep average ~125-190 mV; honest regulation near the
     * knee moves it ~50 mV per tick. Reject the sharp falls, keep the
     * rest (rises are always clean — nothing collapses upward). */
    if (m->state != MPPT_DISABLED &&
        ctx->meas.panel_voltage >= PANEL_SAFETY_MV &&
        ((int32_t)ctx->meas.panel_voltage + (int32_t)MPPT_SP_VPANEL_DROP_MV
            >= (int32_t)m->last_good_vpanel_mv))
        m->last_good_vpanel_mv = ctx->meas.panel_voltage;

    switch (m->state) {

    case MPPT_DISABLED:
        /* Charger just activated (this very tick — energy_mode runs at
         * step 5, we run at step 6, so the panel reading is still the
         * unloaded IDLE value). Two ways in:
         *
         * FRESH (cold boot, new panel, long gap): capture Voc and seed
         * from FOCV, then start climbing. No settle gate needed — the
         * first dwell's 2 s settle phase absorbs the activation
         * transient.
         *
         * RE-PROBE (input-loss bounce): the learned point is still good,
         * so resume from it at the fine step instead of throwing it away.
         * Re-seeding here was the dominant cost of a teardown — bench
         * 10.09.26 measured a median 12.4 s to climb back to full current
         * against the 2 s the re-arm block itself costs, and the freshly
         * seeded setpoint sat below the knee in the meantime, walking the
         * panel off the cliff again (the teardowns arrived in bursts).
         * See MPPT_RESEED_GAP_MS.
         *
         * The panel_voc_mv guard is not optional: it stops a stale
         * seed_invalidated from routing a boot with no credible Voc into
         * the re-probe path, which would have neither a real setpoint nor
         * a setpoint ceiling. */
        if (has_sun) {
            if (m->seed_invalidated || m->panel_voc_mv < PANEL_MIN_MV)
                enter_tracking_fresh(ctx);
            else
                enter_tracking_reprobe(ctx);
        }
        break;

    case MPPT_TRACKING:
        /* P1: sun lost → DISABLED (EM teardown follows). */
        if (!has_sun) {
            enter_disabled(ctx);
            break;
        }
        /* P2: charger reached CV → freeze. cv_regulate now regulates
         * V_bat and ignores the setpoint, so probing would move the
         * setpoint blind. HOLD keeps it for a later CC return. */
        if (ctx->charger.state == CHG_CV) {
            enter_hold(ctx);
            break;
        }
        /* P3: session runtime cap → HOLD with the best point so far. */
        if ((time_now() - m->tracking_start_ms) >= MPPT_SP_RUNTIME_MS) {
            enter_hold(ctx);
            break;
        }
        /* P4: run the dwell/decision machinery. */
        tracking_tick(ctx);
        break;

    case MPPT_HOLD:
        /* P1: sun lost → DISABLED. */
        if (!has_sun) {
            enter_disabled(ctx);
            break;
        }
        /* P2: collapse escape. The held setpoint started collapsing the
         * panel — the knee moved ABOVE the held band (sun got brighter:
         * knee voltage rises with irradiance) or a shadow cut the
         * available current. Either way the inner loop alone would
         * sawtooth here until the hold expired. Step the setpoint up
         * one notch and re-enter TRACKING: its collapse branch takes
         * over the (paced) ratchet if one step isn't enough, and the
         * climb re-converges on the new optimum.
         *
         * Blanked for MPPT_SP_COLLAPSE_BLANK_MS after HOLD entry: when
         * TRACKING converges right after a collapse correction, the
         * collapsed reading persists through the MA filter for ~1 s —
         * reacting to that stale tail would double-correct. And only
         * escape if the step actually moved (a setpoint pinned at the
         * ceiling has nowhere to go — churn into TRACKING would learn
         * nothing the hold-expiry re-probe won't). */
        if (ctx->meas.panel_voltage < PANEL_SAFETY_MV &&
            (time_now() - m->hold_start_ms) >= MPPT_SP_COLLAPSE_BLANK_MS) {
            uint16_t up = sp_clamp(ctx, (int32_t)m->vreg_setpoint_mv +
                                      (int32_t)MPPT_SP_STEP_MV);
            if (up != m->vreg_setpoint_mv) {
                m->vreg_setpoint_mv = up;
                m->prev_sp_mv       = up;
                enter_tracking_reprobe(ctx);
            }
            break;
        }
        /* P3: re-probe when the hold expires — but only while the PANEL
         * is the binding constraint (panel_limited) and the charger is
         * actually running a bulk state. If the battery/budget is what
         * bounds the current (CV taper, precharge trickle, near-full)
         * or the charger is fault-parked, probing cannot observe
         * anything: stay held. This re-probe is the upward path that
         * recovers capability after clouds pass. */
        if ((time_now() - m->hold_start_ms) >= MPPT_SP_HOLD_TIME_MS &&
            ctx->panel_limited &&
            (ctx->charger.state == CHG_PRECHARGE ||
             ctx->charger.state == CHG_CC)) {
            enter_tracking_reprobe(ctx);
        }
        break;

    default:
        enter_disabled(ctx);
        break;
    }
}

#else  /* !CHARGER_INPUT_VREG — legacy PWM-perturbing incremental conductance */

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
    m->stuck_ticks    = 0;
    m->max_power      = 0;
    m->max_power_pwm  = ctx->pwm;
    m->last_direction = -1;  /* first perturbation is pwm -= step (V falls) */
    m->tracking_start_ms = time_now();
    m->last_step_ms      = time_now();  /* first observe is one interval out */

    /* Force first perturbation. The observe happens MPPT_STEP_INTERVAL_MS
     * later (not next tick) so the moving-average filter has settled onto
     * this perturbation before we measure dV/dI against the entry baseline
     * snapshotted in v_prev/i_prev above. */
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

    /* Park at best operating point. max_power_pwm is seeded with the
     * entry pwm in enter_tracking(), so this is also the safe fallback
     * when nothing was learned (max_power == 0) — e.g. stiff bench PSU
     * where dV=dI=0 makes tracking march monotonically into a high-duty
     * zone. Leaving pwm at the last walked-down value would park HOLD at
     * the most aggressive point tried, which is the worst possible
     * default. */
    ctx->pwm = m->max_power_pwm;

    /* Derive I_bus max from P_max / V_bat. Guard against tiny V_bat. */
    uint16_t v_bat = ctx->meas.bat_voltage;
    if (v_bat < 1000) v_bat = 1000;  /* avoid divide-by-tiny on missing battery */

    int32_t i_bus_ma = (m->max_power * 1000) / (int32_t)v_bat;
    if (i_bus_ma < 0) i_bus_ma = 0;
    if (i_bus_ma > BUCK_MAX_CURRENT_MA) i_bus_ma = BUCK_MAX_CURRENT_MA;

    m->mppt_limit_ma = (uint16_t)i_bus_ma;
}

/*
 * Enter DISABLED — CC/CV resumes PWM control. mppt_limit_ma is
 * PRESERVED across this transition; the last-known panel capability
 * stays in effect so power_budget continues to clamp allowed_chg.
 *
 * The HOLD `!panel_limited` exit calls us when MPPT thinks the panel
 * is no longer the bottleneck. With the preserved limit semantics,
 * that transition becomes a no-op for the budget — DISABLED stays
 * clamped at the same value HOLD published. This is intentional:
 * releasing to BUCK_MAX here would re-trigger the panel overload that
 * caused MPPT to engage in the first place. Upward re-probe happens
 * when MPPT enters TRACKING again (currently only on activation when
 * panel_limited fires).
 */
static void enter_disabled(system_ctx_t *ctx)
{
    ctx->mppt.state = MPPT_DISABLED;
    ctx->mppt.step_size = MPPT_MAX_STEP_SIZE;
    ctx->mppt.reversals = 0;
    ctx->mppt.stuck_ticks = 0;
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
     * Count consecutive ticks at MAX step with no reversal. With a
     * current-limited PV panel the buck pulls V_panel down the I-V
     * curve and dV/dI provide the feedback that drives reversals.
     * With a stiff source (bench PSU), dV ≈ dI ≈ 0 and direction
     * stays pinned at last_direction — the loop marches one way at
     * full step until something trips. Bail to HOLD before that. */
    if (m->step_size >= MPPT_MAX_STEP_SIZE && !reversed) {
        if (m->stuck_ticks < 255) m->stuck_ticks++;
    } else {
        m->stuck_ticks = 0;
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
 * PUBLIC: step 6 entry point (legacy)
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
    const bool charging      = (ctx->charger.state == CHG_PRECHARGE) ||
                               (ctx->charger.state == CHG_CC) ||
                               (ctx->charger.state == CHG_CV);

    /* CHG_BUCK_SETTLE owns PWM while Q49 is open. Treat it like an inactive
     * charger here so MPPT cannot perturb the unloaded acquisition rail. */
    if (!charging) {
        if (m->state != MPPT_DISABLED) enter_disabled(ctx);
        return;
    }

    switch (m->state) {

    case MPPT_DISABLED:
        /* P1: panel_limited AND has_sun AND charger past settle window → TRACKING.
         *
         * Settle gate: when the charger has just activated from
         * CHG_INACTIVE, ctx->pwm starts at PWM_MIN_DUTY (off) and
         * chg_current=0 trivially makes panel_limited=true. Without
         * this gate, MPPT would steal PWM control on the very first
         * charging tick and walk PWM down by MAX_STEP_SIZE before any
         * current measurement comes back — slamming a stiff source
         * straight into FAULT_OVERCURRENT_CHG. CC_PWM_STEP=1 needs
         * ~1 s to descend through the responsive PWM range first. */
        {
            uint32_t since_active = time_now() - ctx->charger.active_start_ms;
            bool settled = (ctx->charger.state != CHG_INACTIVE) &&
                           (since_active >= CHARGER_MPPT_SETTLE_MS);
            if (panel_limited && has_sun && settled) {
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
        /* Run one perturb/observe iteration, but only once per
         * MPPT_STEP_INTERVAL_MS so each step settles through the ADC
         * moving average before the next observation. Between paced
         * steps we hold PWM and let the filter catch up. Convergence /
         * stuck checks only change after a step, so they're evaluated
         * inside the paced block. */
        if ((time_now() - m->last_step_ms) >= MPPT_STEP_INTERVAL_MS) {
            tracking_step(ctx);
            m->last_step_ms = time_now();

            /* P2: converged → HOLD */
            if (tracking_converged(m)) {
                enter_hold(ctx);
                break;
            }
            /* P4: stiff source → HOLD (parks PWM at max_power_pwm = entry
             *     pwm, since stuck means dV=dI=0 and max_power never updated). */
            if (m->stuck_ticks >= MPPT_STUCK_TICK_LIMIT) {
                enter_hold(ctx);
                break;
            }
        }
        /* P3: runtime timeout → HOLD (time-based, checked every tick) */
        if (tracking_timed_out(m)) {
            enter_hold(ctx);
            break;
        }
        /* P5: continue tracking */
        break;

    case MPPT_HOLD:
        /* P3 (checked first for safety): !has_sun → DISABLED */
        if (!has_sun) {
            enter_disabled(ctx);
            break;
        }
        /* P1: !panel_limited → DISABLED, EXCEPT when the mppt cap itself
         *     is what's holding allowed_chg below PANEL_LIMITED_MARGIN_MA.
         *     In that case panel_limited is structurally forced false by
         *     the measurements guard (allowed_chg <= margin), and dropping
         *     to DISABLED would release mppt_limit to BUCK_MAX, re-trigger
         *     panel_limited, and bounce us straight back into TRACKING.
         *     Stay parked in HOLD instead — the panel really can't deliver
         *     more, and the periodic re-entry to TRACKING after
         *     MPPT_HOLD_TIME_MS will retest. */
        if (!panel_limited && m->mppt_limit_ma > PANEL_LIMITED_MARGIN_MA) {
            enter_disabled(ctx);
            break;
        }
        /* P2: hold time expired AND has_sun → TRACKING */
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

#endif /* CHARGER_INPUT_VREG */
