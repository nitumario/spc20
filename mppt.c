/*
 * mppt.c — Maximum Power Point Tracker (Pipeline Step 6)
 * =========================================================================
 *
 * Two implementations, selected by CHARGER_INPUT_VREG (hw_config.h):
 *
 * ── CHARGER_INPUT_VREG=1 (current): KNEE P&O ON THE PWM FENCE ──────────
 *
 * Perturb ctx->mppt.cliff_pwm_min — the regulator's current ceiling, in PWM
 * counts — one count at a time, and measure averaged delivered charge
 * current. The count that stops paying is the knee; park one margin above
 * it. cc_regulate's voltage loop drives the operating point down into the
 * fence and handles sag; charger_panel_droop_guard handles the 10 ms
 * emergency. This region never writes ctx->pwm and never updates
 * mppt_limit_ma.
 *
 * There is no seed, no Voc fraction and nothing that knows the panel: the
 * full rationale, the bench measurements behind it, and what it replaces
 * (the FOCV setpoint tracker of v0.18-v0.39) are in the block comment
 * immediately below the #if — read that before changing anything here.
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
 * KNEE P&O ON THE PWM FENCE  (v0.40)
 * =========================================================================
 *
 * WHAT CHANGED AND WHY (read this before touching anything below).
 *
 * v0.18-v0.39 perturbed ctx->mppt.vreg_setpoint_mv — a millivolt target —
 * and let cc_regulate realise it in PWM counts. The 14.09.26 bench session
 * is the case against that arrangement, and it is not a tuning case:
 *
 *   1. THE SEED WAS PAST THE CLIFF. MPPT_SP_FRACTION_PCT = 87 on a panel
 *      with Voc 13208 mV seeds 11.49 V. The knee that day was at 11.9 V.
 *      The inner loop drove faithfully toward the seed and went over the
 *      edge at ms 463508 — the first of four collapses, caused by the
 *      seeding constant itself. No fixed fraction of Voc can be right here:
 *      Vmp/Voc runs 0.71-0.82 at STC and above 0.90 at the low irradiance
 *      this system actually lives in, and it moves again with array
 *      topology — which is the one thing the firmware is meant not to know.
 *
 *   2. THE SETPOINT CANNOT RESOLVE THE KNEE. The plant gain measured 26-55
 *      mV/count, so the regulation band (2 counts, floored at
 *      PANEL_VREG_DEADBAND_MIN_MV) was several counts wide — while the knee
 *      is ONE count wide: pwm 79 delivered 1297 mA, pwm 78 delivered 1284,
 *      pwm 77 collapsed the panel. Asking a millivolt setpoint to park one
 *      count off a knee is asking for a resolution it does not have.
 *
 *   3. THE FENCE WAS A SCAR, AND IT ATE THE HARVEST. cliff_pwm_min was only
 *      ever written by a collapse, CHG_CLIFF_PWM_MARGIN (6) counts above the
 *      count that fell over, and released one count per 180 s. Four
 *      collapses walked it 0 -> 79 -> 83 -> 91 -> 95 in ten minutes. The
 *      session's last 280 s ran at 1.7 W on a panel that had just delivered
 *      4.6 W, with the tracker "converged" — on the fence, not on the panel.
 *
 * So the fence became the actuator. This region now perturbs cliff_pwm_min
 * one PWM count at a time and measures averaged delivered charge current;
 * cc_regulate's voltage loop drives the operating point down into it and
 * pwm_draw_more meters the descent. The tracker owns WHERE the ceiling is;
 * the voltage loop owns getting there and backing off a sag; the new
 * charger_panel_droop_guard owns the 10 ms emergency.
 *
 * Nothing in the search knows anything about the panel. Fitness is
 * delivered current, the actuator is a PWM count, and the stopping rule is
 * "the next count did not pay". Four panels in parallel, six, eight, or the
 * same array rewired in series, all put the knee at a different count and a
 * different current, and the search is identical. That is the property the
 * FOCV seed could never have.
 *
 * WHY WE PARK OFF THE KNEE ON PURPOSE (MPPT_KNEE_MARGIN).
 * A buck held at a fixed PWM into a stiff cell is a constant-power load,
 * and a CPL load line is tangent to the panel I-V curve exactly at the MPP.
 * Every operating point at or left of the MPP is therefore open-loop
 * unstable, and this firmware's feedback — 400 ms loop, 320 ms of ADC group
 * delay — is four orders of magnitude too slow to stabilise it. The MPP is
 * a boundary, not a seat: two of the four collapses on 14.09 happened with
 * the PWM frozen for seconds. One count off costs ~3 % and is the whole of
 * the stability margin, so that is where we sit.
 *
 * Fitness is chg_current, NOT panel power: V_bat is constant over a dwell,
 * so max delivered current is max delivered power, converter losses
 * included — which is what we actually want to maximise. It is also the
 * cleanest channel on the board (the panel-current sense disagrees with the
 * output by 7 % of efficiency across a 3-count step, so P_panel ratios are
 * not trustworthy at this resolution).
 *
 * ── CHARGER_INPUT_VREG=0 (legacy): incremental conductance on PWM ──────
 * Unchanged, below the #else. Stiff-source bring-up only.
 */

/* =========================================================================
 * FENCE HELPERS
 *
 * PWM sign, every time: LOWER count = HIGHER duty = MORE current.
 * cliff_pwm_min is a FLOOR on the count, i.e. a CEILING on current.
 * "Probing down" means lowering the count to ask for more.
 * ========================================================================= */

static inline int32_t abs_diff_i32(int32_t a, int32_t b)
{
    return (a > b) ? (a - b) : (b - a);
}

/*
 * fence_floor — the lowest count the search may command.
 *
 * A proven knee fences MPPT_KNEE_MARGIN above itself; with nothing proven
 * the only bound is the hardware limit. PWM_MAX_DUTY (1) is the smallest
 * legal count — reaching it means the panel is stiffer than the buck, which
 * is the bench-PSU case, and the allowed_chg clamp takes over from there.
 */
static uint16_t fence_floor(const mppt_ctx_t *m)
{
    if (m->knee_pwm == 0)
        return PWM_MAX_DUTY;
    uint32_t f = (uint32_t)m->knee_pwm + MPPT_KNEE_MARGIN;
    if (f > PWM_MIN_DUTY) f = PWM_MIN_DUTY;
    return (uint16_t)f;
}

/*
 * knee_learn — record a count that the panel could not hold, and fence
 * above it.
 *
 * Two sources, two margins:
 *   - a MEASURED knee (a probe that delivered less) gets MPPT_KNEE_MARGIN,
 *     because the count is exact and one count is the whole distance from
 *     the optimum to the cliff;
 *   - a COLLAPSED knee (droop guard, input-guard rescue, or teardown) gets
 *     CHG_CLIFF_PWM_MARGIN, because the event moved the PWM before anyone
 *     could read it and the snapshot is one conversion old at best.
 * Both land on 79 for the 14.09 array (measured 78 + 1; collapsed 77 + 2),
 * which is the count that delivered the session's best 1297 mA.
 *
 * Ratchets only: a knee already known to be higher (more conservative) is
 * not lowered here. HOLD's periodic down-probe is the way back, and it is
 * the only way back — the blind relax timers it replaces (v0.39's 2 s
 * draw-blocked release and 180 s ratchet relax) both failed in the same
 * session, one by marching straight back into the cliff and one by being
 * far too slow to follow the sun.
 *
 * Bounded by charger.delivering_pwm exactly as v0.39's learn_cliff_pwm was:
 * a count this session has been SEEN delivering at is proof the panel
 * sustains it, and a fence at or above it is provably wrong.
 */
static void knee_learn(system_ctx_t *ctx, uint32_t bad_pwm, uint32_t margin,
                       uint16_t vpanel_at)
{
    mppt_ctx_t *m = &ctx->mppt;

    if (bad_pwm == 0 || bad_pwm >= PWM_MIN_DUTY)
        return;                         /* no trace, or it fell over while off */

    /* A COARSE probe that fell over says the knee is somewhere inside the
     * jump. Fencing at bad_pwm + margin can land BELOW the count that last
     * paid — on the 14.09 array a coarse probe 82 -> 79 -> 76 droops at 76
     * and fences at 78, one count under the 79 that had just delivered the
     * session's best current — and the next probe then walks back into the
     * same fall. Give the known-good count back first; the ratchet below
     * still applies, so the fence ends at whichever is more conservative. */
    if (m->probing && m->probe_step > 1U &&
        m->probe_from_pwm > m->cliff_pwm_min) {
        m->cliff_pwm_min = m->probe_from_pwm;
        m->probe_step    = 0;
        m->probing       = false;
    }

    if (bad_pwm > (uint32_t)m->knee_pwm) {
        m->knee_pwm        = (uint16_t)bad_pwm;
        m->knee_learned_ms = time_now();
        m->knee_vpanel_mv  = vpanel_at;     /* the light it was learned in */
    }

    uint32_t fence = bad_pwm + margin;
    if (fence > PWM_MIN_DUTY)
        fence = PWM_MIN_DUTY;

    /* Counter-evidence (v0.39, kept): never fence at or above a count this
     * session actually delivered at, and only while that evidence sits ABOVE
     * the count that fell over — inside one margin the two say the same
     * thing and the margin is the one to trust. */
    uint16_t delivered = ctx->charger.delivering_pwm;
    if (delivered != 0 && (uint32_t)delivered > bad_pwm &&
        fence > (uint32_t)delivered)
        fence = (uint32_t)delivered;

    if (fence > (uint32_t)m->cliff_pwm_min)
        m->cliff_pwm_min = (uint16_t)fence;
}

/*
 * knee_learn_event (v0.41) — knee_learn for a sighting that came from an
 * EVENT (a droop, an input-guard rescue, a teardown) rather than from a
 * measured dwell: the count must have been delivering, or the event says
 * nothing about the knee. The guard's backoff has already happened either
 * way; only the tracker's memory is gated. Returns whether the claim was
 * accepted, for the log.
 *
 * v0.42 dropped the Voc-fraction gate v0.41 added here: it rejected a real
 * collapse at 96.6 % of Voc on the 14.09.26 afternoon session, and the
 * open-circuit "flickers" it existed for were the buck's own switching
 * transient, which the droop guard now recognises by its current signature
 * (CHG_DROOP_SPIKE_MA) and never reports as an event at all.
 */
static bool knee_learn_event(system_ctx_t *ctx, uint32_t bad_pwm,
                             int16_t ichg_at, uint16_t vpanel_at,
                             bool settled)
{
    if (ichg_at < (int16_t)CHG_DELIVERING_MIN_MA)
        return false;                   /* nothing was being drawn there */
    if (!settled)
        return false;                   /* v0.44: the average describes the
                                         * PREVIOUS count, not this one     */

    knee_learn(ctx, bad_pwm, CHG_CLIFF_PWM_MARGIN, vpanel_at);
    return true;
}

/* Begin a new dwell at the CURRENT fence: settle, then measure. */
static void start_dwell(system_ctx_t *ctx)
{
    mppt_ctx_t *m       = &ctx->mppt;
    m->dwell_start_ms   = time_now();
    m->measure_start_ms = m->dwell_start_ms;
    m->dwell_phase      = 0;        /* settling */
    m->arrived_ms       = 0;        /* v0.41: settle is timed from arrival */
    m->ichg_acc         = 0;
    m->ichg_acc_cnt     = 0;
    m->dwell_dipped     = false;
}

/* v0.41: remember the best-paying fence of this search — where a search
 * that ends on its runtime cap parks, instead of on whatever it was trying. */
static void note_best(mppt_ctx_t *m, int16_t avg)
{
    if (m->best_pwm == 0 || avg > m->best_ichg) {
        m->best_pwm  = m->cliff_pwm_min;
        m->best_ichg = avg;
    }
}

/*
 * knee_probe_step — lower the fence by `step` counts and remember where we
 * came from, so a probe that does not pay can be reverted exactly.
 *
 * Returns false when there is nothing left to probe (already at the floor):
 * the caller parks in HOLD, where the periodic re-probe will try again once
 * MPPT_KNEE_RETREAT_MS has passed and the knee is allowed to move.
 */
static bool knee_probe_step(system_ctx_t *ctx, uint8_t step)
{
    mppt_ctx_t *m  = &ctx->mppt;
    uint16_t  cur  = m->cliff_pwm_min;
    uint16_t  lo   = fence_floor(m);

    if (cur == 0 || cur <= lo)
        return false;

    int32_t next = (int32_t)cur - (int32_t)step;
    if (next < (int32_t)lo)
        next = (int32_t)lo;

    m->probe_from_pwm = cur;
    m->cliff_pwm_min  = (uint16_t)next;
    m->probe_step     = (uint8_t)(cur - (uint16_t)next);
    m->probing        = true;

    /* The loop is about to be granted current it was being refused; whatever
     * starvation dwell it accumulated against the old fence is spent. */
    ctx->charger.draw_blocked_ms = 0;
    return true;
}

/*
 * sp_floor_mv (v0.44) — the knee VOLTAGE floor: the averaged V_panel at which
 * the panel last collapsed or was measured not to pay, plus
 * MPPT_KNEE_VFLOOR_MV, minus whatever is currently lowered under test
 * (floor_probe_mv). 0 = no knee voltage known. See MPPT_KNEE_VFLOOR_MV.
 */
static uint16_t sp_floor_mv(const system_ctx_t *ctx)
{
    const mppt_ctx_t *m = &ctx->mppt;
    if (m->knee_vpanel_mv == 0)
        return 0;
    uint32_t f = (uint32_t)m->knee_vpanel_mv + MPPT_KNEE_VFLOOR_MV;
    if (f > m->floor_probe_mv) f -= m->floor_probe_mv; else f = 0;
    /* A knee cannot sit at open circuit. A sighting right under Voc (a
     * one-sample dip on an unloaded panel) would otherwise put the floor
     * above anything the panel can show and the loop would back off
     * forever; cap it one margin under the learned Voc. */
    if (m->panel_voc_mv >= PANEL_MIN_MV &&
        f + MPPT_KNEE_VFLOOR_MV > (uint32_t)m->panel_voc_mv)
        f = (uint32_t)m->panel_voc_mv - MPPT_KNEE_VFLOOR_MV;
    if (f < PANEL_MIN_MV)
        return 0;
    return (uint16_t)f;
}

/* The voltage-loop target that makes the backoff fire AT the floor: the loop
 * steps up once V_panel < setpoint - band, so the setpoint sits one band
 * above the floor. */
static uint32_t sp_floor_target_mv(const system_ctx_t *ctx)
{
    uint16_t floor = sp_floor_mv(ctx);
    if (floor == 0)
        return 0;
    return (uint32_t)floor + charger_vreg_deadband_mv(ctx);
}

/*
 * sp_follow — re-synchronise the voltage setpoint to where the plant is.
 *
 * The setpoint keeps exactly one job now: sag protection. Parked at the
 * settled operating voltage, cc_regulate's err < -band branch backs the draw
 * off when irradiance falls, as it always did — the fence is a floor on the
 * count, not a target, so backing off above it is free. Called on entry to
 * HOLD, once the fence has stopped moving, and every HOLD tick the reading
 * is in band.
 *
 * v0.44: never below the knee voltage floor. A gradual sag used to be
 * followed all the way down (the setpoint moved with the reading, so the
 * loop never saw an error); now the follow stops at the floor and the loop
 * backs off from there.
 */
static void sp_follow(system_ctx_t *ctx)
{
    mppt_ctx_t *m = &ctx->mppt;
    if (ctx->meas.panel_voltage >= PANEL_MIN_MV) {
        uint32_t sp = ctx->meas.panel_voltage;
        uint32_t f  = sp_floor_target_mv(ctx);
        if (sp < f) sp = f;
        m->vreg_setpoint_mv = (uint16_t)sp;
        m->prev_sp_mv       = m->vreg_setpoint_mv;
    }
}

/*
 * sp_track_target — the TRACKING setpoint.
 *
 * With no knee voltage known (a fresh panel), parked below anything the
 * panel will show, so cc_regulate's "above the band with headroom" branch
 * fires every interval and the fence is the only thing metering the descent.
 * Just above PANEL_SAFETY_MV, not at MPPT_SP_MIN_MV: on the 6.5 V array the
 * operating point IS ~6500 mV, so a setpoint there sits inside its own band
 * and the descent would stall.
 *
 * With a knee voltage known (v0.44), the floor target instead: the fence
 * still meters the descent, but the loop stops asking for more one band
 * above the last collapse voltage and backs off below it — the descent
 * cannot be driven into a knee the light has moved, and a sag during a
 * search is answered by the 400 ms loop rather than the droop guard.
 * Re-evaluated every TRACKING tick (the band and the floor both move).
 */
static void sp_track_target(system_ctx_t *ctx)
{
    uint32_t sp = (uint32_t)PANEL_SAFETY_MV + PANEL_VREG_DEADBAND_MIN_MV;
    uint32_t f  = sp_floor_target_mv(ctx);
    if (f > sp) sp = f;
    ctx->mppt.vreg_setpoint_mv = (uint16_t)sp;
    ctx->mppt.prev_sp_mv       = (uint16_t)sp;
}

/* v0.44: end a floor probe. Committing lowers the remembered knee voltage
 * by the amount that was under test (the dwell paid there); dropping it
 * puts the floor back where it was (the loop then backs off to it). */
static void floor_probe_commit(mppt_ctx_t *m)
{
    if (m->floor_probe_mv != 0) {
        if (m->knee_vpanel_mv > m->floor_probe_mv)
            m->knee_vpanel_mv -= m->floor_probe_mv;
        else
            m->knee_vpanel_mv = 0;
    }
    m->floor_probe_mv = 0;
    m->floor_probing  = false;
}
static void floor_probe_drop(mppt_ctx_t *m)
{
    m->floor_probe_mv = 0;
    m->floor_probing  = false;
}

/* =========================================================================
 * STATE ENTRY ACTIONS
 * ========================================================================= */

/* Shared TRACKING entry: fresh session counters + a baseline dwell at the
 * fence we are starting from. knee_pwm and cliff_pwm_min are deliberately
 * NOT reset here — they belong to the learned panel, not to the session, and
 * enter_tracking_fresh is the one place that clears them. */
static void tracking_common(system_ctx_t *ctx)
{
    mppt_ctx_t *m          = &ctx->mppt;
    m->state               = MPPT_TRACKING;
    m->prev_avg_valid      = false;   /* dwell 1 establishes the baseline */
    m->probing             = false;
    /* First probe size. Fine by default — every entry except a cold search
     * resumes from a fence that was already measured, so the answer is a
     * count or two away and a coarse first probe would drive straight past
     * the knee it is meant to re-confirm. enter_tracking_fresh overrides. */
    m->probe_step          = 1U;
    m->probe_from_pwm      = m->cliff_pwm_min;
    m->seen_dip_events     = ctx->charger.input_dip_events;
    m->seen_droop_events   = ctx->charger.droop_events;
    m->tracking_start_ms   = time_now();
    m->best_pwm            = 0;       /* v0.41: per-search best point */
    m->best_ichg           = 0;
    floor_probe_drop(m);              /* v0.44: a search starts at the floor */
    sp_track_target(ctx);
    start_dwell(ctx);
}

/*
 * Enter TRACKING from DISABLED on a FRESH activation — cold boot, a new
 * panel, a new day, or a gap past MPPT_RESEED_GAP_MS.
 *
 * Everything learned about the previous panel goes: its Voc estimate, its
 * knee, its fence, the counter-evidence bounding that fence, and the plant
 * gain that sizes the regulation band. The fence starts AT the current
 * count — the activation pre-position, which CHG_BUCK_SETTLE left at
 * roughly the zero-draw point — so the search begins from a draw of
 * ~nothing and walks down. That direction is the safe one by construction:
 * approaching the knee from the high-voltage side is the stable half of the
 * I-V curve, and every step is measured before the next is taken.
 *
 * There is no seed. That is the point of v0.40.
 */
static void enter_tracking_fresh(system_ctx_t *ctx)
{
    mppt_ctx_t *m = &ctx->mppt;

    m->panel_voc_mv        = 0;   /* relearn Voc from the running max      */
    m->knee_pwm            = 0;   /* ...and its knee                       */
    m->knee_learned_ms     = 0;
    m->sp_session_floor_mv = MPPT_SP_MIN_MV;
    m->last_good_vpanel_mv = 0;

    ctx->charger.delivering_pwm     = 0;  /* a different panel's delivering
                                           * count says nothing about this one */
    ctx->charger.draw_blocked_ms    = 0;
    ctx->charger.plant_mv_per_count = 0;  /* ...and its gain: the band goes
                                           * back to the static fallback until
                                           * the voltage loop measures this
                                           * panel (hw_config.h)               */
    ctx->charger.vloop_measure_armed = false;

    /* Start the fence where the plant already is. A fence of 0 means "no
     * ceiling", which would let the voltage loop run all the way to the
     * knee in one unmeasured descent — exactly the behaviour being removed. */
    m->cliff_pwm_min = ctx->pwm;

    tracking_common(ctx);

    /* A cold search starts tens of counts from any knee: open coarse. The
     * step refines itself the moment a probe pays less than
     * MPPT_KNEE_COARSE_MA. */
    m->probe_step = (uint8_t)PANEL_VREG_STEP_MAX;
}

/*
 * Enter TRACKING from HOLD (periodic down-probe) or from DISABLED after an
 * input-loss bounce. Keep the fence, the knee and the Voc estimate: the
 * panel has not changed, and re-deriving a knee that is already measured is
 * what made a 2 s stand-down cost 12 s of harvest (bench 10.09.26).
 *
 * Always fine-stepped: we resume from a converged fence, so the answer is a
 * count or two away. The coarse step re-arms by itself the moment a probe
 * pays more than MPPT_KNEE_COARSE_MA, which is what a genuine irradiance
 * increase looks like.
 */
static void enter_tracking_reprobe(system_ctx_t *ctx)
{
    ctx->mppt.voc_pending = false;
    tracking_common(ctx);
}

/*
 * Enter HOLD — the fence is where the panel says it should be. Re-point the
 * setpoint at the plant on the way in, so the voltage loop resumes its sag
 * duty for the length of the hold.
 */
static void enter_hold(system_ctx_t *ctx)
{
    ctx->mppt.state         = MPPT_HOLD;
    ctx->mppt.hold_start_ms = time_now();
    ctx->mppt.knee_probe_ms = time_now();
    ctx->mppt.probing       = false;
    floor_probe_drop(&ctx->mppt);     /* v0.44: unmeasured = not accepted */
    sp_follow(ctx);
}

/*
 * Enter DISABLED — charger region went inactive. The fence, the knee and
 * the Voc estimate are PRESERVED: a brief EM bounce must not forget the
 * measured operating point, and energy_mode's warm resume enters AT
 * cliff_pwm_min, which is why a bounce now costs the re-arm block and
 * almost nothing else.
 */
static void enter_disabled(system_ctx_t *ctx)
{
    ctx->mppt.state             = MPPT_DISABLED;
    ctx->mppt.disabled_since_ms = time_now();
    ctx->mppt.seed_invalidated  = false;
    ctx->mppt.probing           = false;
}

/* =========================================================================
 * TRACKING TICK — dwell sequencing + the P&O decision
 * ========================================================================= */

static void tracking_tick(system_ctx_t *ctx)
{
    mppt_ctx_t *m   = &ctx->mppt;
    uint32_t now    = time_now();
    uint32_t dwelt  = now - m->dwell_start_ms;

    /* ── Collapsed reading: never measure it ──
     * panel_safety_backoff and the foreground guards own the recovery; the
     * knee itself is learned from the event counters in mppt_update, which
     * see it with a PWM attached. All this branch has to do is refuse to
     * treat the wreckage as fitness. */
    if (ctx->meas.panel_voltage < PANEL_SAFETY_MV) {
        m->prev_avg_valid = false;
        start_dwell(ctx);
        return;
    }

    /* ── Battery-limited: nothing to optimise ── (v0.41)
     * If the intake clamp, not the panel, bounds the current (precharge
     * trickle, CV taper, a near-full cell), a fence probe cannot observe
     * anything and a comparison made here would be a comparison of the
     * battery limit with itself. Park; HOLD's re-probe is gated on
     * panel_limited too and comes back when the panel is the constraint. */
    if (!ctx->panel_limited) {
        enter_hold(ctx);
        return;
    }

    /* ── Phase 0: settle ──
     *
     * "Settled" means the regulator has ARRIVED at the fence — the count
     * sitting EXACTLY on it, since v0.41 made the fence two-sided — and has
     * stayed there for MPPT_KNEE_SETTLE_MS, the 64-sample window plus
     * margin. The clock starts at arrival, not at the request: a rejected
     * probe's rollback takes the regulator an interval or two to realise,
     * and a window that straddles two counts measures neither (v0.40 timed
     * the settle from the request and could start measuring the instant
     * the count landed, with the old operating point still in the
     * average). MPPT_SP_SETTLE_MAX_MS caps the wait for a fence that is
     * never reached at all (a stiff source), which then measures a
     * not-quite-arrived plant exactly as every dwell used to. */
    if (m->dwell_phase == 0) {
        bool on_fence = (m->cliff_pwm_min != 0) &&
                        (ctx->pwm == m->cliff_pwm_min);
        if (!on_fence)
            m->arrived_ms = 0;
        else if (m->arrived_ms == 0)
            m->arrived_ms = (now != 0) ? now : 1U;

        bool settled = (m->arrived_ms != 0) &&
                       ((now - m->arrived_ms) >= MPPT_KNEE_SETTLE_MS);
        if (settled || dwelt >= MPPT_SP_SETTLE_MAX_MS) {
            m->dwell_phase      = 1;
            m->measure_start_ms = now;
            return;
        }

        /* ── v0.44: not arriving. Is the voltage floor what holds the loop? ──
         *
         * The loop stops asking for more once V_panel is inside the band
         * around the floor target, so a fence below that voltage is never
         * reached. Two cases:
         *   - the loop has been backing off (a sag step within
         *     MPPT_FLOOR_PROBE_QUIET_MS): the light is falling; the floor is
         *     right and the fence is stale. Fence follows the plant up, park.
         *   - the loop is quiet at the floor: the floor may be stale-high
         *     (learned in brighter light). Lower it one MPPT_KNEE_VFLOOR_MV
         *     under test and give the loop another settle; the measurement
         *     at the fence decides whether the lowered floor stays. */
        if (!on_fence && dwelt >= MPPT_KNEE_SETTLE_MS &&
            ctx->pwm > m->cliff_pwm_min) {
            uint16_t floor = sp_floor_mv(ctx);
            int32_t  band  = (int32_t)charger_vreg_deadband_mv(ctx);
            bool voltage_limited = (floor != 0) &&
                ((int32_t)ctx->meas.panel_voltage <=
                     (int32_t)m->vreg_setpoint_mv + band);
            if (voltage_limited) {
                /* "Quiet": no sag step lately — or the loop has already
                 * backed off to zero delivery, where a step changes nothing
                 * and waiting for it to stop would wait forever. */
                bool quiet = ((now - ctx->charger.last_backoff_ms) >=
                                  MPPT_FLOOR_PROBE_QUIET_MS) ||
                             (ctx->charger.zero_draw_pwm != 0 &&
                              ctx->pwm >= ctx->charger.zero_draw_pwm);
                bool room  = (uint32_t)floor >=
                             (uint32_t)PANEL_MIN_MV + MPPT_KNEE_VFLOOR_MV;
                if (quiet && room) {
                    m->floor_probe_mv += MPPT_KNEE_VFLOOR_MV;
                    m->floor_probing   = true;
                    sp_track_target(ctx);
                    m->dwell_start_ms  = now;     /* fresh settle from here */
                } else {
                    m->cliff_pwm_min = ctx->pwm;  /* the floor is the fence */
                    m->probing       = false;
                    enter_hold(ctx);              /* drops the probe */
                }
            }
        }
        return;
    }

    /* ── Phase 1: measure ──
     * A count that moves under the measurement (a clamp, a guard, a fence
     * change) contaminates it: the window then describes two operating
     * points. Start the dwell over rather than judge it. Only for a dwell
     * that had actually arrived — the capped never-arrived case measures
     * what it can, as before. */
    if (m->arrived_ms != 0 && ctx->pwm != m->cliff_pwm_min) {
        start_dwell(ctx);
        return;
    }

    m->ichg_acc += (int32_t)ctx->meas.chg_current;
    m->ichg_acc_cnt++;

    if ((now - m->measure_start_ms) < MPPT_KNEE_MEASURE_MS)
        return;

    int16_t avg = (int16_t)(m->ichg_acc / (int32_t)m->ichg_acc_cnt);

    /* ── Acquisition: no fitness signal exists yet ──
     *
     * CHG_BUCK_SETTLE hands over with the rail pre-positioned just above
     * V_bat, which on the 14.09 array is 5-15 counts ABOVE the zero-draw
     * count. Every count in that stretch delivers the same zero, so a P&O
     * comparison there measures delta ~0, concludes "this count did not pay",
     * and fences the charger off a panel it has never drawn from. (That is
     * not hypothetical — it is what this function did when it was first
     * written, and it is the same class of bug as v0.39's fence-above-
     * zero-draw: a decision made where the signal does not exist.)
     *
     * So while the draw is under CHG_DELIVERING_MIN_MA, descend on a fixed
     * step and judge nothing. Gated on knee_pwm == 0 so this is strictly a
     * cold-start behaviour: once a knee is known, a low reading means the
     * panel dipped, not that we have not started, and the ordinary path
     * (which cannot probe past the floor) handles it. */
    if (avg < (int16_t)CHG_DELIVERING_MIN_MA && m->knee_pwm == 0) {
        m->prev_avg_ichg  = avg;
        m->prev_avg_valid = true;
        floor_probe_commit(m);
        if (!knee_probe_step(ctx, MPPT_KNEE_ACQUIRE_STEP)) {
            enter_hold(ctx);
            return;
        }
        start_dwell(ctx);
        return;
    }

    /* ── Baseline dwell: record fitness here, then take the first probe ──
     * The first probe of a session is coarse: we start from the zero-draw
     * pre-position on a fresh entry (tens of counts from any knee), and from
     * a converged fence on a re-probe, where the first coarse probe either
     * pays — meaning conditions really did improve — or reverts and
     * brackets, costing one extra dwell. */
    if (!m->prev_avg_valid) {
        uint8_t step = (m->probe_step != 0) ? m->probe_step : 1U;
        m->prev_avg_ichg  = avg;
        m->prev_avg_valid = true;
        note_best(m, avg);
        floor_probe_commit(m);           /* v0.44: measured, nothing fell */
        if (!knee_probe_step(ctx, step)) {
            enter_hold(ctx);       /* nothing left to probe: already fenced */
            return;
        }
        start_dwell(ctx);
        return;
    }

    /* ── Comparison dwell: did the extra current we asked for arrive? ── */
    int32_t delta = (int32_t)avg - (int32_t)m->prev_avg_ichg;

    if (delta > (int32_t)MPPT_KNEE_MIN_DELTA_MA) {
        /* It paid. Accept this fence and keep descending — coarse while
         * there is clearly distance left, fine once the payoff is within a
         * couple of counts of the gate. */
        m->prev_avg_ichg = avg;
        note_best(m, avg);
        floor_probe_commit(m);           /* v0.44: it paid — the floor was high */
        uint8_t step = (delta > (int32_t)MPPT_KNEE_COARSE_MA)
                         ? (uint8_t)PANEL_VREG_STEP_MAX : 1U;
        if (!knee_probe_step(ctx, step)) {
            enter_hold(ctx);
            return;
        }
        start_dwell(ctx);
        return;
    }

    /* It did not pay. */
    if (m->probe_step > 1U) {
        /* A COARSE probe that failed says the knee is somewhere inside the
         * jump, not that this count is the knee — fencing here would park us
         * up to PANEL_VREG_STEP_MAX counts short of the optimum, which is
         * the v0.39 failure written a different way. Revert to the last count
         * that paid and bracket it one at a time. The baseline is dropped so
         * the good point is RE-measured under current conditions: irradiance
         * may have moved between the two dwells, and comparing against a
         * stale number is how a tracker converges on weather. */
        m->cliff_pwm_min  = m->probe_from_pwm;
        m->probe_step     = 0;
        m->probing        = false;
        m->prev_avg_valid = false;
        floor_probe_drop(m);             /* v0.44: not paid — floor stays */
        sp_track_target(ctx);
        start_dwell(ctx);
        return;
    }

    /* A single count that did not pay IS the knee — measured, at no cost,
     * with the panel still up. Fence one margin above it and hold.
     *
     * Only when there was something to measure: two dwells that both
     * delivered under CHG_DELIVERING_MIN_MA compare noise against noise, and
     * a knee learned from that would fence a working panel out of reach for
     * MPPT_KNEE_RETREAT_MS. Park without learning instead — HOLD's re-probe
     * comes back in MPPT_KNEE_PROBE_MS and tries again against whatever the
     * panel is doing then. */
    if (avg >= (int16_t)CHG_DELIVERING_MIN_MA ||
        m->prev_avg_ichg >= (int16_t)CHG_DELIVERING_MIN_MA)
        knee_learn(ctx, (uint32_t)m->cliff_pwm_min, MPPT_KNEE_MARGIN,
                   ctx->meas.panel_voltage);
    enter_hold(ctx);
}

/* =========================================================================
 * PUBLIC: step 6 entry point (knee P&O)
 * ========================================================================= */

void mppt_update(system_ctx_t *ctx)
{
    mppt_ctx_t *m = &ctx->mppt;
    const bool has_sun  = ctx->flag_has_sun.value;
    const bool charging = (ctx->charger.state == CHG_PRECHARGE) ||
                          (ctx->charger.state == CHG_CC) ||
                          (ctx->charger.state == CHG_CV);

    /* Nothing to optimise until Q49 is connected. CHG_BUCK_SETTLE runs the
     * buck intentionally unloaded and owns PWM acquisition; starting a dwell
     * there would measure zero charge current and pollute its baseline. */
    if (!charging) {
        if (m->state != MPPT_DISABLED) {
            /* Why did the region go down? energy_mode (step 5, this same
             * tick) stamps rearm_block_ms only for an input-loss stand-down,
             * so a block still in the future means the panel fell over under
             * us: learn the count before forgetting the session. Any other
             * exit (CV, fault, sun lost) teaches nothing about the knee. */
            if ((int32_t)(ctx->charger.rearm_block_ms - time_now()) > 0)
                m->event_knee_accepted = knee_learn_event(ctx,
                        (uint32_t)ctx->charger.input_trace_pwm_from,
                        ctx->charger.input_trace_ichg_from,
                        ctx->charger.input_trace_vpanel_from,
                        ctx->charger.input_trace_settled);
            enter_disabled(ctx);
        }

        /* While the region is down, decide whether what we learned survives
         * the gap. Both signals mean "this may not be the same panel, or not
         * the same day": has_sun cleared (what a genuine disconnect trips in
         * 1.5 s, and what an input-loss bounce never manages), or the gap ran
         * past MPPT_RESEED_GAP_MS. Latched; the DISABLED entry path consumes
         * it. */
        if (!has_sun ||
            (time_now() - m->disabled_since_ms) >= MPPT_RESEED_GAP_MS)
            m->seed_invalidated = true;

        return;
    }

    /* Voc estimate: running max of the filtered panel voltage while the
     * region is active. Diagnostic now rather than structural — nothing
     * seeds off it any more — but it is still the credibility test that
     * routes an activation to the fresh path, and it still costs nothing:
     * V_panel never exceeds open circuit and touches near-OC repeatedly. */
    if (m->state != MPPT_DISABLED &&
        ctx->meas.panel_voltage > m->panel_voc_mv)
        m->panel_voc_mv = ctx->meas.panel_voltage;

    /* ── Knee sightings that cost nothing ──
     *
     * A droop the fast guard arrested (v0.40) and a collapse the input guard
     * rescued (v0.32) are both "this count did not hold", with the count
     * snapshotted before any backoff moved it. They are the cheapest knee
     * measurements available — no teardown, no re-acquisition — and the
     * droop path in particular sees the event ~10 ms in, at a PWM that is
     * still exactly the one that failed.
     *
     * Both use CHG_CLIFF_PWM_MARGIN rather than MPPT_KNEE_MARGIN: a probe
     * that merely underdelivered is a measurement, but a fall is an event
     * whose snapshot is at best one conversion old. */
    if (m->state != MPPT_DISABLED &&
        ctx->charger.droop_events != m->seen_droop_events) {
        m->seen_droop_events = ctx->charger.droop_events;
        m->event_knee_accepted = knee_learn_event(ctx,
                (uint32_t)ctx->charger.droop_pwm_from,
                ctx->charger.droop_ichg_from,
                ctx->charger.droop_vpanel_from,
                ctx->charger.droop_settled);
        floor_probe_drop(m);             /* v0.44: the panel just answered */
        if (m->state == MPPT_TRACKING) {
            m->prev_avg_valid = false;   /* samples spanning a droop are noise */
            start_dwell(ctx);
        }
    }
    if (m->state != MPPT_DISABLED &&
        ctx->charger.input_dip_events != m->seen_dip_events) {
        m->seen_dip_events = ctx->charger.input_dip_events;
        m->event_knee_accepted = knee_learn_event(ctx,
                (uint32_t)ctx->charger.input_trace_pwm_from,
                ctx->charger.input_trace_ichg_from,
                ctx->charger.input_trace_vpanel_from,
                ctx->charger.input_trace_settled);
        floor_probe_drop(m);
        if (m->state == MPPT_TRACKING) {
            m->prev_avg_valid = false;
            start_dwell(ctx);
        }
    }

    /* ── A fence the regulator sits on delivering nothing is wrong ── (v0.41)
     *
     * The fence is a ceiling on current. If the loop has been pinned exactly
     * on it for a full settled window and the cell is receiving under
     * CHG_DELIVERING_MIN_MA while the panel is the binding constraint, the
     * fence is at or above the zero-delivery count: it is not bounding the
     * draw, it is forbidding it (v0.39's failure, reproduced in v0.40 by the
     * droop guard ratcheting to the zero-draw count — 14.09.26, fence 110 on
     * a 110 zero-draw, 0x0100 on every attempt to sit there). Only a fence
     * that a KNEE is holding up can be wrong this way; during a cold
     * acquisition the fence is the actuator and sits at zero delivery on
     * purpose (knee_pwm == 0). Forget the knee and re-acquire: the next
     * baseline measures under CHG_DELIVERING_MIN_MA with no knee and takes
     * the acquisition step. */
    if (m->state != MPPT_DISABLED && m->knee_pwm != 0 &&
        m->cliff_pwm_min != 0 && ctx->pwm == m->cliff_pwm_min &&
        ctx->charger.settled_ticks >= CHG_PWM_SETTLED_TICKS &&
        ctx->meas.chg_current < (int16_t)CHG_DELIVERING_MIN_MA &&
        ctx->panel_limited &&
        (ctx->charger.state == CHG_CC || ctx->charger.state == CHG_PRECHARGE)) {
        m->knee_pwm        = 0;
        m->knee_learned_ms = 0;
        if (m->fence_dropped < UINT16_MAX) m->fence_dropped++;
        if (m->state == MPPT_HOLD) {
            m->knee_probe_ms = time_now();
            enter_tracking_reprobe(ctx);
        } else {
            m->prev_avg_valid = false;
            start_dwell(ctx);
        }
    }

    /* Last clean averaged V_panel — the reading a post-mortem would want,
     * and the one sp_follow parks the setpoint on. A collapsed 10 ms sample
     * costs the 64-deep average ~125-190 mV; honest regulation near the knee
     * moves it ~50 mV per tick. Reject the sharp falls, keep the rest. */
    if (m->state != MPPT_DISABLED &&
        ctx->meas.panel_voltage >= PANEL_SAFETY_MV &&
        ((int32_t)ctx->meas.panel_voltage + (int32_t)MPPT_SP_VPANEL_DROP_MV
            >= (int32_t)m->last_good_vpanel_mv))
        m->last_good_vpanel_mv = ctx->meas.panel_voltage;

    switch (m->state) {

    case MPPT_DISABLED:
        /* Charger just activated. FRESH (cold boot, new panel, long gap):
         * throw the learned panel away and search from the zero-draw
         * pre-position. RE-PROBE (input-loss bounce, or any short gap): the
         * measured fence is still good — resume at it and re-bracket.
         *
         * The panel_voc_mv guard stops a stale seed_invalidated from routing
         * a boot with no credible reading into the re-probe path, where it
         * would resume against a fence it never measured. */
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
        /* P2: charger reached CV → freeze. cv_regulate regulates V_bat and
         * ignores both the fence and the setpoint, so a probe would measure
         * the battery's taper, not the panel. HOLD keeps the fence for the
         * CC return. */
        if (ctx->charger.state == CHG_CV) {
            enter_hold(ctx);
            break;
        }
        /* P3: search cap → park at the best fence found so far. v0.40 said
         * that and parked on the probe under test; v0.41 actually restores
         * the best-paying measured fence (bounded by the knee floor), and
         * the two-sided fence rule in cc_regulate realises it. */
        if ((time_now() - m->tracking_start_ms) >= MPPT_KNEE_SEARCH_MS) {
            if (m->best_pwm != 0 && m->best_pwm >= fence_floor(m))
                m->cliff_pwm_min = m->best_pwm;
            m->probing = false;
            enter_hold(ctx);
            break;
        }
        /* P4: run the dwell/decision machinery (v0.44: with the setpoint
         * re-derived from the floor and the live band first). */
        sp_track_target(ctx);
        tracking_tick(ctx);
        break;

    case MPPT_HOLD:
        /* P1: sun lost → DISABLED. */
        if (!has_sun) {
            enter_disabled(ctx);
            break;
        }

        /* P2: keep the setpoint on the plant. The fence is frozen, but the
         * operating VOLTAGE drifts with irradiance and cell temperature, and
         * a setpoint left behind by that drift either stops protecting
         * against a sag (too low) or commands a pointless backoff (too
         * high). Only ever follows a reading that is already inside the
         * band, so it can never chase the plant off its own knee — a real
         * sag leaves the band, and the loop backs off instead. */
        if (ctx->meas.panel_voltage >= PANEL_MIN_MV &&
            (uint32_t)abs_diff_i32((int32_t)ctx->meas.panel_voltage,
                                   (int32_t)m->vreg_setpoint_mv)
              <= (uint32_t)charger_vreg_deadband_mv(ctx))
            sp_follow(ctx);

        /* P3: the periodic down-probe. This is the ONLY thing that loosens
         * the fence, and it replaces the two blind timers of v0.39 —
         * CHG_CLIFF_PROBE_MS (released a count every 2 s while the loop was
         * starved, which on 14.09 walked 79 -> 78 -> 77 straight back into
         * the cliff) and CHG_CLIFF_PWM_RELAX_MS (one count per 180 s, far
         * too slow to follow the sun, and the reason the session's last five
         * minutes ran fenced at 95).
         *
         * Gated on the panel actually being the binding constraint: if the
         * battery bounds the current (CV taper, precharge trickle, a
         * near-full cell) a probe cannot observe anything. Gated too on
         * MPPT_KNEE_RETREAT_MS since the knee was last proven, unless the
         * fence has since been pushed above it by an event — in which case
         * there is slack to recover and no need to wait. */
        /* v0.42: a rail sitting ABOVE the fence in HOLD was kicked there — a
         * droop backoff, a rescue restore, a sag step — and nothing in HOLD
         * brings it back: the setpoint follows the plant, so the voltage
         * loop sees no error, and the 30 s probe timer (or the 120 s knee
         * retreat) was the only way down. Bench 14.09.26 v0.41: 100 s at
         * pwm 110 / 146 mA with the fence at 100 / 460 mA. Re-probe at once
         * instead: TRACKING parks the setpoint low, the loop walks back to
         * the fence and re-measures it, and HOLD resumes ~3 s later. Only
         * once the guard's own event is over (droop_count == 0). */
        bool kicked = (m->cliff_pwm_min != 0) &&
                      (ctx->pwm > m->cliff_pwm_min) &&
                      (ctx->charger.droop_count == 0);
        /* v0.44: ...and only if there is voltage room to descend into. A rail
         * held above the fence BY THE FLOOR is where it should be. */
        {
            uint32_t ft = sp_floor_target_mv(ctx);
            if (ft != 0 &&
                (uint32_t)ctx->meas.panel_voltage <=
                    ft + charger_vreg_deadband_mv(ctx))
                kicked = false;
        }

        /* v0.43: a knee is evidence about the LIGHT it was learned in. If the
         * panel at the present count now reads well above what that count
         * would have read then (the count difference converted through the
         * measured gain), the light has come back and the knee is stale —
         * release it now, not in MPPT_KNEE_RETREAT_MS. See
         * MPPT_KNEE_RELEASE_MV for the 120 s at 1.2 W that motivated it. */
        bool headroom = false;
        if (m->knee_pwm != 0 && m->knee_vpanel_mv != 0 &&
            ctx->pwm >= m->knee_pwm) {
            uint32_t expect = (uint32_t)m->knee_vpanel_mv +
                              (uint32_t)ctx->charger.plant_mv_per_count *
                                  (uint32_t)(ctx->pwm - m->knee_pwm);
            headroom = (uint32_t)ctx->meas.panel_voltage >
                       expect + MPPT_KNEE_RELEASE_MV;
        }
        bool retreat_over = (m->knee_pwm != 0) &&
                            ((time_now() - m->knee_learned_ms) >=
                                 MPPT_KNEE_RETREAT_MS || headroom);

        /* v0.44: a rail held ABOVE the fence by the voltage floor is worth a
         * periodic look too — the floor may have been learned in brighter
         * light (tracking_tick's floor probe, quiet-gated, decides). */
        bool floor_bound = (m->cliff_pwm_min != 0) &&
                           (ctx->pwm > m->cliff_pwm_min) &&
                           (sp_floor_mv(ctx) != 0);

        if ((kicked || retreat_over ||
             (time_now() - m->knee_probe_ms) >= MPPT_KNEE_PROBE_MS) &&
            ctx->panel_limited &&
            (ctx->charger.state == CHG_PRECHARGE ||
             ctx->charger.state == CHG_CC) &&
            (kicked || retreat_over || floor_bound || m->knee_pwm == 0 ||
             m->cliff_pwm_min > (uint16_t)(m->knee_pwm + MPPT_KNEE_MARGIN))) {

            /* A knee older than MPPT_KNEE_RETREAT_MS, or one the light has
             * clearly moved past, is no longer evidence. Release it so the
             * probe can go past it. */
            if (retreat_over)
                m->knee_pwm = 0;

            m->knee_probe_ms = time_now();
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
