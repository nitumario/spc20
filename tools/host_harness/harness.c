/*
 * v0.41 behavioural harness: drives the real mppt.c + charger.c against a
 * simple plant model. Not an electrical simulation — a control-logic check
 * of the invariants the review asked for.
 *
 * Plant: I_buck(pwm) = (ZERO - pwm) * 45 mA (negative past ZERO = reverse),
 *        V_panel(pwm) = VOC - (ZERO - pwm) * GAIN_MV for pwm < ZERO.
 *        Averages are a 64-sample FIFO of the raw 10 ms conversions.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "stubs.h"
#include "system_types.h"
#include "charger.h"
#include "mppt.h"

static int ZERO = 110, GAIN_MV = 20, VOC = 12930, KNEE = 0;   /* KNEE 0 = no cliff */
static int16_t  fifo_i[64]; static uint16_t fifo_v[64]; static int fifo_n = 0, fifo_h = 0;
static int fails = 0, checks = 0;
static double noise_sigma = 0.0;
static unsigned rng = 12345;
static double gauss(void) { double u1 = (rng = rng * 1103515245u + 12345u, ((rng >> 8) & 0xffff) + 1) / 65537.0;
                            double u2 = (rng = rng * 1103515245u + 12345u, ((rng >> 8) & 0xffff) + 1) / 65537.0;
                            return sqrt(-2 * log(u1)) * cos(6.283185307 * u2); }

#define CHECK(cond, ...) do { checks++; if (!(cond)) { fails++; printf("  FAIL: " __VA_ARGS__); printf("\n"); } else { printf("  ok:   " __VA_ARGS__); printf("\n"); } } while (0)

static int  model_i(int pwm)   { return (ZERO - pwm) * 45; }
static int  model_v(int pwm)   { int v = VOC - (ZERO - pwm) * GAIN_MV; if (KNEE && pwm < KNEE) v = 3500; if (v > VOC) v = VOC; return v; }

/* one 10 ms conversion: sample the plant at the HARDWARE pwm, push into FIFOs, refresh the averaged ctx->meas */
static int force_raw_v = 0;     /* >0: override the next raw V_panel conversion (droop injection) */
static int force_raw_i = 0;     /* nonzero: override raw I_buck                                  */
static int force_raw_i_once = 0;/* nonzero: override raw I_buck for ONE conversion                */
static void conversion(system_ctx_t *c)
{
    int pwm = h_hw_pwm;
    int iv  = h_q49 ? model_i(pwm) : 0;
    int vv  = model_v(pwm);
    if (force_raw_v) { vv = force_raw_v; force_raw_v = 0; }
    if (vv < 3320 + 500 && h_q49) iv = 150;   /* dropout: the buck is a pass-through, current is forward (bench: every rescue trace) */
    if (force_raw_i) { iv = force_raw_i; }
    if (force_raw_i_once) { iv = force_raw_i_once; force_raw_i_once = 0; }
    if (noise_sigma > 0) iv += (int)(gauss() * noise_sigma);
    h_raw_ibuck  = (int16_t)iv;
    h_raw_vpanel = (uint16_t)vv;
    h_raw_vchg   = (uint16_t)(3320 + (ZERO - pwm) * 3);
    fifo_i[fifo_h] = (int16_t)iv; fifo_v[fifo_h] = (uint16_t)vv; fifo_h = (fifo_h + 1) % 64; if (fifo_n < 64) fifo_n++;
    for (unsigned i = 0; i < ADC_PANEL_TRACE_DEPTH; i++) h_trace[i] = fifo_v[(fifo_h + 64 - ADC_PANEL_TRACE_DEPTH + i) % 64];
    long si = 0, sv = 0; for (int i = 0; i < fifo_n; i++) { si += fifo_i[i]; sv += fifo_v[i]; }
    c->meas.chg_current   = (int16_t)(si / fifo_n);
    c->meas.panel_voltage = (uint16_t)(sv / fifo_n);
    c->meas.bat_voltage   = 3320;
    c->meas.dsg_current   = 0;
    c->panel_limited      = c->meas.chg_current < (int16_t)(c->allowed_chg - PANEL_LIMITED_MARGIN_MA);
    h_seq++;
}

/* advance 10 ms: conversion, then the foreground guards (as main.c orders them) */
static void step10(system_ctx_t *c)
{
    h_now += 10;
    conversion(c);
    charger_panel_droop_guard(c);
    charger_input_guard(c);
    charger_fast_guard(c);
    if (h_now % 50 == 0) {           /* the 50 ms pipeline tick */
        mppt_update(c);
        charger_update(c);
        h_hw_pwm = c->pwm;           /* apply_pwm */
    }
}
static void run_ms(system_ctx_t *c, int ms) { for (int i = 0; i < ms / 10; i++) step10(c); }

static system_ctx_t fresh_cc(int pwm, int fence)
{
    system_ctx_t c; memset(&c, 0, sizeof c);
    fifo_n = fifo_h = 0; h_seq = 0; h_now = 1000; h_faults_raised = 0; noise_sigma = 0; force_raw_i = 0; force_raw_v = 0; force_raw_i_once = 0;
    c.energy_mode = EM_CHARGE_ONLY; c.charger.state = CHG_CC; c.flag_has_sun.value = true;
    c.allowed_chg = 2000; c.i_buck_max = 2000;
    c.pwm = (uint16_t)pwm; h_hw_pwm = (uint16_t)pwm; h_q49 = true; h_buck_en = true;
    c.charger.connect_ms = 0; c.charger.plant_mv_per_count = (uint16_t)GAIN_MV;
    c.charger.cc_last_downstep_ms = h_now;
    c.mppt.state = MPPT_DISABLED; c.mppt.seed_invalidated = true;
    c.mppt.cliff_pwm_min = (uint16_t)fence;
    c.mppt.vreg_setpoint_mv = 4860;
    for (int i = 0; i < 70; i++) conversion(&c);     /* fill the averages at the start point */
    return c;
}

/* ------------------------------------------------------------------ tests */
static void t1_fine_reject_rolls_back(void)
{
    printf("T1 fine-probe rejection is realised physically\n");
    system_ctx_t c = fresh_cc(79, 79);
    c.mppt.state = MPPT_TRACKING; c.mppt.knee_pwm = 0; c.mppt.probing = true; c.mppt.probe_step = 1;
    c.mppt.probe_from_pwm = 79; c.mppt.cliff_pwm_min = 78; c.pwm = 78; h_hw_pwm = 78;
    c.mppt.prev_avg_valid = true; c.mppt.prev_avg_ichg = 1300;    /* baseline said 79 paid 1300 */
    c.mppt.dwell_phase = 1; c.mppt.arrived_ms = h_now - 1000; c.mppt.measure_start_ms = h_now - 500; c.mppt.dwell_start_ms = h_now - 2000;
    c.mppt.tracking_start_ms = h_now - 5000;
    for (int i = 0; i < 70; i++) conversion(&c);
    c.mppt.ichg_acc = 1290 * 5; c.mppt.ichg_acc_cnt = 5;        /* 78 measured 1290: not better */
    h_now += 10; conversion(&c); mppt_update(&c);
    CHECK(c.mppt.state == MPPT_HOLD && c.mppt.cliff_pwm_min == 79 && c.mppt.knee_pwm == 78, "HOLD, fence 79, knee 78 (fence=%u knee=%u state=%d)", c.mppt.cliff_pwm_min, c.mppt.knee_pwm, c.mppt.state);
    CHECK(c.pwm == 78, "pwm still 78 on the decision tick (%u)", c.pwm);
    run_ms(&c, 1000);
    CHECK(c.pwm == 79 && h_hw_pwm == 79, "regulator walked the rail back UP to the fence within 1 s (pwm=%u)", c.pwm);
}

static void t2_coarse_reject_and_arrival_settle(void)
{
    printf("T2 coarse-probe rejection reverts, and the baseline waits for ARRIVAL + settle\n");
    system_ctx_t c = fresh_cc(76, 76);
    c.mppt.state = MPPT_TRACKING; c.mppt.probing = true; c.mppt.probe_step = 3; c.mppt.probe_from_pwm = 79;
    c.mppt.prev_avg_valid = true; c.mppt.prev_avg_ichg = 1400;
    c.mppt.dwell_phase = 1; c.mppt.arrived_ms = h_now - 1000; c.mppt.measure_start_ms = h_now - 500; c.mppt.dwell_start_ms = h_now - 2000;
    c.mppt.tracking_start_ms = h_now - 5000;
    c.mppt.ichg_acc = 1390 * 5; c.mppt.ichg_acc_cnt = 5;
    h_now += 10; conversion(&c); mppt_update(&c);
    CHECK(c.mppt.state == MPPT_TRACKING && c.mppt.cliff_pwm_min == 79 && c.mppt.dwell_phase == 0, "reverted to 79, new baseline dwell (fence=%u phase=%u)", c.mppt.cliff_pwm_min, c.mppt.dwell_phase);
    uint32_t t0 = h_now;
    while (c.pwm != 79 && h_now - t0 < 3000) step10(&c);
    uint32_t t_arrive = h_now;
    CHECK(c.pwm == 79, "pwm walked 76 -> 79 in %u ms", (unsigned)(t_arrive - t0));
    CHECK(c.mppt.dwell_phase == 0, "not measuring at arrival");
    while (c.mppt.dwell_phase == 0 && h_now - t_arrive < 3000) step10(&c);
    CHECK(c.mppt.dwell_phase == 1 && (h_now - t_arrive) >= MPPT_KNEE_SETTLE_MS - 50, "measure began %u ms after arrival (settle %lu)", (unsigned)(h_now - t_arrive), (unsigned long)MPPT_KNEE_SETTLE_MS);
}

static void t3_switching_glitch_is_not_a_droop(void)
{
    printf("T3 the buck's own transient (raw I spike + 1-sample V dip): no backoff, no knee, traced GLITCH\n");
    system_ctx_t c = fresh_cc(106, 106);                  /* on the fence, 180 mA */
    c.mppt.state = MPPT_HOLD; c.mppt.knee_pwm = 105; c.mppt.panel_voc_mv = 12930; c.mppt.vreg_setpoint_mv = c.meas.panel_voltage;
    c.mppt.knee_probe_ms = h_now; c.mppt.hold_start_ms = h_now; c.mppt.knee_learned_ms = h_now;
    c.charger.panel_op_mv = c.meas.panel_voltage; c.charger.panel_op_ms = h_now;
    run_ms(&c, 1000);
    CHECK(c.charger.zero_draw_pwm == 110, "zero-draw learned as 110 from a settled delivering tick (zd=%u)", c.charger.zero_draw_pwm);
    uint16_t pwm0 = c.pwm, fence0 = c.mppt.cliff_pwm_min, knee0 = c.mppt.knee_pwm;
    force_raw_v = 10800; force_raw_i_once = c.meas.chg_current + 1500; step10(&c);
    CHECK(c.charger.droop_glitch_events == 1 && c.charger.droop_events == 0, "classified GLITCH (glt=%u drp=%u)", c.charger.droop_glitch_events, c.charger.droop_events);
    CHECK(c.pwm == pwm0, "no backoff (pwm %u)", c.pwm);
    CHECK(c.charger.droop_kind == DROOP_KIND_GLITCH && c.charger.droop_trace_pending, "traced as GLITCH");
    run_ms(&c, 2000);
    CHECK(c.mppt.knee_pwm == knee0 && c.mppt.cliff_pwm_min == fence0 && c.mppt.state == MPPT_HOLD, "fence/knee/HOLD untouched (fence %u knee %u)", c.mppt.cliff_pwm_min, c.mppt.knee_pwm);
    CHECK(c.fault.code == 0, "no fault (fault=%04x)", c.fault.code);
}

static void t3b_real_dip_near_voc(void)
{
    printf("T3b a real 1-sample dip at 99%% Voc delivering 90 mA: bounded backoff, KNEE, then the zero-draw fence is dropped\n");
    system_ctx_t c = fresh_cc(108, 108);                  /* on the fence, 90 mA, 2 counts off zero-draw */
    c.mppt.state = MPPT_HOLD; c.mppt.knee_pwm = 107; c.mppt.panel_voc_mv = 12930; c.mppt.vreg_setpoint_mv = c.meas.panel_voltage;
    c.mppt.knee_probe_ms = h_now; c.mppt.hold_start_ms = h_now; c.mppt.knee_learned_ms = h_now;
    c.charger.panel_op_mv = c.meas.panel_voltage; c.charger.panel_op_ms = h_now;
    run_ms(&c, 1000);
    force_raw_v = 12500; step10(&c);
    CHECK(c.charger.droop_events == 1 && c.charger.droop_glitch_events == 0, "a FALL sighting (drp=%u)", c.charger.droop_events);
    CHECK(c.pwm <= ZERO + CHG_ZERO_DRAW_MARGIN && c.pwm > 108, "backoff bounded at zero-draw+%u: 108 -> %u", CHG_ZERO_DRAW_MARGIN, c.pwm);
    run_ms(&c, 100);
    CHECK(c.mppt.event_knee_accepted && c.mppt.knee_pwm == 108, "KNEE accepted (knee %u fence %u)", c.mppt.knee_pwm, c.mppt.cliff_pwm_min);
    run_ms(&c, 3000);
    CHECK(c.mppt.fence_dropped == 1 && c.mppt.knee_pwm == 0, "fence at the zero-draw count was discarded (fdrop=%u knee=%u)", c.mppt.fence_dropped, c.mppt.knee_pwm);
    run_ms(&c, 6000);
    CHECK(c.meas.chg_current > 100 && c.fault.code == 0, "delivering again (%d mA), no fault", c.meas.chg_current);
}

static void t4_droop_at_the_knee_is_a_knee(void)
{
    printf("T4 a droop at 92%% of Voc delivering 560 mA: knee learned\n");
    system_ctx_t c = fresh_cc(89, 89);
    c.mppt.state = MPPT_HOLD; c.mppt.knee_pwm = 0; c.mppt.panel_voc_mv = 12930;
    c.mppt.knee_probe_ms = h_now; c.mppt.hold_start_ms = h_now;
    VOC = 12930; GAIN_MV = 50;                             /* steeper so 89 sits at ~91 % Voc */
    for (int i = 0; i < 70; i++) conversion(&c);
    c.charger.panel_op_mv = c.meas.panel_voltage; c.charger.panel_op_ms = h_now;
    c.mppt.vreg_setpoint_mv = c.meas.panel_voltage;
    run_ms(&c, 1000);
    printf("  (V_panel %u = %u%% of Voc, Ichg %d)\n", c.meas.panel_voltage, 100 * c.meas.panel_voltage / c.mppt.panel_voc_mv, c.meas.chg_current);
    force_raw_v = (int)(c.meas.panel_voltage - 400); step10(&c);
    run_ms(&c, 100);
    CHECK(c.mppt.event_knee_accepted && c.mppt.knee_pwm == 89 && c.mppt.cliff_pwm_min == 89 + CHG_CLIFF_PWM_MARGIN, "KNEE: knee 89, fence %u (got knee %u fence %u)", 89 + CHG_CLIFF_PWM_MARGIN, c.mppt.knee_pwm, c.mppt.cliff_pwm_min);
    GAIN_MV = 20;
}

static void t5_pinned_at_zero_delivery_drops_the_knee(void)
{
    printf("T5 a fence the loop sits on delivering nothing is discarded\n");
    system_ctx_t c = fresh_cc(110, 110);                  /* fence == zero-draw, as 14.09 v0.40 ended up */
    c.mppt.state = MPPT_HOLD; c.mppt.knee_pwm = 108; c.mppt.knee_learned_ms = h_now; c.mppt.panel_voc_mv = 12930;
    c.mppt.knee_probe_ms = h_now; c.mppt.hold_start_ms = h_now; c.mppt.vreg_setpoint_mv = c.meas.panel_voltage;
    run_ms(&c, 1500);
    CHECK(c.mppt.knee_pwm == 0 && c.mppt.fence_dropped == 1, "knee cleared, fence_dropped=%u (knee=%u)", c.mppt.fence_dropped, c.mppt.knee_pwm);
    CHECK(c.mppt.state == MPPT_TRACKING, "re-acquiring (state=%d)", c.mppt.state);
    run_ms(&c, 6000);
    CHECK(c.pwm < 110 && c.meas.chg_current > 100, "delivering again: pwm %u, Ichg %d", c.pwm, c.meas.chg_current);
    CHECK(c.fault.code == 0, "no fault while parked at zero (fault=%04x)", c.fault.code);
}

static void t6_fast_guard_noise_vs_real_reversal(void)
{
    printf("T6 reverse-pump guard: noise about zero must not latch; a real reversal must\n");
    system_ctx_t c = fresh_cc(110, 0);                    /* zero delivery, live panel */
    noise_sigma = 240.0;
    run_ms(&c, 20000);
    CHECK(c.fault.code == 0, "20 s at zero delivery with 240 mA RMS raw noise: no latch (fault=%04x, raised=%u)", c.fault.code, h_faults_raised);
    noise_sigma = 0;
    /* real, large reversal */
    force_raw_i = -1010; run_ms(&c, 100);
    CHECK((c.fault.code & FAULT_REVERSE_PUMP) && c.charger.reverse_trace_kind == 1, "-1010 mA raw latched on the FAST path within 100 ms (kind=%u)", c.charger.reverse_trace_kind);
    /* moderate sustained reversal: the averaged path, after the escape has failed */
    c = fresh_cc(110, 0); force_raw_i = -150;
    run_ms(&c, 3000);
    CHECK((c.fault.code & FAULT_REVERSE_PUMP) && c.charger.reverse_trace_kind == 2, "-150 mA sustained latched on the AVG path (kind=%u fault=%04x)", c.charger.reverse_trace_kind, c.fault.code);
    force_raw_i = 0;
}

static void t7_settle_lands_on_fence_and_drops_an_impossible_one(void)
{
    printf("T7 CHG_BUCK_SETTLE lands on the fence; a fence the rail cannot clear under is dropped\n");
    system_ctx_t c = fresh_cc(114, 110);
    c.charger.state = CHG_BUCK_SETTLE; h_q49 = false; c.charger.active_start_ms = h_now; c.charger.activation_ready_mv = 3340;
    c.mppt.knee_pwm = 108;
    /* the rail clears V_bat + 20 only at pwm <= 108: h_raw_vchg = 3320 + (110-pwm)*3 → 3326 at 108 ... make ready need 3329 */
    c.charger.activation_ready_mv = 3329;
    run_ms(&c, 400);
    CHECK(c.charger.state == CHG_BUCK_SETTLE && c.pwm == 110, "ramp stopped ON the fence at 110 (pwm=%u state=%d)", c.pwm, c.charger.state);
    uint32_t t0 = h_now;
    while (c.mppt.fence_dropped == 0 && h_now - t0 < 2000) step10(&c);
    CHECK(c.mppt.fence_dropped == 1 && c.mppt.cliff_pwm_min == 0 && c.charger.state == CHG_BUCK_SETTLE, "fence dropped %u ms after landing on it, still in SETL (fence=%u fdrop=%u)", (unsigned)(h_now - t0), c.mppt.cliff_pwm_min, c.mppt.fence_dropped);
    run_ms(&c, 1200);
    CHECK(c.charger.state == CHG_CC && h_q49 && c.pwm < 110, "Q49 closed under the old fence at pwm %u, no snap back to it (state=%d)", c.pwm, c.charger.state);
    CHECK(c.mppt.state == MPPT_TRACKING && c.mppt.cliff_pwm_min == c.pwm, "fresh search re-seeded the fence at the count that cleared the cell (fence=%u pwm=%u)", c.mppt.cliff_pwm_min, c.pwm);
}

static void t8_rescue_restores_the_rail(void)
{
    printf("T8 input-guard rescue restores the rail instead of leaving it 20 counts into reverse\n");
    KNEE = 0; ZERO = 110;
    system_ctx_t c = fresh_cc(95, 95);                    /* delivering 675 mA */
    c.mppt.state = MPPT_HOLD; c.mppt.knee_pwm = 0; c.mppt.panel_voc_mv = 12930; c.mppt.vreg_setpoint_mv = c.meas.panel_voltage;
    c.mppt.knee_probe_ms = h_now; c.mppt.hold_start_ms = h_now;
    run_ms(&c, 1000);
    /* collapse: two raw conversions at the cell, then the panel is back */
    force_raw_v = 3494; step10(&c);
    uint16_t after1 = c.pwm;
    force_raw_v = 3494; step10(&c);
    uint16_t after2 = c.pwm;
    step10(&c);                                            /* recovered: raw V back to the model */
    printf("  backoff 95 -> %u -> %u, restore -> %u (zd=%u)\n", after1, after2, c.pwm, c.charger.zero_draw_pwm);
    CHECK(c.charger.input_dip_events == 1, "RECOVERED (dips=%u)", c.charger.input_dip_events);
    CHECK(c.pwm <= c.charger.zero_draw_pwm + CHG_ZERO_DRAW_MARGIN && c.pwm >= 95 + CHG_CLIFF_PWM_MARGIN, "restored to zero-draw+%u (pwm=%u), not left at %u", CHG_ZERO_DRAW_MARGIN, c.pwm, after2);
    run_ms(&c, 2000);
    CHECK(c.fault.code == 0, "no FAULT_REVERSE_PUMP after the rescue (fault=%04x)", c.fault.code);
}

static void t9_hold_kicked_returns_to_fence(void)
{
    printf("T9 HOLD with the rail kicked above the fence re-probes at once and returns\n");
    system_ctx_t c = fresh_cc(104, 100);                  /* kicked 4 above a fence of 100 */
    c.mppt.state = MPPT_HOLD; c.mppt.knee_pwm = 99; c.mppt.panel_voc_mv = 12930; c.mppt.vreg_setpoint_mv = c.meas.panel_voltage;
    c.mppt.knee_probe_ms = h_now; c.mppt.hold_start_ms = h_now; c.mppt.knee_learned_ms = h_now;
    run_ms(&c, 200);
    CHECK(c.mppt.state == MPPT_TRACKING, "re-probe fired without waiting the 30 s timer (state=%d)", c.mppt.state);
    uint32_t t0 = h_now;
    while (c.pwm != 100 && h_now - t0 < 5000) step10(&c);
    CHECK(c.pwm == 100, "rail back on the fence in %u ms", (unsigned)(h_now - t0));
    run_ms(&c, 4000);
    CHECK(c.mppt.state == MPPT_HOLD && c.pwm == 100 && c.mppt.cliff_pwm_min == 100, "HOLD again at the fence (state=%d pwm=%u fence=%u)", c.mppt.state, c.pwm, c.mppt.cliff_pwm_min);
}

static void t10_stale_knee_released_on_headroom(void)
{
    printf("T10 a knee learned in weak light is released when the panel shows headroom, before the 120 s retreat\n");
    system_ctx_t c = fresh_cc(105, 105);                  /* fence 105 (225 mA), knee 104 learned at 11.2 V */
    c.mppt.state = MPPT_HOLD; c.mppt.knee_pwm = 104; c.mppt.knee_vpanel_mv = 11200; c.mppt.knee_learned_ms = h_now;
    c.mppt.panel_voc_mv = 12930; c.mppt.vreg_setpoint_mv = c.meas.panel_voltage;
    c.mppt.knee_probe_ms = h_now; c.mppt.hold_start_ms = h_now;
    c.charger.plant_mv_per_count = 165;
    /* the model puts 105 at 12830: knee_v 11200 + 165*1 = 11365, headroom = 12830 > 11865 */
    run_ms(&c, 500);
    CHECK(c.mppt.state == MPPT_TRACKING && c.mppt.knee_pwm == 0, "knee released and re-probing within 0.5 s (state=%d knee=%u)", c.mppt.state, c.mppt.knee_pwm);
    /* and NOT released when the panel reads what the knee predicts */
    c = fresh_cc(105, 105);
    c.mppt.state = MPPT_HOLD; c.mppt.knee_pwm = 104; c.mppt.knee_vpanel_mv = 12830 - 165; c.mppt.knee_learned_ms = h_now;
    c.mppt.panel_voc_mv = 12930; c.mppt.vreg_setpoint_mv = c.meas.panel_voltage;
    c.mppt.knee_probe_ms = h_now; c.mppt.hold_start_ms = h_now;
    c.charger.plant_mv_per_count = 165;
    run_ms(&c, 5000);
    CHECK(c.mppt.state == MPPT_HOLD && c.mppt.knee_pwm == 104, "no headroom: still HOLD with the knee (state=%d knee=%u)", c.mppt.state, c.mppt.knee_pwm);
}

static void t11_floor_holds_under_falling_light(void)
{
    printf("T11 falling light with a knee voltage known: the loop backs off at the floor, no collapse\n");
    /* steep plant near the knee: 150 mV/count; the knee is a count below which the panel collapses */
    GAIN_MV = 150; KNEE = 95; VOC = 12930;
    system_ctx_t c = fresh_cc(97, 97);                    /* 2 counts above the knee, 585 mA */
    c.charger.plant_mv_per_count = 150;
    int v97 = 12930 - (110 - 97) * 150;                   /* 10980 */
    int v95 = 12930 - (110 - 95) * 150;                   /* 10680 */
    c.mppt.state = MPPT_HOLD; c.mppt.knee_pwm = 95; c.mppt.knee_vpanel_mv = (uint16_t)(v95 - 300); /* floor = v95, target = v95+band(300) = v97 */
    c.mppt.knee_learned_ms = h_now; c.mppt.panel_voc_mv = 12930; c.mppt.knee_probe_ms = h_now; c.mppt.hold_start_ms = h_now;
    c.mppt.vreg_setpoint_mv = (uint16_t)v97;
    c.charger.panel_op_mv = (uint16_t)v97; c.charger.panel_op_ms = h_now;
    run_ms(&c, 1000);
    printf("  (start: pwm %u V %u sp %u knee %u)\n", c.pwm, c.meas.panel_voltage, c.mppt.vreg_setpoint_mv, c.mppt.knee_pwm);
    /* the light fades for 20 s: Voc -50 mV every 500 ms, the knee count climbs one every 2 s */
    int worst_margin = 99;
    for (int i = 0; i < 40; i++) {
        run_ms(&c, 500);
        VOC -= 50;
        if (i % 4 == 3) KNEE++;
        int margin = (int)c.pwm - KNEE;
        if (margin < worst_margin) worst_margin = margin;
    }
    printf("  (end: pwm %u knee-count %d V %u dips %u drp %u)\n", c.pwm, KNEE, c.meas.panel_voltage, c.charger.input_dip_events, c.charger.droop_events);
    CHECK(c.charger.input_dip_events == 0, "the input guard never had to rescue (dips=%u)", c.charger.input_dip_events);
    CHECK(worst_margin >= 0, "the rail stayed at or above the moving knee count (worst margin %d)", worst_margin);
    CHECK(c.fault.code == 0, "no fault");
    GAIN_MV = 20; KNEE = 0; VOC = 12930;
}

static void t12_stale_high_floor_is_probed_down(void)
{
    printf("T12 a floor learned in brighter light is lowered under test and the harvest recovers\n");
    GAIN_MV = 150; KNEE = 0; VOC = 12930;
    system_ctx_t c = fresh_cc(105, 100);                  /* fence 100 (knee 99), but the floor holds the loop at 105 */
    c.charger.plant_mv_per_count = 150;
    int v105 = 12930 - (110 - 105) * 150;                 /* 12180 */
    c.mppt.state = MPPT_HOLD; c.mppt.knee_pwm = 99; c.mppt.knee_vpanel_mv = (uint16_t)(v105 - 300 - 300); /* floor = v105-300, target = v105 */
    c.mppt.knee_learned_ms = h_now; c.mppt.panel_voc_mv = 12930; c.mppt.knee_probe_ms = h_now; c.mppt.hold_start_ms = h_now;
    c.mppt.vreg_setpoint_mv = (uint16_t)v105;
    run_ms(&c, 2000);
    CHECK(c.mppt.state == MPPT_HOLD && c.pwm == 105, "held above the fence by the floor, not kicked (state=%d pwm=%u)", c.mppt.state, c.pwm);
    uint16_t floor0 = c.mppt.knee_vpanel_mv;
    run_ms(&c, 60000);                                    /* the 30 s re-probe, then the floor probe */
    printf("  (after 60 s: pwm %u fence %u knee_v %u -> %u state %d)\n", c.pwm, c.mppt.cliff_pwm_min, floor0, c.mppt.knee_vpanel_mv, c.mppt.state);
    CHECK(c.mppt.knee_vpanel_mv < floor0, "knee voltage lowered by the floor probe (%u -> %u)", floor0, c.mppt.knee_vpanel_mv);
    CHECK(c.pwm <= 101, "rail descended to the fence region (pwm=%u)", c.pwm);
    CHECK(c.fault.code == 0 && c.charger.input_dip_events == 0, "no fault, no rescue");
    GAIN_MV = 20;
}

int main(void)
{
    t1_fine_reject_rolls_back();
    t2_coarse_reject_and_arrival_settle();
    t3_switching_glitch_is_not_a_droop();
    t3b_real_dip_near_voc();
    t4_droop_at_the_knee_is_a_knee();
    t5_pinned_at_zero_delivery_drops_the_knee();
    t6_fast_guard_noise_vs_real_reversal();
    t7_settle_lands_on_fence_and_drops_an_impossible_one();
    t8_rescue_restores_the_rail();
    t9_hold_kicked_returns_to_fence();
    t10_stale_knee_released_on_headroom();
    t11_floor_holds_under_falling_light();
    t12_stale_high_floor_is_probed_down();
    printf("\n%d checks, %d failed\n", checks, fails);
    return fails ? 1 : 0;
}
