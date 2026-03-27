/*
 * power_budget.c — Current Allocation (Pipeline Step 3)
 * ======================================================
 *
 * Physical model
 * --------------
 * The battery bus is a single voltage rail. Three things connect to it:
 *
 *    Buck output ──┬── Battery (via charge/discharge MOSFETs)
 *                  └── Loads   (via output switch)
 *
 * Kirchhoff's current law at the bus node:
 *
 *    I_buck = I_charge + I_load
 *
 * Rearranged for what the battery gets:
 *
 *    I_charge = I_buck - I_load
 *
 * The firmware controls I_buck (via PWM). I_load is measured, not
 * controlled — loads take what they need. So the maximum current
 * available for charging is:
 *
 *    allowed_chg = i_buck_max - I_load
 *
 * Where i_buck_max is the lesser of:
 *    - The buck hardware rating (inductor/FET saturation limit)
 *    - The panel's deliverable current at the current operating point
 *      (set by MPPT when it's tracking, or BUCK_MAX when panel has headroom)
 *
 * Then we clamp allowed_chg to the battery's safe intake limit,
 * which depends on how full the cell is.
 *
 *
 * Worked examples
 * ---------------
 *
 * Example 1: Sunny day, 500 mA load, normal battery (3.3V)
 *   i_buck_max  = MIN(2000, 2000) = 2000  (MPPT disabled, no panel constraint)
 *   budget      = 2000 - 500 = 1500
 *   bat_limit   = 2000  (CC zone)
 *   allowed_chg = 1500 mA    ← charger CC targets this
 *
 * Example 2: Cloudy, panel limited to 1200 mA, 800 mA load, normal battery
 *   i_buck_max  = MIN(2000, 1200) = 1200  (MPPT says panel maxes at 1200)
 *   budget      = 1200 - 800 = 400
 *   bat_limit   = 2000
 *   allowed_chg = 400 mA
 *
 * Example 3: Heavy load exceeds panel capacity
 *   i_buck_max  = MIN(2000, 1200) = 1200
 *   budget      = 1200 - 1500 = -300
 *   Clamped → 0
 *   allowed_chg = 0 mA    ← no current for battery.
 *   The battery automatically covers the 300 mA deficit (physics, not code).
 *
 * Example 4: Empty battery (2.8V), no load, full sun
 *   i_buck_max  = 2000
 *   budget      = 2000 - 0 = 2000
 *   bat_limit   = 200  (precharge zone: V_bat < 3000 mV)
 *   allowed_chg = 200 mA   ← gentle trickle to protect damaged cell
 *
 * Example 5: Nearly full battery (3.66V), no load, full sun
 *   i_buck_max  = 2000
 *   budget      = 2000 - 0 = 2000
 *   bat_limit   = 200  (near-full zone: V_bat >= 3650 mV)
 *   allowed_chg = 200 mA   ← safety cap. Charger should be in CV by now,
 *                              but if it hasn't transitioned yet, this
 *                              prevents slamming 2A into a full cell.
 */

#include "power_budget.h"

void power_budget_update(system_ctx_t *ctx)
{
    /* ── Step 1: i_buck_max ──
     *
     * The maximum total current the buck can push into the bus.
     * Two ceilings, take the lower:
     *
     *   BUCK_MAX_CURRENT_MA (2000 mA)
     *     Hardware limit. The inductor saturates and the FET overheats
     *     above this. Fixed by component selection, never changes.
     *
     *   mppt_limit_ma
     *     What the solar panel can actually deliver through the buck
     *     at the current operating point on its I-V curve.
     *     Set by MPPT module:
     *       MPPT_DISABLED  → BUCK_MAX (panel has headroom, not the bottleneck)
     *       MPPT_HOLD      → computed from best power found during tracking
     *       MPPT_TRACKING  → previous value (MPPT is still searching)
     *
     * When the panel is the bottleneck (cloudy, late afternoon),
     * mppt_limit < BUCK_MAX, and i_buck_max follows the panel.
     */
    uint16_t mppt_lim = ctx->mppt.mppt_limit_ma;
    ctx->i_buck_max = (BUCK_MAX_CURRENT_MA < mppt_lim)
                    ? BUCK_MAX_CURRENT_MA
                    : mppt_lim;

    /* ── Step 2: battery_limit ──
     *
     * The maximum current the battery can safely accept, based on
     * its state of charge (approximated by terminal voltage).
     *
     * LiFePO4 has three charging zones:
     *
     *   PRECHARGE (V_bat < 3000 mV):
     *     Cell is deeply discharged or possibly damaged.
     *     Limit to 200 mA trickle current. If the cell can't climb
     *     above 3.0V within 15 minutes at this rate, it's dead
     *     (fault_mgr handles that timeout).
     *
     *   CC (3000 mV <= V_bat < 3650 mV):
     *     Normal bulk charging. Battery can accept up to 2000 mA
     *     (1C for a 2Ah cell, 0.1C for 20Ah — safe for LiFePO4).
     *     This is where most of the energy transfer happens.
     *
     *   NEAR-FULL (V_bat >= 3650 mV):
     *     Cell is approaching full charge. The charger SHOULD be in
     *     CV mode by now (regulating voltage, not current). But as
     *     defense-in-depth: if the charger is still in CC due to a
     *     one-tick transition delay, cap current at 200 mA.
     *     This prevents pushing 2A into a cell that's at 3.65V,
     *     which would overshoot to 3.7V+ and trigger the overvoltage
     *     fault (BAT_OVERVOLT_MV = 3700).
     */
    uint16_t bat_limit;

    if (ctx->meas.bat_voltage < BAT_PRECHARGE_MV) {
        bat_limit = BAT_PRECHARGE_MAX_MA;       /* 200 mA — gentle trickle   */
    } else if (ctx->meas.bat_voltage < BAT_CV_VOLTAGE_MV) {
        bat_limit = BAT_CC_MAX_MA;              /* 2000 mA — full rate       */
    } else {
        bat_limit = BAT_CV_TAPER_MA;            /* 200 mA — safety cap       */
    }

    /* ── Step 3: allowed_chg ──
     *
     * Subtract load current from the buck's capacity.
     * What's left is available for charging.
     *
     * Signed arithmetic because dsg_current (load) can exceed
     * i_buck_max — this is the "battery covering the deficit" case.
     * The result goes negative, and we clamp to zero.
     */
    int32_t budget = (int32_t)ctx->i_buck_max - (int32_t)ctx->meas.dsg_current;

    /* Upper clamp: don't exceed what the battery can safely accept */
    if (budget > (int32_t)bat_limit) {
        budget = (int32_t)bat_limit;
    }

    /* Lower clamp: allowed_chg is a CC target, never negative.
     * When budget <= 0, the charger targets zero — the buck feeds
     * loads only. If loads exceed the buck, the battery discharges
     * to cover the deficit. That's physics, not a command. */
    if (budget < 0) {
        budget = 0;
    }

    ctx->allowed_chg = (uint16_t)budget;
}
