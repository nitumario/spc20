/*
 * power_budget.c — Current Allocation (Pipeline Step 3)
 *
 *   i_buck_max  = MIN(BUCK_MAX_CURRENT_MA, mppt_limit_ma)
 *   allowed_chg = CLAMP(i_buck_max - I_load, 0, bat_limit)
 *
 * bat_limit: <3V → 200 mA trickle; <3.65V → 1000 mA CC; ≥3.65V → 200 mA cap.
 */

#include "power_budget.h"

void power_budget_update(system_ctx_t *ctx)
{
    /* ── Step 1: i_buck_max = min(hardware limit, panel deliverable) ── */
    uint16_t mppt_lim = ctx->mppt.mppt_limit_ma;
    ctx->i_buck_max = (BUCK_MAX_CURRENT_MA < mppt_lim)
                    ? BUCK_MAX_CURRENT_MA
                    : mppt_lim;

    /* ── Step 2: bat_limit by voltage zone ── */
    uint16_t bat_limit;

    if (ctx->meas.bat_voltage < BAT_PRECHARGE_MV) {
        bat_limit = BAT_PRECHARGE_MAX_MA;   /* 200 mA trickle */
    } else if (ctx->meas.bat_voltage < BAT_CV_VOLTAGE_MV) {
        bat_limit = BAT_CC_MAX_MA;          /* 1000 mA bulk */
    } else {
        bat_limit = BAT_CV_TAPER_MA;        /* 200 mA near-full cap */
    }

    /* ── Step 3: allowed_chg = i_buck_max - I_load, clamped to [0, bat_limit] ── */
    int32_t budget = (int32_t)ctx->i_buck_max - (int32_t)ctx->meas.dsg_current;

    if (budget > (int32_t)bat_limit) budget = (int32_t)bat_limit;
    if (budget < 0)                  budget = 0;

    ctx->allowed_chg = (uint16_t)budget;
}
