#include "power_budget.h"

void power_budget_init(power_budget_t *budget) {
    budget->allowed_chg = 0;
    budget->mppt_limit = BUCK_MAX_CURRENT_MA;
    budget->i_buck_max = BUCK_MAX_CURRENT_MA;
    budget->battery_limit = 0;
    budget->i_bat_net = 0;
    budget->has_sun = false;
    budget->has_load = false;
    budget->bat_low = false;
    budget->bat_full = false;
    budget->can_charge = false;
    budget->buck_needed = false;
    budget->panel_limited = false;
}

void power_budget_update(power_budget_t *budget, const measurements_t *meas) {
    /* equation needs to be implemented */
    (void)budget;
    (void)meas;
}