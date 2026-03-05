/**
 * @file main.c
 * @brief top-level system state machine
 */

#include "system_types.h"
#include "hw_config.h"
#include "hw_hal.h"
#include "measurements.h"
#include "power_budget.h"
#include "energy_mode.h"
#include "charger.h"
#include "mppt.h"
#include "fault_mgr.h"
#include "ti_msp_dl_config.h"


/* System context */

static system_ctx_t sys;

/* Timing helpers */
/**
 * @brief Check if enough time has passed since the last timestamp
 * @param last_ms Pointer to timestamp to check (updated if elapsed)
 * @param period_ms Period to check against
 * @return true if enough time has passed, false otherwise, and updates the timestamp if true
 */

 static bool timer_elapsed(uint32_t *last_ms, uint32_t period_ms) {
    uint32_t now = get_system_time_ms();
        if ((now - *last_ms) >= period_ms) {
        *last_ms = now;
        return true;
    }
    return false;
}
