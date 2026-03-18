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


/* timestamps for periodic tasks */
static uint32_t last_fault_ms;
static uint32_t last_budget_ms;
static uint32_t last_charger_ms;
static uint32_t last_mppt_ms;
static uint32_t last_fault_recovery_ms;


/* System init */

static void system_init(void){

    SYSCFG_DL_init();

    /*disable all outputs so its in a safe starting state*/
    hal_charger_enable(false);
    hal_battery_enable(false);
    hal_output_enable(false);
    hal_usb_enable(false);
    hal_led_driver_enable(false);
    hal_buck_set_pwm(PWM_MAX); /*minimum duty cycle*/


    /* initialize all modules */

    measurements_init();
    power_budget_init(&sys.budget);
    energy_mode_init(&sys.energy);
    charger_init(&sys.charger);
    mppt_init(&sys.mppt);
    fault_mgr_init(&sys.faults);



    

}
