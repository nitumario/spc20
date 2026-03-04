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

/* System context */

