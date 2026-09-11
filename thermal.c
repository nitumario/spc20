/*
 * thermal.c — Thermal foldback (pipeline step 4b)
 * ================================================
 *
 * See thermal.h for why this module exists and what it can and cannot do.
 */

#include "thermal.h"
#include "hw_config.h"
#include "SPCBoardAPI.h"

/* Clamp a candidate derate to the legal band [MIN_PCT, 100]. */
static uint8_t clamp_derate(int16_t pct)
{
    if (pct > 100)
        return 100;
    if (pct < (int16_t)THERMAL_FOLDBACK_MIN_PCT)
        return (uint8_t)THERMAL_FOLDBACK_MIN_PCT;
    return (uint8_t)pct;
}

bool thermal_update(system_ctx_t *ctx)
{
    thermal_ctx_t *th  = &ctx->thermal;
    uint8_t        old = th->derate_pct;

    /*
     * A dead sensor must not derate. temp_sensor_ok false means board_temp is
     * a held-over stale value; acting on it would dim the lamps on the
     * strength of a reading measurements_update has already disowned. The
     * charge block it raises is the conservative response; dimming the light
     * as well would just be damage. Restore full brightness and stand down.
     */
    if (!ctx->temp_sensor_ok) {
        th->derate_pct = 100;
        th->active     = false;
        return (old != th->derate_pct);
    }

    /*
     * A latched OVERTEMP means fault_mgr has already shed the LED boost, the
     * output switch and USB. There is no lamp current left to trade, and
     * holding a derate here would mean the lamps came back dim after the
     * fault cleared for no reason the user could see. Reset and let the
     * normal foldback re-engage from 100 % on the next hot tick.
     */
    if (ctx->fault.code & FAULT_OVERTEMP) {
        th->derate_pct = 100;
        th->active     = false;
        return (old != th->derate_pct);
    }

    int16_t board = ctx->meas.board_temp;

    /*
     * Deadband between START and RESUME. Inside it the derate simply holds:
     * this is the band the loop is *meant* to settle in, and stepping here
     * would hunt against a plant whose time constant is minutes.
     */
    bool too_hot   = (board >= (int16_t)THERMAL_FOLDBACK_START_C);
    bool cool_down = (board <  (int16_t)THERMAL_FOLDBACK_RESUME_C);

    if (!too_hot && !cool_down) {
        th->active = (th->derate_pct < 100);
        return false;
    }

    /* Nothing to do if we are already at the rail in the wanted direction. */
    if (too_hot && th->derate_pct == (uint8_t)THERMAL_FOLDBACK_MIN_PCT) {
        th->active = true;
        return false;
    }
    if (cool_down && th->derate_pct == 100) {
        th->active = false;
        return false;
    }

    /*
     * Pacing. One step per THERMAL_FOLDBACK_INTERVAL_MS — the same discipline
     * as the charger's CC_DOWNSTEP_INTERVAL_MS and the MPPT dwell: never step
     * faster than the measurement can confirm the last step. board_temp is a
     * 64-sample average (~640 ms of group delay) sitting on a plant that
     * moves in minutes, so 4 s per 5 % is already generous.
     */
    uint32_t now = time_now();
    if ((uint32_t)(now - th->last_step_ms) < THERMAL_FOLDBACK_INTERVAL_MS) {
        return false;
    }
    th->last_step_ms = now;

    if (too_hot) {
        th->derate_pct = clamp_derate((int16_t)th->derate_pct
                                      - (int16_t)THERMAL_FOLDBACK_STEP_PCT);
    } else {
        th->derate_pct = clamp_derate((int16_t)th->derate_pct
                                      + (int16_t)THERMAL_FOLDBACK_STEP_PCT);
    }

    th->active = (th->derate_pct < 100);
    return (old != th->derate_pct);
}
