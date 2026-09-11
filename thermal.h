/*
 * thermal.h — Thermal foldback (pipeline step 4b)
 * ================================================
 *
 * The graceful stage underneath FAULT_OVERTEMP.
 *
 * WHY THIS EXISTS
 * ---------------
 * The board's only thermal response used to be the hard fault: cross
 * BOARD_TEMP_MAX_C and fault_mgr sheds the lamps, USB, output switch and the
 * buck at once, then refuses to restore any of it until the board has cooled
 * 10 degC. On a unit that simply runs warm under a large lamp load that turns
 * a margin problem into a blackout, and the 10 degC recovery band means the
 * blackout lasts minutes.
 *
 * Bench capture serial_20260910_155320.log (corrected for the thermistor
 * bias-rail error — see THERMISTOR_BIAS_MV) shows the shape of the problem: a
 * sustained ~2.7 A load walks the board 33 degC -> ~58 degC over half an hour
 * and then holds there. That is 2 degC of margin, permanently, at full
 * brightness. Sooner or later it clips the fault.
 *
 * WHAT IT DOES
 * ------------
 * Above THERMAL_FOLDBACK_START_C it walks ctx->thermal.derate_pct down, one
 * THERMAL_FOLDBACK_STEP_PCT step per THERMAL_FOLDBACK_INTERVAL_MS, to a floor
 * of THERMAL_FOLDBACK_MIN_PCT. Below THERMAL_FOLDBACK_RESUME_C it walks it
 * back up to 100. Between the two it holds — that deadband is what stops the
 * loop hunting.
 *
 * derate_pct is applied by main.c's lamp_apply() as a multiplier on the
 * requested LED current. It never touches ctx->lamp_level[], so the user's
 * brightness choice survives the event, the buttons keep working while
 * derated, and led_boost_follow_lamps() still sees a lit lamp as lit.
 *
 * WHAT IT CANNOT DO
 * -----------------
 * Lamps are the only load the firmware can modulate. USB cannot be derated at
 * all (the AP2151 load switches cannot be pulse-gated — see SPCBoardAPI.c and
 * the reverted idle-gating attempt), and the charger is already governed by
 * its own limits. If the heat is coming from a USB load, foldback will run to
 * its floor with no effect and FAULT_OVERTEMP remains the backstop.
 *
 * PLACEMENT IN THE PIPELINE
 * -------------------------
 * Runs as step 4b, immediately after fault_mgr_update and before
 * energy_mode_update: it needs this tick's temperatures and this tick's fault
 * state (a latched OVERTEMP means the rails are already shed and there is
 * nothing left to derate), and it must publish derate_pct before anything
 * downstream drives the lamps.
 */

#ifndef THERMAL_H
#define THERMAL_H

#include "system_types.h"

/*
 * Update ctx->thermal.derate_pct from ctx->meas.board_temp.
 *
 * Returns true if derate_pct CHANGED on this tick, i.e. the caller must
 * re-apply the lamp currents for the new multiplier to take effect. Returning
 * the edge rather than re-driving the LED timers every tick keeps the 50 ms
 * pipeline off the LED current LUT interpolation in the common case.
 */
bool thermal_update(system_ctx_t *ctx);

#endif /* THERMAL_H */
