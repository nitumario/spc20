/*
 * measurements.h — ADC Reading + Flag Evaluation
 * ================================================
 *
 * Pipeline steps 1 and 2:
 *
 *   Step 1: measurements_update(ctx)
 *     Reads all ADC channels via HAL, converts to engineering units,
 *     stores into ctx->meas. No decisions — just "what do the sensors say?"
 *
 *   Step 2: flags_update(ctx)
 *     Evaluates ctx->meas against thresholds from hw_config.h.
 *     Applies debounce (bat_low, has_sun) or hysteresis (has_load)
 *     to produce the boolean flags that state machines read.
 *
 * Separation:
 *   State-machine transitions run off the debounced flags rather than
 *   raw ctx->meas, so noise filtering happens in one place (here). The
 *   few deliberate exceptions read raw meas on purpose: the charger's
 *   V_bat precharge/CC/CV thresholds, and SAFE_MODE recovery (which must
 *   NOT be debounced — see energy_mode.c and the CHANGELOG).
 */

#ifndef MEASUREMENTS_H
#define MEASUREMENTS_H

#include "system_types.h"

/* Step 1: read hardware, fill ctx->meas */
void measurements_update(system_ctx_t *ctx);

/* Step 2: evaluate ctx->meas against thresholds, update flags */
void flags_update(system_ctx_t *ctx);

#endif /* MEASUREMENTS_H */
