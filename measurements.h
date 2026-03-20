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
 *   State machines NEVER read ctx->meas for transition decisions.
 *   They read flags only. This ensures all noise filtering happens
 *   in one place (here), and state machines see clean signals.
 */

#ifndef MEASUREMENTS_H
#define MEASUREMENTS_H

#include "system_types.h"

/* Step 1: read hardware, fill ctx->meas */
void measurements_update(system_ctx_t *ctx);

/* Step 2: evaluate ctx->meas against thresholds, update flags */
void flags_update(system_ctx_t *ctx);

#endif /* MEASUREMENTS_H */
