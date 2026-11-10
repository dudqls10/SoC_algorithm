#ifndef MOBILE_SOC_EVALUATION_TRACE_H
#define MOBILE_SOC_EVALUATION_TRACE_H

#include <stdint.h>

#define EVAL_TRACE_DEPTH 512U
#define EVAL_STAGE_COUNT 8U
/* Stage order: I2C pair, Kalman, drift, EMA, calibration, ToF,
 * online curve fitting, Fusion. Entries contain MCU time, not host estimates. */
typedef struct {
    uint32_t sequence;
    uint32_t tick_ms;
    uint32_t raw_ch0;
    uint32_t raw_ch1;
    uint32_t filtered_ch0;
    uint32_t filtered_ch1;
    uint32_t pre_ema;
    uint32_t post_ema;
    uint32_t alpha_ppm;
    uint32_t flags;
    uint32_t learning_updates;
    int32_t coefficient;
    int32_t offset;
    int32_t cap_mm;
    int32_t fusion_mm;
    uint32_t tof_mm;
    uint32_t total_us;
    uint32_t stage_us[EVAL_STAGE_COUNT];
} EvaluationTraceEntry;

extern volatile uint32_t eval_tim7_interrupts;
extern volatile uint32_t eval_tim7_coalesced;
extern volatile uint32_t eval_tim6_interrupts;
extern volatile uint32_t eval_tim6_coalesced;

#endif
