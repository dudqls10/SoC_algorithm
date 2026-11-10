#ifndef SENSOR1_FUSION_CAP_NOISE_H
#define SENSOR1_FUSION_CAP_NOISE_H

#include <stdint.h>
#include <string.h>
#include <math.h>

#define FUSION_CAP_NOISE_SAMPLES 80U

/* Private fusion statistics. Never write the compensation or legacy DO state.
 * v3 feeds the previous pre-EMA compensated count through DI/integration before
 * collecting DOinit80. This is not the standard deviation of chip RAW counts. */
typedef struct {
    uint32_t di[3];
    int64_t integral;
    int32_t startup[FUSION_CAP_NOISE_SAMPLES];
    int32_t mean;
    uint32_t count;
    uint32_t std_count;
    uint8_t ready;
} FusionCapNoiseState;

static inline void FusionCapNoiseReset(FusionCapNoiseState *s)
{
    memset(s, 0, sizeof(*s));
    s->integral = 10000000;
}

static inline void FusionCapNoiseStep(FusionCapNoiseState *s,
        uint32_t previous_pre_ema, uint8_t integrate, uint8_t collect)
{
    s->di[2] = s->di[1];
    s->di[1] = s->di[0];
    s->di[0] = previous_pre_ema;
    if (integrate != 0U)
        s->integral += (int64_t)s->di[2] - (int64_t)s->di[1];

    if ((collect == 0U) || (s->count >= FUSION_CAP_NOISE_SAMPLES)) return;
    s->startup[s->count++] = (int32_t)s->integral;
    if (s->count == FUSION_CAP_NOISE_SAMPLES)
    {
        int64_t total = 0;
        uint64_t squared = 0U;
        for (uint32_t i = 0U; i < FUSION_CAP_NOISE_SAMPLES; ++i)
            total += s->startup[i];
        s->mean = (int32_t)((double)total * 0.0125);
        for (uint32_t i = 0U; i < FUSION_CAP_NOISE_SAMPLES; ++i)
        {
            int64_t delta = (int64_t)s->startup[i] - s->mean;
            squared += (uint64_t)(delta * delta);
        }
        s->std_count = (uint32_t)sqrt((double)squared * 0.0125);
        s->ready = 1U;
    }
}

#endif
