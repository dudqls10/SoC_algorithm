#ifndef MOBILE_SOC_DRIFT_NOISE_REFERENCE_H
#define MOBILE_SOC_DRIFT_NOISE_REFERENCE_H

#include <math.h>
#include <stdint.h>
#include <string.h>

/* Independent from the 80-pair regression and the 10-sample detector mean.
 * A short EMA window measures local smoothness, not multi-second variation.
 * Fit this reference once from an approved clear interval, then freeze it. */
#define DRIFT_NOISE_SETTLE_MS 1000U
#define DRIFT_NOISE_WINDOW_MS 5000U
#define DRIFT_NOISE_MIN_SAMPLES 500U
#define DRIFT_NOISE_MAX_GAP_MS 50U

typedef struct {
    double mean;
    double m2;
    double measured_sigma;
    uint32_t sample_count;
    uint32_t clear_since_tick;
    uint32_t first_sample_tick;
    uint32_t last_tick;
    uint32_t sigma_counts;
    uint8_t collecting;
    uint8_t ready;
} DriftNoiseReference;

static inline void DriftNoiseReferenceReset(DriftNoiseReference *n)
{
    memset(n, 0, sizeof(*n));
}

/* Reject only a pending calibration. A later object must never inflate the
 * accepted noise or reset a valid reference into relearning that object. */
static inline void DriftNoiseReferenceReject(DriftNoiseReference *n)
{
    if (!n->ready) DriftNoiseReferenceReset(n);
}

static inline uint32_t DriftNoiseReferenceSigma(const DriftNoiseReference *n,
        uint32_t fallback_sigma)
{
    return n->ready ? n->sigma_counts : fallback_sigma;
}

static inline uint8_t DriftNoiseReferenceAllowsLearning(
        const DriftNoiseReference *n, uint8_t first_model_ready)
{
    /* The first regression must complete before its output can be measured. */
    return (uint8_t)(!first_model_ready || n->ready);
}

static inline uint8_t DriftNoiseReferenceStep(DriftNoiseReference *n,
        uint32_t tick, uint32_t post_ema, uint8_t approved_clear)
{
    if (n->ready) return 1U;
    if (!approved_clear)
    {
        DriftNoiseReferenceReject(n);
        return 0U;
    }
    if (n->collecting &&
        (uint32_t)(tick - n->last_tick) > DRIFT_NOISE_MAX_GAP_MS)
        DriftNoiseReferenceReject(n);
    if (!n->collecting)
    {
        n->collecting = 1U;
        n->clear_since_tick = tick;
    }
    n->last_tick = tick;
    if ((uint32_t)(tick - n->clear_since_tick) < DRIFT_NOISE_SETTLE_MS)
        return 0U;

    if (!n->sample_count) n->first_sample_tick = tick;
    n->sample_count++;
    double delta = (double)post_ema - n->mean;
    n->mean += delta / (double)n->sample_count;
    n->m2 += delta * ((double)post_ema - n->mean);
    if (n->sample_count < DRIFT_NOISE_MIN_SAMPLES ||
        (uint32_t)(tick - n->first_sample_tick) < DRIFT_NOISE_WINDOW_MS)
        return 0U;

    double variance = n->m2 / (double)n->sample_count;
    if (!isfinite(variance) || variance < 0.0)
    {
        DriftNoiseReferenceReject(n);
        return 0U;
    }
    n->measured_sigma = sqrt(variance);
    double sigma = ceil(n->measured_sigma);
    if (sigma > (double)UINT32_MAX) sigma = (double)UINT32_MAX;
    n->sigma_counts = (uint32_t)sigma;
    n->ready = 1U;
    return 1U;
}

#endif
