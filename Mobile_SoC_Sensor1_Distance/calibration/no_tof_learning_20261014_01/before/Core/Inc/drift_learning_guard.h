#ifndef MOBILE_SOC_DRIFT_LEARNING_GUARD_H
#define MOBILE_SOC_DRIFT_LEARNING_GUARD_H

#include <math.h>
#include <stdint.h>
#include <string.h>

/* Trial parameters, to be checked with stationary and thermal measurements.
 * These govern learning only; they do not change the object detector or EMA. */
#define DRIFT_LEARN_CLEAR_MS 2000U
#define DRIFT_LEARN_RESIDUAL_HOLD_MS 200U
#define DRIFT_LEARN_QUARANTINE_MS 2400U
#define DRIFT_LEARN_TOF_FRESH_MS 200U
#define DRIFT_LEARN_NOISE_FLOOR 50.0
#define DRIFT_LEARN_REFERENCE_MS 16000U
#define DRIFT_LEARN_CAP_ONLY_REFERENCE_MS 64000U
#define DRIFT_LEARN_TOF_SAMPLES 16U
#define DRIFT_LEARN_PENDING_DEPTH 8U
#define DRIFT_LEARN_RESIDUAL_DEPTH 10U

enum {
    DRIFT_LEARN_OBJECT = 1U,
    DRIFT_LEARN_TOF_NEAR = 2U,
    DRIFT_LEARN_TOF_UNTRUSTED = 4U,
    DRIFT_LEARN_RESIDUAL = 8U,
    DRIFT_LEARN_INVALID = 16U,
    DRIFT_LEARN_WAIT_CLEAR = 32U
};

typedef struct {
    uint32_t ch0, ch1, tick_ms;
} DriftPendingPair;

typedef struct {
    int64_t coefficient, offset;
    double noise, residual_mean, clear_sum;
    double residuals[DRIFT_LEARN_RESIDUAL_DEPTH];
    uint16_t tof_samples[DRIFT_LEARN_TOF_SAMPLES];
    double tof_reference, tof_noise;
    uint32_t reference_tick, clear_tick, last_tof_sequence, residual_high_tick, clear_count;
    DriftPendingPair pending[DRIFT_LEARN_PENDING_DEPTH];
    uint8_t ready, allowed, reason, clear_timer, residual_high_timer;
    uint8_t residual_count, residual_head, tof_count, tof_near;
    uint8_t pending_count, pending_head;
} DriftLearningGuard;

static inline void DriftLearningGuardPause(DriftLearningGuard *g, uint8_t reason)
{
    g->allowed = 0U;
    g->reason = reason;
    g->clear_timer = 0U;
    g->clear_sum = 0.0;
    g->clear_count = 0U;
    g->pending_count = 0U;
    g->pending_head = 0U;
}

static inline void DriftLearningGuardReset(DriftLearningGuard *g)
{
    memset(g, 0, sizeof(*g));
    g->noise = DRIFT_LEARN_NOISE_FLOOR;
    g->tof_noise = 1.0;
}

/* Seed from the first fitted, stationary CH0/CH1 window. Use this residual's
 * own noise, rather than the unrelated post-EMA detector standard deviation. */
static inline void DriftLearningGuardSeed(DriftLearningGuard *g, uint32_t tick,
        int64_t coefficient, int64_t offset, const uint32_t *ch0,
        const uint32_t *ch1, uint32_t count)
{
    double mean = 0.0, variance = 0.0;
    double gain = (double)coefficient * .0001;
    for (uint32_t i = 0U; i < count; ++i)
        mean += (double)ch0[i] - gain * (double)ch1[i] - (double)offset;
    mean /= (double)count;
    for (uint32_t i = 0U; i < count; ++i)
    {
        double d = (double)ch0[i] - gain * (double)ch1[i] - (double)offset - mean;
        variance += d * d;
    }
    g->noise = sqrt(variance / (double)count);
    if (g->noise < DRIFT_LEARN_NOISE_FLOOR) g->noise = DRIFT_LEARN_NOISE_FLOOR;
    g->coefficient = coefficient;
    g->offset = offset;
    g->reference_tick = tick;
    g->ready = 1U;
    g->residual_count = 0U;
    g->residual_head = 0U;
    DriftLearningGuardPause(g, DRIFT_LEARN_WAIT_CLEAR);
}

static inline uint8_t DriftLearningGuardStep(DriftLearningGuard *g,
        uint32_t tick, uint32_t ch0, uint32_t ch1, uint8_t pair_valid,
        uint8_t object, uint32_t tof_sequence, uint16_t tof_mm,
        uint8_t tof_valid)
{
    uint8_t reason = (object != 0U) ? DRIFT_LEARN_OBJECT : 0U;
    if (!pair_valid)
    {
        DriftLearningGuardPause(g, DRIFT_LEARN_INVALID);
        g->residual_count = 0U;
        g->residual_head = 0U;
        g->residual_high_timer = 0U;
        g->reference_tick = tick;
        return 0U;
    }
    /* The fixture's confirmed clear startup is the optical reference. Never
     * compare an uncalibrated ToF measurement with a hard-coded 300 mm. */
    if (tof_valid && tof_sequence != g->last_tof_sequence)
    {
        g->last_tof_sequence = tof_sequence;
        if (g->tof_count < DRIFT_LEARN_TOF_SAMPLES && !object)
        {
            g->tof_samples[g->tof_count++] = tof_mm;
            if (g->tof_count == DRIFT_LEARN_TOF_SAMPLES)
            {
                double mean = 0.0, variance = 0.0;
                for (uint32_t i = 0U; i < DRIFT_LEARN_TOF_SAMPLES; ++i)
                    mean += g->tof_samples[i];
                mean /= DRIFT_LEARN_TOF_SAMPLES;
                for (uint32_t i = 0U; i < DRIFT_LEARN_TOF_SAMPLES; ++i)
                {
                    double d = (double)g->tof_samples[i] - mean;
                    variance += d * d;
                }
                g->tof_reference = mean;
                g->tof_noise = sqrt(variance / DRIFT_LEARN_TOF_SAMPLES);
                if (g->tof_noise < 1.0) g->tof_noise = 1.0;
            }
        }
    }
    if (g->tof_count == DRIFT_LEARN_TOF_SAMPLES)
    {
        if (!tof_valid) reason |= DRIFT_LEARN_TOF_UNTRUSTED;
        else
        {
            double enter = fmax(5.0, 4.0 * g->tof_noise);
            double release = fmax(3.0, 2.0 * g->tof_noise);
            if ((double)tof_mm < g->tof_reference - enter) g->tof_near = 1U;
            else if ((double)tof_mm >= g->tof_reference - release) g->tof_near = 0U;
        }
        if (g->tof_near) reason |= DRIFT_LEARN_TOF_NEAR;
    }
    else if (g->ready && g->tof_count != 0U)
        reason |= DRIFT_LEARN_TOF_UNTRUSTED;

    if (g->ready)
    {
        double residual = (double)ch0 - (double)g->coefficient * .0001 * (double)ch1
                          - (double)g->offset;
        g->residuals[g->residual_head] = residual;
        g->residual_head = (g->residual_head + 1U) % DRIFT_LEARN_RESIDUAL_DEPTH;
        if (g->residual_count < DRIFT_LEARN_RESIDUAL_DEPTH) g->residual_count++;
        g->residual_mean = 0.0;
        for (uint32_t i = 0U; i < g->residual_count; ++i)
            g->residual_mean += g->residuals[i];
        g->residual_mean /= g->residual_count;
        if (fabs(g->residual_mean) >= 1.5 * g->noise)
        {
            if (!g->residual_high_timer)
            {
                g->residual_high_timer = 1U;
                g->residual_high_tick = tick;
            }
            if ((uint32_t)(tick - g->residual_high_tick) >= DRIFT_LEARN_RESIDUAL_HOLD_MS)
                reason |= DRIFT_LEARN_RESIDUAL;
        }
        else g->residual_high_timer = 0U;
    }
    if (reason)
    {
        DriftLearningGuardPause(g, reason);
        g->reference_tick = tick;
        return 0U;
    }
    if (!g->ready) { g->allowed = 1U; g->reason = 0U; return 1U; }
    if (g->allowed) { g->reason = 0U; return 1U; }
    if (g->residual_count < DRIFT_LEARN_RESIDUAL_DEPTH)
    {
        g->clear_timer = 0U;
        g->reason = DRIFT_LEARN_WAIT_CLEAR;
        return 0U;
    }
    if (!g->clear_timer)
    {
        g->clear_timer = 1U;
        g->clear_tick = tick;
        g->clear_sum = 0.0;
        g->clear_count = 0U;
    }
    g->clear_sum += g->residual_mean;
    g->clear_count++;
    if ((uint32_t)(tick - g->clear_tick) < DRIFT_LEARN_CLEAR_MS)
    {
        g->reason = DRIFT_LEARN_WAIT_CLEAR;
        return 0U;
    }
    if (fabs(g->clear_sum / g->clear_count) > g->noise)
    {
        g->clear_timer = 0U;
        g->reason = DRIFT_LEARN_WAIT_CLEAR;
        return 0U;
    }
    g->allowed = 1U;
    g->reason = 0U;
    return 1U;
}

/* A newly admitted pair waits for later clear observations. Any pause discards
 * the entire pending queue; committed normal samples and the model survive. */
static inline uint8_t DriftLearningGuardTakePair(DriftLearningGuard *g,
        uint32_t tick, uint32_t ch0, uint32_t ch1, uint32_t *admitted0,
        uint32_t *admitted1)
{
    if (!g->allowed) return 0U;
    if (!g->ready) { *admitted0 = ch0; *admitted1 = ch1; return 1U; }
    uint8_t taken = 0U;
    if (g->pending_count)
    {
        DriftPendingPair *oldest = &g->pending[g->pending_head];
        if ((uint32_t)(tick - oldest->tick_ms) >= DRIFT_LEARN_QUARANTINE_MS)
        {
            *admitted0 = oldest->ch0;
            *admitted1 = oldest->ch1;
            g->pending_head = (g->pending_head + 1U) % DRIFT_LEARN_PENDING_DEPTH;
            g->pending_count--;
            taken = 1U;
        }
    }
    if (g->pending_count >= DRIFT_LEARN_PENDING_DEPTH) return 0U;
    uint32_t tail = (g->pending_head + g->pending_count) % DRIFT_LEARN_PENDING_DEPTH;
    g->pending[tail] = (DriftPendingPair){ch0, ch1, tick};
    g->pending_count++;
    return taken;
}

/* Refresh only after a sustained interval of approved learning. The optical
 * clear anchor stays fixed. Without ToF, slow target/drift ambiguity remains. */
static inline void DriftLearningGuardCommit(DriftLearningGuard *g, uint32_t tick,
        int64_t coefficient, int64_t offset)
{
    uint32_t interval = (g->tof_count == DRIFT_LEARN_TOF_SAMPLES) ?
            DRIFT_LEARN_REFERENCE_MS : DRIFT_LEARN_CAP_ONLY_REFERENCE_MS;
    if (g->allowed && (uint32_t)(tick - g->reference_tick) >= interval)
    {
        g->coefficient = coefficient;
        g->offset = offset;
        g->reference_tick = tick;
        g->residual_count = 0U;
        g->residual_head = 0U;
        g->residual_high_timer = 0U;
    }
}

#endif
