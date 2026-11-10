#ifndef DRIFT_APPROACH_GATE_H
#define DRIFT_APPROACH_GATE_H

#include <math.h>
#include <stdint.h>
#include <string.h>

/* DriftApproach only: guarded rolling120_confirm1_weak1 virtual candidate.
 * RAW median3 -> 0.5 s / 0.1 s means. No physical count floor and no ToF.
 * Startup assumes a stationary reference; subsequent accepted observations
 * are only stationary candidates, not independently verified ground truth. */
#define DRIFT_APPROACH_WINDOW 50U
#define DRIFT_APPROACH_FAST_WINDOW 10U
#define DRIFT_APPROACH_SETTLE_MS 2000U
#define DRIFT_APPROACH_CAL_BLOCKS 40U
#define DRIFT_APPROACH_MAX_GAP_MS 50U
#define DRIFT_APPROACH_NUMERIC_EPSILON 1.0e-6
#define DRIFT_APPROACH_SUSPECT_MS 1000U
#define DRIFT_APPROACH_CONFIRM_MS 1000U
#define DRIFT_APPROACH_FAST_CONFIRM_MS 100U
#define DRIFT_APPROACH_RELEASE_MS 500U
#define DRIFT_APPROACH_CLEAR_MS 1000U
#define DRIFT_APPROACH_QUARANTINE_MS 2400U
#define DRIFT_APPROACH_PENDING 16U
#define DRIFT_APPROACH_SIGMA_WINDOW_MS 120000U
#define DRIFT_APPROACH_CONTEXT_MS 2000U
#define DRIFT_APPROACH_SHIFT_MS 500U
#define DRIFT_APPROACH_SIGMA_SLOW_CAPACITY 242U
#define DRIFT_APPROACH_SIGMA_FAST_CAPACITY 1202U
#define DRIFT_APPROACH_CONTEXT_CAPACITY 202U
#define DRIFT_APPROACH_NOISE_PENDING 32U
#define DRIFT_APPROACH_SIGMA_MIN_SAMPLES 20U

enum { DRIFT_APPROACH_CALIBRATING, DRIFT_APPROACH_CLEAR, DRIFT_APPROACH_SUSPECT,
       DRIFT_APPROACH_OBJECT, DRIFT_APPROACH_RECOVERY };

typedef struct { uint32_t ch0, ch1, tick; } DriftApproachPendingPair;
typedef struct { double value; uint32_t tick; } DriftApproachNoiseSample;
typedef struct { double sum, sumsq; uint16_t head, count; } DriftApproachNoiseWindow;
typedef struct { double value; uint32_t tick; uint8_t fast; } DriftApproachPendingNoise;
typedef struct {
    uint32_t window[DRIFT_APPROACH_WINDOW];
    uint32_t raw_history[3];
    uint64_t sum;
    uint32_t last_tick, step_dt, settle_tick, weak_tick, strong_tick, fast_tick;
    uint32_t release_tick, clear_tick;
    double mean, fast_mean, baseline, sigma, fast_sigma, delta, fast_delta;
    double cal_mean, cal_m2, fast_cal_mean, fast_cal_m2;
    uint16_t head, count, cal_blocks, fast_cal_blocks, cal_samples;
    uint8_t ready, object, phase, learning_allowed, sample_valid, have_tick;
    uint8_t settling, weak_timer, strong_timer, fast_timer, release_timer, clear_timer;
    uint8_t suspicion_confirmed;
    DriftApproachPendingPair pending[DRIFT_APPROACH_PENDING];
    uint8_t pending_head, pending_count;
    uint8_t raw_history_head, raw_history_count;
    DriftApproachNoiseSample slow_noise[DRIFT_APPROACH_SIGMA_SLOW_CAPACITY];
    DriftApproachNoiseSample fast_noise[DRIFT_APPROACH_SIGMA_FAST_CAPACITY];
    DriftApproachNoiseSample context[DRIFT_APPROACH_CONTEXT_CAPACITY];
    DriftApproachNoiseWindow slow_stats, fast_stats, context_stats;
    DriftApproachPendingNoise noise_pending[DRIFT_APPROACH_NOISE_PENDING];
    double noise_center, sigma_candidate;
    uint32_t shift_tick, sigma_observations, sigma_rejected;
    uint16_t sigma_samples;
    uint8_t noise_head, noise_count, shift_timer, mean_shift, sigma_safe;
} DriftApproachGate;

static inline void DriftApproachGateReset(DriftApproachGate *g)
{
    memset(g, 0, sizeof(*g));
    g->sigma = g->fast_sigma = DRIFT_APPROACH_NUMERIC_EPSILON;
}

static inline void DriftApproachNoisePop(DriftApproachNoiseWindow *s,
        DriftApproachNoiseSample *v, uint16_t capacity)
{
    const double d = v[s->head].value;
    s->sum -= d; s->sumsq -= d * d;
    s->head = (s->head + 1U) % capacity;
    if (--s->count == 0U) s->sum = s->sumsq = 0.0;
}

static inline void DriftApproachNoiseExpire(DriftApproachNoiseWindow *s,
        DriftApproachNoiseSample *v, uint16_t capacity, uint32_t tick, uint32_t width)
{
    while (s->count && (uint32_t)(tick - v[s->head].tick) > width)
        DriftApproachNoisePop(s, v, capacity);
}

static inline void DriftApproachNoiseAdd(DriftApproachNoiseWindow *s,
        DriftApproachNoiseSample *v, uint16_t capacity, uint32_t tick,
        double value, uint32_t width)
{
    DriftApproachNoiseExpire(s, v, capacity, tick, width);
    if (s->count == capacity) DriftApproachNoisePop(s, v, capacity);
    uint16_t tail = (s->head + s->count) % capacity;
    v[tail] = (DriftApproachNoiseSample){value, tick};
    s->count++; s->sum += value; s->sumsq += value * value;
}

static inline double DriftApproachNoiseStd(const DriftApproachNoiseWindow *s)
{
    const double mean = s->sum / s->count;
    return sqrt(fmax(0.0, s->sumsq / s->count - mean * mean));
}

static inline void DriftApproachNoiseDiscardPending(DriftApproachGate *g)
{
    g->sigma_rejected += g->noise_count;
    g->noise_head = g->noise_count = 0U;
}

static inline void DriftApproachGateBlockLearning(DriftApproachGate *g)
{
    g->learning_allowed = 0U;
    g->clear_timer = 0U;
    g->pending_count = g->pending_head = 0U;
}

static inline void DriftApproachGateBreakContinuity(DriftApproachGate *g)
{
    DriftApproachGateBlockLearning(g);
    g->sum = 0U;
    g->count = g->head = 0U;
    g->raw_history_count = g->raw_history_head = 0U;
    g->weak_timer = g->strong_timer = g->fast_timer = g->release_timer = 0U;
    g->suspicion_confirmed = 0U;
    DriftApproachNoiseDiscardPending(g);
    memset(&g->context_stats, 0, sizeof(g->context_stats));
    g->shift_timer = g->mean_shift = g->sigma_safe = 0U;
    g->sigma_samples = 0U;
    if (!g->ready) {
        g->settling = 0U;
        g->cal_blocks = g->fast_cal_blocks = g->cal_samples = 0U;
        g->cal_mean = g->cal_m2 = g->fast_cal_mean = g->fast_cal_m2 = 0.0;
        memset(&g->slow_stats, 0, sizeof(g->slow_stats));
        memset(&g->fast_stats, 0, sizeof(g->fast_stats));
    }
    g->phase = g->ready ? (g->object ? DRIFT_APPROACH_OBJECT : DRIFT_APPROACH_RECOVERY)
                       : DRIFT_APPROACH_CALIBRATING;
    /* Keep a confirmed object and the accepted reference across bad reads. */
}

static inline uint8_t DriftApproachGateHeld(uint8_t *active, uint32_t *since,
        uint32_t tick, uint8_t condition, uint32_t duration)
{
    if (!condition) { *active = 0U; return 0U; }
    if (!*active) { *active = 1U; *since = tick; }
    return (uint8_t)((uint32_t)(tick - *since) >= duration);
}

static inline void DriftApproachNoiseQueue(DriftApproachGate *g, uint32_t tick,
        double mean, uint8_t fast)
{
    if (g->noise_count == DRIFT_APPROACH_NOISE_PENDING) {
        /* Fail closed if the admission schedule is ever changed. */
        DriftApproachNoiseDiscardPending(g);
        return;
    }
    uint8_t tail = (g->noise_head + g->noise_count) % DRIFT_APPROACH_NOISE_PENDING;
    g->noise_pending[tail] = (DriftApproachPendingNoise){mean - g->noise_center, tick, fast};
    g->noise_count++;
}

static inline void DriftApproachGateUpdateSigma(DriftApproachGate *g, uint32_t tick)
{
    DriftApproachNoiseAdd(&g->context_stats, g->context, DRIFT_APPROACH_CONTEXT_CAPACITY,
            tick, g->mean - g->noise_center, DRIFT_APPROACH_CONTEXT_MS);
    const double local_mean = g->noise_center + g->context_stats.sum / g->context_stats.count;
    g->mean_shift = DriftApproachGateHeld(&g->shift_timer, &g->shift_tick, tick,
            (uint8_t)(fabs(local_mean - g->baseline) >= 2.0 * g->sigma), DRIFT_APPROACH_SHIFT_MS);
    g->sigma_safe = (uint8_t)(!g->mean_shift && !g->object &&
            g->delta < 3.0 * g->sigma && g->fast_delta < 6.0 * g->fast_sigma);
    if (!g->sigma_safe) DriftApproachNoiseDiscardPending(g);

    /* Candidate reference recovery is independent of learning permission.
     * The 2count/s cap and strong/fast gate protect the measured approaches. */
    if (!g->object && g->fast_delta < 6.0 * g->fast_sigma &&
            fabs(g->fast_mean - g->mean) <= 3.0 * g->fast_sigma) {
        uint32_t dt = g->step_dt;
        double change = (g->mean - g->baseline) * dt / (10000.0 + dt);
        double limit = 2.0 * dt / 1000.0;
        g->baseline += fmax(-limit, fmin(limit, change));
    }

    g->sigma_samples = (g->sigma_samples + 1U) % DRIFT_APPROACH_WINDOW;
    uint8_t fast_due = (uint8_t)(g->sigma_samples % DRIFT_APPROACH_FAST_WINDOW == 0U);
    uint8_t slow_due = (uint8_t)(g->sigma_samples == 0U);
    if (!g->sigma_safe) return;
    if (fast_due) DriftApproachNoiseQueue(g, tick, g->fast_mean, 1U);
    if (slow_due) DriftApproachNoiseQueue(g, tick, g->mean, 0U);
    while (g->noise_count) {
        DriftApproachPendingNoise *p = &g->noise_pending[g->noise_head];
        if ((uint32_t)(tick - p->tick) < DRIFT_APPROACH_QUARANTINE_MS) break;
        DriftApproachNoiseAdd(p->fast ? &g->fast_stats : &g->slow_stats,
                p->fast ? g->fast_noise : g->slow_noise,
                p->fast ? DRIFT_APPROACH_SIGMA_FAST_CAPACITY : DRIFT_APPROACH_SIGMA_SLOW_CAPACITY,
                p->tick, p->value, DRIFT_APPROACH_SIGMA_WINDOW_MS);
        g->sigma_observations++;
        g->noise_head = (g->noise_head + 1U) % DRIFT_APPROACH_NOISE_PENDING;
        g->noise_count--;
    }
    if (slow_due) {
        DriftApproachNoiseExpire(&g->slow_stats, g->slow_noise, DRIFT_APPROACH_SIGMA_SLOW_CAPACITY,
                tick, DRIFT_APPROACH_SIGMA_WINDOW_MS);
        if (g->slow_stats.count >= DRIFT_APPROACH_SIGMA_MIN_SAMPLES) {
            g->sigma_candidate = DriftApproachNoiseStd(&g->slow_stats);
            g->sigma = fmax(DRIFT_APPROACH_NUMERIC_EPSILON,
                    g->sigma + (g->sigma_candidate - g->sigma) * 0.5 / 5.5);
        }
    }
    if (fast_due) {
        DriftApproachNoiseExpire(&g->fast_stats, g->fast_noise, DRIFT_APPROACH_SIGMA_FAST_CAPACITY,
                tick, DRIFT_APPROACH_SIGMA_WINDOW_MS);
        if (g->fast_stats.count >= DRIFT_APPROACH_SIGMA_MIN_SAMPLES)
            g->fast_sigma = fmax(DRIFT_APPROACH_NUMERIC_EPSILON,
                    g->fast_sigma + (DriftApproachNoiseStd(&g->fast_stats) - g->fast_sigma) * 0.1 / 5.1);
    }
}

static inline void DriftApproachGateStep(DriftApproachGate *g, uint32_t tick,
        uint32_t raw, uint8_t valid)
{
    uint32_t dt = g->have_tick ? (uint32_t)(tick - g->last_tick) : 10U;
    g->step_dt = dt;
    g->sample_valid = (uint8_t)(valid && raw != 0U && raw < 0xFFFFFFU);
    if (!g->sample_valid || (g->have_tick && dt > DRIFT_APPROACH_MAX_GAP_MS))
        DriftApproachGateBreakContinuity(g);
    g->have_tick = 1U;
    g->last_tick = tick;
    if (!g->sample_valid) return;

    /* A single bus/sample spike must not be stretched into a 0.5 s event by
     * the mean. This detector-only median leaves CAN RAW and regression intact. */
    g->raw_history[g->raw_history_head] = raw;
    g->raw_history_head = (g->raw_history_head + 1U) % 3U;
    if (g->raw_history_count < 3U) g->raw_history_count++;
    if (g->raw_history_count < 3U) { DriftApproachGateBlockLearning(g); return; }
    uint32_t a = g->raw_history[0], b = g->raw_history[1], c = g->raw_history[2];
    if (a > b) { uint32_t t = a; a = b; b = t; }
    if (b > c) { uint32_t t = b; b = c; c = t; }
    if (a > b) b = a;
    raw = b;

    if (g->count == DRIFT_APPROACH_WINDOW) g->sum -= g->window[g->head];
    else g->count++;
    g->window[g->head] = raw;
    g->sum += raw;
    g->head = (g->head + 1U) % DRIFT_APPROACH_WINDOW;
    if (g->count < DRIFT_APPROACH_WINDOW) { DriftApproachGateBlockLearning(g); return; }
    g->mean = (double)g->sum / DRIFT_APPROACH_WINDOW;
    uint64_t fast_sum = 0U;
    for (uint32_t i = 0U; i < DRIFT_APPROACH_FAST_WINDOW; ++i)
        fast_sum += g->window[(g->head + DRIFT_APPROACH_WINDOW - 1U - i) % DRIFT_APPROACH_WINDOW];
    g->fast_mean = (double)fast_sum / DRIFT_APPROACH_FAST_WINDOW;

    if (!g->ready) {
        DriftApproachGateBlockLearning(g);
        if (!g->settling) { g->settling = 1U; g->settle_tick = tick; }
        if ((uint32_t)(tick - g->settle_tick) < DRIFT_APPROACH_SETTLE_MS) return;
        g->cal_samples++;
        if (g->cal_samples == 1U) g->noise_center = g->mean;
        if (g->cal_samples % DRIFT_APPROACH_FAST_WINDOW == 0U) {
            g->fast_cal_blocks++;
            double d = g->fast_mean - g->fast_cal_mean;
            g->fast_cal_mean += d / g->fast_cal_blocks;
            g->fast_cal_m2 += d * (g->fast_mean - g->fast_cal_mean);
            DriftApproachNoiseAdd(&g->fast_stats, g->fast_noise, DRIFT_APPROACH_SIGMA_FAST_CAPACITY,
                    tick, g->fast_mean - g->noise_center, DRIFT_APPROACH_SIGMA_WINDOW_MS);
        }
        if (g->cal_samples % DRIFT_APPROACH_WINDOW != 0U) return;
        g->cal_blocks++;
        double d = g->mean - g->cal_mean;
        g->cal_mean += d / g->cal_blocks;
        g->cal_m2 += d * (g->mean - g->cal_mean);
        DriftApproachNoiseAdd(&g->slow_stats, g->slow_noise, DRIFT_APPROACH_SIGMA_SLOW_CAPACITY,
                tick, g->mean - g->noise_center, DRIFT_APPROACH_SIGMA_WINDOW_MS);
        if (g->cal_blocks < DRIFT_APPROACH_CAL_BLOCKS) return;
        g->baseline = g->cal_mean;
        g->sigma = fmax(DRIFT_APPROACH_NUMERIC_EPSILON,
                sqrt(g->cal_m2 / g->cal_blocks));
        g->fast_sigma = fmax(DRIFT_APPROACH_NUMERIC_EPSILON,
                sqrt(g->fast_cal_m2 / g->fast_cal_blocks));
        g->ready = 1U;
        g->phase = DRIFT_APPROACH_CLEAR;
        g->sigma_candidate = g->sigma;
    }

    g->delta = g->baseline - g->mean;
    g->fast_delta = g->baseline - g->fast_mean;
    const uint8_t weak = (uint8_t)(g->delta >= 2.0 * g->sigma);
    const uint8_t strong = (uint8_t)(g->delta >= 3.0 * g->sigma);
    const uint8_t fast = (uint8_t)(g->fast_delta >= 6.0 * g->fast_sigma);
    g->suspicion_confirmed = DriftApproachGateHeld(&g->weak_timer, &g->weak_tick,
            tick, weak, DRIFT_APPROACH_SUSPECT_MS);
    uint8_t confirmed = DriftApproachGateHeld(&g->strong_timer, &g->strong_tick,
            tick, strong, DRIFT_APPROACH_CONFIRM_MS);
    confirmed |= DriftApproachGateHeld(&g->fast_timer, &g->fast_tick,
            tick, fast, DRIFT_APPROACH_FAST_CONFIRM_MS);
    if (g->suspicion_confirmed || fast || confirmed) {
        DriftApproachGateBlockLearning(g);
        g->release_timer = 0U;
        if (confirmed) g->object = 1U;
        g->phase = g->object ? DRIFT_APPROACH_OBJECT : DRIFT_APPROACH_SUSPECT;
    } else {
        if (g->phase != DRIFT_APPROACH_CLEAR) {
            DriftApproachGateBlockLearning(g);
            uint8_t released = DriftApproachGateHeld(&g->release_timer, &g->release_tick,
                    tick, (uint8_t)(g->delta <= 2.0 * g->sigma), DRIFT_APPROACH_RELEASE_MS);
            if (!released) g->phase = DRIFT_APPROACH_RECOVERY;
            else {
                g->object = 0U;
                g->phase = DRIFT_APPROACH_CLEAR;
                g->release_timer = 0U;
            }
        }
        if (g->phase == DRIFT_APPROACH_CLEAR) {
            if (!g->clear_timer) { g->clear_timer = 1U; g->clear_tick = tick; }
            g->learning_allowed = (uint8_t)((uint32_t)(tick - g->clear_tick) >= DRIFT_APPROACH_CLEAR_MS);
        }
    }
    DriftApproachGateUpdateSigma(g, tick);
}

/* Store filtered regression pairs without changing their values. They are
 * admitted only after a further 2.4 s of uninterrupted approved clear data. */
static inline uint8_t DriftApproachGateTakePair(DriftApproachGate *g, uint32_t tick,
        uint32_t ch0, uint32_t ch1, uint32_t *out0, uint32_t *out1)
{
    if (!g->learning_allowed || !g->sample_valid) {
        g->pending_count = g->pending_head = 0U;
        return 0U;
    }
    uint8_t taken = 0U;
    if (g->pending_count) {
        DriftApproachPendingPair *p = &g->pending[g->pending_head];
        if ((uint32_t)(tick - p->tick) >= DRIFT_APPROACH_QUARANTINE_MS) {
            *out0 = p->ch0; *out1 = p->ch1;
            g->pending_head = (g->pending_head + 1U) % DRIFT_APPROACH_PENDING;
            g->pending_count--;
            taken = 1U;
        }
    }
    if (g->pending_count < DRIFT_APPROACH_PENDING) {
        uint32_t tail = (g->pending_head + g->pending_count) % DRIFT_APPROACH_PENDING;
        g->pending[tail] = (DriftApproachPendingPair){ch0, ch1, tick};
        g->pending_count++;
    }
    return taken;
}

/* Fixed 100 Hz detector cadence preserves the 0.5/0.1 s windows even
 * when acquisition/CAN run at 500 Hz. Invalid reads always break continuity. */
static inline void DriftApproachGateStepTimed(DriftApproachGate *g,
        uint32_t tick, uint32_t raw, uint8_t valid)
{
    if (!valid || raw == 0U || raw >= 0xFFFFFFU || !g->have_tick ||
        (uint32_t)(tick - g->last_tick) >= 10U)
        DriftApproachGateStep(g, tick, raw, valid);
}

#endif
