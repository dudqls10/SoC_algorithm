#ifndef MOBILE_SOC_DRIFT_GATE_H
#define MOBILE_SOC_DRIFT_GATE_H

#include <stdint.h>
#include <stddef.h>

/* Pilot stationary post-EMA sigma was 43.97 counts over 42 seconds. A 50-count
 * noise floor prevents the short startup window (sigma 17) from understating
 * baseline noise. Revalidate this floor with independent motion measurements. */
#define DRIFT_OBJECT_NOISE_FLOOR_COUNTS 50U

/* The 80-sample startup window already contains real recent measurements.
 * Seed the 10-sample detector from those measurements before enabling it;
 * zeros in its new rolling window otherwise report an object for 100 ms. */
static inline uint8_t DriftSeedDetectionWindow(const int32_t *recent,
        uint64_t *window, int32_t *mean, uint64_t *sum)
{
    if (!recent || !window || !mean || !sum) return 0U;
    for (uint32_t i = 0U; i < 10U; ++i)
        if (recent[i] < 0) return 0U;
    uint64_t total = 0U;
    for (uint32_t i = 0U; i < 10U; ++i)
    {
        window[i] = (uint32_t)recent[i];
        total += window[i];
    }
    *sum = total;
    *mean = (int32_t)(total / 10U);
    return 1U;
}

/* Integration telescopes to anchor - previous compensated count. Its object
 * reference must represent the nominal 4e7 compensated baseline, rather than
 * the incidental EMA level during the short startup fit. */
static inline uint32_t DriftIntegralReference(int64_t integral,
        uint32_t previous_count, uint32_t compensated_baseline)
{
    int64_t reference = integral + (int64_t)previous_count - compensated_baseline;
    if (reference < 0) return 0U;
    if (reference > UINT32_MAX) return UINT32_MAX;
    return (uint32_t)reference;
}

/* idle_fdcQ1 is a legacy name: 1 means an object is detected, so baseline
 * learning must freeze. Use the measured startup reference, not 1e7. */
static inline uint8_t DriftObjectDetected(uint32_t mean, uint32_t reference,
        uint32_t startup_std, uint32_t current_std, uint8_t previous,
        uint8_t reference_ready)
{
    if (reference_ready == 0U) return 0U;
    int64_t delta = (int64_t)mean - (int64_t)reference;
    uint64_t magnitude = (uint64_t)((delta < 0) ? -delta : delta);
    uint32_t baseline_noise = startup_std;
    if (baseline_noise < DRIFT_OBJECT_NOISE_FLOOR_COUNTS)
        baseline_noise = DRIFT_OBJECT_NOISE_FLOOR_COUNTS;
    /* Motion itself increases current_std. Bound that contribution so a large
     * moving object cannot raise its own detection threshold and resume learning. */
    if (current_std > baseline_noise) current_std = baseline_noise;
    uint64_t noise = (uint64_t)baseline_noise + (uint64_t)current_std;
    /* Enter at 3 sigma, release at 1.5 sigma; detect either count polarity.
     * Integer arithmetic keeps the large reference out of float subtraction. */
    return (previous != 0U) ? (uint8_t)(2ULL * magnitude > 3ULL * noise) :
                             (uint8_t)(magnitude >= 3ULL * noise);
}

static inline uint32_t DriftGateSelfTest(void)
{
    uint32_t failures = 0U;
    if (DriftObjectDetected(10000115U,10000115U,40U,10U,0U,1U) != 0U) failures |= 1U;
    if (DriftObjectDetected(10000167U,10000115U,40U,8U,0U,1U) != 0U) failures |= 2U;
    if (DriftObjectDetected(10020115U,10000115U,40U,10U,0U,1U) != 1U) failures |= 4U;
    if (DriftObjectDetected(9980115U,10000115U,40U,10U,0U,1U) != 1U) failures |= 8U;
    if (DriftObjectDetected(10000215U,10000115U,40U,10U,1U,1U) != 1U) failures |= 16U;
    if (DriftObjectDetected(10000165U,10000115U,40U,10U,1U,1U) != 0U) failures |= 32U;
    if (DriftObjectDetected(10020115U,10000115U,0U,0U,0U,0U) != 0U) failures |= 64U;
    if (DriftObjectDetected(0U,UINT32_MAX,UINT32_MAX,UINT32_MAX,0U,1U) != 0U) failures |= 128U;
    if (DriftIntegralReference(10000133LL,39999980U,40000000U) != 10000113U) failures |= 256U;
    if (DriftObjectDetected(10000117U,10000113U,17U,16U,1U,1U) != 0U) failures |= 512U;
    if (DriftObjectDetected(10000243U,10000113U,17U,18U,0U,1U) != 0U) failures |= 1024U;
    if (DriftObjectDetected(10020113U,10000113U,17U,20000U,0U,1U) != 1U) failures |= 2048U;
    if (DriftObjectDetected(9980113U,10000113U,17U,20000U,1U,1U) != 1U) failures |= 4096U;
    if (DriftIntegralReference(0LL,0U,40000000U) != 0U) failures |= 8192U;
    if (DriftIntegralReference(INT64_C(9000000000),UINT32_MAX,0U) != UINT32_MAX) failures |= 16384U;
    int32_t recent[10];
    uint64_t window[10];
    int32_t mean = 0;
    uint64_t sum = 0U;
    for (uint32_t i = 0U; i < 10U; ++i) recent[i] = 10000000;
    if (!DriftSeedDetectionWindow(recent,window,&mean,&sum) || mean != 10000000U ||
        sum != 100000000ULL || window[9] != 10000000ULL ||
        DriftObjectDetected(mean,10000000U,50U,0U,0U,1U) != 0U) failures |= 32768U;
    for (uint32_t i = 0U; i < 10U; ++i) recent[i] = 9999995 + (int32_t)i;
    if (!DriftSeedDetectionWindow(recent,window,&mean,&sum) || mean != 9999999U ||
        sum != 99999995ULL || window[0] != 9999995ULL || window[9] != 10000004ULL)
        failures |= 65536U;
    recent[9] = -1;
    if (DriftSeedDetectionWindow(recent,window,&mean,&sum) || mean != 9999999U ||
        sum != 99999995ULL || window[9] != 10000004ULL) failures |= 131072U;
    if (DriftSeedDetectionWindow(NULL,window,&mean,&sum) ||
        DriftSeedDetectionWindow(recent,NULL,&mean,&sum) ||
        DriftSeedDetectionWindow(recent,window,NULL,&sum) ||
        DriftSeedDetectionWindow(recent,window,&mean,NULL)) failures |= 262144U;
    return failures;
}

#endif
