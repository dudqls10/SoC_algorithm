#ifndef MOBILE_SOC_EMA_PRECISION_H
#define MOBILE_SOC_EMA_PRECISION_H

#include <stdint.h>
#include <math.h>

/* Preserve the fractional state. Rounding the state at every iteration creates
 * a dead band of several counts, amplified by a slow alpha at the 4e7 baseline. */
static inline uint32_t EMA2PrecisionStep(double *accumulator, uint32_t input,
        double delta_low, double delta_high, double alpha_slow,
        double alpha_fast, float *selected_alpha)
{
    double delta = (double)input - *accumulator;
    double magnitude = fabs(delta);
    double alpha = alpha_slow;
    if (magnitude >= delta_high) alpha = alpha_fast;
    else if (magnitude > delta_low)
        alpha += (magnitude - delta_low) * (alpha_fast - alpha_slow) /
                 (delta_high - delta_low);
    *accumulator += alpha * delta;
    if (!isfinite(*accumulator) || *accumulator < 0.0) *accumulator = 0.0;
    if (*accumulator > 4294967295.0) *accumulator = 4294967295.0;
    *selected_alpha = (float)alpha;
    return (uint32_t)(*accumulator + 0.5);
}

static inline uint32_t EMA2PrecisionSelfTest(void)
{
    uint32_t failures = 0U;
    const uint32_t targets[] = {39999900U,39999999U,40000001U,40000100U};
    for (uint32_t j = 0U; j < 4U; ++j)
    {
        double value = 40000000.0;
        float alpha = 0.0f;
        uint32_t output = 40000000U;
        for (uint32_t i = 0U; i < 1000U; ++i)
            output = EMA2PrecisionStep(&value, targets[j], 5000.0,42000.0,
                                      0.04,0.45,&alpha);
        if (output != targets[j]) failures |= 1U << j;
    }
    double value = 40000000.0;
    float alpha = 0.0f;
    uint32_t output = EMA2PrecisionStep(&value, 39900000U,
                                      5000.0,42000.0,0.04,0.45,&alpha);
    if (output != 39955000U || fabsf(alpha - 0.45f) > 1.0e-6f) failures |= 16U;
    value = 40000000.0;
    EMA2PrecisionStep(&value,39976500U,5000.0,42000.0,0.04,0.45,&alpha);
    if (fabsf(alpha - 0.245f) > 1.0e-6f) failures |= 32U;
    value = 0.0;
    output = EMA2PrecisionStep(&value,0U,5000.0,42000.0,0.04,0.45,&alpha);
    if (output != 0U || !isfinite(value)) failures |= 64U;
    value = 4294967295.0;
    output = EMA2PrecisionStep(&value,UINT32_MAX,5000.0,42000.0,0.04,0.45,&alpha);
    if (output != UINT32_MAX) failures |= 128U;
    return failures;
}

/* Diagnostic reproduction of the former filter, never used for sensor output. */
static inline int32_t EMA2LegacyConstantBias(void)
{
    uint32_t previous = 40000000U;
    volatile float alpha = 0.04f;
    for (uint32_t i = 0U; i < 1000U; ++i)
        previous = (uint32_t)((1.0f-alpha)*(float)previous +
                              alpha*(float)40000100U);
    return (int32_t)previous - 40000100;
}

#endif
