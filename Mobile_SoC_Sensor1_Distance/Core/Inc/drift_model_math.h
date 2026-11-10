#ifndef MOBILE_SOC_DRIFT_MODEL_MATH_H
#define MOBILE_SOC_DRIFT_MODEL_MATH_H

#include <stdint.h>
#include <stddef.h>

/* Preserve the current 1e-4 coefficient units and validated live limits.
 * A constant reference cannot identify a new gain: retain the previous gain. */
static inline uint8_t DriftFitGainScaled(int64_t numerator, int64_t denominator,
        int64_t *coefficient)
{
    if ((coefficient == NULL) || (denominator <= 0)) return 0U;
    double fitted = ((double)numerator / (double)denominator) * 10000.0;
    if (fitted < 0.0) fitted = 0.0;
    if (fitted > 9999.0) fitted = 9999.0;
    *coefficient = (int64_t)fitted;
    return 1U;
}

/* The offset must follow the applied gain at the same reference means.
 * Independent absolute clipping/rate limiting of this offset moves the anchor. */
static inline int64_t DriftOffsetFromMeans(uint64_t sensor_mean,
        int64_t coefficient, uint64_t reference_mean)
{
    return (int64_t)((double)sensor_mean -
            ((double)coefficient * 0.0001) * (double)reference_mean);
}

static inline int64_t DriftCompensatedResidual(uint32_t sensor,
        uint32_t reference, double gain, int64_t offset)
{
    return (int64_t)((double)sensor - gain * (double)reference - (double)offset);
}

static inline uint32_t DriftModelMathSelfTest(void)
{
    uint32_t failures = 0U;
    int64_t coefficient = 4000;
    if (!DriftFitGainScaled(-8,20,&coefficient) || coefficient != 0) failures |= 1U;
    if (!DriftFitGainScaled(8,20,&coefficient) || coefficient != 4000) failures |= 2U;
    if (!DriftFitGainScaled(28,20,&coefficient) || coefficient != 9999) failures |= 4U;
    if (!DriftFitGainScaled(0,20,&coefficient) || coefficient != 0) failures |= 8U;
    if (!DriftFitGainScaled(1,10000,&coefficient) || coefficient != 1) failures |= 16U;
    coefficient = 4000;
    if (DriftFitGainScaled(8,0,&coefficient) || coefficient != 4000) failures |= 32U;
    if (DriftFitGainScaled(8,-1,&coefficient) || coefficient != 4000) failures |= 64U;
    if (DriftFitGainScaled(8,20,NULL)) failures |= 128U;
    if (!DriftFitGainScaled(INT64_MAX,1,&coefficient) || coefficient != 9999) failures |= 256U;
    if (!DriftFitGainScaled(INT64_MIN,1,&coefficient) || coefficient != 0) failures |= 512U;
    if (DriftOffsetFromMeans(8000000U,4000,10000000U) != 4000000) failures |= 1024U;
    if (DriftOffsetFromMeans(8000000U,0,10000000U) != 8000000) failures |= 2048U;
    if (DriftCompensatedResidual(8000000U,10000000U,0.4,4000000) != 0) failures |= 4096U;
    if (DriftCompensatedResidual(7999999U,10000000U,0.4,4000000) != -1) failures |= 8192U;
    if (DriftCompensatedResidual(10U,7U,0.4,0) != 7) failures |= 16384U;
    if (DriftCompensatedResidual(1U,7U,0.4,0) != -1) failures |= 32768U;
    if (DriftCompensatedResidual(10000U,10000U,0.9999,0) != 1) failures |= 65536U;
    if (DriftCompensatedResidual(0xFFFFFFU,0U,0.9999,0xFFFFFF) != 0) failures |= 131072U;
    /* Retain a gain at zero variance while allowing a trusted idle baseline
     * to recenter through its offset. An object-gated update remains forbidden. */
    coefficient = 4000;
    (void)DriftFitGainScaled(0,0,&coefficient);
    int64_t offset = DriftOffsetFromMeans(8000100U,coefficient,10000000U);
    if (coefficient != 4000 || offset != 4000100 ||
        DriftCompensatedResidual(8000100U,10000000U,0.4,offset) != 0) failures |= 262144U;
    return failures;
}

#endif
