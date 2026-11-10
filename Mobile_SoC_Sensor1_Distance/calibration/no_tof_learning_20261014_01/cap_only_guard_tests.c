#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "drift_learning_guard.h"
#include "drift_noise_reference.h"

int main(void)
{
    unsigned failures = 0;
    DriftLearningGuard g;
    DriftNoiseReference n;
    DriftLearningGuardReset(&g);
    DriftNoiseReferenceReset(&n);
    g.tof_count = 16U; g.tof_near = 1U;
    if (!DriftLearningGuardStepCapOnly(&g, 0U, 8000000U, 10000000U, 1U, 0U))
        failures |= 1U;
    g.coefficient = 5000; g.offset = 3000000; g.ready = 1U;
    DriftLearningGuardPause(&g, DRIFT_LEARN_WAIT_CLEAR);
    uint32_t first_admit = UINT32_MAX;
    for (uint32_t tick = 10U; tick <= 18000U; tick += 10U)
    {
        DriftLearningGuardStepCapOnly(&g, tick, 8000000U, 10000000U, 1U, 0U);
        DriftNoiseReferenceStep(&n, tick, 40000000U,
            (uint8_t)(g.ready && g.allowed && g.reason == 0U));
        if (g.reason & (DRIFT_LEARN_TOF_NEAR | DRIFT_LEARN_TOF_UNTRUSTED)) failures |= 2U;
        if (tick % 800U == 0U && DriftNoiseReferenceAllowsLearning(&n, g.ready))
        {
            uint32_t a = 0, b = 0;
            if (DriftLearningGuardTakePair(&g, tick, 8000000U, 10000000U, &a, &b)
                    && first_admit == UINT32_MAX) first_admit = tick;
        }
    }
    if (!n.ready || first_admit > 12000U || g.tof_count || g.tof_near) failures |= 4U;
    DriftLearningGuardStepCapOnly(&g, 18010U, 8000000U, 10000000U, 1U, 1U);
    if (g.allowed || !(g.reason & DRIFT_LEARN_OBJECT) || g.pending_count) failures |= 8U;
    DriftLearningGuardStepCapOnly(&g, 18020U, 8000000U, 10000000U, 0U, 0U);
    if (g.allowed || g.reason != DRIFT_LEARN_INVALID) failures |= 16U;
    for (uint32_t tick = 18030U; tick <= 18530U; tick += 10U)
        DriftLearningGuardStepCapOnly(&g, tick, 8000200U, 10000000U, 1U, 0U);
    if (g.allowed || !(g.reason & DRIFT_LEARN_RESIDUAL)) failures |= 32U;
    printf("{\"failure_bitmask\":%u,\"noise_ready_without_ToF\":%u,"
           "\"first_admit_ms\":%u,\"object_invalid_residual_protection_checked\":true}\n",
           failures, n.ready, first_admit);
    return failures ? 1 : 0;
}
