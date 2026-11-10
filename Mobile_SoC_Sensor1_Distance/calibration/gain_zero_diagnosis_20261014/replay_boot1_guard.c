/* Recorded first boot input replay. ToF quality is assumed valid over the
 * selected valid-pair interval. Fixed-ToF cases are counterfactuals only.
 * Object flags remain the measured detector output; models are held fixed. */
#include <stdio.h>
#include <stdint.h>
#include <inttypes.h>
#include <string.h>
#include "drift_learning_guard.h"
#include "drift_noise_reference.h"

typedef struct {
    DriftLearningGuard guard;
    DriftNoiseReference noise;
    uint32_t first_noise_ready;
    uint32_t near_cycles, residual_cycles, object_cycles, allowed_cycles;
    uint32_t peak_noise_samples;
} Case;

int main(int argc, char **argv) {
    if (argc != 2) return 2;
    FILE *input = NULL;
    if (fopen_s(&input, argv[1], "r") || !input) return 3;
    Case cases[4];
    memset(cases, 0, sizeof(cases));
    for (unsigned i = 0; i < 4; ++i) {
        DriftLearningGuardReset(&cases[i].guard);
        DriftNoiseReferenceReset(&cases[i].noise);
        cases[i].guard.coefficient = 112;
        cases[i].guard.offset = 8164207;
        cases[i].guard.noise = 117.19532066039415;
        cases[i].guard.tof_reference = 296.3125;
        cases[i].guard.tof_noise = 1.2103072956898178;
        cases[i].guard.tof_count = 16;
        cases[i].guard.ready = 1;
        cases[i].guard.reference_tick = 22782;
        DriftLearningGuardPause(&cases[i].guard, DRIFT_LEARN_WAIT_CLEAR);
    }
    uint32_t tick, ch0, ch1, previous_post, object, tof;
    uint32_t records = 0;
    while (fscanf_s(input, "%u,%u,%u,%u,%u,%u", &tick, &ch0, &ch1,
                   &previous_post, &object, &tof) == 6) {
        ++records;
        for (unsigned i = 0; i < 4; ++i) {
            Case *c = &cases[i];
            uint8_t detected = (uint8_t)((i & 2U) ? 0U : object);
            uint16_t optical = (uint16_t)((i & 1U) ? 296U : tof);
            DriftLearningGuardStep(&c->guard, tick, ch0, ch1, 1, detected,
                                   tick / 100U, optical, 1);
            uint8_t clear = (uint8_t)(c->guard.ready && c->guard.allowed &&
                    c->guard.reason == 0U && !detected);
            DriftNoiseReferenceStep(&c->noise, tick, previous_post, clear);
            if (c->noise.ready && !c->first_noise_ready)
                c->first_noise_ready = tick;
            if (c->noise.sample_count > c->peak_noise_samples)
                c->peak_noise_samples = c->noise.sample_count;
            c->near_cycles += (c->guard.reason & DRIFT_LEARN_TOF_NEAR) != 0;
            c->residual_cycles += (c->guard.reason & DRIFT_LEARN_RESIDUAL) != 0;
            c->object_cycles += (c->guard.reason & DRIFT_LEARN_OBJECT) != 0;
            c->allowed_cycles += c->guard.allowed != 0;
        }
    }
    fclose(input);
    const char *labels[4] = {"recorded_inputs", "fixed_ToF_only_counterfactual",
        "object_zero_only_counterfactual", "fixed_ToF_and_object_zero_counterfactual"};
    for (unsigned i = 0; i < 4; ++i) {
        Case *c = &cases[i];
        printf("{\"case\":\"%s\",\"records\":%u,\"noise_ready\":%u,"
            "\"first_noise_ready_tick\":%u,\"peak_noise_samples\":%u,"
            "\"near_cycles\":%u,\"residual_cycles\":%u,"
            "\"object_cycles\":%u,\"allowed_cycles\":%u}\n",
            labels[i], records, c->noise.ready, c->first_noise_ready,
            c->peak_noise_samples, c->near_cycles, c->residual_cycles,
            c->object_cycles, c->allowed_cycles);
    }
    return 0;
}
