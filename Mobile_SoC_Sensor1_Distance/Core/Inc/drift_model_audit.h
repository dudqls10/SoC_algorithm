#ifndef MOBILE_SOC_DRIFT_MODEL_AUDIT_H
#define MOBILE_SOC_DRIFT_MODEL_AUDIT_H

#include <stdint.h>

/* Learning events, detection transitions, and a 1 s heartbeat. This provides
 * minutes of history without a debugger attached during motion. Ring overwrite
 * is counted explicitly; this is evaluation RAM, not an extra CAN protocol. */
/* Reserve RAM for guarded 120 s RAW sigma windows; audit overwrite is counted. */
#define DRIFT_MODEL_AUDIT_DEPTH 960U
#define DRIFT_AUDIT_LARGE_STEP_COUNTS 10000U
typedef struct {
    uint32_t sequence;
    uint32_t tick_ms;
    uint32_t filtered_ch0;
    uint32_t filtered_ch1;
    uint32_t pre_ema;
    uint32_t post_ema;
    int64_t numerator;
    int64_t denominator;
    int64_t sensor_variance;
    int64_t offset;
    int32_t coefficient;
    uint32_t learning_updates;
    uint32_t flags; /* valid0/1, object2, startup-ready3, fitted4,
                    * lower-clipped5, upper-clipped6, reference-constant7 */
    uint32_t mean_ch0;
    uint32_t mean_ch1;
    uint32_t object_mean;
    uint32_t object_reference;
    uint32_t drift_us;
    uint32_t model_reset_count;
    uint32_t raw_ch0;
    uint32_t raw_ch1;
    uint32_t kalman_input_ch0;
    uint32_t kalman_input_ch1;
    uint32_t kalman_x0_before_bits;
    uint32_t kalman_p0_before_bits;
    uint32_t kalman_gain0_bits;
    /* Observational large steps: raw0/1 bits0/1, filtered0/1 bits2/3,
     * pre/post EMA bits4/5. Does not reject or modify an input/output. */
    uint32_t large_step_flags;
    /* raw status0:7, HAL config8:9/ch0 10:11/ch1 12:13,
     * reject ch0 16:19/ch1 20:23, most recent transaction error24:31. */
    uint32_t sample_diagnostics;
    /* Schema4: pair actually committed to learning, after quarantine. These
     * differ from the current filtered pair. Entry size is136 on the target.
     * flags8:13=learning pause reasons, bit14=guard-ready, bit15=allowed. */
    uint32_t learning_input_ch0;
    uint32_t learning_input_ch1;
    /* Schema6: RAW rolling sigma, sigma candidate, actual entry
     * threshold and candidate sample count. flags16=RAW sigma-safe,
     * flags17=RAW ready but sigma candidate not safe. Entry size152. */
    uint32_t detection_baseline_sigma;
    uint32_t detection_recent_sigma;
    uint32_t detection_entry_threshold;
    uint32_t noise_reference_samples;
} DriftModelAuditEntry;

#endif
