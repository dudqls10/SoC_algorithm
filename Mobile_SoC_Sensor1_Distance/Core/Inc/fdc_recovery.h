#ifndef FDC_RECOVERY_H
#define FDC_RECOVERY_H

#include <stdint.h>

#define FDC_RECOVERY_RETRY_MS       1000U
#define FDC_RECOVERY_IDLE_MS        20U

typedef struct
{
    uint32_t last_attempt_tick;
    uint32_t idle_since_tick;
    uint8_t pending;
    uint8_t idle_tracking;
} FDCRecoverySchedule;

static inline void FDCRecoveryReset(FDCRecoverySchedule *schedule)
{
    schedule->pending = 0U;
    schedule->idle_tracking = 0U;
}

static inline void FDCRecoveryArm(FDCRecoverySchedule *schedule, uint32_t now)
{
    schedule->last_attempt_tick = now;
    schedule->pending = 1U;
    schedule->idle_tracking = 0U;
}

/* Sampled idle qualification; this is not an I2C waveform measurement.
 * Do not issue STARTs while the slave holds SCL or SDA low. */
static inline uint8_t FDCRecoveryDue(FDCRecoverySchedule *schedule,
                                     uint32_t now, uint8_t lines_high)
{
    if (schedule->pending == 0U) FDCRecoveryArm(schedule, now);
    if (lines_high == 0U)
    {
        schedule->idle_tracking = 0U;
        return 0U;
    }
    if (schedule->idle_tracking == 0U)
    {
        schedule->idle_since_tick = now;
        schedule->idle_tracking = 1U;
    }
    if (((uint32_t)(now - schedule->last_attempt_tick) < FDC_RECOVERY_RETRY_MS) ||
        ((uint32_t)(now - schedule->idle_since_tick) < FDC_RECOVERY_IDLE_MS))
    {
        return 0U;
    }
    FDCRecoveryArm(schedule, now);
    return 1U;
}

#endif
