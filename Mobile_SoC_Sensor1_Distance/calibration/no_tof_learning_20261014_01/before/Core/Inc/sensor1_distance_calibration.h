#ifndef SENSOR1_DISTANCE_CALIBRATION_H
#define SENSOR1_DISTANCE_CALIBRATION_H

#include <math.h>
#include <stdint.h>

/* Sensor1, continuous20mm/s,2026-10-14. See calibration/continuous_20mms_20261014_01. */
#define SENSOR1_TOF_GAIN                0.8978048056331662
#define SENSOR1_TOF_OFFSET_MM           65.22306168323131
#define SENSOR1_TOF_MIN_CALIBRATED_MM    80.0
#define SENSOR1_CAP_BASELINE_COUNT      40000240U
#define SENSOR1_CAP_BASELINE_EXACT      40000240.36548223
#define SENSOR1_CAP_DELTA_SCALE         265070.74048223346
#define SENSOR1_CAP_EXP_A_MM            119.04750937643614
#define SENSOR1_CAP_EXP_B               8.8777187566756
#define SENSOR1_CAP_EXP_C_MM            161.28565358363457
#define SENSOR1_CAP_EXP_D               243.0964649674322
#define SENSOR1_CAP_GAIN                0.980981295523833
#define SENSOR1_CAP_OFFSET_MM           3.880087619661225

static inline uint16_t Sensor1CorrectToFDistance(uint16_t raw_mm, uint8_t quality_valid)
{
    /* Reject invalid measurements before adding a positive calibration offset. */
    if (quality_valid == 0U || raw_mm == 0U || raw_mm > 4000U) return 0U;
    double corrected = SENSOR1_TOF_GAIN * (double)raw_mm + SENSOR1_TOF_OFFSET_MM;
    if (!isfinite(corrected) || corrected < SENSOR1_TOF_MIN_CALIBRATED_MM) return 0U;
    if (corrected > 4000.0) corrected = 4000.0;
    return (uint16_t)round(corrected);
}

static inline uint32_t Sensor1CalibratedCapDistance(uint32_t compensated_count, uint32_t limit_mm)
{
    /* Center in double precision: a float cannot retain individual counts at4e7. */
    const double u = (SENSOR1_CAP_BASELINE_EXACT - (double)compensated_count) /
                     SENSOR1_CAP_DELTA_SCALE;
    double exponent_b = -SENSOR1_CAP_EXP_B * u;
    double exponent_d = -SENSOR1_CAP_EXP_D * u;
    if (exponent_b > 80.0) exponent_b = 80.0;
    if (exponent_d > 80.0) exponent_d = 80.0;
    const double model_mm = SENSOR1_CAP_EXP_A_MM * exp(exponent_b) +
                            SENSOR1_CAP_EXP_C_MM * exp(exponent_d);
    const double corrected_mm = SENSOR1_CAP_GAIN * model_mm + SENSOR1_CAP_OFFSET_MM;
    if (!isfinite(corrected_mm) || corrected_mm >= (double)limit_mm) return limit_mm;
    if (corrected_mm <= 0.0) return 0U;
    return (uint32_t)round(corrected_mm);
}

#endif
