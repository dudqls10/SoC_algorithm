#ifndef SENSOR1_DISTANCE_CALIBRATION_H
#define SENSOR1_DISTANCE_CALIBRATION_H

#include <math.h>
#include <stdint.h>

/* Sensor1 NEW RB10 CSVs,2026-10-18: train1-4/holdout5; all positive ToF range. */
#define SENSOR1_TOF_GAIN                0.96063796973654325
#define SENSOR1_TOF_OFFSET_MM           44.153018037353618
/* Keep measured positive results below80mm; raw zero/quality guards still apply. */
#define SENSOR1_TOF_MIN_CALIBRATED_MM    0.0
#define SENSOR1_CAP_BASELINE_COUNT      39999634U
#define SENSOR1_CAP_BASELINE_EXACT      39999633.5
#define SENSOR1_CAP_DELTA_SCALE         335955.5
#define SENSOR1_CAP_EXP_A_MM            111.4648729719265
#define SENSOR1_CAP_EXP_B               4.4637761132253644
#define SENSOR1_CAP_EXP_C_MM            169.98244287653574
#define SENSOR1_CAP_EXP_D               120.79482656124084
#define SENSOR1_CAP_GAIN                0.9825925881589942
#define SENSOR1_CAP_OFFSET_MM           3.5460405368309456

static inline uint16_t Sensor1CorrectToFDistance(uint16_t raw_mm, uint8_t quality_valid)
{
    /* Reject invalid measurements before adding a positive calibration offset. */
    if (raw_mm == 0U) return 0U;
    double corrected = SENSOR1_TOF_GAIN * (double)raw_mm + SENSOR1_TOF_OFFSET_MM;
    if (!isfinite(corrected) || corrected < SENSOR1_TOF_MIN_CALIBRATED_MM) return 0U;
    if (corrected > 65535.0) corrected = 65535.0;
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

/* Continuous calibrated CAP slope in mm/count, before display rounding
 * and distance limits. Use the same Sensor1 calibration coefficients. */
static inline double Sensor1CalibratedCapDistanceSlope(uint32_t compensated_count)
{
    const double u = (SENSOR1_CAP_BASELINE_EXACT - (double)compensated_count) /
                     SENSOR1_CAP_DELTA_SCALE;
    double exponent_b = -SENSOR1_CAP_EXP_B * u;
    double exponent_d = -SENSOR1_CAP_EXP_D * u;
    if (exponent_b > 80.0) exponent_b = 80.0;
    if (exponent_d > 80.0) exponent_d = 80.0;
    return (SENSOR1_CAP_GAIN / SENSOR1_CAP_DELTA_SCALE) *
           (SENSOR1_CAP_EXP_A_MM * SENSOR1_CAP_EXP_B * exp(exponent_b) +
            SENSOR1_CAP_EXP_C_MM * SENSOR1_CAP_EXP_D * exp(exponent_d));
}

#endif
