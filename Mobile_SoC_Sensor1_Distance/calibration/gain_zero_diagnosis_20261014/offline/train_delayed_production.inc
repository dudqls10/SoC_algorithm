static uint8_t TrainDelayedQ1Pair(uint32_t input0, uint32_t input1, uint32_t tick)
{
    uint32_t sensor[n_samplingLR], reference[n_samplingLR];
    uint64_t sum0 = input0, sum1 = input1;
    sensor[0] = input0;
    reference[0] = input1;
    for (uint32_t i = 1U; i < n_samplingLR; ++i)
    {
        sensor[i] = sample.fdc300[i - 1U];
        reference[i] = sample.fdc302[i - 1U];
        sum0 += sensor[i];
        sum1 += reference[i];
    }
    uint32_t mean0 = (uint32_t)(sum0 / n_samplingLR);
    uint32_t mean1 = (uint32_t)(sum1 / n_samplingLR);
    int64_t numerator = 0, denominator = 0, sensor_variance = 0;
    for (uint32_t i = 0U; i < n_samplingLR; ++i)
    {
        int64_t x = (int64_t)sensor[i] - mean0;
        int64_t y = (int64_t)reference[i] - mean1;
        numerator += x * y;
        denominator += y * y;
        sensor_variance += x * x;
    }
    int64_t coefficient = dComps.fdcCoeff_Q1;
    uint8_t identifiable = DriftFitGainScaled(numerator, denominator, &coefficient);
    int64_t offset = DriftOffsetFromMeans(mean0, coefficient, mean1);
    if (drift_learning_guard.ready)
    {
        double candidate = (double)fdc.ch300 - (double)coefficient * .0001 * fdc.ch302 - offset;
        double applied = (double)fdc.ch300 - (double)dComps.fdcCoeff_Q1 * .0001 * fdc.ch302
                         - dComps.fdcOffset_Q1;
        if (!isfinite(candidate) || fabs(candidate) > 3.0 * drift_learning_guard.noise ||
            fabs(candidate - applied) > 2.0 * drift_learning_guard.noise)
        {
            drift_learning_candidate_rejects++;
            return 0U;
        }
    }
    memcpy(sample.fdc300, sensor, sizeof(sensor));
    memcpy(sample.fdc302, reference, sizeof(reference));
    sample.sum_fdc300 = sum0;
    sample.sum_fdc302 = sum1;
    sample.mean_fdc300 = mean0;
    sample.mean_fdc302 = mean1;
    dComps.sumNumnerator_fdcQ1 = numerator;
    dComps.sumDenominator_fdcQ1 = denominator;
    drift_model_sensor_variance = sensor_variance;
    dComps.fdcCoeff_Q1 = coefficient;
    dComps.fdcOffset_Q1 = offset;
    if (!identifiable) drift_eval_zero_variance_skips++;
    drift_eval_learning_updates++;
    drift_learning_input_ch0 = input0;
    drift_learning_input_ch1 = input1;
    peroid_sampling = 80U;
    flag_di = 1U;
    flag_algorithmDO = 1U;
    if (!drift_learning_guard.ready)
        DriftLearningGuardSeed(&drift_learning_guard, tick, coefficient, offset,
                               sensor, reference, n_samplingLR);
    else
        DriftLearningGuardCommit(&drift_learning_guard, tick, coefficient, offset);
    return 1U;
}
