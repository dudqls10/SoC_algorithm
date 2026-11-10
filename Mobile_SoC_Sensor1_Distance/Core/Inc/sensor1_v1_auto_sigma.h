#ifndef SENSOR1_V1_AUTO_SIGMA_H
#define SENSOR1_V1_AUTO_SIGMA_H
#include <stdint.h>
/* Tested v1 Q1 candidate: absolute 4e7 detector anchor; guarded recent quiet sigma + 3sigma.
 * Counts, filter timing, LR periods are retained; initial forced-idle0 guard is zero.
 * Step computes compensation; main applies Adaptive EMA2 once and commits
 * that final output before the next detector/quiet-sigma update. */
typedef struct {
 uint32_t object,allowed,source_event,accepted_pairs,n_data,guard,do_ready,period;
 double gain,offset,output,pre,delta,threshold,init_sigma,recent_sigma,filtered0,filtered1;
 int64_t denominator;
 uint32_t final_count,pre_count,di_ready,algorithm_ready,do_count,sampling_count;
 uint32_t filter_input0,filter_input1,filter_x0_before,filter_p0_before;
 float filter_gain0;
 int64_t coefficient,offset_count,numerator;
 uint32_t mean0,mean1,detector_mean,zero_variance_skips;
 double sigma_used,sigma_fast_used,sigma_center;
 uint32_t noise_count,pending_count,sigma_ready,sigma_safe,admitted_noise,discarded_noise,last_noise_source;
} Sensor1V1AutoSigmaResult;
void Sensor1V1AutoSigmaReset(void);
void Sensor1V1AutoSigmaSetInputContext(uint32_t tick,uint8_t valid);
/* Call once after Step and the existing ApplyEMA2Filter stage. */
void Sensor1V1AutoSigmaSetFilteredOutput(uint32_t output);
const Sensor1V1AutoSigmaResult *Sensor1V1AutoSigmaStep(uint32_t raw0,uint32_t raw1);
const Sensor1V1AutoSigmaResult *Sensor1V1AutoSigmaGet(void);
#ifdef SENSOR1_V1_HOST_TEST
const Sensor1V1AutoSigmaResult *Sensor1V1AutoSigmaStepFiltered(uint32_t f0,uint32_t f1);
#endif
#endif
