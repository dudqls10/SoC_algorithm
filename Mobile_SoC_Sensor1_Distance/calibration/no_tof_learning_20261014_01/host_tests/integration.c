/* Post-fit integration test. Real guard/noise/math headers and real trainer.
 * Inputs are hypothetical filtered pairs; this is not a reconstruction of
 * the missing Oct14 boot trace. Object=0 isolates the most permissive case. */
#include <stdio.h>
#include <stdint.h>
#include <inttypes.h>
#include <string.h>
#include <math.h>
#include "drift_learning_guard.h"
#include "drift_noise_reference.h"
#include "drift_model_math.h"
#include "ema_precision.h"
#include "production_types.h"

#define n_samplingLR 80U
ProductionSample sample;
driftCompensation dComps;
struct { uint32_t ch300,ch302; } fdc;
DriftLearningGuard drift_learning_guard;
DriftNoiseReference drift_noise_reference;
uint32_t drift_learning_candidate_rejects,drift_eval_zero_variance_skips;
uint32_t drift_eval_learning_updates,drift_learning_input_ch0,drift_learning_input_ch1;
int64_t drift_model_sensor_variance;
uint32_t peroid_sampling,flag_di,flag_algorithmDO;
#include "train_delayed_production.inc"

typedef struct {
    const char *name;
    int64_t initial_coefficient,final_coefficient;
    uint32_t updates,admitted,rejects,zero_variance,first_admit;
    uint32_t reason,noise_ready,noise_samples,noise_peak;
    uint32_t residual_blocks,wait_clear_blocks,object_blocks;
    uint32_t updates_after_shift;
    double residual,guard_noise;
} Result;

static int reference_delta(uint32_t index) {
    return ((int)(index % 29U)-14)*5;
}

static int initialize_fixture(int positive,int constant) {
    memset(&sample,0,sizeof(sample));memset(&dComps,0,sizeof(dComps));
    DriftLearningGuardReset(&drift_learning_guard);
    DriftNoiseReferenceReset(&drift_noise_reference);
    drift_learning_candidate_rejects=drift_eval_zero_variance_skips=0;
    drift_eval_learning_updates=drift_learning_input_ch0=drift_learning_input_ch1=0;
    drift_model_sensor_variance=0;peroid_sampling=20;flag_di=flag_algorithmDO=0;
    /* Production bootstrap provisional gain is0.4000, not persistent gain0. */
    dComps.fdcCoeff_Q1=4000;
    for(uint32_t i=0;i<n_samplingLR;i++) {
        int r=reference_delta(i);
        sample.fdc302[i]=(uint32_t)(10000000+(constant?0:r));
        sample.fdc300[i]=(uint32_t)(8000000+(positive?r/2:-r/2));
    }
    for(uint32_t i=1;i<=16;i++)
        DriftLearningGuardStepCapOnly(&drift_learning_guard,i*50U,8000000,10000000,1,0);
    fdc.ch300=sample.fdc300[0];fdc.ch302=sample.fdc302[0];
    return TrainDelayedQ1Pair(fdc.ch300,fdc.ch302,1000U)!=0;
}

static Result scenario(const char *name,int positive,int constant,
        int shift,uint32_t shift_at,uint32_t return_at,int object) {
    Result out={0};out.name=name;out.first_admit=UINT32_MAX;
    if(!initialize_fixture(positive,constant)){out.reason=UINT32_MAX;return out;}
    out.initial_coefficient=dComps.fdcCoeff_Q1;
    uint32_t initial_updates=drift_eval_learning_updates;
    uint32_t updates_at_shift=initial_updates;
    uint32_t post=40000000U;
    double accumulator=40000000.0;float alpha=0;
    for(uint32_t elapsed=0;elapsed<=120000U;elapsed+=10U) {
        uint32_t tick=1000U+elapsed;
        int r=reference_delta(elapsed/10U);
        int shifted=elapsed>=shift_at && (!return_at || elapsed<return_at);
        int added=shifted?shift:0;
        uint8_t detected=(uint8_t)(object && shifted);
        fdc.ch302=(uint32_t)(10000000+r);
        fdc.ch300=(uint32_t)(8000000+r/2+added);
        if(elapsed==shift_at)updates_at_shift=drift_eval_learning_updates;
        DriftLearningGuardStepCapOnly(&drift_learning_guard,tick,fdc.ch300,fdc.ch302,1,detected);
        /* Same admission order as LinearRegression; detector/EMA readiness
         * are deliberately true to isolate guard/noise/trainer behavior. */
        uint8_t clear=(uint8_t)(drift_learning_guard.ready &&
            drift_learning_guard.allowed && !drift_learning_guard.reason && !detected);
        DriftNoiseReferenceStep(&drift_noise_reference,tick,post,clear);
        if(drift_noise_reference.sample_count>out.noise_peak)
            out.noise_peak=drift_noise_reference.sample_count;
        if(elapsed%800U==0U && DriftNoiseReferenceAllowsLearning(
                &drift_noise_reference,drift_learning_guard.ready)) {
            uint32_t a=0,b=0;
            if(DriftLearningGuardTakePair(&drift_learning_guard,tick,
                    fdc.ch300,fdc.ch302,&a,&b)) {
                out.admitted++;
                if(out.first_admit==UINT32_MAX)out.first_admit=elapsed;
                (void)TrainDelayedQ1Pair(a,b,tick);
            }
        }
        if(drift_learning_guard.reason&DRIFT_LEARN_RESIDUAL)out.residual_blocks++;
        if(drift_learning_guard.reason&DRIFT_LEARN_WAIT_CLEAR)out.wait_clear_blocks++;
        if(drift_learning_guard.reason&DRIFT_LEARN_OBJECT)out.object_blocks++;
        int64_t residual=DriftCompensatedResidual(fdc.ch300,fdc.ch302,
            (double)dComps.fdcCoeff_Q1*.0001,dComps.fdcOffset_Q1);
        post=EMA2PrecisionStep(&accumulator,(uint32_t)(40000000LL+residual),
                              5000.0,42000.0,0.12,0.45,&alpha);
    }
    out.final_coefficient=dComps.fdcCoeff_Q1;
    out.updates=drift_eval_learning_updates-initial_updates;
    out.updates_after_shift=drift_eval_learning_updates-updates_at_shift;
    out.rejects=drift_learning_candidate_rejects;
    out.zero_variance=drift_eval_zero_variance_skips;
    out.reason=drift_learning_guard.reason;
    out.noise_ready=drift_noise_reference.ready;
    out.noise_samples=drift_noise_reference.sample_count;
    out.residual=drift_learning_guard.residual_mean;
    out.guard_noise=drift_learning_guard.noise;
    return out;
}

static void show(Result r) {
    printf("{\"name\":\"%s\",\"initial_coefficient\":%" PRId64
           ",\"final_coefficient\":%" PRId64
           ",\"updates\":%u,\"admitted\":%u,\"rejects\":%u,\"zero_variance\":%u"
           ",\"first_admit_elapsed_ms\":%u,\"reason\":%u,\"noise_ready\":%u"
           ",\"noise_samples\":%u,\"noise_peak\":%u,\"residual_blocks\":%u"
           ",\"wait_clear_blocks\":%u,\"object_blocks\":%u,\"updates_after_shift\":%u"
           ",\"final_residual_mean\":%.3f,\"guard_noise\":%.3f}\n",
           r.name,r.initial_coefficient,r.final_coefficient,r.updates,r.admitted,
           r.rejects,r.zero_variance,r.first_admit,r.reason,r.noise_ready,
           r.noise_samples,r.noise_peak,r.residual_blocks,r.wait_clear_blocks,
           r.object_blocks,r.updates_after_shift,r.residual,r.guard_noise);
}

int main(void) {
    Result a=scenario("zero_fit_then_stable_clear",0,0,0,1000,0,0);
    Result b=scenario("zero_fit_then_plus60_immediately_after_seed",0,0,60,0,0,0);
    Result c=scenario("zero_fit_then_plus200_before_noise_ready",0,0,200,1000,0,0);
    Result d=scenario("zero_fit_plus200_then_return_at30s",0,0,200,1000,30000,0);
    Result e=scenario("positive_fit_then_plus200",1,0,200,1000,0,0);
    Result f=scenario("zero_fit_plus200_after_noise_ready",0,0,200,20000,0,0);
    Result g=scenario("zero_fit_real_object_flag",0,0,0,1000,0,1);
    Result h=scenario("constant_reference_retains_provisional_gain",0,1,0,1000,0,0);
    Result i=scenario("zero_fit_plus60_after1s_of_clear",0,0,60,1000,0,0);
    show(a);show(b);show(c);show(d);show(e);show(f);show(g);show(h);show(i);
    unsigned failures=0;
    if(a.initial_coefficient!=0 || a.final_coefficient<3000 || !a.updates || !a.noise_ready)failures|=1;
    if(b.final_coefficient!=0 || b.updates || b.reason!=DRIFT_LEARN_WAIT_CLEAR || b.noise_ready)failures|=2;
    if(c.final_coefficient!=0 || c.updates || !(c.reason&DRIFT_LEARN_RESIDUAL) || c.noise_ready)failures|=4;
    if(d.final_coefficient<3000 || !d.updates || !d.noise_ready)failures|=8;
    if(e.initial_coefficient<3000 || e.updates || !(e.reason&DRIFT_LEARN_RESIDUAL))failures|=16;
    if(!f.noise_ready || f.updates_after_shift || !(f.reason&DRIFT_LEARN_RESIDUAL))failures|=32;
    if(g.updates || !(g.reason&DRIFT_LEARN_OBJECT))failures|=64;
    if(h.initial_coefficient!=4000 || !h.zero_variance)failures|=128;
    if(i.final_coefficient<3000 || !i.updates || !i.noise_ready)failures|=256;
    printf("{\"failure_bitmask\":%u,\"all_scenario_expectations_passed\":%s}\n",failures,failures?"false":"true");
    return failures?1:0;
}
