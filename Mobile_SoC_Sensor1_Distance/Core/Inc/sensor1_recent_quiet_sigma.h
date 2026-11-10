#ifndef SENSOR1_RECENT_QUIET_SIGMA_H
#define SENSOR1_RECENT_QUIET_SIGMA_H
/* Exact auto_3sigma candidate: 500ms observations, 60s quiet history,
 * 20 committed points, 2.4s pending validation. No physical count floor. */
#include <stdint.h>
#include <math.h>
#include <string.h>
#ifndef SIGMA_MODE
#define SIGMA_MODE 2
#endif
/* Mode0: fixed-anchor baseline;1: original1.55*(quiet slowSD+quiet fastSD);
   2:3*quiet slowSD. Only noise estimator and threshold connection differ. */
#ifdef SIGMA_FAST_INIT
#define QUIET_CAPACITY 640
#define PENDING_CAPACITY 32
#define SIGMA_OBSERVATION_MS 100
#else
#define QUIET_CAPACITY 128
#define PENDING_CAPACITY 8
#define SIGMA_OBSERVATION_MS 500
#endif
typedef struct {uint32_t tick,index;double value,fast;} QuietPoint;
static QuietPoint quiet[QUIET_CAPACITY],pending_noise[PENDING_CAPACITY];
static uint32_t sigma_tick,sigma_valid,next_noise_tick;
static int quiet_n,pending_n,sigma_ready,sigma_safe;
static double sigma_slow,sigma_fast,sigma_center,threshold_used;
static uint32_t admitted_noise,discarded_noise;
static uint32_t sigma_index,last_noise_source=UINT32_MAX;
static void quiet_commit(QuietPoint p){
 int first=0;while(first<quiet_n&&sigma_tick-quiet[first].tick>60000)first++;
 if(first){memmove(quiet,quiet+first,(quiet_n-first)*sizeof(QuietPoint));quiet_n-=first;}
 if(quiet_n==QUIET_CAPACITY){memmove(quiet,quiet+1,(QUIET_CAPACITY-1)*sizeof(QuietPoint));quiet_n=QUIET_CAPACITY-1;}
 quiet[quiet_n++]=p;admitted_noise++;last_noise_source=p.index;
 if(quiet_n>=20){double mean=0,fast=0;for(int j=0;j<quiet_n;j++){mean+=quiet[j].value;fast+=quiet[j].fast;}mean/=quiet_n;fast/=quiet_n;double ss=0;for(int j=0;j<quiet_n;j++){double d=quiet[j].value-mean;ss+=d*d;}sigma_slow=fmax(1e-6,sqrt(fmax(0.,ss/quiet_n)));sigma_fast=fast;sigma_center=mean;sigma_ready=1;}
}
static double RecentSigmaThreshold(double delta,double initial,double fast,int previous_object,int guard,int detector_ready){
 double original=1.55*(initial+fast);
#if SIGMA_MODE == 0
 threshold_used=original;return original;
#else
 double oldsigma=sigma_ready?sigma_slow:fmax(1e-6,initial);
 /* Causal quiet admission. Values past2sigma in approach direction are not
    noise; future2.4s must remain clear. No motor/ToF labels or model changes. */
 /* Hook is inside original guard-OFF branch; no observations are taken
    during the forced idle0 startup period in either cadence variant. */
 int guard_allows_noise=!guard;
 sigma_safe=(int)(sigma_valid&&detector_ready&&guard_allows_noise&&!previous_object&&delta<2.*oldsigma);
 if(!sigma_safe){discarded_noise+=pending_n;pending_n=0;}
 if(sigma_tick>=next_noise_tick){next_noise_tick=sigma_tick+SIGMA_OBSERVATION_MS;
  if(sigma_safe){
   if(pending_n&&sigma_tick-pending_noise[0].tick>=2400){QuietPoint p=pending_noise[0];memmove(pending_noise,pending_noise+1,(--pending_n)*sizeof(QuietPoint));quiet_commit(p);}
   if(pending_n<PENDING_CAPACITY)pending_noise[pending_n++]=(QuietPoint){sigma_tick,sigma_index,delta,fast};
  }
 }
 if(!sigma_ready)threshold_used=original;
 else{
#if SIGMA_MODE == 1
  threshold_used=1.55*(sigma_slow+sigma_fast);
#else
  threshold_used=3.*sigma_slow;
#endif
 }
 return threshold_used;
#endif
}

static uint32_t sigma_context_seen,sigma_previous_tick;
static void RecentSigmaReset(void){
 memset(quiet,0,sizeof(quiet));memset(pending_noise,0,sizeof(pending_noise));
 sigma_tick=sigma_valid=next_noise_tick=0;sigma_index=UINT32_MAX;
 quiet_n=pending_n=sigma_ready=sigma_safe=0;
 sigma_slow=sigma_fast=sigma_center=threshold_used=0.;
 admitted_noise=discarded_noise=0;last_noise_source=UINT32_MAX;
 sigma_context_seen=sigma_previous_tick=0;
}
static void RecentSigmaSetInput(uint32_t tick,uint8_t valid){
 uint8_t gap=(uint8_t)(sigma_context_seen && (uint32_t)(tick-sigma_previous_tick)>100U);
 sigma_tick=tick;sigma_valid=(uint32_t)(valid && !gap);sigma_index++;
 if(!sigma_valid){discarded_noise+=(uint32_t)pending_n;pending_n=0;sigma_safe=0;}
 sigma_previous_tick=tick;sigma_context_seen=1U;
}
#endif
