/* Ported from the recorded v1_fixed.h. See application record 20261018. */
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include "sensor1_v1_fixed_types.h"
#include "sensor1_v1_auto_sigma.h"
#include <string.h>
#include "sensor1_recent_quiet_sigma.h"
#define Operation_Single 0
#define Operation_Double 1
#define Operation_Quadruple 0
#define FDC2214 1
#define FDC2214_ADD 0
#define Drift_Compensation 1
#define Drift_Compensation_Proximity 0
#define Drift_Compensation_Temperature 0
#define KalmanFilters 1
#define n_samplingLR 80
#define Gain1st_Compensation1 0.4000
#define Gain1st_Compensation2 0.1000
#define Offset1 2.00E+07
#define Offset2 2.00E+07
#define IDLE_GUARD_PERIOD 0 /* User request: no initial forced-idle0 waiting period. */
static states s1v1_state;static fdc2214 s1v1_fdc;static driftCompensation s1v1_dComps;static samples s1v1_sample;static kalmanFilter s1v1_kalman;
static uint16_t s1v1_count_sampling=0,s1v1_peroid_sampling=0,s1v1_idle_guard_Q1=0,s1v1_idle_guard_Q2=0;
static uint8_t s1v1_n_PMsamplingLR=0,s1v1_n_data=0,s1v1_n_PMdata=0,s1v1_idle_guard_armed=0;
static uint8_t s1v1_flag_di=0,s1v1_n_samplingDI=0,s1v1_flag_algorithmDO=0,s1v1_flag_DOinit=0,s1v1_countDO=0;
static struct {uint32_t MeanData_Q1[1];int32_t tmpCoeff_Q1;}s1v1_cvf;
static struct {double mean_CapRaw,prev_CapRaw;uint64_t temp_b_uc;}s1v1_fusion;
static void s1v1_InitializeCompensation(void);static void s1v1_LinearRegression(void);
static int64_t s1v1_TwinSensorCompensation(uint32_t*,uint32_t*,float,int64_t);
static void s1v1_InitializeKalmanFilters(void);static void s1v1_GetFilteredData(void);
static uint32_t s1v1_KalmanFilter(uint32_t,uint8_t,float,float,uint8_t,uint32_t,uint32_t,uint32_t,float,float);
static uint32_t s1v1_MergeData(uint8_t*);static void s1v1_SplitData(uint32_t,uint8_t*);
static uint32_t v1_zero_variance_skips;
static void s1v1_InitializeCompensation(void)
{
	/* Parameter Initialization for Drift Compensation */
	s1v1_peroid_sampling = 20;
	s1v1_n_data = 0;
	s1v1_n_PMdata = 0;
	s1v1_n_PMsamplingLR = 40;
	s1v1_n_samplingDI = 3;

	#if FDC2214
	s1v1_state.idle_fdcQ1 = 0;
	s1v1_sample.Integral_fdcQ1[2] = 0;
	s1v1_sample.sumIntegral_fdcQ1 = 10000000;
	s1v1_sample.meanDOinit_fdcQ1 = s1v1_sample.sumIntegral_fdcQ1;

	#if Drift_Compensation_Proximity
	s1v1_sample.Integral_RfdcQ1[2] = 0;
	s1v1_sample.sumIntegral_RfdcQ1 = 10000000;
	#endif
	#endif

	#if FDC2214_ADD
	s1v1_state.idle_fdcQ2 = 0;
	s1v1_sample.Integral_fdcQ2[2] = 0;
	s1v1_sample.sumIntegral_fdcQ2 = 10000000;
	s1v1_sample.meanDOinit_fdcQ2 = s1v1_sample.sumIntegral_fdcQ2;
	#endif

	/* Drift Compensation Using the Temperature Measurement */
	#if Drift_Compensation_Temperature
	s1v1_state.movingObject_fdc = 1;
	s1v1_sample.Integral_tmp117[2] = 0;
	s1v1_sample.sumIntegral_tmp117 = 10000000;
	#endif
}

static void s1v1_LinearRegression(void)
{
    /* ============================================================
     * [Idle Guard] 1. Counter Decrement Logic
     * ============================================================ */
    if (s1v1_idle_guard_Q1 > 0) s1v1_idle_guard_Q1--;
    if (s1v1_idle_guard_Q2 > 0) s1v1_idle_guard_Q2--;

    /* Derivative Integration Algorithm */
    if(s1v1_flag_di == 0)
    {
        for(uint8_t i = 0; i < s1v1_n_samplingDI-1; i ++)
        {
            #if (Operation_Single || Operation_Double)
            #if FDC2214
            s1v1_sample.Integral_fdcQ1[2] = 0;
            s1v1_sample.DI_fdcQ1[s1v1_n_samplingDI-1-i] = s1v1_sample.DI_fdcQ1[s1v1_n_samplingDI-2-i];
            #endif
            #if FDC2214_ADD
            s1v1_sample.Integral_fdcQ2[2] = 0;
            s1v1_sample.DI_fdcQ2[s1v1_n_samplingDI-1-i] = s1v1_sample.DI_fdcQ2[s1v1_n_samplingDI-2-i];
            #endif
            #if Drift_Compensation_Proximity
            s1v1_sample.Integral_RfdcQ1[2] = 0;
            s1v1_sample.DI_RfdcQ1[s1v1_n_samplingDI-1-i] = s1v1_sample.DI_RfdcQ1[s1v1_n_samplingDI-2-i];
            #endif
            #if Drift_Compensation_Temperature
            s1v1_sample.Integral_tmp117[2] = 0;
            s1v1_sample.DI_tmp117[s1v1_n_samplingDI-1-i] = s1v1_sample.DI_tmp117[s1v1_n_samplingDI-2-i];
            #endif
            #endif
        }
        #if (Operation_Single || Operation_Double)
        #if FDC2214
        s1v1_sample.DI_fdcQ1[0] = s1v1_dComps.fdc_Q1;
        #endif
        #if FDC2214_ADD
        s1v1_sample.DI_fdcQ2[0] = s1v1_dComps.fdc_Q2;
        #endif
        #if Drift_Compensation_Proximity
        s1v1_sample.DI_RfdcQ1[0] = s1v1_dComps.Vfdc_Q1;
        #endif
        #if Drift_Compensation_Temperature
        s1v1_sample.DI_tmp117[0] = s1v1_dComps.Tfdc_tmp[1];
        #endif
        #endif
    }

    if(s1v1_flag_di == 1)
    {
        for(uint8_t i = 0; i < s1v1_n_samplingDI-1; i ++)
        {
            #if (Operation_Single || Operation_Double)
            #if FDC2214
            s1v1_sample.Integral_fdcQ1[2] = 0;
            s1v1_sample.DI_fdcQ1[s1v1_n_samplingDI-1-i] = s1v1_sample.DI_fdcQ1[s1v1_n_samplingDI-2-i];
            #endif

            #if FDC2214_ADD
            s1v1_sample.Integral_fdcQ2[2] = 0;
            s1v1_sample.DI_fdcQ2[s1v1_n_samplingDI-1-i] = s1v1_sample.DI_fdcQ2[s1v1_n_samplingDI-2-i];
            #endif

            /* Compensation in the Proximity mode */
            #if Drift_Compensation_Proximity
            #if FDC2214
            s1v1_sample.Integral_RfdcQ1[2] = 0;
            s1v1_sample.DI_RfdcQ1[s1v1_n_samplingDI-1-i] = s1v1_sample.DI_RfdcQ1[s1v1_n_samplingDI-2-i];
            #endif
            #endif

            #if Drift_Compensation_Temperature
            s1v1_sample.Integral_tmp117[2] = 0;
            s1v1_sample.DI_tmp117[s1v1_n_samplingDI-1-i] = s1v1_sample.DI_tmp117[s1v1_n_samplingDI-2-i];
            #endif
            #endif
        }

        #if (Operation_Single || Operation_Double)
        #if FDC2214
        s1v1_sample.DI_fdcQ1[0] = s1v1_dComps.fdc_Q1;
        s1v1_sample.Derivative_fdcQ1[0] = s1v1_sample.DI_fdcQ1[2] - s1v1_sample.DI_fdcQ1[1];
        s1v1_sample.Integral_fdcQ1[2] = s1v1_sample.Derivative_fdcQ1[0];
        s1v1_sample.sumIntegral_fdcQ1 = 50000000LL - (int64_t)s1v1_sample.DI_fdcQ1[1];
        #endif
        #if FDC2214_ADD
        s1v1_sample.DI_fdcQ2[0] = s1v1_dComps.fdc_Q2;
        s1v1_sample.Derivative_fdcQ2[0] = s1v1_sample.DI_fdcQ2[2] - s1v1_sample.DI_fdcQ2[1];
        s1v1_sample.Integral_fdcQ2[2] = s1v1_sample.Derivative_fdcQ2[0];
        s1v1_sample.sumIntegral_fdcQ2 += s1v1_sample.Integral_fdcQ2[2];
        #endif

        /* Compensation in the Proximity mode */
        #if Drift_Compensation_Proximity
        #if FDC2214
        s1v1_sample.DI_RfdcQ1[0] = s1v1_dComps.Vfdc_Q1;
        s1v1_sample.Derivative_RfdcQ1[0] = s1v1_sample.DI_RfdcQ1[2] - s1v1_sample.DI_RfdcQ1[1];
        s1v1_sample.Integral_RfdcQ1[2] = s1v1_sample.Derivative_RfdcQ1[0];
        s1v1_sample.sumIntegral_RfdcQ1 += s1v1_sample.Integral_RfdcQ1[2];
        #endif
        #endif

        #if Drift_Compensation_Temperature
        //s1v1_sample.DI_tmp117[0] = s1v1_dComps.Tfdc_Q1;
        s1v1_sample.DI_tmp117[0] = s1v1_dComps.Tfdc_tmp[1];
        s1v1_sample.Derivative_tmp117[0] = s1v1_sample.DI_tmp117[2] - s1v1_sample.DI_tmp117[1];
        s1v1_sample.Integral_tmp117[2] = s1v1_sample.Derivative_tmp117[0];
        s1v1_sample.sumIntegral_tmp117 += s1v1_sample.Integral_tmp117[2];
        #endif
        #endif

    }

    /* Detecting an Object Algorithm */
    if(s1v1_flag_algorithmDO == 1)
    {
        /* Mean value in initial s1v1_state */
        if(s1v1_countDO < n_samplingLR)
        {
            /* Shifting data to next arrays */
            for(uint8_t i = 0; i < n_samplingLR-1; i ++)
            {
                #if (Operation_Single || Operation_Double || Operation_Quadruple)
                #if FDC2214
                s1v1_sample.DOinit_fdcQ1[(n_samplingLR-1)-i] = s1v1_sample.DOinit_fdcQ1[(n_samplingLR-2)-i];
                #endif
                #if FDC2214_ADD
                s1v1_sample.DOinit_fdcQ2[(n_samplingLR-1)-i] = s1v1_sample.DOinit_fdcQ2[(n_samplingLR-2)-i];
                #endif
                #endif
            }

            #if (Operation_Single || Operation_Double || Operation_Quadruple)
            #if FDC2214
            s1v1_sample.DOinit_fdcQ1[0] = s1v1_sample.sumIntegral_fdcQ1;
            #endif
            #if FDC2214_ADD
            s1v1_sample.DOinit_fdcQ2[0] = s1v1_sample.sumIntegral_fdcQ2;
            #endif
            #endif

            /* Mean value */
            for(uint8_t i = 0; i < n_samplingLR; i ++)
            {
                if(i == 0)
                {
                    #if (Operation_Single || Operation_Double || Operation_Quadruple)
                    #if FDC2214
                    s1v1_sample.sumDO_fdcQ1 = 0;
                    #endif
                    #if FDC2214_ADD
                    s1v1_sample.sumDO_fdcQ2 = 0;
                    #endif
                    #endif
                }
                #if (Operation_Single || Operation_Double || Operation_Quadruple)
                #if FDC2214
                s1v1_sample.sumDO_fdcQ1 += s1v1_sample.DOinit_fdcQ1[i];
                #endif
                #if FDC2214_ADD
                s1v1_sample.sumDO_fdcQ2 += s1v1_sample.DOinit_fdcQ2[i];
                #endif
                #endif
            }
            #if (Operation_Single || Operation_Double || Operation_Quadruple)
            #if FDC2214
            s1v1_sample.meanDOinit_fdcQ1 = s1v1_sample.sumDO_fdcQ1*0.0125;
            #endif
            #if FDC2214_ADD
            s1v1_sample.meanDOinit_fdcQ2 = s1v1_sample.sumDO_fdcQ2*0.0125;
            #endif
            #endif

            s1v1_countDO ++;

            /* STD in initial s1v1_state */
            for(uint8_t i = 0; i < n_samplingLR; i ++)
            {
                if(i == 0)
                {
                    #if (Operation_Single || Operation_Double || Operation_Quadruple)
                    #if FDC2214
                    s1v1_sample.sumSTDDO_fdcQ1 = 0;
                    #endif
                    #if FDC2214_ADD
                    s1v1_sample.sumSTDDO_fdcQ2 = 0;
                    #endif
                    #endif
                }
                #if (Operation_Single || Operation_Double || Operation_Quadruple)
                #if FDC2214
                s1v1_sample.sumSTDDO_fdcQ1 += (s1v1_sample.DOinit_fdcQ1[i]-s1v1_sample.meanDOinit_fdcQ1)*(s1v1_sample.DOinit_fdcQ1[i]-s1v1_sample.meanDOinit_fdcQ1);
                #endif
                #if FDC2214_ADD
                s1v1_sample.sumSTDDO_fdcQ2 += (s1v1_sample.DOinit_fdcQ2[i]-s1v1_sample.meanDOinit_fdcQ2)*(s1v1_sample.DOinit_fdcQ2[i]-s1v1_sample.meanDOinit_fdcQ2);
                #endif
                #endif
            }
            #if (Operation_Single || Operation_Double || Operation_Quadruple)
            #if FDC2214
            s1v1_sample.stdDOinit_fdcQ1 = sqrt(s1v1_sample.sumSTDDO_fdcQ1*0.0125);
            #endif
            #if FDC2214_ADD
            s1v1_sample.stdDOinit_fdcQ2 = sqrt(s1v1_sample.sumSTDDO_fdcQ2*0.0125);
            #endif
            #endif
        }
        else if(s1v1_countDO >= n_samplingLR)
        {
            for(uint8_t i = 0; i < 10-1; i ++)
            {
                #if (Operation_Single || Operation_Double || Operation_Quadruple)
                #if FDC2214
                s1v1_sample.DO_fdcQ1[(10-1)-i] = s1v1_sample.DO_fdcQ1[(10-2)-i];
                #endif
                #if FDC2214_ADD
                s1v1_sample.DO_fdcQ2[(10-1)-i] = s1v1_sample.DO_fdcQ2[(10-2)-i];
                #endif

                #if Drift_Compensation_Proximity
                s1v1_sample.DO_RfdcQ1[(10-1)-i] = s1v1_sample.DO_RfdcQ1[(10-2)-i];
                #endif
                #endif

            }
            #if (Operation_Single || Operation_Double || Operation_Quadruple)
            #if FDC2214
            s1v1_sample.DO_fdcQ1[0] = s1v1_sample.sumIntegral_fdcQ1;
            #endif
            #if FDC2214_ADD
            s1v1_sample.DO_fdcQ2[0] = s1v1_sample.sumIntegral_fdcQ2;
            #endif

            #if Drift_Compensation_Proximity
            s1v1_sample.DO_RfdcQ1[0] = s1v1_sample.sumIntegral_RfdcQ1;
            #endif
            #endif


            /* Mean value */
            for(uint8_t i = 0; i < 10; i ++)
            {
                if(i == 0)
                {
                    #if (Operation_Single || Operation_Double || Operation_Quadruple)
                    #if FDC2214
                    s1v1_sample.sumDO_fdcQ1 = 0;
                    #endif
                    #if FDC2214_ADD
                    s1v1_sample.sumDO_fdcQ2 = 0;
                    #endif

                    #if Drift_Compensation_Proximity
                    s1v1_sample.sumDO_RfdcQ1 = 0;
                    #endif
                    #endif

                }
                #if (Operation_Single || Operation_Double || Operation_Quadruple)
                #if FDC2214
                s1v1_sample.sumDO_fdcQ1 += s1v1_sample.DO_fdcQ1[i];
                #endif
                #if FDC2214_ADD
                s1v1_sample.sumDO_fdcQ2 += s1v1_sample.DO_fdcQ2[i];
                #endif
                #if Drift_Compensation_Proximity
                s1v1_sample.sumDO_RfdcQ1 += s1v1_sample.DO_RfdcQ1[i];
                #endif
                #endif

            }
            #if (Operation_Single || Operation_Double || Operation_Quadruple)
            #if FDC2214
            s1v1_sample.meanDO_fdcQ1 = s1v1_sample.sumDO_fdcQ1 * 0.1;
            #endif
            #if FDC2214_ADD
            s1v1_sample.meanDO_fdcQ2 = s1v1_sample.sumDO_fdcQ2 * 0.1;
            #endif

            #if Drift_Compensation_Proximity
            s1v1_sample.meanDO_RfdcQ1 = s1v1_sample.sumDO_RfdcQ1 * 0.1;
            #endif
            #endif
        }


        /* STD after initial s1v1_state */
        for(uint8_t i = 0; i < 10; i ++)
        {
            if(i == 0)
            {
                #if (Operation_Single || Operation_Double || Operation_Quadruple)
                #if FDC2214
                s1v1_sample.sumSTDDO_fdcQ1 = 0;
                #endif
                #if FDC2214_ADD
                s1v1_sample.sumSTDDO_fdcQ2 = 0;
                #endif
                #if Drift_Compensation_Proximity
                s1v1_sample.sumSTDDO_RfdcQ1 = 0;
                #endif
                #endif
            }
            #if (Operation_Single || Operation_Double || Operation_Quadruple)
            #if FDC2214
            s1v1_sample.sumSTDDO_fdcQ1 += (s1v1_sample.DO_fdcQ1[i]-s1v1_sample.meanDO_fdcQ1)*(s1v1_sample.DO_fdcQ1[i]-s1v1_sample.meanDO_fdcQ1);
            #endif
            #if FDC2214_ADD
            s1v1_sample.sumSTDDO_fdcQ2 += (s1v1_sample.DO_fdcQ2[i]-s1v1_sample.meanDO_fdcQ2)*(s1v1_sample.DO_fdcQ2[i]-s1v1_sample.meanDO_fdcQ2);
            #endif
            #if Drift_Compensation_Proximity
            s1v1_sample.sumSTDDO_RfdcQ1 += (s1v1_sample.DO_RfdcQ1[i]-s1v1_sample.meanDO_RfdcQ1)*(s1v1_sample.DO_RfdcQ1[i]-s1v1_sample.meanDO_RfdcQ1);
            #endif
            #endif
        }
        #if (Operation_Single || Operation_Double || Operation_Quadruple)
        #if FDC2214
        s1v1_sample.stdDO_fdcQ1 = sqrt(s1v1_sample.sumSTDDO_fdcQ1*0.1);
        #endif
        #if FDC2214_ADD
        s1v1_sample.stdDO_fdcQ2 = sqrt(s1v1_sample.sumSTDDO_fdcQ2*0.1);
        #endif
        #if Drift_Compensation_Proximity
        s1v1_sample.stdDO_RfdcQ1 = sqrt(s1v1_sample.sumSTDDO_RfdcQ1*0.1);
        #endif
        #endif

        /* Judging Proximity detection */
        /* ============================================================
         * [Idle Guard] 2. Blocking Logic
         * ============================================================ */
        #if (Operation_Single || Operation_Double || Operation_Quadruple)
        #if FDC2214
        if (s1v1_idle_guard_Q1 == 0) // Guard OFF: Run Normal Logic
        {
            if((s1v1_sample.meanDO_fdcQ1 - 1.0E+07) >= RecentSigmaThreshold(s1v1_sample.meanDO_fdcQ1 - 1.0E+07, s1v1_sample.stdDOinit_fdcQ1, s1v1_sample.stdDO_fdcQ1, s1v1_state.idle_fdcQ1, s1v1_idle_guard_Q1, s1v1_countDO >= n_samplingLR))
            {
                /* Detecting an object */
                s1v1_state.idle_fdcQ1 = 1;
            }
            else
            {
                /* Idle s1v1_state */
                s1v1_state.idle_fdcQ1 = 0;
            }
        }
        else // Guard ON: Force Non-Idle
        {
            s1v1_state.idle_fdcQ1 = 0;
        }
        #endif

        #if FDC2214_ADD
        if (s1v1_idle_guard_Q2 == 0) // Guard OFF: Run Normal Logic
        {
            if((s1v1_sample.meanDO_fdcQ2 - 1.0E+07) >= 1.55*(s1v1_sample.stdDO_fdcQ2 + s1v1_sample.stdDOinit_fdcQ2))
            {
                /* Detecting an object */
                s1v1_state.idle_fdcQ2 = 1;
            }
            else
            {
                /* Idle s1v1_state */
                s1v1_state.idle_fdcQ2 = 0;
            }
        }
        else // Guard ON: Force Non-Idle
        {
            s1v1_state.idle_fdcQ2 = 0;
        }
        #endif

        #if Drift_Compensation_Proximity
        //if(((s1v1_sample.stdDO_RfdcQ1 < 5*(s1v1_sample.stdDO_fdcQ1)) && (s1v1_state.idle_fdc)))
        //if(((s1v1_sample.meanDO_RfdcQ1 - 1.0E+07 >= 4.0*s1v1_sample.stdDO_RfdcQ1) && (s1v1_state.idle_fdc)))
        if((s1v1_sample.stdDO_RfdcQ1 < 4*(s1v1_sample.stdDO_fdcQ1)) && (s1v1_sample.meanDO_RfdcQ1 - 1.0E+07 >= 4.0*s1v1_sample.stdDO_RfdcQ1) && (s1v1_state.idle_fdc))
        {
            /* Proximity in Constant distance */
            s1v1_state.stationaryObject_fdc = 1;
        }
        else
        {
            /* Object moving */
            s1v1_state.stationaryObject_fdc = 0;
        }
        #endif
        #endif
    }

    /* Linear Regression */
    if((s1v1_count_sampling % s1v1_peroid_sampling) == 0)
    {
        s1v1_count_sampling = 0;

        if((!(s1v1_state.idle_fdcQ1)) || (!(s1v1_state.idle_fdcQ2) || (s1v1_state.movingObject_fdc)) == 1)
        {
            /* Shifting data to next arrays */
            for(uint8_t i = 0; i < n_samplingLR-1; i ++)
            {
                #if (Operation_Single || Operation_Double || Operation_Quadruple)
                #if FDC2214
                if((!(s1v1_state.idle_fdcQ1)) == 1)
                {
                    s1v1_sample.fdc300[(n_samplingLR-1)-i] = s1v1_sample.fdc300[(n_samplingLR-2)-i];
                    s1v1_sample.fdc302[(n_samplingLR-1)-i] = s1v1_sample.fdc302[(n_samplingLR-2)-i];
                }
                #endif
                #if FDC2214_ADD
                if((!(s1v1_state.idle_fdcQ2)) == 1)
                {
                    s1v1_sample.fdc100[(n_samplingLR-1)-i] = s1v1_sample.fdc100[(n_samplingLR-2)-i];
                    s1v1_sample.fdc102[(n_samplingLR-1)-i] = s1v1_sample.fdc102[(n_samplingLR-2)-i];
                }
                #endif
                #if Drift_Compensation_Temperature
                if(s1v1_state.movingObject_fdc == 1)
                {
                    s1v1_sample.tmp117[(n_samplingLR-1)-i] = s1v1_sample.tmp117[(n_samplingLR-2)-i];
                    s1v1_sample.tmp117_f302[(n_samplingLR-1)-i] = s1v1_sample.tmp117_f302[(n_samplingLR-2)-i];
                }
                #endif
                #endif

                /*-----------*/
                if(s1v1_n_data == 0)
                {
                    #if (Operation_Single || Operation_Double || Operation_Quadruple)
                    #if FDC2214
                    if((!(s1v1_state.idle_fdcQ1)) == 1)
                    {
                        s1v1_sample.fdc300[i] = s1v1_fdc.ch300;
                        s1v1_sample.fdc302[i] = s1v1_fdc.ch302;
                    }
                    #endif
                    #if FDC2214_ADD
                    if((!(s1v1_state.idle_fdcQ2)) == 1)
                    {
                        s1v1_sample.fdc100[i] = s1v1_fdc.ch100;
                        s1v1_sample.fdc102[i] = s1v1_fdc.ch102;
                    }
                    #endif
                    #if Drift_Compensation_Temperature
                    if(s1v1_state.movingObject_fdc == 1)
                    {
                        s1v1_sample.tmp117[i] = tmp117.data;
                        s1v1_sample.tmp117_f302[i] = s1v1_fdc.ch302;
                    }
                    #endif
                    #endif

                }
                /*-----------*/
            }
            #if (Operation_Single || Operation_Double || Operation_Quadruple)
            #if FDC2214
            if((!(s1v1_state.idle_fdcQ1)) == 1)
            {
                s1v1_sample.fdc300[0] = s1v1_fdc.ch300;
                s1v1_sample.fdc302[0] = s1v1_fdc.ch302;
            }
            #endif
            #if FDC2214_ADD
            if((!(s1v1_state.idle_fdcQ2)) == 1)
            {
                s1v1_sample.fdc100[0] = s1v1_fdc.ch100;
                s1v1_sample.fdc102[0] = s1v1_fdc.ch102;
            }
            #endif
            #if Drift_Compensation_Temperature
            if(s1v1_state.movingObject_fdc == 1)
            {
                s1v1_sample.tmp117[0] = tmp117.data;
                s1v1_sample.tmp117_f302[0] = s1v1_fdc.ch302;
            }
            #endif
            #endif


            /* Mean value */
            for(uint8_t i = 0; i < n_samplingLR; i ++)
            {
                /* Reset the SUM variables to zero */
                if(i == 0)
                {
                    #if (Operation_Single || Operation_Double || Operation_Quadruple)
                    #if FDC2214
                    if((!(s1v1_state.idle_fdcQ1)) == 1)
                    {
                        s1v1_sample.sum_fdc300 = 0;
                        s1v1_sample.sum_fdc302 = 0;
                    }
                    #endif
                    #if FDC2214_ADD
                    if((!(s1v1_state.idle_fdcQ2)) == 1)
                    {
                        s1v1_sample.sum_fdc100 = 0;
                        s1v1_sample.sum_fdc102 = 0;
                    }
                    #endif
                    #if Drift_Compensation_Temperature
                    if(s1v1_state.movingObject_fdc == 1)
                    {
                        s1v1_sample.sum_tmp117 = 0;
                        s1v1_sample.sum_tmp117_f302 = 0;
                    }
                    #endif
                    #endif
                }
                #if (Operation_Single || Operation_Double || Operation_Quadruple)
                #if FDC2214
                if((!(s1v1_state.idle_fdcQ1)) == 1)
                {
                    s1v1_sample.sum_fdc300 += s1v1_sample.fdc300[i];
                    s1v1_sample.sum_fdc302 += s1v1_sample.fdc302[i];
                }
                #endif
                #if FDC2214_ADD
                if((!(s1v1_state.idle_fdcQ2)) == 1)
                {
                    s1v1_sample.sum_fdc100 += s1v1_sample.fdc100[i];
                    s1v1_sample.sum_fdc102 += s1v1_sample.fdc102[i];
                }
                #endif
                #if Drift_Compensation_Temperature
                if(s1v1_state.movingObject_fdc == 1)
                {
                    s1v1_sample.sum_tmp117 += s1v1_sample.tmp117[i];
                    s1v1_sample.sum_tmp117_f302 += s1v1_sample.tmp117_f302[i];
                }
                #endif
                #endif
            }
            #if (Operation_Single || Operation_Double || Operation_Quadruple)
            #if FDC2214
            if((!(s1v1_state.idle_fdcQ1)) == 1)
            {
                s1v1_sample.mean_fdc300 = s1v1_sample.sum_fdc300 * 0.0125;
                s1v1_sample.mean_fdc302 = s1v1_sample.sum_fdc302 * 0.0125;
            }
            #endif
            #if FDC2214_ADD
            if((!(s1v1_state.idle_fdcQ2)) == 1)
            {
                s1v1_sample.mean_fdc100 = s1v1_sample.sum_fdc100 * 0.0125;
                s1v1_sample.mean_fdc102 = s1v1_sample.sum_fdc102 * 0.0125;
            }
            #endif
            #if Drift_Compensation_Temperature
            if(s1v1_state.movingObject_fdc == 1)
            {
                s1v1_sample.mean_tmp117 = s1v1_sample.sum_tmp117 * 0.02;
                s1v1_sample.mean_tmp117_f302 = s1v1_sample.sum_tmp117_f302 * 0.02;
            }
            #endif
            #endif


            /* Error */
            for(uint8_t i = 0; i < n_samplingLR; i ++)
            {
                if(i == 0)
                {
                    #if (Operation_Single || Operation_Double || Operation_Quadruple)
                    #if FDC2214
                    if((!(s1v1_state.idle_fdcQ1)) == 1)
                    {
                        s1v1_dComps.sumNumnerator_fdcQ1 = 0;
                        s1v1_dComps.sumDenominator_fdcQ1 = 0;
                    }
                    #endif
                    #if FDC2214_ADD
                    if((!(s1v1_state.idle_fdcQ2)) == 1)
                    {
                        s1v1_dComps.sumNumnerator_fdcQ2 = 0;
                        s1v1_dComps.sumDenominator_fdcQ2 = 0;
                    }
                    #endif
                    #if Drift_Compensation_Temperature
                    if(s1v1_state.movingObject_fdc == 1)
                    {
                        s1v1_dComps.sumNumnerator_tmp117 = 0;
                        s1v1_dComps.sumDenominator_tmp117 = 0;
                    }
                    #endif
                    #endif
                }

                #if (Operation_Single || Operation_Double || Operation_Quadruple)
                #if FDC2214
                if((!(s1v1_state.idle_fdcQ1)) == 1)
                {
                    #if 0
                    s1v1_sample.err_fdc300[i] = abs(s1v1_sample.fdc300[i] - s1v1_sample.mean_fdc300);
                    s1v1_sample.err_fdc302[i] = abs(s1v1_sample.fdc302[i] - s1v1_sample.mean_fdc302);
                    #endif

                    #if 1
                    s1v1_sample.err_fdc300[i] = s1v1_sample.fdc300[i] - s1v1_sample.mean_fdc300;
                    s1v1_sample.err_fdc302[i] = s1v1_sample.fdc302[i] - s1v1_sample.mean_fdc302;
                    #endif
                }
                #endif
                #if FDC2214_ADD
                if((!(s1v1_state.idle_fdcQ2)) == 1)
                {
                    #if 0
                    s1v1_sample.err_fdc100[i] =  abs(s1v1_sample.fdc100[i] - s1v1_sample.mean_fdc100);
                    s1v1_sample.err_fdc102[i] =  abs(s1v1_sample.fdc102[i] - s1v1_sample.mean_fdc102);
                    #endif

                    #if 1
                    s1v1_sample.err_fdc100[i] = (s1v1_sample.fdc100[i] - s1v1_sample.mean_fdc100);
                    s1v1_sample.err_fdc102[i] = (s1v1_sample.fdc102[i] - s1v1_sample.mean_fdc102);
                    #endif
                }
                #endif
                #if Drift_Compensation_Temperature
                if(s1v1_state.movingObject_fdc == 1)
                {
                    s1v1_sample.err_tmp117[i] = abs(s1v1_sample.tmp117[i] - s1v1_sample.mean_tmp117);
                    s1v1_sample.err_tmp117_f302[i] = abs(s1v1_sample.tmp117_f302[i] - s1v1_sample.mean_tmp117_f302);
                }
                #endif
                #endif


                /* Necessary to fill array of sum_XXXX */
                #if (Operation_Single || Operation_Double || Operation_Quadruple)
                #if FDC2214
                if((!(s1v1_state.idle_fdcQ1)) == 1)
                {
                    s1v1_dComps.sumNumnerator_fdcQ1 += (s1v1_sample.err_fdc300[i]*s1v1_sample.err_fdc302[i]);
                    s1v1_dComps.sumDenominator_fdcQ1 += (s1v1_sample.err_fdc302[i]*s1v1_sample.err_fdc302[i]);
                }
                #endif
                #if FDC2214_ADD
                if((!(s1v1_state.idle_fdcQ2)) == 1)
                {
                    s1v1_dComps.sumNumnerator_fdcQ2 += (s1v1_sample.err_fdc100[i]*s1v1_sample.err_fdc102[i]);
                    s1v1_dComps.sumDenominator_fdcQ2 += (s1v1_sample.err_fdc102[i]*s1v1_sample.err_fdc102[i]);
                }
                #endif
                #if Drift_Compensation_Temperature
                if(s1v1_state.movingObject_fdc == 1)
                {
                    s1v1_dComps.sumNumnerator_tmp117 += (s1v1_sample.err_tmp117_f302[i]*s1v1_sample.err_tmp117[i]);
                    s1v1_dComps.sumDenominator_tmp117 += (s1v1_sample.err_tmp117[i]*s1v1_sample.err_tmp117[i]);
                }
                #endif
                #endif
            }

        }

        /* Linear Regression When an Object Approaches */
        #if Drift_Compensation_Proximity
        //s1v1_state.stationaryObject_fdc = ((s1v1_sample.stdDO_fdcQ1 < 0.7*(s1v1_sample.stdDOinit_fdcQ1 + s1v1_sample.stdDO_fdcQ1)) && s1v1_state.idle_fdc);
        if((s1v1_state.stationaryObject_fdc) == 1)
        {
            /* Shifting data to next arrays */
            for(uint8_t i = 0; i < s1v1_n_PMsamplingLR-1; i ++)
            {
                #if (Operation_Single || Operation_Double || Operation_Quadruple)
                if(s1v1_state.stationaryObject_fdc == 1)
                {
                    s1v1_sample.PMfdc300[(s1v1_n_PMsamplingLR-1)-i] = s1v1_sample.PMfdc300[(s1v1_n_PMsamplingLR-2)-i];
                    s1v1_sample.PMfdc302[(s1v1_n_PMsamplingLR-1)-i] = s1v1_sample.PMfdc302[(s1v1_n_PMsamplingLR-2)-i];
                }
                #endif

                /*-----------*/
                if(s1v1_n_PMdata == 0)
                {
                    #if (Operation_Single || Operation_Double || Operation_Quadruple)
                    if(s1v1_state.stationaryObject_fdc == 1)
                    {
                        s1v1_sample.PMfdc300[i] = s1v1_fdc.ch300;
                        s1v1_sample.PMfdc302[i] = s1v1_fdc.ch302;
                    }
                    #endif
                }
                /*-----------*/
            }
            s1v1_n_PMdata = 1;


            #if (Operation_Single || Operation_Double || Operation_Quadruple)
            if(s1v1_state.stationaryObject_fdc == 1)
            {
                s1v1_sample.PMfdc300[0] = s1v1_fdc.ch300;
                s1v1_sample.PMfdc302[0] = s1v1_fdc.ch302;
            }
            #endif

            /* Mean value */
            for(uint8_t i = 0; i < s1v1_n_PMsamplingLR; i ++)
            {
                if(i == 0)
                {
                    #if (Operation_Single || Operation_Double || Operation_Quadruple)
                    if(s1v1_state.stationaryObject_fdc == 1)
                    {
                        s1v1_sample.sum_PMfdc300 = 0;
                        s1v1_sample.sum_PMfdc302 = 0;
                    }
                    #endif
                }
                #if (Operation_Single || Operation_Double || Operation_Quadruple)
                if(s1v1_state.stationaryObject_fdc == 1)
                {
                    s1v1_sample.sum_PMfdc300 += s1v1_sample.PMfdc300[i];
                    s1v1_sample.sum_PMfdc302 += s1v1_sample.PMfdc302[i];
                }
                #endif
            }
            #if (Operation_Single || Operation_Double || Operation_Quadruple)
            if(s1v1_state.stationaryObject_fdc == 1)
            {
                s1v1_sample.mean_PMfdc300 = s1v1_sample.sum_PMfdc300 * 0.025;
                s1v1_sample.mean_PMfdc302 = s1v1_sample.sum_PMfdc302 * 0.025;
            }
            #endif


            /* Error */
            for(uint8_t i = 0; i < s1v1_n_PMsamplingLR; i ++)
            {
                /* Reset the variables to zero */
                if(i == 0)
                {
                    #if (Operation_Single || Operation_Double || Operation_Quadruple)
                    if(s1v1_state.stationaryObject_fdc == 1)
                    {
                        s1v1_dComps.sumNumnerator_PMfdcQ1 = 0;
                        s1v1_dComps.sumDenominator_PMfdcQ1 = 0;
                    }
                    #endif
                }
                #if (Operation_Single || Operation_Double || Operation_Quadruple)
                if(s1v1_state.stationaryObject_fdc == 1)
                {
                    s1v1_sample.err_PMfdc300[i] = abs(s1v1_sample.PMfdc300[i] - s1v1_sample.mean_PMfdc300);
                    s1v1_sample.err_PMfdc302[i] = abs(s1v1_sample.PMfdc302[i] - s1v1_sample.mean_PMfdc302);
                }
                #endif

                /* Necessary to fill array of sum_XXXX */
                #if (Operation_Single || Operation_Double || Operation_Quadruple)
                if(s1v1_state.stationaryObject_fdc == 1)
                {
                    s1v1_dComps.sumNumnerator_PMfdcQ1 += (s1v1_sample.err_PMfdc300[i]*s1v1_sample.err_PMfdc302[i]);
                    s1v1_dComps.sumDenominator_PMfdcQ1 += (s1v1_sample.err_PMfdc302[i]*s1v1_sample.err_PMfdc302[i]);
                }
                #endif
            }

            s1v1_dComps.PMfdcCoeff_Q1 = 1.5*(s1v1_dComps.sumNumnerator_PMfdcQ1*10000/s1v1_dComps.sumDenominator_PMfdcQ1);
            #if 1
            if(s1v1_dComps.PMfdcCoeff_Q1 < 0)
            {
                s1v1_dComps.PMfdcCoeff_Q1 = 0;
            }
            else if(s1v1_dComps.PMfdcCoeff_Q1 > 9990)
            {
                s1v1_dComps.PMfdcCoeff_Q1 = 9990;
            }
            #endif
            s1v1_dComps.PMfdcOffset_Q1 = s1v1_sample.mean_fdc300-((float)s1v1_dComps.PMfdcCoeff_Q1*0.0001)*s1v1_sample.mean_fdc302;
        }
        else if(s1v1_state.stationaryObject_fdc == 0)
        {
            #if (Operation_Single || Operation_Double || Operation_Quadruple)
            s1v1_dComps.PMfdcCoeff_Q1 = s1v1_dComps.fdcCoeff_Q1;
            s1v1_dComps.PMfdcOffset_Q1 = s1v1_sample.fdc300[0] - ((float)s1v1_dComps.PMfdcCoeff_Q1*0.0001)*s1v1_sample.fdc302[0];

            s1v1_sample.sum_PMfdc300 = 0;
            s1v1_sample.sum_PMfdc302 = 0;
            #endif

            s1v1_n_PMdata = 0;
        }
        #endif

        if(s1v1_n_data < n_samplingLR)
        {
            /* Before Sampling 50 data */
            s1v1_n_data ++;

            #if (Operation_Single || Operation_Double || Operation_Quadruple)
            #if FDC2214
            s1v1_dComps.fdcCoeff_Q1 = Gain1st_Compensation1*10000;
            s1v1_dComps.fdcOffset_Q1 = s1v1_sample.fdc300[0]-((float)s1v1_dComps.fdcCoeff_Q1*0.0001)*s1v1_sample.fdc302[0];
            //s1v1_dComps.fdcOffset_Q1 = s1v1_fdc.ch300-((float)s1v1_dComps.fdcCoeff_Q1*0.0001)*s1v1_fdc.ch302;
            #endif
            #if FDC2214_ADD
            s1v1_dComps.fdcCoeff_Q2= Gain1st_Compensation2*10000;
            s1v1_dComps.fdcOffset_Q2 = s1v1_sample.fdc100[0]-((float)s1v1_dComps.fdcCoeff_Q2*0.0001)*s1v1_sample.fdc102[0];
            #endif
            #if Drift_Compensation_Temperature
            s1v1_dComps.tmp117_fdcCoeff_Q1 = (s1v1_dComps.sumNumnerator_tmp117*100000/s1v1_dComps.sumDenominator_tmp117);
            s1v1_dComps.tmp117_fdcOffset_Q1 = s1v1_sample.fdc302[0]-((float)s1v1_dComps.tmp117_fdcCoeff_Q1*0.00001)*s1v1_sample.tmp117[0];
            #endif
            #endif

            //s1v1_flag_di = 0;
        }
        else if(s1v1_n_data >= n_samplingLR)
        {
            /* After Sampling 50 data */

            #if (Operation_Single || Operation_Double || Operation_Quadruple)
            #if FDC2214
            if (s1v1_dComps.sumDenominator_fdcQ1 > 0)
            {
            s1v1_dComps.fdcCoeff_Q1 = (s1v1_dComps.sumNumnerator_fdcQ1*10000/s1v1_dComps.sumDenominator_fdcQ1);
            if(s1v1_dComps.fdcCoeff_Q1 > 9999)
            {
                s1v1_dComps.fdcCoeff_Q1 = 9999;
            }
            else if(s1v1_dComps.fdcCoeff_Q1 < 0)
            {
                s1v1_dComps.fdcCoeff_Q1 = 0;
            }
            s1v1_dComps.fdcOffset_Q1 = s1v1_sample.mean_fdc300-((float)s1v1_dComps.fdcCoeff_Q1*0.0001)*s1v1_sample.mean_fdc302;
            }
            else
            {
                v1_zero_variance_skips++;
            }
            #endif
            #if FDC2214_ADD
            s1v1_dComps.fdcCoeff_Q2 = (s1v1_dComps.sumNumnerator_fdcQ2*10000/s1v1_dComps.sumDenominator_fdcQ2);
            if(s1v1_dComps.fdcCoeff_Q2 > 9999)
            {
                s1v1_dComps.fdcCoeff_Q2 = 9999;
            }
            else if(s1v1_dComps.fdcCoeff_Q2 < 0)
            {
                s1v1_dComps.fdcCoeff_Q2 = 0;
            }
            s1v1_dComps.fdcOffset_Q2 = s1v1_sample.mean_fdc100-((float)s1v1_dComps.fdcCoeff_Q2*0.0001)*s1v1_sample.mean_fdc102;
            #endif
            #if Drift_Compensation_Temperature
            s1v1_dComps.tmp117_fdcCoeff_Q1 = (s1v1_dComps.sumNumnerator_tmp117*100000/s1v1_dComps.sumDenominator_tmp117);
            s1v1_dComps.tmp117_fdcOffset_Q1 = s1v1_sample.mean_tmp117_f302-((float)s1v1_dComps.tmp117_fdcCoeff_Q1*0.00001)*s1v1_sample.tmp117[0];
            #endif
            #endif


            /* Low frequency */
            #if 0
            //s1v1_peroid_sampling = 1000;
            #endif

            /* High frequency */
            #if 0
            s1v1_peroid_sampling = 80;
            #endif

            #if 1
            if((!(s1v1_state.idle_fdcQ1)) || (!(s1v1_state.idle_fdcQ2) || (s1v1_state.movingObject_fdc)) == 1)
            {
                s1v1_peroid_sampling = 80;
            }
            else
            {
                s1v1_peroid_sampling = 400;
            }
            #endif

            /* ============================================================
             * [Idle Guard] 3. Triggering/Arming Logic
             * ============================================================ */
            if (s1v1_idle_guard_armed == 0)
            {
                s1v1_idle_guard_Q1 = IDLE_GUARD_PERIOD;
                s1v1_idle_guard_Q2 = IDLE_GUARD_PERIOD;
                s1v1_idle_guard_armed = 1;
                s1v1_state.idle_fdcQ1 = 0;
                s1v1_state.idle_fdcQ2 = 0;
            }

            s1v1_flag_di = 1;
            s1v1_flag_algorithmDO = 1;
        }

    }
    s1v1_count_sampling ++;

    #if (Operation_Single || Operation_Double || Operation_Quadruple)
    #if FDC2214
    s1v1_dComps.fdc_Q1 = s1v1_TwinSensorCompensation(&s1v1_fdc.ch300, &s1v1_fdc.ch302, (((float)s1v1_dComps.fdcCoeff_Q1)*0.0001), s1v1_dComps.fdcOffset_Q1) + 2*Offset1;
//    if (s1v1_dComps.fdc_Q1 > 2 * Offset1)
//        {
//            s1v1_dComps.fdc_Q1 = 2 * Offset1;
//        }
    //s1v1_dComps.Vfdc_Q1 = s1v1_TwinSensorCompensation(&s1v1_fdc.ch300, &s1v1_fdc.ch302, (((float)s1v1_dComps.PMfdcCoeff_Q1)*0.0001), s1v1_dComps.PMfdcOffset_Q1) + 2*Offset1;
    #endif
    #if FDC2214_ADD

    s1v1_dComps.fdc_Q2 = s1v1_TwinSensorCompensation(&s1v1_fdc.ch100, &s1v1_fdc.ch102, (((float)s1v1_dComps.fdcCoeff_Q2)*0.0001), s1v1_dComps.fdcOffset_Q2) + 2*Offset2;
//    if (s1v1_dComps.fdc_Q2 > 2 * Offset2)
//        {
//            s1v1_dComps.fdc_Q2 = 2 * Offset2;
//        }
    #endif
    #endif


    #if Drift_Compensation_Temperature
    s1v1_dComps.Tfdc_tmp[1] = TemperatureCompensation(&s1v1_fdc.ch300, &tmp117.data, (((float)s1v1_dComps.tmp117_fdcCoeff_Q1)*0.00001), s1v1_dComps.tmp117_fdcOffset_Q1 )+ 2*Offset1;
    //s1v1_dComps.Tfdc_tmp[1] = s1v1_fdc.ch300 + (s1v1_fdc.ch302 - s1v1_dComps.Tfdc_tmp[0]);
    s1v1_dComps.Tfdc_Q1 = 2*Offset1 - (s1v1_sample.sumIntegral_tmp117 - 1E+06);
    #endif


    /* Debugging the Compensated Value */
    #if 1
    s1v1_SplitData(s1v1_dComps.fdc_Q1, s1v1_dComps.temp_fdcQ1);
    s1v1_SplitData(s1v1_dComps.fdc_Q2, s1v1_dComps.temp_fdcQ2);
    #endif

    #if 1
    if(s1v1_n_data >= n_samplingLR)
    {
        s1v1_fusion.mean_CapRaw = 0.98*s1v1_fusion.prev_CapRaw + 0.02*abs(s1v1_cvf.MeanData_Q1[0] - s1v1_dComps.fdc_Q1);
        s1v1_fusion.prev_CapRaw = s1v1_fusion.mean_CapRaw;

        s1v1_fusion.temp_b_uc = (uint64_t)(abs(s1v1_cvf.tmpCoeff_Q1*0.0001*s1v1_fusion.mean_CapRaw));
        if(s1v1_fusion.temp_b_uc > 0xFFFFFFFF)
        {
            s1v1_fusion.temp_b_uc = 0xFFFFFFFF;
        }
        //s1v1_fusion.temp_b_uc = (uint64_t)(s1v1_cvf.tmpCoeff_Q1);
        //s1v1_fusion.temp_b_uc = (uint32_t)(s1v1_fusion.std_CapDist*10000);
        //s1v1_fusion.temp_b_uc = s1v1_fusion.mean_CapRaw;
    }

    #endif
}

static int64_t s1v1_TwinSensorCompensation(uint32_t* input_sensor, uint32_t* input_compensation, float gain_1st, int64_t offset)
{
	int64_t merge_sensor = 0;
	int64_t merge_compensation = 0;
	int64_t result = 0;

	merge_sensor = *input_sensor;
	merge_compensation = *input_compensation;

	result = merge_sensor - gain_1st*(merge_compensation) - offset;
	return result;
}

static void s1v1_InitializeKalmanFilters(void)
{
	/* Kalman Filter */
	#if KalmanFilters
	s1v1_kalman.A_Q1 = 1;
	s1v1_kalman.H_Q1 = 1;
	s1v1_kalman.Q_Q1 = 0.1;
	s1v1_kalman.R_Q1 = 1;
	s1v1_kalman.x_Q1 = 2.85E+07;
	s1v1_kalman.P_Q1 = 1;

	s1v1_kalman.A_Q1cmp = 1;
	s1v1_kalman.H_Q1cmp = 1;
	s1v1_kalman.Q_Q1cmp = 0.1;
	s1v1_kalman.R_Q1cmp = 1;
	s1v1_kalman.x_Q1cmp = 3.8E+07;
	s1v1_kalman.P_Q1cmp = 1;

	s1v1_kalman.A_Q2 = 1;
	s1v1_kalman.H_Q2 = 1;
	s1v1_kalman.Q_Q2 = 0.01;
	s1v1_kalman.R_Q2 = 1;
	s1v1_kalman.x_Q2 = 2.85E+07;
	s1v1_kalman.P_Q2 = 1;

	s1v1_kalman.A_Q2cmp = 1;
	s1v1_kalman.H_Q2cmp = 1;
	s1v1_kalman.Q_Q2cmp = 0.1;
	s1v1_kalman.R_Q2cmp = 1;
	s1v1_kalman.x_Q2cmp = 3.8E+07;
	s1v1_kalman.P_Q2cmp = 1;
	#endif
}

static uint32_t s1v1_KalmanFilter(uint32_t input, uint8_t A, float H, float Q, uint8_t R, uint32_t x, uint32_t P, uint32_t xp, float Pp, float K)
{
	uint32_t result = 0;

	xp = A*x;
	Pp = A*P*A + Q;
	K = Pp*H/(H*Pp*H + R);
	x = xp + K*(input - H*xp);
	P = Pp - K*H*Pp;

	return result = x;
}

static uint32_t s1v1_MergeData(uint8_t* temp_data)
{
	uint32_t result = 0;

	result = (temp_data[0] << 24)|(temp_data[1] << 16)|(temp_data[2] << 8)|(temp_data[3] << 0);

	return result;
}

static void s1v1_SplitData(uint32_t data_32bit, uint8_t *data_8bit)
{
	data_8bit[3] = (uint8_t)((data_32bit >> 0) & 0xFF);
	data_8bit[2] = (uint8_t)((data_32bit >> 8) & 0xFF);
	data_8bit[1] = (uint8_t)((data_32bit >> 16) & 0xFF);
	data_8bit[0] = (uint8_t)((data_32bit >> 24) & 0xFF);
}

static void s1v1_GetFilteredData(void)
{
	/* Kalman Filter */
	#if (KalmanFilters)
	/* Merging data */
	#if FDC2214
	s1v1_kalman.fdc300 = s1v1_MergeData(s1v1_fdc.temp_ch300);
	s1v1_kalman.fdc302 = s1v1_MergeData(s1v1_fdc.temp_ch302);
	#endif
	#if FDC2214_ADD
	s1v1_kalman.fdc100 = s1v1_MergeData(s1v1_fdc.temp_ch100);
	s1v1_kalman.fdc102 = s1v1_MergeData(s1v1_fdc.temp_ch102);
	#endif
	s1v1_kalman.result_Q1 = s1v1_KalmanFilter(s1v1_kalman.fdc300, s1v1_kalman.A_Q1, s1v1_kalman.H_Q1, s1v1_kalman.Q_Q1, s1v1_kalman.R_Q1, s1v1_kalman.x_Q1, s1v1_kalman.P_Q1, s1v1_kalman.xp_Q1, s1v1_kalman.Pp_Q1, s1v1_kalman.K_Q1);
	s1v1_kalman.result_Q1cmp = s1v1_KalmanFilter(s1v1_kalman.fdc302, s1v1_kalman.A_Q1cmp, s1v1_kalman.H_Q1cmp, s1v1_kalman.Q_Q1cmp, s1v1_kalman.R_Q1cmp, s1v1_kalman.x_Q1cmp, s1v1_kalman.P_Q1cmp, s1v1_kalman.xp_Q1cmp, s1v1_kalman.Pp_Q1cmp, s1v1_kalman.K_Q1cmp);
	s1v1_kalman.result_Q2 = s1v1_KalmanFilter(s1v1_kalman.fdc100, s1v1_kalman.A_Q2, s1v1_kalman.H_Q2, s1v1_kalman.Q_Q2, s1v1_kalman.R_Q2, s1v1_kalman.x_Q2, s1v1_kalman.P_Q2, s1v1_kalman.xp_Q2, s1v1_kalman.Pp_Q2, s1v1_kalman.K_Q2);
	s1v1_kalman.result_Q2cmp = s1v1_KalmanFilter(s1v1_kalman.fdc102, s1v1_kalman.A_Q2cmp, s1v1_kalman.H_Q2cmp, s1v1_kalman.Q_Q2cmp, s1v1_kalman.R_Q2cmp, s1v1_kalman.x_Q2cmp, s1v1_kalman.P_Q2cmp, s1v1_kalman.xp_Q2cmp, s1v1_kalman.Pp_Q2cmp, s1v1_kalman.K_Q2cmp);
	s1v1_kalman.x_Q1    = s1v1_kalman.result_Q1;
	s1v1_kalman.x_Q1cmp = s1v1_kalman.result_Q1cmp;
	s1v1_kalman.x_Q2    = s1v1_kalman.result_Q2;
	s1v1_kalman.x_Q2cmp = s1v1_kalman.result_Q2cmp;
	/* Debugging Kalman Filter */
	#if 0
	s1v1_SplitData(s1v1_kalman.result_Q1, s1v1_kalman.temp_resultQ1);
	s1v1_SplitData(s1v1_kalman.result_Q1cmp, s1v1_kalman.temp_resultQ1cmp);
	#endif
	#endif

	#if (KalmanFilters)
	#if FDC2214
	s1v1_fdc.ch300 = s1v1_kalman.result_Q1;
	s1v1_fdc.ch302 = s1v1_kalman.result_Q1cmp;
	#endif
	#if FDC2214_ADD
	s1v1_fdc.ch100 = s1v1_kalman.result_Q2;
	s1v1_fdc.ch102 = s1v1_kalman.result_Q2cmp;
	#endif
	#endif

	#if !(KalmanFilters)
	/* Merging data */
	#if FDC2214
	s1v1_fdc.ch300 = s1v1_MergeData(s1v1_fdc.temp_ch300);
	s1v1_fdc.ch302 = s1v1_MergeData(s1v1_fdc.temp_ch302);
	#endif
	#if FDC2214_ADD
	s1v1_fdc.ch100 = s1v1_MergeData(s1v1_fdc.temp_ch100);
	s1v1_fdc.ch102 = s1v1_MergeData(s1v1_fdc.temp_ch102);
	#endif
	#endif

	#if Drift_Compensation_Temperature
	tmp117.data = (tmp117.temp_data[0] << 8)|(tmp117.temp_data[1] << 0);
	#endif
}

/* Adapter: same call order as the selected fixed.exe replay. No HAL or ToF input. */
static Sensor1V1AutoSigmaResult v1_result;
static uint32_t v1_accepted;
void Sensor1V1AutoSigmaSetInputContext(uint32_t tick,uint8_t valid){RecentSigmaSetInput(tick,valid);}
void Sensor1V1AutoSigmaReset(void)
{
 RecentSigmaReset();
 memset(&s1v1_state,0,sizeof(s1v1_state)); memset(&s1v1_fdc,0,sizeof(s1v1_fdc));
 memset(&s1v1_dComps,0,sizeof(s1v1_dComps)); memset(&s1v1_sample,0,sizeof(s1v1_sample));
 memset(&s1v1_kalman,0,sizeof(s1v1_kalman)); memset(&s1v1_cvf,0,sizeof(s1v1_cvf));
 memset(&s1v1_fusion,0,sizeof(s1v1_fusion)); memset(&v1_result,0,sizeof(v1_result));
 s1v1_count_sampling=s1v1_peroid_sampling=s1v1_idle_guard_Q1=s1v1_idle_guard_Q2=0;
 s1v1_n_PMsamplingLR=s1v1_n_data=s1v1_n_PMdata=s1v1_idle_guard_armed=0;
 s1v1_flag_di=s1v1_n_samplingDI=s1v1_flag_algorithmDO=s1v1_flag_DOinit=s1v1_countDO=0;
 v1_zero_variance_skips=v1_accepted=0;
 s1v1_InitializeCompensation(); s1v1_InitializeKalmanFilters();
}
static const Sensor1V1AutoSigmaResult *V1FinishStep(void)
{
 uint32_t due=(s1v1_count_sampling%s1v1_peroid_sampling)==0;
 threshold_used=1.55*((double)s1v1_sample.stdDO_fdcQ1+s1v1_sample.stdDOinit_fdcQ1);
 s1v1_LinearRegression();
 uint32_t pair=due&&!s1v1_state.idle_fdcQ1;
 if(pair)++v1_accepted;
 uint32_t pre=s1v1_dComps.fdc_Q1;
 /* Adaptive EMA is applied once by main, then fed back explicitly. */
 v1_result.object=s1v1_state.idle_fdcQ1;
 v1_result.allowed=!s1v1_state.idle_fdcQ1;
 v1_result.source_event=pair; v1_result.accepted_pairs=v1_accepted;
 v1_result.n_data=s1v1_n_data; v1_result.guard=s1v1_idle_guard_Q1;
 v1_result.do_ready=s1v1_flag_algorithmDO&&s1v1_countDO>=n_samplingLR;
 v1_result.period=s1v1_peroid_sampling;
 v1_result.gain=s1v1_dComps.fdcCoeff_Q1*.0001;
 v1_result.offset=(double)s1v1_dComps.fdcOffset_Q1;
 v1_result.output=(double)s1v1_dComps.fdc_Q1-40000000.;
 v1_result.pre=(double)pre-40000000.;
 v1_result.delta=(double)s1v1_sample.meanDO_fdcQ1-1e7;
 v1_result.threshold=threshold_used;
 v1_result.init_sigma=s1v1_sample.stdDOinit_fdcQ1;
 v1_result.recent_sigma=s1v1_sample.stdDO_fdcQ1;
 v1_result.filtered0=s1v1_fdc.ch300; v1_result.filtered1=s1v1_fdc.ch302;
 v1_result.denominator=s1v1_dComps.sumDenominator_fdcQ1;
 v1_result.final_count=s1v1_dComps.fdc_Q1; v1_result.pre_count=pre;
 v1_result.di_ready=s1v1_flag_di; v1_result.algorithm_ready=s1v1_flag_algorithmDO;
 v1_result.do_count=s1v1_countDO; v1_result.sampling_count=s1v1_count_sampling;
 v1_result.coefficient=s1v1_dComps.fdcCoeff_Q1;
 v1_result.offset_count=s1v1_dComps.fdcOffset_Q1;
 v1_result.numerator=s1v1_dComps.sumNumnerator_fdcQ1;
 v1_result.mean0=(uint32_t)s1v1_sample.mean_fdc300;
 v1_result.mean1=(uint32_t)s1v1_sample.mean_fdc302;
 v1_result.detector_mean=(uint32_t)s1v1_sample.meanDO_fdcQ1;
 v1_result.zero_variance_skips=v1_zero_variance_skips;
 v1_result.sigma_used=sigma_ready?sigma_slow:s1v1_sample.stdDOinit_fdcQ1;
 v1_result.sigma_fast_used=sigma_ready?sigma_fast:s1v1_sample.stdDO_fdcQ1;
 v1_result.sigma_center=sigma_center;
 v1_result.noise_count=(uint32_t)quiet_n;v1_result.pending_count=(uint32_t)pending_n;
 v1_result.sigma_ready=(uint32_t)sigma_ready;v1_result.sigma_safe=(uint32_t)sigma_safe;
 v1_result.admitted_noise=admitted_noise;v1_result.discarded_noise=discarded_noise;
 v1_result.last_noise_source=last_noise_source;
 return &v1_result;
}
const Sensor1V1AutoSigmaResult *Sensor1V1AutoSigmaStep(uint32_t raw0,uint32_t raw1)
{
 v1_result.filter_input0=raw0; v1_result.filter_input1=raw1;
 v1_result.filter_x0_before=s1v1_kalman.x_Q1;
 v1_result.filter_p0_before=s1v1_kalman.P_Q1;
 float pp=s1v1_kalman.A_Q1*s1v1_kalman.P_Q1*s1v1_kalman.A_Q1+s1v1_kalman.Q_Q1;
 v1_result.filter_gain0=pp*s1v1_kalman.H_Q1/(s1v1_kalman.H_Q1*pp*s1v1_kalman.H_Q1+s1v1_kalman.R_Q1);
 s1v1_SplitData(raw0,s1v1_fdc.temp_ch300); s1v1_SplitData(raw1,s1v1_fdc.temp_ch302);
 s1v1_GetFilteredData();
 return V1FinishStep();
}
void Sensor1V1AutoSigmaSetFilteredOutput(uint32_t output)
{
 /* The next detector/quiet-sigma update must see the actual final EMA,
  * not an unfiltered compensation value or the removed ADKF state. */
 s1v1_dComps.fdc_Q1=output;
 v1_result.final_count=output;
 v1_result.output=(double)output-40000000.;
}
const Sensor1V1AutoSigmaResult *Sensor1V1AutoSigmaGet(void){return &v1_result;}
#ifdef SENSOR1_V1_HOST_TEST
const Sensor1V1AutoSigmaResult *Sensor1V1AutoSigmaStepFiltered(uint32_t f0,uint32_t f1)
{
 s1v1_fdc.ch300=f0; s1v1_fdc.ch302=f1;
 return V1FinishStep();
}
#endif
