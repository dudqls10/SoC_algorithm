#include <stdlib.h>
#include <math.h>
#include "main.h"
#include "sensor1_distance_calibration.h"
#define FDC_ADD 0
#define CAN_CH0 0x41
Curvefitting cvf; driftCompensation dComps; distance dist; states state;
volatile unsigned test_failures; volatile unsigned test_done;
void InitializeCurveFitting(void)
{
	uint16_t d0 = 270;
	uint8_t d = 10;

	cvf.X[0] = 320;
	cvf.X[1] = d0;
	cvf.X[2] = d0 - 1*d;
	cvf.X[3] = d0 - 2*d;
	cvf.X[4] = d0 - 3*d;
	cvf.X[5] = d0 - 4*d;
	cvf.X[6] = d0 - 5*d;
	cvf.X[7] = d0 - 6*d;
	cvf.X[8] = d0 - 7*d;
	cvf.X[9] = d0 - 8*d;
	cvf.X[10] = d0 - 9*d;
	cvf.X[11] = d0 - 10*d;
	cvf.X[12] = d0 - 11*d;
	cvf.X[13] = d0 - 12*d;
	cvf.X[14] = d0 - 13*d;
	cvf.X[15] = d0 - 14*d;
	cvf.X[16] = d0 - 15*d;
	cvf.X[17] = d0 - 16*d;
	cvf.X[18] = d0 - 17*d;
	cvf.X[19] = d0 - 18*d;
	cvf.X[20] = d0 - 19*d;
	cvf.X[21] = d0 - 20*d;
	cvf.X[22] = d0 - 21*d;
	cvf.X[23] = d0 - 22*d;
	cvf.X[24] = d0 - 23*d;
	cvf.X[25] = d0 - 24*d;

	cvf.Sum_X = cvf.X[1]+cvf.X[2]+cvf.X[3]+cvf.X[4]+cvf.X[5]+cvf.X[6]+cvf.X[7]+cvf.X[8]+cvf.X[9]+cvf.X[10]
				+cvf.X[11]+cvf.X[12]+cvf.X[13]+cvf.X[14]+cvf.X[15]+cvf.X[16]+cvf.X[17]+cvf.X[18]+cvf.X[19]+cvf.X[20]
				+cvf.X[21]+cvf.X[22]+cvf.X[23]+cvf.X[24]+cvf.X[25];
	cvf.Mean_X = cvf.Sum_X / 25;

	cvf.MeanData_Q1[0] = SENSOR1_CAP_BASELINE_COUNT;
	cvf.dataQ1_curr[0] = SENSOR1_CAP_BASELINE_COUNT;
	cvf.dataQ1_prev[0] = SENSOR1_CAP_BASELINE_COUNT;

	/* Existing25 slots:27..3cm. Recorded low-signal26cm value is retained. */
	#if 1
	cvf.MeanData_Q1[0] = SENSOR1_CAP_BASELINE_COUNT;
	cvf.MeanData_Q1[1] = 180; /* 27cm, measured approach mean */
	cvf.MeanData_Q1[2] = 86; /* 26cm, measured approach mean */
	cvf.MeanData_Q1[3] = 230; /* 25cm, measured approach mean */
	cvf.MeanData_Q1[4] = 247; /* 24cm, measured approach mean */
	cvf.MeanData_Q1[5] = 351; /* 23cm, measured approach mean */
	cvf.MeanData_Q1[6] = 414; /* 22cm, measured approach mean */
	cvf.MeanData_Q1[7] = 554; /* 21cm, measured approach mean */
	cvf.MeanData_Q1[8] = 605; /* 20cm, measured approach mean */
	cvf.MeanData_Q1[9] = 817; /* 19cm, measured approach mean */
	cvf.MeanData_Q1[10] = 942; /* 18cm, measured approach mean */
	cvf.MeanData_Q1[11] = 1273; /* 17cm, measured approach mean */
	cvf.MeanData_Q1[12] = 1430; /* 16cm, measured approach mean */
	cvf.MeanData_Q1[13] = 1725; /* 15cm, measured approach mean */
	cvf.MeanData_Q1[14] = 2119; /* 14cm, measured approach mean */
	cvf.MeanData_Q1[15] = 2645; /* 13cm, measured approach mean */
	cvf.MeanData_Q1[16] = 3432; /* 12cm, measured approach mean */
	cvf.MeanData_Q1[17] = 4307; /* 11cm, measured approach mean */
	cvf.MeanData_Q1[18] = 5459; /* 10cm, measured approach mean */
	cvf.MeanData_Q1[19] = 7038; /* 9cm, measured approach mean */
	cvf.MeanData_Q1[20] = 9078; /* 8cm, measured approach mean */
	cvf.MeanData_Q1[21] = 12242; /* 7cm, measured approach mean */
	cvf.MeanData_Q1[22] = 17028; /* 6cm, measured approach mean */
	cvf.MeanData_Q1[23] = 24066; /* 5cm, measured approach mean */
	cvf.MeanData_Q1[24] = 34688; /* 4cm, measured approach mean */
	cvf.MeanData_Q1[25] = 53939; /* 3cm, measured approach mean */


	cvf.dataQ1_curr[0] = SENSOR1_CAP_BASELINE_COUNT;								cvf.dataQ1_prev[0] = SENSOR1_CAP_BASELINE_COUNT;
	cvf.dataQ1_curr[1] = SENSOR1_CAP_BASELINE_COUNT - cvf.MeanData_Q1[1];			cvf.dataQ1_prev[1] = SENSOR1_CAP_BASELINE_COUNT - cvf.MeanData_Q1[1];
	cvf.dataQ1_curr[2] = SENSOR1_CAP_BASELINE_COUNT - cvf.MeanData_Q1[2];			cvf.dataQ1_prev[2] = SENSOR1_CAP_BASELINE_COUNT - cvf.MeanData_Q1[2];
	cvf.dataQ1_curr[3] = SENSOR1_CAP_BASELINE_COUNT - cvf.MeanData_Q1[3];			cvf.dataQ1_prev[3] = SENSOR1_CAP_BASELINE_COUNT - cvf.MeanData_Q1[3];
	cvf.dataQ1_curr[4] = SENSOR1_CAP_BASELINE_COUNT - cvf.MeanData_Q1[4];			cvf.dataQ1_prev[4] = SENSOR1_CAP_BASELINE_COUNT - cvf.MeanData_Q1[4];
	cvf.dataQ1_curr[5] = SENSOR1_CAP_BASELINE_COUNT - cvf.MeanData_Q1[5];			cvf.dataQ1_prev[5] = SENSOR1_CAP_BASELINE_COUNT - cvf.MeanData_Q1[5];
	cvf.dataQ1_curr[6] = SENSOR1_CAP_BASELINE_COUNT - cvf.MeanData_Q1[6];			cvf.dataQ1_prev[6] = SENSOR1_CAP_BASELINE_COUNT - cvf.MeanData_Q1[6];
	cvf.dataQ1_curr[7] = SENSOR1_CAP_BASELINE_COUNT - cvf.MeanData_Q1[7];			cvf.dataQ1_prev[7] = SENSOR1_CAP_BASELINE_COUNT - cvf.MeanData_Q1[7];
	cvf.dataQ1_curr[8] = SENSOR1_CAP_BASELINE_COUNT - cvf.MeanData_Q1[8];			cvf.dataQ1_prev[8] = SENSOR1_CAP_BASELINE_COUNT - cvf.MeanData_Q1[8];
	cvf.dataQ1_curr[9] = SENSOR1_CAP_BASELINE_COUNT - cvf.MeanData_Q1[9];			cvf.dataQ1_prev[9] = SENSOR1_CAP_BASELINE_COUNT - cvf.MeanData_Q1[9];
	cvf.dataQ1_curr[10] = SENSOR1_CAP_BASELINE_COUNT - cvf.MeanData_Q1[10];		cvf.dataQ1_prev[10] = SENSOR1_CAP_BASELINE_COUNT - cvf.MeanData_Q1[10];
	cvf.dataQ1_curr[11] = SENSOR1_CAP_BASELINE_COUNT - cvf.MeanData_Q1[11];		cvf.dataQ1_prev[11] = SENSOR1_CAP_BASELINE_COUNT - cvf.MeanData_Q1[11];
	cvf.dataQ1_curr[12] = SENSOR1_CAP_BASELINE_COUNT - cvf.MeanData_Q1[12];		cvf.dataQ1_prev[12] = SENSOR1_CAP_BASELINE_COUNT - cvf.MeanData_Q1[12];
	cvf.dataQ1_curr[13] = SENSOR1_CAP_BASELINE_COUNT - cvf.MeanData_Q1[13];		cvf.dataQ1_prev[13] = SENSOR1_CAP_BASELINE_COUNT - cvf.MeanData_Q1[13];
	cvf.dataQ1_curr[14] = SENSOR1_CAP_BASELINE_COUNT - cvf.MeanData_Q1[14];		cvf.dataQ1_prev[14] = SENSOR1_CAP_BASELINE_COUNT - cvf.MeanData_Q1[14];
	cvf.dataQ1_curr[15] = SENSOR1_CAP_BASELINE_COUNT - cvf.MeanData_Q1[15];		cvf.dataQ1_prev[15] = SENSOR1_CAP_BASELINE_COUNT - cvf.MeanData_Q1[15];
	cvf.dataQ1_curr[16] = SENSOR1_CAP_BASELINE_COUNT - cvf.MeanData_Q1[16];		cvf.dataQ1_prev[16] = SENSOR1_CAP_BASELINE_COUNT - cvf.MeanData_Q1[16];
	cvf.dataQ1_curr[17] = SENSOR1_CAP_BASELINE_COUNT - cvf.MeanData_Q1[17];		cvf.dataQ1_prev[17] = SENSOR1_CAP_BASELINE_COUNT - cvf.MeanData_Q1[17];
	cvf.dataQ1_curr[18] = SENSOR1_CAP_BASELINE_COUNT - cvf.MeanData_Q1[18];		cvf.dataQ1_prev[18] = SENSOR1_CAP_BASELINE_COUNT - cvf.MeanData_Q1[18];
	cvf.dataQ1_curr[19] = SENSOR1_CAP_BASELINE_COUNT - cvf.MeanData_Q1[19];		cvf.dataQ1_prev[19] = SENSOR1_CAP_BASELINE_COUNT - cvf.MeanData_Q1[19];
	cvf.dataQ1_curr[20] = SENSOR1_CAP_BASELINE_COUNT - cvf.MeanData_Q1[20];		cvf.dataQ1_prev[20] = SENSOR1_CAP_BASELINE_COUNT - cvf.MeanData_Q1[20];
	cvf.dataQ1_curr[21] = SENSOR1_CAP_BASELINE_COUNT - cvf.MeanData_Q1[21];		cvf.dataQ1_prev[21] = SENSOR1_CAP_BASELINE_COUNT - cvf.MeanData_Q1[21];
	cvf.dataQ1_curr[22] = SENSOR1_CAP_BASELINE_COUNT - cvf.MeanData_Q1[22];		cvf.dataQ1_prev[22] = SENSOR1_CAP_BASELINE_COUNT - cvf.MeanData_Q1[22];
	cvf.dataQ1_curr[23] = SENSOR1_CAP_BASELINE_COUNT - cvf.MeanData_Q1[23];		cvf.dataQ1_prev[23] = SENSOR1_CAP_BASELINE_COUNT - cvf.MeanData_Q1[23];
	cvf.dataQ1_curr[24] = SENSOR1_CAP_BASELINE_COUNT - cvf.MeanData_Q1[24];		cvf.dataQ1_prev[24] = SENSOR1_CAP_BASELINE_COUNT - cvf.MeanData_Q1[24];
	cvf.dataQ1_curr[25] = SENSOR1_CAP_BASELINE_COUNT - cvf.MeanData_Q1[25];		cvf.dataQ1_prev[25] = SENSOR1_CAP_BASELINE_COUNT - cvf.MeanData_Q1[25];
	#endif


	#if FDC_ADD
	cvf.MeanData_Q2[0] = 4.00E+07;
	cvf.MeanData_Q2[1] = 80;
	cvf.MeanData_Q2[2] = 80;
	cvf.MeanData_Q2[3] = 80;
	cvf.MeanData_Q2[4] = 80;
	cvf.MeanData_Q2[5] = 80;
	cvf.MeanData_Q2[6] = 100;
	cvf.MeanData_Q2[7] = 377;
	cvf.MeanData_Q2[8] = 400;
	cvf.MeanData_Q2[9] = 506;
	cvf.MeanData_Q2[10] = 813;
	cvf.MeanData_Q2[11] = 1151;
	cvf.MeanData_Q2[12] = 1595;
	cvf.MeanData_Q2[13] = 1857;
	cvf.MeanData_Q2[14] = 2188;
	cvf.MeanData_Q2[15] = 2846;
	cvf.MeanData_Q2[16] = 4231;
	cvf.MeanData_Q2[17] = 5054;
	cvf.MeanData_Q2[18] = 6337;
	cvf.MeanData_Q2[19] = 7887;
	cvf.MeanData_Q2[20] = 10092;
	cvf.MeanData_Q2[21] = 12326;
	cvf.MeanData_Q2[22] = 14846;
	cvf.MeanData_Q2[23] = 19088;
	cvf.MeanData_Q2[24] = 25246;
	cvf.MeanData_Q2[25] = 35039;


	cvf.dataQ2_curr[0] = 4.00E+07;								cvf.dataQ2_prev[0] = 4.00E+07;
	cvf.dataQ2_curr[1] = 4.00E+07 - cvf.MeanData_Q2[1];			cvf.dataQ2_prev[1] = 4.00E+07 - cvf.MeanData_Q2[1];
	cvf.dataQ2_curr[2] = 4.00E+07 - cvf.MeanData_Q2[2];			cvf.dataQ2_prev[2] = 4.00E+07 - cvf.MeanData_Q2[2];
	cvf.dataQ2_curr[3] = 4.00E+07 - cvf.MeanData_Q2[3];			cvf.dataQ2_prev[3] = 4.00E+07 - cvf.MeanData_Q2[3];
	cvf.dataQ2_curr[4] = 4.00E+07 - cvf.MeanData_Q2[4];			cvf.dataQ2_prev[4] = 4.00E+07 - cvf.MeanData_Q2[4];
	cvf.dataQ2_curr[5] = 4.00E+07 - cvf.MeanData_Q2[5];			cvf.dataQ2_prev[5] = 4.00E+07 - cvf.MeanData_Q2[5];
	cvf.dataQ2_curr[6] = 4.00E+07 - cvf.MeanData_Q2[6];			cvf.dataQ2_prev[6] = 4.00E+07 - cvf.MeanData_Q2[6];
	cvf.dataQ2_curr[7] = 4.00E+07 - cvf.MeanData_Q2[7];			cvf.dataQ2_prev[7] = 4.00E+07 - cvf.MeanData_Q2[7];
	cvf.dataQ2_curr[8] = 4.00E+07 - cvf.MeanData_Q2[8];			cvf.dataQ2_prev[8] = 4.00E+07 - cvf.MeanData_Q2[8];
	cvf.dataQ2_curr[9] = 4.00E+07 - cvf.MeanData_Q2[9];			cvf.dataQ2_prev[9] = 4.00E+07 - cvf.MeanData_Q2[9];
	cvf.dataQ2_curr[10] = 4.00E+07 - cvf.MeanData_Q2[10];		cvf.dataQ2_prev[10] = 4.00E+07 - cvf.MeanData_Q2[10];
	cvf.dataQ2_curr[11] = 4.00E+07 - cvf.MeanData_Q2[11];		cvf.dataQ2_prev[11] = 4.00E+07 - cvf.MeanData_Q2[11];
	cvf.dataQ2_curr[12] = 4.00E+07 - cvf.MeanData_Q2[12];		cvf.dataQ2_prev[12] = 4.00E+07 - cvf.MeanData_Q2[12];
	cvf.dataQ2_curr[13] = 4.00E+07 - cvf.MeanData_Q2[13];		cvf.dataQ2_prev[13] = 4.00E+07 - cvf.MeanData_Q2[13];
	cvf.dataQ2_curr[14] = 4.00E+07 - cvf.MeanData_Q2[14];		cvf.dataQ2_prev[14] = 4.00E+07 - cvf.MeanData_Q2[14];
	cvf.dataQ2_curr[15] = 4.00E+07 - cvf.MeanData_Q2[15];		cvf.dataQ2_prev[15] = 4.00E+07 - cvf.MeanData_Q2[15];
	cvf.dataQ2_curr[16] = 4.00E+07 - cvf.MeanData_Q2[16];		cvf.dataQ2_prev[16] = 4.00E+07 - cvf.MeanData_Q2[16];
	cvf.dataQ2_curr[17] = 4.00E+07 - cvf.MeanData_Q2[17];		cvf.dataQ2_prev[17] = 4.00E+07 - cvf.MeanData_Q2[17];
	cvf.dataQ2_curr[18] = 4.00E+07 - cvf.MeanData_Q2[18];		cvf.dataQ2_prev[18] = 4.00E+07 - cvf.MeanData_Q2[18];
	cvf.dataQ2_curr[19] = 4.00E+07 - cvf.MeanData_Q2[19];		cvf.dataQ2_prev[19] = 4.00E+07 - cvf.MeanData_Q2[19];
	cvf.dataQ2_curr[20] = 4.00E+07 - cvf.MeanData_Q2[20];		cvf.dataQ2_prev[20] = 4.00E+07 - cvf.MeanData_Q2[20];
	cvf.dataQ2_curr[21] = 4.00E+07 - cvf.MeanData_Q2[21];		cvf.dataQ2_prev[21] = 4.00E+07 - cvf.MeanData_Q2[21];
	cvf.dataQ2_curr[22] = 4.00E+07 - cvf.MeanData_Q2[22];		cvf.dataQ2_prev[22] = 4.00E+07 - cvf.MeanData_Q2[22];
	cvf.dataQ2_curr[23] = 4.00E+07 - cvf.MeanData_Q2[23];		cvf.dataQ2_prev[23] = 4.00E+07 - cvf.MeanData_Q2[23];
	cvf.dataQ2_curr[24] = 4.00E+07 - cvf.MeanData_Q2[24];		cvf.dataQ2_prev[24] = 4.00E+07 - cvf.MeanData_Q2[24];
	cvf.dataQ2_curr[25] = 4.00E+07 - cvf.MeanData_Q2[25];		cvf.dataQ2_prev[25] = 4.00E+07 - cvf.MeanData_Q2[25];
	#endif
}
void ExpCurvefitting(void)
{
	/* Log */
	for(uint8_t i = 1; i < 25+1; i ++)
	{
		if(i == 1)
		{
			cvf.Sum_lnY_Q1 = 0;

			#if FDC_ADD
			cvf.Sum_lnY_Q2 = 0;
			#endif
		}
		cvf.lnY_Q1[i] = log(cvf.MeanData_Q1[i]);
		cvf.Sum_lnY_Q1 += cvf.lnY_Q1[i];

		/**/
		#if FDC_ADD
		cvf.lnY_Q2[i] = log(cvf.MeanData_Q2[i]);
		cvf.Sum_lnY_Q2 += cvf.lnY_Q2[i];
		#endif
	}
	cvf.Mean_lnY_Q1 = cvf.Sum_lnY_Q1 *0.04;

	#if FDC_ADD
	cvf.Mean_lnY_Q2 = cvf.Sum_lnY_Q2 *0.04;
	#endif

	for(uint8_t i = 1; i < 25+1; i ++)
	{
		if(i == 1)
		{
			cvf.sumNumnerator_Q1 = 0;
			cvf.sumDenominator_Q1 = 0;

			#if FDC_ADD
			cvf.sumNumnerator_Q2 = 0;
			cvf.sumDenominator_Q2 = 0;
			#endif
		}
		cvf.err_X[i] = cvf.X[i] - cvf.Mean_X;

		cvf.err_lnY_Q1[i] = cvf.lnY_Q1[i] - cvf.Mean_lnY_Q1;
		cvf.sumNumnerator_Q1 += cvf.err_lnY_Q1[i]*cvf.err_X[i];
		cvf.sumDenominator_Q1 += cvf.err_X[i]*cvf.err_X[i];

		#if FDC_ADD
		cvf.err_lnY_Q2[i] = cvf.lnY_Q2[i] - cvf.Mean_lnY_Q2;
		cvf.sumNumnerator_Q2 += cvf.err_lnY_Q2[i]*cvf.err_X[i];
		cvf.sumDenominator_Q2 += cvf.err_X[i]*cvf.err_X[i];
		#endif
	}

	cvf.Coeff_Q1 = cvf.sumNumnerator_Q1 / cvf.sumDenominator_Q1;
	cvf.tmpCoeff_Q1 = abs((cvf.sumNumnerator_Q1*100000000) / cvf.sumDenominator_Q1);

	#if FDC_ADD
	cvf.Coeff_Q2 = cvf.sumNumnerator_Q2 / cvf.sumDenominator_Q2;
	cvf.tmpCoeff_Q2 = abs((cvf.sumNumnerator_Q2*100000000) / cvf.sumDenominator_Q2);
	#endif

	#if 0
	if(cvf.Coeff_Q1 > -0.008)
	{
		cvf.Coeff_Q1 = -0.008;
	}
	else if(cvf.Coeff_Q1 < -0.08)
	{
		cvf.Coeff_Q1 = -0.08;
	}
	#endif

	cvf.Offset_Q1 = cvf.Mean_lnY_Q1 - (cvf.Coeff_Q1*cvf.Mean_X);

	#if FDC_ADD
	cvf.Offset_Q2 = cvf.Mean_lnY_Q2 - (cvf.Coeff_Q2*cvf.Mean_X);
	#endif

	cvf.logY_Q1 = cvf.MeanData_Q1[0] - dComps.fdc_Q1;
	if(cvf.logY_Q1 <= 1)
	{
		cvf.logY_Q1 = 1;
	}

	#if FDC_ADD
	cvf.logY_Q2 = cvf.MeanData_Q2[0] - dComps.fdc_Q2;
	if(cvf.logY_Q2 <= 1)
	{
		cvf.logY_Q2 = 1;
	}
	#endif

	cvf.distancePredict_Q1 = (1/cvf.Coeff_Q1)*(log(cvf.logY_Q1) - cvf.Offset_Q1);
	#if 1
	if(cvf.distancePredict_Q1 <= 0)
	{
		cvf.distancePredict_Q1 = 0;
	}
	else if(cvf.distancePredict_Q1 > 500)
	{
		cvf.distancePredict_Q1 = 500;
	}
	#endif

	#if FDC_ADD
	cvf.distancePredict_Q2 = (1/cvf.Coeff_Q2)*(log(cvf.logY_Q2) - cvf.Offset_Q2);
	#if 1
	if(cvf.distancePredict_Q2 <= 0)
	{
		cvf.distancePredict_Q2 = 0;
	}
	else if(cvf.distancePredict_Q2 > 500)
	{
		cvf.distancePredict_Q2 = 500;
	}
	#endif
	#endif
}
void CalibrateDistance(void)
{
    /* New fitted model followed by its affine correction, applied exactly once. */
    dist.Q1 = Sensor1CalibratedCapDistance(dComps.fdc_Q1, 500U);

    #if FDC_ADD
    dist.Q2 = (12.89*exp(-0.000007192*(dComps.fdc_Q2 - 2*Offset1)) + 220.1*exp(0.0001462*(dComps.fdc_Q2 - 2*Offset2)));
    #endif
}
void CalibrateDistanceNormalized(void)
{
    /* Match the active calibrated CAP model, retaining the existing350mm limit. */
    dist.Normalized_Q1 = Sensor1CalibratedCapDistance(dComps.fdc_Q1, 350U);

    #if FDC_ADD
    dist.Q2 = (12.89*exp(-0.000007192*(dComps.fdc_Q2 - 2*Offset1)) + 220.1*exp(0.0001462*(dComps.fdc_Q2 - 2*Offset2)));
    #endif
}
void InitializeCANProtocol(void)
{
	state.onOff_ID = CAN_CH0;
	state.onOff_sensing = 1;
	/* Continue automatic curve collection from the measured initial seeds. */
	state.onOff_cftool = 1;
	/* 0: DisatanceData, 1: RawData */
	state.onOff_rawData = 0;
	state.onOff_reset = 0;
}
void _start(void) {
if (Sensor1CorrectToFDistance(0U,1U) != 0U) test_failures |= 1U;
if (Sensor1CorrectToFDistance(200U,0U) != 0U) test_failures |= 2U;
if (Sensor1CorrectToFDistance(1U,1U) != 0U) test_failures |= 4U;
if (Sensor1CorrectToFDistance(200U,1U) != 245U) test_failures |= 8U;
if (Sensor1CorrectToFDistance(65535U,1U) != 0U) test_failures |= 16U;
if (Sensor1CalibratedCapDistance(0U,350U) != 4U) test_failures |= 32U;
if (Sensor1CalibratedCapDistance(0U,500U) != 4U) test_failures |= 32U;
if (Sensor1CalibratedCapDistance(40000240U,350U) != 279U) test_failures |= 32U;
if (Sensor1CalibratedCapDistance(40000240U,500U) != 279U) test_failures |= 32U;
if (Sensor1CalibratedCapDistance(39998515U,350U) != 147U) test_failures |= 32U;
if (Sensor1CalibratedCapDistance(39998515U,500U) != 147U) test_failures |= 32U;
if (Sensor1CalibratedCapDistance(39785013U,350U) != 4U) test_failures |= 32U;
if (Sensor1CalibratedCapDistance(39785013U,500U) != 4U) test_failures |= 32U;
if (Sensor1CalibratedCapDistance(39995240U,350U) != 104U) test_failures |= 32U;
if (Sensor1CalibratedCapDistance(39995240U,500U) != 104U) test_failures |= 32U;
if (Sensor1CalibratedCapDistance(4294967295U,350U) != 350U) test_failures |= 32U;
if (Sensor1CalibratedCapDistance(4294967295U,500U) != 500U) test_failures |= 32U;
InitializeCurveFitting();
if (sizeof(cvf.X)/sizeof(cvf.X[0]) != 30U || cvf.X[1] != 270U || cvf.X[25] != 30U) test_failures |= 64U;
if (cvf.MeanData_Q1[0] != 40000240) test_failures |= 128U;
if (cvf.MeanData_Q1[1] != 180 || cvf.dataQ1_curr[1] != 40000060U || cvf.dataQ1_prev[1] != 40000060U) test_failures |= 256U;
if (cvf.MeanData_Q1[2] != 86 || cvf.dataQ1_curr[2] != 40000154U || cvf.dataQ1_prev[2] != 40000154U) test_failures |= 256U;
if (cvf.MeanData_Q1[3] != 230 || cvf.dataQ1_curr[3] != 40000010U || cvf.dataQ1_prev[3] != 40000010U) test_failures |= 256U;
if (cvf.MeanData_Q1[4] != 247 || cvf.dataQ1_curr[4] != 39999993U || cvf.dataQ1_prev[4] != 39999993U) test_failures |= 256U;
if (cvf.MeanData_Q1[5] != 351 || cvf.dataQ1_curr[5] != 39999889U || cvf.dataQ1_prev[5] != 39999889U) test_failures |= 256U;
if (cvf.MeanData_Q1[6] != 414 || cvf.dataQ1_curr[6] != 39999826U || cvf.dataQ1_prev[6] != 39999826U) test_failures |= 256U;
if (cvf.MeanData_Q1[7] != 554 || cvf.dataQ1_curr[7] != 39999686U || cvf.dataQ1_prev[7] != 39999686U) test_failures |= 256U;
if (cvf.MeanData_Q1[8] != 605 || cvf.dataQ1_curr[8] != 39999635U || cvf.dataQ1_prev[8] != 39999635U) test_failures |= 256U;
if (cvf.MeanData_Q1[9] != 817 || cvf.dataQ1_curr[9] != 39999423U || cvf.dataQ1_prev[9] != 39999423U) test_failures |= 256U;
if (cvf.MeanData_Q1[10] != 942 || cvf.dataQ1_curr[10] != 39999298U || cvf.dataQ1_prev[10] != 39999298U) test_failures |= 256U;
if (cvf.MeanData_Q1[11] != 1273 || cvf.dataQ1_curr[11] != 39998967U || cvf.dataQ1_prev[11] != 39998967U) test_failures |= 256U;
if (cvf.MeanData_Q1[12] != 1430 || cvf.dataQ1_curr[12] != 39998810U || cvf.dataQ1_prev[12] != 39998810U) test_failures |= 256U;
if (cvf.MeanData_Q1[13] != 1725 || cvf.dataQ1_curr[13] != 39998515U || cvf.dataQ1_prev[13] != 39998515U) test_failures |= 256U;
if (cvf.MeanData_Q1[14] != 2119 || cvf.dataQ1_curr[14] != 39998121U || cvf.dataQ1_prev[14] != 39998121U) test_failures |= 256U;
if (cvf.MeanData_Q1[15] != 2645 || cvf.dataQ1_curr[15] != 39997595U || cvf.dataQ1_prev[15] != 39997595U) test_failures |= 256U;
if (cvf.MeanData_Q1[16] != 3432 || cvf.dataQ1_curr[16] != 39996808U || cvf.dataQ1_prev[16] != 39996808U) test_failures |= 256U;
if (cvf.MeanData_Q1[17] != 4307 || cvf.dataQ1_curr[17] != 39995933U || cvf.dataQ1_prev[17] != 39995933U) test_failures |= 256U;
if (cvf.MeanData_Q1[18] != 5459 || cvf.dataQ1_curr[18] != 39994781U || cvf.dataQ1_prev[18] != 39994781U) test_failures |= 256U;
if (cvf.MeanData_Q1[19] != 7038 || cvf.dataQ1_curr[19] != 39993202U || cvf.dataQ1_prev[19] != 39993202U) test_failures |= 256U;
if (cvf.MeanData_Q1[20] != 9078 || cvf.dataQ1_curr[20] != 39991162U || cvf.dataQ1_prev[20] != 39991162U) test_failures |= 256U;
if (cvf.MeanData_Q1[21] != 12242 || cvf.dataQ1_curr[21] != 39987998U || cvf.dataQ1_prev[21] != 39987998U) test_failures |= 256U;
if (cvf.MeanData_Q1[22] != 17028 || cvf.dataQ1_curr[22] != 39983212U || cvf.dataQ1_prev[22] != 39983212U) test_failures |= 256U;
if (cvf.MeanData_Q1[23] != 24066 || cvf.dataQ1_curr[23] != 39976174U || cvf.dataQ1_prev[23] != 39976174U) test_failures |= 256U;
if (cvf.MeanData_Q1[24] != 34688 || cvf.dataQ1_curr[24] != 39965552U || cvf.dataQ1_prev[24] != 39965552U) test_failures |= 256U;
if (cvf.MeanData_Q1[25] != 53939 || cvf.dataQ1_curr[25] != 39946301U || cvf.dataQ1_prev[25] != 39946301U) test_failures |= 256U;
dComps.fdc_Q1 = 39995240U; ExpCurvefitting();
if (!isfinite(cvf.Coeff_Q1) || fabs(cvf.Coeff_Q1 - (-0.023668984352662868)) > 1e-10 || fabs(cvf.Offset_Q1 - (11.144119314977052)) > 1e-9) test_failures |= 512U;
CalibrateDistance(); CalibrateDistanceNormalized(); if (dist.Q1 != 104U || dist.Normalized_Q1 != 104U) test_failures |= 1024U;
InitializeCANProtocol(); if (state.onOff_cftool != 1U) test_failures |= 2048U;
test_done = 0xC0DEU; __asm volatile("bkpt #0"); for (;;) {} }