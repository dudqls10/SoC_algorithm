typedef struct
{
	uint32_t fdc_Q1;
	uint32_t fdc_Q2;

	int32_t drift_fdcQ1;
	int32_t drift_fdcQ2;

	uint32_t Vfdc_Q1;
	uint32_t Vfdc_Q2;

	uint32_t Tfdc_Q1;
	int32_t Tfdc_tmp[2];

	uint8_t temp_fdcQ1[4];
	uint8_t temp_fdcQ2[4];

	int64_t fdcCoeff_Q1;
	int64_t fdcCoeff_Q2;
	int64_t tmp117_fdcCoeff_Q1;

	int32_t PMfdcCoeff_Q1;
	int32_t PMfdcCoeff_Q2;

	int64_t fdcOffset_Q1;
	int64_t fdcOffset_Q2;
	int64_t tmp117_fdcOffset_Q1;

	int64_t PMfdcOffset_Q1;
	int64_t PMfdcOffset_Q2;

	int64_t sumNumnerator_fdcQ1;
	int64_t sumNumnerator_fdcQ2;

	int64_t sumNumnerator_PMfdcQ1;
	int64_t sumNumnerator_PMfdcQ2;

	int64_t sumDenominator_fdcQ1;
	int64_t sumDenominator_fdcQ2;

	int64_t sumDenominator_PMfdcQ1;
	int64_t sumDenominator_PMfdcQ2;

	int64_t sumNumnerator_tmp117;
	int64_t sumDenominator_tmp117;

}driftCompensation;
typedef struct
{
	/* Nextlab FDC */

	uint32_t fdc300[100];
	uint32_t fdc302[100];
	uint32_t fdc100[100];
	uint32_t fdc102[100];

	uint32_t PMfdc300[50];
	uint32_t PMfdc302[50];
	uint32_t PMfdc100[50];
	uint32_t PMfdc102[50];

	uint64_t sum_fdc300;
	uint64_t sum_fdc302;
	uint64_t sum_fdc100;
	uint64_t sum_fdc102;

	uint64_t sum_PMfdc300;
	uint64_t sum_PMfdc302;
	uint64_t sum_PMfdc100;
	uint64_t sum_PMfdc102;

	uint64_t mean_fdc300;
	uint64_t mean_fdc302;
	uint64_t mean_fdc100;
	uint64_t mean_fdc102;

	uint64_t mean_PMfdc300;
	uint64_t mean_PMfdc302;
	uint64_t mean_PMfdc100;
	uint64_t mean_PMfdc102;

	int64_t err_fdc300[100];
	int64_t err_fdc302[100];
	int64_t err_fdc100[100];
	int64_t err_fdc102[100];

	int32_t err_PMfdc300[50];
	int32_t err_PMfdc302[50];
	int32_t err_PMfdc100[50];
	int32_t err_PMfdc102[50];


	/* Temperature */

	uint32_t tmp117[50];
	int32_t sum_tmp117;
	int32_t mean_tmp117;
	int32_t err_tmp117[50];

	uint32_t tmp117_f302[50];
	uint64_t sum_tmp117_f302;
	uint64_t mean_tmp117_f302;
	uint32_t err_tmp117_f302[50];


	uint32_t DI_fdcQ1[100];
	uint32_t DI_fdcQ2[100];
	uint32_t DI_tmp117[50];

	uint32_t DI_RfdcQ1[50];
	uint32_t DI_RfdcQ2[50];

	int64_t Derivative_fdcQ1[100];
	int64_t Derivative_fdcQ2[100];
	int64_t Derivative_tmp117[50];

	int32_t Derivative_RfdcQ1[50];
	int32_t Derivative_RfdcQ2[50];

	int32_t Integral_fdcQ1[20];
	int32_t Integral_fdcQ2[20];
	int32_t Integral_tmp117[20];

	int32_t Integral_RfdcQ1[20];
	int32_t Integral_RfdcQ2[20];

	int64_t sumIntegral_fdcQ1;
	int64_t sumIntegral_fdcQ2;
	int64_t sumIntegral_tmp117;

	int64_t sumIntegral_RfdcQ1;
	int64_t sumIntegral_RfdcQ2;

	int32_t DOinit_fdcQ1[100];
	int32_t DOinit_fdcQ2[100];

	uint64_t DO_fdcQ1[100];
	uint64_t DO_fdcQ2[100];

	int32_t DO_RfdcQ1[100];
	int32_t DO_RfdcQ2[100];

	uint64_t sumDO_fdcQ1;
	uint64_t sumDO_fdcQ2;

	uint32_t sumDO_RfdcQ1;
	uint32_t sumDO_RfdcQ2;

	uint64_t sumSTDDO_fdcQ1;
	uint64_t sumSTDDO_fdcQ2;

	uint64_t sumSTDDO_RfdcQ1;
	uint64_t sumSTDDO_RfdcQ2;

	int32_t meanDOinit_fdcQ1;
	int32_t meanDOinit_fdcQ2;

	int32_t meanDO_fdcQ1;
	int32_t meanDO_fdcQ2;

	int32_t meanDO_RfdcQ1;
	int32_t meanDO_RfdcQ2;

	uint32_t stdDOinit_fdcQ1;
	uint32_t stdDOinit_fdcQ2;

	uint32_t stdDO_fdcQ1;
	uint32_t stdDO_fdcQ2;

	uint32_t stdDO_RfdcQ1;
	uint32_t stdDO_RfdcQ2;

}samples;
typedef samples ProductionSample;
