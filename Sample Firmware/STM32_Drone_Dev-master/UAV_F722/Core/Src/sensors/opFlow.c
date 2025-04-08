/*
 * opFlow.c
 *
 *  Created on: Nov 4, 2024
 *      Author: Vincent
 */

#include "opFlow.h"

opFlow_Filter_t opFlow;

void opFlow_init(void)
{
	mtf02_init(&opFlow.mtf02,&OFSensorBusyType);

	float opFlowPT2X_Gain = pt2FilterGain(opFlowPT2_CutoffHZ, 1.0f/MTF_SampleRate);
	pt2FilterInit(&opFlow.opFlowPT2X,opFlowPT2X_Gain);

	float opFlowPT2Y_Gain = pt2FilterGain(opFlowPT2_CutoffHZ, 1.0f/MTF_SampleRate);
	pt2FilterInit(&opFlow.opFlowPT2Y,opFlowPT2Y_Gain);

	float opFlowPT2Z_Gain = pt2FilterGain(opFlowPT2_CutoffHZ, 1.0f/MTF_SampleRate);
	pt2FilterInit(&opFlow.opFlowPT2Z,opFlowPT2Z_Gain);

	float opFlowPT2ZDerivative_Gain = pt2FilterGain(opFlowPT2_CutoffHZ/10.0f, 1.0f/MTF_SampleRate);
	pt2FilterInit(&opFlow.opFlowPT2ZDerivative,opFlowPT2ZDerivative_Gain);


	laggedMovingAverageInit(&opFlow.opFlowZMA, opFlowZ_Window_Size, opFlow.opFlowZMAbuf); //downsample and smoothen baro input

	laggedMovingAverageInit(&opFlow.opFlowZDerivativeMA, opFlowZDerivative_Window_Size, opFlow.opFlowZDerivativeMAbuf); //downsample and smoothen baro input

}




static float previousRFAltitudeCm = 0.0f;

void opFlow_Data_Process(void)
{
	HAL_UARTEx_ReceiveToIdle_DMA(opFlow.mtf02.huart, opFlow.mtf02.MTF_UartWorkingBuffer, sizeof(opFlow.mtf02.MTF_UartWorkingBuffer));

	parse_mtf02_data(&opFlow.mtf02);
	//Process opFlow Distance in CM
	if((opFlow.mtf02.MTF_DistStatus ==1)&&(opFlow.mtf02.MTF_DistMM!=0))
	{
		//filter and MA also to further reduce the noise in derivative
		float distCMf = pt2FilterApply(&opFlow.opFlowPT2X,opFlow.mtf02.MTF_DistMM/10.0f);
		opFlow.distCMf = laggedMovingAverageUpdate(&opFlow.opFlowZMA,distCMf);


		//calculate derivative
		float deltaCm = opFlow.distCMf - previousRFAltitudeCm;

		if(ABS(deltaCm)>15)
		{
			deltaCm=0;
		}

		//filter then MA due to poor sensor quality -> high fluctuation result in spikes in derivative
		float RF_CMs = deltaCm * MTF_SampleRate;
		opFlow.distDerivativeCm = pt2FilterApply(&opFlow.opFlowPT2ZDerivative, RF_CMs);
		opFlow.distDerivativeCm = laggedMovingAverageUpdate(&opFlow.opFlowZDerivativeMA,opFlow.distDerivativeCm);
		previousRFAltitudeCm = opFlow.distCMf;


		opFlow.xFlowf = pt2FilterApply(&opFlow.opFlowPT2X,(float)opFlow.mtf02.MTF_Flow_velX);
		opFlow.yFlowf = pt2FilterApply(&opFlow.opFlowPT2Y,(float)opFlow.mtf02.MTF_Flow_velY);
		sysHealth.opFlowHealthy = true;
	}
	//First check the optical flow validity
	else if(opFlow.mtf02.MTF_FlowStatus != 1)
	{
		sysHealth.opFlowHealthy = false;
	}




}
