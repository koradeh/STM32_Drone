/*
 * opFlow.h
 *
 *  Created on: Nov 4, 2024
 *      Author: Vincent
 */

#ifndef SRC_SENSORS_OPFLOW_H_
#define SRC_SENSORS_OPFLOW_H_

#include "../CommonMath/filters.h"
#include "../flight/ResourceDefine.h"
#include "../flight/defines.h"
#include "../drivers/MTF02.h"
#include "../flight/Flight_System.h"
#include "../CommonMath/maths.h"


#define opFlowZDerivative_Window_Size 10
#define opFlowZ_Window_Size 20
typedef struct
{
	MTF_Data_t	mtf02;
	pt2Filter_t opFlowPT2X;
	pt2Filter_t opFlowPT2Y;
	pt2Filter_t opFlowPT2Z;
	laggedMovingAverage_t opFlowZMA;
	pt2Filter_t opFlowPT2ZDerivative;
	laggedMovingAverage_t opFlowZDerivativeMA;

	float opFlowZDerivativeMAbuf[opFlowZDerivative_Window_Size];
	float opFlowZMAbuf[opFlowZ_Window_Size];

	float	xFlowf;
	float yFlowf;
	float distCMf;

	float distDerivativeCm;
}opFlow_Filter_t;


extern opFlow_Filter_t opFlow;

void opFlow_init(void);
void opFlow_Data_Process(void);
#endif /* SRC_SENSORS_OPFLOW_H_ */
