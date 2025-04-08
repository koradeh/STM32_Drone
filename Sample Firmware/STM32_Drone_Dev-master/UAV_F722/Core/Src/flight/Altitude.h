/*
 * Altitude.h
 *
 *  Created on: Oct 2, 2024
 *      Author: Vincent
 */

#ifndef INC_ALTITUDE_H_
#define INC_ALTITUDE_H_
#include "../Src/rx/rx.h"
#include "../Src/flight/defines.h"
#include "../Src/CommonMath/maths.h"
#include "../Src/sensors/RangeFinder.h"
#include "../Src/sensors/opFlow.h"
#include "Flight_System.h"

#include "../sensors/BMP280.h"
#include "ICM42688P.h"
#include "../Src/CommonMath/filters.h"
#include "math.h"
#include <stdbool.h>

#define baroFreq		(PID_Freq / 80.0f)
#define baroMA_window 	20
#define baroALT_HZ 		50
#define TASK_ALTITUDE_RATE_HZ 100
#define baroCalibration_Cycle 100

#define ALTErrorThreshold	10 //10cm
#define ALTRateErrorThreshold 150

typedef struct
{
	float RFAltitude;
	float RFAltitudeDerivative;

	float baroGroundALT;
	float rawAltitude;
	float Altitude;
	float rawVz;
	float veloCompleGain;

	float Vz;
	float AccelAltitude;

	float derivative;
	pt2Filter_t ALT_LPF;
	pt2Filter_t altitudeDerivativeLpf;
	laggedMovingAverage_t baroDerivative_MA;
	float baroALTDerivative_buf[baroMA_window];

}Altitude_Data_t;

extern Altitude_Data_t altitude;

void Alt_Init(void);
bool baroCalibrate(float altitude);
bool isBaroReady(void);
void calculateEstimatedAltitude(Baro_HandleTypedef *baro, bool ARMED);
float getAltitudeDerivative(void);
float getAltitudeCm(void);
bool haveRFDist(void);
void Altitude_Rate_Data_Fusion(Altitude_Data_t *alt, ICM42688_AccData_t *accData, Baro_HandleTypedef *dev, bool baro_on, bool ARMED);
void Altitude_Data_Fusion(Altitude_Data_t *alt, ICM42688_AccData_t *accData);

float altitudeSetpointReset(float currentSetpoint);
float altitudeSetpointGenerate(float currentSetpoint);
float altitudePidCalculate (float *desiredAltitude);
float altitudeControl(void);
#endif /* INC_ALTITUDE_H_ */
