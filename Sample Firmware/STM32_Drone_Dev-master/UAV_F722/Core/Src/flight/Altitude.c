/*
 * Altitude.c
 *
 *  Created on: Oct 2, 2024
 *      Author: Vincent
 */

#include "Altitude.h"

#define BIAS_THROTTLE ((Hoover_Throttle -988) * 2.0f)
Altitude_Data_t altitude;


static uint16_t calibrationCycles = 0;      // baro calibration = get new ground pressure value
static uint16_t calibrationCycleCount = 0;

static float baroGroundAltitude = 0.0f;
static bool baroCalibrated = false;
static bool baroReady = false;

static bool RFGood = false;

static float zeroedAltitudeCm = 0.0f;
static float zeroedAltitudeDerivative = 0.0f;

static float previousZeroedAltitudeCm = 0.0f;

static bool useOpticalFlowRangeFinder = false;
static bool useLiDarRangeFinder = false;

bool RFRateisValid = false;

void Alt_Init(void)
{
	altitude.RFAltitude =0;
	altitude.baroGroundALT = 0;
	baroGroundAltitude = 0;
	baroCalibrated = false;
	baroReady = false;
	calibrationCycleCount = 0;

	//Filters Init
	float ALT_LPF_Gain = pt2FilterGain (baroALT_HZ, baroFreq);
	pt2FilterInit(&altitude.ALT_LPF, ALT_LPF_Gain);			//Filter baro alt input

	float altitudeDerivativeLpf_Gain = pt2FilterGain(TASK_ALTITUDE_RATE_HZ,baroFreq);
	pt2FilterInit(&altitude.altitudeDerivativeLpf,altitudeDerivativeLpf_Gain);

	laggedMovingAverageInit(&altitude.baroDerivative_MA, baroMA_window, altitude.baroALTDerivative_buf); //downsample and smoothen baro input

	rangeFinderFilterInit();
}

bool baroCalibrate(float baroalt)
{
    baroGroundAltitude += baroalt;
    calibrationCycleCount++;

    if (calibrationCycleCount >= calibrationCycles)
    {
        baroGroundAltitude /= calibrationCycleCount;  // simple average
        baroCalibrated = true;
        calibrationCycleCount = 0;
        baroReady = true;
        altitude.baroGroundALT = baroGroundAltitude;
    }
    return baroCalibrated;
}

bool isBaroReady(void)
{
	return baroReady;
}



static bool calculateHeightFromGround(void)
{
	//check if the altitude is within range
	if((getAltitudeCm() <= 1200) || (sysHealth.RangeFinderHealthy) || ((sysHealth.opFlowHealthy))) //need to add flag to check optical flow RF health
	{
		if((sysHealth.RangeFinderHealthy))
		{
			altitude.RFAltitude = RangeFinder.LiDar.Distance;

			useOpticalFlowRangeFinder = false;
			useLiDarRangeFinder = true;
			//implement later when have it
//			altitude.RFAltitudeDerivative = (altitude.RFAltitude - previousRFAltitudeCm)*PID_Freq;
//			previousRFAltitudeCm = altitude.RFAltitude;
			return 1;
		}
		else if((sysHealth.opFlowHealthy)&& (!(sysHealth.RangeFinderHealthy)))
		{
			altitude.RFAltitude = opFlow.distCMf;
			altitude.RFAltitudeDerivative = opFlow.distDerivativeCm;
			useOpticalFlowRangeFinder = true;
			useLiDarRangeFinder = false;
			return 1;
		}


	}
	useOpticalFlowRangeFinder = false;
	useLiDarRangeFinder = false;
	altitude.RFAltitudeDerivative = 0;
	return 0;
}

bool haveRFDist(void)
{
	return calculateHeightFromGround();
}

void calculateEstimatedAltitude(Baro_HandleTypedef *baro, bool ARMED)
{
	//Arming Handle
	static bool wasArmed = false;

	//GPS Altitude
    static float gpsAltCm = 0.0f; // will hold last value on transient loss of 3D fix
    static float gpsAltOffsetCm = 0.0f;

    //Baro Altitude
    static float baroAltOffsetCm = 0.0f;
    static float newBaroAltOffsetCm = 0.0f;


    float baroAltCm = 0.0f;
    //float gpsTrust = 0.3f; // if no pDOP value, use 0.3, intended range 0-1;


    bool haveBaroAlt = false; // true if baro exists and has been calibrated on power up
    bool haveGpsAlt = false; // true if GPS is connected and while it has a 3D fix, set each run to false


    //---------Algorithm Start---------------------------------
    //---------------------------------------------------------

    //update baro altitude
    if(isBaroReady()) {baroAltCm = baro->Altitude - altitude.baroGroundALT;}




    //---DISARMED---
    if(!ARMED)
    {
        if (wasArmed)
        { // things to run once, on disarming, after being armed
           // useZeroedGpsAltitude = false; // reset, and wait for valid GPS data to zero the GPS signal
            wasArmed = false;
        }

    	newBaroAltOffsetCm = 0.2f * baroAltCm + 0.8f * newBaroAltOffsetCm; // smooth some recent baro samples
       // displayAltitudeCm = baroAltCm - baroAltOffsetCm; // if no GPS altitude, show un-smoothed Baro altitude in OSD and sensors tab, using most recent offset.
    	zeroedAltitudeCm = 0.0f;
    }

    //---ARMED---
    else
    {
        if (!wasArmed)
        { // things to run once, on arming, after being disarmed
            baroAltOffsetCm = newBaroAltOffsetCm;
            wasArmed = true;
        }

        baroAltCm -= baroAltOffsetCm; // use smoothed baro with most recent zero from disarm period

        //if there is gps
//        if (useZeroedGpsAltitude) { // normal situation
//            zeroedAltitudeCm = gpsAltCm - gpsAltOffsetCm; // now that we have a GPS offset value, we can use it to zero relativeAltitude
//        }
        zeroedAltitudeCm = baroAltCm;
    }

    zeroedAltitudeCm = pt2FilterApply(&altitude.ALT_LPF, zeroedAltitudeCm);
    // NOTE: this filter must receive 0 as its input, for the whole disarmed time, to ensure correct zeroed values on arming

//    if (wasArmed) {
//        displayAltitudeCm = zeroedAltitudeCm; // while armed, show filtered relative altitude in OSD / sensors tab
//    }


    // *** calculate Vario signal

	zeroedAltitudeDerivative = (zeroedAltitudeCm - previousZeroedAltitudeCm) * baroFreq; // cm/s
	previousZeroedAltitudeCm = zeroedAltitudeCm;
	zeroedAltitudeDerivative = laggedMovingAverageUpdate(&altitude.baroDerivative_MA,zeroedAltitudeDerivative);

		//From betaflight, but it's not working well : pt2FilterApply(&altitude.altitudeDerivativeLpf, zeroedAltitudeDerivative);


}

float getAltitudeCm(void)
{
    return zeroedAltitudeCm;
}



void Altitude_Rate_Data_Fusion(Altitude_Data_t *alt, ICM42688_AccData_t *accData, Baro_HandleTypedef *dev, bool baro_on, bool ARMED)
{
	RFRateisValid = false;
	bool BarorateUpdated = false;
	alt->Vz += accData->AccZ * (1.0f/PID_Freq) * 100; // calculate m/s -> cm/s

	//if it's not calibrated it will calibrate
	if(!(isBaroReady()))
    {
		if(Process_BMP280(dev))
		{
			baroCalibrate(dev->Altitude);
		}
    }

	//altitude.Altitude += accData->AccZ * sq((1.0f/PID_Freq)) * 100.0f;
	static float ALT_Derivative_Error;

	//get RF data updated and generate flag
	RFGood = haveRFDist();

	//process baro altitude
	if(baro_on)
	{
		calculateEstimatedAltitude(dev, ARMED); //update altitude based on arming signal
		BarorateUpdated = true;
	}

	//Check if acceleration is near 0
//	if(RFGood)
//	{
//		if((ABS(accData->AccZ)<0.2)&&((ABS(getAltitudeDerivative())<10)||(ABS(altitude.RFAltitudeDerivative)<8)))
//		{
//			ALT_Derivative_Error = alt->Vz; //if the acceleration is quite and baro derivative is within oscillation range
//		}
//		else if((ABS(accData->AccZ)<0.2)&&((ABS(getAltitudeDerivative())>10)||(ABS(altitude.RFAltitudeDerivative)>8)))
//		{
//			if(ABS(altitude.RFAltitudeDerivative)<8)
//			{
//				ALT_Derivative_Error = alt->Vz-altitude.RFAltitudeDerivative;
//			}
//			else
//			{
//				ALT_Derivative_Error = alt->Vz-getAltitudeDerivative();
//			}
//		}
//		else
//		{
//			ALT_Derivative_Error = 0;
//		}
//	}
//	else
//	{
		if((ABS(accData->AccZ)<0.1)&&(ABS(getAltitudeDerivative())<5)) //stationary
		{
			ALT_Derivative_Error = alt->Vz; //if the acceleration is quite and baro derivative is within oscillation range
		}
		else if((ABS(accData->AccZ)<0.1)&&(ABS(alt->Vz)>5)) //has velocity no acceleration
		{
			ALT_Derivative_Error = alt->Vz - getAltitudeDerivative();
		}
		else //is accelerating
		{
			ALT_Derivative_Error=0;
		}
//	}

	alt->Vz -= (ALT_Derivative_Error * 0.0035f);

}


float getAltitudeDerivative(void)
{
	return zeroedAltitudeDerivative; // cm/s
}


//not used
void Altitude_Data_Fusion(Altitude_Data_t *alt, ICM42688_AccData_t *accData)
{
	//integrate first to get an estimate
	alt->AccelAltitude += alt->Vz * (1.0f/PID_Freq);

	float altErrorRF = 0;
	float altErrorBaro = 0;


	float RFGain = 0;
	float baroGain = 0;
	if(RFGood)
	{
		RFGain = altitude.RFAltitude/getAltitudeCm();
		baroGain= (1-RFGain);
		//RFGain = 1-baroGain;

		altErrorRF = alt->AccelAltitude -altitude.RFAltitude;
	}
	else
	{
		baroGain = 1;
	}
	altErrorBaro = alt->AccelAltitude - getAltitudeCm();



	//calibrate the altitude
	alt->AccelAltitude -= 0.05*((RFGain*altErrorRF + baroGain*altErrorBaro));


}



static bool altitudeRateCheck()
{
	return RFRateisValid;
}



float altitudeSetpointReset(float currentSetpoint)
{

	float newSetpoint = 0;

	if(RFGood)
	{
		newSetpoint = altitude.RFAltitude;
	}
	else
	{
		newSetpoint = getAltitudeCm();
	}

	return newSetpoint;


//	/*The altitude is controlled based on velocity, so every time when the velocity control is non-zero,
//	 * altitude hold is basically disabled and new setpoint will be generated when the velocity control is
//	 * back to zero. (first altitude setpoint generation task)
//	 *
//	 * Another case where ONLY the range finder is being used, and when flying over different structure, the measured
//	 * altitude will be changing all the time, it is ok for small distance, but for large difference like when flying over
//	 * a 15cm high cliff, the setpoint needs to be updated when the rate does not mathc the imu rate. (second generation task)
//	 *
//	 * GPS and Baro will be used for first task if range finder is not used.
//	 *
//	 */
//	float altitudeSetpoint = currentSetpoint;
//	float updatedSetpoint = 0;
//
//
//	float currentALT = getAltitudeCm(); //baro or gps altitude, always available
//
//	//First do altitude rate check to see if there is a shift in RF altitude when scanning high object
//	if(RFGood)
//	{
//		if(!altitudeRateCheck()) //if the rate does not match the imu rate, the update the setpoint
//		{
//			updatedSetpoint = altitude.RFAltitude;
//		}
//	}
//
//	//if RF is not valid, use gps
//	else if(sysHealth.gpsHealthy)
//	{
//
//	}
//	//if gps is not valid, use baro
//	else if(sysHealth.baroHealthy)
//	{
//
//	}
//	//if baro is not valid, disable altitude hold
//	else
//	{
//
//	}
//
//	//after getting a new setpoint (if any), do a sanity check again to make sure the setpoint doesn't update with small value
//	//Only update when system is getting big changes
//	if(isWithinTolerance(updatedSetpoint,currentSetpoint,10))
//	{
//
//	}

}

float altitudeSetpointGenerate(float currentSetpoint)
{
	float newSetpoint = 0;
	if( ! isWithinTolerance(rx.rxChannel.mapped_Channel[Thro],1000,10))
	{
		if(RFGood)
		{
			newSetpoint = altitude.RFAltitude;
		}
		else
		{
			newSetpoint = getAltitudeCm();
		}
	}
	else
	{
		newSetpoint = currentSetpoint;
	}
	return newSetpoint;
}


float altitudeErrorCM = 0;
float altitudePidCalculate (float *desiredAltitude)
{
	static float pidIntegral = 0.0f;




	//check which height to use first
	if(useOpticalFlowRangeFinder)
	{
		altitudeErrorCM = *desiredAltitude - altitude.RFAltitude;

		//in case the error exceed the threshold, reset the setpoint
		//the sudden change can be detected by comparing an accumulated sum average with the current value
		if((ABS(altitudeErrorCM)>ALTErrorThreshold)&&(ABS(altitude.RFAltitudeDerivative)>ALTRateErrorThreshold))
		{
			*desiredAltitude = altitudeSetpointReset(*desiredAltitude);
		}
	}
	else if(useLiDarRangeFinder)
	{

	}
	else
	{
		altitudeErrorCM = *desiredAltitude - getAltitudeCm();
	}



	//----------P Term----------
	const float pTerm = P_Alt * altitudeErrorCM;

	//----------I Term----------
    // input limit iTerm so that it doesn't grow fast with large errors
    // very important at the start if there are massive initial errors to prevent iTerm windup

    // much less iTerm change for errors greater than 2m, otherwise it winds up badly
    const float itermNormalRange = 200.0f; // 2m
    const float itermRelax = (fabsf(altitudeErrorCM) < itermNormalRange) ? 1.0f : 0.1f;

    pidIntegral += altitudeErrorCM * I_Alt * itermRelax * (1.0f/PID_Freq);
    const float iTerm = pidIntegral;

    //----------D Term----------
	// boost D by 'increasing apparent velocity' when vertical velocity exceeds 5 m/s ( D of 75 on defaults)
	// usually we don't see fast ascend/descend rates if the altitude hold starts under stable conditions
	// this is important primarily to arrest pre-existing fast drops or climbs at the start;

    float vel = getAltitudeDerivative(); // cm/s altitude derivative is always available
    const float kinkPoint = 500.0f; // velocity at which D should start to increase
    const float kinkPointAdjustment = kinkPoint * 2.0f; // Precompute constant
    const float sign = (vel > 0) ? 1.0f : -1.0f;
    if (fabsf(vel) > kinkPoint) {
        vel = vel * 3.0f - sign * kinkPointAdjustment;
    }

    const float dOut = vel * D_Alt;

    // ----------Feed Forward----------
    // if error is used, we get a 'free kick' in derivative from changes in the target value
    // but this is delayed by the smoothing, leading to lag and overshoot.
    // calculating feedforward separately avoids the filter lag.
    // Use user's D gain for the feedforward gain factor, works OK with a scaling factor of 0.01
    // A commanded drop at 100cm/s will return feedforward of the user's D value. or 15 on defaults

    //const float fOut = pidCoefs->Kf * altHoldState.targetAltitudeAdjustRate;

    // to further improve performance...
    // adding a high-pass filtered amount of FF would give a boost when altitude changes were requested
    // this would offset motor lag and kick off some initial vertical acceleration.
    // this would be exactly what an experienced pilot would do.

    const float output = pTerm + iTerm - dOut; // + fOut

    return output;

}

float desiredAltitude = 0.0f;
float altitudeControl(void)
{

	//Calculate Required ALTHold Setpoint

	desiredAltitude = altitudeSetpointGenerate(desiredAltitude);

	//Calculate control throttle
	float altitudeControlThrottle = BIAS_THROTTLE + altitudePidCalculate(&desiredAltitude);

	return altitudeControlThrottle;


}
