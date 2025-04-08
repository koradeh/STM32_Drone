#include "main.h"
#include "stm32f7xx_hal.h"
#include "PID.h"
#include "Timers.h"


float Motor_Idle = 0.05 ; //idle percent ranging from 0~100%, meaning the motor will spin at this rate no matter how low the throttle is
float PIDLimit;





void PID_Init(PID_Variables *pid, float P, float I, float D, float Dmax, float I_min, float I_max) //should be placed at the end of initialization
{

	/*PID parameter Init*/
	pid->kp = P;
	pid->ki = I;
	pid->kd = D;
	pid->Iterm_Min = I_min;
	pid->Iterm_Max = I_max;
	pid->Setpoint = 0.0f;
	pid->Integral = 0.0f;
	pid->PrevError = 0.0f;
	pid->Dterm = 0.0f;

	if((D>0.0f) && (Dmax >= D))
	{
		pid->dmaxPercent = Dmax/D;
	}
	else
	{
		pid->dmaxPercent = 1.0f;
	}

	pid->dmaxGyroGain = D_MAX_GAIN_FACTOR * Dmax / Dmax_range_lpf;

	/*Filter Init*/


	laggedMovingAverageInit(&pid->gyroDtermDownsample, gyroDownsample_size, pid->buf); //downsample dterm input

	float DtermFilterGain = pt2FilterGain(Dterm_PT2, (1.0f/PID_Freq)); //Init PT1 filter for dterm
	pt2FilterInit(&pid->PT2Filter, DtermFilterGain);

	float rateDterm_lpf1_Gain = pt1FilterGain(rateDterm_lp1, (1.0f/PID_Freq));
	pt1FilterInit(&pid->GyroLPF1, rateDterm_lpf1_Gain); //dterm input

	float Dmax_range_lpf_Gain = pt2FilterGain(Dmax_range_lpf, 1.0f/PID_Freq);
	pt2FilterInit(&pid->DmaxRangeLPF, Dmax_range_lpf_Gain);// dmax range filter

	float Dmax_lpf_Gain = pt2FilterGain(Dmax_lpf, 1.0f/PID_Freq);
	pt2FilterInit(&pid->DmaxLPF, Dmax_lpf_Gain);//dmax filter

	float PID_PT3FilterGain = pt3FilterGain(AngleRate_PT3, (1.0f/PID_Freq));//rateAngle
	pt3FilterInit(&pid->PID_PT3Filter, PID_PT3FilterGain);

	float YawLPFGain = pt1FilterGain(YawLPFhz, (1.0f/PID_Freq));
	pt1FilterInit(&pid->YawLPF, YawLPFGain);

	//Reset Time Variables
	pid->Current_Tick= 0.0f;
	pid->dt = 0.0f;
}

float pidLevelSetpoint (PID_Variables *pid, float setpoint, uint16_t angle)
{
	//implement FF later;

	const float errorAngle = setpoint - angle;

	float angleRate = errorAngle * pid->kp;

	//implement minimise cross-axis wobble later for alt hold

	angleRate = pt3FilterApply(&pid->PID_PT3Filter,angleRate);

	return angleRate;
}

float PID_Equation (PID_Variables *PID_Name, float Setpoint, float InputMeasurement, float dt, int axis)
{

	PID_Name->Setpoint = Setpoint;
	PID_Name->dt = dt;

	// -------------PID controller-------------


	//ERROR Calculation
	float Current_Error = PID_Name->Setpoint - InputMeasurement;


	//--------Proportional Calculation-----
	if(PID_Name->kp > 0.00001f)
	{
		PID_Name->Pterm = PID_Name->kp*(Current_Error);

		if(axis == yaw)
		{
			PID_Name->Pterm = pt1FilterApply(&PID_Name->YawLPF,PID_Name->Pterm);

			//P term rate limiter for yaw
			if (PID_Name->Pterm > MaxYawRate) PID_Name->Pterm = MaxYawRate;
			else if (PID_Name->Pterm < -MaxYawRate) PID_Name->Pterm = -MaxYawRate;
		}
	}

	//--------Integral Calculation---------
	if(PID_Name->ki > 0.00001f)
	{
		float itermLimit = PIDSUM_LIMIT * itermWindup * 0.01f;

		if(axis == yaw)
		{
			itermLimit = PIDSUM_LIMIT_YAW * itermWindup * 0.01f;
			//also make accelerator 0
		}

		PID_Name->Integral += Current_Error * (PID_Name->dt); //perform integration

		//I term relax (prevent I stack up) I*Integral = I_limit -> Integral_limit = I_Limit / I
		constrainf((PID_Name->ki * PID_Name->Integral), -itermLimit, itermLimit);

//		if(isnan(PID_Name->Integral))
//		{
//			PID_Name->Integral = 0.0f;
//		}
	}


	//------------Derivative Calculation--------

    //static float previousRawGyroRateDterm;	//unfiltered dterm
    float rateDtermraw;
    //Pre assign dterm rate, it is for dterm only, for P and I we use raw gyro
    if(PID_Name->kd > 0.0f)
    {
    	rateDtermraw = laggedMovingAverageUpdate(&PID_Name->gyroDtermDownsample, InputMeasurement);
        //previousRawGyroRateDterm = rateDtermraw;

    	rateDtermraw = pt1FilterApply(&PID_Name->GyroLPF1,rateDtermraw);//first lpf apply
    }

	if(PID_Name->kd > 0.00001f)
	{
		// Divide rate change by dT to get differential (ie dr/dt).
		const float rawD = -(rateDtermraw- PID_Name->previousGyroRateDterm)/(PID_Name->dt);
		float preTpaD = PID_Name->kd * rawD;


#if defined(Dmax_rateX) || defined(Dmax_rateY) || defined(Dmax_rateZ)

        float dMaxMultiplier = 1.0f;

        if(PID_Name->dmaxPercent > 1.0f)
        {
        	float dMaxGyroFactor = pt2FilterApply(&PID_Name->DmaxRangeLPF, rawD);
        	dMaxGyroFactor = fabsf(dMaxGyroFactor) * PID_Name->dmaxGyroGain;
            const float dMaxSetpointFactor = fabsf(0) * PID_Name->dmaxSetpointGain; //pidSetpointDelta = 0 because ff is disabled
            const float dMaxBoost = fmaxf(dMaxGyroFactor, dMaxSetpointFactor);

            // dMaxBoost starts at zero, and by 1.0 we get Dmax, but it can exceed 1.
            dMaxMultiplier += (PID_Name->dmaxPercent - 1.0f) * dMaxBoost;
            dMaxMultiplier = pt2FilterApply(&PID_Name->DmaxLPF, dMaxMultiplier);

            // limit the gain to the fraction that DMax is greater than Min
            dMaxMultiplier = MIN(dMaxMultiplier, PID_Name->dmaxPercent);
        }
        // Apply the gain that increases D towards Dmax
        preTpaD *= dMaxMultiplier;

#endif


		PID_Name->Dterm = pt2FilterApply(&PID_Name->PT2Filter, preTpaD); //dterm filter
	}

//
//	if (ABS(PID_Name->Dterm) >= 200)
//	{
//		PID_Name->Dterm = PID_Name->PrevError;
//	}

	//Output Calculation
	//kp * e(t)  +  ki * Integral( e(t)*dt )  +  kd * delta(e(t)) / delta (t)
	float PIDOutput = PID_Name->Pterm + PID_Name->ki * PID_Name->Integral + PID_Name->Dterm;


	//Update the current error to previous error
	PID_Name->PrevError =PID_Name->Dterm ;
	PID_Name->previousGyroRateDterm = rateDtermraw;

	return PIDOutput;
}


//Can be apply later for smoother control
//static float accelerationLimit(int axis, float currentPidSetpoint)
//{
//    static float previousSetpoint[XYZ_AXIS_COUNT];
//    const float currentVelocity = currentPidSetpoint - previousSetpoint[axis];
//
//    if (fabsf(currentVelocity) > pidRuntime.maxVelocity[axis]) {
//        currentPidSetpoint = (currentVelocity > 0) ? previousSetpoint[axis] + pidRuntime.maxVelocity[axis] : previousSetpoint[axis] - pidRuntime.maxVelocity[axis];
//    }
//
//    previousSetpoint[axis] = currentPidSetpoint;
//    return currentPidSetpoint;
//}



float Cascade_PID(PID_Variables *OuterLoop, PID_Variables *InnerLoop, float Setpoint, float AngleMeasurement, float RateMeasurement, float PID_Frequency, int axis)
{

	float AngleRate = PID_Equation (OuterLoop, Setpoint, AngleMeasurement, PID_Frequency, axis); // angle pid

	AngleRate = pt3FilterApply(&OuterLoop->PID_PT3Filter,AngleRate);//smoothen the setpoint

	float Motor_Command = PID_Equation (InnerLoop, AngleRate, RateMeasurement, PID_Frequency, axis); //rate pid

	return Motor_Command;
}


float Calculate_Setpoint(uint16_t RC_Input, int Range)
{
	float Setpoint = (((RC_Input-988)*2*Range)/(2012-988))-Range;
	return Setpoint;
}


