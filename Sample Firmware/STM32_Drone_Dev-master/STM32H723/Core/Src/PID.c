#include "main.h"
#include "stm32h7xx_hal.h"
#include <PID.h>

float Motor_Idle = 0.05 ; //idle percent ranging from 0~100%, meaning the motor will spin at this rate no matter how low the throttle is


float PIDLimit;


float Previous_Time =0;
float Current_Time;
float dt;



void PID_Init(PID_Variables *pid, float P, float I, float D, float I_min, float I_max) //should be placed at the end of initialization
{
	pid->kp = P;
	pid->ki = I;
	pid->kd = D;
	pid->Iterm_Min = I_min;
	pid->Iterm_Max = I_max;
	pid->Setpoint = 0;
	pid->Integral = 0;
	pid->PrevError = 0;

	//these will be deleted after moved to timer interrupt
	Current_Time= 0;
	dt = 0;
	Previous_Time  = HAL_GetTick();
}


float PID_Equation (PID_Variables *PID_Name, float Setpoint, float InputMeasurement, float dt)
{
	//Assign Set Point
	PID_Name->Setpoint = Setpoint;

	//Calculation of dt
//	Current_Time = HAL_GetTick(); //update current time
//	dt = (Current_Time - Previous_Time)/1000; //get dt divide by 1000 to convert back to sec
//	Previous_Time=Current_Time; //update previous time
	//ERROR Calculation
	float Current_Error = PID_Name->Setpoint - InputMeasurement;


	//Integral Calculation
	PID_Name->Integral += Current_Error * (1/dt); //perform integration

	//I term relax (prevent I stack up) I*Integral = I_limit -> Integral_limit = I_Limit / I
	if (PID_Name->Integral > (PID_Name->Iterm_Max / PID_Name->ki))
	{
		PID_Name->Integral = PID_Name->Iterm_Max / PID_Name->ki;
	}
	else if (PID_Name->Integral < (PID_Name->Iterm_Min / PID_Name->ki))
	{
		PID_Name->Integral = PID_Name->Iterm_Min / PID_Name->ki;
	}

	//Derivative Calculation
	float Derivative = (Current_Error - PID_Name->PrevError)/(1/dt);


	//Output Calculation
	//kp * e(t)  +  ki * Integral( e(t)*dt )  +  kd * delta(e(t)) / delta (t)
	float PIDOutput = PID_Name->kp*(Current_Error) + PID_Name->ki * PID_Name->Integral + PID_Name->kd * Derivative;


	//Update the current error to previous error
	PID_Name->PrevError = Current_Error;

	return PIDOutput;
}


float Cascade_PID(PID_Variables *OuterLoop, PID_Variables *InnerLoop, float Setpoint, float AngleMeasurement, float RateMeasurement, float PID_Frequency)
{

	float Desired_Angular_Velocity = PID_Equation (OuterLoop, Setpoint, AngleMeasurement, PID_Frequency); // angle pid

	float Motor_Command = PID_Equation (InnerLoop, Desired_Angular_Velocity, RateMeasurement, PID_Frequency); //rate pid

	return Motor_Command;
}


float Calculate_Setpoint(uint16_t RC_Input, int Range)
{
	float Setpoint = (((RC_Input-988)*2*Range)/(2012-988))-Range;
	return Setpoint;
}

//-------------------------
//Output Limit Calculation
//PIDLimit = 2048 - ThrottleInput; //Throttle Range is from 47 ~ 2048, so PID can't exceed that value
//
//#ifdef AirMode == 1
//if(PIDOutput>PIDLimit)
//{
//	PIDOutput = PIDLimit;
//}
//else if(ThrottleInput+PIDOutput<(2048*Motor_Idle))
//{
//	PIDOutput = (2048*Motor_Idle) - ThrottleInput ; //min motor command
//}
//
//#else if AirMode == 0
//if(PIDOutput>PIDLimit*0.2)
//{
//	PIDOutput = (PIDLimit*0.2); //smaller error adjustment range compare to airmode
//}
//else if(PIDOutput<(2048*Motor_Idle))
//{
//	PIDOutput = 2048*Motor_Idle;
//}
//
//#endif

