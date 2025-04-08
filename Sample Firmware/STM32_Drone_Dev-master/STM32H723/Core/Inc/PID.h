#include "main.h"
#include "stm32h7xx_hal.h"

//AirMode allow the motor to spin much faster than the throttle input (Stronger response), set it to 0 will disable it
#define AirMode 1;

extern float Motor_Idle; //idle percent ranging from 0~100%, meaning the motor will spin at this rate no matter how low the throttle is

typedef struct {
	float kp;
	float ki;
	float kd;
	float Setpoint;
	float Integral;
	float PrevError;

	float Iterm_Min;
	float Iterm_Max;
}PID_Variables;

void PID_Init(PID_Variables *pid, float P, float I, float D, float I_min, float I_max);

float PID_Equation (PID_Variables *PID_Name, float Setpoint, float InputMeasurement, float dt);

float Cascade_PID(PID_Variables *OuterLoop, PID_Variables *InnerLoop, float Setpoint, float AngleMeasurement, float Ratemeasurement, float PID_Frequency);

float Calculate_Setpoint(uint16_t RC_Channel, int Range);

//---------------------
//In Setup:
//---------------------
//PID_Variables Pitch_Angle
//PID_Variables Pitch_Rate

//PID_Init(&Pitch_Angle, kp, ki, kd, i_min, i_max);
//PID_Init(&Pitch_Rate, kp, ki, kd, i_min, i_max);

//---------------------
//In While Loop:
//---------------------
//int PItch_Command = Cascade_PID(&Pitch_Angle, &Pitch_Rate, &Angle_Y, &Rate_Y);

