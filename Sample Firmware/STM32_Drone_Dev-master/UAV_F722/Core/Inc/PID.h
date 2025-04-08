#include "main.h"
#include "../Src/CommonMath/filters.h"
#include "../Src/flight/imu.h"
#include "../Src/flight/defines.h"
#include "../Src/CommonMath/maths.h"
#include "stm32f7xx_hal.h"

#define PT2_Dterm 13



//AirMode allow the motor to spin much faster than the throttle input (Stronger response), set it to 0 will disable it
#define AirMode 1;

extern float Motor_Idle; //idle percent ranging from 0~100%, meaning the motor will spin at this rate no matter how low the throttle is

typedef struct pidf_s {
    uint8_t P;   // Proportional gain
    uint8_t I;   // Integral gain
    uint8_t D;   // Derivative gain
    uint16_t F;  // Feedforward gain
    uint8_t S;   // Setpoint Weighting or any other tuning parameter

} pidf_t;

typedef enum {
    PID_ROLL,
    PID_PITCH,
    PID_YAW,
    PID_LEVEL,
    PID_MAG,
    PID_ITEM_COUNT
} pidIndex_e;

typedef struct {

	//pidf_t pid[PID_ITEM_COUNT];
	float kp;
	float ki;
	float kd;
	float Setpoint;
	float Integral;
	float PrevError;

	float Pterm;

	float previousGyroRateDterm;
	float dmaxPercent;
	float dmaxGyroGain;
	float dmaxSetpointGain;
	float buf[gyroDownsample_size];
	float Dterm;

	float Iterm_Min;
	float Iterm_Max;

	float Current_Tick;
	float Prev_Tick;
	float dt;

	laggedMovingAverage_t gyroDtermDownsample;
	pt3Filter_t PID_PT3Filter; //for setpoint

	pt1Filter_t GyroLPF1;
	pt1Filter_t GyroLPF2;

	pt2Filter_t DmaxRangeLPF;
	pt2Filter_t DmaxLPF;

	//Yaw Filter
	pt1Filter_t YawLPF;

	//Final Dterm filter
	#ifdef PT1_Dterm
	pt1Filter_t PT1Filter;
	#endif
	#ifdef PT2_Dterm
	pt2Filter_t PT2Filter;
	#endif

}PID_Variables;




void PID_Init(PID_Variables *pid, float P, float I, float D, float Dmax, float I_min, float I_max);

float pidLevelSetpoint (PID_Variables *pid, float setpoint, uint16_t angle);

float PID_Equation (PID_Variables *PID_Name, float Setpoint, float InputMeasurement, float dt, int axis);

float Cascade_PID(PID_Variables *OuterLoop, PID_Variables *InnerLoop, float Setpoint, float AngleMeasurement, float Ratemeasurement, float PID_Frequency, int axis);

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

