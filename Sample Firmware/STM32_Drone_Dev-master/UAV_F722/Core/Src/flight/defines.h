/*
 * defines.h
 *
 *  Created on: Oct 7, 2024
 *      Author: Vincent
 */

#ifndef SRC_FLIGHT_DEFINES_H_
#define SRC_FLIGHT_DEFINES_H_

//-----------System Default-----------------
// Scaling factors for Pids for better tunable range in configurator for betaflight pid controller. The scaling is based on legacy pid controller or previous float
#define PTERM_SCALE 0.032029f
#define ITERM_SCALE 0.244381f
#define DTERM_SCALE 0.000529f
// The constant scale factor to replace the Kd component of the feedforward calculation.
// This value gives the same "feel" as the previous Kd default of 26 (26 * DTERM_SCALE)
#define FEEDFORWARD_SCALE 0.013754f
//-------------------------------------------


//-------------------------------------------
//---------Set up----------------------------

//Max Pitch, Roll Angle
#define Max_Angle 45
#define Max_Yaw_Rate 200

#define PID_Freq 2000


//MAG
#define mag_declination (-0.85f)
#define mag_x_alignment 180
#define mag_y_alignment 0
#define mag_z_alignment 0
//------------------------------------







/*----------PID Profiles-----------*/ //Gain range from 0 to 250
#define MasterMulti		1.0f //max 3
//-------Pitch Tuninging-------------
#define Pp 	25
#define Ip	20
#define Dp	10
#define Dmaxp	10
#define Fp	0
//-------Roll Tuninging-------------
#define Pr	20
#define Ir	0
#define Dr	10
#define Dmaxr	15
#define Fr 0
//-------Yaw Tuninging-------------
#define Py	40
#define Iy	0
#define Dy	0
#define Dmaxy	0
#define Fy 0
#define MaxYawRate 200
//--------ALT HOLD Tuning----------
#define Hoover_Throttle 1300
#define P_Alt		5
#define I_Alt		1
#define D_Alt		0
#define P_veloAlt	4.0f
#define I_veloAlt	0.2f
#define D_veloAlt	0

//-------POS HOLD Tuning------------
//-------------------------------------------

//------------PID Filters CUTOFF Hz
#define D_MAX_GAIN_FACTOR 			0.00008f
#define D_MAX_SETPOINT_GAIN_FACTOR 	0.00008f

#define Dterm_PT1 		10.0f
#define Dterm_PT2		15.0f
#define AngleRate_PT3 	20.0f

#define Dmax_range_lpf	85.0f
#define Dmax_lpf		35.0f
#define rateDterm_lp1	15.0f
#define rateDterm_lp2

#define YawLPFhz 			50.0f
//--------------------------------------------------




//--------System PID Do not touch -----------------
//---------Angle Mode-----------
#define P_angleX 	5.0f
#define I_angleX 	0
#define D_angleX 	0

#define P_rateX 	(Pp*PTERM_SCALE)
#define I_rateX 	(Ip*ITERM_SCALE)
#define D_rateX 	(Dp*DTERM_SCALE)
#define Dmax_rateX	(Dmaxp*DTERM_SCALE)

//#define FF_Xd		Master*0
//#define FF_Xdd		Master*0

#define P_angleY 	5.0f //5.0
#define I_angleY 	0
#define D_angleY 	0

#define P_rateY 	(Pr*PTERM_SCALE)
#define I_rateY 	(Ir*ITERM_SCALE)
#define D_rateY 	(Dr*DTERM_SCALE)
#define Dmax_rateY	(Dmaxr*DTERM_SCALE)
//
//#define FF_Yd		Master*0
//#define FF_Ydd		Master*0

#define P_angleZ 	5.0f //5.0
#define I_angleZ 	0
#define D_angleZ 	0

#define P_rateZ 	(Py*PTERM_SCALE)
#define I_rateZ 	(Iy*ITERM_SCALE)
#define D_rateZ 	(Dy*DTERM_SCALE)
#define Dmax_rateZ	(Dmaxy*DTERM_SCALE)
//------------------------------------------------------

//-------------PID Paremeters--------------------
//Iterm Limiting constant
#define itermWindup 80.0f
#define PIDSUM_LIMIT                500
#define PIDSUM_LIMIT_YAW            400
//-------------Smoothen Dterm---------------
#define gyroDownsample_size		5



//---------------Sensor Filters------------------
#define opFlowPT2_CutoffHZ 100

#endif /* SRC_FLIGHT_DEFINES_H_ */
