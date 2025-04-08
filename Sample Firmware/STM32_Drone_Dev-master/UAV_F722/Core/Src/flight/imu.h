/*
 * imu.h
 *
 *  Created on: Oct 5, 2024
 *      Author: Vincent
 */

#ifndef INC_IMU_H_
#define INC_IMU_H_

#include "ResourceDefine.h"
#include "ICM42688P.h"
#include "math.h"
#include "defines.h"
#include "../drivers/QMC5883L.h"
#include "../CommonMath/vector.h"

extern QMC_t mag;

/*Quaternion Definition*/
typedef struct
{
	float w,x,y,z;
} quaternion;
#define QUATERNION_INITIALIZE  {.w=1, .x=0, .y=0,.z=0}

typedef struct
{
	float ww,wx,wy,wz,xx,xy,xz,yy,yz,zz;
} quaternionProducts;
#define QUATERNION_PRODUCTS_INITIALIZE  {.ww=1, .wx=0, .wy=0, .wz=0, .xx=0, .xy=0, .xz=0, .yy=0, .yz=0, .zz=0}

typedef enum
{
	roll,
	pitch,
	yaw,
	alt
}axis;
/*Euler Angle structure define*/
typedef union
{
    int16_t raw[XYZ_AXIS_COUNT];
    struct
	{
        // absolute angle inclination in multiple of 0.1 degree    180 deg = 1800
        int16_t roll;
        int16_t pitch;
        int16_t yaw;
    } values;
} attitudeEulerAngles_t;


#define EULER_INITIALIZE  { { 0, 0, 0 } }

extern attitudeEulerAngles_t attitude;

/*PI gain*/
typedef struct imuRuntimeConfig_s
{
    float imuDcmKi;
    float imuDcmKp;
} imuRuntimeConfig_t;


//Function Declare
void imu_init(void);
void imuMahonyAHRSupdate(float dt, float gx, float gy, float gz, float ax, float ay, float az, float accMagnitude, const float dcmKpGain,float headingErrMag, float headingErrCog);
bool imuIsAccelerometerHealthy(float accMagnitude);
void imuUpdateEulerAngles(void);
void imuAttitude(float dt, float gx, float gy, float gz, float ax, float ay, float az, float accMagnitude, bool armedSignal);
#endif /* INC_IMU_H_ */
