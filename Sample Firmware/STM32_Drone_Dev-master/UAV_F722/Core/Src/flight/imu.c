/*
 * imu.c
 *
 *  Created on: Oct 5, 2024
 *      Author: Vincent
 */

#include "imu.h"
#include "../CommonMath/maths.h"
#include "../CommonMath/vector.h"


// the limit (in degrees/second) beyond which we stop integrating
// omega_I. At larger spin rates the DCM PI controller can get 'dizzy'
// which results in false gyro drift. See
// https://drive.google.com/file/d/0ByvTkVQo3tqXQUVCVUNyZEgtRGs/view?usp=sharing&resourcekey=0-Mo4254cxdWWx2Y4mGN78Zw
#define SPIN_RATE_LIMIT 20

static imuRuntimeConfig_t imuRuntimeConfig;
matrix33_t rMat; //3x3 rotation matrix

// quaternion of sensor frame relative to earth frame
quaternion q = QUATERNION_INITIALIZE;
quaternionProducts qP = QUATERNION_PRODUCTS_INITIALIZE;

// absolute angle inclination in multiple of 0.1 degree    180 deg = 1800
attitudeEulerAngles_t attitude = EULER_INITIALIZE;

//Magnetometer QMC5883L
QMC_t mag;


static vector2_t north_ef;


static void imuQuaternionComputeProducts(quaternion *quat, quaternionProducts *quatProd)
{
    quatProd->ww = quat->w * quat->w;
    quatProd->wx = quat->w * quat->x;
    quatProd->wy = quat->w * quat->y;
    quatProd->wz = quat->w * quat->z;
    quatProd->xx = quat->x * quat->x;
    quatProd->xy = quat->x * quat->y;
    quatProd->xz = quat->x * quat->z;
    quatProd->yy = quat->y * quat->y;
    quatProd->yz = quat->y * quat->z;
    quatProd->zz = quat->z * quat->z;
}


void imuComputeRotationMatrix(void)
{
    imuQuaternionComputeProducts(&q, &qP);

    rMat.m[0][0] = 1.0f - 2.0f * qP.yy - 2.0f * qP.zz;
    rMat.m[0][1] = 2.0f * (qP.xy + -qP.wz);
    rMat.m[0][2] = 2.0f * (qP.xz - -qP.wy);

    rMat.m[1][0] = 2.0f * (qP.xy - -qP.wz);
    rMat.m[1][1] = 1.0f - 2.0f * qP.xx - 2.0f * qP.zz;
    rMat.m[1][2] = 2.0f * (qP.yz + -qP.wx);

    rMat.m[2][0] = 2.0f * (qP.xz + -qP.wy);
    rMat.m[2][1] = 2.0f * (qP.yz - -qP.wx);
    rMat.m[2][2] = 1.0f - 2.0f * qP.xx - 2.0f * qP.yy;

#if defined(SIMULATOR_BUILD) && !defined(USE_IMU_CALC) && !defined(SET_IMU_FROM_EULER)
    rMat.m[1][0] = -2.0f * (qP.xy - -qP.wz);
    rMat.m[2][0] = -2.0f * (qP.xz + -qP.wy);
#endif
}


static float invSqrt(float x)
{
	return 1.0f/ sqrtf(x);
}


static bool useMag;
static void mag_init(void)
{
	useMag = (QMC_init(&mag, &magBusType, 200) == 0);  //if receive 1 it means init error

    // magnetic declination has negative sign (positive clockwise when seen from top)
    const float imuMagneticDeclinationRad = DEGREES_TO_RADIANS(mag_declination);
    north_ef.x = cos_approx(imuMagneticDeclinationRad);
    north_ef.y = -sin_approx(imuMagneticDeclinationRad);

}


//Gyro x, y, z are in deg/s needs to be converted to rad/s
//Accel x, y, z, normalized within function

//IMU Init
void imu_init(void)
{
	mag_init();
	imuComputeRotationMatrix();
	imuRuntimeConfig.imuDcmKp = 0.25f;
	imuRuntimeConfig.imuDcmKi = 0.0f;
}

void imuMahonyAHRSupdate(float dt, float gx, float gy, float gz, float ax, float ay, float az, float accMagnitude, const float dcmKpGain, float headingErrMag, float headingErrCog)
{
	static float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;
	float ex = 0, ey = 0, ez = 0;//initialize error terms

	//Convert deg/s to rad/s
	gx = DEGREES_TO_RADIANS(gx);
	gy = DEGREES_TO_RADIANS(gy);
	gz = DEGREES_TO_RADIANS(gz);

	//General Spin Rate
	const float spin_rate = sqrtf(sq(gx)+sq(gy)+sq(gz));

    // Add error from magnetometer and Cog
    // just rotate input value to body frame
    ex += rMat.m[Z][X] * (headingErrCog + headingErrMag);
    ey += rMat.m[Z][Y] * (headingErrCog + headingErrMag);
    ez += rMat.m[Z][Z] * (headingErrCog + headingErrMag);


	//To normalize a 3d vector : 1/sqrt(sqSum) sqSum = ax^2 + ay^2 + az^2
	float sqSum = sq(ax) + sq(ay) + sq(az);


	if(imuIsAccelerometerHealthy(accMagnitude) && (sqSum > 0.01f))
	{
		sqSum = invSqrt(sqSum);

		ax *= sqSum;
		ay *= sqSum;
		az *= sqSum;

		//Error is the sum of the cross product between estimated direction and gravity direction
		ex += (ay * rMat.m[2][2] - az * rMat.m[2][1]);
		ey += (az * rMat.m[2][0] - ax * rMat.m[2][2]);
		ez += (ax * rMat.m[2][1] - ay * rMat.m[2][0]);
	}


    // Compute and apply integral feedback if enabled (ki larger than 0)
    if (imuRuntimeConfig.imuDcmKi > 0.0f)
    {
        // Stop integrating if spinning beyond the certain limit
        if (spin_rate < DEGREES_TO_RADIANS(SPIN_RATE_LIMIT))
        {
            const float dcmKiGain = imuRuntimeConfig.imuDcmKi;
            integralFBx += dcmKiGain * ex * dt;    // integral error scaled by Ki
            integralFBy += dcmKiGain * ey * dt;
            integralFBz += dcmKiGain * ez * dt;
        }
    }
    else
    {
        integralFBx = 0.0f;    // prevent integral windup
        integralFBy = 0.0f;
        integralFBz = 0.0f;
    }

    //Apply porportional and integral feedback
    gx += dcmKpGain * ex + integralFBx;
    gy += dcmKpGain * ey + integralFBy;
    gz += dcmKpGain * ez + integralFBz;

    //Integrate the quaternion rate of change
    gx *= (0.5f * dt);
    gy *= (0.5f * dt);
    gz *= (0.5f * dt);

    quaternion buffer;
    buffer.w = q.w;
    buffer.x = q.x;
    buffer.y = q.y;
    buffer.z = q.z;

    q.w += (-buffer.x * gx - buffer.y * gy - buffer.z * gz);
    q.x += (+buffer.w * gx + buffer.y * gz - buffer.z * gy);
    q.y += (+buffer.w * gy - buffer.x * gz + buffer.z * gx);
    q.z += (+buffer.w * gz + buffer.x * gy - buffer.y * gx);

    //Normalize quaternion

    float recipNorm = invSqrt(sq(q.w) + sq(q.x) + sq(q.y) + sq(q.z));
    q.w *= recipNorm;
    q.x *= recipNorm;
    q.y *= recipNorm;
    q.z *= recipNorm;

    // Pre-compute rotation matrix from quaternion
    imuComputeRotationMatrix();
}

bool imuIsAccelerometerHealthy(float accMagnitude)
{
    // Accept accel readings only in range 0.9g - 1.1g
    return (0.9f < accMagnitude) && (accMagnitude < 1.1f);
}


//Calculate heading error using Mag
static float imuCalcMagErr(void)
{
    // Use measured magnetic field vector
    vector3_t mag_bf = mag.magADC;

    //alignment correction
    mag_bf.y = mag_bf.y * -1;
    mag_bf.z = mag_bf.z * -1;


    float magNormSquared = vector3NormSq(&mag_bf);

    if (magNormSquared > 0.01f)
    {

        // project magnetometer reading into Earth frame
        vector3_t mag_ef;
        matrixVectorMul(&mag_ef, &rMat, &mag_bf); // BF->EF true north
        // Normalise magnetometer measurement
        vector3Scale(&mag_ef, &mag_ef, 1.0f / sqrtf(magNormSquared));

        // For magnetometer correction we make an assumption that magnetic field is perpendicular to gravity (ignore Z-component in EF).
        // This way magnetic field will only affect heading and wont mess roll/pitch angles
        vector2_t mag2d_ef = {.x = mag_ef.x, .y = mag_ef.y};
        // mag2d_ef - measured mag field vector in EF (2D ground plane projection)
        // north_ef - reference mag field vector heading due North in EF (2D ground plane projection).
        //              Adjusted for magnetic declination (in imuConfigure)

        // magnetometer error is cross product between estimated magnetic north and measured magnetic north (calculated in EF)
        // increase gain on large misalignment
        const float dot = vector2Dot(&mag2d_ef, &north_ef);
        const float cross = vector2Cross(&mag2d_ef, &north_ef);
        return (dot > 0) ? cross : (cross < 0 ? -1.0f : 1.0f) * vector2Norm(&mag2d_ef);
    } else {
        // invalid magnetometer data
        return 0.0f;
    }
}


void imuUpdateEulerAngles(void)
{
//	quaternionProducts buffer;
//	WE DON'T HAVE HEADFREE_MODE YET
//    if (FLIGHT_MODE(HEADFREE_MODE))
//    {
//       imuQuaternionComputeProducts(&headfree, &buffer);
//
//       attitude.values.roll = lrintf(atan2_approx((+2.0f * (buffer.wx + buffer.yz)), (+1.0f - 2.0f * (buffer.xx + buffer.yy))) * (1800.0f / M_PIf));
//       attitude.values.pitch = lrintf(((0.5f * M_PIf) - acos_approx(+2.0f * (buffer.wy - buffer.xz))) * (1800.0f / M_PIf));
//       attitude.values.yaw = lrintf((-atan2_approx((+2.0f * (buffer.wz + buffer.xy)), (+1.0f - 2.0f * (buffer.yy + buffer.zz))) * (1800.0f / M_PIf)));
//    } else
//    {
//       attitude.values.roll = lrintf(atan2_approx(rMat.m[2][1], rMat.m[2][2]) * (1800.0f / M_PIf));
//       attitude.values.pitch = lrintf(((0.5f * M_PIf) - acos_approx(-rMat.m[2][0])) * (1800.0f / M_PIf));
//       attitude.values.yaw = lrintf((-atan2_approx(rMat.m[1][0], rMat.m[0][0]) * (1800.0f / M_PIf)));
//    }

   attitude.values.roll = lrintf(atan2_approx(rMat.m[2][1], rMat.m[2][2]) * (1800.0f / M_PIf));
   attitude.values.pitch = lrintf(((0.5f * M_PIf) - acos_approx(-rMat.m[2][0])) * (1800.0f / M_PIf));
   attitude.values.yaw = lrintf((-atan2_approx(rMat.m[1][0], rMat.m[0][0]) * (1800.0f / M_PIf)));

    if (attitude.values.yaw < 0)
    {
        attitude.values.yaw += 3600;
    }

}

float magErr = 0;
float cogErr = 0;
void imuAttitude(float dt, float gx, float gy, float gz, float ax, float ay, float az, float accMagnitude, bool armedSignal)
{
	static float track_time;

	//raise kp to quickly converge the estimated attitude after crash
	float kp = 0.0f;
	if(armedSignal || (track_time>=0.250))
	{
		kp = 0.25;
		if(armedSignal)
		{
			track_time = 0.0f;
		}
	}
	else
	{
		track_time += dt;
		kp = 2.5;
	}



	//update heading error using mag
	if(useMag)
	{
		 magErr = imuCalcMagErr();
	}

	imuMahonyAHRSupdate(dt, gx, gy, gz, ax, ay, az, accMagnitude, kp, magErr,cogErr);
	imuUpdateEulerAngles();

	QMC_read(&mag);
}

/***APPLY THIS LATER***/
// Calculate the dcmKpGain to use. When armed, the gain is imuRuntimeConfig.imuDcmKp, i.e., the default value
// When disarmed after initial boot, the scaling is 10 times higher  for the first 20 seconds to speed up initial convergence.
// After disarming we want to quickly reestablish convergence to deal with the attitude estimation being incorrect due to a crash.
//   - wait for a 250ms period of low gyro activity to ensure the craft is not moving
//   - use a large dcmKpGain value for 500ms to allow the attitude estimate to quickly converge
//   - reset the gain back to the standard setting
//static float imuCalcKpGain(timeUs_t currentTimeUs, bool useAcc, float *gyroAverage)
//{
//    static enum {
//        stArmed,
//        stRestart,
//        stQuiet,
//        stReset,
//        stDisarmed
//    } arState = stDisarmed;
//
//    static timeUs_t stateTimeout;
//
//    const bool armState = ARMING_FLAG(ARMED);
//
//    if (!armState) {
//        // If gyro activity exceeds the threshold then restart the quiet period.
//        // Also, if the attitude reset has been complete and there is subsequent gyro activity then
//        //  start the reset cycle again. This addresses the case where the pilot rights the craft after a crash.
//        if (   (fabsf(gyroAverage[X]) > ATTITUDE_RESET_GYRO_LIMIT)  // gyro axis limit exceeded
//            || (fabsf(gyroAverage[Y]) > ATTITUDE_RESET_GYRO_LIMIT)
//            || (fabsf(gyroAverage[Z]) > ATTITUDE_RESET_GYRO_LIMIT)
//            || !useAcc                                              // acc reading out of range
//            ) {
//            arState = stRestart;
//        }
//
//        switch (arState) {
//        default: // should not happen, safeguard only
//        case stArmed:
//        case stRestart:
//            stateTimeout = currentTimeUs + ATTITUDE_RESET_QUIET_TIME;
//            arState = stQuiet;
//            // fallthrough
//        case stQuiet:
//            if (cmpTimeUs(currentTimeUs, stateTimeout) >= 0) {
//                stateTimeout = currentTimeUs + ATTITUDE_RESET_ACTIVE_TIME;
//                arState = stReset;
//            }
//            // low gain (default value of 0.25) during quiet phase
//            return imuRuntimeConfig.imuDcmKp;
//        case stReset:
//            if (cmpTimeUs(currentTimeUs, stateTimeout) >= 0) {
//                arState = stDisarmed;
//            }
//            // high gain, 100x greater than normal, or 25, after quiet period
//            return imuRuntimeConfig.imuDcmKp * 100.0f;
//        case stDisarmed:
//            // Scale the kP to converge 10x faster when disarmed, ie 2.5
//            return imuRuntimeConfig.imuDcmKp * 10.0f;
//        }
//    } else {
//        arState = stArmed;
//        return imuRuntimeConfig.imuDcmKp;
//    }
//}


