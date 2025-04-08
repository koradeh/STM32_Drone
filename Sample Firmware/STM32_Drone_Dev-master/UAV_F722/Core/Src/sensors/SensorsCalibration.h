/*
 * SensorsCalibration.h
 *
 *  Created on: Nov 21, 2024
 *      Author: Vincent
 */

#ifndef SRC_SENSORS_SENSORSCALIBRATION_H_
#define SRC_SENSORS_SENSORSCALIBRATION_H_

#include "string.h"
#include "../../Inc/Timers.h"
#include <ICM42688P.h>
#include "../CommonMath/vector.h"
#include "../flash/Flash_Sector_F4.h"
#include "../flight/imu.h"


#define CALIBRATING_ACC_CYCLES 1000
#define CALIBRATING_GYRO_CYCLES 1000
#define CALIBRATING_MAG_SECONDS 30


// Flash memory addresses for storing offsets
#define FLASH_USER_START_ADDR   (0x08062000) // Start address of the user flash area (adjust as needed)



extern bool MagisCalibrating;


typedef struct
{


}accTrim_t;

typedef enum
{
	Xmax,
	Xmin,
	Ymax,
	Ymin,
	Zmax,
	Zmin,
}magCalibration_t;

typedef enum
{
	ax,
	ay,
	az,
	gx,
	gy,
	gz,
	mx,
	my,
	mz,
}CalibrationIndex;

void accCalibration(ICM42688_AccData_t *accData, int iteration);
void gyroCalibration(ICM42688_GyroData_t *gyroData, int iteration);
void magCalibration(vector3_t *magData);
void saveCalibrationOffsets(void);
void loadCalibrationOffsets(ICM42688_AccData_t *accData,ICM42688_GyroData_t *gyroData, QMC_t *mag);

#endif /* SRC_SENSORS_SENSORSCALIBRATION_H_ */
