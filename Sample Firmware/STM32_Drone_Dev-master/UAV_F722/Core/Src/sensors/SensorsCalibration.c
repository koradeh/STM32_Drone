/*
 * SensorsCalibration.c
 *
 *  Created on: Jul 19, 2024
 *      Author: Vincent
 */
#include "SensorsCalibration.h"

int32_t a[3] = {0,0,0};//accel accumulator for 3 axis
int32_t g[3] = {0,0,0};//gyro accumulator for 3 axis
int32_t m[6] = {0,0,0,0,0,0};//mag data for 3 axis * max and min: max_X min_X, max_Y......


//buffer to store the calibration data
/*Structure:
 * [0] Accel x
 * [1] Accel y
 * [2] Accel z
 *
 *
 * [3] Gyro x
 * [4] Gyro y
 * [5] Gyro z
 *
 *
 * [6] Mag x
 * [7] Mag y
 * [8] Mag z
 */
uint32_t CalibrationDataBuffer[9];

bool MagisCalibrating = false;

void accCalibration(ICM42688_AccData_t *accData, int iteration)
{
	if(iteration == 1) //first thing to do
	{
		a[0] = 0;
		a[1] = 0;
		a[2] = 0;
		accData->RawX_Trim =0;
		accData->RawY_Trim =0;
		accData->RawZ_Trim =0;
	}
	if(iteration<CALIBRATING_ACC_CYCLES) //during sample collection
	{
		a[0]+= accData->RawX;
		a[1]+= accData->RawY;
		a[2]+= accData->RawZ;
	}

	if(iteration == CALIBRATING_ACC_CYCLES) //last step
	{
		accData->RawX_Trim = a[0]/iteration;
		memcpy(&CalibrationDataBuffer[ax], &accData->RawX_Trim, sizeof(accData->RawX_Trim));

		accData->RawY_Trim = a[1]/iteration;
		memcpy(&CalibrationDataBuffer[ay], &accData->RawY_Trim, sizeof(accData->RawY_Trim));

		accData->RawZ_Trim = ((a[2])/iteration)-4096;
		memcpy(&CalibrationDataBuffer[az], &accData->RawZ_Trim, sizeof(accData->RawZ_Trim));

		saveCalibrationOffsets();
	}
}

void gyroCalibration(ICM42688_GyroData_t *gyroData, int iteration)
{
	if(iteration == 1) //first thing to do
	{
		g[0] = 0;
		g[1] = 0;
		g[2] = 0;
		gyroData->RawX_Trim =0;
		gyroData->RawY_Trim =0;
		gyroData->RawZ_Trim =0;
	}
	if(iteration<CALIBRATING_GYRO_CYCLES) //during sample collection
	{
		g[0]+= gyroData->RawX;
		g[1]+= gyroData->RawY;
		g[2]+= gyroData->RawZ;
	}

	if(iteration == CALIBRATING_GYRO_CYCLES) //last step
	{
		gyroData->RawX_Trim = g[0]/iteration;
		memcpy(&CalibrationDataBuffer[gx], &gyroData->RawX_Trim, sizeof(gyroData->RawX_Trim));

		gyroData->RawY_Trim = g[1]/iteration;
		memcpy(&CalibrationDataBuffer[gy], &gyroData->RawY_Trim, sizeof(gyroData->RawY_Trim));

		gyroData->RawZ_Trim = g[2]/iteration;
		memcpy(&CalibrationDataBuffer[gz], &gyroData->RawZ_Trim, sizeof(gyroData->RawZ_Trim));
		saveCalibrationOffsets();
	}
}

void magCalibration(vector3_t *magData)
{
	//initialize target calibration time
	const uint32_t TargetTime = CALIBRATING_MAG_SECONDS*1000000;
	static uint16_t start_tick = 0;
	static uint32_t elapsed_us = 0;

	 // Start timing on the first call of the calibration loop
	if(elapsed_us == 0)
	{
		start_tick = TIM6_GetTick();
	}

	// Update elapsed time in microseconds
	uint16_t current_tick = TIM6_GetTick();
	elapsed_us += GetMicroseconds(current_tick, start_tick);
	start_tick = current_tick;


	//during calibration
	if(elapsed_us<= TargetTime) //total cali time is 30 second
	{
		  HAL_GPIO_WritePin(beeperPort, beeperPin, GPIO_PIN_SET);

		//process X axis
		if (magData->x > m[Xmax])
		{
			m[Xmax] = magData->x; //if the measured x data is larger than current max x, then replace
		}
		else if(magData->x <m[Xmin])
		{
			m[Xmin] = magData->x; //if the measured x data is smaller than current min x, then replace
		}

		//process Y axis
		if (magData->y > m[Ymax])
		{
			m[Ymax] = magData->y;
		}
		else if(magData->y <m[Ymin])
		{
			m[Ymin] = magData->y;
		}

		//process Z axis
		if (magData->z > m[Zmax])
		{
			m[Zmax] = magData->z;
		}
		else if(magData->z <m[Zmin])
		{
			m[Zmin] = magData->z;
		}
	}

	//finalize calibration
	else
	{
	  HAL_GPIO_WritePin(beeperPort, beeperPin, GPIO_PIN_RESET);
		float magX_trim = 0;
		float magY_trim = 0;
		float magZ_trim = 0;


		magX_trim = ((float)m[Xmax] + (float) m[Xmin]) / 2.0f;
		memcpy(&CalibrationDataBuffer[mx], &magX_trim, sizeof(magX_trim));

		magY_trim = ((float)m[Ymax] + (float) m[Ymin]) / 2.0f;
		memcpy(&CalibrationDataBuffer[my], &magY_trim, sizeof(magY_trim));

		magZ_trim = ((float)m[Zmax] + (float) m[Zmin]) / 2.0f;
		memcpy(&CalibrationDataBuffer[mz], &magZ_trim, sizeof(magZ_trim));

		saveCalibrationOffsets();

		mag.Xtrim = magX_trim;
		mag.Ytrim = magY_trim;
		mag.Ztrim = magZ_trim;

		MagisCalibrating = false;
        // Reset elapsed time to allow for a new calibration session if needed
        elapsed_us = 0;
	}
}

void saveCalibrationOffsets(void)
{
	Flash_Write_Data(FLASH_USER_START_ADDR,CalibrationDataBuffer, 9);

}

void loadCalibrationOffsets(ICM42688_AccData_t *accData,ICM42688_GyroData_t *gyroData, QMC_t *mag)
{
	Flash_Read_Data(FLASH_USER_START_ADDR,CalibrationDataBuffer,9);

	//decode accel trim
	memcpy(&accData->RawX_Trim,&CalibrationDataBuffer[ax],sizeof(float));
	memcpy(&accData->RawY_Trim,&CalibrationDataBuffer[ay],sizeof(float));
	memcpy(&accData->RawZ_Trim,&CalibrationDataBuffer[az],sizeof(float));

	//decode gyro trim
	memcpy(&gyroData->RawX_Trim,&CalibrationDataBuffer[gx],sizeof(float));
	memcpy(&gyroData->RawY_Trim,&CalibrationDataBuffer[gy],sizeof(float));
	memcpy(&gyroData->RawZ_Trim,&CalibrationDataBuffer[gz],sizeof(float));

	//decode mag trim
	memcpy(&mag->Xtrim,&CalibrationDataBuffer[mx],sizeof(float));
	memcpy(&mag->Ytrim,&CalibrationDataBuffer[my],sizeof(float));
	memcpy(&mag->Ztrim,&CalibrationDataBuffer[mz],sizeof(float));



	//  memcpy(&readValue, rxData, sizeof(readValue));

}

