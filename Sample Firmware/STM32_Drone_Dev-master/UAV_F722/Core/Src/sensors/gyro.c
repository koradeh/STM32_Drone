/*
 * gyro.c
 *
 *  Created on: Oct 6, 2024
 *      Author: Vincent
 */
#include "gyro.h"


void gyroSlewLimiter(ICM42688_GyroData_t *gyro)
{
	//check individual gyro value with previous reading
	if((gyro->RawX - gyro->prevRawX)>(1<<14))
	{
		gyro->RawX = gyro->prevRawX;
	}

	if((gyro->RawY - gyro->prevRawY)>(1<<14))
	{
		gyro->RawY = gyro->prevRawY;
	}

	if((gyro->RawZ - gyro->prevRawZ)>(1<<14))
	{
		gyro->RawZ = gyro->prevRawZ;
	}

	gyro->prevRawX = gyro->RawX;
	gyro->prevRawY = gyro->RawY;
	gyro->prevRawZ = gyro->RawZ;

}

