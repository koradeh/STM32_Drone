/*
 * gyro.h
 *
 *  Created on: Oct 6, 2024
 *      Author: Vincent
 */

#ifndef SRC_SENSORS_GYRO_H_
#define SRC_SENSORS_GYRO_H_

#include "ICM42688P.h"
//#include "stdint.h"


void gyroSlewLimiter(ICM42688_GyroData_t *gyro);
#endif /* SRC_SENSORS_GYRO_H_ */
