/*
 * init.h
 *
 *  Created on: Nov 20, 2024
 *      Author: Vincent
 */

#ifndef SRC_FLIGHT_INIT_H_
#define SRC_FLIGHT_INIT_H_

#include "../../Inc/Timers.h"
#include "../drivers/dShot.h"
#include "../rx/rx.h"
#include "../sensors/gps.h"
#include "imu.h"
#include "Altitude.h"
#include "position.h"

void init(void);

#endif /* SRC_FLIGHT_INIT_H_ */
