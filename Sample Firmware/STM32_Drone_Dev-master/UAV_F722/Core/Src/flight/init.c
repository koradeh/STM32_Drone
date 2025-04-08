/*
 * init.c
 *
 *  Created on: Nov 20, 2024
 *      Author: Vincent
 */

#ifndef SRC_FLIGHT_INIT_C_
#define SRC_FLIGHT_INIT_C_
#include "init.h"

void init(void)
{
	  //Timer
	HAL_TIM_Base_Start(&sysTimerType);

	  //Motor Protocol Init
	dshot_init(DSHOT600);

	  //RX
	rxInit();

	  //GPS
	gps_init(&gps, &gpsBusType);

	  //Pos
	geo_init();

	  //IMU
	ICM42688_Init();

	Alt_Init();

	  //Optical Flow
	opFlow_init();


}
#endif /* SRC_FLIGHT_INIT_C_ */
