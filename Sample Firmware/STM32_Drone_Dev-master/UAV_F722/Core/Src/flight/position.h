/*
 * position.h
 *
 *  Created on: Oct 24, 2024
 *      Author: Vincent
 */

#ifndef SRC_FLIGHT_POSITION_H_
#define SRC_FLIGHT_POSITION_H_

#include "../sensors/gps.h"
#include "../CommonMath/vector.h"
#include "stdbool.h"
#define DECELERATION_RATE_NORMAL 5 //Normal deceleration rate when the drone is stopping (m/s^2), from 5m/s to 0m/s in 1s
#define DECELERATION_RATE_BREAKING 15 //Deceleration rate when breaking is activated

typedef struct
{

	vector3_t origin; 	//track origin coordinate
	vector3_t llh;		//current coordinate
	vector3_t pos;		//position relative to origin in Cartesian
	float originScale;
	bool originValid;
}position_t;

extern position_t geo;


void geo_init(void);

#endif /* SRC_FLIGHT_POSITION_H_ */
