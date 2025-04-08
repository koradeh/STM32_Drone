/*
 * Flight_System.h
 *
 *  Created on: Nov 21, 2024
 *      Author: Vincent
 */

#ifndef SRC_FLIGHT_FLIGHT_SYSTEM_H_
#define SRC_FLIGHT_FLIGHT_SYSTEM_H_

#include "stdbool.h"
#include "main.h"

/*Flight Mode*/
enum System_Mode
{
	AngleMode,
	ALTMode,
	POSHold,
	magCalibrate,
};

enum Crsf_Channels
{
	Thro = 0,
	Yaw,
	Pitch,
	Roll,
	SA,
	SB,
	SC,
	SD,
};

typedef struct
{
	bool ARMED;
	bool imuHealty;
	bool gpsHealthy;
	bool baroHealthy;
	bool rxHealthy;
	bool RangeFinderHealthy;
	bool opFlowHealthy;
	bool magHealthy;
	bool Calibrating;

}System_Health_t;

extern System_Health_t sysHealth;


bool ArmingHandle (uint16_t* mapped_Channel);
enum System_Mode System_Mode_Handle(uint16_t* mapped_Channel);

#endif /* SRC_FLIGHT_FLIGHT_SYSTEM_H_ */
