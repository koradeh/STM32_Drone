/*
 * Flight_System.c
 *
 *  Created on: Aug 9, 2024
 *      Author: Vincent
 */

#include "Flight_System.h"
System_Health_t sysHealth;
/*Handling Arming Signal*/
bool Stick_Reset;
bool ARMED;
bool ArmingHandle (uint16_t* mapped_Channel)
{
	/* Arming Handling */
	if ((mapped_Channel[SD]>1800) && (mapped_Channel[Thro]<=990) && Stick_Reset)
	{
		ARMED = 1;
	}
	else if (mapped_Channel[SD]<1800)
	{
		Stick_Reset = 0;
		ARMED = 0 ;
	}

	if((mapped_Channel[SD]<1800) && (mapped_Channel[Thro]<=1000))
	{
		Stick_Reset = 1;
		//return 0;
	}
	return ARMED;
}


uint16_t magCaliAccumulator = 0;
enum System_Mode System_Mode_Handle(uint16_t* mapped_Channel)
{



	//-----Stick Command-------
	if((mapped_Channel[Thro]<=995)&&(mapped_Channel[Yaw]<=998)&&(mapped_Channel[Pitch]<=995)&&(mapped_Channel[Roll]>=1990))
	{
		magCaliAccumulator ++;
	}
	else
	{
		magCaliAccumulator = 0;
	}

	if(magCaliAccumulator >= 3000)
	{
		return magCalibrate;
	}


	//------System Mode------
	if (mapped_Channel[SC] > 1800)
	{
	    return POSHold;
	}
	else if (mapped_Channel[SC] < 1800 && mapped_Channel[SC] > 1300)
	{
	    return ALTMode;
	}
	else if (mapped_Channel[SC] < 1300)
	{
	    return AngleMode;
	}
}






	//DJI activation stick pos
//	if((mapped_Channel[Thro]<=990)&&(mapped_Channel[yaw]>=1800)&&(mapped_Channel[pitch]<=990)&&(mapped_Channel[roll]<=990))
//	{
//
//	}




