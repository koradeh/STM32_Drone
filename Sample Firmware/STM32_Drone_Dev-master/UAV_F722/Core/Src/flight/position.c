/*
 * position.c
 *
 *  Created on: Oct 24, 2024
 *      Author: Vincent
 */
#include "position.h"


position_t geo;

void geo_init(void)
{
	//reset geo data
	for(int i = 0; i < 3 ; i++)
	{
		geo.origin.v[i] = 0;
		geo.llh.v[i] 	= 0;
		geo.pos.v[i] 	= 0;
	}
}







void position_update(bool ARMED)
{
	static bool wasArmed = false;

	//Handle height data

    //---DISARMED---
    if(!ARMED)
    {
        if (wasArmed)
        { // things to run once, on disarming, after being armed
            wasArmed = false;
        }
    }

    //---ARMED---
    else
    {
        if (!wasArmed)
        { // things to run once, on arming, after being disarmed

        	//update the home location after arming
        	//position.homeLon = gps.fLon;
        	//position.homeLat = gps.fLat;
            wasArmed = true;
        }
    }
}

static bool isStablized(void)
{

}

void PositionHoldSetpointCalculation(void)
{
	static bool wasStablized = false;

	//--------attain the desired position after stabilized
	if(isStablized())
	{
		if(!wasStablized)
		{
			//Run once after stabilizing from unstable phase
			//position.setlon = gps.fLon;
			//position.setlat = gps.fLat;
			wasStablized = true;
		}


	}

	//-------Unstable phase (when the drone is command to move)
	else
	{
		if(wasStablized)
		{
			//Run once after being unstable from stable phase
			//position.setlon = 0;
			//position.setlat = 0;
			wasStablized = false;
		}



	}

	//calculate current


	//Calculate Position Error

}
