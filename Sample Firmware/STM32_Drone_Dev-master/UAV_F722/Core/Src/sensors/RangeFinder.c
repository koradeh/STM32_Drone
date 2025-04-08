/*
 * RangeFinder.c
 *
 *  Created on: Oct 29, 2024
 *      Author: Vincent
 */

#include "RangeFinder.h"


RangeFinder_t RangeFinder;

void rangeFinderFilterInit(void)
{
	float RangeFinderPT1FilterGain = pt1FilterGain(RangeFinderPT1HZ,1.0f/RangeFinderSamplingRate);
	pt1FilterInit(&RangeFinder.RangeFinderPT1,RangeFinderPT1FilterGain);
}


void RF_DataProcess(void)
{
	//Generate RangeFinder Health Flag (need to fix it to based on strength)
	sysHealth.RangeFinderHealthy = (TFmini_Plus_Process_Data(&RangeFinder.LiDar) == 1);

	if((RangeFinder.LiDar.Strength<=65535)&&(RangeFinder.LiDar.Strength>=100))
	{
		sysHealth.RangeFinderHealthy = true;
	}
	else
	{
		sysHealth.RangeFinderHealthy = false;
	}


	RangeFinder.Distancef = pt1FilterApply(&RangeFinder.RangeFinderPT1,RangeFinder.LiDar.Distance);
}
