/*
 * RangeFinder.h
 *
 *  Created on: Oct 29, 2024
 *      Author: Vincent
 */

#ifndef SRC_SENSORS_RANGEFINDER_H_
#define SRC_SENSORS_RANGEFINDER_H_

#include "../drivers/TFmini_Plus.h"
#include "../flight/Flight_System.h"
#include "../CommonMath/filters.h"

#define RangeFinderPT1HZ 100.0f
#define RangeFinderSamplingRate 1000


typedef struct
{
	TFminiPlus_t LiDar;
	pt1Filter_t RangeFinderPT1;

	uint16_t Distancef;

}RangeFinder_t;

extern RangeFinder_t RangeFinder;


void rangeFinderFilterInit(void);
void RF_DataProcess(void);


#endif /* SRC_SENSORS_RANGEFINDER_H_ */
