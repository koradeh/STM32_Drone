/*
 * filters.h
 *
 *  Created on: Jun 15, 2024
 *      Author: Vincent
 */

#ifndef SRC_COMMONMATH__FILTERS_H_
#define SRC_COMMONMATH__FILTERS_H_
#include "../../Inc/Timers.h"
#include "stdbool.h"
#include "stdint.h"
//Complementray Filter Gain Define
#define Comple_Alpha 0.98f

typedef struct pt1Filter_s {
    float state;
    float k;
} pt1Filter_t;


typedef struct pt2Filter_s {
    float state;
    float state1;
    float k;
} pt2Filter_t;


typedef struct pt3Filter_s {
    float state;
    float state1;
    float state2;
    float k;
} pt3Filter_t;

typedef struct laggedMovingAverage_s {
    uint16_t movingWindowIndex;
    uint16_t windowSize;
    float movingSum;
    float *buf;
    bool primed;
} laggedMovingAverage_t;


typedef struct compleFilter_s {
	float alpha;
	float EstiAngle;
	float prevT;
}compleFilter_t;



//PT1 Function Declare
float pt1FilterGain(float f_cut, float dT);
float pt1FilterGainFromDelay(float delay, float dT);
void pt1FilterInit(pt1Filter_t *filter, float k);
void pt1FilterUpdateCutoff(pt1Filter_t *filter, float k);
float pt1FilterApply(pt1Filter_t *filter, float input);


//PT2 Function Declare
float pt2FilterGain(float f_cut, float dT);
float pt2FilterGainFromDelay(float delay, float dT);
void pt2FilterInit(pt2Filter_t *filter, float k);
void pt2FilterUpdateCutoff(pt2Filter_t *filter, float k);
float pt2FilterApply(pt2Filter_t *filter, float input);


//PT3 Function Declare
float pt3FilterGain(float f_cut, float dT);
float pt3FilterGainFromDelay(float delay, float dT);
void pt3FilterInit(pt3Filter_t *filter, float k);
void pt3FilterUpdateCutoff(pt3Filter_t *filter, float k);
float pt3FilterApply(pt3Filter_t *filter, float input);


//Moving Average Function Dclare
void laggedMovingAverageInit(laggedMovingAverage_t *filter, uint16_t windowSize, float *buf);
float laggedMovingAverageUpdate(laggedMovingAverage_t *filter, float input);

//Complementary Filter Declare
void ComplementaryFilterInit(compleFilter_t *filter, float k);
float ComplementaryFilter(compleFilter_t *filter, float accAngle, float rate, float dt);

#endif /* INC_FILTERS_H_ */
