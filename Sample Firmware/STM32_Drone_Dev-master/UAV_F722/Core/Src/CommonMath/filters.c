/*
 * filters.c
 *
 *  Created on: Jun 15, 2024
 *      Author: Vincent
 */


#include <stdbool.h>
#include <string.h>
#include <math.h>
#include "filters.h"

#define M_PIf       3.14159265358979323846f



// PT1 Low Pass filter

float pt1FilterGain(float f_cut, float dT)
{
    float omega = 2.0f * M_PIf * f_cut * dT;
    return omega / (omega + 1.0f);
}

// Calculates filter gain based on delay (time constant of filter) - time it takes for filter response to reach 63.2% of a step input.
float pt1FilterGainFromDelay(float delay, float dT)
{
    if (delay <= 0)
    {
        return 1.0f; // gain = 1 means no filtering
    }

    const float cutoffHz = 1.0f / (2.0f * M_PIf * delay);
    return pt1FilterGain(cutoffHz, dT);
}

void pt1FilterInit(pt1Filter_t *filter, float k)
{
    filter->state = 0.0f;
    filter->k = k;
}

void pt1FilterUpdateCutoff(pt1Filter_t *filter, float k)
{
    filter->k = k;
}

float pt1FilterApply(pt1Filter_t *filter, float input)
{
    filter->state = filter->state + filter->k * (input - filter->state);
    return filter->state;
}

//Application of PT1 filter
//-----------------------------
////Filter definition (for testing)
//pt1Filter_t Testing;
//float f_cut = 10.0f;
//float dT = 1/500; //500hz
//float k = pt1FilterGain(f_cut, dT);
//
//float Input = 0.0f;
//float Output = 0.0f;
//pt1FilterInit(&Testing, k); //FOR TESTING
//
//Output = pt1FilterApply(&Testing, Input);
//-----------------------------


// PT2 Low Pass filter

float pt2FilterGain(float f_cut, float dT)
{
    // PTn cutoff correction = 1 / sqrt(2^(1/n) - 1)
    #define CUTOFF_CORRECTION_PT2 1.553773974f

    // shift f_cut to satisfy -3dB cutoff condition
    return pt1FilterGain(f_cut * CUTOFF_CORRECTION_PT2, dT);
}

// Calculates filter gain based on delay (time constant of filter) - time it takes for filter response to reach 63.2% of a step input.
float pt2FilterGainFromDelay(float delay, float dT)
{
    if (delay <= 0)
    {
        return 1.0f; // gain = 1 means no filtering
    }

    const float cutoffHz = 1.0f / (M_PIf * delay * CUTOFF_CORRECTION_PT2);
    return pt2FilterGain(cutoffHz, dT);
}

void pt2FilterInit(pt2Filter_t *filter, float k)
{
    filter->state = 0.0f;
    filter->state1 = 0.0f;
    filter->k = k;
}

void pt2FilterUpdateCutoff(pt2Filter_t *filter, float k)
{
    filter->k = k;
}

float pt2FilterApply(pt2Filter_t *filter, float input)
{
    filter->state1 = filter->state1 + filter->k * (input - filter->state1);
    filter->state = filter->state + filter->k * (filter->state1 - filter->state);
    return filter->state;
}



// PT3 Low Pass filter

float pt3FilterGain(float f_cut, float dT)
{
    // PTn cutoff correction = 1 / sqrt(2^(1/n) - 1)
    #define CUTOFF_CORRECTION_PT3 1.961459177f

    // shift f_cut to satisfy -3dB cutoff condition
    return pt1FilterGain(f_cut * CUTOFF_CORRECTION_PT3, dT);
}

// Calculates filter gain based on delay (time constant of filter) - time it takes for filter response to reach 63.2% of a step input.
float pt3FilterGainFromDelay(float delay, float dT)
{
    if (delay <= 0)
    {
        return 1.0f; // gain = 1 means no filtering
    }

    const float cutoffHz = 1.0f / (M_PIf * delay * CUTOFF_CORRECTION_PT3);
    return pt3FilterGain(cutoffHz, dT);
}

void pt3FilterInit(pt3Filter_t *filter, float k)
{
    filter->state = 0.0f;
    filter->state1 = 0.0f;
    filter->state2 = 0.0f;
    filter->k = k;
}

void pt3FilterUpdateCutoff(pt3Filter_t *filter, float k)
{
    filter->k = k;
}

float pt3FilterApply(pt3Filter_t *filter, float input)
{
    filter->state1 = filter->state1 + filter->k * (input - filter->state1);
    filter->state2 = filter->state2 + filter->k * (filter->state1 - filter->state2);
    filter->state = filter->state + filter->k * (filter->state2 - filter->state);
    return filter->state;
}

void ComplementaryFilterInit(compleFilter_t *filter, float k)
{
	filter->alpha=k;
}

float ComplementaryFilter(compleFilter_t *filter, float accAngle, float rate, float dt)
{
	filter->EstiAngle = (filter->alpha)*(filter->EstiAngle+rate*dt) + (1-filter->alpha)*accAngle;
	return filter->EstiAngle;
}



//Moving Average Filter
void laggedMovingAverageInit(laggedMovingAverage_t *filter, uint16_t windowSize, float *buf)
{
    filter->movingWindowIndex = 0;
    filter->windowSize = windowSize;
    filter->buf = buf;
    filter->movingSum = 0;
    memset(filter->buf, 0, windowSize * sizeof(float));
    filter->primed = false;
}

float laggedMovingAverageUpdate(laggedMovingAverage_t *filter, float input)
{
    filter->movingSum -= filter->buf[filter->movingWindowIndex];
    filter->buf[filter->movingWindowIndex] = input;
    filter->movingSum += input;

    if (++filter->movingWindowIndex == filter->windowSize) {
        filter->movingWindowIndex = 0;
        filter->primed = true;
    }

    const uint16_t denom = filter->primed ? filter->windowSize : filter->movingWindowIndex;
    return filter->movingSum / denom;
}




