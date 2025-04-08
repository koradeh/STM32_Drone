/*
 * adc.h
 *
 *  Created on: Oct 25, 2024
 *      Author: Vincent
 */

#ifndef SRC_SENSORS_ADC_H_
#define SRC_SENSORS_ADC_H_

#include "stdint.h"
//Voltage divider parameter define (in kOHM):
#define R1 20
#define R2 2

typedef enum
{
	Batt1S,
	Batt2S,
	Batt3S,
	Batt4S,
	Batt5S,
	Batt6S,
	Batt7S,
	Batt8S,
	Batt9S,
	Batt10S,
	Batt11S,
	Batt12S,
}Batt_Type;

float getBatteryVoltage(uint32_t adcValue);
Batt_Type GetBatteryType(float vBatt);
#endif /* SRC_SENSORS_ADC_H_ */
