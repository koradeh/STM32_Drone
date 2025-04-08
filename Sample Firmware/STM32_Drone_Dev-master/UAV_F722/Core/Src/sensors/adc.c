/*
 * adc.c
 *
 *  Created on: Oct 25, 2024
 *      Author: Vincent
 */
#include "adc.h"

float getBatteryVoltage(uint32_t adcValue)
{
	//converting adc value to voltage, ranging from 0 to 36.3v
	return ((adcValue * 3.65f) /4095) * ((R1+R2)/R2); //v_ref is attained by experiment
}

Batt_Type GetBatteryType(float voltage)
{
    // Step 2: Define the voltage range for each cell count (LiPo nominal and full charge)
    const float nominalCellVoltage = 3.7f;
    const float maxCellVoltage = 4.35f;
    const float minCellVoltage = 3.0f;

    // Step 3: Check which range the voltage falls into
    for (uint8_t cellCount = 1; cellCount <= 12; cellCount++)
    {
        float minVoltage = cellCount * minCellVoltage; // Minimum for this cell count
        float maxVoltage = cellCount * maxCellVoltage; // Maximum for this cell count

        if (voltage >= minVoltage && voltage <= maxVoltage)
        {
            return (Batt_Type)(cellCount - 1); // Match found, return the type
        }
    }

    // Step 4: If voltage is out of range, handle it as an error or return a default
    return Batt1S; // Default to Batt1S if no match is found (or handle error differently)
}

