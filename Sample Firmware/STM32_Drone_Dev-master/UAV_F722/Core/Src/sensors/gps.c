/*
 * gps.c
 *
 *  Created on: Oct 22, 2024
 *      Author: Vincent
 */
#include "gps.h"

#define UART_TIMEOUT 2000

// **********************
// GPS
// **********************
union u_Short uShort;
union i_Short iShort;
union u_Long uLong;
union i_Long iLong;

gpsLocation_t GPS_home_llh;

uint16_t GPS_distanceToHome;        // distance to home point in meters
uint32_t GPS_distanceToHomeCm;
int16_t GPS_directionToHome;        // direction to home or hol point in degrees * 10
uint32_t GPS_distanceFlownInCm;     // distance flown since armed in centimeters

gpsSolutionData_t gpsSol;

GNSS_StateHandle gps;
void gps_init(GNSS_StateHandle *GNSS, UART_HandleTypeDef *huart)
{
	GNSS->huart = huart;
	GNSS->year = 0;
	GNSS->month = 0;
	GNSS->day = 0;
	GNSS->hour = 0;
	GNSS->min = 0;
	GNSS->sec = 0;
	GNSS->fixType = 0;
	GNSS->lon = 0;
	GNSS->lat = 0;
	GNSS->height = 0;
	GNSS->hMSL = 0;
	GNSS->hAcc = 0;
	GNSS->vAcc = 0;
	GNSS->gSpeed = 0;
	GNSS->headMot = 0;

	GNSS_LoadConfig(GNSS);

	HAL_UARTEx_ReceiveToIdle_DMA(&GNSS->huart, GNSS->uartWorkingBuffer, sizeof(GNSS->uartWorkingBuffer));

}


void GNSS_ParseBuffer(GNSS_StateHandle *GNSS)
{

	for (int var = 0; var <= 100; ++var) {
		if (GNSS->uartWorkingBuffer[var] == 0xB5
				&& GNSS->uartWorkingBuffer[var + 1] == 0x62) {
			if (GNSS->uartWorkingBuffer[var + 2] == 0x27
					&& GNSS->uartWorkingBuffer[var + 3] == 0x03) { //Look at: 32.19.1.1 u-blox 8 Receiver description
				GNSS_ParseUniqID(GNSS);
			} else if (GNSS->uartWorkingBuffer[var + 2] == 0x01
					&& GNSS->uartWorkingBuffer[var + 3] == 0x21) { //Look at: 32.17.14.1 u-blox 8 Receiver description
				GNSS_ParseNavigatorData(GNSS);
			} else if (GNSS->uartWorkingBuffer[var + 2] == 0x01
					&& GNSS->uartWorkingBuffer[var + 3] == 0x07) { //ook at: 32.17.30.1 u-blox 8 Receiver description
				GNSS_ParsePVTData(GNSS);
			} else if (GNSS->uartWorkingBuffer[var + 2] == 0x01
					&& GNSS->uartWorkingBuffer[var + 3] == 0x02) { // Look at: 32.17.15.1 u-blox 8 Receiver description
				GNSS_ParsePOSLLHData(GNSS);
			}
		}
	}
}

void GNSS_GetUniqID(GNSS_StateHandle *GNSS)
{
	HAL_UART_Transmit_IT(GNSS->huart, getDeviceID,
			sizeof(getDeviceID) / sizeof(uint8_t));
	HAL_UART_Receive_IT(GNSS->huart, gps.uartWorkingBuffer, 17);
}


void GNSS_GetNavigatorData(GNSS_StateHandle *GNSS)
{
	HAL_UART_Transmit_IT(GNSS->huart, getNavigatorData,
			sizeof(getNavigatorData) / sizeof(uint8_t));
	HAL_UART_Receive_IT(GNSS->huart, gps.uartWorkingBuffer, 28);
}


void GNSS_GetPOSLLHData(GNSS_StateHandle *GNSS)
{
	HAL_UART_Transmit_IT(GNSS->huart, getPOSLLHData,
			sizeof(getPOSLLHData) / sizeof(uint8_t));
	HAL_UART_Receive_IT(GNSS->huart, gps.uartWorkingBuffer, 36);
}

void GNSS_GetPVTData(GNSS_StateHandle *GNSS)
{
	HAL_UART_Transmit_IT(GNSS->huart, getPVTData, sizeof(getPVTData) / sizeof(uint8_t));
	HAL_UARTEx_ReceiveToIdle_DMA(GNSS->huart, gps.uartWorkingBuffer, 100);
}


void GNSS_ParseUniqID(GNSS_StateHandle *GNSS)
{
	for (int var = 0; var < 5; ++var) {
		GNSS->uniqueID[var] = gps.uartWorkingBuffer[10 + var];
	}
}


void GNSS_SetMode(GNSS_StateHandle *GNSS, short gnssMode)
{
	if (gnssMode == 0) {
		HAL_UART_Transmit(GNSS->huart, setPortableMode,sizeof(setPortableMode) / sizeof(uint8_t),HAL_MAX_DELAY);
	} else if (gnssMode == 1) {
		HAL_UART_Transmit(GNSS->huart, setStationaryMode,sizeof(setStationaryMode) / sizeof(uint8_t),HAL_MAX_DELAY);
	} else if (gnssMode == 2) {
		HAL_UART_Transmit(GNSS->huart, setPedestrianMode,sizeof(setPedestrianMode) / sizeof(uint8_t),HAL_MAX_DELAY);
	} else if (gnssMode == 3) {
		HAL_UART_Transmit(GNSS->huart, setAutomotiveMode,sizeof(setAutomotiveMode) / sizeof(uint8_t),HAL_MAX_DELAY);
	} else if (gnssMode == 4) {
		HAL_UART_Transmit(GNSS->huart, setAutomotiveMode,sizeof(setAutomotiveMode) / sizeof(uint8_t),HAL_MAX_DELAY);
	} else if (gnssMode == 5) {
		HAL_UART_Transmit(GNSS->huart, setAirbone1GMode,sizeof(setAirbone1GMode) / sizeof(uint8_t),HAL_MAX_DELAY);
	} else if (gnssMode == 6) {
		HAL_UART_Transmit(GNSS->huart, setAirbone2GMode,sizeof(setAirbone2GMode) / sizeof(uint8_t),HAL_MAX_DELAY);
	} else if (gnssMode == 7) {
		HAL_UART_Transmit(GNSS->huart, setAirbone4GMode,sizeof(setAirbone4GMode) / sizeof(uint8_t),HAL_MAX_DELAY);
	} else if (gnssMode == 8) {
		HAL_UART_Transmit(GNSS->huart, setWirstMode,sizeof(setWirstMode) / sizeof(uint8_t),HAL_MAX_DELAY);
	} else if (gnssMode == 9) {
		HAL_UART_Transmit(GNSS->huart, setBikeMode,sizeof(setBikeMode) / sizeof(uint8_t),HAL_MAX_DELAY);
	}
}


void GNSS_ParsePVTData(GNSS_StateHandle *GNSS)
{
	uShort.bytes[0] = gps.uartWorkingBuffer[10];
	GNSS->yearBytes[0]=gps.uartWorkingBuffer[10];
	uShort.bytes[1] = gps.uartWorkingBuffer[11];
	GNSS->yearBytes[1]=gps.uartWorkingBuffer[11];
	GNSS->year = uShort.uShort;
	GNSS->month = gps.uartWorkingBuffer[12];
	GNSS->day = gps.uartWorkingBuffer[13];
	GNSS->hour = gps.uartWorkingBuffer[14];
	GNSS->min = gps.uartWorkingBuffer[15];
	GNSS->sec = gps.uartWorkingBuffer[16];
	GNSS->fixType = gps.uartWorkingBuffer[26];
	GNSS->numSV = gps.uartWorkingBuffer[29];

	for (int var = 0; var < 4; ++var) {
		iLong.bytes[var] = gps.uartWorkingBuffer[var + 30];
		GNSS->lonBytes[var]= gps.uartWorkingBuffer[var + 30];
	}
	GNSS->lon = iLong.iLong;
	GNSS->fLon=(float)iLong.iLong/10000000.0;
	for (int var = 0; var < 4; ++var) {
		iLong.bytes[var] = gps.uartWorkingBuffer[var + 34];
		GNSS->latBytes[var]=gps.uartWorkingBuffer[var + 34];
	}
	GNSS->lat = iLong.iLong;
	GNSS->fLat=(float)iLong.iLong/10000000.0;
	for (int var = 0; var < 4; ++var) {
		iLong.bytes[var] = gps.uartWorkingBuffer[var + 38];
	}
	GNSS->height = iLong.iLong;
	GNSS->height = GNSS->height/10.0f;

	for (int var = 0; var < 4; ++var) {
		iLong.bytes[var] = gps.uartWorkingBuffer[var + 42];
		GNSS->hMSLBytes[var] = gps.uartWorkingBuffer[var + 42];
	}
	GNSS->hMSL = iLong.iLong;
	GNSS->hMSL = GNSS->hMSL/10.0f;

	for (int var = 0; var < 4; ++var) {
		uLong.bytes[var] = gps.uartWorkingBuffer[var + 46];
	}
	GNSS->hAcc = uLong.uLong;

	for (int var = 0; var < 4; ++var) {
		uLong.bytes[var] = gps.uartWorkingBuffer[var + 50];
	}
	GNSS->vAcc = uLong.uLong;

	for (int var = 0; var < 4; ++var) {
		iLong.bytes[var] = gps.uartWorkingBuffer[var + 66];
		GNSS->gSpeedBytes[var] = gps.uartWorkingBuffer[var + 66];
	}
	GNSS->gSpeed = iLong.iLong;

	for (int var = 0; var < 4; ++var) {
		iLong.bytes[var] = gps.uartWorkingBuffer[var + 70];
	}
	GNSS->headMot = iLong.iLong * 1e-5; // todo I'm not sure this good options.
}

void GNSS_ParseNavigatorData(GNSS_StateHandle *GNSS)
{
	uShort.bytes[0] = gps.uartWorkingBuffer[18];
	uShort.bytes[1] = gps.uartWorkingBuffer[19];
	GNSS->year = uShort.uShort;
	GNSS->month = gps.uartWorkingBuffer[20];
	GNSS->day = gps.uartWorkingBuffer[21];
	GNSS->hour = gps.uartWorkingBuffer[22];
	GNSS->min = gps.uartWorkingBuffer[23];
	GNSS->sec = gps.uartWorkingBuffer[24];
}

void GNSS_ParsePOSLLHData(GNSS_StateHandle *GNSS)
{
	for (int var = 0; var < 4; ++var) {
		iLong.bytes[var] = gps.uartWorkingBuffer[var + 10];
	}
	GNSS->lon = iLong.iLong;
	GNSS->fLon=(float)iLong.iLong/10000000.0;

	for (int var = 0; var < 4; ++var) {
		iLong.bytes[var] = gps.uartWorkingBuffer[var + 14];
	}
	GNSS->lat = iLong.iLong;
	GNSS->fLat=(float)iLong.iLong/10000000.0;

	for (int var = 0; var < 4; ++var) {
		iLong.bytes[var] = gps.uartWorkingBuffer[var + 18];
	}
	GNSS->height = iLong.iLong;

	for (int var = 0; var < 4; ++var) {
		iLong.bytes[var] = gps.uartWorkingBuffer[var + 22];
	}
	GNSS->hMSL = iLong.iLong;

	for (int var = 0; var < 4; ++var) {
		uLong.bytes[var] = gps.uartWorkingBuffer[var + 26];
	}
	GNSS->hAcc = uLong.uLong;

	for (int var = 0; var < 4; ++var) {
		uLong.bytes[var] = gps.uartWorkingBuffer[var + 30];
	}
	GNSS->vAcc = uLong.uLong;
}


/*!
 *  Sends the basic configuration: Activation of the UBX standard, change of NMEA version to 4.10 and turn on of the Galileo system.
 * @param GNSS Pointer to main GNSS structure.
 */
void GNSS_LoadConfig(GNSS_StateHandle *GNSS)
{

	Delay_ms(2500);
 	HAL_UART_Transmit(GNSS->huart, configUBX, sizeof(configUBX) / sizeof(uint8_t), HAL_MAX_DELAY);


	Delay_ms(250);

	HAL_UART_Transmit(GNSS->huart, setNMEA410, sizeof(setNMEA410) / sizeof(uint8_t), HAL_MAX_DELAY);
	Delay_ms(250);

	HAL_UART_Transmit(GNSS->huart, setGNSS, sizeof(setGNSS) / sizeof(uint8_t), HAL_MAX_DELAY);
	Delay_ms(250);

	HAL_UART_Transmit(GNSS->huart, setSampleRate, sizeof(setSampleRate) / sizeof(uint8_t), HAL_MAX_DELAY);
	Delay_ms(250);

	GNSS_SetMode(GNSS,Airbone2G);
	Delay_ms(250);

}

/*!
 *  Creates a checksum based on UBX standard.
 * @param class Class value from UBX doc.
 * @param messageID MessageID value from UBX doc.
 * @param dataLength Data length value from UBX doc.
 * @param payload Just payload.
 * @return  Returns checksum.
 */
#include <stdint.h>

void GNSS_Checksum(uint8_t class, uint8_t messageID, uint8_t dataLength, uint8_t *payload, uint8_t *ck_a, uint8_t *ck_b)
{
    // Initialize CK_A and CK_B
    *ck_a = 0;
    *ck_b = 0;

    // Add Class byte to checksum
    *ck_a += class;
    *ck_b += *ck_a;

    // Add Message ID byte to checksum
    *ck_a += messageID;
    *ck_b += *ck_a;

    // Add Length bytes to checksum (Low byte and High byte)
    *ck_a += dataLength;       // Low byte of length
    *ck_b += *ck_a;

    *ck_a += 0x00;             // High byte of length (assuming 0 for short messages)
    *ck_b += *ck_a;

    // Add payload bytes to checksum
    for (uint8_t i = 0; i < dataLength; i++)
    {
        *ck_a += payload[i];
        *ck_b += *ck_a;
    }
}

