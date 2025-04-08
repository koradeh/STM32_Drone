/*
 * MTF02.h
 *
 *  Created on: Nov 4, 2024
 *      Author: Vincent
 */

#ifndef SRC_DRIVERS_MTF02_H_
#define SRC_DRIVERS_MTF02_H_

#include "stdint.h"
#include "stdbool.h"
#include "../../Inc/usart.h"

#define MTF_SampleRate 100

#define MTF_FrameHeader 0xEF
#define MTF_DeviceID	0x0F
#define MTF_SystemID	0x00
#define MTF_MessageID	0x51

#define MTF_HEADER_SIZE 5 //5 header then len, then payload

#define MTF_PayloadLen	0x14
#define MTF_messageMaxLen	(MTF_PayloadLen+0x07)

//Micolink protocol
/*
 * ----------------------Message Structure----------------------------------
 * Header	DevID	SysID	MesID	xx		PayloadLen	Payload		Checksum
 * 0xEF		0x0F	0x00	0x51	0-0xFF	0x14		payload		checksum
 *
 *
 * ----------------------Payload Structure-----------------------------------
 * uint32_t MTF_Systime			ms
 * uint32_t MTF_Dist			mm
 * uint8_t	MTF_DistStrength
 * uint8_t	MTF_DistAccuracy
 * uint8_t	MTF_DistStatus		1 = can be used
 * uint8_t	Reserved
 * uint16_t	MTF_FlowVx			cm/s @1m
 * uint16_t MTF_FlowVy			cm/s @1m
 * uint8_t	MTF_FlowQuality		Higher better
 * uint8_t	MTF_FlowStatus		1 = can be used
 * uint16_t Reserved
 */

typedef struct
{
	UART_HandleTypeDef *huart;

	uint8_t MTF_UartWorkingBuffer[MTF_messageMaxLen];
	uint32_t MTF_SystimeMs;
	uint32_t MTF_DistMM;
	uint8_t	MTF_DistStrength;
	uint8_t	MTF_DistAccuracy;
	uint8_t	MTF_DistStatus;
	uint8_t	Reserved1;
	int16_t MTF_Flow_velX;
	int16_t MTF_Flow_velY;
	uint8_t	MTF_FlowQuality;
	uint8_t	MTF_FlowStatus;
	uint16_t Reserved2;
}MTF_Data_t;


void mtf02_init(MTF_Data_t *mtf02, UART_HandleTypeDef *huart);
void parse_mtf02_data(MTF_Data_t *mtf02);

#endif /* SRC_DRIVERS_MTF02_H_ */
