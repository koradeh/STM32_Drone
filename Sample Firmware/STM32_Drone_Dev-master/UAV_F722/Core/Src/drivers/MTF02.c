/*
 * MTF02.c
 *
 *  Created on: Nov 4, 2024
 *      Author: Vincent
 */


#include "MTF02.h"

void mtf02_init(MTF_Data_t *mtf02, UART_HandleTypeDef *huart)
{
	mtf02->huart = huart;

	HAL_UARTEx_ReceiveToIdle_DMA(mtf02->huart, mtf02->MTF_UartWorkingBuffer, sizeof(mtf02->MTF_UartWorkingBuffer));
}

static bool mtf02_check_sum(MTF_Data_t *mtf02)
{
	uint8_t calculated_checksum = 0;

	// Calculate checksum by summing up the header and payload bytes
	for (uint8_t i = 0; i < (MTF_messageMaxLen - 1); i++)
	{
	        calculated_checksum += mtf02->MTF_UartWorkingBuffer[i];
	}

	// The last byte in the buffer is the received checksum
	uint8_t received_checksum = mtf02->MTF_UartWorkingBuffer[MTF_messageMaxLen - 1];

	// Return true if checksums match, false otherwise
	return (calculated_checksum == received_checksum);

}

void parse_mtf02_data(MTF_Data_t *mtf02)
{

	if((mtf02_check_sum(mtf02))&&((mtf02->MTF_UartWorkingBuffer[0] == MTF_FrameHeader) && (mtf02->MTF_UartWorkingBuffer[1] == MTF_DeviceID) && (mtf02->MTF_UartWorkingBuffer[2] == MTF_SystemID) && (mtf02->MTF_UartWorkingBuffer[3] == MTF_MessageID)))
	{




		// Parse payload into struct fields, starting at offset MTF_HEADER_SIZE + 1
		    mtf02->MTF_SystimeMs = 		(mtf02->MTF_UartWorkingBuffer[MTF_HEADER_SIZE + 4] << 24)|
		    							(mtf02->MTF_UartWorkingBuffer[MTF_HEADER_SIZE + 3] << 16)|
										(mtf02->MTF_UartWorkingBuffer[MTF_HEADER_SIZE + 2] << 8) |
										(mtf02->MTF_UartWorkingBuffer[MTF_HEADER_SIZE + 1]);

		    mtf02->MTF_DistMM =			(mtf02->MTF_UartWorkingBuffer[MTF_HEADER_SIZE + 8] << 24)|
		                        		(mtf02->MTF_UartWorkingBuffer[MTF_HEADER_SIZE + 7] << 16)|
										(mtf02->MTF_UartWorkingBuffer[MTF_HEADER_SIZE + 6] << 8) |
										(mtf02->MTF_UartWorkingBuffer[MTF_HEADER_SIZE + 5]);

		    mtf02->MTF_DistStrength = 	mtf02->MTF_UartWorkingBuffer[MTF_HEADER_SIZE + 9];
		    mtf02->MTF_DistAccuracy = 	mtf02->MTF_UartWorkingBuffer[MTF_HEADER_SIZE + 10];
		    mtf02->MTF_DistStatus = 	mtf02->MTF_UartWorkingBuffer[MTF_HEADER_SIZE + 11];
		   // mtf02->Reserved1 = 			mtf02->MTF_UartWorkingBuffer[MTF_HEADER_SIZE + 12];

		    mtf02->MTF_Flow_velX = 	(int16_t)(mtf02->MTF_UartWorkingBuffer[MTF_HEADER_SIZE + 14] << 8) |
		                           	   		 (mtf02->MTF_UartWorkingBuffer[MTF_HEADER_SIZE + 13]);

		    mtf02->MTF_Flow_velY = 	(int16_t)(mtf02->MTF_UartWorkingBuffer[MTF_HEADER_SIZE + 16] << 8) |
		                           	   	     (mtf02->MTF_UartWorkingBuffer[MTF_HEADER_SIZE + 15]);

		    mtf02->MTF_FlowQuality = 	mtf02->MTF_UartWorkingBuffer[MTF_HEADER_SIZE + 17];
		    mtf02->MTF_FlowStatus = 	mtf02->MTF_UartWorkingBuffer[MTF_HEADER_SIZE + 18];

		   // mtf02->Reserved2 = 			(mtf02->MTF_UartWorkingBuffer[MTF_HEADER_SIZE + 20] << 8) |
		                       	   	   	//(mtf02->MTF_UartWorkingBuffer[MTF_HEADER_SIZE + 19]);
	}
}







//do later, refer to micoari.cn/docs/Micolink-xie-yi-ding-yi-yu-jie-xi


