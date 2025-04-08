/*
 * rx.c
 *
 *  Created on: Nov 16, 2024
 *      Author: Vincent
 */


#include "rx.h"


rx_data_t rx;


void rxInit(void)
{
	HAL_UARTEx_ReceiveToIdle_DMA(&rxBusType, rx.rxChannel.uartWorkingBuffer, sizeof(rx.rxChannel.uartWorkingBuffer));

}
void processRX(void)
{
	Packet_Type_Arrange(rx.rxChannel.uartWorkingBuffer);
	Parse_ELRS_Channels(rx.rxChannel.mapped_Channel);//Process the data in RxBuffer
}
