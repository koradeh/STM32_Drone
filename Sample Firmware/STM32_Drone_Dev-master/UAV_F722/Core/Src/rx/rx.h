/*
 * rx.h
 *
 *  Created on: Nov 16, 2024
 *      Author: Vincent
 */

#ifndef SRC_RX_DRIVERS_RX_H_
#define SRC_RX_DRIVERS_RX_H_

#include "drivers/crsf.h"
#include "../flight/ResourceDefine.h"
#include "../../Inc/usart.h"


typedef struct
{
	UART_HandleTypeDef *huart;
	crsfData_t rxChannel;

}rx_data_t;

extern rx_data_t rx;

void rxInit(void);
void processRX(void);

#endif /* SRC_RX_DRIVERS_RX_H_ */
