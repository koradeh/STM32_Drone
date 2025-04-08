/*
 * ResourceDefine.h
 *
 *  Created on: Oct 30, 2024
 *      Author: Vincent
 */

#ifndef SRC_FLIGHT_RESOURCEDEFINE_H_
#define SRC_FLIGHT_RESOURCEDEFINE_H_

#include "../../Inc/i2c.h"
#include "../../Inc/spi.h"
#include "../../Inc/main.h"
#include "../../Inc/usart.h"
#include "../../Inc/adc.h"
#include "../../Inc/tim.h"


/*Resource Mapping*/

//System Timer
#define sysTimerType   htim6
//imu
#define imuBusType hspi1
#define spi1CSport SPI1_CS_GPIO_Port
#define spi1CSpin  SPI1_CS_Pin
#define imuDataReadyPin GPIO_PIN_4

//Receiver
#define rxBusType  huart1
//baro
#define baroBuyType hi2c1

//mag
#define magBusType hi2c1


//gps
#define gpsBusType huart5

//RangeFinder
#define RangeFinderBusType huart4

//Optical Flow
#define OFSensorBusyType	huart2

//Onboard ADC
#define BattADC	   hadc1

//Onboard LED
#define imuLEDport GPIOC
#define imuLEDpin  GPIO_PIN_14

#define mcuLEDport GPIOC
#define mcuLEDpin  GPIO_PIN_14

//LED Strip
#define ledStripPort GPIOB
#define ledStripPin	 GPIO_PIN_3

//Beeper
#define beeperPort	GPIOB
#define beeperPin	GPIO_PIN_2

//Button
#define GP_ButtonPin GPIO_PIN_0


#endif /* SRC_FLIGHT_RESOURCEDEFINE_H_ */
