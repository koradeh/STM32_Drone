/*
 * dShot.h
 *
 *  Created on: Nov 21, 2024
 *      Author: Vincent
 */

#ifndef SRC_DRIVERS_DSHOT_H_
#define SRC_DRIVERS_DSHOT_H_

#include "tim.h"    	// header from stm32cubemx code generate
#include <stdbool.h>
#include <math.h>		// lrintf


/* User Configuration */
// Timer Clock
#define TIMER1_CLOCK				216000000
#define TIMER3_CLOCK				108000000	// 108MHz

// MOTOR 1 (PA8) - TIM1 Channel 1, DMA2 Stream 1
#define MOTOR_1_TIM             (&htim1)
#define MOTOR_1_TIM_CHANNEL     TIM_CHANNEL_1

// MOTOR 2 (PA9) - TIM1 Channe2 , DMA2 Stream 1
#define MOTOR_2_TIM             (&htim1)
#define MOTOR_2_TIM_CHANNEL     TIM_CHANNEL_2

// MOTOR 3 (PC8) - TIM3 Channel 3, DMA1 Stream 7
#define MOTOR_3_TIM             (&htim3)
#define MOTOR_3_TIM_CHANNEL     TIM_CHANNEL_3

// MOTOR 4 (PC9) - TIM3 Channel 4, DMA1 Stream 2
#define MOTOR_4_TIM             (&htim3)
#define MOTOR_4_TIM_CHANNEL     TIM_CHANNEL_4


/* Definition */
#define MHZ_TO_HZ(x) 			((x) * 1000000)

#define DSHOT600_HZ     		MHZ_TO_HZ(12)  //6
#define DSHOT300_HZ     		MHZ_TO_HZ(6) //3
#define DSHOT150_HZ     		MHZ_TO_HZ(3)//1.5

#define MOTOR_BIT_0            	7
#define MOTOR_BIT_1            	14
#define MOTOR_BITLENGTH        	20

#define DSHOT_FRAME_SIZE       	16
#define DSHOT_DMA_BUFFER_SIZE   18 /* resolution + frame reset (2us) */

#define DSHOT_MIN_THROTTLE      48
#define DSHOT_MAX_THROTTLE     	2047
#define DSHOT_RANGE 			(DSHOT_MAX_THROTTLE - DSHOT_MIN_THROTTLE)


/* Enumeration */
typedef enum
{
    DSHOT150,
    DSHOT300,
    DSHOT600
} dshot_type_e;


/* Functions */
//Check Value
void dshot_check_threshold(uint16_t* motor_cmd, bool Armed);
void dshot_init(dshot_type_e dshot_type);
void dshot_write(uint16_t* motor_value);

#endif /* SRC_DRIVERS_DSHOT_H_ */
