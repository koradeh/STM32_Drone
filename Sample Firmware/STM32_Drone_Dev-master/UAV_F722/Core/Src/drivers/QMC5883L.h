/*
 * QMC5883L.h
 *
 *  Created on: Oct 25, 2024
 *      Author: Vincent
 */

#ifndef SRC_DRIVERS_QMC5883L_H_
#define SRC_DRIVERS_QMC5883L_H_

#include "main.h"
#include "stm32f7xx.h"
#include "stdbool.h"

#include "../CommonMath/vector.h"

#define QMC_OK 0
#define QMC_FALSE 1

#define QMC_X_LSB			0x00
#define QMC_X_MSB			0x01
#define QMC_Y_LSB			0x02
#define QMC_Y_MSB			0x03
#define QMC_Z_LSB			0x04
#define QMC_Z_MSB			0x05

#define QMC_STATUS			0x06

#define QMC_Write_Address	0x1A //can be apply to read too because HAL add last bit automatically so it becomes 1B
#define Control_Register1 	0x09
#define Control_Register2 	0x0A
#define SetResetPeriod		0x0B

typedef enum
{
	Mag_Mode,
	Mag_ODR = 2,
	Mag_Scale = 4,
	Mag_OSRatio = 6,
}Control_Reg1;

typedef enum
{
	//MODE
	Standby,
	Continuous,

	//ODR
	HZ10 = 0,
	HZ50,
	HZ100,
	HZ200,

	//Scale
	g2 = 0,
	g8,

	//OSRatio
	Sample512 = 0,
	Sample256,
	Sample128,
	Sample64,
}Control_Reg1_Options;

typedef enum
{
	INT_Enable,
	ROL_PNT = 6, //pointer rollover
	SOFT_RST,
}Control_Reg2;

typedef enum
{
	Status,
	X_LSB = 0,
	X_MSB,
	Y_LSB,
	Y_MSB,
	Z_LSB,
	Z_MSB,
}QMC_Data;

typedef struct QMC
{
	I2C_HandleTypeDef   *i2c;
	uint8_t				Control_Register;
	uint8_t             datas[6];
	int16_t             Xaxis;
	int16_t             Yaxis;
	int16_t             Zaxis;

	float				Xtrim;
	float 				Ytrim;
	float 				Ztrim;

	vector3_t			magADC;
	float			    heading;
	float               compas;

}QMC_t;



bool QMC_Reset(QMC_t *qmc);
bool QMC_init(QMC_t *qmc,I2C_HandleTypeDef *i2c,uint8_t Output_Data_Rate);
bool QMC_read(QMC_t *qmc);
float   QMC_readHeading(QMC_t *qmc);

#endif /* SRC_DRIVERS_QMC5883L_H_ */
