/*
 * QMC5883L.c
 *
 *  Created on: Oct 25, 2024
 *      Author: Vincent
 */


#include"QMC5883L.h"
#include "../CommonMath/maths.h"


bool QMC_Reset(QMC_t *qmc)
{
	uint8_t resetQMC = 0x00;

	resetQMC |= (1<<SOFT_RST);

	if(HAL_I2C_Mem_Write(qmc->i2c, QMC_Write_Address, Control_Register2, 1, &resetQMC, 1, 3000)!=HAL_OK)return 1;
	return 0;
}


bool QMC_init(QMC_t *qmc,I2C_HandleTypeDef *i2c,uint8_t Output_Data_Rate)
{
	qmc->i2c=i2c;

	//prepare to set the output mode to continuous mode, scale = 8G, and 64samples
	uint8_t mag_config = 0x00;
	//it is suggested that the setReset Period to write 0x01
	uint8_t Set_Reset_Period = 0x01;

	mag_config |= (Continuous<<Mag_Mode) | (g8<<Mag_Scale)|(Sample64<<Mag_OSRatio);

	switch(Output_Data_Rate)
	{
	case 200:
		mag_config |= (HZ200<<Mag_ODR);
		break;
	case 100:
		mag_config |= (HZ100<<Mag_ODR);
		break;
	case 50:
		mag_config |= (HZ50<<Mag_ODR);
		break;
	case 10:
		mag_config |= (HZ10<<Mag_ODR);
		break;
	}

	//QMC_Reset(qmc);
	if(HAL_I2C_Mem_Write(qmc->i2c, QMC_Write_Address, SetResetPeriod, 1, &Set_Reset_Period, 1, 3000)!=HAL_OK)return 1;
	if(HAL_I2C_Mem_Write(qmc->i2c, QMC_Write_Address, Control_Register1, 1, &mag_config, sizeof(mag_config), 3000)!=HAL_OK)return 1;

	return 0;
}

bool QMC_read(QMC_t *qmc)
{
	  qmc->datas[Status]=0;
	  HAL_I2C_Mem_Read(qmc->i2c, 0x1A, QMC_STATUS, 1, qmc->datas, 1, 3000);


	  //check status
	  if((qmc->datas[Status]&0x01)==1)
	  {
		  HAL_I2C_Mem_Read(qmc->i2c, 0x1A, 0x00, 1, qmc->datas, 6, 3000);
		  qmc->Xaxis= (int16_t)((qmc->datas[X_MSB]<<8) | qmc->datas[X_LSB]);
		  qmc->Yaxis= (int16_t)((qmc->datas[Y_MSB]<<8) | qmc->datas[Y_LSB]);
		  qmc->Zaxis= (int16_t)((qmc->datas[Z_MSB]<<8) | qmc->datas[Z_LSB]);

		  qmc->magADC.x = (float)qmc->Xaxis - qmc->Xtrim;
		  qmc->magADC.y = (float)qmc->Yaxis - qmc->Ytrim;
		  qmc->magADC.z = (float)qmc->Zaxis - qmc->Ztrim;

		  //convert to Gauss

		  qmc->compas=RADIANS_TO_DEGREES(atan2_approx(qmc->Yaxis,qmc->Xaxis));

		  if(qmc->compas>0)
		  {
			  qmc->heading= qmc->compas;
		  }
		  else
		  {
			  qmc->heading=360+qmc->compas;
		  }
	  }
	  else
	  {
		  return 1;
	  }
return 0;
}

float QMC_readHeading(QMC_t *qmc)
{
	QMC_read(qmc);
	return qmc->heading;
}


