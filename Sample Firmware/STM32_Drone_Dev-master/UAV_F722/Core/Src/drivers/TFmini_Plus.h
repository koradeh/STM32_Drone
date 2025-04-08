/*
 * TFmini_Plus.h
 *
 *  Created on: Oct 29, 2024
 *      Author: Vincent
 */

#ifndef SRC_DRIVERS_TFMINI_PLUS_H_
#define SRC_DRIVERS_TFMINI_PLUS_H_

#include "stdint.h"
#include "stdbool.h"
#include "../../Inc/main.h"
#include "../../Inc/Timers.h"


/*message structure
 * Byte0	Byte1	Byte2		Byte3		Byte4			Byte5			Byte6		Byte7		Byte8
 * 0x59		0x59	Dist_L		Dist_H		Strength_L		Strength_H		Temp_L		Temp_H		CheckSum
 */

typedef struct
{
	UART_HandleTypeDef *huart;

	uint8_t tfminiUartBuffer [9];

	uint16_t Distance;
	uint16_t Strength;
	uint16_t Temp;


}TFminiPlus_t;

/*Distance*/
/* Represents the output of the distance value detected by TFmini Plus, with the unit in cm
by default. This value is interpreted into the decimal value in the range of 0-1200. When the signal strength
is lower than 100 or equal to 65535, the detection is unreliable, TFmini Plus will set distance value to 0. */

/*Strength*/
/*Represents the signal strength with the default value in the range of 0-65535. After the distance
mode is set, the longer the measurement distance is, the lower the signal strength will be; the lower the
reflectivity is, the lower the signal strength will be. When the signal strength is lower than 100 or equal to
65535, the detection is unreliable, TFmini Plus will set distance value to 0. */

/*Temp*/
/*Represents the chip temperature of TFmini Plus. Degree centigrade = Temp / 8 -256 */

#define TFmini_FrameHeader 0x59
typedef enum
{
	TFmini_Header1, 	//0x59, frame header, same for each frame
	TFmini_Header2,
	TFmini_Dist_L, 		//Dist_L distance value lower by 8 bits
	TFmini_Dist_H, 		//Dist_L distance value higher by 8 bits
	TFmini_Strength_L, 	//Strength_L low 8 bits
	TFmini_Strength_H, 	//Strength_L high 8 bits
	TFmini_Temp_L,		//Temp_L low 8 bits (suit for version later than V1.3.0)
	TFmini_Temp_H, 		//Temp_H high 8 bits (suit for version later than V1.3.0)
	TFmini_CheckSum, 	//Checksum is the lower 8 bits of the cumulative sum of the numbers of the first 8 bytes
}TFmini_Plus_Data;


//Command (frame header is 0x5A) datasheet 7.4
/*
 * Byte0	Byte1	Byte2	Byte3	...
 * 0x5A		Len		ID		Payload		Checksum
 *
 */
static const uint8_t TF_FirmwareVersion[]=		{0x5A, 0x04, 0x01, 0x5F};
static const uint8_t TF_SystemReset[]=			{0x5A, 0x04, 0x02, 0x60};
static const uint8_t TF_TriggerDetection[]= 	{0x5A, 0x04, 0x04, 0x62};
static const uint8_t TF_OutputFormatCM[]=		{0x5A, 0x05, 0x05, 0x01, 0x65};
static const uint8_t TF_OutputFormatPixHawk[]=	{0x5A, 0x05, 0x05, 0x02, 0x66};
static const uint8_t TF_OutputFormatMM[]=		{0x5A, 0x05, 0x05, 0x06, 0x6A};
static const uint8_t TF_DisableOutput[]=		{0x5A, 0x05, 0x07, 0x00, 0x66};
static const uint8_t TF_EnableOutput[]=			{0x5A, 0x05, 0x07, 0x01, 0x67};
static const uint8_t TF_FactoryReset[]=			{0x5A, 0x04, 0x10, 0x6E};
static const uint8_t TF_SaveSetting[]=			{0x5A, 0x04, 0x11, 0x6F};

typedef enum
{
	TFmini_FirmwareVersion,
	TFmini_SystemReset,
	TFmini_TriggerDetection,
	TFmini_OutputFormatCM,
	TFmini_OutputFormatPixHawk,
	TFmini_OutputFormatMM,
	TFmini_DisableOutput,
	TFmini_EnableOutput,
	TFmini_FactoryReset,
	TFmini_SetFreq,
	TFmini_SetBaud,
	TFmini_SaveSetting,

}TFmini_Plus_Command;
void TFmini_SendCommand(TFminiPlus_t *LiDar, TFmini_Plus_Command commandType, uint32_t value);


void TFmini_Plus_init(TFminiPlus_t *LiDar, UART_HandleTypeDef *huart,uint16_t Freq, uint32_t baud);
bool TFmini_Plus_Process_Data(TFminiPlus_t *LiDar);



#endif /* SRC_DRIVERS_TFMINI_PLUS_H_ */
