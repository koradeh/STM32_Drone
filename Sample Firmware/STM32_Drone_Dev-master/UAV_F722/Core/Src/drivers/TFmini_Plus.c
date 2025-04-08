/*
 * TPmini_Plus.c
 *
 *  Created on: Oct 29, 2024
 *      Author: Vincent
 */

#include "TFmini_Plus.h"


static void TFmini_FrequencyMessage(uint16_t Frequency, uint8_t *message)
{
    // Prefix bytes
    message[0] = 0x5A;
    message[1] = 0x06;
    message[2] = 0x03;

    // Frequency in little-endian order
    message[3] = Frequency & 0xFF;          // Least significant byte (LL)
    message[4] = (Frequency >> 8) & 0xFF;   // Most significant byte (HH)

    // Temporary checksum calculation (sum of the first 7 bytes)
    uint16_t cumulative_sum = 0;
    for (int i = 0; i < 5; i++)
    {
        cumulative_sum += message[i];
    }

    message[5] = cumulative_sum  & 0xFF;           // Modulo 256
}


static void TFmini_BaudRateMessage(uint32_t BaudrRate, uint8_t *message)
{
    // Prefix bytes
    message[0] = 0x5A;
    message[1] = 0x08;
    message[2] = 0x06;

    // Baud rate in little-endian order
    message[3] = BaudrRate & 0xFF;          // Least significant byte (H1)
    message[4] = (BaudrRate >> 8) & 0xFF;   // Next byte (H2)
    message[5] = (BaudrRate >> 16) & 0xFF;  // Next byte (H3)
    message[6] = (BaudrRate >> 24) & 0xFF;  // Most significant byte (H4)

    // Temporary checksum calculation (sum of the first 7 bytes)
    uint16_t cumulative_sum = 0;
    for (int i = 0; i < 7; i++)
    {
        cumulative_sum += message[i];
    }

    message[7] = cumulative_sum  & 0xFF;           // Modulo 256
}



void TFmini_SendCommand(TFminiPlus_t *LiDar, TFmini_Plus_Command commandType, uint32_t value)
{
    switch (commandType)
    {
        case TFmini_FirmwareVersion:
        	HAL_UART_Transmit(LiDar->huart, TF_FirmwareVersion,sizeof(TF_FirmwareVersion) / sizeof(uint8_t),HAL_MAX_DELAY);
            break;

        case TFmini_SystemReset:
        	HAL_UART_Transmit(LiDar->huart, TF_SystemReset,sizeof(TF_SystemReset) / sizeof(uint8_t),HAL_MAX_DELAY);
            break;

        case TFmini_TriggerDetection:
        	HAL_UART_Transmit(LiDar->huart, TF_TriggerDetection,sizeof(TF_TriggerDetection) / sizeof(uint8_t),HAL_MAX_DELAY);
            break;

        case TFmini_OutputFormatCM:
        	HAL_UART_Transmit(LiDar->huart, TF_OutputFormatCM,sizeof(TF_OutputFormatCM) / sizeof(uint8_t),HAL_MAX_DELAY);
            break;

        case TFmini_OutputFormatPixHawk:
        	HAL_UART_Transmit(LiDar->huart, TF_OutputFormatPixHawk,sizeof(TF_OutputFormatPixHawk) / sizeof(uint8_t),HAL_MAX_DELAY);
            break;

        case TFmini_OutputFormatMM:
        	HAL_UART_Transmit(LiDar->huart, TF_OutputFormatMM,sizeof(TF_OutputFormatMM) / sizeof(uint8_t),HAL_MAX_DELAY);
            break;

        case TFmini_DisableOutput:
        	HAL_UART_Transmit(LiDar->huart, TF_DisableOutput,sizeof(TF_DisableOutput) / sizeof(uint8_t),HAL_MAX_DELAY);
            break;

        case TFmini_EnableOutput:
        	HAL_UART_Transmit(LiDar->huart, TF_EnableOutput,sizeof(TF_EnableOutput) / sizeof(uint8_t),HAL_MAX_DELAY);
            break;

        case TFmini_FactoryReset:
        	HAL_UART_Transmit(LiDar->huart, TF_FactoryReset,sizeof(TF_FactoryReset) / sizeof(uint8_t),HAL_MAX_DELAY);
            break;

        case TFmini_SetFreq:
        	uint8_t Freq_Message[6] = {0x00};
        	TFmini_FrequencyMessage((uint16_t)value, Freq_Message);

        	HAL_UART_Transmit(LiDar->huart, Freq_Message,sizeof(Freq_Message) / sizeof(uint8_t),HAL_MAX_DELAY);
            break;

        case TFmini_SetBaud:
        	uint8_t Baud_Message[8] = {0x00};
        	TFmini_BaudRateMessage(value, Baud_Message);

        	HAL_UART_Transmit(LiDar->huart, Baud_Message,sizeof(Baud_Message) / sizeof(uint8_t),HAL_MAX_DELAY);
            break;

        case TFmini_SaveSetting:
        	HAL_UART_Transmit(LiDar->huart, TF_SaveSetting,sizeof(TF_SaveSetting) / sizeof(uint8_t),HAL_MAX_DELAY);
            break;
    }
}

void TFmini_Plus_init(TFminiPlus_t *LiDar, UART_HandleTypeDef *huart,uint16_t Freq, uint32_t baud)
{
	LiDar->huart = huart;
	LiDar->Strength = 0;
	LiDar->Distance = 0;
	LiDar->Temp = 0;

	TFmini_SendCommand(LiDar,TFmini_SystemReset,0);
	Delay_ms(100);

	TFmini_SendCommand(LiDar,TFmini_SetFreq,1000);
	Delay_ms(100);

	if (baud != 115200)
	{
		TFmini_SendCommand(LiDar,TFmini_SetBaud,baud);
		Delay_ms(100);
	}

	HAL_UARTEx_ReceiveToIdle_DMA(LiDar->huart, LiDar->tfminiUartBuffer, sizeof(LiDar->tfminiUartBuffer));

}


bool TFmini_Plus_Process_Data(TFminiPlus_t *LiDar)
{
	if((LiDar->tfminiUartBuffer[TFmini_Header1] == LiDar->tfminiUartBuffer[TFmini_Header2]) && (LiDar->tfminiUartBuffer[TFmini_Header1] == TFmini_FrameHeader))
	{
		LiDar->Distance = (uint16_t)((LiDar->tfminiUartBuffer[TFmini_Dist_H]<<8) | (LiDar->tfminiUartBuffer[TFmini_Dist_L]));

		LiDar->Strength = (uint16_t)((LiDar->tfminiUartBuffer[TFmini_Strength_H]<<8) | (LiDar->tfminiUartBuffer[TFmini_Strength_L]));

		//if the data is unreliable
		if ((LiDar->Strength <= 100) || (LiDar->Strength == 65535))
		{
			return 0;
		}

		LiDar->Temp = (uint16_t)((LiDar->tfminiUartBuffer[TFmini_Temp_H]<<8) | (LiDar->tfminiUartBuffer[TFmini_Temp_L]));


		//Erase the header for future checking
		LiDar->tfminiUartBuffer[TFmini_Header1] = 0x00;
		return 1;
	}
	else
	{
		return 0;
	}
}





