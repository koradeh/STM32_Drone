/*
 * crsf.c
 *
 *  Created on: May 1, 2024
 *      Author: Vincent
 */
//  ******************************************************************************
//  * @file    crsf.c
//  * @author  Vincent Huang
//  * @version V1.0.0
//  * @date    01-May-2024
//  * @brief   Source file to parse data from crsf buffer
//  ******************************************************************************
#include "crsf.h"
#include "math.h"

//RC expo and rate define https://www.desmos.com/calculator/ilpkpeawsc
#define RATE 0.75
#define EXPO 0.25

#define channel_number 16



//For Channel

uint8_t rx_buffer[30];
uint16_t Channel_Packet[30]; //should be 26, add 4 just to prevent overflow or data corruption
uint8_t Link_Stat_Packet[16];
uint16_t Raw_Channel_Value[16];
extern uint16_t mapped_Channel[16];

uint8_t element_size = 0;//for decoding

//ELRS RC_Channel;

//For Link Stat
int Uplink_RSSI = 0;
uint8_t Uplink_LQ = 0;
uint8_t Uplink_TX_Power = 0;




void Packet_Type_Arrange(uint8_t* Uart_Buffer)
{
	if(Uart_Buffer[0] == CRSF_SYNC_BYTE)
	{
		switch(Uart_Buffer[2])
		{
			//-----------------------------------
			case CRSF_FRAMETYPE_LINK_STATISTICS:

				for (int i = 0; i<14 ;i++)//16
				{
					Link_Stat_Packet[i] = Uart_Buffer [i] ; //copy the buffer to a separated array before it got deleted
				}

			break;

			//-----------------------------------
			case CRSF_FRAMETYPE_RC_CHANNELS_PACKED:

				for (int i = 0; i<30 ;i++)
				{
					rx_buffer [i] = Uart_Buffer[i] ; //copy the buffer to a separated array before it got deleted
				}

			break;

			//-----------------------------------
		}
	}
	//Clear the Buffer
//	memset(Uart_Buffer,0,sizeof(Uart_Buffer));
}


//Decode RC Channels
void Parse_ELRS_Channels(void) //Channel_Packet[3]~Channel_Packet[24] are the bytes that contains channel data, and each channel takes 11bits.
{
	int channel_bit = 11; //each channel is 11 bits
	int bit_index = 0; //because we have 176bits, this index determine the current bit number when parsing the channels

	for (int i = 0; i<channel_number ; i++)
	{
		int byte_index = (bit_index / 8) + 3;       // Calculate the index of the byte where the current channel starts, add 3 cuz we start at byte 3
		int bit_offset = bit_index % 8;       // Calculate the bit position inside the byte where the channel starts

		// Read the first part of the channel value from the current byte, shifting right to drop preceding bits
		uint32_t value = rx_buffer[byte_index] >> bit_offset;

		// Read the next byte, shift left, and combine it to fill the rest of the bits of the current channel
		value |= (uint32_t)rx_buffer[byte_index + 1] << (8 - bit_offset);

		// If the channel bits span into a third byte, read and combine it
		if (bit_offset > 5)
		{  // When bit_offset is greater than 5, bits spill into a third byte
		    value |= (uint32_t)rx_buffer[byte_index + 2] << (16 - bit_offset);
		}

		// Mask off to get only the 11 bits of interest for the current channel
        Raw_Channel_Value[i] = value & ((1 << channel_bit) - 1);

		// Move the bit index forward by the number of bits per channel
		bit_index += channel_bit;
	}
	map_channels(); //map the channel output from 988 to 2012
}


//	mapped_Channel[i] =(input[i] - Raw_min) * ((Output_max - Output_min) / (Raw_max - Raw_min)) + Output_min;
void map_channels()
{
#ifdef RATE
	for(int i = 0; i < 16 ; i++) //Throttle is linear
	{
		if((i!=0) && (i<4))
		{
			mapped_Channel[i] =(Raw_Channel_Value[i] - 172) *0.624771202  + 988; //Map to linear range first

			double Normalized_Input_Value = (double)(mapped_Channel[i]-1500)/512; //convert to between -1 to 1
			double Normalized_Output_Value = RATE * Normalized_Input_Value + EXPO * pow(Normalized_Input_Value, 3); //calculate expo and rate

			mapped_Channel[i] =(uint16_t)(512 * Normalized_Output_Value + 1500); //convert back to original range
		}
		else
		{
			mapped_Channel[i] =(Raw_Channel_Value[i] - 172) *0.624771202  + 988; //Normal Operation
		}
	}

#else
	for (int i = 0; i < 16 ; i++)
	{
		mapped_Channel[i] =(Raw_Channel_Value[i] - 172) *0.624771202  + 988;
	}
#endif
}



//process in case of the packet that contains link statistic
void Process_Link_Stat_Packets(void)
{

		//only when both conditions are met then we store the data, so the value doesn't change all the time
		if((rx_buffer[0] == CRSF_SYNC_BYTE)&&(rx_buffer[1] == 12)&&(rx_buffer[2] == CRSF_FRAMETYPE_LINK_STATISTICS) ) //if the length [1] and the channel size [2] is correct, then we copy the packet.
		{
			for (int i = 0; i<16 ;i++)
			{
				Link_Stat_Packet[i]=rx_buffer [i] ; //copy the buffer to a separated array before it got deleted
			}

			Uplink_RSSI = (Link_Stat_Packet[3]*-1); //in dbm
			Uplink_LQ   = Link_Stat_Packet[5]; //in percent
		}
}


