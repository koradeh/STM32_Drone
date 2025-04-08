/*
 * crsf.h
 *
 *  Created on: May 1, 2024
 *      Author: Vincent
 */

#ifndef INC_CRSF_H_
#define INC_CRSF_H_


#include "main.h"



//frame size
typedef enum
{

	CRSF_SYNC_BYTE = 0xC8,//<------Start byte on the FC side

    CRSF_FRAMETYPE_GPS = 0x02,
    CRSF_FRAMETYPE_VARIO = 0x07,
    CRSF_FRAMETYPE_BATTERY_SENSOR = 0x08,
    CRSF_FRAMETYPE_BARO_ALTITUDE = 0x09,
    CRSF_FRAMETYPE_LINK_STATISTICS = 0x14,
    CRSF_FRAMETYPE_OPENTX_SYNC = 0x10,
    CRSF_FRAMETYPE_RADIO_ID = 0x3A,
    CRSF_FRAMETYPE_RC_CHANNELS_PACKED = 0x16, //<-------------use this
    CRSF_FRAMETYPE_ATTITUDE = 0x1E,
    CRSF_FRAMETYPE_FLIGHT_MODE = 0x21,
    // Extended Header Frames, range: 0x28 to 0x96
    CRSF_FRAMETYPE_DEVICE_PING = 0x28,
    CRSF_FRAMETYPE_DEVICE_INFO = 0x29,
    CRSF_FRAMETYPE_PARAMETER_SETTINGS_ENTRY = 0x2B,
    CRSF_FRAMETYPE_PARAMETER_READ = 0x2C,
    CRSF_FRAMETYPE_PARAMETER_WRITE = 0x2D,

    //CRSF_FRAMETYPE_ELRS_STATUS = 0x2E, ELRS good/bad packet count and status flags

    CRSF_FRAMETYPE_COMMAND = 0x32,
    // KISS frames
    CRSF_FRAMETYPE_KISS_REQ  = 0x78,
    CRSF_FRAMETYPE_KISS_RESP = 0x79,
    // MSP commands
    CRSF_FRAMETYPE_MSP_REQ = 0x7A,   // response request using msp sequence as command
    CRSF_FRAMETYPE_MSP_RESP = 0x7B,  // reply with 58 byte chunked binary
    CRSF_FRAMETYPE_MSP_WRITE = 0x7C, // write with 8 byte chunked binary (OpenTX outbound telemetry buffer limit)
    // Ardupilot frames
    CRSF_FRAMETYPE_ARDUPILOT_RESP = 0x80,
} crsf_frame_type_e;



typedef struct
{
	uint16_t Throttle;
	uint16_t Rudder;
	uint16_t Elevator;
	uint16_t Aileron;
	uint16_t Arm;

}ELRS;

typedef struct
{
	int RSSI1; //Antenna 1 (dBm * -1)
	int RSSI2; //Antenna 2
	uint8_t LQ; //Link Quality (%)
	uint8_t SNR; //
	uint8_t Active_Ant; //Diversity active antenna 1 ant=0, 2 ant=1
	uint8_t RF_Mode;
	uint8_t TX_Power;
	int RSSI_Down; //Downlink RSSI
	uint8_t LQ_Down; //Downlink Link Quality
	uint8_t SNR_Down; //Downlink SNR

	//Uplink is ground to UAV, Downlink is UAV to ground
}Link_Stat;


////Channels Each channel is 11 bit *16 so 22bytes
//typedef struct crsf_channels_s
//{
//    unsigned ch0 : 11;
//    unsigned ch1 : 11;
//    unsigned ch2 : 11;
//    unsigned ch3 : 11;
//    unsigned ch4 : 11;
//    unsigned ch5 : 11;
//    unsigned ch6 : 11;
//    unsigned ch7 : 11;
//    unsigned ch8 : 11;
//    unsigned ch9 : 11;
//    unsigned ch10 : 11;
//    unsigned ch11 : 11;
//    unsigned ch12 : 11;
//    unsigned ch13 : 11;
//    unsigned ch14 : 11;
//    unsigned ch15 : 11;
//} crsf_channels_t;


void Packet_Type_Arrange(uint8_t* Uart_Buffer);

void Parse_ELRS_Channels(void);

void Process_Link_Stat_Packets(void);

void map_channels(void);

#endif /* INC_CRSF_H_ */
