#ifndef __ICM42688P_H
#define __ICM42688P_H

#include <stdint.h>
#include "../Src/flight/ResourceDefine.h"


#define IMU_Freq 2000

/*Hardware Connection*/
#define ICM42688_SPI_HANDLE imuBusType
#define ICM42688_CS_GPIO spi1CSport
#define ICM42688_CS_PIN spi1CSpin

/*System Bank Define*/
#define ICM426XX_RA_REG_BANK_SEL                    0x76
#define ICM426XX_BANK_SELECT0                       0x00
#define ICM426XX_BANK_SELECT1                       0x01
#define ICM426XX_BANK_SELECT2                       0x02
#define ICM426XX_BANK_SELECT3                       0x03
#define ICM426XX_BANK_SELECT4                       0x04


/*Bank 0 Device Configuration*/
#define ICM426XX_DEV_CONFIG 	0x11
typedef enum
{
	MODE0_MODE3, //<<4 (default)
	MODE1_MODE2, //<<4
	SOFT_RESET =1,//<<0
}SPI_MODE_AND_SOFT_RESET;

#define ICM426XX_DRIVE_CONFIG 	0x13
#define ICM42688P_INT_CONFIG  	0x14
typedef enum
{
	/*Function*/
	Pulsed_Mode,
	Latched_Mode,
	Open_Drain = 0,
	Push_Pull,
	Active_Low = 0,
	Active_High,

	/*Bit Location*/
	INT1_Polarity = 0,
	INT1_Drive_Circuit,
	INT1_Mode,
	INT2_Polarity,
	INT2_Drive_Circuit,
	INT2_Mode,
}INT_CONFIG;

#define ICM426XX_FIFO_CONFIG 	0x16

/*Data Registers*/ /*Located In Bank 0*/
#define ICM426XX_TEMP_DATA1 	0x1D
#define ICM426XX_TEMP_DATA0 	0x1E
#define ICM42688_ACCEL_DATA_X1 	0x1F //Only the first one need to be located
#define ICM42688_ACCEL_DATA_X0 	0x20
#define ICM42688_ACCEL_DATA_Y1 	0x21
#define ICM42688_ACCEL_DATA_Y0 	0x22
#define ICM42688_ACCEL_DATA_Z1 	0x23
#define ICM42688_ACCEL_DATA_Z0 	0x24
#define ICM42688_GYRO_DATA_X1 	0x25//Only the first one need to be located
#define ICM42688_GYRO_DATA_X0 	0x26
#define ICM42688_GYRO_DATA_Y1 	0x27
#define ICM42688_GYRO_DATA_Y0 	0x28
#define ICM42688_GYRO_DATA_Z1 	0x29
#define ICM42688_GYRO_DATA_Z0 	0x2A

#define ICM42688_TMST_FYNCH		0x2B
#define ICM42688_TMST_FYNCL		0x2C
#define ICM42688_INT_STATUS		0x2D
typedef enum
{
	AGC_RDY_INT,
	FIFO_FULL_INT,
	FIFO_THS_INT,
	DATA_RDY_INT,
	RESET_DONE_INT,
	PLL_RDY_INT,
	UI_FSYNC_INT,
}INT_STATUS; //These register will be cleared to 0 after it's read

/*Un-used Registers*/
#define ICM42688_FIFO_COUNTH	0x2E
#define ICM42688_FIFO_COUNTL	0x2F
#define ICM42688_FIFO_DATA		0x30
#define ICM42688_APEX_DATA0		0x31
#define ICM42688_APEX_DATA1		0x32
#define ICM42688_APEX_DATA2		0x33
#define ICM42688_APEX_DATA3		0x34
#define ICM42688_APEX_DATA4		0x35
#define ICM42688_APEX_DATA5		0x36
#define ICM42688_INT_STATUS2	0x37
#define ICM42688_INT_STATUS3	0x38
#define ICM42688_SINGLE_PATH_RESET	0x4B
#define ICM42688_INTF_CONFIG0	0x4C
#define ICM42688_INTF_CONFIG1	0x4D
typedef enum
{
	ACCEL_WakeUp_OSCEL,
	ACCEL_RC_OSCEL,
	NO_RTC_CLK = 0,
	REQ_RTC_CLK,
	ALWS_RC_OSCEL = 0,
	PLL_ELSE_RC,
	DISABLE_ALL_CLK = 3,

	/*Location*/
	CLKSEL = 0,
	RTC_MODE = 2,
	ACCEL_LP_CLK_SEL = 3,

}INTF_CONFIG1;

#define ICM42688_PWR_MGMT0		0x4E
typedef enum
{
	/*Function*/
	ACCEL_OFF =1,
	ACCEL_LP,
	ACCEL_LN,
	GYRO_OFF = 0,
	GYRO_STANDBY,
	GYRO_LN = 3,
	RC_OSC_OFF = 0,
	RC_OSC_ON,
	TEMP_ON = 0,
	TEMP_OFF,

	/*Location*/
	ACCEL_MODE = 0,
	GYRO_MODE = 2,
	IDLE = 4,
	TEMP_DIS,

	RESET_POWER = (ACCEL_OFF<<ACCEL_MODE | GYRO_OFF<<GYRO_MODE | RC_OSC_ON<<IDLE),
	POWER_ON = (ACCEL_LN<<ACCEL_MODE | GYRO_LN<<GYRO_MODE | RC_OSC_ON<<IDLE),

}PWR_MGMT; //When turning the Gyro and Accel on, it needs to waite 45ms, and can't write register for 200us

#define ICM42688_GYRO_Config0 	0x4F
typedef enum
{
	DPS2000 = 0, //default
	DPS1000,
	DPS500,
	DPS250,
	DPS125,
	DPS62_5,
	DPS31_25,
	DPS15_625,

	GYRO_FS_SEL = 5, //location

}GYRO_SENSITIVITY; // <<5 then "|" with Gyro_ODR

/*Gyro ODR*/
typedef enum
{
	GYRO_32KHZ = 1,
	GYRO_16KHZ,
	GYRO_8KHZ,
	GYRO_4KHZ,
	GYRO_2KHZ,
	GYRO_1KHZ, //default
	GYRO_200HZ,
	GYRO_100HZ,
	GYRO_50HZ,
	GYRO_25HZ,
	GYRO_12_5HZ,
	GYRO_500HZ = 15,

	GYRO_ODR = 0, //location

}GYRO_SPEED;// <<0 (no need shift) then "|" with Gyro_FS_SEL

#define ICM42688_ACCEL_Config0	0x50
typedef enum
{
	G16, //default
	G8,
	G4,
	G2,

	ACCEL_FS_SEL = 5, //location
}ACCEL_SENSITIVITY;// <<5 then "|" with ACCEL_ODR

/*Accel ODR*/
typedef enum
{
	ACC_32KHZ = 1,
	ACC_16KHZ,
	ACC_8KHZ,
	ACC_4KHZ,
	ACC_2KHZ,
	ACC_1KHZ, //default
	ACC_200HZ,
	ACC_100HZ,
	ACC_50HZ,
	ACC_25HZ,
	ACC_12_5HZ,
	ACC_6_25HZ,
	ACC_3_125HZ,
	ACC_1_5625HZ,
	ACC_500HZ,

	ACCEL_ODR = 0, //location

}ACCEL_SPEED;// <<0 (no need shift) then "|" with ACCEL_FS_SEL

#define ICM42688_GYRO_CONFIG1	0x51
#define ICM42688_GYRO_ACCEL_Config0	0x52
typedef enum
{
	MODE1,
	MODE2,
	MODE3,
	MODE4,
	MODE5,
	MODE6,
	MODE7,
	MODE8,
	LOW_LATENCY1 = 14, //max(400hz,ODR)
	LOW_LATENCY2 = 15, //max(200hz,8*ODR)
	/*Location*/
	GYRO_UI_FILT_BW = 0,
	ACCEL_UI_FILT_BW = 4,
}ACCEL_GYRO_UI_FILTER; //Specific Mode see 14.40

#define ICM42688_ACCEL_CONFIG1	0x53
#define ICM42688_TMST_CONFIG 	0x54
#define ICM42688_APEX_CONFIG0 	0x56
#define ICM42688_SMD_CONFIG 	0x57
#define ICM42688_FIFO_CONFIG1	0x5F
#define ICM42688_FIFO_CONFIG2	0x60
#define ICM42688_FIFO_CONFIG3	0x61
#define ICM42688_FSYNC_CONFIG 	0x62

#define ICM42688P_INT_CONFIG0	0x63
typedef enum
{
	CLEAR_ON_STATUS_READ = 1,
	CLEAR_ON_SENSOR_READ,
	CLEAR_ON_STAT_AND_SEN_READ,

	CLEAR_ON_FIFO_1B_READ = 2,
	CLEAR_ON_STAT_AND_FIFO_1B_READ,

	/*Location*/
	FIFO_FULL_INT_CLR = 0,
	FIFO_THS_INT_CLR = 2,
	UI_DRDY_INT_CLR = 4,

	CLEAR_WITH_SENSOR_READ = (CLEAR_ON_SENSOR_READ<<UI_DRDY_INT_CLR)
}INT_CONFIG0;

#define ICM42688P_INT_CONFIG1 	0x64
typedef enum
{
	INT_RESET, //change to 0 for proper INT operation
	DE_ASSERTION_ON = 0,
	DE_ASSERTION_OFF, //required if odr>=4khx
	PULSE_100us = 0, //for odr <4khz
	PULSE_8us,

	/*Location*/
	INT_ASYNC_RESET = 4,
	INT_TDEASSERT_DISABLE,
	INT_TPULSE_DURATION,

	HIGH_ODR_SETTING = (PULSE_8us<<INT_TPULSE_DURATION | DE_ASSERTION_OFF << INT_TDEASSERT_DISABLE | INT_RESET << INT_ASYNC_RESET),
}INT_CONFIG1;

#define ICM42688P_INT_SOURCE0 	0x65
typedef enum
{
	DISCONNECT_WITH_INT1,
	CONNECT_WITH_INT1,

	/*Location*/
	UI_AGC_RDY = 0,
	FIFO_FULL,
	FIFO_THS,
	UI_DRDY,
	RESET_DONE,
	PLL_RDY,
	UI_FSYNC,
}INT_SOURCE0;

#define ICM42688P_INT_SOURCE1 	0x66
#define ICM42688P_INT_SOURCE3 	0x68
#define ICM42688P_INT_SOURCE4 	0x69
#define ICM42688P_FIFO_LOST_PKT0 	0x6C
#define ICM42688P_FIFO_LOST_PKT1 	0x6D
#define ICM42688P_SELF_TEST_CONFIG 	0x70

#define ICM42688_WHO_AM_I 		0x75


/*Bank 1 Device Configuration*/
#define ICM42688_SENSOR_CONFIG0 0x03
#define ICM42688_GYRO_CONFIG_STATIC2 0x0B //AAF filter and Notch filter are automatically enabled
#define ICM42688_GYRO_CONFIG_STATIC3 0x0C //delt
#define ICM42688_GYRO_CONFIG_STATIC4 0x0D //deltsqr 7:0
#define ICM42688_GYRO_CONFIG_STATIC5 0x0E //deltsqr 3:0, bitshift 7:4
#define ICM42688_GYRO_CONFIG_STATIC6 0x0F
#define ICM42688_GYRO_CONFIG_STATIC7 0x10
#define ICM42688_GYRO_CONFIG_STATIC8 0x11
#define ICM42688_GYRO_CONFIG_STATIC9 0x12
#define ICM42688_GYRO_CONFIG_STATIC10 0x13 //gyro notch filter 6:4
#define ICM42688_XG_ST_DATA 	0x5F
#define ICM42688_YG_ST_DATA 	0x60
#define ICM42688_ZG_ST_DATA 	0x61
#define ICM42688_TMSTVAL0 		0x62
#define ICM42688_TMSTVAL1 		0x63
#define ICM42688_TMSTVAL2 		0x64
#define ICM42688_INTF_CONFIG4	0x7A //default is 4-wire spi
#define ICM42688_INTF_CONFIG5	0x7B
#define ICM42688_INTF_CONFIG6	0x7C

/*Bank 2 Device Configuration*/
#define ICM42688_ACCEL_CONFIG_STATIC2 0x03 //AAF default on, delt 6:1
#define ICM42688_ACCEL_CONFIG_STATIC3 0x04 //deltsqr
#define ICM42688_ACCEL_CONFIG_STATIC4 0x05 //deltsqr 3:0, bitshift 7:4
#define ICM42688_XA_ST_DATA 	0x38
#define ICM42688_YA_ST_DATA 	0x3C
#define ICM42688_ZA_ST_DATA 	0x3D

/*Bank 3 Device Configuration*/
#define ICM42688_CLKDIV 0x2A //factory tuned clck div, read only


/*Bank 4 Device Configuration*/
#define ICM42688_APEX_CONFIG1 	0x40
#define ICM42688_APEX_CONFIG2 	0x41
#define ICM42688_APEX_CONFIG3 	0x42
#define ICM42688_APEX_CONFIG4 	0x43
#define ICM42688_APEX_CONFIG5 	0x44
#define ICM42688_APEX_CONFIG6 	0x45
#define ICM42688_APEX_CONFIG7 	0x46
#define ICM42688_APEX_CONFIG8 	0x47
#define ICM42688_APEX_CONFIG9 	0x48
#define ICM42688_ACCEL_WOM_X_THR 	0x4A
#define ICM42688_ACCEL_WOM_Y_THR 	0x4B
#define ICM42688_ACCEL_WOM_Z_THR 	0x4C
#define ICM42688P_INT_SOURCE6 	0x4D
#define ICM42688P_INT_SOURCE7 	0x4E
#define ICM42688P_INT_SOURCE8 	0x4F
#define ICM42688P_INT_SOURCE9 	0x50
#define ICM42688P_INT_SOURCE10 	0x51

//Gyro max +-64dps, resolution 1/32dps, ACCEL max +-1g, resolution 0.5mg see 18.18
#define ICM42688P_OFFSET_USER0 	0x77
#define ICM42688P_OFFSET_USER1 	0x78
#define ICM42688P_OFFSET_USER2 	0x79
#define ICM42688P_OFFSET_USER3 	0x7A
#define ICM42688P_OFFSET_USER4 	0x7B
#define ICM42688P_OFFSET_USER5 	0x7C
#define ICM42688P_OFFSET_USER6 	0x7D
#define ICM42688P_OFFSET_USER7 	0x7E
#define ICM42688P_OFFSET_USER8 	0x7F




typedef struct
{
    float x; //filtered data
    float y;
    float z;
    float AngleX;
    float AngleY;
    float AngleZ;
    float Magnitude;
    float prevMagnitude;
    float accDelta;

    float RawX;
    float RawY;
    float RawZ;
    float RawX_Trim;
    float RawY_Trim;
    float RawZ_Trim;

    float PT2_rawX;
    float PT2_rawY;
    float PT2_rawZ;

    float CompleX;
    float CompleY;
    float CompleZ;

    float FinalX;
    float FinalY;


    /*Acceleration Projection*/
    float CosPitch;
    float SinPitch;
    float CosRoll;
    float SinRoll;
    float TanPitch;
    float TanRoll;
    float rawAccx;
    float rawAccy;
    float rawAccz;

    float AccZ;

    /*Integrated ACC Projection -> Velocity*/
    float Vx;
    float Vy;
    float Vz;
} ICM42688_AccData_t;

typedef struct
{
    float x;
    float y;
    float z;

    float RawX, RawY, RawZ;
    float prevRawX, prevRawY, prevRawZ;
    float RawX_Trim;
    float RawY_Trim;
    float RawZ_Trim;

    float EulerX;
    float EulerY;
    float EulerZ;

    float AngleX;
    float AngleY;
    float AngleZ;

    float PT2_X;
    float PT2_Y;
    float PT2_Z;

    uint16_t Current_Tick;
    uint16_t Prev_Tick;
    float dt;
} ICM42688_GyroData_t;

//General Purpose Function
void ICM42688_SPI_CS_LOW(void);
void ICM42688_SPI_CS_HIGH(void);
uint8_t ICM42688_SPI_Transfer(uint8_t data);
void ICM42688_SPI_Read(uint8_t reg_addr, uint8_t *data, uint16_t len);
void ICM42688_SPI_Write(uint8_t reg_addr, uint8_t data);


//ICM Register Setting
int Set_ACCEL_Para (int ACCEL_SPEED, int ACCEL_Sen, float *ACCEL_Scale_Factor);
int Set_GYRO_Para (int GYRO_ODR, int GYRO_Sen, float *GYRO_Scale_Factor);
void ICM42688_Init(void);

//Status Flag
uint8_t ICM42688_WHO_AM_I_Test(void);
uint8_t Data_Ready(void);

//SPI_DM_Read
void ICM42688_SPI1_DMA__READ(uint8_t *IMU_Buffer);
//Read both at the same time to reduce time
void ICM42688_ReadAccelGyroData(ICM42688_AccData_t *accData,ICM42688_GyroData_t *gyroData);
void ACCEL_GYRO_Angle_Convert(ICM42688_AccData_t *accData, ICM42688_GyroData_t *gyroData, uint8_t *IMU_SPI_Buffer);
//Acceleration
void ICM42688_ReadAccData(ICM42688_AccData_t *accData);
void Accel_Angle_Convert(ICM42688_AccData_t *accData);

//Angular Velocity
void ICM42688_ReadGyroData(ICM42688_GyroData_t *gyroData);
void Gyro_Angle_Convert(ICM42688_GyroData_t *gyroData);

void Complementary_Angle_Apply(ICM42688_GyroData_t *gyroData, ICM42688_AccData_t *accData);


//Get Velocity from ACC integration
void ICM42688_GetVelocity(ICM42688_AccData_t *accData);

void fixRate_to_eulerRated (ICM42688_GyroData_t *gyroData, ICM42688_AccData_t *accData);
#endif /* __ICM42688P_H */
