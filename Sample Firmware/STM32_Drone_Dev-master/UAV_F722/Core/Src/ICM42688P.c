#include "ICM42688P.h"
#include "sensors/gyro.h"
#include "sensors/SensorsCalibration.h"
#include "CommonMath/filters.h"
#include "Timers.h"
#include "math.h"
#include "spi.h"
#include "Timers.h"
#include "CommonMath/maths.h"
#include "flight/imu.h"

/*User Define Filter*/
#define USE_PT1 50 //100hz cutoff


#define ICM42688_STATUS 0x2A
#define ICM426XX_PWR_MGMT0_GYRO_ACCEL_MODE_OFF      ((0 << 0) | (0 << 2))
#define ICM42688_WHO_AM_I_RESULT 0x47
#define SPI_TIMEOUT 50 //1000



/*Scale Factor*/
static float ACCEL_Scale_Factor;
static float GYRO_Scale_Factor;


//Filter Definition
//Complementary Filter
compleFilter_t Comple_Angle_X;
compleFilter_t Comple_Angle_Y;

//PT2 Filter
pt2Filter_t Accel_X_PT2;
pt2Filter_t Accel_Y_PT2;
pt2Filter_t Accel_Z_PT2;

pt2Filter_t Gyro_X_PT2;
pt2Filter_t Gyro_Y_PT2;
pt2Filter_t Gyro_Z_PT2;

//PT Filter for Z velocity
pt1Filter_t VeloZ_PT1;

#ifdef USE_PT1
pt1Filter_t AngleX_PT1;
pt1Filter_t AngleY_PT1;
#endif

typedef enum {
    AAF_CONFIG_258HZ = 0,
    AAF_CONFIG_536HZ,
    AAF_CONFIG_997HZ,
    AAF_CONFIG_1962HZ,
    AAF_CONFIG_COUNT
} aafConfig_e;

//Anti-Alias Filters
typedef struct aafConfig_s {
    uint8_t delt;
    uint16_t deltSqr;
    uint8_t bitshift;
} aafConfig_t;



// Possible gyro Anti-Alias Filter (AAF) cutoffs for ICM-42688P
static aafConfig_t aafLUT42688[AAF_CONFIG_COUNT] = {  // see table in section 5.3
    [AAF_CONFIG_258HZ]  = {  6,   36, 10 },
    [AAF_CONFIG_536HZ]  = { 12,  144,  8 },
    [AAF_CONFIG_997HZ]  = { 21,  440,  6 },
    [AAF_CONFIG_1962HZ] = { 37, 1376,  4 },
};


/*General Purpose*/
void ICM42688_SPI_CS_LOW(void)
{
    HAL_GPIO_WritePin(ICM42688_CS_GPIO, ICM42688_CS_PIN, GPIO_PIN_RESET);
}

void ICM42688_SPI_CS_HIGH(void)
{
    HAL_GPIO_WritePin(ICM42688_CS_GPIO, ICM42688_CS_PIN, GPIO_PIN_SET);
}

uint8_t ICM42688_SPI_Transfer(uint8_t data)
{
    uint8_t rx_data;
    HAL_SPI_TransmitReceive(&ICM42688_SPI_HANDLE, &data, &rx_data, 1, SPI_TIMEOUT);
    return rx_data;
}

void ICM42688_SPI_Read(uint8_t reg_addr, uint8_t *data, uint16_t len)
{
    ICM42688_SPI_CS_LOW();
    ICM42688_SPI_Transfer(reg_addr | 0x80);
    for (uint16_t i = 0; i < len; i++)
    {
        data[i] = ICM42688_SPI_Transfer(0);
    }
    ICM42688_SPI_CS_HIGH();
}

void ICM42688_SPI_Write(uint8_t reg_addr, uint8_t data)
{
    ICM42688_SPI_CS_LOW();
    ICM42688_SPI_Transfer(reg_addr);
    ICM42688_SPI_Transfer(data);
    ICM42688_SPI_CS_HIGH();
}

void ICM42688_SPI1_DMA__READ(uint8_t *IMU_Buffer)
{
//	 uint8_t txBuffer = ICM42688_ACCEL_DATA_X1 | 0x80;
//    ICM42688_SPI_CS_LOW();
//    HAL_SPI_Transmit(&ICM42688_SPI_HANDLE, &txBuffer, 1, SPI_TIMEOUT);
//	HAL_SPI_Receive_DMA(&ICM42688_SPI_HANDLE, IMU_Buffer, 12);
}

uint8_t ICM42688_WHO_AM_I_Test(void)
{
    uint8_t who_am_i;
    ICM42688_SPI_Read(ICM42688_WHO_AM_I, &who_am_i, 1);
    return who_am_i == ICM42688_WHO_AM_I_RESULT;
}

uint8_t Data_Ready(void)
{
    uint8_t Data[8];
    ICM42688_SPI_Read(0x2D, Data, 1);
    return Data[3] == 1;
}





/*ACCEL and GYRO Config Setting*/
int Set_ACCEL_Para (int ACCEL_SPEED, int ACCEL_Sen, float *ACCEL_Scale_Factor)
{
	int ACCEL_Config0_Para = ((ACCEL_Sen << ACCEL_FS_SEL)| (ACCEL_SPEED<<ACCEL_ODR));

	switch(ACCEL_Sen)
	{
		case G16:
			*ACCEL_Scale_Factor = 2048.f;
			break;
		case G8:
			*ACCEL_Scale_Factor = 4096.f;
			break;
		case G4:
			*ACCEL_Scale_Factor = 8192.f;
			break;
		case G2:
			*ACCEL_Scale_Factor = 16384.f;
			break;
	}

	return ACCEL_Config0_Para;
}

int Set_GYRO_Para (int GYRO_ODR, int GYRO_Sen, float *GYRO_Scale_Factor)
{
	int GYRO_Config0_Para = ((GYRO_Sen << ACCEL_FS_SEL)| (GYRO_ODR << ACCEL_ODR));
	switch(GYRO_Sen)
	{
		case DPS2000:
			*GYRO_Scale_Factor = 16.4f;
			break;
		case DPS1000:
			*GYRO_Scale_Factor = 32.8f;
			break;
		case DPS500:
			*GYRO_Scale_Factor = 65.5f;
			break;
		case DPS250:
			*GYRO_Scale_Factor = 131.f;
			break;
		case DPS125:
			*GYRO_Scale_Factor = 262.f;
			break;
		case DPS62_5:
			*GYRO_Scale_Factor = 524.3f;
			break;
		case DPS31_25:
			*GYRO_Scale_Factor = 1048.6f;
			break;
		case DPS15_625:
			*GYRO_Scale_Factor = 2097.2f;
			break;
	}
	return GYRO_Config0_Para;
}

int check = 0;
int check1 = 0;
int check2 = 0;

/*Initialization*/
void ICM42688_Init(void)
{
	/*Check Dev ID*/
	if(ICM42688_WHO_AM_I_Test())
	{	check = 1;
		for(int x =0; x<5; x++)
		{	HAL_GPIO_TogglePin(imuLEDport, imuLEDpin);
			Delay_ms(200);
		}
	}


	/*Bank 0 Configuration - Device Reset*/
	ICM42688_SPI_Write(ICM426XX_RA_REG_BANK_SEL, ICM426XX_BANK_SELECT0);
    Delay_ms(10);
    ICM42688_SPI_Write(ICM42688_PWR_MGMT0, RESET_POWER); //power reset so registers can be modified
    Delay_ms(10);
    ICM42688_SPI_Write(ICM426XX_DEV_CONFIG, (SOFT_RESET)); //Select SPI mode and Reset
    Delay_ms(50);

    /*Bank 1 Configuration - GYRO AntiAliasFilter Configuration*/
	ICM42688_SPI_Write(ICM426XX_RA_REG_BANK_SEL, ICM426XX_BANK_SELECT1);
    Delay_ms(10);
    aafConfig_t aafConfig = aafLUT42688[AAF_CONFIG_258HZ];
    ICM42688_SPI_Write(ICM42688_GYRO_CONFIG_STATIC3, aafConfig.delt);
    ICM42688_SPI_Write(ICM42688_GYRO_CONFIG_STATIC4, aafConfig.deltSqr & 0xFF);
    ICM42688_SPI_Write(ICM42688_GYRO_CONFIG_STATIC5, (aafConfig.deltSqr >> 8) | (aafConfig.bitshift << 4));
    Delay_ms(50);

    /*Bank 2 Configuration - ACCEL AntiAliasFilter Configuration*/
    aafConfig_t aafConfig2 = aafLUT42688[AAF_CONFIG_536HZ];//for 8khz
    ICM42688_SPI_Write(ICM426XX_RA_REG_BANK_SEL, ICM426XX_BANK_SELECT2);
    Delay_ms(10);
    ICM42688_SPI_Write(ICM42688_ACCEL_CONFIG_STATIC2, aafConfig2.delt << 1);
    ICM42688_SPI_Write(ICM42688_ACCEL_CONFIG_STATIC3, aafConfig2.deltSqr & 0xFF);
    ICM42688_SPI_Write(ICM42688_ACCEL_CONFIG_STATIC4, (aafConfig2.deltSqr >> 8) | (aafConfig2.bitshift << 4));
    Delay_ms(50);

    check1 = 1;


    /*Bank 0 Configuration*/
    // Configure accelerometer UI Filter
    ICM42688_SPI_Write(ICM426XX_RA_REG_BANK_SEL, ICM426XX_BANK_SELECT0);
    ICM42688_SPI_Write(ICM42688_GYRO_ACCEL_Config0, ((LOW_LATENCY2<<ACCEL_UI_FILT_BW) | (LOW_LATENCY2<<GYRO_UI_FILT_BW)));
    Delay_ms(50);

    //Ignore this part
//    uint8_t intfConfig1Value = 0;
//    intfConfig1Value &= ~(0xC0);
//    intfConfig1Value |= 0x40;
//    ICM42688_SPI_Write(0x4D, intfConfig1Value);


    //Interrupt Config
    ICM42688_SPI_Write(ICM42688P_INT_CONFIG, (Pulsed_Mode<<INT1_Mode)|(Push_Pull<<INT1_Drive_Circuit)|(Active_High<<INT1_Polarity));// INT1 Active High, Push Pull, Pulse Mode
    ICM42688_SPI_Write(ICM42688P_INT_CONFIG0, CLEAR_WITH_SENSOR_READ);//Clear on Sensor Register Read (20)
    ICM42688_SPI_Write(ICM42688P_INT_CONFIG1, HIGH_ODR_SETTING);// 8 µs pulse, disable de-assert duration, proper INT1/INT2 operation
    ICM42688_SPI_Write(ICM42688P_INT_SOURCE0, (CONNECT_WITH_INT1<<UI_DRDY));//Enable data ready and route it to INT1

    Delay_ms(100);
    // Configure gyroscope
    ICM42688_SPI_Write(ICM42688_GYRO_Config0, Set_GYRO_Para (GYRO_2KHZ,DPS2000, &GYRO_Scale_Factor));//DPS2000
    Delay_ms(100);
    // Configure accelerometer
    ICM42688_SPI_Write(ICM42688_ACCEL_Config0, Set_ACCEL_Para (ACC_2KHZ, G8, &ACCEL_Scale_Factor));
    Delay_ms(100);

    // Configure power management to enable accel and gyro in Low Noise mode
 	ICM42688_SPI_Write(ICM42688_PWR_MGMT0, POWER_ON);
 	Delay_ms(10);


	/*System Filter Init*/

    //Acc Filter
	float Accel_X_PT2_k = pt2FilterGain(20.0f, (1.0f/IMU_Freq)); //Init PT1 filter for x
	pt2FilterInit(&Accel_X_PT2, Accel_X_PT2_k);
	float Accel_Y_PT2_k = pt2FilterGain(20.0f, (1.0f/IMU_Freq)); //Init PT1 filter for y
	pt2FilterInit(&Accel_Y_PT2, Accel_Y_PT2_k);
	float Accel_Z_PT2_k = pt2FilterGain(20.0f, (1.0f/IMU_Freq)); //Init PT1 filter for y
	pt2FilterInit(&Accel_Z_PT2, Accel_Z_PT2_k);

	//Gyro Filter
	float Gyro_X_PT2_k = pt2FilterGain(13.0f, (1.0f/IMU_Freq)); //Init PT1 filter for x
	pt2FilterInit(&Gyro_X_PT2, Gyro_X_PT2_k);
	float Gyro_Y_PT2_k = pt2FilterGain(13.0f, (1.0f/IMU_Freq)); //Init PT1 filter for y
	pt2FilterInit(&Gyro_Y_PT2, Gyro_Y_PT2_k);
	float Gyro_Z_PT2_k = pt2FilterGain(13.0f, (1.0f/IMU_Freq)); //Init PT1 filter for y
	pt2FilterInit(&Gyro_Z_PT2, Gyro_Z_PT2_k);

	//Velocity Filter
	float VeloZ_PT1_k = pt1FilterGain(50.0f, (1.0f/IMU_Freq));
	pt1FilterInit(&VeloZ_PT1, VeloZ_PT1_k);

	//Complementary Filter
	ComplementaryFilterInit(&Comple_Angle_X, 0.98f);
	ComplementaryFilterInit(&Comple_Angle_Y, 0.98f);

	/*User Filter Init*/
#ifdef USE_PT1
		//Complementary Filter's Filter
	float AngleX_PT1_k = pt1FilterGain(150.0f, (1.0f/IMU_Freq)); //Init PT1 filter for x
	pt1FilterInit(&AngleX_PT1, AngleX_PT1_k);
	float AngleY_PT1_k = pt1FilterGain(150.0f, (1.0f/IMU_Freq)); //Init PT1 filter for y
	pt1FilterInit(&AngleY_PT1, AngleY_PT1_k);
#endif

	Delay_ms(50);



}


/* SPI Normal Mode : ACCEL and Gyro Data Processing*/
void ICM42688_ReadAccelGyroData(ICM42688_AccData_t *accData,ICM42688_GyroData_t *gyroData)
{
    uint8_t data[12];
    ICM42688_SPI_Read(ICM42688_ACCEL_DATA_X1, data, 12);
    gyroData->dt = 1.0f/IMU_Freq;

	/*Accel Data*/
    accData->RawX = (int16_t)((data[0] << 8) | data[1]);
    accData->RawY = (int16_t)((data[2] << 8) | data[3]);
    accData->RawZ = (int16_t)((data[4] << 8) | data[5]);

    /*Gyro Data*/
    gyroData->RawX = (int16_t)((data[6] << 8) | data[7]);
    gyroData->RawY = (int16_t)((data[8] << 8) | data[9]);
    gyroData->RawZ = (int16_t)((data[10] << 8) | data[11]);
    gyroSlewLimiter(gyroData);
}


void ICM42688_ReadAccData(ICM42688_AccData_t *accData) /*Read Accel Data Independently*/
{
    uint8_t data[6];
    ICM42688_SPI_Read(ICM42688_ACCEL_DATA_X1, data, 6);

    accData->RawX = (int16_t)((data[0] << 8) | data[1]);
    accData->RawY = (int16_t)((data[2] << 8) | data[3]);
    accData->RawZ = (int16_t)((data[4] << 8) | data[5]);
	accData->x = (float)(accData->RawX - accData->RawX_Trim) / ACCEL_Scale_Factor;
	accData->y = (float)(accData->RawY - accData->RawY_Trim) / ACCEL_Scale_Factor;
	accData->z = (float)(accData->RawZ - accData->RawZ_Trim) / ACCEL_Scale_Factor;
	Accel_Angle_Convert(accData);

}

void ICM42688_ReadGyroData(ICM42688_GyroData_t *gyroData) /*Read Gyro Data Independently*/
{
//	//Before reading the data, dt should be attained
//	gyroData-> Current_Tick= TIM6_GetTick();
//
//	gyroData->dt =((float)GetMicroseconds(gyroData->Current_Tick, gyroData->Prev_Tick))/1000000.0; //calculate dt in us to s
//	gyroData->Prev_Tick = gyroData->Current_Tick;//Assign current tick as previous for next round

    uint8_t data[6];
    ICM42688_SPI_Read(ICM42688_GYRO_DATA_X1, data, 6);

    gyroData->RawX = (int16_t)((data[0] << 8) | data[1]);
    gyroData->RawY = (int16_t)((data[2] << 8) | data[3]);
    gyroData->RawZ = (int16_t)((data[4] << 8) | data[5]);

    gyroData->x = (float)(gyroData->RawX - gyroData->RawX_Trim) / (GYRO_Scale_Factor);
    gyroData->y = (float)(gyroData->RawY - gyroData->RawY_Trim) / (GYRO_Scale_Factor);
    gyroData->z = (float)(gyroData->RawZ - gyroData->RawZ_Trim) / (GYRO_Scale_Factor);

    Gyro_Angle_Convert(gyroData);
}

void Accel_Angle_Convert(ICM42688_AccData_t *accData)
{
	accData->PT2_rawX = (float)(accData->RawX - accData->RawX_Trim) / ACCEL_Scale_Factor;
	accData->PT2_rawY = (float)(accData->RawY - accData->RawY_Trim) / ACCEL_Scale_Factor;
	accData->PT2_rawZ = (float)(accData->RawZ - accData->RawZ_Trim) / ACCEL_Scale_Factor;

	accData->x = pt2FilterApply(&Accel_X_PT2, accData->PT2_rawX);
	accData->y = pt2FilterApply(&Accel_Y_PT2, accData->PT2_rawY);
	accData->z = pt2FilterApply(&Accel_Z_PT2, accData->PT2_rawZ);

	//calculate magnitude
	float accSquaredSum = 0.0f;
	accSquaredSum  = sq(accData->x) + sq(accData->y) + sq(accData->z);
	accData->Magnitude = sqrtf(accSquaredSum);

	accData->accDelta = (accData->Magnitude - accData->prevMagnitude)*(float)IMU_Freq;

	//No need to calculate angle, quaternion handle it
//	accData->AngleX= RADIANS_TO_DEGREES(atan2_approx(accData->y,sqrtf(sq(accData->x)+sq(accData->z))));
//	accData->AngleY= RADIANS_TO_DEGREES(atan2_approx(-accData->x,sqrtf(sq(accData->z)+sq(accData->y))));


}

void Gyro_Angle_Convert(ICM42688_GyroData_t *gyroData)
{

	gyroData->x = (float)(gyroData->RawX - gyroData->RawX_Trim) / (GYRO_Scale_Factor);
	gyroData->y = (float)(gyroData->RawY - gyroData->RawY_Trim) / (GYRO_Scale_Factor);
	gyroData->z = (float)(gyroData->RawZ - gyroData->RawZ_Trim) / (GYRO_Scale_Factor);

	/*PT2 Filter Applied*/
	gyroData->PT2_X = pt2FilterApply(&Gyro_X_PT2, gyroData->x);
	gyroData->PT2_Y = pt2FilterApply(&Gyro_Y_PT2, gyroData->y);
	gyroData->PT2_Z = pt2FilterApply(&Gyro_Z_PT2, gyroData->z);

	/*Gyro_Angle Calculation*/
	gyroData->AngleX += gyroData->PT2_X * (gyroData->dt);
	gyroData->AngleY += gyroData->PT2_Y * (gyroData->dt);
	gyroData->AngleZ += gyroData->PT2_Z * (gyroData->dt);

}


/* SPI DMA : ACCEL and Gyro Data Processing*/
void ACCEL_GYRO_Angle_Convert(ICM42688_AccData_t *accData, ICM42688_GyroData_t *gyroData, uint8_t *IMU_SPI_Buffer)
{
    ICM42688_SPI_CS_HIGH();

	/*Accel Data*/
    accData->RawX = (int16_t)((IMU_SPI_Buffer[0] << 8) | IMU_SPI_Buffer[1]);
    accData->RawY = (int16_t)((IMU_SPI_Buffer[2] << 8) | IMU_SPI_Buffer[3]);
    accData->RawZ = (int16_t)((IMU_SPI_Buffer[4] << 8) | IMU_SPI_Buffer[5]);

    /*Gyro Data*/
    gyroData->RawX = (int16_t)((IMU_SPI_Buffer[6] << 8) | IMU_SPI_Buffer[7]);
    gyroData->RawY = (int16_t)((IMU_SPI_Buffer[8] << 8) | IMU_SPI_Buffer[9]);
    gyroData->RawZ = (int16_t)((IMU_SPI_Buffer[10] << 8) | IMU_SPI_Buffer[11]);

}

void Complementary_Angle_Apply(ICM42688_GyroData_t *gyroData, ICM42688_AccData_t *accData)
{
//	accData->SinPitch = sin(accData->FinalX);
//	accData->CosPitch = cos(accData->FinalX);
//	accData->SinRoll = sin(accData->FinalY);
//	accData->CosRoll = cos(accData->FinalY);
//	accData->TanPitch = tan(accData->FinalX);
//	accData->TanRoll = tan(accData->FinalY);

	//fixRate_to_eulerRated(gyroData,accData);
	accData->CompleX = ComplementaryFilter(&Comple_Angle_X, accData->AngleX, gyroData->PT2_X, gyroData->dt);
	accData->CompleY = ComplementaryFilter(&Comple_Angle_Y, accData->AngleY, gyroData->PT2_Y, gyroData->dt);


#ifdef USE_PT1
    accData->FinalX = pt1FilterApply(&AngleX_PT1, accData->CompleX);
    accData->FinalY = pt1FilterApply(&AngleY_PT1, accData->CompleY);

#endif
}

void fixRate_to_eulerRated (ICM42688_GyroData_t *gyroData, ICM42688_AccData_t *accData)
{
	gyroData->EulerX = gyroData->PT2_X + accData->TanRoll * ((gyroData->PT2_Y * accData->SinPitch) + (gyroData->PT2_Z * accData->CosPitch)); //p' + y' cos(x)tan(y) + r' sin(x)tan(y)
	gyroData->EulerY = gyroData->PT2_Y * accData->CosPitch - gyroData->PT2_Z * accData->SinPitch; //r' cos(x) - y' sin(x)
	gyroData->EulerZ = ((gyroData->PT2_Z * accData->CosPitch)/accData->CosRoll) + ((gyroData->PT2_Y * accData->SinPitch)/accData->CosRoll); // y' cos(x) / cos(y) + r' sin(x)/cos(y)
}


void ICM42688_GetVelocity(ICM42688_AccData_t *accData)
{
	/*For Efficiency, cos and sin should be calculated first.*/
	accData->SinPitch = sin_approx(DECIDEGREES_TO_RADIANS(attitude.values.roll));
	accData->CosPitch = cos_approx(DECIDEGREES_TO_RADIANS(attitude.values.roll));
	accData->TanPitch = tan_approx(DECIDEGREES_TO_RADIANS(attitude.values.roll));

	accData->SinRoll = sin_approx(DECIDEGREES_TO_RADIANS(attitude.values.pitch));
	accData->CosRoll = cos_approx(DECIDEGREES_TO_RADIANS(attitude.values.pitch));
	accData->TanRoll = tan_approx(DECIDEGREES_TO_RADIANS(attitude.values.pitch));


	//Accz,i = -AccX * sin (pitch) + AccY * sin (roll) * cos(pitch) + AccZ * cos(roll) * cos(pitch)
	accData->rawAccz = (-1*accData->x* (accData->CosPitch * accData->SinRoll) + accData->y * accData->SinPitch + accData->z * (accData->CosPitch * accData->CosRoll))-1.0f;

	//Accx = AccX *cos(pitch) + AccY * sin(roll) * sin(pitch) + AccZ cos(roll) *sin (pitch)
	accData->rawAccx = (accData->x * accData->CosPitch) + (accData->y * accData->SinRoll * accData->SinPitch) + (accData->z * accData->CosRoll * accData->SinPitch);

	//Accy = AccY * cos(roll) - AccZ - sin(roll)
	accData->rawAccy = (accData->y * accData->CosRoll) - (accData->z * accData->SinRoll);


	accData->AccZ = pt1FilterApply(&VeloZ_PT1, (accData->rawAccz*Gravity));
}
