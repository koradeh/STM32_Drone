#include "main.h"
#include "stm32h7xx_hal.h"

#define MPU6050_ADDR 0xD0 //Write to the device, '0' for writing and '1' for reading
#define MPU6050_I2C_ADDR 0x68 //Default I2C address

#define MPU6050_SMPRT_DIV 0x19 //Sample Rate Divider (8-bit unsigned value) which is used for gyroscope output rate (Sample Rate)
#define MPU6050_CONFIG 0X1A //Configures the external Frame Synchronization (FSYNC) pin sampling, Digital Low Pass Filter (DLPF)
#define MPU6050_GYRO_CONFIG 0X1B //Trigger gyro self-test and configure the gyroscopes' full scale range
#define MPU6050_ACCEL_CONFIG 0x1C //Trigger accel self-test and configure the accel's full range

#define MPU6050_INT_PIN_CFG 0x37 //Bypass Enable Configuration. It configures the behavior of the interrupt signals at the INT pins. Enable FSYNC pin, enable Bypass Mode, enables the clock output
#define MPU6050_INT_ENABLE 0x38 //Interrupt Enable (FIFO_OFLOW_EN, I2C_MST_INT_EN, DATA_RDY_EN)
#define MPU6050_INT_STATUS 0X3A //It shows the interrupt status of each interrupt generation source. Each bit will clear after the register is read.

#define MPU6050_ACCEL_XOUT_H 0x3B //Store the most recent X-axis accelerometer measurements.
#define MPU6050_ACCEL_XOUT_L 0x3C
#define MPU6050_ACCEL_YOUT_H 0x3D //Store the most recent Y-axis ...
#define MPU6050_ACCEL_YOUT_L 0x3E
#define MPU6050_ACCEL_ZOUT_H 0x3F //...Z-axis...
#define MPU6050_ACCEL_ZOUT_L 0x40

#define TEMP_OUT_H 0x41 //Stores the most recent temperature sensor measurement (not surrounding temperature). Considering these values is important for achieving stable Gyro and Accel values.
#define TEMP_OUT_L 0x4

#define MPU6050_PWR_MGMT_1 0x6B //Power Management, it configures the Power Mode. It also provides a bit for resetting the entire device and disabling the temperature sensor.
#define MPU6050_WHO_AM_I 0x75 //This register is used to verify the identity of the device (6-bit)

#define MPU6050_RA_XA_OFFS_H 0x06 //[15:0] XA_OFFS
#define MPU6050_RA_XA_OFFS_L 0x07
#define MPU6050_RA_YA_OFFS_H 0x08 //[15:0] YA_OFFS
#define MPU6050_RA_YA_OFFS_L 0x09
#define MPU6050_RA_ZA_OFFS_H 0x0A //[15:0] ZA_OFFS (offset)
#define MPU6050_RA_ZA_OFFS_L 0x0B
#define GRAVITY_RAW_UNIT 16384 //Raw units, output 16-bit signed integer for acceleration, the most sensitive scale
#define BUFFER_SIZE 2000 //Total number of data which will be taken to calculate the average and the offset, (the higher, the more precision).

#define MPU6050_INT_PORT GPIOB //Interrupt Port B
#define MPU6050_INT_PIN GPIO_PIN_5 //Pin 5 selected



typedef struct _MPU6050{
	//Raw data is collected directly from the sensor
	short Acc_X_Raw;
	short Acc_Y_Raw;
	short Acc_Z_Raw;

	short Temperature_Raw;
	short Gyro_X_Raw;
	short Gyro_Y_Raw;
	short Gyro_Z_Raw;

	//Store the averaged values
	float Average_Acc_X;
	float Average_Acc_Y;
	float Average_Acc_Z;

	float Average_Gyro_X;
	float Average_Gyro_Y;
	float Average_Gyro_Z;

	//Store the offset values of the MPU6050. Offset = tolerance itself
	float Acc_X_Offset;
	float Acc_Y_Offset;
	float Acc_Z_Offset;

	float Gyro_X_Offset;
	float Gyro_Y_Offset;
	float Gyro_Z_Offset;

	 //Store the real value after the calibration
	float Acc_X_RealValue;
	float Acc_Y_RealValue;
	float Acc_Z_RealValue;

	float Gyro_X_RealValue;
	float Gyro_Y_RealValue;
	float Gyro_Z_RealValue;

	float Temperature;

	//KALMAN filter
	double KalmanAngleX;
	double KalmanAngleY;
} Struct_MPU6050;


typedef struct {
	double Q_Angle; //Angle Process Noise (related to Variance)
	double Q_Bias; //Bias Process Noise (related to Variance)
	double R_Measurement; //Measurement of noise Variance

	double Angle; //The angle calculated by the KALMAN filter - part of the 2x1 state
	double Bias; //The gyro bias calculated by the KALMAN filter - part of the 2x1 state
	double P[2][2]; //Error covariance matrix - This is a 2x2 matrix
}Kalman_t;


extern Struct_MPU6050 MPU6050;

extern Kalman_t;

//Function declarations
void MPU6050_WriteByte(uint8_t Reg_Addr, uint8_t BitValue);
void MPU6050_WriteBytes(uint8_t Reg_Addr, uint8_t BitLength, uint8_t* Data);
void MPU6050_ReadByte (uint8_t Reg_Addr, uint8_t* Data);
void MPU6050_ReadBytes (uint8_t Reg_Addr, uint8_t BitLength, uint8_t* Data);

void MPU6050_Initialization(void);
void MPU6050_Get_6_AxisRawData(Struct_MPU6050* mpu6050);
void MPU6050_Get_LSB_Sensitivity(uint8_t FULL_SCALE_GYRO, uint8_t FULL_SCALE_ACC);
int MPU6050_DataReady(void);

//void MPU6050_Initial_Flatness_Check(Struct_MPU6050* mpu6050, int FlatEnableBit);
void LED_Indicate_Flatness_Check();
void MPU6050_Calibration(Struct_MPU6050* mpu6050, int FlatEnableBit);
void MPU6050_Real_Values(Struct_MPU6050* mpu6050);


//Kalman section:
void MPU6050_AngleConvert(Struct_MPU6050* mpu6050);
void MPU6050_KalmanFilter(Struct_MPU6050* mpu6050);
double KalmanGetAngle(Kalman_t *Kalman, double NewAngle, double NewRate, double dt);

void MPU_Config(void);
