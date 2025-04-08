#include <main.h>
#include <i2c.h>
#include <math.h>
#include <MPU6050.h>

Struct_MPU6050 MPU6050;


uint8_t ClockSource_XGyro = 0x01; // Using PLL with X-axis gyroscope as reference
static float LSB_Sensitivity_ACC;
static float LSB_Sensitivity_GYRO;


const FlatLimit = 1500;
double AngleRoll, AnglePitch;
double AngleRoll_sqrt, AnglePitch_sqrt;


uint32_t Timer;


Kalman_t KalmanX = { //Set up the initial conditions for X axis
		.Q_Angle = 0.001f,
		.Q_Bias = 0.003f,
		.R_Measurement = 0.03f,
};


Kalman_t KalmanY = { //Set up the initial conditions for Y axis
		.Q_Angle = 0.001f,
		.Q_Bias = 0.003f,
		.R_Measurement = 0.03f,
};




//MemAddSize: I2C Target memory address
//DevAddress: I2C Target device address
//HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout);

//Creating 4 functions to write to and read the bit of the specific register to manipulating the MPU6050's settings
void MPU6050_WriteByte(uint8_t Reg_Addr, uint8_t BitValue){
	HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, Reg_Addr, I2C_MEMADD_SIZE_8BIT, &BitValue, 1, 1);
}

void MPU6050_WriteBytes(uint8_t Reg_Addr, uint8_t BitLength, uint8_t* Data){
	HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, Reg_Addr, I2C_MEMADD_SIZE_8BIT, Data, BitLength, 1);
}

void MPU6050_ReadByte (uint8_t Reg_Addr, uint8_t* Data){
	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, Reg_Addr, I2C_MEMADD_SIZE_8BIT, Data, 1, 1);
}

void MPU6050_ReadBytes (uint8_t Reg_Addr, uint8_t BitLength, uint8_t* Data){
	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, Reg_Addr, I2C_MEMADD_SIZE_8BIT, Data, BitLength, 1);
}


///////////////////////////
//MPU6050 INITIALIZATION
///////////////////////////
void MPU6050_Initialization(void){
	uint8_t who_am_i = 0;
	MPU6050_ReadByte(MPU6050_WHO_AM_I, &who_am_i); //Verify the identity of the device
	if (who_am_i == MPU6050_I2C_ADDR){ //MPU6050 I2C Address
	}
	else{
		while(1){
		}
	}
	//Reset the whole module before initialization
	MPU6050_WriteByte(MPU6050_PWR_MGMT_1, 0x1 << 7); //Device reset (Error recovery, Clear previous configuration, improve reliability,...)
	HAL_Delay(100);

	//Clock source selection
	MPU6050_WriteByte(MPU6050_PWR_MGMT_1, ClockSource_XGyro);

	//Power Management setting
	/*Default is sleep mode
	 * necessary to wake up MPU6050*/
	MPU6050_WriteByte(MPU6050_PWR_MGMT_1, 0x00);

	//Sample Rate Divider
	//Sample Rate = Gyroscope Output Rate / (1 + SMPRT_DIV)
	//Gyroscope Output Rate = 8kHz. Therefore, DLPF is disabled
	//MPU6050_WriteByte(MPU6050_SMPRT_DIV, 0x00);
	//MPU6050_WriteByte(MPU6050_SMPRT_DIV, 39); //Sample Rate = 200Hz
	MPU6050_WriteByte(MPU6050_SMPRT_DIV, 7); //Sample Rate = 1000Hz

	//FSYNC and DLPF setting
	//DLPF is set to 0
	MPU6050_WriteByte(MPU6050_CONFIG, 0x00);

	//GYRO FULL SCALE SETTING
	//FS_SEL Full Scale Range
	/* 0	+-250 deg/s
	 * 1	+-500 deg/s
	 * 2	+-1000 deg/s
	 * 3	+-2000 deg/s */
	uint8_t FULL_SCALE_GYRO = 0x00;
	MPU6050_WriteByte(MPU6050_GYRO_CONFIG, FULL_SCALE_GYRO << 3);

	//ACCELEROMETER FULL SCALE SETTING.
	//FS_SEL Full scale range
	/*
	0	+-2g
	1	+-4g
	2	+-8g
	3	+-16g	*/
	uint8_t FULL_SCALE_ACC = 0x00;
	MPU6050_WriteByte(MPU6050_ACCEL_CONFIG, FULL_SCALE_ACC << 3);

	MPU6050_Get_LSB_Sensitivity(FULL_SCALE_GYRO, FULL_SCALE_ACC);

	//Interrupt PIN setting
	uint8_t INT_LEVEL = 0x0; //0 - Active High, 1 - Active low
	uint8_t LATCH_INT_EN = 0x0; //0 - INT 50us pulse, 1 - Interrupt clear required
	uint8_t INT_RD_CLEAR = 0x1; //0 - INT flag cleared by reading INT_STATUS, 1 - INT flag cleared by any read operation
	MPU6050_WriteByte(MPU6050_INT_PIN_CFG, (INT_LEVEL << 7) | (LATCH_INT_EN << 5) | (INT_RD_CLEAR << 4));

	//Interrupt enable setting
	uint8_t DATA_RDY_EN = 0x1; //1 - Enable, 0 - Disable
	MPU6050_WriteByte(MPU6050_INT_ENABLE, DATA_RDY_EN);
}


////////////////////////////////////
//GETTING RAW DATA FROM THE SENSOR
////////////////////////////////////
void MPU6050_Get_6_AxisRawData(Struct_MPU6050* mpu6050){
	uint8_t StoreData[14];
	MPU6050_ReadBytes(MPU6050_ACCEL_XOUT_H, 14, StoreData);

	//Read Acc
	mpu6050 -> Acc_X_Raw = (StoreData[0] << 8) | StoreData[1];
	mpu6050 -> Acc_Y_Raw = (StoreData[2] << 8) | StoreData[3];
	mpu6050 -> Acc_Z_Raw = (StoreData[4] << 8) | StoreData[5];

	//Read Temperature
	mpu6050 -> Temperature_Raw = (StoreData[6] << 8) | StoreData[7];

	//Read Gyro
	mpu6050 -> Gyro_X_Raw = (StoreData[8] << 8) | StoreData[9];
	mpu6050 -> Gyro_Y_Raw = (StoreData[10] << 8) | StoreData[11];
	mpu6050 -> Gyro_Z_Raw = (StoreData[12] << 8) | StoreData[13];
}


//////////////////////////////////////////
//CREATING THE MPU6050'S SENSITIVITY LEVEL
//////////////////////////////////////////
void MPU6050_Get_LSB_Sensitivity(uint8_t FULL_SCALE_GYRO, uint8_t FULL_SCALE_ACC){
	switch(FULL_SCALE_GYRO){
	case 0:
		LSB_Sensitivity_GYRO = 131.f;
		break;
	case 1:
		LSB_Sensitivity_GYRO = 65.5f;
		break;
	case 2:
		LSB_Sensitivity_GYRO = 32.8f;
		break;
	case 3:
		LSB_Sensitivity_GYRO = 16.4f;
		break;
	}
	switch(FULL_SCALE_ACC){
	case 0:
		LSB_Sensitivity_ACC = 16384.f;
		break;
	case 1:
		LSB_Sensitivity_ACC = 8192.f;
		break;
	case 2:
		LSB_Sensitivity_ACC = 4096.f;
		break;
	case 3:
		LSB_Sensitivity_ACC = 2048.f;
		break;
	}
}


int MPU6050_DataReady(void){
	return HAL_GPIO_ReadPin (MPU6050_INT_PORT, MPU6050_INT_PIN);
}

void LED_Indicate_Flatness_Check(){
	  HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_7); //PG7
	  HAL_Delay(250);
}


/////////////////////
//CALIBRATE MPU6050
/////////////////////
//TAKING THE AVERAGE AND THEN CALCULATING THE OFFSETS BASED ON THE AVERAGE

//Default mode: Flatness check before the calibration
//	It will fail to calibrate if you put the IMU not flat enough. Then, the LED on STM32H7 will flash which indicates that
//	the calibration process is failed. Then, quickly adjust your IMU on a flat surface. The calibration process will automatically
//	re-calibrate.

//Easy mode: No flatness check.
//	It is always calibrated even if you place the IMU randomly.

void MPU6050_Calibration(Struct_MPU6050* mpu6050, int FlatEnableBit){ //Taking the average
	long Buffer_AccX, Buffer_AccY, Buffer_AccZ;
	long Buffer_GyroX, Buffer_GyroY, Buffer_GyroZ;
	int i, LoopFlag;

	do {
		Buffer_AccX = 0; Buffer_AccY = 0; Buffer_AccZ = 0;
		Buffer_GyroX = 0; Buffer_GyroY = 0; Buffer_GyroZ = 0;
		i = 0;

		while (i < (BUFFER_SIZE + 101)){
			MPU6050_Get_6_AxisRawData(mpu6050);
			if (i > 100 && i <= (BUFFER_SIZE + 100)){ //Discard 100 first measures to ensure the IMU entered the stable state
				Buffer_AccX += mpu6050 -> Acc_X_Raw;
				Buffer_AccY += mpu6050 -> Acc_Y_Raw;
				Buffer_AccZ += mpu6050 -> Acc_Z_Raw;

				Buffer_GyroX += mpu6050 -> Gyro_X_Raw;
				Buffer_GyroY += mpu6050 -> Gyro_Y_Raw;
				Buffer_GyroZ += mpu6050 -> Gyro_Z_Raw;
			}


			if (i == (BUFFER_SIZE + 100)){ //Taking the average
				mpu6050 -> Average_Acc_X = Buffer_AccX / BUFFER_SIZE;
				mpu6050 -> Average_Acc_Y = Buffer_AccY / BUFFER_SIZE;
				mpu6050 -> Average_Acc_Z = Buffer_AccZ / BUFFER_SIZE;

				mpu6050 -> Average_Gyro_X = Buffer_GyroX / BUFFER_SIZE;
				mpu6050 -> Average_Gyro_Y = Buffer_GyroY / BUFFER_SIZE;
				mpu6050 -> Average_Gyro_Z = Buffer_GyroZ / BUFFER_SIZE;

				if (FlatEnableBit == 1){ //Enable the flatness check
					if ((mpu6050 -> Average_Acc_X < FlatLimit && mpu6050 -> Average_Acc_X > -FlatLimit) &&
						(mpu6050 -> Average_Acc_Y < FlatLimit && mpu6050 -> Average_Acc_Y > -FlatLimit)){
							//Calculate offset
							mpu6050 -> Acc_X_Offset =- mpu6050 -> Average_Acc_X;
							mpu6050 -> Acc_Y_Offset =- mpu6050 -> Average_Acc_Y;
							mpu6050 -> Acc_Z_Offset = GRAVITY_RAW_UNIT - (mpu6050 -> Average_Acc_Z); //Assuming vertical alignment

							mpu6050 -> Gyro_X_Offset =- mpu6050 -> Average_Gyro_X;
							mpu6050 -> Gyro_Y_Offset =- mpu6050 -> Average_Gyro_Y;
							mpu6050 -> Gyro_Z_Offset =- mpu6050 -> Average_Gyro_Z;
							LoopFlag = 0; //Calibration succeed, exit the loop
						} else{
							int LED_Count = 0;
									while (LED_Count < 21){ //LED turns on and off 10 times
										LED_Indicate_Flatness_Check();
										LED_Count++;
									}
							LoopFlag = 1; //Calibration failed, calibrate again
						}
				}


				if (FlatEnableBit == 2){ //Disable flatness check
					//Calculate offset
					mpu6050 -> Acc_X_Offset =- mpu6050 -> Average_Acc_X;
					mpu6050 -> Acc_Y_Offset =- mpu6050 -> Average_Acc_Y;
					mpu6050 -> Acc_Z_Offset = GRAVITY_RAW_UNIT - (mpu6050 -> Average_Acc_Z); //Assuming vertical alignment

					mpu6050 -> Gyro_X_Offset =- mpu6050 -> Average_Gyro_X;
					mpu6050 -> Gyro_Y_Offset =- mpu6050 -> Average_Gyro_Y;
					mpu6050 -> Gyro_Z_Offset =- mpu6050 -> Average_Gyro_Z;
					LoopFlag = 0; //Calibration succeed, exit the loop
				}
			}
			i++;
			HAL_Delay(2); //Delay to avoid repeated measures
		}
	} while (LoopFlag != 0); //Continue looping if calibration failed.
}



//////////////////////////////////////////////////////////
//UPDATING THE GYRO AND ACC VALUES (put this in the loop)
//////////////////////////////////////////////////////////
void MPU6050_Real_Values(Struct_MPU6050* mpu6050){//Update the Gyro and Acc after calibrating
	mpu6050 -> Acc_X_Raw += mpu6050 -> Acc_X_Offset;
	mpu6050 -> Acc_Y_Raw += mpu6050 -> Acc_Y_Offset;
	mpu6050 -> Acc_Z_Raw += mpu6050 -> Acc_Z_Offset;

	mpu6050 -> Gyro_X_Raw += mpu6050 -> Gyro_X_Offset;
	mpu6050 -> Gyro_Y_Raw += mpu6050 -> Gyro_X_Offset;
	mpu6050 -> Gyro_Z_Raw += mpu6050 -> Gyro_X_Offset;

	//Updating and converting the data to SI unit (deg/s for Gyro and m/s^2 for Acc)
	mpu6050 -> Acc_X_RealValue = mpu6050 -> Acc_X_Raw / LSB_Sensitivity_ACC; //(m/s^2)
	mpu6050 -> Acc_Y_RealValue = mpu6050 -> Acc_Y_Raw / LSB_Sensitivity_ACC;
	mpu6050 -> Acc_Z_RealValue = mpu6050 -> Acc_Z_Raw / LSB_Sensitivity_ACC;

	mpu6050 -> Temperature = (float)(mpu6050 -> Temperature_Raw)/340 + 36.53;

	mpu6050 -> Gyro_X_RealValue = mpu6050 -> Gyro_X_Raw / LSB_Sensitivity_GYRO; //(deg/s)
	mpu6050 -> Gyro_Y_RealValue = mpu6050 -> Gyro_Y_Raw / LSB_Sensitivity_GYRO;
	mpu6050 -> Gyro_Z_RealValue = mpu6050 -> Gyro_Z_Raw / LSB_Sensitivity_GYRO;
}


//////////////////////////////
//ANGLE CONVERSATION (DEGREE)
//////////////////////////////
void MPU6050_AngleConvert(Struct_MPU6050* mpu6050){
	//For AngleRoll
	AngleRoll_sqrt = sqrt((mpu6050 -> Acc_X_RealValue) * (mpu6050 -> Acc_X_RealValue) + (mpu6050 -> Acc_Z_RealValue) * (mpu6050 -> Acc_Z_RealValue));

	if (AngleRoll_sqrt != 0.0){
		AngleRoll = atan2((mpu6050 -> Acc_Y_RealValue), AngleRoll_sqrt) *  (180.0/M_PI);
	} else{
		AngleRoll = 0.0;
	}

	//For AnglePitch
	AnglePitch = -atan2(mpu6050 -> Acc_X_RealValue, sqrt(mpu6050 -> Acc_Y_RealValue * mpu6050 -> Acc_Y_RealValue + mpu6050 -> Acc_Z_RealValue * mpu6050 -> Acc_Z_RealValue)) * (180.0/M_PI);

//	AnglePitch_sqrt = sqrt((mpu6050 -> Acc_Y_RealValue) * (mpu6050 -> Acc_Y_RealValue) + (mpu6050 -> Acc_Z_RealValue) * (mpu6050 -> Acc_Z_RealValue));
//
//	if (AnglePitch_sqrt != 0.0){
//		AnglePitch = -atan((mpu6050 -> Acc_X_RealValue) / AnglePitch_sqrt) * 1/(3.142/180);
//	} else{
//		AnglePitch = 0.0;
//	}
}

//THE RAW VALUE IN THE SAMPLE IS = CALIBRATED VALUES IN MY CODE
//GX IN THE SAMPLE CODE = CONVERTED VALUES IN MY CODE

/*KALMAN FILTER
FUSE GYRO AND ACC TO HAVE THE FINAL STABLE ANGLE ROLL AND PITCH
It operates by predicting about the future state of the system and then updating those prediction based on new measurements.

/*
 Process Noise: It reflects the inherent uncertainty in the model that you are using to predict the next state of the system.

 Q_Angle: Process Noise Variance. It quantifies the inherent uncertainties in the model's ability to predict the angle's next
 state based solely on its previous states. It can address the uncertainty in how well the model predicts the change in angle
 without considering the new measurements. A higher "Q_Angle" means the model predictions are considered less reliable, leading
 the filter to weigh new sensor data more heavily.

 Q_Bias: Process Noise Variance for Gyro Bias(lead to drift in angular rate measurement over time). "Q_Bias" quantifies the
 uncertainty in predicting how this bias will evolve between measurements. A higher "Q_Bias" suggests greater expected variability
 in bias drift, prompting the filter to adjust its estimates more aggressively in response to observed changes.

 "Angle" and "Bias" estimates of the actual angle and gyro bias, respectively.
 P[0][0]: Variance of the angle estimate
 P[1][1]: Variance of the bias estimate
 P[0][1] & P[1][0]: The covariance terms that show how much the error in the angle and the bias estimates are correlated.
 */

void MPU6050_KalmanFilter(Struct_MPU6050* mpu6050){

	/*HAL_GetTick: it returns the current system tick count, representing the number of milliseconds that have elapsed since the microcontroller
	started running the current program*/
	double dt = (double) (HAL_GetTick() - Timer) / 1000; //Convert to second
	Timer = HAL_GetTick();
	MPU6050_AngleConvert(mpu6050);


	//Check the instability or sudden jump across the threshold is detected
	//Store the unreliable measurements first
	if ((AnglePitch < -90 && (mpu6050 -> KalmanAngleY) > 90) || (AnglePitch > 90 && mpu6050 -> KalmanAngleY < -90)){ //Gimbal lock
		//Resetting the Kalman filter's estimate to align with the new measurement (AnglePitch).
		//Prevent the filter from producing unsteady outputs when sudden large jumps in angles occur due to mathematical angle calculation
		KalmanY.Angle = AnglePitch;
		mpu6050 -> KalmanAngleY = AnglePitch;
	} else{
		mpu6050 -> KalmanAngleY = KalmanGetAngle(&KalmanY, AnglePitch, mpu6050 -> Gyro_Y_RealValue, dt);
	}


	if (fabs(mpu6050 -> KalmanAngleY) > 90)
		mpu6050 -> Gyro_X_RealValue = -mpu6050 -> Gyro_X_RealValue;
	mpu6050 -> KalmanAngleX = KalmanGetAngle(&KalmanX, AngleRoll, mpu6050 -> Gyro_Y_RealValue, dt);
}



//KALMAN FILTER FUNCTION
double KalmanGetAngle(Kalman_t *Kalman, double NewAngle, double NewRate, double dt){
	//"NewRate": Angle velocity from the gyroscope
	//"NewAngle": New angle measurement
	double AdjustedRate = NewRate - Kalman -> Bias; //Rate correction by adjusting the incoming gyro measurement
	Kalman -> Angle += dt * AdjustedRate; //The current estimate of the angle "Kalman -> Angle" is estimated based on motion model and previous state



	//Adjusting the angle's variance based on its covariance with bias
	//The more uncertain the bias, the more it can influence the angle's uncertainty as time process
	Kalman -> P[0][0] += dt * (dt * Kalman -> P[1][1] - Kalman -> P[0][1] - Kalman -> P[1][0] + Kalman -> Q_Angle);

	//Covariance between the angle and the bias. If the angle and bias are positively correlated, this can reduce the angle's variance.
	Kalman -> P[0][1] -= dt * Kalman -> P[1][1];
	Kalman -> P[1][0] -= dt * Kalman -> P[1][1];

	//The variance of the bias estimate:
	Kalman -> P[1][1] += Kalman -> Q_Bias * dt;



	//Calculating the Kalman gain
	//Kalman gains determine how much the estimates should be corrected based on the new measurements.
	double S = Kalman -> P[0][0] + Kalman -> R_Measurement; //S = combining both the uncertainty of the prediction and the noise of the measurement itself
	double K[2]; //Store the Kalman gains
	K[0] = Kalman -> P[0][0] / S; //Kalman gain for the angle. If S large -> high uncertainty in the new measurement -> Rely more on the pred
	K[1] = Kalman -> P[1][0] / S; //Kalman gain for bias. Determines how much the new measurement influences the bias estimate



	//Updating the phase of the Kalman filter
	//Adjusting the estimate of the state variables based on the discrepancy between the actual measurement and the predicted state.
	double y = NewAngle - Kalman -> Angle; //NewAngle: The lastest update from the sensor
	Kalman -> Angle += K[0] * y;//The adjustment made to the angle estimate based on the measurement residual, scaled by Kalman gain
	Kalman -> Bias += K[1] * y ;//Adjusts the bias estimate based on the same measurement residual



	double P00_Term = Kalman -> P[0][0];
	double P01_Term = Kalman -> P[0][1];



	Kalman -> P[0][0] -= K[0] * P00_Term;
	Kalman -> P[0][1] -= K[0] * P01_Term;
	Kalman -> P[1][0] -= K[1] * P00_Term;
	Kalman -> P[1][1] -= K[1] * P01_Term;

	return Kalman -> Angle;
};


void MPU_Config(void){
	 MPU_Region_InitTypeDef MPU_InitStruct = {0};

	 /* Disables the MPU */
	 HAL_MPU_Disable();

	 /** Initializes and configures the Region and the memory to be protected
	 */
	 MPU_InitStruct.Enable = MPU_REGION_ENABLE;
	 MPU_InitStruct.Number = MPU_REGION_NUMBER0;
	 MPU_InitStruct.BaseAddress = 0x0;
	 MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
	 MPU_InitStruct.SubRegionDisable = 0x87;
	 MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
	 MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
	 MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
	 MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
	 MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
	 MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

	 HAL_MPU_ConfigRegion(&MPU_InitStruct);
	 /* Enables the MPU */
	 HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}
