/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "flight/Flight_System.h"
#include "ICM42688P.h"
#include "sensors/SensorsCalibration.h"
#include "stdbool.h"
#include "string.h"
#include "PID.h"
#include "sensors/BMP280.h"
#include "flight/defines.h"
#include "sensors/adc.h"
#include "flight/ResourceDefine.h"
#include "flash/Flash_Sector_F4.h"
#include "sensors/opFlow.h"

#include "flight/init.h"
//#include "usbd_cdc_if.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//System
enum System_Mode System;

//IMU
ICM42688_AccData_t ACC;
ICM42688_GyroData_t Gyro;
int Cali_count=1; //IMU Clibration
bool imuCalibration = false;

bool New_IMU_DATA = 0; //flag for IMU data handling
uint8_t IMU_SPI_Buffer[12];


//Barometer
Baro_HandleTypedef BMP280;


//PID

/*Angle Mode*/
PID_Variables Pitch_Angle_PID;
PID_Variables Pitch_Rate_PID;
PID_Variables Roll_Angle_PID;
PID_Variables Roll_Rate_PID;
PID_Variables Yaw_PID;

/*Alititude Mode*/
Altitude_Data_t ALT;
PID_Variables Altitude_PID;
PID_Variables AltVelo_PID;

//PID output definition
float Pitch_CMD = 0;
float Roll_CMD = 0;
float Yaw_CMD = 0;
float Alt_Throttle = 0;

//Motor Control Variables
uint16_t motor_cmd[4]={0,0,0,0};
uint16_t Throttle;
float Pitch_Setpoint;
float Roll_Setpoint;
float Yaw_Angle_Setpoint;
float Yaw_Rate_Setpoint;
float altVelo_Setpoint;


float altCommand = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */
void PROCCESS_PID(void);


float ct =0 ;
float pt = 0;
float dt = 0;



/* Use RxEventCallBack so there is idle detection feature */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	//Receiver RX
    if (huart->Instance == USART1)
    {
    	ct = TIM6_GetTick();
    	dt = ct-pt;
    	pt=ct;

    	processRX();

	    /* Clear Interrupt Flag */
	    __HAL_UART_CLEAR_FLAG(&huart1, UART_FLAG_TC);

	    /* start the DMA again */
		HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx.rxChannel.uartWorkingBuffer, sizeof(rx.rxChannel.uartWorkingBuffer));
		Disable_Uart1_Half_Callback();
    }

    //Flow RX
    if(huart->Instance == USART2)
    {
    	opFlow_Data_Process();
    	__HAL_UART_CLEAR_FLAG(&huart2, UART_FLAG_TC);
    }

    //LiDar RX
    if(huart->Instance == UART4)
    {
    	RF_DataProcess();
    	HAL_UARTEx_ReceiveToIdle_DMA(&RangeFinderBusType, RangeFinder.LiDar.tfminiUartBuffer, sizeof(RangeFinder.LiDar.tfminiUartBuffer));
    	__HAL_UART_CLEAR_FLAG(&RangeFinderBusType, UART_FLAG_TC);
    }

    //GPS RX
    if (huart->Instance == UART5)
    {
    	GNSS_ParseBuffer(&gps);

	    /* Clear Interrupt Flag */
	    __HAL_UART_CLEAR_FLAG(&huart5, UART_FLAG_TC);

	    /* start the DMA again */
		HAL_UARTEx_ReceiveToIdle_DMA(&huart5, gps.uartWorkingBuffer, 100);
	    Disable_Uart5_Half_Callback();

    }
}

/* Uart1 Error Handler */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        // Handle the error condition
        __HAL_UART_CLEAR_FLAG(&huart1, UART_FLAG_ORE | UART_FLAG_NE | UART_FLAG_FE); //clear error flag
		HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx.rxChannel.uartWorkingBuffer, sizeof(rx.rxChannel.uartWorkingBuffer));
    }
    if (huart->Instance == USART2)
    {
        __HAL_UART_CLEAR_FLAG(&huart2, UART_FLAG_ORE | UART_FLAG_NE | UART_FLAG_FE); //clear error flag
    	HAL_UARTEx_ReceiveToIdle_DMA(opFlow.mtf02.huart, opFlow.mtf02.MTF_UartWorkingBuffer, sizeof(opFlow.mtf02.MTF_UartWorkingBuffer));
    }
    if (huart->Instance == UART4)
    {
        __HAL_UART_CLEAR_FLAG(&huart4, UART_FLAG_ORE | UART_FLAG_NE | UART_FLAG_FE); //clear error flag
    	HAL_UARTEx_ReceiveToIdle_DMA(&RangeFinderBusType, RangeFinder.LiDar.tfminiUartBuffer, sizeof(RangeFinder.LiDar.tfminiUartBuffer));
    }

    if (huart->Instance == UART5)
    {
        // Handle the error condition
        __HAL_UART_CLEAR_FLAG(&huart5, UART_FLAG_ORE | UART_FLAG_NE | UART_FLAG_FE); //clear error flag
    	HAL_UARTEx_ReceiveToIdle_DMA(&huart5, gps.uartWorkingBuffer, 100);
    }
}


Batt_Type batt;

// Callback function for ADC
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC1)
  {
    uint32_t adc_value = HAL_ADC_GetValue(hadc);
    // Do something with adc_value
    batt = GetBatteryType(getBatteryVoltage(adc_value));


  }
}



//Timer Interrupt
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

	//double check with timer 7
	if(htim->Instance == TIM7) //Currently this timer interrupt runs at 8kHz
	{


	}
}

/* Callback function for GPIO EXTI */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == imuDataReadyPin)
    {
    	//default disarmed
    	sysHealth.ARMED = false;
    	//check calibration and system mode
    	if((imuCalibration == false)&&(MagisCalibrating == false))
    	{
    		sysHealth.ARMED = ArmingHandle (rx.rxChannel.mapped_Channel); //arming flag
			System = System_Mode_Handle(rx.rxChannel.mapped_Channel);
			/*-------System Algorithm-------*/
			switch(System)
			{
			case AngleMode:
				// Read IMU data
				ICM42688_ReadAccelGyroData(&ACC,&Gyro);
				Accel_Angle_Convert(&ACC);
				Gyro_Angle_Convert(&Gyro);
				imuAttitude(Gyro.dt, Gyro.PT2_X, Gyro.PT2_Y, Gyro.PT2_Z, ACC.x, ACC.y, ACC.z, ACC.Magnitude,sysHealth.ARMED);
				ICM42688_GetVelocity(&ACC);
				Altitude_Rate_Data_Fusion(&ALT, &ACC, &BMP280, Process_BMP280(&BMP280),sysHealth.ARMED);

				static int prescale;
				prescale++;
				if(prescale >=200)
				{
					prescale = 0;
					GNSS_GetPVTData(&gps);
				}

				PROCCESS_PID();
				dshot_write(motor_cmd);

				break;
			case ALTMode:
				// Read IMU data
				ICM42688_ReadAccelGyroData(&ACC,&Gyro);
				Accel_Angle_Convert(&ACC);
				Gyro_Angle_Convert(&Gyro);
				imuAttitude(Gyro.dt, Gyro.PT2_X, Gyro.PT2_Y, Gyro.PT2_Z, ACC.x, ACC.y, ACC.z, ACC.Magnitude, ArmingHandle (rx.rxChannel.mapped_Channel));

				//Get Z Velocity From IMU and Barometer
				ICM42688_GetVelocity(&ACC);
				//Process_BMP280(&BMP280);

				//Z Velocity Data Fusion
				Altitude_Rate_Data_Fusion(&ALT, &ACC, &BMP280, Process_BMP280(&BMP280),sysHealth.ARMED);

				PROCCESS_PID();
				dshot_write(motor_cmd);
				break;
			case POSHold:
				ICM42688_ReadAccelGyroData(&ACC,&Gyro);
				Gyro_Angle_Convert(&Gyro);
				PROCCESS_PID();
				dshot_write(motor_cmd);
				break;

			case magCalibrate:
				MagisCalibrating = true;
				//reset all mag trim
				mag.Xtrim = 0;
				mag.Ytrim = 0;
				mag.Ztrim = 0;
				break;
				}
    	}

    	/*if in calibration mode*/
    	else if(imuCalibration == true)
    	{
			__disable_irq();

			ICM42688_ReadAccData(&ACC);
			ICM42688_ReadGyroData(&Gyro);

			accCalibration(&ACC, Cali_count);
			gyroCalibration(&Gyro, Cali_count);

			Cali_count++;

			if(Cali_count> CALIBRATING_GYRO_CYCLES)
			{
				HAL_GPIO_WritePin(imuLEDport, imuLEDpin, GPIO_PIN_SET);

				imuCalibration = false;
				Cali_count = 1;
			}
			ALT.Vz = 0;
			__enable_irq();
    	}

    	/*if in mag calibration mode*/
    	if((MagisCalibrating == true) && (imuCalibration != true)) //if both true, then prioritize imu
    	{
    		QMC_read(&mag);
    		magCalibration(&mag.magADC);
    	}
    }


    if(GPIO_Pin == GP_ButtonPin)
    {
    	__disable_irq();
    	HAL_GPIO_WritePin(imuLEDport, imuLEDpin, GPIO_PIN_RESET);

    	imuCalibration = true;
    	Cali_count=1;
    	__enable_irq();
    }
}

void PROCCESS_PID(void)
{
	if(sysHealth.ARMED && isBaroReady())
	{

		/*General Purpose Calculation*/
		if((System == AngleMode) || (System == ALTMode))
		{
			//Setpoint Assigning
			Pitch_Setpoint = -1*(Calculate_Setpoint(rx.rxChannel.mapped_Channel[2], Max_Angle));
			Roll_Setpoint = Calculate_Setpoint(rx.rxChannel.mapped_Channel[3], Max_Angle);

			Yaw_Rate_Setpoint = -1* Calculate_Setpoint(rx.rxChannel.mapped_Channel[1], Max_Yaw_Rate); //yaw is direct rate control

			//PID Calculation
			Pitch_CMD = Cascade_PID(&Pitch_Angle_PID, &Pitch_Rate_PID,Pitch_Setpoint, attitude.values.roll/10.0f, Gyro.PT2_X, 1.0f/PID_Freq, pitch);//RatePitch is not avaliable so I use AnglePitch to Debug
			Roll_CMD = Cascade_PID(&Roll_Angle_PID, &Roll_Rate_PID,Roll_Setpoint, attitude.values.pitch/10.0f, Gyro.PT2_Y, 1.0f/PID_Freq, roll);

			//manage yaw heading setpoint differently
			if(Yaw_Rate_Setpoint == 0)
			{
				float yawRateCMD = 0;
				float angleDifference = Yaw_Angle_Setpoint - attitude.values.yaw/10.0f;
				angleDifference = mod(angleDifference + 180, 360) - 180;
				yawRateCMD = (-1*angleDifference) * P_angleZ; //pre-calculate the angle setpoint

				Yaw_CMD = PID_Equation(&Yaw_PID, yawRateCMD, Gyro.PT2_Z, 1.0f/PID_Freq, yaw);
			}
			else
			{
				Yaw_Angle_Setpoint = attitude.values.yaw/10.0f;
				Yaw_CMD = PID_Equation(&Yaw_PID, Yaw_Rate_Setpoint, Gyro.PT2_Z, 1.0f/PID_Freq, yaw);//Yaw PID should input Yaw angular velocity
			}


		}

		/*Specific Mode Calculation*/
		switch(System)
		{
			case AngleMode:
				//Throttle Assignment
				Throttle =((rx.rxChannel.mapped_Channel[0]-988)*1.999) + 200; //+100 for 5%idle


				if(rx.rxChannel.mapped_Channel[0]<990)
				{
					Roll_Rate_PID.Integral = 0;
					Pitch_Rate_PID.Integral = 0;
				}
				motor_cmd[3]= Throttle - Pitch_CMD + Roll_CMD - Yaw_CMD ;//M4
				motor_cmd[2]= Throttle + Pitch_CMD + Roll_CMD + Yaw_CMD ;//M3
				motor_cmd[1]= Throttle - Pitch_CMD - Roll_CMD + Yaw_CMD ;//M2
				motor_cmd[0]= Throttle + Pitch_CMD - Roll_CMD - Yaw_CMD ;//M1
				break;

			case ALTMode:
				//setpoint Calculation
				int sign = 0;
				sign = (rx.rxChannel.mapped_Channel[4] <= 1000) ? 1 : -1;

				altVelo_Setpoint = sign * (300.0f/1023.0f)*(rx.rxChannel.mapped_Channel[0]-989);



				if(((attitude.values.pitch /10.0f) <= Max_Angle) && ((attitude.values.roll/10.0f) <= Max_Angle))
				{
					Alt_Throttle = PID_Equation(&AltVelo_PID, altVelo_Setpoint, ALT.Vz, 1.0f/PID_Freq, alt);
				}

				altCommand = altitudeControl();

				motor_cmd[3]= Alt_Throttle - Pitch_CMD + Roll_CMD - Yaw_CMD + altCommand;//M4
				motor_cmd[2]= Alt_Throttle + Pitch_CMD + Roll_CMD + Yaw_CMD + altCommand;//M3
				motor_cmd[1]= Alt_Throttle - Pitch_CMD - Roll_CMD + Yaw_CMD + altCommand;//M2
				motor_cmd[0]= Alt_Throttle + Pitch_CMD - Roll_CMD - Yaw_CMD + altCommand;//M1

				break;

			case POSHold:


				break;
		}






	//			//PID Calculation
	//			Pitch_CMD = Cascade_PID(&Pitch_Angle_PID, &Pitch_Rate_PID,Pitch_Setpoint, AnglePitch, AnglePitch, 500);//RatePitch is not avaliable so I use AnglePitch to Debug
	//			Roll_CMD = Cascade_PID(&Roll_Angle_PID, &Roll_Rate_PID,Roll_Setpoint, AngleRoll, AngleRoll, 500);
	//			Yaw_CMD = PID_Equation(&Yaw_PID, Yaw_Setpoint, AngleRoll, 500);//Yaw PID should input Yaw angular velocity

	//		 M4			  M2			y+
	//		    .       .				|
	//            .   . 				|
	//			  .   . 				|
	//			.       .				|
	//		 M3 		  M1			|-----------x+

	//			//Assign new motor value

		dshot_check_threshold(motor_cmd, ArmingHandle (rx.rxChannel.mapped_Channel));
	}
	else
	{
		//Reset All Throttle Value
		Throttle = 0;
		Alt_Throttle = 0;

		//Reset Motor Command
		memset(motor_cmd,0,sizeof(motor_cmd));

		//Reset All Integral Term
		Roll_Angle_PID.Integral = 0;
		Pitch_Angle_PID.Integral = 0;
		Roll_Rate_PID.Integral = 0;
		Pitch_Rate_PID.Integral = 0;
		AltVelo_PID.Integral = 0;


		//reset heading setpoint
		Yaw_Angle_Setpoint = attitude.values.yaw;

	}
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  __disable_irq(); // Disable global interrupts for proper init

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_TIM6_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();
  MX_TIM7_Init();
  MX_UART5_Init();
  MX_ADC1_Init();
  MX_USART2_UART_Init();
  MX_UART4_Init();
  /* USER CODE BEGIN 2 */

  	  //Reset All System Health Falge
  sysHealth.ARMED = false;
  sysHealth.RangeFinderHealthy = false;
  sysHealth.rxHealthy = false;
  sysHealth.baroHealthy = false;
  sysHealth.gpsHealthy = false;
  sysHealth.imuHealty = false;
  sysHealth.magHealthy = false;
  sysHealth.opFlowHealthy = false;



  init();


  HAL_GPIO_WritePin(beeperPort, beeperPin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(beeperPort, beeperPin, GPIO_PIN_SET);
  Delay_ms(1000);
  HAL_GPIO_WritePin(beeperPort, beeperPin, GPIO_PIN_RESET);

  	  //Barometer
  sysHealth.baroHealthy = (BMP280_Init(&BMP280, &BMP280.params, &baroBuyType)==true);


	  //LiDar
  //TFmini_Plus_init(&RangeFinder.LiDar, &RangeFinderBusType, 1000,115200);
	//HAL_UARTEx_ReceiveToIdle_DMA(&mtf02.huart, mtf02.MTF_UartWorkingBuffer, sizeof(mtf02.MTF_UartWorkingBuffer));


  	  //PID
  PID_Init(&Pitch_Angle_PID, P_angleX, I_angleX, D_angleX,0, -100, 100);	//Pitch Angle
  PID_Init(&Pitch_Rate_PID, P_rateX, I_rateX, D_rateX, Dmax_rateX, -500, 500);		//Pitch Rate

  PID_Init(&Roll_Angle_PID, P_angleY, I_angleY, D_angleY,0, -100, 100);	//Roll Angle
  PID_Init(&Roll_Rate_PID, P_rateY, I_rateY, D_rateY, Dmax_rateY, -500, 500);		//Roll Rate

  PID_Init(&Yaw_PID, P_rateZ, I_rateZ, D_rateZ,0,  -500, 500);				//Yaw Rate

  PID_Init(&Altitude_PID, P_Alt, I_Alt, D_Alt,0, -100, 100); 				//Altitude
  PID_Init(&AltVelo_PID, P_veloAlt, I_veloAlt, D_veloAlt,0, -500, 500);	//Altitude Velocity


  	  //Load Calibration Offset
  loadCalibrationOffsets(&ACC,&Gyro,&mag);

  	  //Attitude
  imu_init();

  HAL_TIM_Base_Start_IT(&htim7); //Start the timer


  	  //Vbatt ADC
  HAL_ADC_Start_IT(&BattADC);

  //System = AngleMode;
  __enable_irq(); // Enable global interrupts for proper USB Init

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
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

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
