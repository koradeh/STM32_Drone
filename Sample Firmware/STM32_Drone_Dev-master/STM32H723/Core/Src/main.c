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
#include "octospi.h"
#include "rtc.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "GNSS.h"
#include "bmp180.h"
#include "MPU6050.h"
#include "dshot.h"
#include "crsf.h"
#include "PID.h"
#include "filters.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */


Link_Stat RX_Link;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//Max Pitch, Roll Angle
#define Max_Angle 45
#define Max_Yaw_Rate 10

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

//{PA5, PA7, PB10, PB1}
uint16_t motor_cmd[4]={0,0,0,0};

//receiver buffer
uint8_t Uart1_Buffer[64];



//Output CRSF Channel
uint16_t mapped_Channel[16];

//Motor Control Variables
uint16_t Throttle;
float Pitch_Setpoint;
float Roll_Setpoint;
float Yaw_Setpoint;

int inputPrev = 0;

//RC Flag
bool ARMED = 0;
bool Stick_Reset = 0;
bool rc_flag = 0;



//PID output definition
float Pitch_CMD = 0;
float Roll_CMD = 0;
float Yaw_CMD = 0;

uint8_t PID_Software_Prescaler = 0;

//PID loop definition
PID_Variables Pitch_Angle_PID;
PID_Variables Pitch_Rate_PID;
PID_Variables Roll_Angle_PID;
PID_Variables Roll_Rate_PID;
PID_Variables Yaw_PID;

float Roll =0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
//int _write(int file, char *ptr, int len)
//{
//  /* Implement your write code here, this is used by puts and printf for example */
//  int i=0;
//  for(i=0 ; i<len ; i++)
//    ITM_SendChar((*ptr++));
//  return len;
//}

//Receiver Buffer Interrupt
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart1) {
    if (huart1->Instance == USART1)
    {
    	//Arrange each packet into a specific packet
    	Packet_Type_Arrange(Uart1_Buffer);
    }
    HAL_UART_Receive_DMA(&huart1, Uart1_Buffer, sizeof(Uart1_Buffer));
}


//Timer Interrupt
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{


	//double check with timer 7
	if(htim->Instance == TIM7) //Currently this timer interrupt runs at 8kHz
	{

//		MPU6050_Signal();
		if (MPU6050_DataReady() == 1){
			MPU6050_Get_6_AxisRawData(&MPU6050);
			MPU6050_Real_Values(&MPU6050);
			//MPU6050_AngleConvert(&MPU6050);
			MPU6050_KalmanFilter(&MPU6050);
		}

		//Process the data in RxBuffer at 8kHz
		Parse_ELRS_Channels();

		if ((mapped_Channel[7]>1800) && (mapped_Channel[0]<=990) & Stick_Reset)
		{
			ARMED = 1;
		}
		else if (mapped_Channel[7]<1800)
		{
			ARMED = 0;
			Stick_Reset = 0;
		}

		if((mapped_Channel[7]<1800) && (mapped_Channel[0]<=1000))
		{
			Stick_Reset = 1;
		}

		PID_Software_Prescaler ++;




		if((PID_Software_Prescaler == 16)&& ARMED)//8kHz/16 = 500hz
		{

			//Low pass Filter
			int input =(mapped_Channel[0]-988)*1.999;
			Throttle = 0.1*input + (0.9*inputPrev);

			//Setpoint Assigning
			Pitch_Setpoint = Calculate_Setpoint(mapped_Channel[2], Max_Angle);
			Roll_Setpoint = Calculate_Setpoint(mapped_Channel[3], Max_Angle);
			Yaw_Setpoint = Calculate_Setpoint(mapped_Channel[1], Max_Yaw_Rate);

			//PID Calculation
			Pitch_CMD = Cascade_PID(&Pitch_Angle_PID, &Pitch_Rate_PID,Pitch_Setpoint, MPU6050.KalmanAngleY, MPU6050.KalmanAngleY, 500);//RatePitch is not avaliable so I use AnglePitch to Debug
			Roll_CMD = Cascade_PID(&Roll_Angle_PID, &Roll_Rate_PID,Roll_Setpoint, MPU6050.KalmanAngleX, MPU6050.KalmanAngleX, 500);
			Yaw_CMD = PID_Equation(&Yaw_PID, Yaw_Setpoint, MPU6050.Gyro_Z_RealValue, 500);//Yaw PID should input Yaw angular velocity

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
			motor_cmd[0]= Throttle - Pitch_CMD + Roll_CMD + Yaw_CMD;//M1
			motor_cmd[1]= Throttle + Pitch_CMD + Roll_CMD - Yaw_CMD;//M2
			motor_cmd[2]= Throttle - Pitch_CMD - Roll_CMD - Yaw_CMD;//M3
			motor_cmd[3]= Throttle + Pitch_CMD - Roll_CMD + Yaw_CMD;//M4

			//Process Link Statistic at 500Hz
			Process_Link_Stat_Packets();

			//assign prev throttle
			inputPrev = Throttle;

			//Reset Prescaler
			PID_Software_Prescaler = 0;
		}

		//Continuously sending dshot to prevent ESC to trigger failsafe
		dshot_write(motor_cmd);


		Roll = MPU6050.KalmanAngleY;
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

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_OCTOSPI1_Init();
  MX_RTC_Init();
  MX_USB_DEVICE_Init();
  MX_I2C1_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */

//  GNSS_Init(&GNSS_Handle, &huart4);
//  HAL_Delay(500);
//  GNSS_LoadConfig(&GNSS_Handle);


  //Receiver Init
  HAL_UART_Receive_DMA(&huart1, Uart1_Buffer, sizeof(Uart1_Buffer));

  //Acc Gyro Init
  MPU_Config();
  MPU6050_Initialization();
  MPU6050_Calibration(&MPU6050, 1); //1 is for Enabling the Flatness check, 2 is for disable Flatness check

  //Motor Protocol Init
  dshot_init(DSHOT600);

  //PID Control Init
  PID_Init(&Pitch_Angle_PID, 0, 1, 0, -100, 100);
  PID_Init(&Pitch_Rate_PID, 1, 0, 0, -100, 100);
  PID_Init(&Roll_Angle_PID, 0, 1, 0, -100, 100);
  PID_Init(&Roll_Rate_PID, 1, 0, 0, -100, 100);
  PID_Init(&Yaw_PID, 1, 0, 0, -100, 100);


  //Timer Begin
  HAL_TIM_Base_Start_IT(&htim7); //Start the timer



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if(ARMED)
	  {

		  //HAL_TIM_Base_Start_IT(&htim7); //Start the timer
	  }
	  else
	  {
		  //HAL_TIM_Base_Stop_IT(&htim7); //Pause or stop the timer
		  Throttle = 0;
		  memset(motor_cmd,0,sizeof(motor_cmd)); //clear motor command
	  }

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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE
                              |RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 10;
  RCC_OscInitStruct.PLL.PLLN = 220;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 5;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_1;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */



/* USER CODE END 4 */

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
