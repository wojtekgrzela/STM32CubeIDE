/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "adc.h"
#include "dma.h"
#include "fatfs.h"
#include "i2c.h"
#include "sdio.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "../GPS/GPS_Parsing.h"
#include "Init_Functions.h"

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

extern ENCButton_struct ENC_button_menu;
extern volatile uint16_t ADC1Measures[NO_OF_ADC1_MEASURES];
extern volatile uint16_t ADC3Measures[NO_OF_ADC3_MEASURES];
extern GPS_data_struct GPS;
extern uint64_t Tim7_Counter_100us;
extern osTimerId My_Timer_ENC_Menu_ButtonHandle;

extern volatile uint32_t RPM_counter;
extern volatile uint32_t SPEED_counter;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

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
	Error_Code error = NO_ERROR;
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
  MX_ADC1_Init();
  MX_ADC3_Init();
  MX_I2C2_Init();
  MX_SDIO_SD_Init();
  MX_I2C3_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_FATFS_Init();
  MX_TIM1_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */

	error = (Error_Code)HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);

	if (NO_ERROR != error)
	{
		error = TIM__ENCODER_START_FAIL;
		my_error_handler(error);
	}

	/* There is no Calibration services in STM32F407IGTx devices */
	/* HAL_ADCEx_Calibration_Start */

	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC1Measures, NO_OF_ADC1_MEASURES);
	HAL_ADC_Start_DMA(&hadc3, (uint32_t*)ADC3Measures, NO_OF_ADC3_MEASURES);

//	error = (Error_Code)HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);

	if (NO_ERROR != error)
	{
		error = TIM__PWM_START_FAIL;
		my_error_handler(error);
	}

	HAL_Delay(300);

	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
	/** !!!!TURNING THIS ON WILL ERASE WHOLE EEPROMS!!!! **/
#if 0
error = EraseWholeEEPROM(&EEPROM_car);
	if(NO_ERROR != error)
	{
		error = EEPROM__WHOLE_MEMORY_ERASE_FAIL;
		my_error_handler(error);
	}
#endif

#if 0
error = EraseWholeEEPROM(&EEPROM_board);
	if(NO_ERROR != error)
	{
		error = EEPROM__WHOLE_MEMORY_ERASE_FAIL;
		my_error_handler(error);
	}
#endif
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

  /* USER CODE END 2 */

  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();
  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
		error = OS__MAIN_WHILE_LOOP_REACHED;
		my_error_handler(error);
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
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* UART communication via Interrupts handling */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	static uint8_t counter = 0;
	static uint8_t nCounter = 0;
	static uint8_t rCounter = 0;
	static uint8_t dataStarted = 0; /* Set when "$" (so the first sign in the string) is found and Reset when string ends (after second \n\r */

#ifdef GPS_RECEIVING
	/* Checking if interrupt is from UART1 (GPS) */
	if (&huart1 == huart)
	{
		if (FALSE == GPS.DataReady)
		{
			if ('$' == GPS.receivedByte)
				dataStarted = TRUE;

			if (TRUE == dataStarted)
			{
				if ('\r' == GPS.receivedByte)
				{
					++rCounter;
					GPS.GPS_buffer[counter] = GPS.receivedByte;
					++counter;
				}
				else
				{
					if ('\n' == GPS.receivedByte)
					{
						++nCounter;
						GPS.GPS_buffer[counter] = GPS.receivedByte;
						++counter;
					}
					else
					{
						if (((2 > rCounter) || (2 > nCounter)) && (GPS_BUFFER_SIZE > counter))
						{
							GPS.GPS_buffer[counter] = GPS.receivedByte;
							++counter;
						}
						else
						{
							GPS.DataReady = TRUE;
							counter = 0;
							rCounter = 0;
							nCounter = 0;
							dataStarted = FALSE;
						}    //if((2 > rCounter) || (2 > nCounter))
					}    //if('\n' == GPS.receivedByte)
				}    //if('\r' == GPS.receivedByte)
			}    //if(TRUE == dataStarted)
		}    //if(FALSE == GPS.DataReady)

		HAL_UART_Receive_IT(&huart1, (uint8_t*)(&GPS.receivedByte), 1u);
	}    //if(&huart1 == *huart)
#endif
}



void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	static uint64_t lastTimerCounter = 0;
	Error_Code error = NO_ERROR;

	/* Checking if interrupt is from ENC_BUTTON pin */
	if (ENC_BUTTON_MENU_Pin == GPIO_Pin)
	{
		if ((Tim7_Counter_100us - lastTimerCounter) > (DEBOUNCING_TIME_FOR_ENCODER_BUTTON * 10U) /*ms*/)
		{
			if (GPIO_PIN_RESET == HAL_GPIO_ReadPin(ENC_BUTTON_MENU_GPIO_Port, ENC_BUTTON_MENU_Pin)) /*Button Pushed In*/
			{
				error = (Error_Code)osTimerStart(My_Timer_ENC_Menu_ButtonHandle, ENC_BUTTON_LONG_PRESS_TIME);
				if (NO_ERROR != error)
				{
					error = OS__STARTING_TIMER_FAILED;
					my_error_handler(error);
				}
			}
			else /*Button Released*/
			{
				if (FALSE == ENC_button_menu.longPressInfoForISR) /* If long press was NOT detected then it is a short press */
				{
					error = (Error_Code)osTimerStop(My_Timer_ENC_Menu_ButtonHandle); /* Stop the timer */
					if (NO_ERROR != error)
					{
						error = OS__STOPPING_TIMER_FAILED;
						my_error_handler(error);
					}
					ENC_button_menu.allFlags = 0b00001001; /* shortPressDetected and shortPressDetectedBuzzer set to 1 */
				}
				else /* Long press was detected so do not set short press and reset the flag for ISR for the next time */
				{
					ENC_button_menu.longPressInfoForISR = FALSE;
				}

			}
			lastTimerCounter = Tim7_Counter_100us;
		}
	}

	if(RPM_SIGNAL_Pin == GPIO_Pin)
	{
		++RPM_counter;
	}

	if(SPEED_SIGNAL_Pin == GPIO_Pin)
	{
		++SPEED_counter;
	}
}

/* USER CODE END 4 */

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

	/* Timer 7 is used for stats and for time counting (resolution is 100us) */
	if (&htim7 == htim)
	{
		++Tim7_Counter_100us;
	}

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */

	my_error_handler(UNKNOWN_ERROR);

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
