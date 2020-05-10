/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "../GPS/GPS_Parsing.h"

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
volatile uint32_t licznikShortPresses = 0;
extern ENCButton_struct ENC_button;
extern uint16_t ADC3Measures[NO_OF_ADC3_MEASURES];
extern GPS_data_struct GPS;
extern LCD_parameters_struct LCD;
extern uint32_t Tim7_Counter_100us;
extern osTimerId My_Timer_ENC_ButtonHandle;

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
  MX_I2C2_Init();
  MX_TIM8_Init();
  MX_ADC3_Init();
  MX_USART1_UART_Init();
  MX_TIM7_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL);
  HAL_ADC_Start_DMA(&hadc3, (uint32_t*)ADC3Measures, NO_OF_ADC3_MEASURES);

  HAL_Delay(300);

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
	static uint8_t dataStarted = 0;		/* Set when "$" (so the first sign in the string) is found and Reset when string ends (after second \n\r */

#ifdef GPS_RECEIVING
	/* Checking if interrupt is from UART1 (GPS) */
	if(&huart1 == huart)
	{
		if(FALSE == GPS.DataReady)
		{
			if('$' == GPS.receivedByte)
				dataStarted = TRUE;

			if(TRUE == dataStarted)
			{
				if('\r' == GPS.receivedByte)
				{
					++rCounter;
						GPS.GPS_buffer[counter] = GPS.receivedByte;
						++counter;
				}
				else
				{
					if('\n' == GPS.receivedByte)
					{
						++nCounter;
							GPS.GPS_buffer[counter] = GPS.receivedByte;
							++counter;
					}
					else
					{
						if(((2 > rCounter) || (2 > nCounter)) &&  (GPS_BUFFER_SIZE > counter))
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
						}//if((2 > rCounter) || (2 > nCounter))
					}//if('\n' == GPS.receivedByte)
				}//if('\r' == GPS.receivedByte)
			}//if(TRUE == dataStarted)
		}//if(FALSE == GPS.DataReady)

		HAL_UART_Receive_IT(&huart1, (uint8_t *)(&GPS.receivedByte), 1u);
	}//if(&huart1 == *huart)
#endif
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	static uint32_t lastTimerCounter = 0;
	Error_Code error = NO_ERROR;

	/* Checking if interrupt is from ENC_BUTTON pin */
	if(ENC_BUTTON_Pin == GPIO_Pin)
	{
		if((Tim7_Counter_100us - lastTimerCounter) > 100/*10ms*/)
		{
			if(GPIO_PIN_SET == HAL_GPIO_ReadPin(ENC_BUTTON_GPIO_Port, ENC_BUTTON_Pin))	/*Button Pushed In*/
			{
				error = osTimerStart(My_Timer_ENC_ButtonHandle, ENC_BUTTON_LONG_PRESS_TIME);
				if(NO_ERROR != error)
				{
					error = OS__STARTING_TIMER_FAILED;
				}
			}
			else	/*Button Released*/
			{
				if(FALSE == ENC_button.longPressInfoForISR)	/* If long press was NOT detected then it is a short press */
				{
					error = osTimerStop(My_Timer_ENC_ButtonHandle);	/* Stop the timer */
					if(NO_ERROR != error)
					{
						error = OS__STOPPING_TIMER_FAILED;
					}
					ENC_button.shortPressDetected = TRUE;
					HAL_GPIO_TogglePin(LED_7_GPIO_Port, LED_7_Pin);
					++licznikShortPresses;
				}
				else	/* Long press was detected so do not set short press and reset the flag for ISR for the next time */
				{
					ENC_button.longPressInfoForISR = FALSE;
				}

			}
			lastTimerCounter = Tim7_Counter_100us;
		}
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
#ifdef RUNTIME_STATS_TIMER_CONFIG
  if (&htim7 == htim)
  {
	  ++Tim7_Counter_100us;
  }
#endif
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
