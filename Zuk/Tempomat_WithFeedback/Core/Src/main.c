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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "PID.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef enum {
	NO_ERROR =						(0u),

	EXT_INTERRUPT_NOT_RECOGNIZED =	(2u)
}ERROR_CODE;

typedef enum {
	NOTHING = 						(0),
	DECREASE = 						(-1),
	INCREASE = 						(1)
}DIRECTION;

typedef enum{
	NO_SOUND =						(0u),

	BUTTON_CLICK =					(2u)
}BUZZER_SOUNDS;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* PID regulator defines */
#define P_REGULATOR_GAIN			((float)(1.0))
#define I_REGULATOR_GAIN			((float)(0.5))
#define D_REGULATOR_GAIN			((float)(1.0))
#define DT_SAMPLING_RERIOD			((float)(1000))		//in ms

/* BUZZER defines */
#define BUTTON_CLICK_BUZZER_LENGTH	((uint32_t)(100))	// in ms

/* ADC defines */
#define ADC_NO_OF_READINGS			((uint32_t)(1))
#define MAX_ADC_ACCELERATION_READOUT	((uint32_t)(2000))	//TODO: max what an ADC can read as a max gas pedal position
#define MIN_ADC_ACCELERATION_READOUT	((uint32_t)(1000))	//TODO: min what an ADC can read as a min gas pedal position
#define ADC_VALUE_MULTIPLER			((uint32_t)(1))		// in case the control values were too strong for the ADC values, the values can be adjusted
#define ADC_ERROR_GAIN				((int32_t)(1))

/* RPM sefines */
#define MAX_ALLOWED_RPM				((uint32_t)(3500))
#define MIN_ALLOWED_RPM				((uint32_t)(1200))
#define SHUTDOWN_HIGH_RPM			((uint32_t)(3700))
#define ENCODER_RPM_SETTING_STEP	((uint32_t)(10))
#define IMPULSES_PER_ONE_ENGINE_REVOLUTION	((uint32_t)(12))

/* TIMERs defines */
#define PWM_TIMER					(TIM3)
#define PWM_RESOLUTION				(PWM_TIMER->ARR)
#define PWM_PULSE					(PWM_TIMER->CCR3)

/* H-BRIDGE defines */
#define DECREASE_ENG_TERMINAL_PORT	(IN2_ENG_GPIO_Port)
#define DECREASE_ENG_TERMINAL_PIN	(IN2_ENG_Pin)
#define INCREASE_ENG_TERMINAL_PORT	(IN1_ENG_GPIO_Port)
#define INCREASE_ENG_TERMINAL_PIN	(IN1_ENG_Pin)

/* REGULATING constants */
#define MAX_CONTROL_SIGNAL			((int32_t)(100))
#define MIN_CONTROL_SIGNAL			((int32_t)(-100))

/* LEDs defines */
#define BUILT_IN_LED_PORT			(LED_GPIO_Port)
#define BUILT_IN_LED_PIN			(LED_GPIO_Pin)

/* Constant values */
#define TRUE	((uint8_t)(1))
#define True	((uint8_t)(1))
#define true	((uint8_t)(1))
#define FALSE	((uint8_t)(0))
#define False	((uint8_t)(0))
#define false	((uint8_t)(0))
#define ON		((uint8_t)(1))
#define OFF		((uint8_t)(0))

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

osThreadId My_Task_1000msHandle;
osThreadId EngineControlHandle;
/* USER CODE BEGIN PV */

volatile uint32_t ADC1_results[ADC_NO_OF_READINGS] = {0};
volatile uint32_t ADCDesiredAccelerationValue = 0;
static volatile ERROR_CODE ErrorCode = NO_ERROR;
static volatile BUZZER_SOUNDS BuzzerSounds = NO_SOUND;

static volatile uint8_t STATE = OFF;
static volatile uint32_t RPM_counter = 0;
static uint32_t Set_RPM = 2000;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
void Start_My_Task_1000ms(void const * argument);
void Start_EngineControl(void const * argument);

/* USER CODE BEGIN PFP */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void Encoder_Button_Function(void);
void Set_Direction(DIRECTION direction);
void Set_Electromagnes_On(void);
void Set_Electromagnes_Off(void);
void Complete_Shutdown(void);
void Update_Encoder_Settings(void);
void Make_Soud(void);


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
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  HAL_GPIO_WritePin(LED_READY_GPIO_Port, LED_READY_Pin, SET);
  HAL_GPIO_WritePin(LED_WORKING_GPIO_Port, LED_WORKING_Pin, SET);
  HAL_GPIO_WritePin(LED_ERROR_GPIO_Port, LED_ERROR_Pin, SET);

  while(HAL_OK != HAL_ADCEx_Calibration_Start(&hadc1));


  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC1_results, ADC_NO_OF_READINGS);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_Base_Start(&htim2);

  HAL_Delay(1000);

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of My_Task_1000ms */
  osThreadDef(My_Task_1000ms, Start_My_Task_1000ms, osPriorityNormal, 0, 256);
  My_Task_1000msHandle = osThreadCreate(osThread(My_Task_1000ms), NULL);

  /* definition and creation of EngineControl */
  osThreadDef(EngineControl, Start_EngineControl, osPriorityBelowNormal, 0, 128);
  EngineControlHandle = osThreadCreate(osThread(EngineControl), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4095;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 15;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 15;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 15;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 199;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED_READY_Pin|LED_WORKING_Pin|LED_ERROR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, IN2_ENG_Pin|IN1_ENG_Pin|EN_EMAG_Pin|LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_READY_Pin LED_WORKING_Pin LED_ERROR_Pin */
  GPIO_InitStruct.Pin = LED_READY_Pin|LED_WORKING_Pin|LED_ERROR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : ENCODER_BUTTON_Pin */
  GPIO_InitStruct.Pin = ENCODER_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(ENCODER_BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BUZZER_Pin */
  GPIO_InitStruct.Pin = BUZZER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BUZZER_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : IN2_ENG_Pin IN1_ENG_Pin EN_EMAG_Pin */
  GPIO_InitStruct.Pin = IN2_ENG_Pin|IN1_ENG_Pin|EN_EMAG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BRAKE_Pin CLUTCH_Pin */
  GPIO_InitStruct.Pin = BRAKE_Pin|CLUTCH_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : RPM_Pin */
  GPIO_InitStruct.Pin = RPM_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(RPM_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 8, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(RPM_Pin == GPIO_Pin)
	{
		++RPM_counter;
	}
	else
	{
		if((BRAKE_Pin == GPIO_Pin) || (CLUTCH_Pin == GPIO_Pin))
		{
			STATE = OFF;
			Set_Electromagnes_Off();
			Set_Direction(NOTHING);
			PWM_PULSE = 0u;
		}
		else
		{
			if(ENCODER_BUTTON_Pin == GPIO_Pin)
			{
				Encoder_Button_Function();
			}
			else
			{
				ErrorCode = EXT_INTERRUPT_NOT_RECOGNIZED;
			}
		}
	}
}


void Encoder_Button_Function(void)
{
	if(NO_SOUND == BuzzerSounds)
	{
		BuzzerSounds = BUTTON_CLICK;
	}

	if(ON == STATE)
	{
		STATE = OFF;
		Set_Electromagnes_Off();
	}
	else
	{
		STATE = ON;
		Set_Electromagnes_On();
	}
}


void Set_Direction(DIRECTION direction)
{
	switch (direction)
	{
		case DECREASE:
		{
			HAL_GPIO_WritePin(INCREASE_ENG_TERMINAL_PORT, INCREASE_ENG_TERMINAL_PIN, RESET);
			HAL_GPIO_WritePin(DECREASE_ENG_TERMINAL_PORT, DECREASE_ENG_TERMINAL_PIN, SET);
			break;
		}
		case INCREASE:
		{
			HAL_GPIO_WritePin(INCREASE_ENG_TERMINAL_PORT, INCREASE_ENG_TERMINAL_PIN, SET);
			HAL_GPIO_WritePin(DECREASE_ENG_TERMINAL_PORT, DECREASE_ENG_TERMINAL_PIN, RESET);
			break;
		}
		case NOTHING:
		{
			HAL_GPIO_WritePin(INCREASE_ENG_TERMINAL_PORT, INCREASE_ENG_TERMINAL_PIN, RESET);
			HAL_GPIO_WritePin(DECREASE_ENG_TERMINAL_PORT, DECREASE_ENG_TERMINAL_PIN, RESET);
			break;
		}
		default:
		{
			HAL_GPIO_WritePin(INCREASE_ENG_TERMINAL_PORT, INCREASE_ENG_TERMINAL_PIN, RESET);
			HAL_GPIO_WritePin(DECREASE_ENG_TERMINAL_PORT, DECREASE_ENG_TERMINAL_PIN, RESET);
			break;
		}
	}
}


void Set_Electromagnes_On(void)
{
	HAL_GPIO_WritePin(EN_EMAG_GPIO_Port, EN_EMAG_Pin, SET);
}


void Set_Electromagnes_Off(void)
{
	HAL_GPIO_WritePin(EN_EMAG_GPIO_Port, EN_EMAG_Pin, RESET);
}


void Complete_Shutdown(void)
{
	STATE = OFF;
	Set_Electromagnes_Off();
	Set_Direction(NOTHING);
	PWM_PULSE = 0u;
}


void Update_Encoder_Settings(void)
{
	static uint32_t lastEncValue = 0;
	uint32_t encCounterValue = (TIM2->CNT);
	int32_t encDiff = encCounterValue - lastEncValue;
	uint32_t TEMP_Set_RPM = Set_RPM;

	if ((encDiff >= 4u) || (encDiff <= -4))
	{
		encDiff /= 4u;
		TEMP_Set_RPM += (uint32_t)(((int8_t)(encDiff)) * ENCODER_RPM_SETTING_STEP);

		if (TEMP_Set_RPM > MAX_ALLOWED_RPM)
		{
			Set_RPM = MAX_ALLOWED_RPM;
		}
		else
		{
			if (TEMP_Set_RPM < MIN_ALLOWED_RPM)
			{
				Set_RPM = MIN_ALLOWED_RPM;
			}
			else
			{
				Set_RPM = TEMP_Set_RPM;
			}
		}//else

		lastEncValue = encCounterValue;
	}//if

}//void Update_Encoder_Settings(void)


void Make_Soud(void)
{
	static uint32_t iterator = 0;

	switch(BuzzerSounds)
	{
		case NO_SOUND:
		{
			HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, RESET);
			break;
		}
		case BUTTON_CLICK:
		{
			HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, SET);
			++iterator;
			if (iterator > BUTTON_CLICK_BUZZER_LENGTH)
			{
				BuzzerSounds = NO_SOUND;
				iterator = 0;
			}
			break;
		}
		default:
		{
			HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, RESET);
			break;
		}
	}
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_Start_My_Task_1000ms */
/**
  * @brief  Function implementing the My_Task_1000ms thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_Start_My_Task_1000ms */
void Start_My_Task_1000ms(void const * argument)
{
  /* USER CODE BEGIN 5 */
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = 1000;

	uint32_t RPM_from_IRQ = 0;
	int32_t RPM_error = 0;
	uint32_t ADC_value = 0;
	float controlValue = 0.0;

	WritePIDParameters(P_REGULATOR_GAIN, I_REGULATOR_GAIN, D_REGULATOR_GAIN, DT_SAMPLING_RERIOD);


	HAL_GPIO_WritePin(LED_READY_GPIO_Port, LED_READY_Pin, SET);
	HAL_GPIO_WritePin(LED_WORKING_GPIO_Port, LED_WORKING_Pin, RESET);
	HAL_GPIO_WritePin(LED_ERROR_GPIO_Port, LED_ERROR_Pin, RESET);


	xLastWakeTime = xTaskGetTickCount();

	/* Infinite loop */
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
	for(;;)
	{
		RPM_from_IRQ = RPM_counter * 60 / IMPULSES_PER_ONE_ENGINE_REVOLUTION;	// now we have revolution per minute
		RPM_counter = 0;	// reset counter

		Update_Encoder_Settings();

		RPM_error = Set_RPM - RPM_from_IRQ;
		ADC_value = ADC1_results[0] * ADC_VALUE_MULTIPLER;

		if(ON == STATE)
		{
			PWM_PULSE = 0;
			Set_Electromagnes_On();
			controlValue = RunPIDController((float)RPM_error);

			if(MIN_CONTROL_SIGNAL > (int32_t)controlValue)
			{
				controlValue = MIN_CONTROL_SIGNAL;
			}
			if(MAX_CONTROL_SIGNAL < (int32_t)controlValue)
			{
				controlValue = MAX_CONTROL_SIGNAL;
			}

			ADCDesiredAccelerationValue = ADC_value + controlValue;
		}
		else
		{
			Complete_Shutdown();
		}



		vTaskDelayUntil(&xLastWakeTime, xFrequency);
	}
  /* USER CODE END 5 */ 
}

/* USER CODE BEGIN Header_Start_EngineControl */
/**
* @brief Function implementing the EngineControl thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_EngineControl */
void Start_EngineControl(void const * argument)
{
  /* USER CODE BEGIN Start_EngineControl */
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = 1000;

	int32_t ADCError = 0;


	xLastWakeTime = xTaskGetTickCount();

	/* Infinite loop */
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
	for(;;)
	{
		ADCError = (ADCDesiredAccelerationValue - (ADC1_results[0] * ADC_VALUE_MULTIPLER)) * ADC_ERROR_GAIN;

		if(ON == STATE)
		{
			if(0 <= ADCError)
			{
				if(PWM_RESOLUTION < ADCError)
				{
					ADCError = PWM_RESOLUTION;
				}
				Set_Direction(INCREASE);
				PWM_PULSE = ADCError;
			}
			else
			{
				if(PWM_RESOLUTION < (-ADCError))
				{
					ADCError = (-PWM_RESOLUTION);
				}
				Set_Direction(DECREASE);
				PWM_PULSE = (-ADCError);
			}
		}
		else
		{
			Complete_Shutdown();
		}

		vTaskDelayUntil(&xLastWakeTime, xFrequency);
	}
  /* USER CODE END Start_EngineControl */
}

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

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
