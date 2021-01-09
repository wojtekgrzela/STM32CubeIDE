/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 * Author			  : Wojciech Grzelinski
 ******************************************************************************
 * Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                             www.st.com/SLA0044
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */     

#include "../lcd_hd44780_i2c/lcd_hd44780_i2c.h"
#include "../VariousFunctions/Functions.h"
#include "../GPS/GPS_Parsing.h"
#include "../EEPROM/EEPROM.h"
#include "Init_Functions.h"

#include "fatfs.h"
#include "sdio.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
uint8_t tuBylemFLAG = 0;
uint8_t TEMPBUFF[100];
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;
extern UART_HandleTypeDef huart1;
extern TIM_HandleTypeDef htim8;
extern SD_HandleTypeDef hsd;

#ifdef RUNTIME_STATS_TIMER_CONFIG
extern TIM_HandleTypeDef htim7;
#endif

/* Handlers for SD Memory Card usage (TASK: My_DumpToSDCard) */
extern FIL SDFile;
extern FATFS SDFatFS;

#ifdef RUNTIME_STATS_QUEUES
#define QUEUE_EEPROM_WRITE_NAME		"EEPROM_WRITE"
#define QUEUE_EEPROM_READ_NAME		"EEPROM_READ"
#define QUEUE_DIAGNOSTIC_DUMP_NAME	"DIAGNOSTIC_DUMP"
#define QUEUE_ERROR_DUMP_NAME		"ERROR_DUMP"
#endif
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* For debugging - task time etc. */
volatile uint32_t Tim7_Counter_100us;

/* For Signaling ENC button and scroll */
volatile ENCButton_struct ENC_button = {0};
volatile int8_t EncoderCounterDiff = 0;

/* For measurements from ADC3 */
volatile uint16_t ADC1Measures[NO_OF_ADC1_MEASURES] = { 0 };
volatile uint16_t ADC3Measures[NO_OF_ADC3_MEASURES] = { 0 };

/* For GPS data, buffering, messages for LCD */
GPS_data_struct GPS =
{ .TimeZoneAdjPoland = 2,/* .homeLatitude = 52.093731, .homeLongitude = 20.661411,*/
		.forLCD.hours.messageReadyFLAG = FALSE,
		.forLCD.minutes.messageReadyFLAG = FALSE,
		.forLCD.seconds.messageReadyFLAG = FALSE,
		.forLCD.clock.messageReadyFLAG = FALSE,
		.forLCD.latitude.messageReadyFLAG = FALSE,
		.forLCD.latitudeIndicator.messageReadyFLAG = FALSE,
		.forLCD.longitude.messageReadyFLAG = FALSE,
		.forLCD.longitudeIndicator.messageReadyFLAG = FALSE,
		.forLCD.status.messageReadyFLAG = FALSE,
		.forLCD.satellitesUsed.messageReadyFLAG = FALSE,
		.forLCD.altitude.messageReadyFLAG = FALSE,
		.forLCD.speed.messageReadyFLAG = FALSE };

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* For LCD parameters and settings */
LCD_parameters_struct LCD =
{ .addressLCD = 0x27, .noOfRowsLCD = NO_OF_ROWS_IN_LCD, .noOfColumnsLCD = NO_OF_COLUMNS_IN_LCD };

Enum_Layer HOME_SCREEN = Desktop_Layer;

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* For measuring Board temperatures with usage of NTC parameters */
NTC_parameters_struct NTC =
		{ .Beta = 4250, .R25 = 100000, .Rgnd = 10000, .T25 = 298, .beta_x_T25 =
				1266500 };

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Counter for CAR EEPROM */
CAR_EEPROM_counters_struct CAR_EEPROM_counters = {0};

/* For CAR EEPROM usage, its addressing and blocking */
EEPROM_parameters_struct EEPROM_car =
{ .EEPROM_hi2c = &hi2c1, .address = EEPROM_CAR_ADDRESS, .pin =
		EEPROM_CAR_BLOCK_PIN, .port = EEPROM_CAR_BLOCK_PORT };

/* CAR SETTINGS and VALUES*/
CAR_mileage_struct CAR_mileage = {0};
LCD_message totalMileageForLCD = {.messageHandler = NULL, .size = 0, .messageReadyFLAG = 0};
LCD_message tripMileageForLCD = {.messageHandler = NULL, .size = 0, .messageReadyFLAG = 0};

waterTempSettings_struct CAR_waterTemp = {0};
float waterTemperatureValue = 0.0;
LCD_message waterTemperatureValueForLCD = {.messageHandler = NULL, .size = 0, .messageReadyFLAG = 0};

oilTempSettings_struct CAR_oilTemp = {0};
float oilTemperatureValue = 0.0;
LCD_message oilTemperatureValueForLCD = {.messageHandler = NULL, .size = 0, .messageReadyFLAG = 0};

oilPressureSettings_struct CAR_oilPressure = {0};
#ifdef OIL_PRESSURE_ANALOG_SENSOR
float oilPressureValue = 0.0;
LCD_message oilPressureValueForLCD = {.messageHandler = NULL, .size = 0, .messageReadyFLAG = 0};
#endif
#ifdef OIL_PRESSURE_BINARY_SENSOR
uint8_t oilPressureValueBinary = NOK;
LCD_message oilPressureValueBinaryForLCD = {.messageHandler = NULL, .size = 0, .messageReadyFLAG = 0};
#endif


batterySettings_struct CAR_mainBattery = {0};
float mainBatteryVoltageValue = 0.0;
LCD_message mainBatteryVoltageValueForLCD = {.messageHandler = NULL, .size = 0, .messageReadyFLAG = 0};

batterySettings_struct CAR_auxiliaryBattery = {0};
float auxiliaryBatteryVoltageValue = 0.0;
LCD_message auxiliaryBatteryVoltageValueForLCD = {.messageHandler = NULL, .size = 0, .messageReadyFLAG = 0};

fuelSettings_struct CAR_fuel = {0};
float fuelLevelValue = 0.0;
LCD_message fuelLevelValueForLCD = {.messageHandler = NULL, .size = 0, .messageReadyFLAG = 0};
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Counters for BOARD EEPROM */
BOARD_EEPROM_counters_struct BOARD_EEPROM_counters = {0};

/* For BOARD EEPROM usage, its addressing and blocking */
EEPROM_parameters_struct EEPROM_board =
{ .EEPROM_hi2c = &hi2c1, .address = EEPROM_BOARD_ADDRESS, .pin =
		EEPROM_BOARD_BLOCK_PIN, .port = EEPROM_BOARD_BLOCK_PORT };

/* CAR SETTINGS and VALUES*/
boardVoltagesSettings_struct BOARD_voltage = {0};
float voltage3V3 = 0.0;
float voltage5V = 0.0;
float voltageIn = 0.0;
LCD_message voltage3V3ForLCD = {.messageHandler = NULL, .size = 0, .messageReadyFLAG = 0};
LCD_message voltage5VForLCD = {.messageHandler = NULL, .size = 0, .messageReadyFLAG = 0};
LCD_message voltageInForLCD = {.messageHandler = NULL, .size = 0, .messageReadyFLAG = 0};

boardTemperaturesSettings_struct BOARD_temperature = {0};
int16_t temperature3V3DCDC = 0;
int16_t temperature5VDCDC = 0;
LCD_message temperature3V3DCDCForLCD = {.messageHandler = NULL, .size = 0, .messageReadyFLAG = 0};
LCD_message temperature5VDCDCForLCD = {.messageHandler = NULL, .size = 0, .messageReadyFLAG = 0};

uint8_t TimeZoneManualAdj = 0;		// for manual adjusting the time zone in clock

buzzerMainSettings_struct BUZZER_settings = {0};
LCDMainSettings_struct LCD_MainSettings = {0};
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */


/* USER CODE END Variables */
osThreadId My_1000ms_TaskHandle;
osThreadId My_500ms_TaskHandle;
osThreadId My_LCD_TaskHandle;
osThreadId My_GPS_TaskHandle;
osThreadId My_EEPROM_TaskHandle;
osThreadId My_DumpToEEPROMHandle;
osThreadId My_DumpToSDCardHandle;
osThreadId My_250ms_TaskHandle;
osThreadId My_50ms_TaskHandle;
osMessageQId Queue_EEPROM_readHandle;
osMessageQId Queue_EEPROM_writeHandle;
osMessageQId Queue_error_snapshot_dumpHandle;
osMessageQId Queue_diagnostic_snapshot_dumpHandle;
osTimerId My_Timer_ENC_ButtonHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartTask1000ms(void const * argument);
void StartTask500ms(void const * argument);
void StartTaskLCD(void const * argument);
void StartTaskGPS(void const * argument);
void StartTaskEEPROM(void const * argument);
void StartTaskDumpToEEPROM(void const * argument);
void StartTaskDumpToSDCard(void const * argument);
void StartTask250ms(void const * argument);
void StartTask50ms(void const * argument);
void ENC_Button_LongPress_Callback(void const * argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* GetTimerTaskMemory prototype (linked to static allocation support) */
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize );

/* Hook prototypes */
void configureTimerForRunTimeStats(void);
unsigned long getRunTimeCounterValue(void);
void vApplicationStackOverflowHook(TaskHandle_t xTask, signed char *pcTaskName);
void vApplicationMallocFailedHook(void);

/* USER CODE BEGIN 1 */
/* Functions needed when configGENERATE_RUN_TIME_STATS is on */
__weak void configureTimerForRunTimeStats(void)
{	/* FOR DEBUGGING */
#ifdef RUNTIME_STATS_TIMER_CONFIG
	HAL_TIM_Base_Start_IT(&htim7);
#endif
}

__weak unsigned long getRunTimeCounterValue(void)
{	/* FOR DEBUGGING */
#ifdef RUNTIME_STATS_TIMER_CONFIG
	return Tim7_Counter_100us;
#endif
}
/* USER CODE END 1 */

/* USER CODE BEGIN 4 */
__weak void vApplicationStackOverflowHook(TaskHandle_t xTask, signed char *pcTaskName)
{
   /* Run time stack overflow checking is performed if
   configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
   called if a stack overflow is detected. */
	while(1)
	{
		HAL_GPIO_TogglePin(LED_3_GPIO_Port, LED_3_Pin);
		HAL_GPIO_TogglePin(LED_4_GPIO_Port, LED_4_Pin);
		HAL_GPIO_TogglePin(LED_5_GPIO_Port, LED_5_Pin);
		HAL_GPIO_TogglePin(LED_6_GPIO_Port, LED_6_Pin);
		HAL_Delay(500);
	}
}
/* USER CODE END 4 */

/* USER CODE BEGIN 5 */
__weak void vApplicationMallocFailedHook(void)
{
   /* vApplicationMallocFailedHook() will only be called if
   configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h. It is a hook
   function that will get called if a call to pvPortMalloc() fails.
   pvPortMalloc() is called internally by the kernel whenever a task, queue,
   timer or semaphore is created. It is also called by various parts of the
   demo application. If heap_1.c or heap_2.c are used, then the size of the
   heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
   FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
   to query the size of free heap space that remains (although it does not
   provide information on how the remaining heap might be fragmented). */
	while(1)
	{
		HAL_GPIO_TogglePin(LED_3_GPIO_Port, LED_3_Pin);
		HAL_GPIO_TogglePin(LED_4_GPIO_Port, LED_4_Pin);
		HAL_GPIO_TogglePin(LED_5_GPIO_Port, LED_5_Pin);
		HAL_GPIO_TogglePin(LED_6_GPIO_Port, LED_6_Pin);
		HAL_Delay(500);
	}
}
/* USER CODE END 5 */

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer,
		StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize)
{
	*ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
	*ppxIdleTaskStackBuffer = &xIdleStack[0];
	*pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
	/* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/* USER CODE BEGIN GET_TIMER_TASK_MEMORY */
static StaticTask_t xTimerTaskTCBBuffer;
static StackType_t xTimerStack[configTIMER_TASK_STACK_DEPTH];
  
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize )  
{
  *ppxTimerTaskTCBBuffer = &xTimerTaskTCBBuffer;
  *ppxTimerTaskStackBuffer = &xTimerStack[0];
  *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
  /* place for user code */
}                   
/* USER CODE END GET_TIMER_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* definition and creation of My_Timer_ENC_Button */
  osTimerDef(My_Timer_ENC_Button, ENC_Button_LongPress_Callback);
  My_Timer_ENC_ButtonHandle = osTimerCreate(osTimer(My_Timer_ENC_Button), osTimerOnce, NULL);

  /* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of Queue_EEPROM_read */
  osMessageQDef(Queue_EEPROM_read, 5, EEPROM_data_struct);
  Queue_EEPROM_readHandle = osMessageCreate(osMessageQ(Queue_EEPROM_read), NULL);

  /* definition and creation of Queue_EEPROM_write */
  osMessageQDef(Queue_EEPROM_write, 5, EEPROM_data_struct);
  Queue_EEPROM_writeHandle = osMessageCreate(osMessageQ(Queue_EEPROM_write), NULL);

  /* definition and creation of Queue_error_snapshot_dump */
  osMessageQDef(Queue_error_snapshot_dump, 5, Error_Code);
  Queue_error_snapshot_dumpHandle = osMessageCreate(osMessageQ(Queue_error_snapshot_dump), NULL);

  /* definition and creation of Queue_diagnostic_snapshot_dump */
  osMessageQDef(Queue_diagnostic_snapshot_dump, 5, Diagnostic_Snapshot_struct);
  Queue_diagnostic_snapshot_dumpHandle = osMessageCreate(osMessageQ(Queue_diagnostic_snapshot_dump), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of My_1000ms_Task */
  osThreadDef(My_1000ms_Task, StartTask1000ms, osPriorityNormal, 0, 256);
  My_1000ms_TaskHandle = osThreadCreate(osThread(My_1000ms_Task), NULL);

  /* definition and creation of My_500ms_Task */
  osThreadDef(My_500ms_Task, StartTask500ms, osPriorityNormal, 0, 128);
  My_500ms_TaskHandle = osThreadCreate(osThread(My_500ms_Task), NULL);

  /* definition and creation of My_LCD_Task */
  osThreadDef(My_LCD_Task, StartTaskLCD, osPriorityNormal, 0, 1024);
  My_LCD_TaskHandle = osThreadCreate(osThread(My_LCD_Task), NULL);

  /* definition and creation of My_GPS_Task */
  osThreadDef(My_GPS_Task, StartTaskGPS, osPriorityNormal, 0, 128);
  My_GPS_TaskHandle = osThreadCreate(osThread(My_GPS_Task), NULL);

  /* definition and creation of My_EEPROM_Task */
  osThreadDef(My_EEPROM_Task, StartTaskEEPROM, osPriorityHigh, 0, 128);
  My_EEPROM_TaskHandle = osThreadCreate(osThread(My_EEPROM_Task), NULL);

  /* definition and creation of My_DumpToEEPROM */
  osThreadDef(My_DumpToEEPROM, StartTaskDumpToEEPROM, osPriorityBelowNormal, 0, 128);
  My_DumpToEEPROMHandle = osThreadCreate(osThread(My_DumpToEEPROM), NULL);

  /* definition and creation of My_DumpToSDCard */
  osThreadDef(My_DumpToSDCard, StartTaskDumpToSDCard, osPriorityBelowNormal, 0, 1024);
  My_DumpToSDCardHandle = osThreadCreate(osThread(My_DumpToSDCard), NULL);

  /* definition and creation of My_250ms_Task */
  osThreadDef(My_250ms_Task, StartTask250ms, osPriorityIdle, 0, 512);
  My_250ms_TaskHandle = osThreadCreate(osThread(My_250ms_Task), NULL);

  /* definition and creation of My_50ms_Task */
  osThreadDef(My_50ms_Task, StartTask50ms, osPriorityIdle, 0, 128);
  My_50ms_TaskHandle = osThreadCreate(osThread(My_50ms_Task), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartTask1000ms */
/**
 * @brief  Function implementing the My_1000ms_Task thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTask1000ms */
void StartTask1000ms(void const * argument)
{
  /* init code for USB_DEVICE */
//  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartTask1000ms */

	TickType_t xLastWakeTime;
	const TickType_t xFrequency = MY_TASK_1000_MS_TIME_PERIOD;
	Error_Code error = NO_ERROR;

	static uint8_t totalMileageMessage[7] = "";
	static uint8_t tripMileageMessage[8] = "";
	totalMileageForLCD.messageHandler = totalMileageMessage;
	tripMileageForLCD.messageHandler = tripMileageMessage;

	static uint8_t temperature3V3DCDC_message[6] = {SPACE_IN_ASCII};
	static uint8_t temperature5VDCDC_message[6] = {SPACE_IN_ASCII};
	temperature3V3DCDCForLCD.messageHandler = temperature3V3DCDC_message;
	temperature5VDCDCForLCD.messageHandler = temperature5VDCDC_message;

	static uint8_t voltage3V3_message[5] = {SPACE_IN_ASCII};
	static uint8_t voltage5V_message[5] = {SPACE_IN_ASCII};
	static uint8_t voltageIn_message[6] = {SPACE_IN_ASCII};
	voltage3V3ForLCD.messageHandler = voltage3V3_message;
	voltage5VForLCD.messageHandler = voltage5V_message;
	voltageInForLCD.messageHandler = voltageIn_message;

	xLastWakeTime = xTaskGetTickCount();

	/* Infinite loop */
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
	for (;;)
	{
		error = calculate_NTC_temperature(&temperature5VDCDC, ADC3Measures[3]/*NTC near 5V stabilizer*/, &NTC);

		if(NO_ERROR != error)
		{
			my_error_handler(error);
		}
		else
		{
			temperature5VDCDCForLCD.messageReadyFLAG = FALSE;
			snprintf((char*)temperature5VDCDCForLCD.messageHandler, 6, "%" PRIi16 "%cC", temperature5VDCDC, DEGREE_SYMBOL_LCD);
			temperature5VDCDCForLCD.size = strlen((char*)temperature5VDCDCForLCD.messageHandler);
			temperature5VDCDCForLCD.messageReadyFLAG = TRUE;
		}

		error = calculate_NTC_temperature(&temperature3V3DCDC, ADC3Measures[2]/*NTC near DC/DC*/, &NTC);

		if(NO_ERROR != error)
		{
			my_error_handler(error);
		}
		else
		{
			temperature3V3DCDCForLCD.messageReadyFLAG = FALSE;
			snprintf((char*)temperature3V3DCDCForLCD.messageHandler, 6, "%" PRIi16 "%cC", temperature3V3DCDC, DEGREE_SYMBOL_LCD);
			temperature3V3DCDCForLCD.size = strlen((char*)temperature3V3DCDCForLCD.messageHandler);
			temperature3V3DCDCForLCD.messageReadyFLAG = TRUE;
		}

		error = calculate_voltage(&voltageIn, ADC3Measures[0]/*Vin*/, MEASURE_VIN_VOLTAGE_DIVIDER);

		if(NO_ERROR != error)
		{
			my_error_handler(error);
		}
		else
		{
			voltageInForLCD.messageReadyFLAG = FALSE;
			snprintf((char*)voltageInForLCD.messageHandler, 7, "%01" PRIu16 ".%02" PRIu16 "V", (uint16_t)voltageIn, (uint16_t)(voltageIn*100)%100);
			voltageInForLCD.size = strlen((char*)voltageInForLCD.messageHandler);
			voltageInForLCD.messageReadyFLAG = TRUE;
		}

		error = calculate_voltage(&voltage5V, ADC3Measures[1]/*5V*/, MEASURE_5V_VOLTAGE_DIVIDER);

		if(NO_ERROR != error)
		{
			my_error_handler(error);
		}
		else
		{
			voltage5VForLCD.messageReadyFLAG = FALSE;
			snprintf((char*)voltage5VForLCD.messageHandler, 6, "%01" PRIu16 ".%02" PRIi16 "V", (uint16_t)voltage5V, (uint16_t)(voltage5V*100)%100);
			voltage5VForLCD.size = strlen((char*)voltage5VForLCD.messageHandler);
			voltage5VForLCD.messageReadyFLAG = TRUE;;
		}

		totalMileageForLCD.messageReadyFLAG = FALSE;
		snprintf((char*)totalMileageForLCD.messageHandler, 6, "%01" PRIu32, CAR_mileage.totalMileage);
		totalMileageForLCD.size = strlen((char*)totalMileageForLCD.messageHandler);
		totalMileageForLCD.messageReadyFLAG = TRUE;

		tripMileageForLCD.messageReadyFLAG = FALSE;
		snprintf((char*)tripMileageForLCD.messageHandler, 5, "%01" PRIu32 ".%01" PRIu32, (CAR_mileage.tripMileage/10), (CAR_mileage.tripMileage)%10);
		tripMileageForLCD.size = strlen((char*)tripMileageForLCD.messageHandler);
		tripMileageForLCD.messageReadyFLAG = FALSE;

		vTaskDelayUntil(&xLastWakeTime, xFrequency);
	}
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

  /* USER CODE END StartTask1000ms */
}

/* USER CODE BEGIN Header_StartTask500ms */

/**
 * @brief Function implementing the My_500ms_Task thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTask500ms */
void StartTask500ms(void const * argument)
{
  /* USER CODE BEGIN StartTask500ms */

	TickType_t xLastWakeTime;
	const TickType_t xFrequency = MY_TASK_500_MS_TIME_PERIOD;

	xLastWakeTime = xTaskGetTickCount();

	/* Infinite loop */
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
	for (;;)
	{
		HAL_GPIO_TogglePin(LED_2_GPIO_Port, LED_2_Pin);

		vTaskDelayUntil(&xLastWakeTime, xFrequency);
	}
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

  /* USER CODE END StartTask500ms */
}

/* USER CODE BEGIN Header_StartTaskLCD */
/**
 * @brief Function implementing the My_LCD_Task thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTaskLCD */
void StartTaskLCD(void const * argument)
{
  /* USER CODE BEGIN StartTaskLCD */

	TickType_t xLastWakeTime;
	const TickType_t xFrequency = MY_LCD_TASK_TIME_PERIOD;
	Error_Code error = NO_ERROR;

	/* No better option for making a degree symbol was found so far */
	uint8_t degreeSymbolCharacter[2] = "";
	snprintf((char*)degreeSymbolCharacter, 2, "%c", DEGREE_SYMBOL_LCD);
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

	int32_t submenuIterator = 0;

	Enum_Layer currentLayer = HOME_SCREEN;

	/******************************************************/
	/* For reading the data from EEPROM and displaying it */
	DiagnosticDataToEEPROM_struct DiagnosticDataRead =
	{ .DiagnosticDataForEEPROM =
		{ .EEPROMParameters = &EEPROM_car,
		.data = DiagnosticDataRead.data,
		.size = MAX_DIAGNOSTIC_SNAPSHOT_SIZE,
		.memAddress = 0,
		.memAddressSize = 2 },
	.diag_mess_from_queue =
		{ .snapshotIdentificator = DIAGNOSTICS_OK,
		.value = 0 } };

	ErrorDataToEEPROM_struct ErrorDataRead =
	{ .ErrorDataForEEPROM =
		{ .data = ErrorDataRead.data,
		.size = MAX_ERROR_SNAPSHOT_SIZE,
		.memAddress = 0,
		.memAddressSize = 2	},
	.error_mess_from_queue = NO_ERROR };
	/******************************************************/

	LCDBoard LCD_MainMenu =
			{ .name = "Main Menu",
			.nameActualSize = sizeof("Main Menu")-1,
			.layer = MainMenu_Layer,
			.actionForEnter = GoInto_EnterAction,
			.screenType = ScrollList_ScreenType,
			.layerPrevious = HOME_SCREEN };

	LCDBoard LCD_YesNo =
			{ .name = "Are you sure?",
			.nameActualSize = sizeof("Are you sure?")-1,
			.layer = YesNo_Layer,
			.actionForEnter = Done_EnterAction,
			.screenType = YesNo_ScreenType,
			.layerPrevious = MainMenu_Layer /* Should be last one opened */ };

	LCDBoard LCD_Ctrl =
			{ .name = "Ctrl",
			.nameActualSize = sizeof("Ctrl")-1,
			.layer = Ctrl_Layer,
			.actionForEnter = Ctrl_EnterAction,
			.screenType = Ctrl_ScreenType,
			.layerPrevious = MainMenu_Layer /* Should be last one opened */ };

	LCDBoard LCD_Alarm =
			{ .name = "!!! ALARM !!!",
			.nameActualSize = sizeof("!!! ALARM !!!")-1,
			.layer = Alarm_Layer,
			.actionForEnter = Alarm_EnterAction,
			.screenType = Alarm_ScreenType,
			.layerPrevious = MainMenu_Layer /* Should be last one opened */ };


	LCDBoard LCD_MainMenuList[ ] = {
			{ .name = "Desktop",
			.nameActualSize = sizeof("Desktop")-1,
			.layer = Desktop_Layer,
			.actionForEnter = None_EnterAction,
			.screenType = OneScreen_ScreenType,
			.layerPrevious = MainMenu_Layer },

			{ .name = "GPS",
			.nameActualSize = sizeof("GPS")-1,
			.layer = GPS_Layer,
			.actionForEnter = None_EnterAction,
			.screenType = OneScreen_ScreenType,
			.layerPrevious = MainMenu_Layer },

			{ .name = "Car information",
			.nameActualSize = sizeof("Car information")-1,
			.layer = CarInfo_Layer,
			.actionForEnter = None_EnterAction,
			.screenType = OneScreen_ScreenType,
			.layerPrevious = MainMenu_Layer },

			{ .name = "Jarvis information",
			.nameActualSize = sizeof("Jarvis information")-1,
			.layer = JarvisInfo_Layer,
			.actionForEnter = None_EnterAction,
			.screenType = OneScreen_ScreenType,
			.layerPrevious = MainMenu_Layer },

			{ .name = "Last 3 diag. snaps.",
			.nameActualSize = sizeof("Last 3 diag. snaps.")-1,
			.layer = Last3Diag_Layer,
			.actionForEnter = None_EnterAction,
			.screenType = OneScreen_ScreenType,
			.layerPrevious = MainMenu_Layer },

			{ .name = "Last 3 error snaps.",
			.nameActualSize = sizeof("Last 3 error snaps.")-1,
			.layer = Last3Err_Layer,
			.actionForEnter = None_EnterAction,
			.screenType = OneScreen_ScreenType,
			.layerPrevious = MainMenu_Layer },

			{ .name = "Car settings",
			.nameActualSize = sizeof("Car settings")-1,
			.layer = CarSettings_Layer,
			.actionForEnter = GoInto_EnterAction,
			.screenType = ScrollList_ScreenType,
			.layerPrevious = MainMenu_Layer },

			{ .name = "Board settings",
			.nameActualSize = sizeof("Board settings")-1,
			.layer = BoardSettings_Layer,
			.actionForEnter = GoInto_EnterAction,
			.screenType = ScrollList_ScreenType,
			.layerPrevious = MainMenu_Layer },
	};
	uint8_t LCD_MainMenuList_SIZE = sizeof(LCD_MainMenuList)/sizeof(LCDBoard);

	LCDBoard LCD_CarSettingsList[ ] = {
			{ .name = "Clear diag. snaps.",
			.nameActualSize = sizeof("Clear diag. snaps.")-1,
			.layer = ClearDiagnosticSnapshots,
			.actionForEnter = YesNo_EnterAction,
			.screenType = No_ScreenType,
			.layerPrevious = CarSettings_Layer },

			{ .name = "Clear trip mileage",
			.nameActualSize = sizeof("Clear trip mileage")-1,
			.layer = ClearTripMileage,
			.actionForEnter = YesNo_EnterAction,
			.screenType = No_ScreenType,
			.layerPrevious = CarSettings_Layer },

			{ .name = "Water T. sett.",
			.nameActualSize = sizeof("Water T. sett.")-1,
			.layer = WaterSettings_Layer,
			.actionForEnter = GoInto_EnterAction,
			.screenType = ScrollList_ScreenType,
			.layerPrevious = CarSettings_Layer },

			{ .name = "Oil temp. sett.",
			.nameActualSize = sizeof("Oil temp. sett.")-1,
			.layer = OilTempSettings_Layer,
			.actionForEnter = GoInto_EnterAction,
			.screenType = ScrollList_ScreenType,
			.layerPrevious = CarSettings_Layer },

			{ .name = "Oil press. sett.",
			.nameActualSize = sizeof("Oil press. sett.")-1,
			.layer = OilPressureSettings_Layer,
			.actionForEnter = GoInto_EnterAction,
			.screenType = ScrollList_ScreenType,
			.layerPrevious = CarSettings_Layer },

			{ .name = "Fuel settings",
			.nameActualSize = sizeof("Fuel settings")-1,
			.layer = FuelSettings_Layer,
			.actionForEnter = GoInto_EnterAction,
			.screenType = ScrollList_ScreenType,
			.layerPrevious = CarSettings_Layer },

			{ .name = "Main batt. sett.",
			.nameActualSize = sizeof("Main batt. sett.")-1,
			.layer = MainBatterySettings_Layer,
			.actionForEnter = GoInto_EnterAction,
			.screenType = ScrollList_ScreenType,
			.layerPrevious = CarSettings_Layer },

			{ .name = "Aux batt. sett.",
			.nameActualSize = sizeof("Aux Batt. sett.")-1,
			.layer = AuxBatterySettings_Layer,
			.actionForEnter = GoInto_EnterAction,
			.screenType = ScrollList_ScreenType,
			.layerPrevious = CarSettings_Layer }
	};
	uint8_t LCD_CarSettingsList_SIZE = sizeof(LCD_CarSettingsList)/sizeof(LCDBoard);

	LCDBoard LCD_BoardSettingsList[ ] = {
			{ .name = "Clear err. snaps.",
			.nameActualSize = sizeof("Clear err. snaps.")-1,
			.layer = ClearErrorsSnapshots,
			.actionForEnter = YesNo_EnterAction,
			.screenType = No_ScreenType,
			.layerPrevious = BoardSettings_Layer },

			{ .name = "Adj. time PL",
			.nameActualSize = sizeof("Adj. time PL")-1,
			.layer = AdjTimePoland,
			.actionForEnter = WinterSummer_EnterAction,
			.screenType = No_ScreenType,
			.layerPrevious = BoardSettings_Layer },

			{ .name = "Adj. time zone",
			.nameActualSize = sizeof("Adj. time zone")-1,
			.layer = AdjTimeZone,
			.actionForEnter = Ctrl_EnterAction,
			.screenType = No_ScreenType,
			.layerPrevious = BoardSettings_Layer },

			{ .name = "Inter. V. sett.",
			.nameActualSize = sizeof("Inter. V. sett.")-1,
			.layer = InterVoltSettings_Layer,
			.actionForEnter = GoInto_EnterAction,
			.screenType = ScrollList_ScreenType,
			.layerPrevious = BoardSettings_Layer },

			{ .name = "Inter. T. sett.",
			.nameActualSize = sizeof("Inter. T. sett.")-1,
			.layer = InterTempSettings_Layer,
			.actionForEnter = GoInto_EnterAction,
			.screenType = ScrollList_ScreenType,
			.layerPrevious = BoardSettings_Layer },

			{ .name = "Buzzer settings",
			.nameActualSize = sizeof("Buzzer settings")-1,
			.layer = BuzzerSettings_Layer,
			.actionForEnter = GoInto_EnterAction,
			.screenType = ScrollList_ScreenType,
			.layerPrevious = BoardSettings_Layer },

			{ .name = "LCD settings",
			.nameActualSize = sizeof("LCD settings")-1,
			.layer = LCDSettings_Layer,
			.actionForEnter = GoInto_EnterAction,
			.screenType = ScrollList_ScreenType,
			.layerPrevious = BoardSettings_Layer }
	};
	uint8_t LCD_BoardSettingsList_SIZE = sizeof(LCD_BoardSettingsList)/sizeof(LCDBoard);

	LCDBoard LCD_Car_WaterTempSettingsList[ ] = {
			{ .name = "Water H. T. warn.",
			.nameActualSize = sizeof("Water H. T. warn.")-1,
			.layer = WaterSettings_Layer,
			.actionForEnter = Ctrl_EnterAction,
			.screenType = No_ScreenType,
			.layerPrevious = WaterSettings_Layer },

			{ .name = "Water H. T. alarm",
			.nameActualSize = sizeof("Water H. T. alarm")-1,
			.layer = WaterSettings_Layer,
			.actionForEnter = Ctrl_EnterAction,
			.screenType = No_ScreenType,
			.layerPrevious = WaterSettings_Layer },

			{ .name = "Water H. T. FanOn",
			.nameActualSize = sizeof("Water H. T. FanOn")-1,
			.layer = WaterSettings_Layer,
			.actionForEnter = Ctrl_EnterAction,
			.screenType = No_ScreenType,
			.layerPrevious = WaterSettings_Layer },

			{ .name = "Water H. T. FanOff",
			.nameActualSize = sizeof("Water H. T. FanOff")-1,
			.layer = WaterSettings_Layer,
			.actionForEnter = Ctrl_EnterAction,
			.screenType = No_ScreenType,
			.layerPrevious = WaterSettings_Layer },

			{ .name = "Water T. warn.",
			.nameActualSize = sizeof("Water T. warn.")-1,
			.layer = WaterSettings_Layer,
			.actionForEnter = OnOff_EnterAction,
			.screenType = No_ScreenType,
			.layerPrevious = WaterSettings_Layer },

			{ .name = "Water T. alarm",
			.nameActualSize = sizeof("Water T. alarm")-1,
			.layer = WaterSettings_Layer,
			.actionForEnter = OnOff_EnterAction,
			.screenType = No_ScreenType,
			.layerPrevious = WaterSettings_Layer },

			{ .name = "Water T. Fan ctrl",
			.nameActualSize = sizeof("Water T. Fan ctrl")-1,
			.layer = WaterSettings_Layer,
			.actionForEnter = OnOff_EnterAction,
			.screenType = No_ScreenType,
			.layerPrevious = WaterSettings_Layer },

			{ .name = "Wat. T. warn. Buzz.",
			.nameActualSize = sizeof("Wat. T. warn. Buzz.")-1,
			.layer = WaterSettings_Layer,
			.actionForEnter = OnOff_EnterAction,
			.screenType = No_ScreenType,
			.layerPrevious = WaterSettings_Layer },

			{ .name = "Wat. T. alarm Buzz.",
			.nameActualSize = sizeof("Wat. T. alarm Buzz.")-1,
			.layer = WaterSettings_Layer,
			.actionForEnter = OnOff_EnterAction,
			.screenType = No_ScreenType,
			.layerPrevious = WaterSettings_Layer },

			{ .name = "Wat. T. warn. snap.",
			.nameActualSize = sizeof("Wat. T. warn. snap.")-1,
			.layer = WaterSettings_Layer,
			.actionForEnter = OnOff_EnterAction,
			.screenType = No_ScreenType,
			.layerPrevious = WaterSettings_Layer },

			{ .name = "Wat. T. alarm snap.",
			.nameActualSize = sizeof("Wat. T. alarm snap.")-1,
			.layer = WaterSettings_Layer,
			.actionForEnter = OnOff_EnterAction,
			.screenType = No_ScreenType,
			.layerPrevious = WaterSettings_Layer }
	};
	uint8_t LCD_Car_WaterTempSettingsList_SIZE = sizeof(LCD_Car_WaterTempSettingsList)/sizeof(LCDBoard);

	LCDBoard LCD_Car_OilTempSettingsList[ ] = {
			{ .name = "Oil H. T. warning",
			.nameActualSize = sizeof("Oil H. T. warning")-1,
			.layer = OilTempSettings_Layer,
			.actionForEnter = Ctrl_EnterAction,
			.screenType = No_ScreenType,
			.layerPrevious = OilTempSettings_Layer },

			{ .name = "Oil H. T. alarm",
			.nameActualSize = sizeof("Oil H. T. alarm")-1,
			.layer = OilTempSettings_Layer,
			.actionForEnter = Ctrl_EnterAction,
			.screenType = No_ScreenType,
			.layerPrevious = OilTempSettings_Layer },

			{ .name = "Oil T. warning",
			.nameActualSize = sizeof("Oil T. warning")-1,
			.layer = OilTempSettings_Layer,
			.actionForEnter = OnOff_EnterAction,
			.screenType = No_ScreenType,
			.layerPrevious = OilTempSettings_Layer },

			{ .name = "Oil T. alarm",
			.nameActualSize = sizeof("Oil T. alarm")-1,
			.layer = OilTempSettings_Layer,
			.actionForEnter = OnOff_EnterAction,
			.screenType = No_ScreenType,
			.layerPrevious = OilTempSettings_Layer },

			{ .name = "Oil T. warn. Buzz.",
			.nameActualSize = sizeof("Oil T. warn. Buzz.")-1,
			.layer = OilTempSettings_Layer,
			.actionForEnter = OnOff_EnterAction,
			.screenType = No_ScreenType,
			.layerPrevious = OilTempSettings_Layer },

			{ .name = "Oil T. alarm Buzz.",
			.nameActualSize = sizeof("Oil T. alarm Buzz.")-1,
			.layer = OilTempSettings_Layer,
			.actionForEnter = OnOff_EnterAction,
			.screenType = No_ScreenType,
			.layerPrevious = OilTempSettings_Layer },

			{ .name = "Oil T. warn. snap.",
			.nameActualSize = sizeof("Oil T. warn. snap.")-1,
			.layer = OilTempSettings_Layer,
			.actionForEnter = OnOff_EnterAction,
			.screenType = No_ScreenType,
			.layerPrevious = OilTempSettings_Layer },

			{ .name = "Oil T. alarm snap.",
			.nameActualSize = sizeof("Oil T. alarm snap.")-1,
			.layer = OilTempSettings_Layer,
			.actionForEnter = OnOff_EnterAction,
			.screenType = No_ScreenType,
			.layerPrevious = OilTempSettings_Layer }
	};
	uint8_t LCD_Car_OilTempSettingsList_SIZE = sizeof(LCD_Car_OilTempSettingsList)/sizeof(LCDBoard);

	LCDBoard LCD_Car_OilPressureSettingsList[ ] = {
			{ .name = "Oil H. P. alarm",
			.nameActualSize = sizeof("Oil H. P. alarm")-1,
			.layer = OilPressureSettings_Layer,
			.actionForEnter = Ctrl_EnterAction,
			.screenType = No_ScreenType,
			.layerPrevious = OilPressureSettings_Layer },

			{ .name = "Oil L. P. alarm",
			.nameActualSize = sizeof("Oil L. P. alarm")-1,
			.layer = OilPressureSettings_Layer,
			.actionForEnter = Ctrl_EnterAction,
			.screenType = No_ScreenType,
			.layerPrevious = OilPressureSettings_Layer },

			{ .name = "Oil P. analog",
			.nameActualSize = sizeof("Oil P. analog")-1,
			.layer = OilPressureSettings_Layer,
			.actionForEnter = OnOff_EnterAction,
			.screenType = No_ScreenType,
			.layerPrevious = OilPressureSettings_Layer },

			{ .name = "Oil H. P. alarm",
			.nameActualSize = sizeof("Oil H. P. alarm")-1,
			.layer = OilPressureSettings_Layer,
			.actionForEnter = OnOff_EnterAction,
			.screenType = No_ScreenType,
			.layerPrevious = OilPressureSettings_Layer },

			{ .name = "Oil L. P. alarm",
			.nameActualSize = sizeof("Oil L. P. alarm")-1,
			.layer = OilPressureSettings_Layer,
			.actionForEnter = OnOff_EnterAction,
			.screenType = No_ScreenType,
			.layerPrevious = OilPressureSettings_Layer },

			{ .name = "Oil P. alarm Buzz.",
			.nameActualSize = sizeof("Oil P. alarm Buzz.")-1,
			.layer = OilPressureSettings_Layer,
			.actionForEnter = OnOff_EnterAction,
			.screenType = No_ScreenType,
			.layerPrevious = OilPressureSettings_Layer },

			{ .name = "Oil P. alarm snap.",
			.nameActualSize = sizeof("Oil P. alarm snap.")-1,
			.layer = OilPressureSettings_Layer,
			.actionForEnter = OnOff_EnterAction,
			.screenType = No_ScreenType,
			.layerPrevious = OilPressureSettings_Layer }
	};
	uint8_t LCD_Car_OilPressureSettingsList_SIZE = sizeof(LCD_Car_OilPressureSettingsList)/sizeof(LCDBoard);

	LCDBoard LCD_Car_FuelSettingsList[ ] = {
			{ .name = "Fuel Low warn. thr.",
			.nameActualSize = sizeof("Fuel Low warn. thr.")-1,
			.layer = FuelSettings_Layer,
			.actionForEnter = Ctrl_EnterAction,
			.screenType = No_ScreenType,
			.layerPrevious = FuelSettings_Layer },

			{ .name = "Fuel Low warn. ON/OFF",
			.nameActualSize = sizeof("Fuel Low warn. ON/OFF")-1,
			.layer = FuelSettings_Layer,
			.actionForEnter = OnOff_EnterAction,
			.screenType = No_ScreenType,
			.layerPrevious = FuelSettings_Layer },

			{ .name = "Fuel Low Buzzer",
			.nameActualSize = sizeof("Fuel Low Buzzer")-1,
			.layer = FuelSettings_Layer,
			.actionForEnter = OnOff_EnterAction,
			.screenType = No_ScreenType,
			.layerPrevious = FuelSettings_Layer }
	};
	uint8_t LCD_Car_FuelSettingsList_SIZE = sizeof(LCD_Car_FuelSettingsList)/sizeof(LCDBoard);

	LCDBoard LCD_Car_MainBatterySettingsList[ ] = {
			{ .name = "Low V. thres.",
			.nameActualSize = sizeof("Low V. thres.")-1,
			.layer = MainBatterySettings_Layer,
			.actionForEnter = Ctrl_EnterAction,
			.screenType = No_ScreenType,
			.layerPrevious = MainBatterySettings_Layer },

			{ .name = "High V. thres.",
			.nameActualSize = sizeof("High V. thres.")-1,
			.layer = MainBatterySettings_Layer,
			.actionForEnter = Ctrl_EnterAction,
			.screenType = No_ScreenType,
			.layerPrevious = MainBatterySettings_Layer },

			{ .name = "Low V. alarm",
			.nameActualSize = sizeof("Low V. alarm")-1,
			.layer = MainBatterySettings_Layer,
			.actionForEnter = OnOff_EnterAction,
			.screenType = No_ScreenType,
			.layerPrevious = MainBatterySettings_Layer },

			{ .name = "High V. alarm",
			.nameActualSize = sizeof("High V. alarm")-1,
			.layer = MainBatterySettings_Layer,
			.actionForEnter = OnOff_EnterAction,
			.screenType = No_ScreenType,
			.layerPrevious = MainBatterySettings_Layer },

			{ .name = "Volt. alarm Buzz.",
			.nameActualSize = sizeof("Volt. alarm Buzz.")-1,
			.layer = MainBatterySettings_Layer,
			.actionForEnter = OnOff_EnterAction,
			.screenType = No_ScreenType,
			.layerPrevious = MainBatterySettings_Layer },

			{ .name = "L. V. alarm snap.",
			.nameActualSize = sizeof("L. V. alarm snap.")-1,
			.layer = MainBatterySettings_Layer,
			.actionForEnter = OnOff_EnterAction,
			.screenType = No_ScreenType,
			.layerPrevious = MainBatterySettings_Layer },

			{ .name = "H. V. alarm snap.",
			.nameActualSize = sizeof("H. V. alarm snap.")-1,
			.layer = MainBatterySettings_Layer,
			.actionForEnter = OnOff_EnterAction,
			.screenType = No_ScreenType,
			.layerPrevious = MainBatterySettings_Layer }
	};
	uint8_t LCD_Car_MainBatterySettingsList_SIZE = sizeof(LCD_Car_MainBatterySettingsList)/sizeof(LCDBoard);

	LCDBoard LCD_Car_AuxBatterySettingsList[ ] = {
			{ .name = "Low V. thres.",
			.nameActualSize = sizeof("Low V. thres.")-1,
			.layer = AuxBatterySettings_Layer,
			.actionForEnter = Ctrl_EnterAction,
			.screenType = No_ScreenType,
			.layerPrevious = AuxBatterySettings_Layer },

			{ .name = "High V. thres.",
			.nameActualSize = sizeof("High V. thres.")-1,
			.layer = AuxBatterySettings_Layer,
			.actionForEnter = Ctrl_EnterAction,
			.screenType = No_ScreenType,
			.layerPrevious = AuxBatterySettings_Layer },

			{ .name = "Low V. alarm",
			.nameActualSize = sizeof("Low V. alarm")-1,
			.layer = AuxBatterySettings_Layer,
			.actionForEnter = OnOff_EnterAction,
			.screenType = No_ScreenType,
			.layerPrevious = AuxBatterySettings_Layer },

			{ .name = "High V. alarm",
			.nameActualSize = sizeof("High V. alarm")-1,
			.layer = AuxBatterySettings_Layer,
			.actionForEnter = OnOff_EnterAction,
			.screenType = No_ScreenType,
			.layerPrevious = AuxBatterySettings_Layer },

			{ .name = "Volt. alarm Buzz.",
			.nameActualSize = sizeof("Volt. alarm Buzz.")-1,
			.layer = AuxBatterySettings_Layer,
			.actionForEnter = OnOff_EnterAction,
			.screenType = No_ScreenType,
			.layerPrevious = AuxBatterySettings_Layer },

			{ .name = "L. V. alarm snap.",
			.nameActualSize = sizeof("L. V. alarm snap.")-1,
			.layer = AuxBatterySettings_Layer,
			.actionForEnter = OnOff_EnterAction,
			.screenType = No_ScreenType,
			.layerPrevious = AuxBatterySettings_Layer },

			{ .name = "H. V. alarm snap.",
			.nameActualSize = sizeof("H. V. alarm snap.")-1,
			.layer = AuxBatterySettings_Layer,
			.actionForEnter = OnOff_EnterAction,
			.screenType = No_ScreenType,
			.layerPrevious = AuxBatterySettings_Layer }
	};
	uint8_t LCD_Car_AuxBatterySettingsList_SIZE = sizeof(LCD_Car_AuxBatterySettingsList)/sizeof(LCDBoard);

	LCDBoard LCD_Board_InternalVoltSettingsList[ ] = {
			{ .name = "5V Low thres.",
			.nameActualSize = sizeof("5V Low thres.")-1,
			.layer = InterVoltSettings_Layer,
			.actionForEnter = Ctrl_EnterAction,
			.screenType = No_ScreenType,
			.layerPrevious = InterVoltSettings_Layer },

			{ .name = "5V High thres.",
			.nameActualSize = sizeof("5V High thres.")-1,
			.layer = InterVoltSettings_Layer,
			.actionForEnter = Ctrl_EnterAction,
			.screenType = No_ScreenType,
			.layerPrevious = InterVoltSettings_Layer },

			{ .name = "3V3 Low thres.",
			.nameActualSize = sizeof("3V3 Low thres.")-1,
			.layer = InterVoltSettings_Layer,
			.actionForEnter = Ctrl_EnterAction,
			.screenType = No_ScreenType,
			.layerPrevious = InterVoltSettings_Layer },

			{ .name = "3V3 High thres.",
			.nameActualSize = sizeof("3V3 High thres.")-1,
			.layer = InterVoltSettings_Layer,
			.actionForEnter = Ctrl_EnterAction,
			.screenType = No_ScreenType,
			.layerPrevious = InterVoltSettings_Layer },

			{ .name = "VIn Low thres.",
			.nameActualSize = sizeof("VIn Low thres.")-1,
			.layer = InterVoltSettings_Layer,
			.actionForEnter = Ctrl_EnterAction,
			.screenType = No_ScreenType,
			.layerPrevious = InterVoltSettings_Layer }
	};
	uint8_t LCD_Board_InternalVoltSettingsList_SIZE = sizeof(LCD_Board_InternalVoltSettingsList)/sizeof(LCDBoard);

	LCDBoard LCD_Board_InternalTempSettingsList[ ] = {
			{ .name = "3V3 DCDC T. thres.",
			.nameActualSize = sizeof("3V3 DCDC T. thres.")-1,
			.layer = InterTempSettings_Layer,
			.actionForEnter = Ctrl_EnterAction,
			.screenType = No_ScreenType,
			.layerPrevious = InterTempSettings_Layer },

			{ .name = "5V DCDC T. thres.",
			.nameActualSize = sizeof("5V DCDC T. thres.")-1,
			.layer = InterTempSettings_Layer,
			.actionForEnter = Ctrl_EnterAction,
			.screenType = No_ScreenType,
			.layerPrevious = InterTempSettings_Layer }
	};
	uint8_t LCD_Board_InternalTempSettingsList_SIZE = sizeof(LCD_Board_InternalTempSettingsList)/sizeof(LCDBoard);

	LCDBoard LCD_Board_BuzzerSettingsList[ ] = {
			{ .name = "Buzzer Main",
			.nameActualSize = sizeof("Buzzer Main")-1,
			.layer = BuzzerSettings_Layer,
			.actionForEnter = OnOff_EnterAction,
			.screenType = No_ScreenType,
			.layerPrevious = BuzzerSettings_Layer },

			{ .name = "Buzzer Main alarm",
			.nameActualSize = sizeof("Buzzer Main alarm")-1,
			.layer = BuzzerSettings_Layer,
			.actionForEnter = OnOff_EnterAction,
			.screenType = No_ScreenType,
			.layerPrevious = BuzzerSettings_Layer },

			{ .name = "Buzzer Main button",
			.nameActualSize = sizeof("Buzzer Main button")-1,
			.layer = BuzzerSettings_Layer,
			.actionForEnter = OnOff_EnterAction,
			.screenType = No_ScreenType,
			.layerPrevious = BuzzerSettings_Layer },

			{ .name = "Buzzer Short press",
			.nameActualSize = sizeof("Buzzer Short press")-1,
			.layer = BuzzerSettings_Layer,
			.actionForEnter = OnOff_EnterAction,
			.screenType = No_ScreenType,
			.layerPrevious = BuzzerSettings_Layer },

			{ .name = "Buzzer Long press",
			.nameActualSize = sizeof("Buzzer Long press")-1,
			.layer = BuzzerSettings_Layer,
			.actionForEnter = OnOff_EnterAction,
			.screenType = No_ScreenType,
			.layerPrevious = BuzzerSettings_Layer }
	};
	uint8_t LCD_Board_BuzzerSettingsList_SIZE = sizeof(LCD_Board_BuzzerSettingsList)/sizeof(LCDBoard);

	LCDBoard LCD_Board_LCDSettingsList[ ] = {
			{ .name = "Backlight level",
			.nameActualSize = sizeof("Backlight level")-1,
			.layer = LCDSettings_Layer,
			.actionForEnter = Ctrl_EnterAction,
			.screenType = No_ScreenType,
			.layerPrevious = LCDSettings_Layer },

			{ .name = "Sec to off light",
			.nameActualSize = sizeof("Sec to off light")-1,
			.layer = LCDSettings_Layer,
			.actionForEnter = Ctrl_EnterAction,
			.screenType = No_ScreenType,
			.layerPrevious = LCDSettings_Layer },

			{ .name = "Auto off from",
			.nameActualSize = sizeof("Auto off from")-1,
			.layer = LCDSettings_Layer,
			.actionForEnter = Ctrl_EnterAction,
			.screenType = No_ScreenType,
			.layerPrevious = LCDSettings_Layer },

			{ .name = "Auto off to",
			.nameActualSize = sizeof("Auto off to")-1,
			.layer = LCDSettings_Layer,
			.actionForEnter = Ctrl_EnterAction,
			.screenType = No_ScreenType,
			.layerPrevious = LCDSettings_Layer },

			{ .name = "Home screen",
			.nameActualSize = sizeof("Home screen")-1,
			.layer = LCDSettings_Layer,
			.actionForEnter = Ctrl_EnterAction,
			.screenType = No_ScreenType,
			.layerPrevious = LCDSettings_Layer },

			{ .name = "Auto home return",
			.nameActualSize = sizeof("Auto home return")-1,
			.layer = LCDSettings_Layer,
			.actionForEnter = Ctrl_EnterAction,
			.screenType = No_ScreenType,
			.layerPrevious = LCDSettings_Layer },

			{ .name = "Auto light off",
			.nameActualSize = sizeof("Auto light off")-1,
			.layer = LCDSettings_Layer,
			.actionForEnter = OnOff_EnterAction,
			.screenType = No_ScreenType,
			.layerPrevious = LCDSettings_Layer },
	};
	uint8_t LCD_Board_LCDSettingsList_SIZE = sizeof(LCD_Board_LCDSettingsList)/sizeof(LCDBoard);

	/* Buffer of Rows*Columns size for whole display (80 bytes for 4x20) */
	uint8_t LCD_buffer[LCD.noOfRowsLCD][LCD.noOfColumnsLCD];

	/* Setting " " in the whole buffer */
	memset(LCD_buffer, SPACE_IN_ASCII, (LCD.noOfRowsLCD * LCD.noOfColumnsLCD));

	/** LCD Init (setting number of rows, columns, address, I2C handler **/
	if(TRUE != lcdInit(&hi2c2, LCD.addressLCD, LCD.noOfRowsLCD, LCD.noOfColumnsLCD))
	{
		error = LCD__INIT_FAIL;
		my_error_handler(error);
	}

	// Print text at home position 0,0
	if(TRUE != lcdPrintStr((uint8_t*)"  Welcome on board,", 19))
	{
		error = LCD__ERROR;
		my_error_handler(error);
	}

	// Set cursor at zero position of line 3
	if(TRUE != lcdSetCursorPosition(0, Row3))
	{
		error = LCD__ERROR;
		my_error_handler(error);
	}

	// Print text at cursor position
	if(TRUE != lcdPrintStr((uint8_t*)"      Captain!", 14))
	{
		error = LCD__ERROR;
		my_error_handler(error);
	}

	/************/
	/* For the "hello" text to display for 3 seconds */
	vTaskDelay(3000);
	/************/

	xLastWakeTime = xTaskGetTickCount();

	/* Infinite loop */
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
	for (;;)
	{
		/** Cleaning the buffer by writing only spaces into it **/
		memset(LCD_buffer, SPACE_IN_ASCII, (LCD.noOfRowsLCD * LCD.noOfColumnsLCD));

#if 0
		switch (LCD.layer)
		{
			case Desktop_1: /** Main desktop - car info, speed, mileage, temperatures... **/
			{
				/** First Row **/
				error = copy_str_to_buffer((char*)GPS.forLCD.speed.messageHandler, (char*)LCD_buffer[Row1], 0, GPS.forLCD.speed.size);
				error = copy_str_to_buffer("km/h", (char*)LCD_buffer[Row1], (GPS.forLCD.speed.size+1), 4);
				error = copy_str_to_buffer((char*)GPS.forLCD.clock.messageHandler, (char*)LCD_buffer[Row1], 12, GPS.forLCD.clock.size);

				/** Second Row **/
				error = copy_str_to_buffer("Speed, mileage etc.", (char*)LCD_buffer[Row2], 0, 19);

				/** Third Row **/
				error = copy_str_to_buffer("TODO", (char*)LCD_buffer[Row3], 0, 4);

				/** Fourth Row **/
				error = copy_str_to_buffer((char*)GPS.forLCD.clock.messageHandler, (char*)LCD_buffer[Row4], 12, GPS.forLCD.clock.size);

				break;
			}
			case Desktop_2: /** GPS info - position, altitude, fix... **/
			{
				/** First Row **/
				error = copy_str_to_buffer((char*)GPS.forLCD.status.messageHandler, (char*)LCD_buffer[Row1], 0, GPS.forLCD.status.size);
				error = copy_str_to_buffer((char*)GPS.forLCD.clock.messageHandler, (char*)LCD_buffer[Row1], 12, GPS.forLCD.clock.size);

				/** Second Row **/
				error = copy_str_to_buffer("Lat.: ", (char*)LCD_buffer[Row2], 0, 6);
				error = copy_str_to_buffer((char*)GPS.forLCD.latitude.messageHandler, (char*)LCD_buffer[Row2], 6, GPS.forLCD.latitude.size);
				error = copy_str_to_buffer((char*)GPS.forLCD.latitudeIndicator.messageHandler, (char*)LCD_buffer[Row2], (6+GPS.forLCD.latitude.size+1), GPS.forLCD.latitudeIndicator.size);

				/** Third Row **/
				error = copy_str_to_buffer("Lon.: ", (char*)LCD_buffer[Row3], 0, 6);
				error = copy_str_to_buffer((char*)GPS.forLCD.longitude.messageHandler, (char*)LCD_buffer[Row3], 6, GPS.forLCD.longitude.size);
				error = copy_str_to_buffer((char*)GPS.forLCD.longitudeIndicator.messageHandler, (char*)LCD_buffer[Row3], (6+GPS.forLCD.longitude.size+1), GPS.forLCD.longitudeIndicator.size);

				/** Fourth Row **/
				error = copy_str_to_buffer("Alt.: ", (char*)LCD_buffer[Row4], 0, 6);
				error = copy_str_to_buffer((char*)GPS.forLCD.altitude.messageHandler, (char*)LCD_buffer[Row4], 6, GPS.forLCD.altitude.size);
				error = copy_str_to_buffer("m npm", (char*)LCD_buffer[Row4], (6+GPS.forLCD.altitude.size+1), 5);

				break;
			}
			case Desktop_3: /** Information about board internals (voltages, temperatures ...) **/
			{
				/** First Row **/
				error = copy_str_to_buffer((char*)"3V3: ", (char*)LCD_buffer[Row1], 0, 5);
				error = copy_str_to_buffer((char*)DCDC_3V3_temperature_LCD.messageHandler, (char*)LCD_buffer[Row1], 5, DCDC_3V3_temperature_LCD.size);
				error = copy_str_to_buffer((char*)GPS.forLCD.clock.messageHandler, (char*)LCD_buffer[Row1], 12, GPS.forLCD.clock.size);

				/** Second Row **/
				error = copy_str_to_buffer((char*)"5V: ", (char*)LCD_buffer[Row2], 0, 4);
				error = copy_str_to_buffer((char*)Stabilizer_5V_temperature_LCD.messageHandler, (char*)LCD_buffer[Row2], 4, Stabilizer_5V_temperature_LCD.size);
				error = copy_str_to_buffer((char*)Voltage_5V_LCD.messageHandler, (char*)LCD_buffer[Row2], (4+Stabilizer_5V_temperature_LCD.size+1), Voltage_5V_LCD.size);

				/** Third Row **/
				error = copy_str_to_buffer((char*)"In: ", (char*)LCD_buffer[Row3], 0, 4);
				error = copy_str_to_buffer((char*)Voltage_Vin_LCD.messageHandler, (char*)LCD_buffer[Row3], 4, Voltage_Vin_LCD.size);

				/** Fourth Row **/

				break;
			}
			case Menu:
			{

				break;
			}
			case Alarm:
			{

				break;
			}
			default:
			{
				while(1) {};
				break;
			}
		}
#endif

		switch(currentLayer)
		{
			case MainMenu_Layer:
			{
				error = copy_str_to_buffer("1.", (char*)LCD_buffer[Row1], 0, 2);
				error = copy_str_to_buffer(LCD_MainMenu.name, (char*)LCD_buffer[Row1], 2, LCD_MainMenu.nameActualSize);

				scroll_list(LCD_MainMenuList, LCD_MainMenuList_SIZE, &LCD_MainMenu, &(LCD_buffer[Row1]), (int8_t*)&EncoderCounterDiff, &submenuIterator);

				if(ENC_button.shortPressDetected)
				{
					error = shortButtonPressDetected_LCD(&LCD_MainMenu, LCD_MainMenuList, &currentLayer, &submenuIterator);
				}

				if(ENC_button.longPressDetected)
				{
					error = longButtonPressDetected_LCD(&LCD_MainMenu, &currentLayer, &submenuIterator);
				}

				break;
			}
			case Desktop_Layer:
			{
				/*** First Row ***/
					/* Main Battery Voltage */
				if(TRUE == mainBatteryVoltageValueForLCD.messageReadyFLAG)
					error = copy_str_to_buffer((char*)mainBatteryVoltageValueForLCD.messageHandler, (char*)LCD_buffer[Row1], 0, mainBatteryVoltageValueForLCD.size);
				error = copy_str_to_buffer("V", (char*)LCD_buffer[Row1], 5, 1);

				if(1 == tuBylemFLAG)	//TODO: do usuniecia po ogarnieciu obslugi MicroSD
					error = copy_str_to_buffer((char*)TEMPBUFF, (char*)LCD_buffer[Row2],13 , 5);

					/* Aux Battery Voltage */
				if(TRUE == auxiliaryBatteryVoltageValueForLCD.messageReadyFLAG)
					error = copy_str_to_buffer((char*)auxiliaryBatteryVoltageValueForLCD.messageHandler, (char*)LCD_buffer[Row1], 7, auxiliaryBatteryVoltageValueForLCD.size);
				error = copy_str_to_buffer("V", (char*)LCD_buffer[Row1], 12, 1);

					/* clock */
				if((TRUE == GPS.forLCD.hours.messageReadyFLAG) && (TRUE == GPS.forLCD.minutes.messageReadyFLAG))
				{
					error = copy_str_to_buffer((char*)GPS.forLCD.hours.messageHandler, (char*)LCD_buffer[Row1], 15, GPS.forLCD.hours.size);
					error = copy_str_to_buffer(":", (char*)LCD_buffer[Row1], (15+GPS.forLCD.hours.size), 1);
					error = copy_str_to_buffer((char*)GPS.forLCD.minutes.messageHandler, (char*)LCD_buffer[Row1], (15+GPS.forLCD.hours.size+1), GPS.forLCD.minutes.size);
				}

				/*** Second Row ***/
					/* Water temperature */
				error = copy_str_to_buffer("Water: ", (char*)LCD_buffer[Row2], 0, 7);
				if(TRUE == waterTemperatureValueForLCD.messageReadyFLAG)
					error = copy_str_to_buffer((char*)waterTemperatureValueForLCD.messageHandler, (char*)LCD_buffer[Row2], 7, waterTemperatureValueForLCD.size);
				error = copy_str_to_buffer((char*)degreeSymbolCharacter, (char*)LCD_buffer[Row2], 7+waterTemperatureValueForLCD.size, 1);
				error = copy_str_to_buffer("C", (char*)LCD_buffer[Row2], 7+waterTemperatureValueForLCD.size+1, 1);

				/*** Third Row ***/
					/* Speed */
				if(TRUE == GPS.forLCD.speed.messageReadyFLAG)
					error = copy_str_to_buffer((char*)GPS.forLCD.speed.messageHandler, (char*)LCD_buffer[Row3], 0, GPS.forLCD.speed.size);
				error = copy_str_to_buffer("km/h", (char*)LCD_buffer[Row3], (GPS.forLCD.speed.size+1), 4);
					/* Total Mileage */
				if(TRUE == totalMileageForLCD.messageReadyFLAG)
					error = copy_str_to_buffer((char*)totalMileageForLCD.messageHandler, (char*)LCD_buffer[Row3], 9, totalMileageForLCD.size);
				error = copy_str_to_buffer("km", (char*)LCD_buffer[Row3], (9+totalMileageForLCD.size+1), 2);

				/*** Fourth Row ***/
					/* Engine RPM */
				//TODO
//				if(TRUE == RPMForLCD.messageReadyFLAG)
//					error = copy_str_to_buffer((char*)RPMForLCD.messageHandler, (char*)LCD_buffer[Row4], 8, RPM.size);
				error = copy_str_to_buffer("rpm", (char*)LCD_buffer[Row4], 5, 3);
					/* Trip Mileage */
				if(TRUE == tripMileageForLCD.messageReadyFLAG)
					error = copy_str_to_buffer((char*)tripMileageForLCD.messageHandler, (char*)LCD_buffer[Row4], 9, tripMileageForLCD.size);
				error = copy_str_to_buffer("km", (char*)LCD_buffer[Row4], (9+tripMileageForLCD.size+1), 2);

				if(ENC_button.shortPressDetected)
				{
					ENC_button.shortPressDetected = FALSE;
				}

				if(ENC_button.longPressDetected)
				{
					error = longButtonPressDetected_LCD(&LCD_MainMenuList[0], &currentLayer, &submenuIterator);
				}

				break;
			}
			case GPS_Layer:
			{
				/*** First Row ***/
					/* Fix / NoFix */
				if(TRUE == GPS.forLCD.status.messageReadyFLAG)
					error = copy_str_to_buffer((char*)GPS.forLCD.status.messageHandler, (char*)LCD_buffer[Row1], 0, GPS.forLCD.status.size);
					/* Speed */
				if(TRUE == GPS.forLCD.speed.messageReadyFLAG)
					error = copy_str_to_buffer((char*)GPS.forLCD.speed.messageHandler, (char*)LCD_buffer[Row1], 6, GPS.forLCD.speed.size);
				error = copy_str_to_buffer("km/h", (char*)LCD_buffer[Row1], (6+GPS.forLCD.speed.size+1), 4);
					/* clock */
				if((TRUE == GPS.forLCD.hours.messageReadyFLAG) && (TRUE == GPS.forLCD.minutes.messageReadyFLAG))
				{
					error = copy_str_to_buffer((char*)GPS.forLCD.hours.messageHandler, (char*)LCD_buffer[Row1], 15, GPS.forLCD.hours.size);
					error = copy_str_to_buffer(":", (char*)LCD_buffer[Row1], (15+GPS.forLCD.hours.size), 1);
					error = copy_str_to_buffer((char*)GPS.forLCD.minutes.messageHandler, (char*)LCD_buffer[Row1], (15+GPS.forLCD.hours.size+1), GPS.forLCD.minutes.size);
				}

				/*** Second Row ***/
					/* GPS: Latitude */
				error = copy_str_to_buffer("Lat.:", (char*)LCD_buffer[Row2], 0, 5);
				if(TRUE == GPS.forLCD.latitude.messageReadyFLAG)
					error = copy_str_to_buffer((char*)GPS.forLCD.latitude.messageHandler, (char*)LCD_buffer[Row2], 6, GPS.forLCD.latitude.size);
					/* GPS: Latitude Indicator */
				if(TRUE == GPS.forLCD.latitudeIndicator.messageReadyFLAG)
					error = copy_str_to_buffer((char*)GPS.forLCD.latitudeIndicator.messageHandler, (char*)LCD_buffer[Row2], 6+GPS.forLCD.latitude.size+1, GPS.forLCD.latitudeIndicator.size);


				/*** Third Row ***/
					/* GPS: Longitude */
				error = copy_str_to_buffer("Lon.:", (char*)LCD_buffer[Row3], 0, 5);
				if(TRUE == GPS.forLCD.longitude.messageReadyFLAG)
					error = copy_str_to_buffer((char*)GPS.forLCD.longitude.messageHandler, (char*)LCD_buffer[Row3], 6, GPS.forLCD.longitude.size);
					/* GPS: Longitude Indicator */
				error = copy_str_to_buffer((char*)GPS.forLCD.longitudeIndicator.messageHandler, (char*)LCD_buffer[Row3], 6+GPS.forLCD.longitude.size+1, GPS.forLCD.longitudeIndicator.size);

				/*** Fourth Row ***/
					/* GPS: Altitude */
				error = copy_str_to_buffer("Alt.:", (char*)LCD_buffer[Row4], 0, 5);
				if(TRUE == GPS.forLCD.altitude.messageReadyFLAG)
					error = copy_str_to_buffer((char*)GPS.forLCD.altitude.messageHandler, (char*)LCD_buffer[Row4], 6, GPS.forLCD.altitude.size);
				error = copy_str_to_buffer("m npm", (char*)LCD_buffer[Row4], (6+GPS.forLCD.altitude.size+1), 5);

				if(ENC_button.shortPressDetected)
				{
					ENC_button.shortPressDetected = FALSE;
				}

				if(ENC_button.longPressDetected)
				{
					error = longButtonPressDetected_LCD(&LCD_MainMenuList[1], &currentLayer, &submenuIterator);
				}

				break;
			}
			case CarInfo_Layer:
			{
				if(ENC_button.longPressDetected)
				{
					error = longButtonPressDetected_LCD(&LCD_MainMenuList[2], &currentLayer, &submenuIterator);
				}

				break;
			}
			case JarvisInfo_Layer:
			{
				if(ENC_button.longPressDetected)
				{
					error = longButtonPressDetected_LCD(&LCD_MainMenuList[3], &currentLayer, &submenuIterator);
				}
				break;
			}
			case Last3Diag_Layer:
			{
				static uint8_t isDoneOnce = FALSE;
				static uint8_t isEmpty = FALSE;
				error = copy_str_to_buffer("Last 3 Diag. Snaps.:", (char*)LCD_buffer[Row1], 0u, 20u);

				if(FALSE == isDoneOnce)
				{
					if(TRUE == CAR_EEPROM_counters.didTheNumberOfDiagnosticSnapshotsOverflowed)
					{
						for(uint8_t i=0; i<3; ++i)
						{
							DiagnosticDataRead.DiagnosticDataForEEPROM.memAddress = DIAGNOSTIC_SNAPSHOTS_START_ADDRESS + (uint16_t)((uint8_t)(CAR_EEPROM_counters.diagnosticSnapshotEEPROMIndex - i)*EEPROM_PAGE_SIZE);
							xQueueSend(Queue_EEPROM_readHandle, &(DiagnosticDataRead.DiagnosticDataForEEPROM), (TickType_t)100U/*100ms wait time if the queue is full*/);

							vTaskDelay((TickType_t)MAX_WAIT_TIME_FOR_EEPROM);	/* The wait time for the response from EEPROM */

							if(DATA_READY == DiagnosticDataRead.DiagnosticDataForEEPROM.isReady)
							{
								error = copy_str_to_buffer("ERROR", (char*)LCD_buffer[Row3], 7u, 5u);
								vTaskDelay((TickType_t)2000U);	/* Print "ERROR" for 2 seconds */
							}
							//TODO getting description of the diagnostic problem
							//TODO writing the description into the LCD buffer
						}
					}
					else
					{
						if(0u != CAR_EEPROM_counters.diagnosticSnapshotEEPROMIndex)
						{
							for(uint8_t i=0; ((i<3) && (i<=CAR_EEPROM_counters.diagnosticSnapshotEEPROMIndex)); ++i)
							{
								DiagnosticDataRead.DiagnosticDataForEEPROM.memAddress = DIAGNOSTIC_SNAPSHOTS_START_ADDRESS + (uint16_t)((uint8_t)(CAR_EEPROM_counters.diagnosticSnapshotEEPROMIndex - i)*EEPROM_PAGE_SIZE);
								xQueueSend(Queue_EEPROM_readHandle, &(DiagnosticDataRead.DiagnosticDataForEEPROM), (TickType_t)100U/*100ms wait time if the queue is full*/);

								vTaskDelay((TickType_t)MAX_WAIT_TIME_FOR_EEPROM);	/* The wait time for the response from EEPROM */

								if(DATA_READY == DiagnosticDataRead.DiagnosticDataForEEPROM.isReady)
								{
									error = copy_str_to_buffer("ERROR", (char*)LCD_buffer[Row3], 7u, 5u);
									vTaskDelay((TickType_t)2000U);	/* Print "ERROR" for 2 seconds */
								}
								//TODO getting description of the diagnostic problem
								//TODO writing the description into the LCD buffer
							}
						}
						else
						{
							isEmpty = TRUE;
						}
					}
					isDoneOnce = TRUE;
				}//if(FALSE == isDoneOnce)

				if(TRUE == isEmpty)
				{
					error = copy_str_to_buffer("Nothing to display", (char*)LCD_buffer[Row3], 1u, 18u);
				}
				if(ENC_button.longPressDetected)
				{
					isDoneOnce = FALSE;
					isEmpty = FALSE;
					error = longButtonPressDetected_LCD(&LCD_MainMenuList[4], &currentLayer, &submenuIterator);
				}
				break;
			}
			case Last3Err_Layer:
			{
				static uint8_t isDoneOnce = FALSE;
				static uint8_t isEmpty = FALSE;
				error = copy_str_to_buffer("Last 3 Error Snaps.:", (char*)LCD_buffer[Row1], 0u, 20u);

				if(FALSE == isDoneOnce)
				{
					if(TRUE == BOARD_EEPROM_counters.didTheNumberOfErrorSnapshotsOverflowed)
					{
						for(uint8_t i=0; i<3; ++i)
						{
							ErrorDataRead.ErrorDataForEEPROM.memAddress = ERROR_SNAPSHOTS_START_ADDRESS + (uint16_t)((uint8_t)(BOARD_EEPROM_counters.errorSnapshotEEPROMIndex - i)*EEPROM_PAGE_SIZE);
							xQueueSend(Queue_EEPROM_readHandle, &(ErrorDataRead.ErrorDataForEEPROM), (TickType_t)100U/*100ms wait time if the queue is full*/);

							vTaskDelay((TickType_t)MAX_WAIT_TIME_FOR_EEPROM);	/* The wait time for the response from EEPROM */

							if(DATA_READY == ErrorDataRead.ErrorDataForEEPROM.isReady)
							{
								error = copy_str_to_buffer("ERROR", (char*)LCD_buffer[Row3], 7u, 5u);
								vTaskDelay((TickType_t)2000U);	/* Print "ERROR" for 2 seconds */
							}
							//TODO getting description of the error problem
							//TODO writing the description into the LCD buffer
						}
					}
					else
					{
						if(0u != BOARD_EEPROM_counters.errorSnapshotEEPROMIndex)
						{
							for(uint8_t i=0; ((i<3) && (i<=BOARD_EEPROM_counters.errorSnapshotEEPROMIndex)); ++i)
							{
								ErrorDataRead.ErrorDataForEEPROM.memAddress = ERROR_SNAPSHOTS_START_ADDRESS + (uint16_t)((uint8_t)(BOARD_EEPROM_counters.errorSnapshotEEPROMIndex - i)*EEPROM_PAGE_SIZE);
								xQueueSend(Queue_EEPROM_readHandle, &(ErrorDataRead.ErrorDataForEEPROM), (TickType_t)100U/*100ms wait time if the queue is full*/);

								vTaskDelay((TickType_t)MAX_WAIT_TIME_FOR_EEPROM);	/* The wait time for the response from EEPROM */

								if(DATA_READY == ErrorDataRead.ErrorDataForEEPROM.isReady)
								{
									error = copy_str_to_buffer("ERROR", (char*)LCD_buffer[Row3], 7u, 5u);
									vTaskDelay((TickType_t)2000U);	/* Print "ERROR" for 2 seconds */
								}
								//TODO getting description of the error problem
								//TODO writing the description into the LCD buffer
							}
						}
						else
						{
							isEmpty = TRUE;
						}
					}
					isDoneOnce = TRUE;
				}//if(FALSE == isDoneOnce)

				if(TRUE == isEmpty)
				{
					error = copy_str_to_buffer("Nothing to display", (char*)LCD_buffer[Row3], 1u, 19u);
				}
				if(ENC_button.longPressDetected)
				{
					isDoneOnce = FALSE;
					isEmpty = FALSE;
					error = longButtonPressDetected_LCD(&LCD_MainMenuList[5], &currentLayer, &submenuIterator);
				}
				break;
			}
			case CarSettings_Layer:
			{
				error = copy_str_to_buffer("7.", (char*)LCD_buffer[Row1], 0, 2);
				error = copy_str_to_buffer(LCD_MainMenuList[6].name, (char*)LCD_buffer[Row1], 2, LCD_MainMenuList[6].nameActualSize);

				scroll_list(LCD_CarSettingsList, LCD_CarSettingsList_SIZE, &(LCD_MainMenuList[6]), &(LCD_buffer[Row1]), (int8_t*)&EncoderCounterDiff, &submenuIterator);

				if(ENC_button.shortPressDetected)
				{
					error = shortButtonPressDetected_LCD(&(LCD_MainMenuList[6]), LCD_CarSettingsList, &currentLayer, &submenuIterator);
				}

				if(ENC_button.longPressDetected)
				{
					error = longButtonPressDetected_LCD(&(LCD_MainMenuList[6]), &currentLayer, &submenuIterator);
				}

				break;
			}

			/* *** *** *** *** *** *** *** *** *** *** *** *** *** *** */
			/* Car settings list : */
			/* *** *** *** *** *** *** *** *** *** *** *** *** *** *** */

			case ClearDiagnosticSnapshots:
			{
				char tempBuff[4u] = {' '};
				error = copy_str_to_buffer("No. diag snaps:", (char*)LCD_buffer[Row1], 0u, 15u);
				snprintf(tempBuff, 4u, "%3" PRIu16, CAR_EEPROM_counters.diagnosticSnapshotEEPROMIndex);
				error = copy_str_to_buffer(tempBuff, (char*)LCD_buffer[Row1], 16u, 3u);

				error = copy_str_to_buffer("Overflowed?", (char*)LCD_buffer[Row2], 0u, 11u);
				error = copy_str_to_buffer(((TRUE == CAR_EEPROM_counters.didTheNumberOfDiagnosticSnapshotsOverflowed) ? "Yes" : " No"), (char*)LCD_buffer[Row2], 16u, 3u);

				error = copy_str_to_buffer("Click enter to clear", (char*)LCD_buffer[Row3], 0u, 20u);
				error = copy_str_to_buffer("Diagnostic snapshots", (char*)LCD_buffer[Row4], 0u, 20u);

				if(ENC_button.shortPressDetected)
				{
					LCD_YesNo.layerPrevious = ClearDiagnosticSnapshots;
					error = shortButtonPressDetected_LCD(&(LCD_CarSettingsList[0]), LCD_CarSettingsList, &currentLayer, &submenuIterator);
				}

				if(ENC_button.longPressDetected)
				{
					error = longButtonPressDetected_LCD(&LCD_CarSettingsList[0], &currentLayer, &submenuIterator);
				}
				break;
			}
			case ClearTripMileage:
			{
				error = copy_str_to_buffer("Trip mileage:", (char*)LCD_buffer[Row1], 0u, 15u);
				if(TRUE == tripMileageForLCD.messageReadyFLAG)
					error = copy_str_to_buffer((char*)tripMileageForLCD.messageHandler, (char*)LCD_buffer[Row2], 0u, tripMileageForLCD.size);
				error = copy_str_to_buffer("km", (char*)LCD_buffer[Row2], (tripMileageForLCD.size+1), 2u);

				error = copy_str_to_buffer("Click enter to clear", (char*)LCD_buffer[Row3], 0u, 20u);
				error = copy_str_to_buffer("the trip mileage", (char*)LCD_buffer[Row4], 0u, 16u);

				if(ENC_button.shortPressDetected)
				{
					LCD_YesNo.layerPrevious = ClearTripMileage;
					error = shortButtonPressDetected_LCD(&(LCD_CarSettingsList[1]), LCD_CarSettingsList, &currentLayer, &submenuIterator);
				}

				if(ENC_button.longPressDetected)
				{
					error = longButtonPressDetected_LCD(&LCD_CarSettingsList[1], &currentLayer, &submenuIterator);
				}
				break;
			}
			case WaterSettings_Layer:
			{
				error = copy_str_to_buffer("7.3.", (char*)LCD_buffer[Row1], 0, 4);
				error = copy_str_to_buffer(LCD_CarSettingsList[2].name, (char*)LCD_buffer[Row1], 4, LCD_CarSettingsList[2].nameActualSize);

				scroll_list(LCD_Car_WaterTempSettingsList, LCD_Car_WaterTempSettingsList_SIZE, &(LCD_CarSettingsList[2]), &(LCD_buffer[Row1]), (int8_t*)&EncoderCounterDiff, &submenuIterator);

				if(ENC_button.longPressDetected)
				{
					error = longButtonPressDetected_LCD(&LCD_CarSettingsList[2], &currentLayer, &submenuIterator);
				}
				break;
			}

			/* *** *** *** *** *** *** *** *** *** *** *** *** *** *** */
			/* Water settings list : */
			/* *** *** *** *** *** *** *** *** *** *** *** *** *** *** */

//			case WaterSettings_Layer:
//			{
//				error = copy_str_to_buffer("7.3.", (char*)LCD_buffer[Row1], 0, 4);
//				error = copy_str_to_buffer(LCD_CarSettingsList[2].name, (char*)LCD_buffer[Row1], 4, LCD_CarSettingsList[2].nameActualSize);
//
//				scroll_list(LCD_Car_WaterTempSettingsList, LCD_Car_WaterTempSettingsList_SIZE, &(LCD_CarSettingsList[2]), &(LCD_buffer[Row1]), (int8_t*)&EncoderCounterDiff, &submenuIterator);
//
//				if(ENC_button.longPressDetected)
//				{
//					error = longButtonPressDetected_LCD(&LCD_CarSettingsList[2], &currentLayer, &submenuIterator);
//				}
//				break;
//			}
			case OilTempSettings_Layer:
			{
				error = copy_str_to_buffer("7.4.", (char*)LCD_buffer[Row1], 0, 4);
				error = copy_str_to_buffer(LCD_CarSettingsList[3].name, (char*)LCD_buffer[Row1], 4, LCD_CarSettingsList[3].nameActualSize);

				scroll_list(LCD_Car_OilTempSettingsList, LCD_Car_OilTempSettingsList_SIZE, &(LCD_CarSettingsList[3]), &(LCD_buffer[Row1]), (int8_t*)&EncoderCounterDiff, &submenuIterator);

				if(ENC_button.longPressDetected)
				{
					error = longButtonPressDetected_LCD(&LCD_CarSettingsList[3], &currentLayer, &submenuIterator);
				}
				break;
			}
			case OilPressureSettings_Layer:
			{
				error = copy_str_to_buffer("7.5.", (char*)LCD_buffer[Row1], 0, 4);
				error = copy_str_to_buffer(LCD_CarSettingsList[4].name, (char*)LCD_buffer[Row1], 4, LCD_CarSettingsList[4].nameActualSize);

				scroll_list(LCD_Car_OilPressureSettingsList, LCD_Car_OilPressureSettingsList_SIZE, &(LCD_CarSettingsList[4]), &(LCD_buffer[Row1]), (int8_t*)&EncoderCounterDiff, &submenuIterator);

				if(ENC_button.longPressDetected)
				{
					error = longButtonPressDetected_LCD(&LCD_CarSettingsList[4], &currentLayer, &submenuIterator);
				}
				break;
			}
			case FuelSettings_Layer:
			{
				error = copy_str_to_buffer("7.6.", (char*)LCD_buffer[Row1], 0, 4);
				error = copy_str_to_buffer(LCD_CarSettingsList[5].name, (char*)LCD_buffer[Row1], 4, LCD_CarSettingsList[5].nameActualSize);

				scroll_list(LCD_Car_FuelSettingsList, LCD_Car_FuelSettingsList_SIZE, &(LCD_CarSettingsList[5]), &(LCD_buffer[Row1]), (int8_t*)&EncoderCounterDiff, &submenuIterator);

				if(ENC_button.longPressDetected)
				{
					error = longButtonPressDetected_LCD(&LCD_CarSettingsList[5], &currentLayer, &submenuIterator);
				}
				break;
			}
			case MainBatterySettings_Layer:
			{
				error = copy_str_to_buffer("7.7.", (char*)LCD_buffer[Row1], 0, 4);
				error = copy_str_to_buffer(LCD_CarSettingsList[6].name, (char*)LCD_buffer[Row1], 4, LCD_CarSettingsList[6].nameActualSize);

				scroll_list(LCD_Car_MainBatterySettingsList, LCD_Car_MainBatterySettingsList_SIZE, &(LCD_CarSettingsList[6]), &(LCD_buffer[Row1]), (int8_t*)&EncoderCounterDiff, &submenuIterator);

				if(ENC_button.longPressDetected)
				{
					error = longButtonPressDetected_LCD(&LCD_CarSettingsList[6], &currentLayer, &submenuIterator);
				}
				break;
			}
			case AuxBatterySettings_Layer:
			{
				error = copy_str_to_buffer("7.8.", (char*)LCD_buffer[Row1], 0, 4);
				error = copy_str_to_buffer(LCD_CarSettingsList[7].name, (char*)LCD_buffer[Row1], 4, LCD_CarSettingsList[7].nameActualSize);

				scroll_list(LCD_Car_AuxBatterySettingsList, LCD_Car_AuxBatterySettingsList_SIZE, &(LCD_CarSettingsList[7]), &(LCD_buffer[Row1]), (int8_t*)&EncoderCounterDiff, &submenuIterator);

				if(ENC_button.longPressDetected)
				{
					error = longButtonPressDetected_LCD(&LCD_CarSettingsList[7], &currentLayer, &submenuIterator);
				}
				break;
			}

			/* *** *** *** *** *** *** *** *** *** *** *** *** *** *** */

			case BoardSettings_Layer:
			{
				error = copy_str_to_buffer("8.", (char*)LCD_buffer[Row1], 0, 2);
				error = copy_str_to_buffer(LCD_MainMenuList[7].name, (char*)LCD_buffer[Row1], 3, LCD_MainMenuList[7].nameActualSize);

				scroll_list(LCD_BoardSettingsList, LCD_BoardSettingsList_SIZE, &(LCD_MainMenuList[7]), &(LCD_buffer[Row1]), (int8_t*)&EncoderCounterDiff, &submenuIterator);

				if(ENC_button.shortPressDetected)
				{
					error = shortButtonPressDetected_LCD(&LCD_MainMenuList[7], LCD_BoardSettingsList, &currentLayer, &submenuIterator);
				}

				if(ENC_button.longPressDetected)
				{
					error = longButtonPressDetected_LCD(&LCD_MainMenuList[7], &currentLayer, &submenuIterator);
				}
				break;
			}

			/* *** *** *** *** *** *** *** *** *** *** *** *** *** *** */
			/* Board settings list : */
			/* *** *** *** *** *** *** *** *** *** *** *** *** *** *** */

			case ClearErrorsSnapshots:
			{
				char tempBuff[4u] = {' '};
				error = copy_str_to_buffer("No. error snaps:", (char*)LCD_buffer[Row1], 0u, 16u);
				snprintf(tempBuff, 4u, "%3" PRIu16, BOARD_EEPROM_counters.errorSnapshotEEPROMIndex);
				error = copy_str_to_buffer(tempBuff, (char*)LCD_buffer[Row1], 17u, 3u);

				error = copy_str_to_buffer("Overflowed?", (char*)LCD_buffer[Row2], 0u, 11u);
				error = copy_str_to_buffer(((TRUE == BOARD_EEPROM_counters.didTheNumberOfErrorSnapshotsOverflowed) ? "Yes" : " No"), (char*)LCD_buffer[Row2], 16u, 3u);

				error = copy_str_to_buffer("Click enter to clear", (char*)LCD_buffer[Row3], 0u, 20u);
				error = copy_str_to_buffer("All error snapshots", (char*)LCD_buffer[Row4], 0u, 20u);

				if(ENC_button.shortPressDetected)
				{
					LCD_YesNo.layerPrevious = ClearErrorsSnapshots;
					error = shortButtonPressDetected_LCD(&(LCD_CarSettingsList[0]), LCD_CarSettingsList, &currentLayer, &submenuIterator);
				}

				if(ENC_button.longPressDetected)
				{
					error = longButtonPressDetected_LCD(&LCD_CarSettingsList[0], &currentLayer, &submenuIterator);
				}
				break;
				if(ENC_button.longPressDetected)
				{
					error = longButtonPressDetected_LCD(&LCD_BoardSettingsList[0], &currentLayer, &submenuIterator);
				}
				break;
			}
			case AdjTimePoland:
			{
				if(ENC_button.longPressDetected)
				{
					error = longButtonPressDetected_LCD(&LCD_BoardSettingsList[0], &currentLayer, &submenuIterator);
				}
				break;
			}
			case AdjTimeZone:
			{
				if(ENC_button.longPressDetected)
				{
					error = longButtonPressDetected_LCD(&LCD_BoardSettingsList[0], &currentLayer, &submenuIterator);
				}
				break;
			}
			case InterVoltSettings_Layer:
			{
				error = copy_str_to_buffer("8.4.", (char*)LCD_buffer[Row1], 0, 4);
				error = copy_str_to_buffer(LCD_BoardSettingsList[3].name, (char*)LCD_buffer[Row1], 4, LCD_BoardSettingsList[3].nameActualSize);

				scroll_list(LCD_Board_InternalVoltSettingsList, LCD_Board_InternalVoltSettingsList_SIZE, &(LCD_BoardSettingsList[3]), &(LCD_buffer[Row1]), (int8_t*)&EncoderCounterDiff, &submenuIterator);

				if(ENC_button.longPressDetected)
				{
					error = longButtonPressDetected_LCD(&LCD_BoardSettingsList[0], &currentLayer, &submenuIterator);
				}
				break;
			}
			case InterTempSettings_Layer:
			{
				error = copy_str_to_buffer("8.5.", (char*)LCD_buffer[Row1], 0, 4);
				error = copy_str_to_buffer(LCD_BoardSettingsList[4].name, (char*)LCD_buffer[Row1], 4, LCD_BoardSettingsList[4].nameActualSize);

				scroll_list(LCD_Board_InternalTempSettingsList, LCD_Board_InternalTempSettingsList_SIZE, &(LCD_BoardSettingsList[4]), &(LCD_buffer[Row1]), (int8_t*)&EncoderCounterDiff, &submenuIterator);

				if(ENC_button.longPressDetected)
				{
					error = longButtonPressDetected_LCD(&LCD_BoardSettingsList[0], &currentLayer, &submenuIterator);
				}
				break;
			}
			case BuzzerSettings_Layer:
			{
				error = copy_str_to_buffer("8.6.", (char*)LCD_buffer[Row1], 0, 4);
				error = copy_str_to_buffer(LCD_BoardSettingsList[5].name, (char*)LCD_buffer[Row1], 4, LCD_BoardSettingsList[5].nameActualSize);

				scroll_list(LCD_Board_BuzzerSettingsList, LCD_Board_BuzzerSettingsList_SIZE, &(LCD_BoardSettingsList[5]), &(LCD_buffer[Row1]), (int8_t*)&EncoderCounterDiff, &submenuIterator);

				if(ENC_button.longPressDetected)
				{
					error = longButtonPressDetected_LCD(&LCD_BoardSettingsList[0], &currentLayer, &submenuIterator);
				}
				break;
			}
			case LCDSettings_Layer:
			{
				error = copy_str_to_buffer("8.7.", (char*)LCD_buffer[Row1], 0, 2);
				error = copy_str_to_buffer(LCD_BoardSettingsList[6].name, (char*)LCD_buffer[Row1], 4, LCD_BoardSettingsList[6].nameActualSize);

				scroll_list(LCD_Board_LCDSettingsList, LCD_Board_LCDSettingsList_SIZE, &(LCD_BoardSettingsList[6]), &(LCD_buffer[Row1]), (int8_t*)&EncoderCounterDiff, &submenuIterator);

				if(ENC_button.longPressDetected)
				{
					error = longButtonPressDetected_LCD(&LCD_BoardSettingsList[0], &currentLayer, &submenuIterator);
				}
				break;
			}

			/* *** *** *** *** *** *** *** *** *** *** *** *** *** *** */
			/* Actions layers */

			case YesNo_Layer:
			{
				error = copy_str_to_buffer(LCD_YesNo.name, (char*)LCD_buffer[Row1], 3, LCD_YesNo.nameActualSize);
				error = copy_str_to_buffer("Short press: Yes", (char*)LCD_buffer[Row2], 0, 16);
				error = copy_str_to_buffer("Long press: No", (char*)LCD_buffer[Row3], 0, 14);

				switch(LCD_YesNo.layerPrevious)
				{
					case ClearDiagnosticSnapshots:
					{
						if(ENC_button.shortPressDetected)
						{
							EEPROM_data_struct EEPROMData = {0};

							CAR_EEPROM_counters.diagnosticSnapshotEEPROMIndex = 0u;
							CAR_EEPROM_counters.didTheNumberOfDiagnosticSnapshotsOverflowed = FALSE;

							EEPROMData.EEPROMParameters = &EEPROM_car;
							EEPROMData.memAddressSize = EEPROM_PAGES_ADDRESS_SIZE;
							EEPROMData.size = UINT8_T_SIZE;

							EEPROMData.memAddress = TOTAL_SNAPSHOTS_NUMBER_ADDRESS;
							EEPROMData.data = &(CAR_EEPROM_counters.diagnosticSnapshotEEPROMIndex);
							xQueueSend(Queue_EEPROM_writeHandle, &EEPROMData, (TickType_t)100U/*100ms wait time if the queue is full*/);

							EEPROMData.memAddress = NUMBER_OF_DIAGNOSTIC_SNAPSHOTS_OVERFLOWED_ADDRESS;
							EEPROMData.data = &(CAR_EEPROM_counters.didTheNumberOfDiagnosticSnapshotsOverflowed);
							xQueueSend(Queue_EEPROM_writeHandle, &EEPROMData, (TickType_t)100U/*100ms wait time if the queue is full*/);

							vTaskDelay((TickType_t)MAX_WAIT_TIME_FOR_EEPROM);	/* wait for xx ms for EEPROM to be able to process that */

							if(DATA_READY == EEPROMData.isReady)
							{
								error = copy_str_to_buffer("ERROR", (char*)LCD_buffer[Row4], 7u, 5u);
								vTaskDelay((TickType_t)2000U);	/* Print "ERROR" for 2 seconds */
							}

							error = shortButtonPressDetected_LCD(&LCD_YesNo, LCD_CarSettingsList, &currentLayer, &submenuIterator);
						}

						break;
					}//case ClearDiagnosticSnapshots:
					case ClearTripMileage:
					{
						if(ENC_button.shortPressDetected)
						{
							EEPROM_data_struct EEPROMData = {0};

							CAR_mileage.tripMileage = 0U;

							EEPROMData.EEPROMParameters = &EEPROM_car;
							EEPROMData.memAddressSize = EEPROM_PAGES_ADDRESS_SIZE;
							EEPROMData.size = UINT32_T_SIZE;

							EEPROMData.memAddress = TRIP_MILEAGE_START_ADDRESS;	//TODO - improve to write in the place we should as there is a table of addresses where the mileage is written!!
							EEPROMData.data = &(CAR_mileage.tripMileage);
							xQueueSend(Queue_EEPROM_writeHandle, &EEPROMData, (TickType_t)100U/*100ms wait time if the queue is full*/);

							vTaskDelay((TickType_t)MAX_WAIT_TIME_FOR_EEPROM);	/* wait for xx ms for EEPROM to be able to process that */

							if(DATA_READY == EEPROMData.isReady)
							{
								error = copy_str_to_buffer("ERROR", (char*)LCD_buffer[Row4], 7u, 5u);
								vTaskDelay((TickType_t)2000U);	/* Print "ERROR" for 2 seconds */
							}

							error = shortButtonPressDetected_LCD(&LCD_YesNo, LCD_CarSettingsList, &currentLayer, &submenuIterator);
						}

						break;
					}//case ClearTripMileage:
					case ClearErrorsSnapshots:
					{
						if(ENC_button.shortPressDetected)
						{
							EEPROM_data_struct EEPROMData = {0};

							BOARD_EEPROM_counters.errorSnapshotEEPROMIndex = 0u;
							BOARD_EEPROM_counters.didTheNumberOfErrorSnapshotsOverflowed = FALSE;

							EEPROMData.EEPROMParameters = &EEPROM_board;
							EEPROMData.memAddressSize = EEPROM_PAGES_ADDRESS_SIZE;
							EEPROMData.size = UINT8_T_SIZE;

							EEPROMData.memAddress = NUMBER_OF_ERROR_SNAPSHOTS;
							EEPROMData.data = &(CAR_EEPROM_counters.diagnosticSnapshotEEPROMIndex);
							xQueueSend(Queue_EEPROM_writeHandle, &EEPROMData, (TickType_t)100U/*100ms wait time if the queue is full*/);

							EEPROMData.memAddress = NUMBER_OF_ERROR_SNAPSHOTS_OVERFLOWED_ADDRESS;
							EEPROMData.data = &(CAR_EEPROM_counters.didTheNumberOfDiagnosticSnapshotsOverflowed);
							xQueueSend(Queue_EEPROM_writeHandle, &EEPROMData, (TickType_t)100U/*100ms wait time if the queue is full*/);

							vTaskDelay((TickType_t)MAX_WAIT_TIME_FOR_EEPROM);	/* wait for xx ms for EEPROM to be able to process that */

							if(DATA_READY == EEPROMData.isReady)
							{
								error = copy_str_to_buffer("ERROR", (char*)LCD_buffer[Row4], 7u, 5u);
								vTaskDelay((TickType_t)2000U);	/* Print "ERROR" for 2 seconds */
							}

							error = shortButtonPressDetected_LCD(&LCD_YesNo, LCD_CarSettingsList, &currentLayer, &submenuIterator);
						}

						break;
					}//case ClearErrorsSnapshots:
					default:
					{
						error = LCD__LAYER_CHOICE_FAILURE;
						break;
					}//default:
				}//switch(LCD_YesNo.layerPrevious)
				if(ENC_button.longPressDetected)
				{
					error = longButtonPressDetected_LCD(&LCD_BoardSettingsList[0], &currentLayer, &submenuIterator);
				}
				break;
			}

			default:
			{
				error = LCD__LAYER_CHOICE_FAILURE;
				currentLayer = HOME_SCREEN;
			}
				break;
		}

		if(NO_ERROR != error)
		{
			my_error_handler(error);
		}

		/** Send buffers to the LCD **/
		if(TRUE != lcdSetCursorPosition(0, Row1))
		{
			error = LCD__ERROR;
			my_error_handler(error);
		}
		if(TRUE != lcdPrintStr(LCD_buffer[Row1], LCD.noOfColumnsLCD))
		{
			error = LCD__ERROR;
			my_error_handler(error);
		}
		vTaskDelay(1);

		if(TRUE != lcdSetCursorPosition(0, Row2))
		{
			error = LCD__ERROR;
			my_error_handler(error);
		}
		if(TRUE != lcdPrintStr(LCD_buffer[Row2], LCD.noOfColumnsLCD))
		{
			error = LCD__ERROR;
			my_error_handler(error);
		}
		vTaskDelay(1);

		if(TRUE != lcdSetCursorPosition(0, Row3))
		{
			error = LCD__ERROR;
			my_error_handler(error);
		}
		if(TRUE != lcdPrintStr(LCD_buffer[Row3], LCD.noOfColumnsLCD))
		{
			error = LCD__ERROR;
			my_error_handler(error);
		}
		vTaskDelay(1);

		if(TRUE != lcdSetCursorPosition(0, Row4))
		{
			error = LCD__ERROR;
			my_error_handler(error);
		}
		if(TRUE != lcdPrintStr(LCD_buffer[Row4], LCD.noOfColumnsLCD))
		{
			error = LCD__ERROR;
			my_error_handler(error);
		}
		vTaskDelay(1);

		vTaskDelayUntil(&xLastWakeTime, xFrequency);
	}
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

  /* USER CODE END StartTaskLCD */
}

/* USER CODE BEGIN Header_StartTaskGPS */
/**
* @brief Function implementing the My_GPS_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskGPS */
void StartTaskGPS(void const * argument)
{
  /* USER CODE BEGIN StartTaskGPS */

	TickType_t xLastWakeTime;
	const TickType_t xFrequency = MY_GPS_TASK_TIME_PERIOD;
	Error_Code error = NO_ERROR;

#ifdef GPS_LCD_PRINT
	uint8_t tempHour = '\0';
	uint8_t tempbuffer[3u] = {'a'};

	GPS.forLCD.hours.messageHandler 				= GPS.message_buffers.hours;
	GPS.forLCD.minutes.messageHandler 				= GPS.message_buffers.minutes;
	GPS.forLCD.seconds.messageHandler 				= GPS.message_buffers.seconds;
	GPS.forLCD.clock.messageHandler 				= GPS.message_buffers.clock;
	GPS.forLCD.latitude.messageHandler 				= GPS.message_buffers.latitude;
	GPS.forLCD.latitudeIndicator.messageHandler 	= GPS.message_buffers.latitudeIndicator;
	GPS.forLCD.longitude.messageHandler 			= GPS.message_buffers.longitude;
	GPS.forLCD.longitudeIndicator.messageHandler 	= GPS.message_buffers.longitudeIndicator;
	GPS.forLCD.status.messageHandler				= GPS.message_buffers.fixMessage;
	GPS.forLCD.satellitesUsed.messageHandler		= GPS.message_buffers.satellitesUsed;
	GPS.forLCD.altitude.messageHandler				= GPS.message_buffers.altitude;
	GPS.forLCD.speed.messageHandler					= GPS.message_buffers.speed;

	GPS.forLCD.hours.size 				= 2u;
	GPS.forLCD.minutes.size 			= 2u;
	GPS.forLCD.seconds.size 			= 2u;
	GPS.forLCD.clock.size 				= 8u;
	GPS.forLCD.latitude.size			= 10u;
	GPS.forLCD.latitudeIndicator.size	= 1u;
	GPS.forLCD.longitude.size			= 11u;
	GPS.forLCD.longitudeIndicator.size	= 1u;
#endif

	/* Turns on receiving from GPS */
	HAL_UART_Receive_IT(&huart1, (uint8_t *)&(GPS.receivedByte), 1u);

	xLastWakeTime = xTaskGetTickCount();

	/* Infinite loop */
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
	for (;;)
	{
#ifdef GPS_PARSING
		if (TRUE == GPS.DataReady)
		{
			error = parse_GPS_data(&GPS);
		}

		if(NO_ERROR != error)
		{
			my_error_handler(error);
		}
#endif

#ifdef GPS_LCD_PRINT
		if(TRUE == GPS.TimeReady)
		{
			error = copy_buffer_to_str((char*)GPS.rawData.UTC, (char*)tempbuffer, 0, 2);
			tempHour = (uint8_t)atoi((char*)tempbuffer);
			tempHour += GPS.TimeZoneAdjPoland;

			if(24 <= tempHour)
			{
				tempHour -= 24;
			}

			if(10 <= tempHour)
			{
				itoa(tempHour, (char*)tempbuffer, 10/*decimal system*/);
			}
			else
			{
				tempbuffer[0] = '0';
				itoa(tempHour, (char*)&tempbuffer[1], 10);
			}

			GPS.forLCD.hours.messageReadyFLAG = FALSE;
			error = copy_buffer_to_str((char*)tempbuffer, (char*)GPS.message_buffers.hours, 0u, GPS.forLCD.hours.size);
			GPS.forLCD.hours.messageReadyFLAG = TRUE;

			GPS.forLCD.minutes.messageReadyFLAG = FALSE;
			error = copy_buffer_to_str((char*)GPS.rawData.UTC, (char*)GPS.message_buffers.minutes, 2u, GPS.forLCD.minutes.size);
			GPS.forLCD.minutes.messageReadyFLAG = TRUE;

			GPS.forLCD.seconds.messageReadyFLAG = FALSE;
			error = copy_buffer_to_str((char*)GPS.rawData.UTC, (char*)GPS.message_buffers.seconds, 4u, GPS.forLCD.seconds.size);
			GPS.forLCD.seconds.messageReadyFLAG = TRUE;
		}
		else
		{
			GPS.forLCD.hours.messageReadyFLAG = FALSE;
			GPS.forLCD.minutes.messageReadyFLAG = FALSE;
			GPS.forLCD.seconds.messageReadyFLAG = FALSE;
			for(uint i=0; i<2; ++i)
			{
				GPS.message_buffers.hours[i] = 'x';
				GPS.message_buffers.minutes[i] = 'x';
				GPS.message_buffers.seconds[i] = 'x';
			}
			GPS.forLCD.hours.messageReadyFLAG = TRUE;
			GPS.forLCD.minutes.messageReadyFLAG = TRUE;
			GPS.forLCD.seconds.messageReadyFLAG = TRUE;
		}

		GPS.forLCD.clock.messageReadyFLAG = FALSE;
		GPS.message_buffers.clock[2] = ':';
		GPS.message_buffers.clock[5] = ':';
		error = copy_str_to_buffer((char*)GPS.message_buffers.hours, (char*)GPS.message_buffers.clock, 0u, GPS.forLCD.hours.size);
		error = copy_str_to_buffer((char*)GPS.message_buffers.minutes, (char*)GPS.message_buffers.clock, 3u, GPS.forLCD.minutes.size);
		error = copy_str_to_buffer((char*)GPS.message_buffers.seconds, (char*)GPS.message_buffers.clock, 6u, GPS.forLCD.seconds.size);
		GPS.forLCD.clock.messageReadyFLAG = TRUE;

		if(NO_ERROR != error)
		{
			my_error_handler(error);
		}

		if(TRUE == GPS.Fix)
		{
			GPS.forLCD.latitude.messageReadyFLAG = FALSE;
			GPS.forLCD.latitudeIndicator.messageReadyFLAG = FALSE;
			error = copy_str_to_buffer((char*)GPS.rawData.Latitude, (char*)GPS.message_buffers.latitude, 0u, GPS.forLCD.latitude.size);
			error = copy_str_to_buffer((char*)GPS.rawData.LatitudeIndicator, (char*)GPS.message_buffers.latitudeIndicator, 0u, GPS.forLCD.latitudeIndicator.size);
			GPS.forLCD.latitude.messageReadyFLAG = TRUE;
			GPS.forLCD.latitudeIndicator.messageReadyFLAG = TRUE;

			GPS.forLCD.longitude.messageReadyFLAG = FALSE;
			GPS.forLCD.longitudeIndicator.messageReadyFLAG = FALSE;
			error = copy_str_to_buffer((char*)GPS.rawData.Longitude, (char*)GPS.message_buffers.longitude, 0u, GPS.forLCD.longitude.size);
			error = copy_str_to_buffer((char*)GPS.rawData.LongitudeIndicator, (char*)GPS.message_buffers.longitudeIndicator, 0u, GPS.forLCD.longitudeIndicator.size);
			GPS.forLCD.longitude.messageReadyFLAG = TRUE;
			GPS.forLCD.longitudeIndicator.messageReadyFLAG = TRUE;


			GPS.forLCD.status.messageReadyFLAG = FALSE;
			GPS.forLCD.status.size = 5u;	//5u - Because "Fixed" has 5 letters
			error = copy_str_to_buffer("Fixed", (char*)GPS.message_buffers.fixMessage, 0u, GPS.forLCD.status.size);
			GPS.forLCD.status.messageReadyFLAG = TRUE;

			GPS.forLCD.satellitesUsed.messageReadyFLAG = FALSE;
			error = copy_str_to_buffer((char*)GPS.rawData.SatellitesUsed, (char*)GPS.message_buffers.satellitesUsed, 0u, GPS.forLCD.satellitesUsed.size);
			GPS.forLCD.satellitesUsed.messageReadyFLAG = TRUE;

			GPS.forLCD.altitude.messageReadyFLAG = FALSE;
			GPS.forLCD.altitude.size = strlen((char*)GPS.rawData.Altitude);
			error = copy_str_to_buffer((char*)GPS.rawData.Altitude, (char*)GPS.message_buffers.altitude, 0u, GPS.forLCD.altitude.size);
			GPS.forLCD.altitude.messageReadyFLAG = TRUE;

			GPS.forLCD.speed.messageReadyFLAG = FALSE;
			GPS.forLCD.speed.size = find_nearest_symbol('.', (char*)GPS.rawData.Speed, 0u);
			error = copy_str_to_buffer((char*)GPS.rawData.Speed, (char*)GPS.message_buffers.speed, 0u, GPS.forLCD.speed.size);
			GPS.forLCD.speed.messageReadyFLAG = TRUE;
		}
		else
		{
			GPS.forLCD.latitude.messageReadyFLAG = FALSE;
			GPS.forLCD.latitudeIndicator.messageReadyFLAG = FALSE;
			error = copy_str_to_buffer("xxxx.xxxxx", (char*)GPS.message_buffers.latitude, 0u, GPS.forLCD.latitude.size);
			error = copy_str_to_buffer("X", (char*)GPS.message_buffers.latitudeIndicator, 0u, GPS.forLCD.latitudeIndicator.size);
			GPS.forLCD.latitude.messageReadyFLAG = TRUE;
			GPS.forLCD.latitudeIndicator.messageReadyFLAG = TRUE;

			GPS.forLCD.longitude.messageReadyFLAG = FALSE;
			GPS.forLCD.longitudeIndicator.messageReadyFLAG = FALSE;
			error = copy_str_to_buffer("xxxxx.xxxxx", (char*)GPS.message_buffers.longitude, 0u, GPS.forLCD.longitude.size);
			error = copy_str_to_buffer("X", (char*)GPS.message_buffers.longitudeIndicator, 0u, GPS.forLCD.longitudeIndicator.size);
			GPS.forLCD.longitude.messageReadyFLAG = TRUE;
			GPS.forLCD.longitudeIndicator.messageReadyFLAG = TRUE;


			GPS.forLCD.status.messageReadyFLAG = FALSE;
			GPS.forLCD.status.size = 5u;	//5u - Because "Fixed" has 5 letters
			error = copy_str_to_buffer("NoFix", (char*)GPS.message_buffers.fixMessage, 0u, GPS.forLCD.status.size);
			GPS.forLCD.status.messageReadyFLAG = TRUE;

			GPS.forLCD.satellitesUsed.messageReadyFLAG = FALSE;
			error = copy_str_to_buffer((char*)GPS.rawData.SatellitesUsed, (char*)GPS.message_buffers.satellitesUsed, 0u, GPS.forLCD.satellitesUsed.size);
			GPS.forLCD.satellitesUsed.messageReadyFLAG = TRUE;

			GPS.forLCD.altitude.messageReadyFLAG = FALSE;
			GPS.forLCD.altitude.size = 5u;
			error = copy_str_to_buffer("xxx.x", (char*)GPS.message_buffers.altitude, 0u, GPS.forLCD.altitude.size);
			GPS.forLCD.altitude.messageReadyFLAG = TRUE;

			GPS.forLCD.speed.messageReadyFLAG = FALSE;
			GPS.forLCD.speed.size = 3u;
			error = copy_str_to_buffer("xxx", (char*)GPS.message_buffers.speed, 0u, GPS.forLCD.speed.size);
			GPS.forLCD.speed.messageReadyFLAG = TRUE;
		}

		if(NO_ERROR != error)
		{
			my_error_handler(error);
		}
#endif

		vTaskDelayUntil(&xLastWakeTime, xFrequency);
	}
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

  /* USER CODE END StartTaskGPS */
}

/* USER CODE BEGIN Header_StartTaskEEPROM */
/**
* @brief Function implementing the My_EEPROM_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskEEPROM */
void StartTaskEEPROM(void const * argument)
{
  /* USER CODE BEGIN StartTaskEEPROM */

	TickType_t xLastWakeTime;
	const TickType_t xFrequency = MY_EEPROM_TASK_TIME_PERIOD;
	Error_Code error = NO_ERROR;

	EEPROM_data_struct EEPROMData = {};
	uint8_t localBuffer[64] = {0};

#ifdef RUNTIME_STATS_QUEUES
	vQueueAddToRegistry(Queue_EEPROM_writeHandle, QUEUE_EEPROM_WRITE_NAME);
	vQueueAddToRegistry(Queue_EEPROM_readHandle, QUEUE_EEPROM_READ_NAME);
#endif

	/*** EEPROM INIT SEQUENCE ***/

	/* EEPROM Car initialization: */
	error = InitVariablesFromEEPROMCar();

	if(NO_ERROR != error)
	{
		my_error_handler(error);
	}

	/* EEPROM Board initialization: */
	error = InitVariablesFromEEPROMBoard();

	if(NO_ERROR != error)
	{
		my_error_handler(error);
	}

	xLastWakeTime = xTaskGetTickCount();

  /* Infinite loop */
  for(;;)
  {
	  if(pdTRUE == xQueueReceive(Queue_EEPROM_readHandle, &EEPROMData, (TickType_t)0))
	  {
		  error = ReadEEPROM(EEPROMData.EEPROMParameters, &EEPROMData);
		  if(NO_ERROR == error)
		  {
			  EEPROMData.isReady = DATA_READY;//TODO: mem error dump
		  }
		  else
		  {
			  my_error_handler(error);
		  }
	  }
	  else
	  {
		  if(pdTRUE == xQueueReceive(Queue_EEPROM_writeHandle, &EEPROMData, (TickType_t)0))
		  {
			  if(0u == EEPROMData.size)
			  {
				  error = EEPROM__SIZE_TO_WRITE_IS_ZERO;
			  }
			  else
			  {
				  for(uint8_t i=0; i < EEPROMData.size; ++i)
				  {
					  localBuffer[i] = EEPROMData.data[i];
				  }
				  EEPROMData.data = localBuffer;

				  error = WriteEEPROM(EEPROMData.EEPROMParameters, &EEPROMData);
			  }

			  if(NO_ERROR == error)
			  {
				  EEPROMData.isReady = DATA_READY;//TODO: mem error dump
			  }
			  else
			  {
				  my_error_handler(error);
			  }
			  vTaskDelay(5);
		  }
	  }

	  vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
  /* USER CODE END StartTaskEEPROM */
}

/* USER CODE BEGIN Header_StartTaskDumpToEEPROM */
/**
* @brief Function implementing the My_DumpToEEPROM thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskDumpToEEPROM */
void StartTaskDumpToEEPROM(void const * argument)
{
  /* USER CODE BEGIN StartTaskDumpToEEPROM */

	TickType_t xLastWakeTime;
	const TickType_t xFrequency = MY_DUMP_TO_EEPROM_TASK_TIME_PERIOD;
	Error_Code error = NO_ERROR;

	DiagnosticDataToEEPROM_struct DiagnosticDataToSend =
	{ .DiagnosticDataForEEPROM =
		{ .EEPROMParameters = &EEPROM_car,
		.data = DiagnosticDataToSend.data,
		.size = MAX_DIAGNOSTIC_SNAPSHOT_SIZE,
		.memAddress = 0,
		.memAddressSize = 2 },
	.diag_mess_from_queue =
		{ .snapshotIdentificator = DIAGNOSTICS_OK,
		.value = 0 } };

	ErrorDataToEEPROM_struct ErrorDataToSend =
	{ .ErrorDataForEEPROM.EEPROMParameters = &EEPROM_board,
	.error_mess_from_queue = NO_ERROR };

#ifdef RUNTIME_STATS_QUEUES
	vQueueAddToRegistry(Queue_diagnostic_snapshot_dumpHandle, QUEUE_DIAGNOSTIC_DUMP_NAME);
	vQueueAddToRegistry(Queue_error_snapshot_dumpHandle, QUEUE_ERROR_DUMP_NAME);
#endif

	xLastWakeTime = xTaskGetTickCount();

  /* Infinite loop */
  for(;;)
  {
	  if(pdTRUE == xQueueReceive(Queue_diagnostic_snapshot_dumpHandle, &(DiagnosticDataToSend.diag_mess_from_queue), (TickType_t)0))
	  {
		  DiagnosticDataToSend.longitudeIndicator = (TRUE == GPS.Fix) ? GPS.rawData.LongitudeIndicator[0] : 'X';
		  DiagnosticDataToSend.latitudeIndicator = (TRUE == GPS.Fix) ? GPS.rawData.LatitudeIndicator[0] : 'X';

		  for(uint8_t i = 0; i<11; ++i)
		  {
			  if(6>i)
			  {
			  DiagnosticDataToSend.rawTime[i] = (TRUE == GPS.TimeReady) ? GPS.rawData.UTC[i] : 'x';
			  }

			  if(10>i)
			  {
			  DiagnosticDataToSend.latitude[i] = (TRUE == GPS.Fix) ? GPS.rawData.Latitude[i] : 'y';
			  }

			  DiagnosticDataToSend.longitude[i] = (TRUE == GPS.Fix) ? GPS.rawData.Longitude[i] : 'z';
		  }

		  error = WriteEEPROM(DiagnosticDataToSend.DiagnosticDataForEEPROM.EEPROMParameters, &(DiagnosticDataToSend.DiagnosticDataForEEPROM));

		  if(NO_ERROR == error)
		  {
			  DiagnosticDataToSend.DiagnosticDataForEEPROM.isReady = DATA_READY; //TODO
		  }
		  else
		  {
			  my_error_handler(error);
		  }

	  }
	  else
	  {
		  if(pdTRUE == xQueueReceive(Queue_error_snapshot_dumpHandle, &(ErrorDataToSend.error_mess_from_queue), (TickType_t)0))
		  {
			  error = WriteEEPROM(ErrorDataToSend.ErrorDataForEEPROM.EEPROMParameters, &(ErrorDataToSend.ErrorDataForEEPROM));
			  if(NO_ERROR == error)
			  {
				  ErrorDataToSend.ErrorDataForEEPROM.isReady = DATA_READY; //TODO
			  }
			  else
			  {
				  my_error_handler(error);
			  }
			  vTaskDelay((TickType_t)5u);
		  }
	  }

	  vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
  /* USER CODE END StartTaskDumpToEEPROM */
}

/* USER CODE BEGIN Header_StartTaskDumpToSDCard */
/**
* @brief Function implementing the My_DumpToSDCard thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskDumpToSDCard */
void StartTaskDumpToSDCard(void const * argument)
{
  /* USER CODE BEGIN StartTaskDumpToSDCard */

	TickType_t xLastWakeTime;
	const TickType_t xFrequency = MY_DUMP_TO_SDCARD_TASK_TIME_PERIOD;
	Error_Code error = NO_ERROR;

	MX_USB_DEVICE_Init();

	FRESULT result = FR_OK;

	char ReadBuffer[100] = "";
	uint32_t NoOfReadBytes = 0;

	/* Check if SD card is ready to be used */
	while(HAL_SD_STATE_READY != hsd.State)
	{
		vTaskDelay((TickType_t)100);
	}

	result = BSP_SD_Init();

	if(FR_OK == result)
	{
		result = f_mount(&SDFatFS, "", 1);
	}
	if(FR_OK == result)
	{
	result = f_open(&SDFile, "test.txt", FA_READ);
	}
	if(FR_OK == result)
	{
	result = f_read(&SDFile, ReadBuffer, 20, (UINT*)&NoOfReadBytes);
	}
	if(FR_OK == result)
	{
	result = f_close(&SDFile);
	}

	if(TRUE != compare_two_strings(ReadBuffer, TEST_MESSAGE_FOR_CHECK, 0u, 20u))
	{
		error = SDCARD__INITIAL_READ_FAILED;
		my_error_handler(error);

		/* Stop the task now, because if the card is not read correctly then do not proceed! */
		vTaskSuspend(My_DumpToSDCardHandle);
	}

	error = copy_str_to_buffer(ReadBuffer, (char*)TEMPBUFF, 0, 20);
	tuBylemFLAG = 1;

	xLastWakeTime = xTaskGetTickCount();

  /* Infinite loop */
  for(;;)
  {
	  vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
  /* USER CODE END StartTaskDumpToSDCard */
}

/* USER CODE BEGIN Header_StartTask250ms */
/**
* @brief Function implementing the My_250ms_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask250ms */
void StartTask250ms(void const * argument)
{
  /* USER CODE BEGIN StartTask250ms */

	TickType_t xLastWakeTime;
	const TickType_t xFrequency = MY_TASK_250_MS_TIME_PERIOD;
	Error_Code error = NO_ERROR;

	/* For calculating the engine parameters (counting the mean value) */
	static volatile float EngineWaterTemperatureTable[NO_OF_ENGINE_WATER_TEMP_MEASUREMENTS_ADDED] = {0};
	static volatile float EngineWaterTemperatureMovingAverage[NO_OF_ENGINE_WATER_TEMP_MEASUREMENTS_ADDED] = {0};
	static uint8_t i_waterTemp = 0;
	static float tempEngineWaterTemp = 0.0;
	float waterTemperatureValue_beforeMovingAverage = 0.0;
	static uint8_t waterTemperatureValueMessage[4] = "";
	waterTemperatureValueForLCD.messageHandler = waterTemperatureValueMessage;

	static volatile float EngineOilTemperatureTable[NO_OF_ENGINE_OIL_TEMP_MEASUREMENTS_ADDED] = {0};
	static volatile float EngineOilTemperatureTableMovingAverage[NO_OF_ENGINE_OIL_TEMP_MEASUREMENTS_ADDED] = {0};
	static uint8_t i_oilTemp = 0;
	static float tempEngineOilTemp = 0.0;
	float oilTemperatureValue_beforeMovingAverage = 0.0;
	static uint8_t oilTemperatureValueMessage[4] = "";
	oilTemperatureValueForLCD.messageHandler = oilTemperatureValueMessage;

#ifdef OIL_PRESSURE_ANALOG_SENSOR
	static volatile float EngineOilPressureTable[NO_OF_ENGINE_OIL_PRESSURE_MEASUREMENTS_ADDED] = {0};
//	static volatile float EngineOilPressureTableMovingAverage[NO_OF_ENGINE_OIL_PRESSURE_MEASUREMENTS_ADDED] = {0};
	static uint8_t i_oilPressure = 0;
	static float tempEngineOilPressure = 0.0;
	static uint8_t oilPressureValueMessage[4] = "";
	oilPressureValueForLCD.messageHandler = oilPressureValueMessage;
#endif

#ifdef OIL_PRESSURE_BINARY_SENSOR
	static uint8_t oilPressureValueBinaryMessage[4] = "";	//OK, NOK
	oilPressureValueBinaryForLCD.messageHandler = oilPressureValueBinaryMessage;
#endif

	static volatile float MainBatteryVoltageValueTable[NO_OF_MAIN_BATTERY_VOLTAGE_MEASUREMENTS_ADDED] = {0};
//	static volatile float MainBatteryVoltageValueTableMovingAverage[NO_OF_MAIN_BATTERY_VOLTAGE_MEASUREMENTS_ADDED] = {0};
	static uint8_t i_mainBateryVoltage = 0;
	static float tempMainBatteryVoltage = 0.0;
	static uint8_t mainBatteryVoltageValueMessage[6] = "";
	mainBatteryVoltageValueForLCD.messageHandler = mainBatteryVoltageValueMessage;

	static volatile float AuxiliaryBatteryVoltageValueTable[NO_OF_AUXILIARY_BATTERY_VOLTAGE_MEASUREMENTS_ADDED] = {0};
//	static volatile float AuxiliaryBatteryVoltageValueTableMovingAverage[NO_OF_AUXILIARY_BATTERY_VOLTAGE_MEASUREMENTS_ADDED] = {0};
	static uint8_t i_auxBatteryVoltage = 0;
	static float tempAuxBatteryVoltage = 0.0;
	static uint8_t auxiliaryBatteryVoltageValueMessage[6] = "";
	auxiliaryBatteryVoltageValueForLCD.messageHandler = auxiliaryBatteryVoltageValueMessage;

	static volatile float FuelLevelValueTable[NO_OF_FUEL_LEVEL_MEASUREMENTS_ADDED] = {0};
	static volatile float FuelLevelValueTableMovingAverage[NO_OF_FUEL_LEVEL_MEASUREMENTS_ADDED] = {0};
	static uint8_t i_fuelLevel = 0;
	static float tempFuelLevel = 0.0;
	float fuelLevelValue_beforeMovingAverage = 0.0;
	static uint8_t fuelLevelValueMessage[3] = "";
	fuelLevelValueForLCD.messageHandler = fuelLevelValueMessage;


	xLastWakeTime = xTaskGetTickCount();

	/* Infinite loop */
	for (;;)
	{
		/* Water Temperature Value measurement from LM35 */
		if(NO_OF_ENGINE_WATER_TEMP_MEASUREMENTS_ADDED > i_waterTemp)
		{
			error = calculate_LM35_temperature((float*)&(EngineWaterTemperatureTable[i_waterTemp]), ADC1Measures[0]);
			tempEngineWaterTemp += EngineWaterTemperatureTable[i_waterTemp];
			i_waterTemp += 1;
		}
		else
		{
			waterTemperatureValue_beforeMovingAverage = tempEngineWaterTemp / NO_OF_ENGINE_WATER_TEMP_MEASUREMENTS_ADDED;
			EngineWaterTemperatureMovingAverage[NO_OF_ENGINE_WATER_TEMP_MEASUREMENTS_ADDED-1] = waterTemperatureValue_beforeMovingAverage;
			tempEngineWaterTemp = waterTemperatureValue_beforeMovingAverage;

			for(uint8_t j = 0; j< (NO_OF_ENGINE_WATER_TEMP_MEASUREMENTS_ADDED - 1); ++j)
			{
				EngineWaterTemperatureMovingAverage[j] = EngineWaterTemperatureMovingAverage[j+1];
				tempEngineWaterTemp += EngineWaterTemperatureMovingAverage[j];
			}

			waterTemperatureValue = tempEngineWaterTemp / NO_OF_ENGINE_WATER_TEMP_MEASUREMENTS_ADDED;

			waterTemperatureValueForLCD.messageReadyFLAG = FALSE;
			snprintf((char*)waterTemperatureValueForLCD.messageHandler, 4, "%3" PRIu16, (uint16_t)waterTemperatureValue);
			waterTemperatureValueForLCD.size = strlen((char*)waterTemperatureValueForLCD.messageHandler);
			waterTemperatureValueForLCD.messageReadyFLAG = TRUE;

			tempEngineWaterTemp = 0;
			i_waterTemp = 0;
		}

		if(NO_ERROR != error)
		{
		  my_error_handler(error);
		}

		/* Oil Temperature Value measurement */
		if(NO_OF_ENGINE_OIL_TEMP_MEASUREMENTS_ADDED > i_oilTemp)
		{
			error = calculate_LM35_temperature((float*)&(EngineOilTemperatureTable[i_oilTemp]), ADC1Measures[0]);
			tempEngineOilTemp += EngineOilTemperatureTable[i_oilTemp];
			i_oilTemp += 1;
		}
		else
		{
			oilTemperatureValue_beforeMovingAverage = tempEngineOilTemp / NO_OF_ENGINE_OIL_TEMP_MEASUREMENTS_ADDED;
			EngineOilTemperatureTableMovingAverage[NO_OF_ENGINE_OIL_TEMP_MEASUREMENTS_ADDED-1] = oilTemperatureValue_beforeMovingAverage;
			tempEngineOilTemp = oilTemperatureValue_beforeMovingAverage;

			for(uint8_t j = 0; j< (NO_OF_ENGINE_OIL_TEMP_MEASUREMENTS_ADDED - 1); ++j)
			{
				EngineOilTemperatureTableMovingAverage[j] = EngineOilTemperatureTableMovingAverage[j+1];
				tempEngineOilTemp += EngineOilTemperatureTableMovingAverage[j];
			}

			oilTemperatureValue = tempEngineOilTemp / NO_OF_ENGINE_OIL_TEMP_MEASUREMENTS_ADDED;

			oilTemperatureValueForLCD.messageReadyFLAG = FALSE;
			snprintf((char*)oilTemperatureValueForLCD.messageHandler, 4, "%3" PRIu16, (uint16_t)oilTemperatureValue);
			oilTemperatureValueForLCD.size = strlen((char*)oilTemperatureValueForLCD.messageHandler);
			oilTemperatureValueForLCD.messageReadyFLAG = TRUE;

			tempEngineOilTemp = 0;
			i_oilTemp = 0;
		}

		if(NO_ERROR != error)
		{
		  my_error_handler(error);
		}

		/* Oil Pressure Value measurement */
#ifdef OIL_PRESSURE_ANALOG_SENSOR
		if(NO_OF_ENGINE_OIL_PRESSURE_MEASUREMENTS_ADDED > i_oilPressure)
		{
			error = calculate_EngineOilPressure((float*)&(EngineOilPressureTable[i_oilPressure]), ADC1Measures[0]);
			tempEngineOilPressure += EngineOilPressureTable[i_oilPressure];
			i_oilPressure += 1;
		}
		else
		{
			oilPressureValue = tempEngineOilPressure / NO_OF_ENGINE_OIL_PRESSURE_MEASUREMENTS_ADDED;

			oilPressureValueForLCD.messageReadyFLAG = FALSE;
			snprintf((char*)oilPressureValueForLCD.messageHandler, 4, "%01" PRIu16 ".%01" PRIu16, (uint16_t)oilPressureValue, (uint16_t)(oilPressureValue*10)%10);
			oilPressureValueForLCD.size = strlen((char*)oilPressureValueForLCD.messageHandler);
			oilPressureValueForLCD.messageReadyFLAG = TRUE;

			tempEngineOilPressure = 0;
			i_oilPressure = 0;
		}
#endif

#ifdef OIL_PRESSURE_BINARY_SENSOR
		if(GPIO_PIN_RESET == HAL_GPIO_ReadPin(GPIO_PORT_OIL_PRESSURE_SENSOR, GPIO_PORT_OIL_PRESSURE_SENSOR))
		{
			oilPressureValueBinary = OK;
			oilPressureValueBinaryForLCD.messageReadyFLAG = FALSE;
			snprintf((char*)oilPressureValueBinaryForLCD.messageHandler, 4, "OK");
			oilPressureValueBinaryForLCD.size = strlen((char*)oilPressureValueForLCD.messageHandler);
			oilPressureValueBinaryForLCD.messageReadyFLAG = TRUE;
		}
		else
		{
			oilPressureValueBinary = NOK;
			oilPressureValueBinaryForLCD.messageReadyFLAG = FALSE;
			snprintf((char*)oilPressureValueBinaryForLCD.messageHandler, 4, "NOK");
			oilPressureValueBinaryForLCD.size = strlen((char*)oilPressureValueForLCD.messageHandler);
			oilPressureValueBinaryForLCD.messageReadyFLAG = TRUE;
		}
#endif

		if(NO_ERROR != error)
		{
		  my_error_handler(error);
		}

		/* Main Battery Voltage Value measurement */
		if(NO_OF_MAIN_BATTERY_VOLTAGE_MEASUREMENTS_ADDED > i_mainBateryVoltage)
		{
			for(uint8_t j = 1; j< (NO_OF_MAIN_BATTERY_VOLTAGE_MEASUREMENTS_ADDED); ++j)
			{
				MainBatteryVoltageValueTable[j] = MainBatteryVoltageValueTable[j-1];
				tempMainBatteryVoltage += MainBatteryVoltageValueTable[j];
			}

			error = calculate_voltage((float*)&(MainBatteryVoltageValueTable[0u]), ADC1Measures[0], MAIN_BATTERY_VOLTAGE_DIVIDER);
			tempMainBatteryVoltage += MainBatteryVoltageValueTable[0u];
			mainBatteryVoltageValue = tempMainBatteryVoltage / NO_OF_MAIN_BATTERY_VOLTAGE_MEASUREMENTS_ADDED;

			mainBatteryVoltageValueForLCD.messageReadyFLAG = FALSE;
			snprintf((char*)mainBatteryVoltageValueForLCD.messageHandler, 6, "%02" PRIu16 ".%02" PRIu16, (uint16_t)mainBatteryVoltageValue, (uint16_t)(mainBatteryVoltageValue*100)%100);
			mainBatteryVoltageValueForLCD.size = strlen((char*)mainBatteryVoltageValueForLCD.messageHandler);
			mainBatteryVoltageValueForLCD.messageReadyFLAG = TRUE;

			i_mainBateryVoltage += 1;
		}
		else
		{
			tempMainBatteryVoltage = 0;
			i_mainBateryVoltage = 0;
		}

		if(NO_ERROR != error)
		{
		  my_error_handler(error);
		}

		/* Auxiliary Battery Voltage Value measurement */
		if(NO_OF_AUXILIARY_BATTERY_VOLTAGE_MEASUREMENTS_ADDED > i_auxBatteryVoltage)
		{
			for(uint8_t j = 1; j< (NO_OF_AUXILIARY_BATTERY_VOLTAGE_MEASUREMENTS_ADDED); ++j)
			{
				AuxiliaryBatteryVoltageValueTable[j] = AuxiliaryBatteryVoltageValueTable[j-1];
				tempAuxBatteryVoltage += AuxiliaryBatteryVoltageValueTable[j];
			}

			error = calculate_voltage((float*)&(AuxiliaryBatteryVoltageValueTable[0u]), ADC1Measures[0], AUXILIARY_BATTERY_VOLTAGE_DIVIDER);
			tempAuxBatteryVoltage += AuxiliaryBatteryVoltageValueTable[0u];
			auxiliaryBatteryVoltageValue = tempAuxBatteryVoltage / NO_OF_AUXILIARY_BATTERY_VOLTAGE_MEASUREMENTS_ADDED;

			auxiliaryBatteryVoltageValueForLCD.messageReadyFLAG = FALSE;
			snprintf((char*)auxiliaryBatteryVoltageValueForLCD.messageHandler, 6, "%02" PRIu16 ".%02" PRIu16, (uint16_t)auxiliaryBatteryVoltageValue, (uint16_t)(auxiliaryBatteryVoltageValue*100)%100);
			auxiliaryBatteryVoltageValueForLCD.size = strlen((char*)auxiliaryBatteryVoltageValueForLCD.messageHandler);
			auxiliaryBatteryVoltageValueForLCD.messageReadyFLAG = TRUE;

			i_auxBatteryVoltage += 1;
		}
		else
		{
			tempAuxBatteryVoltage = 0;
			i_auxBatteryVoltage = 0;
		}

		if(NO_ERROR != error)
		{
		  my_error_handler(error);
		}

		/* Fuel Level Value measurement */
		if(NO_OF_FUEL_LEVEL_MEASUREMENTS_ADDED > i_fuelLevel)
		{
			error = calculate_fuelLevel((float*)&(FuelLevelValueTable[i_fuelLevel]), ADC1Measures[0]);
			tempFuelLevel += FuelLevelValueTable[i_fuelLevel];
			i_fuelLevel += 1;
		}
		else
		{
			fuelLevelValue_beforeMovingAverage = tempFuelLevel / NO_OF_FUEL_LEVEL_MEASUREMENTS_ADDED;
			FuelLevelValueTableMovingAverage[NO_OF_FUEL_LEVEL_MEASUREMENTS_ADDED-1] = fuelLevelValue_beforeMovingAverage;
			tempFuelLevel = fuelLevelValue_beforeMovingAverage;

			for(uint8_t j = 0; j< (NO_OF_FUEL_LEVEL_MEASUREMENTS_ADDED - 1); ++j)
			{
				FuelLevelValueTableMovingAverage[j] = FuelLevelValueTableMovingAverage[j+1];
				tempFuelLevel += FuelLevelValueTableMovingAverage[j];
			}

			fuelLevelValue = tempFuelLevel / NO_OF_FUEL_LEVEL_MEASUREMENTS_ADDED;

			fuelLevelValueForLCD.messageReadyFLAG = FALSE;
			snprintf((char*)fuelLevelValueForLCD.messageHandler, 4, "%3" PRIu16, (uint16_t)fuelLevelValue);
			fuelLevelValueForLCD.size = strlen((char*)fuelLevelValueForLCD.messageHandler);
			fuelLevelValueForLCD.messageReadyFLAG = TRUE;

			tempEngineWaterTemp = 0;
			i_waterTemp = 0;
		}

		if(NO_ERROR != error)
		{
		  my_error_handler(error);
		}

		vTaskDelayUntil(&xLastWakeTime, xFrequency);
	}
  /* USER CODE END StartTask250ms */
}

/* USER CODE BEGIN Header_StartTask50ms */
/**
* @brief Function implementing the My_50ms_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask50ms */
void StartTask50ms(void const * argument)
{
  /* USER CODE BEGIN StartTask50ms */
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = MY_TASK_50_MS_TIME_PERIOD;

	volatile uint16_t TIM8CounterReadout = 0;
	static uint16_t previousCounterValue = 0;
	volatile int32_t tempDiff = 0;

	xLastWakeTime = xTaskGetTickCount();

  /* Infinite loop */
  for(;;)
  {
	  TIM8CounterReadout = __HAL_TIM_GET_COUNTER(&htim8);
	  tempDiff = TIM8CounterReadout - previousCounterValue;

	  if((4 <= tempDiff) || (-4 >= tempDiff))
	  {
		  tempDiff /= 4;
		  EncoderCounterDiff += (int8_t)tempDiff;
		  previousCounterValue = TIM8CounterReadout;
	  }

	  vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
  /* USER CODE END StartTask50ms */
}

/* ENC_Button_LongPress_Callback function */
void ENC_Button_LongPress_Callback(void const * argument)
{
  /* USER CODE BEGIN ENC_Button_LongPress_Callback */

	ENC_button.allFlags |= 0b00000110; /* Set first and second bit to q to
	 	 	 	 	 	 	 	 	indicate the long press on both bits */

  /* USER CODE END ENC_Button_LongPress_Callback */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
