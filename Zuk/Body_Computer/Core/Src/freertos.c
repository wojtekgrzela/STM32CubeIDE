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
/* USER CODE BEGIN Variables */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;
extern UART_HandleTypeDef huart1;

#ifdef RUNTIME_STATS_TIMER_CONFIG
extern TIM_HandleTypeDef htim7;
#endif

#ifdef RUNTIME_STATS_QUEUES
#define QUEUE_EEPROM_WRITE_NAME		"EEPROM_WRITE"
#define QUEUE_EEPROM_READ_NAME		"EEPROM_READ"
#define QUEUE_DIAGNOSTIC_DUMP_NAME	"DIAGNOSTIC_DUMP"
#define QUEUE_ERROR_DUMP_NAME		"ERROR_DUMP"
#endif
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
volatile uint32_t Tim7_Counter_100us;

/* For Signaling ENC button */
volatile ENCButton_struct ENC_button = {};

/* For measurements from ADC3 */
volatile uint16_t ADC3Measures[NO_OF_ADC3_MEASURES] = { 0 };

/* For GPS data, buffering, messages for LCD */
GPS_data_struct GPS = { .TImeZoneAdjPoland = 2 };

/* For LCD parameters and settings */
LCD_parameters_struct LCD =
{ .addressLCD = 0x27, .noOfRowsLCD = 4, .noOfColumnsLCD = 20, .Row1 = 0, .Row2 =
		1, .Row3 = 2, .Row4 = 3 };

/* For measuring Board temperatures with usage of NTC parameters */
NTC_parameters_struct NTC =
		{ .Beta = 4250, .R25 = 100000, .Rgnd = 10000, .T25 = 298, .beta_x_T25 =
				1266500 };

/* For EEPROMs usage, their addressing and blocking */
EEPROM_parameters_struct EEPROM_car =
{ .EEPROM_hi2c = &hi2c1, .address = EEPROM_CAR_ADDRESS, .pin =
		EEPROM_CAR_BLOCK_PIN, .port = EEPROM_CAR_BLOCK_PORT };
EEPROM_parameters_struct EEPROM_board =
{ .EEPROM_hi2c = &hi2c1, .address = EEPROM_BOARD_ADDRESS, .pin =
		EEPROM_BOARD_BLOCK_PIN, .port = EEPROM_BOARD_BLOCK_PORT };

#ifdef BOARD_TEMPERATURE_MEASUREMENT
LCD_message DCDC_3V3_temperature_LCD = {.messageHandler = NULL, .size = 0};
LCD_message Stabilizer_5V_temperature_LCD = {.messageHandler = NULL, .size = 0};
#endif

#ifdef BOARD_VOLTAGE_MEASUREMENT
LCD_message Voltage_Vin_LCD = {.messageHandler = NULL, .size = 0};
LCD_message Voltage_5V_LCD = {.messageHandler = NULL, .size = 0};
#endif


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* CAR SETTINGS */
uint32_t CAR_totalMileage = 0;
uint32_t CAR_tripMileage = 0;

struct waterTemp_struct
{
	uint8_t waterHighTempWarningThreshold;
	uint8_t waterHighTempAlarmThreshold;
	uint8_t waterHighTempFanOnThreshold;
	uint8_t waterHighTempFanOffThreshold;

	uint16_t waterHighTempWarningThreshold_address;
	uint16_t waterHighTempAlarmThreshold_address;
	uint16_t waterHighTempFanOnThreshold_address;
	uint16_t waterHighTempFanOffThreshold_address;
	uint16_t allSettings_address;

	union
	{
		uint8_t allSettings;
		struct
		{
			uint8_t waterTempWarningOn			:1;
			uint8_t waterTempAlarmOn			:1;
			uint8_t waterFanControlOn			:1;
			uint8_t waterTempAlarmBuzzerOn		:1;
			uint8_t waterTempWarningSnapshotOn 	:1;
			uint8_t waterTempAlarmSnapshotOn	:1;
			/* 2 more free bits here */
		};
	};
}CAR_waterTemp = {.waterHighTempWarningThreshold_address = WATER_HIGH_TEMP_WARNING_THRESHOLD_ADDRESS,
					.waterHighTempAlarmThreshold_address = WATER_HIGH_TEMP_ALARM_THRESHOLD_ADDRESS,
					.waterHighTempFanOnThreshold_address = WATER_HIGH_TEMP_FAN_ON_THRESHOLD_ADDRESS,
					.waterHighTempFanOffThreshold_address = WATER_HIGH_TEMP_FAN_OFF_THRESHOLD_ADDRESS,
					.allSettings_address = WATER_TEMP_ALL_SETTINGS_ADDRESS};

struct oilTemp_struct
{
	uint8_t oilHighTempWarningThreshold;
	uint8_t oilHighTempAlarmThreshold;

	uint16_t oilHighTempWarningThreshold_address;
	uint16_t oilHighTempAlarmThreshold_address;
	uint16_t allSettings_address;

	union
	{
		uint8_t allSettings;
		struct
		{
			uint8_t oilTempWarningOn			:1;
			uint8_t oilTempAlarmOn				:1;
			uint8_t oilTempAlarmBuzzerOn		:1;
			uint8_t oilTempWarningSnapshotOn	:1;
			uint8_t oilTempAlarmSnapshotOn		:1;
			/* 3 more free bits here */
		};
	};
}CAR_oilTemp = {.oilHighTempWarningThreshold_address = OIL_HIGH_TEMP_WARNING_THRESHOLD_ADDRESS,
				.oilHighTempAlarmThreshold_address = OIL_HIGH_TEMP_ALARM_THRESHOLD_ADDRESS,
				.allSettings_address = OIL_TEMP_ALL_SETTINGS_ADDRESS};

struct battery_struct
{
	uint16_t batteryLowVoltageAlarmThreshold;
	uint16_t batteryHighVoltageAlarmThreshold;

	uint16_t batteryLowVoltageAlarmThreshold_address;
	uint16_t batteryHighVoltageAlarmThreshold_address;
	uint16_t allSettings_address;

	union
	{
		uint8_t allSettings;
		struct
		{
			uint8_t lowVoltageAlarmOn				:1;
			uint8_t highVoltageAlarmOn				:1;
			uint8_t lowVoltageAlarmBuzzerOn			:1;
			uint8_t highVoltageAlarmBuzzerOn		:1;
			uint8_t lowVoltageAlarmSnapshotOn		:1;
			uint8_t highVoltageAlarmSnapshotOn		:1;
			/* 2 more free bits here */
		};
	};
}CAR_mainBattery = {.batteryLowVoltageAlarmThreshold_address = MAIN_BATTERY_LOW_VOLTAGE_ALARM_THRESHOLD_ADDRESS,
					.batteryHighVoltageAlarmThreshold_address = MAIN_BATTERY_HIGH_VOLTAGE_ALARM_THRESHOLD_ADDRESS,
					.allSettings_address = MAIN_BATTERY_ALL_SETTINGS_ADDRESS},
CAR_auxiliaryBattery = {.batteryLowVoltageAlarmThreshold_address = AUXILIARY_BATTERY_LOW_VOLTAGE_ALARM_THRESHOLD_ADDRESS,
					.batteryHighVoltageAlarmThreshold_address = AUXILIARY_BATTERY_HIGH_VOLTAGE_ALARM_THRESHOLD_ADDRESS,
					.allSettings_address = AUXILIARY_BATTERY_ALL_SETTINGS_ADDRESS};

struct fuel_struct
{
	uint8_t fuelLowLevelWarningThreshold;

	uint16_t fuelLowLevelWarningThreshold_address;
	uint16_t allSettings_address;

	union
	{
		uint8_t allSettings;
		struct
		{
			uint8_t lowFuelLevelWarningOn			:1;
			uint8_t lowFuelLevelWarningBuzzerOn		:1;
		};
	};
}CAR_fuel = {.fuelLowLevelWarningThreshold_address = FUEL_LOW_LEVEL_WARNING_THRESHOLD_ADDRESS,
				.allSettings_address = FUEL_ALL_SETTINGS_ADDRESS};

struct oilPressure_struct
{
	uint8_t oilHighPressureAlarmThreshold;
	uint8_t oilLowPressureAlarmThreshold;

	uint16_t oilHighPressureAlarmThreshold_address;
	uint16_t oilLowPressureAlarmThreshold_address;
	uint16_t allSettings_address;

	union
	{
		uint8_t allSettings;
		struct
		{
			uint8_t oilPressureAnalogMeasurement	:1;
			uint8_t oilHighPressureAlarmOn			:1;
			uint8_t oilLowPressureAlarmOn			:1;
			uint8_t oilPressureAlarmBuzzerOn		:1;
			uint8_t oilPressureAlarmSnapshotOn		:1;
			/* 3 more free bits here */
		};
	};
}CAR_oilPressure = {.oilHighPressureAlarmThreshold_address = OIL_HIGH_PRESSURE_ALARM_THRESHOLD_ADDRESS,
					.oilLowPressureAlarmThreshold_address = OIL_LOW_PRESSURE_ALARM_THRESHOLD_ADDRESS,
					.allSettings_address = OIL_PRESSURE_ALL_SETTINGS_ADDRESS};

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */


/* USER CODE END Variables */
osThreadId My_1000ms_TaskHandle;
osThreadId My_500ms_TaskHandle;
osThreadId My_LCD_TaskHandle;
osThreadId My_GPS_TaskHandle;
osThreadId My_EEPROM_TaskHandle;
osThreadId My_DumpToEEPROMHandle;
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
void ENC_Button_LongPress_Callback(void const * argument);

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
{
#ifdef RUNTIME_STATS_TIMER_CONFIG
	HAL_TIM_Base_Start_IT(&htim7);
#endif
}

__weak unsigned long getRunTimeCounterValue(void)
{
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
  osMessageQDef(Queue_EEPROM_read, 10, EEPROM_data_struct);
  Queue_EEPROM_readHandle = osMessageCreate(osMessageQ(Queue_EEPROM_read), NULL);

  /* definition and creation of Queue_EEPROM_write */
  osMessageQDef(Queue_EEPROM_write, 10, EEPROM_data_struct);
  Queue_EEPROM_writeHandle = osMessageCreate(osMessageQ(Queue_EEPROM_write), NULL);

  /* definition and creation of Queue_error_snapshot_dump */
  osMessageQDef(Queue_error_snapshot_dump, 2, Error_Snapshot_struct);
  Queue_error_snapshot_dumpHandle = osMessageCreate(osMessageQ(Queue_error_snapshot_dump), NULL);

  /* definition and creation of Queue_diagnostic_snapshot_dump */
  osMessageQDef(Queue_diagnostic_snapshot_dump, 2, Diagnostic_Snapshot_struct);
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
  osThreadDef(My_LCD_Task, StartTaskLCD, osPriorityNormal, 0, 256);
  My_LCD_TaskHandle = osThreadCreate(osThread(My_LCD_Task), NULL);

  /* definition and creation of My_GPS_Task */
  osThreadDef(My_GPS_Task, StartTaskGPS, osPriorityNormal, 0, 128);
  My_GPS_TaskHandle = osThreadCreate(osThread(My_GPS_Task), NULL);

  /* definition and creation of My_EEPROM_Task */
  osThreadDef(My_EEPROM_Task, StartTaskEEPROM, osPriorityNormal, 0, 128);
  My_EEPROM_TaskHandle = osThreadCreate(osThread(My_EEPROM_Task), NULL);

  /* definition and creation of My_DumpToEEPROM */
  osThreadDef(My_DumpToEEPROM, StartTaskDumpToEEPROM, osPriorityBelowNormal, 0, 128);
  My_DumpToEEPROMHandle = osThreadCreate(osThread(My_DumpToEEPROM), NULL);

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
  /* USER CODE BEGIN StartTask1000ms */

	TickType_t xLastWakeTime;
	const TickType_t xFrequency = 1000;
	Error_Code error = NO_ERROR;

#ifdef BOARD_TEMPERATURE_MEASUREMENT
	int16_t tempDCDC = 0;
	int16_t tempStabilizer = 0;

	static uint8_t tempDCDC_message[6] = {SPACE_IN_ASCII};
	static uint8_t tempStabilizer_message[6] = {SPACE_IN_ASCII};

	DCDC_3V3_temperature_LCD.messageHandler = tempDCDC_message;
	Stabilizer_5V_temperature_LCD.messageHandler = tempStabilizer_message;
#endif

#ifdef BOARD_VOLTAGE_MEASUREMENT
	uint16_t voltageIn = 0;
	uint16_t voltage5V = 0;

	static uint8_t voltageIn_message[7] = {SPACE_IN_ASCII};
	static uint8_t voltage5V_message[6] = {SPACE_IN_ASCII};

	Voltage_Vin_LCD.messageHandler = voltageIn_message;
	Voltage_5V_LCD.messageHandler = voltage5V_message;
#endif

#ifdef GPS_RECEIVING
	HAL_UART_Receive_IT(&huart1, (uint8_t *)&(GPS.receivedByte), 1u);
#endif

	xLastWakeTime = xTaskGetTickCount();

	/* Infinite loop */
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
	for (;;)
	{
#ifdef BOARD_TEMPERATURE_MEASUREMENT
		error = calculate_NTC_temperature(&tempDCDC, ADC3Measures[3]/*NTC near 5V stabilizer*/, &NTC);

		if(NO_ERROR != error)
		{
			//TODO: dopisać zrzut błędu do EEPROM_BOARD z kodem błędu
		}
		else
		{
			snprintf((char*)Stabilizer_5V_temperature_LCD.messageHandler, 6, "%" PRIi16 "%cC", tempStabilizer, DEGREE_SYMBOL_LCD);
			Stabilizer_5V_temperature_LCD.size = strlen((char*)Stabilizer_5V_temperature_LCD.messageHandler);
		}

		error = calculate_NTC_temperature(&tempStabilizer, ADC3Measures[2]/*NTC near DC/DC*/, &NTC);

		if(NO_ERROR != error)
		{
			//TODO: dopisać zrzut błędu do EEPROM_BOARD z kodem błędu
		}
		else
		{
			snprintf((char*)DCDC_3V3_temperature_LCD.messageHandler, 6, "%" PRIi16 "%cC", tempDCDC, DEGREE_SYMBOL_LCD);
			DCDC_3V3_temperature_LCD.size = strlen((char*)DCDC_3V3_temperature_LCD.messageHandler);
		}
#endif

#ifdef BOARD_VOLTAGE_MEASUREMENT
		error = calculate_voltage(&voltageIn, ADC3Measures[0]/*Vin*/, MEASURE_VIN_VOLTAGE_DIVIDER_x10000);

		if(NO_ERROR != error)
		{
			//TODO: dopisać zrzut błędu do EEPROM_BOARD z kodem błędu
		}
		else
		{
			snprintf((char*)Voltage_Vin_LCD.messageHandler, 7, "%" PRIu16 ".%02" PRIu16 "V", voltageIn/100, voltageIn%100);
			Voltage_Vin_LCD.size = strlen((char*)Voltage_Vin_LCD.messageHandler);
		}

		error = calculate_voltage(&voltage5V, ADC3Measures[1]/*5V*/, MEASURE_5V_VOLTAGE_DIVIDER_x10000);

		if(NO_ERROR != error)
		{
			//TODO: dopisać zrzut błędu do EEPROM_BOARD z kodem błędu
		}
		else
		{
			snprintf((char*)Voltage_5V_LCD.messageHandler, 6, "%" PRIu16 ".%02" PRIu16 "V", voltage5V/100, voltage5V%100);
			Voltage_5V_LCD.size = strlen((char*)Voltage_5V_LCD.messageHandler);
		}
#endif

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
	const TickType_t xFrequency = 500;
	Error_Code error = NO_ERROR;

	UNUSED(error);

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

	UNUSED(error);

	uint8_t LCD_buffer[LCD.noOfRowsLCD][LCD.noOfColumnsLCD];
	memset(LCD_buffer, SPACE_IN_ASCII, (LCD.noOfRowsLCD * LCD.noOfColumnsLCD));

	uint8_t message[12] = "";
	static uint32_t count = 0;

	/************/
//	vTaskDelay(1000);
	/************/

	lcdInit(&hi2c2, LCD.addressLCD, LCD.noOfRowsLCD, LCD.noOfColumnsLCD);

	// Print text at home position 0,0
	lcdPrintStr((uint8_t*) "  Welcome on board,", 19);

	// Set cursor at zero position of line 3
	lcdSetCursorPosition(0, LCD.Row3);

	// Print text at cursor position
	lcdPrintStr((uint8_t*) "      Captain!", 14);

	/************/
	vTaskDelay(3000);
	/************/

	xLastWakeTime = xTaskGetTickCount();

	/* Infinite loop */
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
	for (;;)
	{
		memset(LCD_buffer, SPACE_IN_ASCII, (LCD.noOfRowsLCD * LCD.noOfColumnsLCD));

		snprintf((char*)message, 11, "TIM: %" PRIu32, count);

		error = copy_str_to_buffer((char*) message, (char*) LCD_buffer[LCD.Row3], 2,
				strlen((char*) message));

		error = copy_str_to_buffer((char*)DCDC_3V3_temperature_LCD.messageHandler, (char*)LCD_buffer[LCD.Row1], 5, DCDC_3V3_temperature_LCD.size);
		error = copy_str_to_buffer((char*)Stabilizer_5V_temperature_LCD.messageHandler, (char*)LCD_buffer[LCD.Row1], 15, Stabilizer_5V_temperature_LCD.size);

		error = copy_str_to_buffer((char*)"3V3: ", (char*)LCD_buffer[LCD.Row1], 0, 5);
		error = copy_str_to_buffer((char*)"5V: ", (char*)LCD_buffer[LCD.Row1], 11, 4);

		error = copy_str_to_buffer((char*)"In: ", (char*)LCD_buffer[LCD.Row2], 0, 4);
		error = copy_str_to_buffer((char*)Voltage_Vin_LCD.messageHandler, (char*)LCD_buffer[LCD.Row2], 4, Voltage_Vin_LCD.size);
		error = copy_str_to_buffer((char*)"5V: ", (char*)LCD_buffer[LCD.Row2], (4+Voltage_Vin_LCD.size+1), 4);
		error = copy_str_to_buffer((char*)Voltage_5V_LCD.messageHandler, (char*)LCD_buffer[LCD.Row2], (4+Voltage_Vin_LCD.size+1+4), Voltage_5V_LCD.size);

		error = copy_str_to_buffer((char*)GPS.forLCD.clock.messageHandler, (char*)LCD_buffer[LCD.Row4], 0, GPS.forLCD.clock.size);

		lcdSetCursorPosition(0, LCD.Row1);
		lcdPrintStr(LCD_buffer[LCD.Row1], LCD.noOfColumnsLCD);
		vTaskDelay(2);
		lcdSetCursorPosition(0, LCD.Row2);
		lcdPrintStr(LCD_buffer[LCD.Row2], LCD.noOfColumnsLCD);
		vTaskDelay(2);
		lcdSetCursorPosition(0, LCD.Row3);
		lcdPrintStr(LCD_buffer[LCD.Row3], LCD.noOfColumnsLCD);
		vTaskDelay(2);
		lcdSetCursorPosition(0, LCD.Row4);
		lcdPrintStr(LCD_buffer[LCD.Row4], LCD.noOfColumnsLCD);

		++count;

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
	uint8_t tempbuffer[3] = {'a'};

	GPS.forLCD.hours.messageHandler = GPS.message_buffers.hours;
	GPS.forLCD.minutes.messageHandler = GPS.message_buffers.minutes;
	GPS.forLCD.seconds.messageHandler = GPS.message_buffers.seconds;
	GPS.forLCD.clock.messageHandler = GPS.message_buffers.clock;

	GPS.forLCD.hours.size = 2;
	GPS.forLCD.minutes.size = 2;
	GPS.forLCD.seconds.size = 2;
#endif

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
#endif

#ifdef GPS_LCD_PRINT

		if(TRUE == GPS.TimeReady)
		{
			error = copy_buffer_to_str((char*)GPS.rawData.UTC, (char*)tempbuffer, 0, 2);
			tempHour = (uint8_t)atoi((char*)tempbuffer);
			tempHour += GPS.TImeZoneAdjPoland;

			if(24 <= tempHour)
				tempHour -= 24;

			if(10 <= tempHour)
			{
				itoa(tempHour, (char*)tempbuffer, 10/*decimal system*/);
			}
			else
			{
				tempbuffer[0] = '0';
				itoa(tempHour, (char*)&tempbuffer[1], 10);
			}
			copy_buffer_to_str((char*)tempbuffer, (char*)GPS.message_buffers.hours, 0, 2);

			copy_buffer_to_str((char*)GPS.rawData.UTC, (char*)GPS.message_buffers.minutes, 2, 2);

			copy_buffer_to_str((char*)GPS.rawData.UTC, (char*)GPS.message_buffers.seconds, 4, 2);

			GPS.message_buffers.clock[2] = ':';
			GPS.message_buffers.clock[5] = ':';
			copy_str_to_buffer((char*)GPS.message_buffers.hours, (char*)GPS.message_buffers.clock, 0, 2);
			copy_str_to_buffer((char*)GPS.message_buffers.minutes, (char*)GPS.message_buffers.clock, 3, 2);
			copy_str_to_buffer((char*)GPS.message_buffers.seconds, (char*)GPS.message_buffers.clock, 6, 2);
			GPS.forLCD.clock.size = strlen((char*)GPS.forLCD.clock.messageHandler);
		}
		else
		{

		}
//		snprintf((char*)Voltage_5V_LCD.messageHandler, 6, "%" PRIu16 ".%02" PRIu16 "V", voltage5V/100, voltage5V%100);
//		Voltage_Vin_LCD.size = strlen((char*)Voltage_Vin_LCD.messageHandler);

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

	EEPROM_data_struct EEPROMData;

#ifdef RUNTIME_STATS_QUEUES
	vQueueAddToRegistry(Queue_EEPROM_writeHandle, QUEUE_EEPROM_WRITE_NAME);
	vQueueAddToRegistry(Queue_EEPROM_readHandle, QUEUE_EEPROM_READ_NAME);
#endif

	xLastWakeTime = xTaskGetTickCount();

  /* Infinite loop */
  for(;;)
  {
	  if(pdTRUE == xQueueReceive(Queue_EEPROM_readHandle, &EEPROMData, (TickType_t)0))
	  {
		  error = ReadEEPROM_DMA(EEPROMData.EEPROMParameters, &EEPROMData);
		  if(NO_ERROR == error)
		  {
			  EEPROMData.isReady = True;//TODO: mem error dump
		  }

	  }
	  else
	  {
		  if(pdTRUE == xQueueReceive(Queue_EEPROM_writeHandle, &EEPROMData, (TickType_t)0))
		  {
			  error = WriteEEPROM_DMA(EEPROMData.EEPROMParameters, &EEPROMData);
			  if(NO_ERROR == error)
			  {
				  EEPROMData.isReady = True;//TODO: mem error dump
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

//	Diagnostic_Snapshot_struct Diag_handler = {};
//	Error_Snapshot_struct Error_handler = {};
	int rozmiar = sizeof(Diagnostic_Snapshot_struct);

	struct DiagnosticDataToSend_struct
	{
		union
		{
			uint8_t data[12];
			Diagnostic_Snapshot_struct diag_mess_from_queue;
		};
		EEPROM_data_struct DiagnosticData;
	} DiagnosticDataToSend = {.DiagnosticData.EEPROMParameters = &EEPROM_car,
								.DiagnosticData.data = DiagnosticDataToSend.data,
								.diag_mess_from_queue = {}};

	struct ErrorDataToSend_struct
	{
		union
		{
			uint8_t data[12];
			Error_Snapshot_struct error_mess_from_queue;
		};
		EEPROM_data_struct ErrorData;
	} ErrorDataToSend = {.ErrorData.EEPROMParameters = &EEPROM_board,
							.error_mess_from_queue = {}};

//	UNUSED(EEPROMData);

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


		  if(NO_ERROR == error)
		  {
  //			  EEPROMDataHandle->isReady = True;
		  }

	  }
	  else
	  {
		  if(pdTRUE == xQueueReceive(Queue_error_snapshot_dumpHandle, &(ErrorDataToSend.error_mess_from_queue), (TickType_t)0))
		  {
			  error = WriteEEPROM_DMA(ErrorDataToSend.ErrorData.EEPROMParameters, &(ErrorDataToSend.ErrorData));
			  if(NO_ERROR == error)
			  {
	//			  EEPROMDataHandle->isReady = True;
			  }
			  vTaskDelay(5);
		  }
	  }

	  	  vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
  /* USER CODE END StartTaskDumpToEEPROM */
}

/* ENC_Button_LongPress_Callback function */
void ENC_Button_LongPress_Callback(void const * argument)
{
  /* USER CODE BEGIN ENC_Button_LongPress_Callback */
//  ++licznikTimera;
  ENC_button.allFlags |= 0b00000110; /* Set first and second bit to q to
   	   	   	   	   	   	   	   	   	   	   indicate the long press on both
   	   	   	   	   	   	   	   	   	   	   bits */
  /* USER CODE END ENC_Button_LongPress_Callback */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
