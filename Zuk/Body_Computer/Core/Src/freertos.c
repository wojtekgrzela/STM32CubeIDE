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
extern TIM_HandleTypeDef htim8;
extern SD_HandleTypeDef hsd;

#ifdef RUNTIME_STATS_TIMER_CONFIG
extern TIM_HandleTypeDef htim7;
#endif

/* Handlers for SD Memory Card usage (TASK: My_DumpToSDCard) */
extern FIL SDFile;
extern FATFS SDFatFS;
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
{ //.TimeZoneAdjPoland = 2,/* .homeLatitude = 52.093731, .homeLongitude = 20.661411,*/
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
carTemperature_type waterTemperatureValue = 0.0;
LCD_message waterTemperatureValueForLCD = {.messageHandler = NULL, .size = 0, .messageReadyFLAG = 0};

oilTempSettings_struct CAR_oilTemp = {0};
carTemperature_type oilTemperatureValue = 0.0;
LCD_message oilTemperatureValueForLCD = {.messageHandler = NULL, .size = 0, .messageReadyFLAG = 0};

oilPressureSettings_struct CAR_oilPressure = {0};
#ifdef OIL_PRESSURE_ANALOG_SENSOR
carOilAnalogPressure_type oilPressureValue = 0.0;
LCD_message oilPressureValueForLCD = {.messageHandler = NULL, .size = 0, .messageReadyFLAG = 0};
#endif
#ifdef OIL_PRESSURE_BINARY_SENSOR
carOilBinaryPressure_type oilPressureValueBinary = NOK;
LCD_message oilPressureValueBinaryForLCD = {.messageHandler = NULL, .size = 0, .messageReadyFLAG = 0};
#endif


batterySettings_struct CAR_mainBattery = {0};
carVoltage_type mainBatteryVoltageValue = 0.0;
LCD_message mainBatteryVoltageValueForLCD = {.messageHandler = NULL, .size = 0, .messageReadyFLAG = 0};

batterySettings_struct CAR_auxiliaryBattery = {0};
carVoltage_type auxiliaryBatteryVoltageValue = 0.0;
LCD_message auxiliaryBatteryVoltageValueForLCD = {.messageHandler = NULL, .size = 0, .messageReadyFLAG = 0};

fuelSettings_struct CAR_fuel = {0};
cafFuelLevel_type fuelLevelValue = 0.0;
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
boardVoltage_type voltage3V3 = 0.0;
boardVoltage_type voltage5V = 0.0;
boardVoltage_type voltageIn = 0.0;
LCD_message voltage3V3ForLCD = {.messageHandler = NULL, .size = 0, .messageReadyFLAG = 0};
LCD_message voltage5VForLCD = {.messageHandler = NULL, .size = 0, .messageReadyFLAG = 0};
LCD_message voltageInForLCD = {.messageHandler = NULL, .size = 0, .messageReadyFLAG = 0};

boardTemperaturesSettings_struct BOARD_temperature = {0};
boardTemperature_type temperature3V3DCDC = 0.0;
boardTemperature_type temperature5VDCDC = 0.0;
LCD_message temperature3V3DCDCForLCD = {.messageHandler = NULL, .size = 0, .messageReadyFLAG = 0};
LCD_message temperature5VDCDCForLCD = {.messageHandler = NULL, .size = 0, .messageReadyFLAG = 0};

buzzerMainSettings_struct BUZZER_settings = {0};
LCDMainSettings_struct LCD_MainSettings = {0};
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* Car information (Key, Engine status etc.) */
CarStateinfo_type CarStateInfo = {0};

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Constant value limits and check - hard-coded, to be changed only by recompilation */
GlobalValuesLimits_struct GlobalValuesLimits = {
	.polishTimeAdj_min = 1,
	.polishTimeAdj_max = 2,
	.timeZoneAdj_min = -12,
	.timeZoneAdj_max = 12,

	.waterHighTempWarningThreshold_min	= 50,
	.waterHighTempWarningThreshold_max	= 150,
	.waterHighTempAlarmThreshold_min	= 50,
	.waterHighTempAlarmThreshold_max	= 150,
	.waterHighTempFanOnThreshold_min	= 50,
	.waterHighTempFanOnThreshold_max	= 150,
	.waterHighTempFanOffThreshold_min	= 50,
	.waterHighTempFanOffThreshold_max	= 150,

	.oilHighTempWarningThreshold_min	= 50,
	.oilHighTempWarningThreshold_max	= 150,
	.oilHighTempAlarmThreshold_min		= 50,
	.oilHighTempAlarmThreshold_max		= 150,

	.oilHighPressureAlarmThreshold_min	= 0.1,
	.oilHighPressureAlarmThreshold_max	= 7.5,
	.oilLowPressureAlarmThreshold_min	= 0.1,
	.oilLowPressureAlarmThreshold_max	= 7.5,

	.batteryLowVoltageAlarmThreshold_min	= 8.00,
	.batteryLowVoltageAlarmThreshold_max	= 18.00,
	.batteryHighVoltageAlarmThreshold_min	= 8.00,
	.batteryHighVoltageAlarmThreshold_max	= 18.00,

	.fuelLowLevelWarningThreshold_min	= 5,
	.fuelLowLevelWarningThreshold_max	= 80,

	.board3V3SupplyLowThreshold_min		= 1.80,
	.board3V3SupplyLowThreshold_max		= 4.00,
	.board3V3SupplyHighThreshold_min	= 1.80,
	.board3V3SupplyHighThreshold_max	= 4.00,
	.board5VSupplyLowThreshold_min		= 4.00,
	.board5VSupplyLowThreshold_max		= 6.00,
	.board5VSupplyHighThreshold_min		= 4.00,
	.board5VSupplyHighThreshold_max		= 6.00,
	.boardVinSupplyLowThreshold_min		= 6.00,
	.boardVinSupplyLowThreshold_max		= 17.00,

	.board5VDCDCTemperatureHighThreshold_min	= 20,
	.board5VDCDCTemperatureHighThreshold_max	= 150,
	.board3V3DCDCTemperatureHighThreshold_min	= 20,
	.board3V3DCDCTemperatureHighThreshold_max	= 150,

	.homeScreen_min						= MainMenu_Layer,
	.homeScreen_max						= JarvisInfo_Layer,
	.autoHomeReturnTime_min				= 3,
	.autoHomeReturnTime_max				= 255,
	.backlightLevel_min					= 1,
	.backlightLevel_max					= 10,
	.secondsToAutoTurnOffBacklight_min	= 2,
	.secondsToAutoTurnOffBacklight_max	= 255,
	.autoBacklightOffHourStart_min		= 0,
	.autoBacklightOffHourStart_max		= 24,
	.autoBacklightOffHourEnd_min		= 0,
	.autoBacklightOffHourEnd_max		= 24
};
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
osThreadId My_DiagCheckHandle;
osThreadId my_AlarmControlHandle;
osMessageQId Queue_EEPROM_readHandle;
osMessageQId Queue_EEPROM_writeHandle;
osMessageQId Queue_error_snapshot_dumpHandle;
osMessageQId Queue_diagnostic_snapshot_dumpHandle;
osTimerId My_Timer_ENC_ButtonHandle;
osTimerId My_Timer_carWaterTempValueCheckHandle;
osTimerId My_Timer_carOilTempValueCheckHandle;
osTimerId My_Timer_carOilAnalogPressureValueCheckHandle;
osTimerId My_Timer_carMainBattVoltageValueCheckHandle;
osTimerId My_Timer_carAuxBattVoltageValueCheckHandle;
osTimerId My_Timer_carFuelLevelValueCheckHandle;
osTimerId My_Timer_board3V3VoltageValueCheckHandle;
osTimerId My_Timer_board5VVoltageValueCheckHandle;
osTimerId My_Timer_boardVinVoltageValueCheckHandle;
osTimerId My_Timer_board3V3TempValueCheckHandle;
osTimerId My_Timer_board5VTempValueCheckHandle;
osTimerId My_Timer_carOilBinaryPressureValueCheckHandle;
osTimerId My_Timer_BuzzerHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartTask1000ms(void const * argument);
void StartTask500ms(void const * argument);
extern void StartTaskLCD(void const * argument);
extern void StartTaskGPS(void const * argument);
extern void StartTaskEEPROM(void const * argument);
extern void StartTaskDumpToEEPROM(void const * argument);
void StartTaskDumpToSDCard(void const * argument);
void StartTask250ms(void const * argument);
void StartTask50ms(void const * argument);
extern void StartTaskDiagCheck(void const * argument);
extern void StartTaskAlarmControl(void const * argument);
void ENC_Button_LongPress_Callback(void const * argument);
extern void Timer_carWaterTempValueCheck(void const * argument);
extern void Timer_carOilTempValueCheck(void const * argument);
extern void Timer_carOilAnalogPressureValueCheck(void const * argument);
extern void Timer_carMainBattVoltageValueCheck(void const * argument);
extern void Timer_carAuxBattVoltageValueCheck(void const * argument);
extern void Timer_carFuelLevelValueCheck(void const * argument);
extern void Timer_board3V3VoltageValueCheck(void const * argument);
extern void Timer_board5VVoltageValueCheck(void const * argument);
extern void Timer_boardVinVoltageValueCheck(void const * argument);
extern void Timer_board3V3TempValueCheck(void const * argument);
extern void Timer_board5VTempValueCheck(void const * argument);
extern void Timer_carOilBinaryPressureValueCheck(void const * argument);
extern void Timer_Buzzer(void const * argument);

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

  /* definition and creation of My_Timer_carWaterTempValueCheck */
  osTimerDef(My_Timer_carWaterTempValueCheck, Timer_carWaterTempValueCheck);
  My_Timer_carWaterTempValueCheckHandle = osTimerCreate(osTimer(My_Timer_carWaterTempValueCheck), osTimerOnce, NULL);

  /* definition and creation of My_Timer_carOilTempValueCheck */
  osTimerDef(My_Timer_carOilTempValueCheck, Timer_carOilTempValueCheck);
  My_Timer_carOilTempValueCheckHandle = osTimerCreate(osTimer(My_Timer_carOilTempValueCheck), osTimerOnce, NULL);

  /* definition and creation of My_Timer_carOilAnalogPressureValueCheck */
  osTimerDef(My_Timer_carOilAnalogPressureValueCheck, Timer_carOilAnalogPressureValueCheck);
  My_Timer_carOilAnalogPressureValueCheckHandle = osTimerCreate(osTimer(My_Timer_carOilAnalogPressureValueCheck), osTimerOnce, NULL);

  /* definition and creation of My_Timer_carMainBattVoltageValueCheck */
  osTimerDef(My_Timer_carMainBattVoltageValueCheck, Timer_carMainBattVoltageValueCheck);
  My_Timer_carMainBattVoltageValueCheckHandle = osTimerCreate(osTimer(My_Timer_carMainBattVoltageValueCheck), osTimerOnce, NULL);

  /* definition and creation of My_Timer_carAuxBattVoltageValueCheck */
  osTimerDef(My_Timer_carAuxBattVoltageValueCheck, Timer_carAuxBattVoltageValueCheck);
  My_Timer_carAuxBattVoltageValueCheckHandle = osTimerCreate(osTimer(My_Timer_carAuxBattVoltageValueCheck), osTimerOnce, NULL);

  /* definition and creation of My_Timer_carFuelLevelValueCheck */
  osTimerDef(My_Timer_carFuelLevelValueCheck, Timer_carFuelLevelValueCheck);
  My_Timer_carFuelLevelValueCheckHandle = osTimerCreate(osTimer(My_Timer_carFuelLevelValueCheck), osTimerOnce, NULL);

  /* definition and creation of My_Timer_board3V3VoltageValueCheck */
  osTimerDef(My_Timer_board3V3VoltageValueCheck, Timer_board3V3VoltageValueCheck);
  My_Timer_board3V3VoltageValueCheckHandle = osTimerCreate(osTimer(My_Timer_board3V3VoltageValueCheck), osTimerOnce, NULL);

  /* definition and creation of My_Timer_board5VVoltageValueCheck */
  osTimerDef(My_Timer_board5VVoltageValueCheck, Timer_board5VVoltageValueCheck);
  My_Timer_board5VVoltageValueCheckHandle = osTimerCreate(osTimer(My_Timer_board5VVoltageValueCheck), osTimerOnce, NULL);

  /* definition and creation of My_Timer_boardVinVoltageValueCheck */
  osTimerDef(My_Timer_boardVinVoltageValueCheck, Timer_boardVinVoltageValueCheck);
  My_Timer_boardVinVoltageValueCheckHandle = osTimerCreate(osTimer(My_Timer_boardVinVoltageValueCheck), osTimerOnce, NULL);

  /* definition and creation of My_Timer_board3V3TempValueCheck */
  osTimerDef(My_Timer_board3V3TempValueCheck, Timer_board3V3TempValueCheck);
  My_Timer_board3V3TempValueCheckHandle = osTimerCreate(osTimer(My_Timer_board3V3TempValueCheck), osTimerOnce, NULL);

  /* definition and creation of My_Timer_board5VTempValueCheck */
  osTimerDef(My_Timer_board5VTempValueCheck, Timer_board5VTempValueCheck);
  My_Timer_board5VTempValueCheckHandle = osTimerCreate(osTimer(My_Timer_board5VTempValueCheck), osTimerOnce, NULL);

  /* definition and creation of My_Timer_carOilBinaryPressureValueCheck */
  osTimerDef(My_Timer_carOilBinaryPressureValueCheck, Timer_carOilBinaryPressureValueCheck);
  My_Timer_carOilBinaryPressureValueCheckHandle = osTimerCreate(osTimer(My_Timer_carOilBinaryPressureValueCheck), osTimerOnce, NULL);

  /* definition and creation of My_Timer_Buzzer */
  osTimerDef(My_Timer_Buzzer, Timer_Buzzer);
  My_Timer_BuzzerHandle = osTimerCreate(osTimer(My_Timer_Buzzer), osTimerOnce, NULL);

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
  osThreadDef(My_LCD_Task, StartTaskLCD, osPriorityNormal, 0, 512);
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

  /* definition and creation of My_DiagCheck */
  osThreadDef(My_DiagCheck, StartTaskDiagCheck, osPriorityAboveNormal, 0, 256);
  My_DiagCheckHandle = osThreadCreate(osThread(My_DiagCheck), NULL);

  /* definition and creation of my_AlarmControl */
  osThreadDef(my_AlarmControl, StartTaskAlarmControl, osPriorityIdle, 0, 256);
  my_AlarmControlHandle = osThreadCreate(osThread(my_AlarmControl), NULL);

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
  MX_USB_DEVICE_Init();
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
			snprintf((char*)temperature5VDCDCForLCD.messageHandler, 6, "%" PRIi16 "%cC", (uint16_t)temperature5VDCDC, DEGREE_SYMBOL_LCD);
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
			snprintf((char*)temperature3V3DCDCForLCD.messageHandler, 6, "%" PRIi16 "%cC", (uint16_t)temperature3V3DCDC, DEGREE_SYMBOL_LCD);
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
		tripMileageForLCD.messageReadyFLAG = TRUE;

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
	uint8_t firstFewRuns = 0u;

	/* Stop My_DiagCheck Task because measurements are not ready yet! */
	vTaskSuspend(My_DiagCheckHandle);

	/* For calculating the engine parameters (counting the mean value) */
	static volatile carTemperature_type EngineWaterTemperatureTable[NO_OF_ENGINE_WATER_TEMP_MEASUREMENTS_ADDED] = {0};
	static volatile carTemperature_type EngineWaterTemperatureMovingAverage[NO_OF_ENGINE_WATER_TEMP_MEASUREMENTS_ADDED] = {0};
	static uint8_t i_waterTemp = 0;
	static carTemperature_type tempEngineWaterTemp = 0.0;
	carTemperature_type waterTemperatureValue_beforeMovingAverage = 0.0;
	static uint8_t waterTemperatureValueMessage[4] = "";
	waterTemperatureValueForLCD.messageHandler = waterTemperatureValueMessage;

	static volatile carTemperature_type EngineOilTemperatureTable[NO_OF_ENGINE_OIL_TEMP_MEASUREMENTS_ADDED] = {0};
	static volatile carTemperature_type EngineOilTemperatureTableMovingAverage[NO_OF_ENGINE_OIL_TEMP_MEASUREMENTS_ADDED] = {0};
	static uint8_t i_oilTemp = 0;
	static carTemperature_type tempEngineOilTemp = 0.0;
	carTemperature_type oilTemperatureValue_beforeMovingAverage = 0.0;
	static uint8_t oilTemperatureValueMessage[4] = "";
	oilTemperatureValueForLCD.messageHandler = oilTemperatureValueMessage;

#ifdef OIL_PRESSURE_ANALOG_SENSOR
	static volatile carOilAnalogPressure_type EngineOilPressureTable[NO_OF_ENGINE_OIL_PRESSURE_MEASUREMENTS_ADDED] = {0};
//	static volatile float EngineOilPressureTableMovingAverage[NO_OF_ENGINE_OIL_PRESSURE_MEASUREMENTS_ADDED] = {0};
	static uint8_t i_oilPressure = 0;
	static carOilAnalogPressure_type tempEngineOilPressure = 0.0;
	static uint8_t oilPressureValueMessage[4] = "";
	oilPressureValueForLCD.messageHandler = oilPressureValueMessage;
#endif

#ifdef OIL_PRESSURE_BINARY_SENSOR
	static uint8_t oilPressureValueBinaryMessage[4] = "";	//OK, NOK
	oilPressureValueBinaryForLCD.messageHandler = oilPressureValueBinaryMessage;
#endif

	static volatile carVoltage_type MainBatteryVoltageValueTable[NO_OF_MAIN_BATTERY_VOLTAGE_MEASUREMENTS_ADDED] = {0};
//	static volatile float MainBatteryVoltageValueTableMovingAverage[NO_OF_MAIN_BATTERY_VOLTAGE_MEASUREMENTS_ADDED] = {0};
	static uint8_t i_mainBateryVoltage = 0;
	static carVoltage_type tempMainBatteryVoltage = 0.0;
	static uint8_t mainBatteryVoltageValueMessage[6] = "";
	mainBatteryVoltageValueForLCD.messageHandler = mainBatteryVoltageValueMessage;

	static volatile carVoltage_type AuxiliaryBatteryVoltageValueTable[NO_OF_AUXILIARY_BATTERY_VOLTAGE_MEASUREMENTS_ADDED] = {0};
//	static volatile float AuxiliaryBatteryVoltageValueTableMovingAverage[NO_OF_AUXILIARY_BATTERY_VOLTAGE_MEASUREMENTS_ADDED] = {0};
	static uint8_t i_auxBatteryVoltage = 0;
	static carVoltage_type tempAuxBatteryVoltage = 0.0;
	static uint8_t auxiliaryBatteryVoltageValueMessage[6] = "";
	auxiliaryBatteryVoltageValueForLCD.messageHandler = auxiliaryBatteryVoltageValueMessage;

	static volatile cafFuelLevel_type FuelLevelValueTable[NO_OF_FUEL_LEVEL_MEASUREMENTS_ADDED] = {0};
	static volatile cafFuelLevel_type FuelLevelValueTableMovingAverage[NO_OF_FUEL_LEVEL_MEASUREMENTS_ADDED] = {0};
	static uint8_t i_fuelLevel = 0;
	static cafFuelLevel_type tempFuelLevel = 0.0;
	cafFuelLevel_type fuelLevelValue_beforeMovingAverage = 0.0;
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

//			waterTemperatureValue = tempEngineWaterTemp / NO_OF_ENGINE_WATER_TEMP_MEASUREMENTS_ADDED;
//			waterTemperatureValue

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
		if(GPIO_PIN_RESET == HAL_GPIO_ReadPin(GPIO_PORT_OIL_PRESSURE_SENSOR, GPIO_PIN_OIL_PRESSURE_SENSOR))
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
			tempMainBatteryVoltage = 0;
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
			tempAuxBatteryVoltage = 0;
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

		/* Release My_DiagCheck Task after some measurements first */
		(NUMBER_OF_MEASUREMENTS_BEFORE_DIAGNOSTIC <= firstFewRuns) ? vTaskResume(My_DiagCheckHandle) : ++firstFewRuns;

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

	ENC_button.allFlags |= 0b00010110; /* Set 1st, 2nd, 3rd bit to 1 to
	 	 	 	 	 	 	 	 	indicate the long press on bits:
	 	 	 	 	 	 	 	 	longPressDetected,
	 	 	 	 	 	 	 	 	longPressDetectedForISR,
	 	 	 	 	 	 	 	 	longPressDetectedBuzzer */

  /* USER CODE END ENC_Button_LongPress_Callback */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
