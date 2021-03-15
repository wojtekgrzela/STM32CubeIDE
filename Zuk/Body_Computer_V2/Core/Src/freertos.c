/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c3;
extern TIM_HandleTypeDef htim8;
extern SD_HandleTypeDef hsd;

#ifdef RUNTIME_STATS_TIMER_CONFIG
extern TIM_HandleTypeDef htim7;
#endif
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* For debugging - task time etc. */
volatile uint32_t Tim7_Counter_100us = 0u;

/* Engine and Car Parameters */
volatile uint32_t RPM_counter = 0u;
volatile uint32_t SPEED_counter = 0u;

/* For Signaling ENC button and scroll for the menu */
volatile ENCButton_struct ENC_button_menu =
	{ 0 };
volatile int8_t EncoderCounterMenuDiff = 0;

/* For Signaling ENC button and scroll for the cruise control */
volatile ENCButton_struct ENC_button_cruise =
	{ 0 };
volatile int8_t EncoderCounterCruiseDiff = 0;

/* For measurements from ADC1 and ADC3 */
volatile uint16_t ADC1Measures[NO_OF_ADC1_MEASURES] =
	{ 0u };
volatile uint16_t ADC3Measures[NO_OF_ADC3_MEASURES] =
	{ 0u };

/* For GPS data, buffering, messages for LCD */
GPS_data_struct GPS =
	{    //.TimeZoneAdjPoland = 2,/* .homeLatitude = 52.093731, .homeLongitude = 20.661411,*/
		.forLCD.hours.messageReadyFLAG = FALSE,
		.forLCD.minutes.messageReadyFLAG = FALSE,
		.forLCD.seconds.messageReadyFLAG = FALSE,
		.forLCD.clock.messageReadyFLAG = FALSE,
		.forLCD.latitude.messageReadyFLAG = FALSE,
		.forLCD.latitudeIndicator.messageReadyFLAG = FALSE,
		.forLCD.longitude.messageReadyFLAG = FALSE,
		.forLCD.longitudeIndicator.messageReadyFLAG = FALSE,
		.forLCD.status.messageReadyFLAG = FALSE,
		.forLCD.satellitesUsed.messageReadyFLAG =
		FALSE,
		.forLCD.altitude.messageReadyFLAG = FALSE,
		.forLCD.speed.messageReadyFLAG = FALSE };

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* For SDCard parameters, settings, information */
SDCard_info_struct SDCard_info =
	{ 0 };

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* For LCD parameters and settings */
LCD_parameters_struct LCD =
	{ 	.addressLCD = 0x27,
		.noOfRowsLCD = NO_OF_ROWS_IN_LCD,
		.noOfColumnsLCD = NO_OF_COLUMNS_IN_LCD };

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* For measuring Board temperatures with usage of NTC parameters */
NTC_parameters_struct NTC =
	{ 	.Beta = 4250.0,
		.R25 = 100000.0,
		.Rgnd = 10000.0,
		.T25 = 298.0,
		.beta_x_T25 = 1266500.0 };

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Counter for CAR EEPROM */
CAR_EEPROM_counters_struct CAR_EEPROM_counters =
	{ 0 };

/* For CAR EEPROM usage, its addressing and blocking */
EEPROM_parameters_struct EEPROM_car =
	{ 	.EEPROM_hi2c = &hi2c3,
		.address = EEPROM_CAR_ADDRESS,
		.pin = EEPROM_CAR_BLOCK_PIN,
		.port = EEPROM_CAR_BLOCK_PORT };

/* CAR SETTINGS and VALUES*/
waterTempSettings_struct CAR_waterTemp =
	{ 0 };
carTemperature_type waterTemperatureValue = 0.0;
LCD_message waterTemperatureValueForLCD =
	{ 	.messageHandler = NULL,
		.size = 0,
		.messageReadyFLAG = 0 };

oilTempSettings_struct CAR_oilTemp =
	{ 0 };
carTemperature_type oilTemperatureValue = 0.0;
LCD_message oilTemperatureValueForLCD =
	{ 	.messageHandler = NULL,
		.size = 0,
		.messageReadyFLAG = 0 };

oilPressureSettings_struct CAR_oilPressure =
	{ 0 };
#ifdef OIL_PRESSURE_ANALOG_SENSOR
carOilAnalogPressure_type oilPressureValue = 0.0;
LCD_message oilPressureValueForLCD =
	{ 	.messageHandler = NULL,
		.size = 0,
		.messageReadyFLAG = 0 };
#endif
#ifdef OIL_PRESSURE_BINARY_SENSOR
carOilBinaryPressure_type oilPressureValueBinary = NOK;
LCD_message oilPressureValueBinaryForLCD =
	{ 	.messageHandler = NULL,
		.size = 0,
		.messageReadyFLAG = 0 };
#endif

batterySettings_struct CAR_mainBattery =
	{ 0 };
carVoltage_type mainBatteryVoltageValue = 0.0;
LCD_message mainBatteryVoltageValueForLCD =
	{ 	.messageHandler = NULL,
		.size = 0,
		.messageReadyFLAG = 0 };

batterySettings_struct CAR_auxiliaryBattery =
	{ 0 };
carVoltage_type auxiliaryBatteryVoltageValue = 0.0;
LCD_message auxiliaryBatteryVoltageValueForLCD =
	{ 	.messageHandler = NULL,
		.size = 0,
		.messageReadyFLAG = 0 };

fuelSettings_struct CAR_fuel =
	{ 0 };
cafFuelLevel_type fuelLevelValue = 0.0;
LCD_message fuelLevelValueForLCD =
	{ 	.messageHandler = NULL,
		.size = 0,
		.messageReadyFLAG = 0 };
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Counters for BOARD EEPROM */
BOARD_EEPROM_counters_struct BOARD_EEPROM_counters =
	{ 0 };

/* For BOARD EEPROM usage, its addressing and blocking */
EEPROM_parameters_struct EEPROM_board =
	{ 	.EEPROM_hi2c = &hi2c3,
		.address = EEPROM_BOARD_ADDRESS,
		.pin = EEPROM_BOARD_BLOCK_PIN,
		.port = EEPROM_BOARD_BLOCK_PORT };

/* CAR SETTINGS and VALUES*/
boardVoltagesSettings_struct BOARD_voltage =
	{ 0 };
boardVoltage_type voltage3V3 = 0.0;
boardVoltage_type voltage5V = 0.0;
boardVoltage_type voltageIn = 0.0;

LCD_message voltage3V3ForLCD =
	{ 	.messageHandler = NULL,
		.size = 0,
		.messageReadyFLAG = 0 };
LCD_message voltage5VForLCD =
	{ 	.messageHandler = NULL,
		.size = 0,
		.messageReadyFLAG = 0 };
LCD_message voltageInForLCD =
	{ 	.messageHandler = NULL,
		.size = 0,
		.messageReadyFLAG = 0 };

boardTemperaturesSettings_struct BOARD_temperature =
	{ 0 };
boardTemperature_type temperature3V3DCDC = 0.0;
boardTemperature_type temperature5VDCDC = 0.0;
boardTemperature_type temperatureHBridge = 0.0;

LCD_message temperature3V3DCDCForLCD =
	{ 	.messageHandler = NULL,
		.size = 0,
		.messageReadyFLAG = 0 };
LCD_message temperature5VDCDCForLCD =
	{ 	.messageHandler = NULL,
		.size = 0,
		.messageReadyFLAG = 0 };
LCD_message temperatureHBridgeForLCD =
	{ 	.messageHandler = NULL,
		.size = 0,
		.messageReadyFLAG = 0 };

buzzerMainSettings_struct BUZZER_settings =
	{ 0 };
LCDMainSettings_struct LCD_MainSettings =
	{ 0 };
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Counters for FRAM */

/* For FRAMM usage, its addressing and blocking */
EEPROM_parameters_struct FRAM_parameters =
	{ 	.EEPROM_hi2c = &hi2c1,
		.address = FRAM_ADDRESS,
		.pin = FRAM_BLOCK_PIN,
		.port = FRAM_BLOCK_PORT };

CAR_mileage_struct CAR_mileage =
	{ 0 };
LCD_message totalMileageForLCD =
	{ 	.messageHandler = NULL,
		.size = 0,
		.messageReadyFLAG = 0 };
LCD_message tripMileageForLCD =
	{ 	.messageHandler = NULL,
		.size = 0,
		.messageReadyFLAG = 0 };
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* Car information (Key, Engine status etc.) */
CarStateinfo_type CarStateInfo =
	{ 0 };
LCD_message RPMForLCD =
	{ 	.messageHandler = NULL,
		.size = 0,
		.messageReadyFLAG = 0 };
LCD_message SPEEDForLCD =
	{ 	.messageHandler = NULL,
		.size = 0,
		.messageReadyFLAG = 0 };

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Constant value limits and check - hard-coded, to be changed only by recompilation */
GlobalValuesLimits_struct GlobalValuesLimits =
	{ 	.polishTimeAdj_min = 1,
		.polishTimeAdj_max = 2,
		.timeZoneAdj_min = -12,
		.timeZoneAdj_max = 12,

		.waterHighTempWarningThreshold_min = 50,
		.waterHighTempWarningThreshold_max = 150,
		.waterHighTempAlarmThreshold_min = 50,
		.waterHighTempAlarmThreshold_max = 150,
		.waterHighTempFanOnThreshold_min = 50,
		.waterHighTempFanOnThreshold_max = 150,
		.waterHighTempFanOffThreshold_min = 50,
		.waterHighTempFanOffThreshold_max = 150,

		.oilHighTempWarningThreshold_min = 50,
		.oilHighTempWarningThreshold_max = 150,
		.oilHighTempAlarmThreshold_min = 50,
		.oilHighTempAlarmThreshold_max = 150,

		.oilHighPressureAlarmThreshold_min = 0.1,
		.oilHighPressureAlarmThreshold_max = 7.5,
		.oilLowPressureAlarmThreshold_min = 0.1,
		.oilLowPressureAlarmThreshold_max = 7.5,

		.batteryLowVoltageAlarmThreshold_min = 8.00,
		.batteryLowVoltageAlarmThreshold_max = 18.00,
		.batteryHighVoltageAlarmThreshold_min = 8.00,
		.batteryHighVoltageAlarmThreshold_max = 18.00,

		.fuelLowLevelWarningThreshold_min = 5,
		.fuelLowLevelWarningThreshold_max = 80,

		.board3V3SupplyLowThreshold_min = 1.80,
		.board3V3SupplyLowThreshold_max = 4.00,
		.board3V3SupplyHighThreshold_min = 1.80,
		.board3V3SupplyHighThreshold_max = 4.00,
		.board5VSupplyLowThreshold_min = 4.00,
		.board5VSupplyLowThreshold_max = 6.00,
		.board5VSupplyHighThreshold_min = 4.00,
		.board5VSupplyHighThreshold_max = 6.00,
		.boardVinSupplyLowThreshold_min = 6.00,
		.boardVinSupplyLowThreshold_max = 17.00,

		.board5VDCDCTemperatureHighThreshold_min = 20,
		.board5VDCDCTemperatureHighThreshold_max = 150,
		.board3V3DCDCTemperatureHighThreshold_min = 20,
		.board3V3DCDCTemperatureHighThreshold_max = 150,
		.boardHBridgeTemperatureHighThreshold_min = 20,
		.boardHBridgeTemperatureHighThreshold_max = 150,

		.homeScreen_min = MainMenu_Layer,
		.homeScreen_max = JarvisInfo_Layer,
		.autoHomeReturnTime_min = 3,
		.autoHomeReturnTime_max = 255,
		.backlightLevel_min = 1,
		.backlightLevel_max = 10,
		.secondsToAutoTurnOffBacklight_min = 2,
		.secondsToAutoTurnOffBacklight_max = 255,
		.autoBacklightOffHourStart_min = 0,
		.autoBacklightOffHourStart_max = 24,
		.autoBacklightOffHourEnd_min = 0,
		.autoBacklightOffHourEnd_max = 24 };
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */


/* USER CODE END Variables */
osThreadId My_StartUp_TaskHandle;
osThreadId My_CruiseCntrlHandle;
osThreadId My_500ms_TaskHandle;
osThreadId My_LCD_TaskHandle;
osThreadId My_GPS_TaskHandle;
osThreadId My_EEPROM_TaskHandle;
osThreadId My_DumpToEEPROMHandle;
osThreadId My_DumpToSDCardHandle;
osThreadId My_Measure_TaskHandle;
osThreadId My_50ms_TaskHandle;
osThreadId My_DiagCheck_TaHandle;
osThreadId My_AlarmControlHandle;
osMessageQId Queue_EEPROM_readHandle;
osMessageQId Queue_EEPROM_writeHandle;
osMessageQId Queue_error_snapshot_dumpHandle;
osMessageQId Queue_diagnostic_snapshot_dumpHandle;
osTimerId My_Timer_ENC_Menu_ButtonHandle;
osTimerId My_Timer_ENC_Cruise_ButtonHandle;
osTimerId My_Timer_carWaterTempValueCheckHandle;
osTimerId My_Timer_carOilTempValueCheckHandle;
osTimerId My_Timer_carOilAnalogPressureValueCheckHandle;
osTimerId My_Timer_carMainBattVoltageValueCheckHandle;
osTimerId My_Timer_carFuelLevelValueCheckHandle;
osTimerId My_Timer_board3V3VoltageValueCheckHandle;
osTimerId My_Timer_board5VVoltageValueCheckHandle;
osTimerId My_Timer_boardVinVoltageValueCheckHandle;
osTimerId My_Timer_board3V3TempValueCheckHandle;
osTimerId My_Timer_board5VTempValueCheckHandle;
osTimerId My_Timer_boardHBridgeTempValueCheckHandle;
osTimerId My_Timer_carOilBinaryPressureValueCheckHandle;
osTimerId My_Timer_BuzzerHandle;
osTimerId My_Timer_carAuxBattVoltageValueCheckHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartStartUpTask(void const * argument);
extern void StartCruiseCntrlTask(void const * argument);
extern void Start500msTask(void const * argument);
extern void StartLCDTask(void const * argument);
extern void StartGPSTask(void const * argument);
extern void StartEEPROMTask(void const * argument);
extern void StartDumpToEEPROMTask(void const * argument);
extern void StartDumpToSDCardTask(void const * argument);
extern void StartMeasureTask(void const * argument);
extern void Start50msTask(void const * argument);
extern void StartDiagCheckTask(void const * argument);
extern void StartAlarmControlTask(void const * argument);
void ENC_Menu_Button_LongPress_Callback(void const * argument);
void ENC_Cruise_Button_LongPress_Callback(void const * argument);
extern void Timer_carWaterTempValueCheck(void const * argument);
extern void Timer_carOilTempValueCheck(void const * argument);
extern void Timer_carOilAnalogPressureValueCheck(void const * argument);
extern void Timer_carMainBattVoltageValueCheck(void const * argument);
extern void Timer_carFuelLevelValueCheck(void const * argument);
extern void Timer_board3V3VoltageValueCheck(void const * argument);
extern void Timer_board5VVoltageValueCheck(void const * argument);
extern void Timer_boardVinVoltageValueCheck(void const * argument);
extern void Timer_board3V3TempValueCheck(void const * argument);
extern void Timer_board5VTempValueCheck(void const * argument);
extern void Timer_boardHBridgeTempValueCheck(void const * argument);
extern void Timer_carOilBinaryPressureValueCheck(void const * argument);
extern void Timer_Buzzer(void const * argument);
extern void Timer_carAuxBattVoltageValueCheck(void const * argument);

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
{ /* FOR DEBUGGING */
#ifdef RUNTIME_STATS_TIMER_CONFIG
	HAL_TIM_Base_Start_IT(&htim7);
#endif
}

__weak unsigned long getRunTimeCounterValue(void)
{ /* FOR DEBUGGING */
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
}
/* USER CODE END 5 */

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
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
  /* definition and creation of My_Timer_ENC_Menu_Button */
  osTimerDef(My_Timer_ENC_Menu_Button, ENC_Menu_Button_LongPress_Callback);
  My_Timer_ENC_Menu_ButtonHandle = osTimerCreate(osTimer(My_Timer_ENC_Menu_Button), osTimerOnce, NULL);

  /* definition and creation of My_Timer_ENC_Cruise_Button */
  osTimerDef(My_Timer_ENC_Cruise_Button, ENC_Cruise_Button_LongPress_Callback);
  My_Timer_ENC_Cruise_ButtonHandle = osTimerCreate(osTimer(My_Timer_ENC_Cruise_Button), osTimerOnce, NULL);

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

  /* definition and creation of My_Timer_boardHBridgeTempValueCheck */
  osTimerDef(My_Timer_boardHBridgeTempValueCheck, Timer_boardHBridgeTempValueCheck);
  My_Timer_boardHBridgeTempValueCheckHandle = osTimerCreate(osTimer(My_Timer_boardHBridgeTempValueCheck), osTimerOnce, NULL);

  /* definition and creation of My_Timer_carOilBinaryPressureValueCheck */
  osTimerDef(My_Timer_carOilBinaryPressureValueCheck, Timer_carOilBinaryPressureValueCheck);
  My_Timer_carOilBinaryPressureValueCheckHandle = osTimerCreate(osTimer(My_Timer_carOilBinaryPressureValueCheck), osTimerOnce, NULL);

  /* definition and creation of My_Timer_Buzzer */
  osTimerDef(My_Timer_Buzzer, Timer_Buzzer);
  My_Timer_BuzzerHandle = osTimerCreate(osTimer(My_Timer_Buzzer), osTimerOnce, NULL);

  /* definition and creation of My_Timer_carAuxBattVoltageValueCheck */
  osTimerDef(My_Timer_carAuxBattVoltageValueCheck, Timer_carAuxBattVoltageValueCheck);
  My_Timer_carAuxBattVoltageValueCheckHandle = osTimerCreate(osTimer(My_Timer_carAuxBattVoltageValueCheck), osTimerOnce, NULL);

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
  /* definition and creation of My_StartUp_Task */
  osThreadDef(My_StartUp_Task, StartStartUpTask, osPriorityHigh, 0, 128);
  My_StartUp_TaskHandle = osThreadCreate(osThread(My_StartUp_Task), NULL);

  /* definition and creation of My_CruiseCntrl */
  osThreadDef(My_CruiseCntrl, StartCruiseCntrlTask, osPriorityAboveNormal, 0, 256);
  My_CruiseCntrlHandle = osThreadCreate(osThread(My_CruiseCntrl), NULL);

  /* definition and creation of My_500ms_Task */
  osThreadDef(My_500ms_Task, Start500msTask, osPriorityNormal, 0, 128);
  My_500ms_TaskHandle = osThreadCreate(osThread(My_500ms_Task), NULL);

  /* definition and creation of My_LCD_Task */
  osThreadDef(My_LCD_Task, StartLCDTask, osPriorityNormal, 0, 512);
  My_LCD_TaskHandle = osThreadCreate(osThread(My_LCD_Task), NULL);

  /* definition and creation of My_GPS_Task */
  osThreadDef(My_GPS_Task, StartGPSTask, osPriorityNormal, 0, 128);
  My_GPS_TaskHandle = osThreadCreate(osThread(My_GPS_Task), NULL);

  /* definition and creation of My_EEPROM_Task */
  osThreadDef(My_EEPROM_Task, StartEEPROMTask, osPriorityAboveNormal, 0, 128);
  My_EEPROM_TaskHandle = osThreadCreate(osThread(My_EEPROM_Task), NULL);

  /* definition and creation of My_DumpToEEPROM */
  osThreadDef(My_DumpToEEPROM, StartDumpToEEPROMTask, osPriorityBelowNormal, 0, 128);
  My_DumpToEEPROMHandle = osThreadCreate(osThread(My_DumpToEEPROM), NULL);

  /* definition and creation of My_DumpToSDCard */
  osThreadDef(My_DumpToSDCard, StartDumpToSDCardTask, osPriorityBelowNormal, 0, 1024);
  My_DumpToSDCardHandle = osThreadCreate(osThread(My_DumpToSDCard), NULL);

  /* definition and creation of My_Measure_Task */
  osThreadDef(My_Measure_Task, StartMeasureTask, osPriorityNormal, 0, 256);
  My_Measure_TaskHandle = osThreadCreate(osThread(My_Measure_Task), NULL);

  /* definition and creation of My_50ms_Task */
  osThreadDef(My_50ms_Task, Start50msTask, osPriorityNormal, 0, 128);
  My_50ms_TaskHandle = osThreadCreate(osThread(My_50ms_Task), NULL);

  /* definition and creation of My_DiagCheck_Ta */
  osThreadDef(My_DiagCheck_Ta, StartDiagCheckTask, osPriorityAboveNormal, 0, 256);
  My_DiagCheck_TaHandle = osThreadCreate(osThread(My_DiagCheck_Ta), NULL);

  /* definition and creation of My_AlarmControl */
  osThreadDef(My_AlarmControl, StartAlarmControlTask, osPriorityNormal, 0, 256);
  My_AlarmControlHandle = osThreadCreate(osThread(My_AlarmControl), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */

  /* Turning off all threads except the default one - there the initialization will take place */
  vTaskSuspend(My_CruiseCntrlHandle);
  vTaskSuspend(My_500ms_TaskHandle);
  vTaskSuspend(My_Measure_TaskHandle);
  vTaskSuspend(My_50ms_TaskHandle);
  vTaskSuspend(My_AlarmControlHandle);
  vTaskSuspend(My_DiagCheck_TaHandle);
  vTaskSuspend(My_DumpToEEPROMHandle);
  vTaskSuspend(My_DumpToSDCardHandle);
  vTaskSuspend(My_EEPROM_TaskHandle);
  vTaskSuspend(My_GPS_TaskHandle);
  vTaskSuspend(My_LCD_TaskHandle);

  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartStartUpTask */
/**
  * @brief  Function implementing the My_StartUp_Task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartStartUpTask */
void StartStartUpTask(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartStartUpTask */

	/* It is unfortunate that USB is the first thing to initialize
	 * however, this is how the CubeMX does it - no control over
	 * that I guess.
	 */

	/***** The following initialization steps are taken: *****\
	 *
	 * Step 0: USB initialization - automatically generated
	 * Step 1: Enable 5V DCDC converter
	 * Step 2: Start sensors and parameters measurements task
	 * Step 3: Start handling of EEPROM requests task
	 * Step 4: Start handling of EEPROM dumps task
	 * Step 5: Start handling of SDCard dumps (with SD initialization) task
	 * Step 6: Start the GPS task
	 * Step 7: Start 1000 milliseconds task
	 * Step 8: Start 500 milliseconds task
	 * Step 9: Start 50 milliseconds task
	 * Step 10: Start LCD task
	 * Step 11: Start Alarm Control task
	 *
	 ***** ***** ***** ***** ***** ***** ***** ***** ***** *****/

	/*** Step 1 ***/
 	(void)enable_5VDCDC(); /* Enable 5V DCDC converter */
	osDelay(50U); /* Wait 50ms for the voltage to set properly */

	/*** Step 2 ***/
	vTaskResume(My_EEPROM_TaskHandle); /* Start handling of EEPROM requests */
	osDelay(50U); /* Wait 50ms for the task to run a bit */

	/*** Step 3 ***/
	vTaskResume(My_Measure_TaskHandle); /* Start measurements of parameters and sensors */
	osDelay(50U); /* Wait 50ms for the task to run a bit */

	/*** Step 4 ***/
	vTaskResume(My_DumpToEEPROMHandle); /* Start handling EEPROM Dumps */
	osDelay(50U); /* Wait 50ms for the task to run a bit */

	/*** Step 5 ***/
	vTaskResume(My_DumpToSDCardHandle); /* Start handling SDCard Dumps */
	osDelay(50U); /* Wait 50ms for the task to run a bit */

	/*** Step 6 ***/
	(void)turnOnPower_GPS(); /* Turn on the power */
	vTaskResume(My_GPS_TaskHandle); /* Resume a task */
	osDelay(50U); /* Wait 50ms for the task to run a bit */

	/*** Step 7 ***/
	vTaskResume(My_CruiseCntrlHandle); /* Turn on cruise control task */
	osDelay(50U); /* Wait 50ms for the task to run a bit */

	/*** Step 8 ***/
	vTaskResume(My_500ms_TaskHandle); /* Turn on 500 milliseconds task */
	osDelay(50U); /* Wait 50ms for the task to run a bit */

	/*** Step 9 ***/
	vTaskResume(My_50ms_TaskHandle); /* Turn on 50 milliseconds task */
	osDelay(50U); /* Wait 50ms for the task to run a bit */

	/*** Step 10 ***/
	(void)turnOnPower_LCD(); /* Turn on the power */
	vTaskResume(My_LCD_TaskHandle); /* Turn on LCD task */
	osDelay(50U); /* Wait 50ms for the task to run a bit */

	osDelay(2000U); /* This is to wait for measurements to happen */

	/*** Step 11 ***/
	vTaskResume(My_DiagCheck_TaHandle); /* Turn on Diagnostic task */
	osDelay(1000U); /* Wait 50ms for the task to run a bit */

	/*** Step 12 ***/
	vTaskResume(My_AlarmControlHandle); /* Turn on Alarm Control task */
	osDelay(50U); /* Wait 50ms for the task to run a bit */

	/* After successful start up - suspend the task.
	 * The task could be destroyed but it may be useful later.
	 */
	vTaskSuspend(My_StartUp_TaskHandle);
	/* Infinite loop */
	for (;;)
	{
		/* We should never get here, as the task should be suspended
		 * before getting to the for loop. Thus error handler. */
		my_error_handler(OS__FOR_LOOP_REACHED);
		osDelay(1000); /* Dummy delay - should never be needed. */
	}
  /* USER CODE END StartStartUpTask */
}

/* ENC_Menu_Button_LongPress_Callback function */
void ENC_Menu_Button_LongPress_Callback(void const * argument)
{
  /* USER CODE BEGIN ENC_Menu_Button_LongPress_Callback */

	ENC_button_menu.allFlags |= 0b00010110;
	/* Set 1st, 2nd, 3rd bit to 1 to
	 indicate the long press on bits:
	 longPressDetected,
	 longPressDetectedForISR,
	 longPressDetectedBuzzer */

  /* USER CODE END ENC_Menu_Button_LongPress_Callback */
}

/* ENC_Cruise_Button_LongPress_Callback function */
void ENC_Cruise_Button_LongPress_Callback(void const * argument)
{
  /* USER CODE BEGIN ENC_Cruise_Button_LongPress_Callback */

	ENC_button_cruise.allFlags |= 0b00010110;
	/* Set 1st, 2nd, 3rd bit to 1 to
	 indicate the long press on bits:
	 longPressDetected,
	 longPressDetectedForISR,
	 longPressDetectedBuzzer */

  /* USER CODE END ENC_Cruise_Button_LongPress_Callback */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
