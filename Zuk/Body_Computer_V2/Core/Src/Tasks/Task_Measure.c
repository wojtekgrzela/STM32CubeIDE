/*
 * Task_Measure.c
 *
 *  Created on: Feb 28, 2021
 *      Author: Wojciech Grzelinski
 */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Includes */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "../../Middlewares/ST/STM32_USB_Device_Library/Core/Inc/usbd_def.h"
#include "defines.h"
#include "../../VariousFunctions/Functions.h"
#include "../../lcd_hd44780_i2c/lcd_hd44780_i2c.h"
#include "../../EEPROM/EEPROM.h"
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Defines And Typedefs */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#define ITERATION_1			(uint32_t)(1u)
#define ITERATION_2			(uint32_t)(2u)
#define ITERATION_10		(uint32_t)(10u)

#define MIN_LCD_BACKLIGHT_LEVEL		((uint16_t)(3u))

#define ON_BOARD_FAN_HBRIDGE_TEMP_ON_THRESHOLD		((float)(50.0))
#define RPM_DIVIDER_COEFFICIENT						((uint32_t)(3u))
#define SPEED_DIVIDER_COEFFICIENT					((float)(11.20392503/*Reaction for two edges (rising, falling)*/ / 2.0f /*/ 2.0f (2 because we calculate every 0.5s) */))

#define WIPERS_TIMING_DELTA_THRESHOLD				((uint32_t)(50u))
#define WIPERS_TIMING_MINIMUM_IDLE_OFFSET			((uint32_t)(3000u))	/* Timer for wipers will be set for at least 3 seconds */

typedef enum
{
	WIPERS_SLOW_INPUT_NO = 0,
	WIPERS_FAST_INPUT_NO = 1
} wipers_status_enum;
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Extern variables */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
extern osMessageQId Queue_EEPROM_writeHandle;
extern USBD_HandleTypeDef hUsbDeviceHS;
extern osTimerId My_Timer_WIPERSHandle;

extern EEPROM_parameters_struct FRAM_parameters;

extern waterTempSettings_struct CAR_waterTemp;
extern oilTempSettings_struct CAR_oilTemp;
extern oilPressureSettings_struct CAR_oilPressure;
extern fuelSettings_struct CAR_fuel;
extern batterySettings_struct CAR_mainBattery;
extern batterySettings_struct CAR_auxiliaryBattery;

extern carTemperature_type waterTemperatureValue;
extern carTemperature_type oilTemperatureValue;
extern carOilAnalogPressure_type oilPressureValue;
extern carOilBinaryPressure_type oilPressureValueBinary;
extern carVoltage_type mainBatteryVoltageValue;
extern carVoltage_type auxiliaryBatteryVoltageValue;
extern cafFuelLevel_type fuelLevelValue;

extern boardVoltage_type voltage3V3;
extern boardVoltage_type voltage5V;
extern boardVoltage_type voltageIn;
extern boardTemperature_type temperature3V3DCDC;
extern boardTemperature_type temperature5VDCDC;
extern boardTemperature_type temperatureHBridge;

extern boardVoltagesSettings_struct BOARD_voltage;
extern boardTemperaturesSettings_struct BOARD_temperature;

extern volatile uint32_t RPM_counter;
extern volatile uint32_t SPEED_counter;

extern CarStateinfo_type CarStateInfo;
extern NTC_parameters_struct NTC;


extern LCD_message mainBatteryVoltageValueForLCD;
extern LCD_message auxiliaryBatteryVoltageValueForLCD;
extern LCD_message waterTemperatureValueForLCD;
extern LCD_message totalMileageForLCD;
extern LCD_message tripMileageForLCD;
extern LCD_message fuelLevelValueForLCD;
extern LCD_message oilPressureValueBinaryForLCD;
extern LCD_message oilPressureValueForLCD;
extern LCD_message oilTemperatureValueForLCD;

extern LCD_message voltage3V3ForLCD;
extern LCD_message voltage5VForLCD;
extern LCD_message voltageInForLCD;
extern LCD_message temperature3V3DCDCForLCD;
extern LCD_message temperature5VDCDCForLCD;
extern LCD_message temperatureHBridgeForLCD;

extern LCD_message RPMForLCD;
extern LCD_message SPEEDForLCD;

extern volatile uint16_t ADC1Measures[NO_OF_ADC1_MEASURES];
extern volatile uint16_t ADC3Measures[NO_OF_ADC3_MEASURES];

extern CAR_mileage_struct CAR_mileage;

extern volatile ENCButton_struct ENC_button_menu;
extern volatile int8_t EncoderCounterMenuDiff;

extern SDCard_info_struct SDCard_info;

extern WDGCounter task_Measure_WDG;

extern LCDMainSettings_struct LCD_MainSettings;
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Local variables */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
static uint16_t inBinSignalsPinTable[8u] = {IN_BIN_SIG_1_Pin, IN_BIN_SIG_2_Pin, IN_BIN_SIG_3_Pin, IN_BIN_SIG_4_Pin, IN_BIN_SIG_5_Pin, IN_BIN_SIG_6_Pin, IN_BIN_SIG_7_Pin, IN_BIN_SIG_8_Pin};
static GPIO_TypeDef* inBinSignalsPortTable[8u] = {IN_BIN_SIG_1_GPIO_Port, IN_BIN_SIG_2_GPIO_Port, IN_BIN_SIG_3_GPIO_Port, IN_BIN_SIG_4_GPIO_Port, IN_BIN_SIG_5_GPIO_Port, IN_BIN_SIG_6_GPIO_Port, IN_BIN_SIG_7_GPIO_Port, IN_BIN_SIG_8_GPIO_Port};
GPIO_PinState inBinSignalsTable[8u] = {RESET};	/* not static on purpose */
GPIO_PinState inWipersOnSignal = {RESET};	/* not static on purpose */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Functions prototypes */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
static inline Error_Code calculate_waterTemperature(carTemperature_type *waterTemp);
static inline Error_Code calculate_oilTemperature(carTemperature_type *oilTemp);
static inline Error_Code calculate_oilPressure(carOilAnalogPressure_type *oilPressure);
static inline Error_Code check_oilBinaryPressure(carOilBinaryPressure_type *oilPressure);
static inline Error_Code calculate_mainBattVoltage(carVoltage_type *voltage);
static inline Error_Code calculate_auxBattVoltage(carVoltage_type *voltage);
static inline Error_Code calculate_CarfuelLevel(cafFuelLevel_type *fuelLevel);
static inline Error_Code calculate_3V3Voltage(boardVoltage_type *voltage);
static inline Error_Code calculate_5VVoltage(boardVoltage_type *voltage);
static inline Error_Code calculate_VinVoltage(boardVoltage_type *voltage);
static inline Error_Code calculate_3V3DCDCTemperatur(boardTemperature_type *temperature);
static inline Error_Code calculate_5VDCDCTemperature(boardTemperature_type *temperature);
static inline Error_Code calculate_HBridgeTemperature(boardTemperature_type *temperature);

static void Read_KeyState(void);
static void Read_CarState(void);
static void Read_AlternatorCharging(void);

static void checkMicroSDPresence(void);
static void checkUSBMSCState(void);

static void controlOnBoardFan(boolean state);
static void checkInBinSignals(void);
static void controlWipers(void);
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */



void StartMeasureTask(void const *argument)
{
	TickType_t xLastWakeTime;
	/* This period cannot be higher than 500 milliseconds - there are functions using half a second time domain */
	const TickType_t xFrequency = MY_TASK_MEASURE_TIME_PERIOD;
	Error_Code error = NO_ERROR;
	static const uint32_t iterationsInSecond = 1000u / MY_TASK_MEASURE_TIME_PERIOD; /* 1000u is Time base */

	static CREATE_EEPROM_data_struct(FRAMData);
	FRAMData.memAddressSize = FRAM_PAGES_ADDRESS_SIZE;
	FRAMData.data = CAR_mileage.data;
	/* We write 8 bytes at once and they are one after another so no need to write second time with another address */
	FRAMData.memAddress = TOTAL_MILEAGE_START_ADDRESS;
	FRAMData.size = 8u; /* 8 bytes is the size of both total and trip mileage */
	FRAMData.EEPROMParameters = &FRAM_parameters;

	// @formatter:off
	CarStateInfo.keyState 		= KeyState_Out;
	CarStateInfo.carState 		= CarState_Off;
	CarStateInfo.AlternatorCharging = FALSE;


	static uint32_t TenSeconds_iteration = ITERATION_1;
	static uint32_t OneSecond_iteration  = ITERATION_1;
	static uint32_t TwoSeconds_iteration = ITERATION_1;
	static uint32_t HalfSecond_iteration = ITERATION_1;
	boolean tenSeconds_FLAG = FALSE;
	boolean oneSecond_FLAG  = FALSE;
	boolean twoSecond_FLAG  = FALSE;
	boolean halfSecond_FLAG = FALSE;

	volatile float temp_mileage 			= 0;
	volatile uint32_t temp_RPM_counter 		= 0;
	volatile float temp_SPEED_counter 	= 0;

	static uint8_t totalMileageMessage[7] 	= "";
	static uint8_t tripMileageMessage[8]	= "";
	totalMileageForLCD.messageHandler		= totalMileageMessage;
	tripMileageForLCD.messageHandler		= tripMileageMessage;

	static uint8_t RPMMessage[5]	= "";
	static uint8_t SPEEDMessage[4]	= "";
	RPMForLCD.messageHandler	= RPMMessage;
	SPEEDForLCD.messageHandler	= SPEEDMessage;

	/* Tables for low pass digital filter */
	static carTemperature_type carWaterTempTable[NO_OF_ENGINE_TEMPERATURE_MEASUREMENTS_ADDED] 	= {0};
	static carTemperature_type carOilTempTable[NO_OF_ENGINE_TEMPERATURE_MEASUREMENTS_ADDED] 	= {0};
	static carVoltage_type carMainBatteryVoltageTable[NO_OF_CAR_VOLTAGES_MEASUREMENTS_ADDED]	= {0};
	static carVoltage_type carAuxBatteryVoltageTable[NO_OF_CAR_VOLTAGES_MEASUREMENTS_ADDED] 	= {0};
	static cafFuelLevel_type carFuelLevelTable[NO_OF_FUEL_LEVEL_MEASUREMENTS_ADDED] 			= {0};

	static boardTemperature_type temperature3V3DCDCTable[NO_OF_BOARD_TEMPERATURES_MEASUREMENTS_ADDED]	= {0};
	static boardTemperature_type temperature5VDCDCTable[NO_OF_BOARD_TEMPERATURES_MEASUREMENTS_ADDED] 	= {0};
	static boardTemperature_type temperatureHBridgeTable[NO_OF_BOARD_TEMPERATURES_MEASUREMENTS_ADDED] 	= {0};
	static boardVoltage_type voltage3V3Table[NO_OF_BOARD_VOLTAGES_MEASUREMENTS_ADDED] 	= {0};
	static boardVoltage_type voltage5VTable[NO_OF_BOARD_VOLTAGES_MEASUREMENTS_ADDED] 	= {0};
	static boardVoltage_type voltageInTable[NO_OF_BOARD_VOLTAGES_MEASUREMENTS_ADDED] 	= {0};

	/* Variables for summing values for low pass digital filter */
	carTemperature_type carWaterTemp_SUM 		= 0;
	carTemperature_type carOilTemp_SUM 			= 0;
	carVoltage_type carMainBatteryVoltage_SUM 	= 0;
	carVoltage_type carAuxBatteryVoltage_SUM 	= 0;
	cafFuelLevel_type carFuelLevel_SUM 			= 0;

	boardTemperature_type temperature3V3DCDC_SUM 	= 0;
	boardTemperature_type temperature5VDCDC_SUM 	= 0;
	boardTemperature_type temperatureHBridge_SUM 	= 0;
	boardVoltage_type voltage3V3_SUM 	= 0;
	boardVoltage_type voltage5V_SUM 	= 0;
	boardVoltage_type voltageIn_SUM		= 0;

	/* Variables for average value for low pass digital filter */
	carTemperature_type carWaterTemp_Average 		= 0;
	carTemperature_type carOilTemp_Average 			= 0;
	carVoltage_type carMainBatteryVoltage_Average	= 0;
	carVoltage_type carAuxBatteryVoltage_Average 	= 0;
	cafFuelLevel_type carFuelLevel_Average 			= 0;

	boardTemperature_type temperature3V3DCDC_Average	= 0;
	boardTemperature_type temperature5VDCDC_Average 	= 0;
	boardTemperature_type temperatureHBridge_Average 	= 0;
	boardVoltage_type voltage3V3_Average	= 0;
	boardVoltage_type voltage5V_Average 	= 0;
	boardVoltage_type voltageIn_Average 	= 0;

	/* Variables for new reading of a value for low pass digital filter */
	carTemperature_type carWaterTemp_NewValue 		= 0;
	carTemperature_type carOilTemp_NewValue 		= 0;
	carVoltage_type carMainBatteryVoltage_NewValue	= 0;
	carVoltage_type carAuxBatteryVoltage_NewValue 	= 0;
	cafFuelLevel_type carFuelLevel_NewValue 		= 0;

	carOilAnalogPressure_type carOilAnalogPressure_NewValue = 0;
	carOilBinaryPressure_type carOilBinaryPressure_NewValue = 0;

	boardTemperature_type temperature3V3DCDC_NewValue 	= 0;
	boardTemperature_type temperature5VDCDC_NewValue	= 0;
	boardTemperature_type temperatureHBridge_NewValue 	= 0;
	boardVoltage_type voltage3V3_NewValue	= 0;
	boardVoltage_type voltage5V_NewValue 	= 0;
	boardVoltage_type voltageIn_NewValue 	= 0;

	/* Buffers for value as a string */
	static uint8_t waterTemperatureValueMessage[4] 			= { SPACE_IN_ASCII };
	static uint8_t oilTemperatureValueMessage[4] 			= { SPACE_IN_ASCII };
	static uint8_t oilPressureValueMessage[4] 				= { SPACE_IN_ASCII };
	static uint8_t oilPressureValueBinaryMessage[4] 		= { SPACE_IN_ASCII };	/* OK, NOK */
	static uint8_t mainBatteryVoltageValueMessage[6] 		= { SPACE_IN_ASCII };
	static uint8_t auxiliaryBatteryVoltageValueMessage[6] 	= { SPACE_IN_ASCII };
	static uint8_t fuelLevelValueMessage[3] = "";

	static uint8_t temperature3V3DCDC_message[6] 	= { SPACE_IN_ASCII };
	static uint8_t temperature5VDCDC_message[6] 	= { SPACE_IN_ASCII };
	static uint8_t temperatureHBridge_message[6] 	= { SPACE_IN_ASCII };
	static uint8_t voltage3V3_message[5] 			= { SPACE_IN_ASCII };
	static uint8_t voltage5V_message[5] 			= { SPACE_IN_ASCII };
	static uint8_t voltageIn_message[6] 			= { SPACE_IN_ASCII };

	waterTemperatureValueForLCD.messageHandler	 		= waterTemperatureValueMessage;
	oilTemperatureValueForLCD.messageHandler 			= oilTemperatureValueMessage;
	oilPressureValueForLCD.messageHandler 				= oilPressureValueMessage;
	oilPressureValueBinaryForLCD.messageHandler 		= oilPressureValueBinaryMessage;
	mainBatteryVoltageValueForLCD.messageHandler 		= mainBatteryVoltageValueMessage;
	auxiliaryBatteryVoltageValueForLCD.messageHandler 	= auxiliaryBatteryVoltageValueMessage;
	fuelLevelValueForLCD.messageHandler 				= fuelLevelValueMessage;

	temperature3V3DCDCForLCD.messageHandler = temperature3V3DCDC_message;
	temperature5VDCDCForLCD.messageHandler 	= temperature5VDCDC_message;
	temperatureHBridgeForLCD.messageHandler	= temperatureHBridge_message;
	voltage3V3ForLCD.messageHandler	= voltage3V3_message;
	voltage5VForLCD.messageHandler 	= voltage5V_message;
	voltageInForLCD.messageHandler 	= voltageIn_message;
// @formatter:on

	/***** Initialization of buffers start *****/
	(void)calculate_CarfuelLevel(&carFuelLevel_NewValue);
	for (uint16_t i = 0; i < NO_OF_FUEL_LEVEL_MEASUREMENTS_ADDED; ++i)
	{
		carFuelLevelTable[i] = carFuelLevel_NewValue;
	}
	/***** Initialization of buffers end *****/

	xLastWakeTime = xTaskGetTickCount();
	/* Infinite loop */
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
	for (;;)
	{
		/* Check if it is an iteration once every 0.5 second (for SPEED calculations) */
		(((iterationsInSecond>>1) * ITERATION_1) == HalfSecond_iteration) ? (halfSecond_FLAG = TRUE) : (halfSecond_FLAG = FALSE);
		/* Check if it is an iteration once every 1 second (for RPM calculations) */
		((iterationsInSecond * ITERATION_1) == OneSecond_iteration) ? (oneSecond_FLAG = TRUE) : (oneSecond_FLAG = FALSE);
		/* Check if it is an iteration once every 2 seconds (for parameters saving and displaying) */
		((iterationsInSecond * ITERATION_2) == TwoSeconds_iteration) ? (twoSecond_FLAG = TRUE) : (twoSecond_FLAG = FALSE);
		/* Check if it is an iteration once every 10 seconds (for fuel level calculations) */
		((iterationsInSecond * ITERATION_10) == TenSeconds_iteration) ? (tenSeconds_FLAG = TRUE) : (tenSeconds_FLAG = FALSE);

		/* Get values of the counters and clear them */
//		taskENTER_CRITICAL();
		if (halfSecond_FLAG)
		{
			temp_SPEED_counter = (float)SPEED_counter / (SPEED_DIVIDER_COEFFICIENT); /* temp_SPEED_counter is in meters/s */
			SPEED_counter = 0;
		}
		if (oneSecond_FLAG)
		{
			temp_RPM_counter = RPM_counter * RPM_DIVIDER_COEFFICIENT;
			RPM_counter = 0;
		}
//		taskEXIT_CRITICAL();

		/* Backlight calculations */
		if(POTENTIOMETER_2_ADC_VALUE > 4050)
		{
			LCD_MainSettings.backlightLevel = MIN_LCD_BACKLIGHT_LEVEL;
		}
		else
		{
			LCD_MainSettings.backlightLevel = 10000 - (POTENTIOMETER_2_ADC_VALUE*2.442f);
		}

		/* In this for loop we make the sum of our measurements */
		for (uint16_t i = 0; i < NO_OF_ENGINE_TEMPERATURE_MEASUREMENTS_ADDED; ++i)
		{
			carWaterTemp_SUM += carWaterTempTable[i];
			carOilTemp_SUM += carOilTempTable[i];
		}
		for (uint16_t i = 0; i < NO_OF_CAR_VOLTAGES_MEASUREMENTS_ADDED; ++i)
		{
			carMainBatteryVoltage_SUM += carMainBatteryVoltageTable[i];
			carAuxBatteryVoltage_SUM += carAuxBatteryVoltageTable[i];
		}
		if (tenSeconds_FLAG) /* This will execute every 10 seconds */
		{
			for (uint16_t i = 0; i < NO_OF_FUEL_LEVEL_MEASUREMENTS_ADDED; ++i)
			{
				carFuelLevel_SUM += carFuelLevelTable[i];
			}
		}
		for (uint16_t i = 0; i < NO_OF_BOARD_TEMPERATURES_MEASUREMENTS_ADDED; ++i)
		{
			temperature3V3DCDC_SUM += temperature3V3DCDCTable[i];
			temperature5VDCDC_SUM += temperature5VDCDCTable[i];
			temperatureHBridge_SUM += temperatureHBridgeTable[i];
		}
		for (uint16_t i = 0; i < NO_OF_BOARD_VOLTAGES_MEASUREMENTS_ADDED; ++i)
		{
			voltage3V3_SUM += voltage3V3Table[i];
			voltage5V_SUM += voltage5VTable[i];
			voltageIn_SUM += voltageInTable[i];
		}

		/* Here we create our average value */
		carWaterTemp_Average = carWaterTemp_SUM / NO_OF_ENGINE_TEMPERATURE_MEASUREMENTS_ADDED;
		carOilTemp_Average = carOilTemp_SUM / NO_OF_ENGINE_TEMPERATURE_MEASUREMENTS_ADDED;
		carMainBatteryVoltage_Average = carMainBatteryVoltage_SUM / NO_OF_CAR_VOLTAGES_MEASUREMENTS_ADDED;
		carAuxBatteryVoltage_Average = carAuxBatteryVoltage_SUM / NO_OF_CAR_VOLTAGES_MEASUREMENTS_ADDED;
		if (tenSeconds_FLAG)
		{
			carFuelLevel_Average = carFuelLevel_SUM / NO_OF_FUEL_LEVEL_MEASUREMENTS_ADDED; /* This will execute every 10 seconds */
		}

		temperature3V3DCDC_Average = temperature3V3DCDC_SUM / NO_OF_BOARD_TEMPERATURES_MEASUREMENTS_ADDED;
		temperature5VDCDC_Average = temperature5VDCDC_SUM / NO_OF_BOARD_TEMPERATURES_MEASUREMENTS_ADDED;
		temperatureHBridge_Average = temperatureHBridge_SUM / NO_OF_BOARD_TEMPERATURES_MEASUREMENTS_ADDED;
		voltage3V3_Average = voltage3V3_SUM / NO_OF_BOARD_VOLTAGES_MEASUREMENTS_ADDED;
		voltage5V_Average = voltage5V_SUM / NO_OF_BOARD_VOLTAGES_MEASUREMENTS_ADDED;
		voltageIn_Average = voltageIn_SUM / NO_OF_BOARD_VOLTAGES_MEASUREMENTS_ADDED;

		/* Here we calculate the new measurement */
		if (NO_ERROR == error)
			error = calculate_waterTemperature(&carWaterTemp_NewValue);
		if (NO_ERROR == error)
			error = calculate_oilTemperature(&carOilTemp_NewValue);
		if (NO_ERROR == error)
			error = calculate_mainBattVoltage(&carMainBatteryVoltage_NewValue);
		if (NO_ERROR == error)
			error = calculate_auxBattVoltage(&carAuxBatteryVoltage_NewValue);
		if (NO_ERROR == error)
			error = calculate_CarfuelLevel(&carFuelLevel_NewValue);

		if (NO_ERROR == error)
			error = calculate_oilPressure(&carOilAnalogPressure_NewValue); /* This value is not filtered */
		if (NO_ERROR == error)
			error = check_oilBinaryPressure(&carOilBinaryPressure_NewValue); /* This value is not filtered */

		if (NO_ERROR == error)
			error = calculate_3V3Voltage(&voltage3V3_NewValue);
		if (NO_ERROR == error)
			error = calculate_5VVoltage(&voltage5V_NewValue);
		if (NO_ERROR == error)
			error = calculate_VinVoltage(&voltageIn_NewValue);
		if (NO_ERROR == error)
			error = calculate_3V3DCDCTemperatur(&temperature3V3DCDC_NewValue);
		if (NO_ERROR == error)
			error = calculate_5VDCDCTemperature(&temperature5VDCDC_NewValue);
		if (NO_ERROR == error)
			error = calculate_HBridgeTemperature(&temperatureHBridge_NewValue);

		if (NO_ERROR != error)
			my_error_handler(error); /* In case of an error go to the error handler function */
#if 0
		/* Here we add the new measurement */
		carWaterTemp_Average += carWaterTemp_NewValue;
		carOilTemp_Average += carOilTemp_NewValue;
		carMainBatteryVoltage_Average += carMainBatteryVoltage_NewValue;
		carAuxBatteryVoltage_Average += carAuxBatteryVoltage_NewValue;
		if (tenSeconds_FLAG)
		{
			carFuelLevel_Average += carFuelLevel_NewValue; /* This will execute every 10 seconds */
		}

		temperature3V3DCDC_Average += temperature3V3DCDC_NewValue;
		temperature5VDCDC_Average += temperature5VDCDC_NewValue;
		temperatureHBridge_Average += temperatureHBridge_NewValue;
		voltage3V3_Average += voltage3V3_NewValue;
		voltage5V_Average += voltage5V_NewValue;
		voltageIn_Average += voltageIn_NewValue;

		/* Here we subtract the oldest measurement */
		carWaterTemp_Average -= carWaterTempTable[0];
		carOilTemp_Average -= carOilTempTable[0];
		carMainBatteryVoltage_Average -= carMainBatteryVoltageTable[0];
		carAuxBatteryVoltage_Average -= carAuxBatteryVoltageTable[0];
		if (tenSeconds_FLAG)
		{
			carFuelLevel_Average -= carFuelLevelTable[0]; /* This will execute every 10 seconds */
		}

		temperature3V3DCDC_Average -= temperature3V3DCDCTable[0];
		temperature5VDCDC_Average -= temperature5VDCDCTable[0];
		temperatureHBridge_Average -= temperatureHBridgeTable[0];
		voltage3V3_Average -= voltage3V3Table[0];
		voltage5V_Average -= voltage5VTable[0];
		voltageIn_Average -= voltageInTable[0];
#endif
		/* In this for loops we shift the values in the table */
		for (uint16_t i = 1u; i <= (NO_OF_ENGINE_TEMPERATURE_MEASUREMENTS_ADDED - 1u) /*1u - because we subtract 1 below from i*/; ++i)
		{
			carWaterTempTable[i - 1u] = carWaterTempTable[i];
			carOilTempTable[i - 1u] = carOilTempTable[i];
		}
		for (uint16_t i = 1u; i <= (NO_OF_CAR_VOLTAGES_MEASUREMENTS_ADDED - 1u) /*1u - because we subtract 1 below from i*/; ++i)
		{
			carMainBatteryVoltageTable[i - 1u] = carMainBatteryVoltageTable[i];
			carAuxBatteryVoltageTable[i - 1u] = carAuxBatteryVoltageTable[i];
		}
		if (tenSeconds_FLAG) /* This will execute every 10 seconds */
		{
			for (uint16_t i = 1u; i <= (NO_OF_FUEL_LEVEL_MEASUREMENTS_ADDED - 1u) /*1u - because we subtract 1 below from i*/; ++i)
			{
				carFuelLevelTable[i - 1u] = carFuelLevelTable[i];
			}
		}
		for (uint16_t i = 1u; i <= (NO_OF_BOARD_TEMPERATURES_MEASUREMENTS_ADDED - 1u) /*1u - because we subtract 1 below from i*/; ++i)
		{
			temperature3V3DCDCTable[i - 1u] = temperature3V3DCDCTable[i];
			temperature5VDCDCTable[i - 1u] = temperature5VDCDCTable[i];
			temperatureHBridgeTable[i - 1u] = temperatureHBridgeTable[i];
		}
		for (uint16_t i = 1u; i <= (NO_OF_BOARD_VOLTAGES_MEASUREMENTS_ADDED - 1u) /*1u - because we subtract 1 below from i*/; ++i)
		{
			voltage3V3Table[i - 1u] = voltage3V3Table[i];
			voltage5VTable[i - 1u] = voltage5VTable[i];
			voltageInTable[i - 1u] = voltageInTable[i];
		}

		/* Here we add the new measurement to the table */
		carWaterTempTable[NO_OF_ENGINE_TEMPERATURE_MEASUREMENTS_ADDED - 1u] = carWaterTemp_NewValue;
		carOilTempTable[NO_OF_ENGINE_TEMPERATURE_MEASUREMENTS_ADDED - 1u] = carOilTemp_NewValue;
		carMainBatteryVoltageTable[NO_OF_CAR_VOLTAGES_MEASUREMENTS_ADDED - 1u] = carMainBatteryVoltage_NewValue;
		carAuxBatteryVoltageTable[NO_OF_CAR_VOLTAGES_MEASUREMENTS_ADDED - 1u] = carAuxBatteryVoltage_NewValue;
		if (tenSeconds_FLAG)
		{
			carFuelLevelTable[NO_OF_FUEL_LEVEL_MEASUREMENTS_ADDED - 1u] = carFuelLevel_NewValue;
		}

		temperature3V3DCDCTable[NO_OF_BOARD_TEMPERATURES_MEASUREMENTS_ADDED - 1u] = temperature3V3DCDC_NewValue;
		temperature5VDCDCTable[NO_OF_BOARD_TEMPERATURES_MEASUREMENTS_ADDED - 1u] = temperature5VDCDC_NewValue;
		temperatureHBridgeTable[NO_OF_BOARD_TEMPERATURES_MEASUREMENTS_ADDED - 1u] = temperatureHBridge_NewValue;
		voltage3V3Table[NO_OF_BOARD_VOLTAGES_MEASUREMENTS_ADDED - 1u] = voltage3V3_NewValue;
		voltage5VTable[NO_OF_BOARD_VOLTAGES_MEASUREMENTS_ADDED - 1u] = voltage5V_NewValue;
		voltageInTable[NO_OF_BOARD_VOLTAGES_MEASUREMENTS_ADDED - 1u] = voltageIn_NewValue;

		/* Here we write our calculated value to the official variable */
		waterTemperatureValue = carWaterTemp_Average;
		oilTemperatureValue = carOilTemp_Average;
		mainBatteryVoltageValue = carMainBatteryVoltage_Average;
		auxiliaryBatteryVoltageValue = carAuxBatteryVoltage_Average;
		if (tenSeconds_FLAG)
		{
			fuelLevelValue = carFuelLevel_Average;
		}

		oilPressureValue = carOilAnalogPressure_NewValue; /* This value is not filtered */
		oilPressureValueBinary = carOilBinaryPressure_NewValue; /* This value is not filtered */

		temperature3V3DCDC = temperature3V3DCDC_Average;
		temperature5VDCDC = temperature5VDCDC_Average;
		temperatureHBridge = temperatureHBridge_Average;
		voltage3V3 = voltage3V3_Average;
		voltage5V = voltage5V_Average;
		voltageIn = voltageIn_Average;

		/* Here we clean our temporary variables */
		carWaterTemp_SUM = 0;
		carOilTemp_SUM = 0;
		carMainBatteryVoltage_SUM = 0;
		carAuxBatteryVoltage_SUM = 0;
		carFuelLevel_SUM = 0;

		temperature3V3DCDC_SUM = 0;
		temperature5VDCDC_SUM = 0;
		temperatureHBridge_SUM = 0;
		voltage3V3_SUM = 0;
		voltage5V_SUM = 0;
		voltageIn_SUM = 0;

		/* Here we make the string from our official variables */
		/*** Car water temperature ***/
		waterTemperatureValueForLCD.messageReadyFLAG = FALSE;
		snprintf((char*)waterTemperatureValueForLCD.messageHandler, 4, "%3" PRIi16, (int16_t)waterTemperatureValue);
		waterTemperatureValueForLCD.size = strlen((char*)waterTemperatureValueForLCD.messageHandler);
		waterTemperatureValueForLCD.messageReadyFLAG = TRUE;
		/*** Car oil temperature ***/
		oilTemperatureValueForLCD.messageReadyFLAG = FALSE;
		snprintf((char*)oilTemperatureValueForLCD.messageHandler, 4, "%3" PRIi16, (int16_t)oilTemperatureValue);
		oilTemperatureValueForLCD.size = strlen((char*)oilTemperatureValueForLCD.messageHandler);
		oilTemperatureValueForLCD.messageReadyFLAG = TRUE;
		/*** Car oil analog pressure in psi ***/
		oilPressureValueForLCD.messageReadyFLAG = FALSE;
		snprintf((char*)oilPressureValueForLCD.messageHandler, 3, "%01" PRIu16, (uint16_t)oilPressureValue);
		oilPressureValueForLCD.size = strlen((char*)oilPressureValueForLCD.messageHandler);
		oilPressureValueForLCD.messageReadyFLAG = TRUE;
		/*** Car oil binary pressure ***/
		oilPressureValueBinaryForLCD.messageReadyFLAG = FALSE;
		snprintf((char*)oilPressureValueBinaryForLCD.messageHandler, 4, (oilPressureValueBinary) ? "NOK" : "OK");
		oilPressureValueBinaryForLCD.size = strlen((char*)oilPressureValueForLCD.messageHandler);
		oilPressureValueBinaryForLCD.messageReadyFLAG = TRUE;
		/*** Car main battery voltage ***/
		mainBatteryVoltageValueForLCD.messageReadyFLAG = FALSE;
		snprintf((char*)mainBatteryVoltageValueForLCD.messageHandler, 6, "%02" PRIu16 ".%02" PRIu16, (uint16_t)mainBatteryVoltageValue,
				(uint16_t)(mainBatteryVoltageValue * 100) % 100);
		mainBatteryVoltageValueForLCD.size = strlen((char*)mainBatteryVoltageValueForLCD.messageHandler);
		mainBatteryVoltageValueForLCD.messageReadyFLAG = TRUE;
		/*** Car auxiliary battery voltage ***/
		auxiliaryBatteryVoltageValueForLCD.messageReadyFLAG = FALSE;
		snprintf((char*)auxiliaryBatteryVoltageValueForLCD.messageHandler, 6, "%02" PRIu16 ".%02" PRIu16, (uint16_t)auxiliaryBatteryVoltageValue,
				(uint16_t)(auxiliaryBatteryVoltageValue * 100) % 100);
		auxiliaryBatteryVoltageValueForLCD.size = strlen((char*)auxiliaryBatteryVoltageValueForLCD.messageHandler);
		auxiliaryBatteryVoltageValueForLCD.messageReadyFLAG = TRUE;
		/*** Car fuel level ***/
		if (tenSeconds_FLAG) /* This will execute every 10 seconds */
		{
			fuelLevelValueForLCD.messageReadyFLAG = FALSE;
			snprintf((char*)fuelLevelValueForLCD.messageHandler, 3, "%2" PRIu16, (uint16_t)fuelLevelValue);
			fuelLevelValueForLCD.size = strlen((char*)fuelLevelValueForLCD.messageHandler);
			fuelLevelValueForLCD.messageReadyFLAG = TRUE;
		}
		/*** Board 3V3 DCDC temperature ***/
		temperature3V3DCDCForLCD.messageReadyFLAG = FALSE;
		snprintf((char*)temperature3V3DCDCForLCD.messageHandler, 6, "%" PRIi16 "%cC", (int16_t)temperature3V3DCDC, DEGREE_SYMBOL_LCD);
		temperature3V3DCDCForLCD.size = strlen((char*)temperature3V3DCDCForLCD.messageHandler);
		temperature3V3DCDCForLCD.messageReadyFLAG = TRUE;
		/*** Board 5V DCDC temperature ***/
		temperature5VDCDCForLCD.messageReadyFLAG = FALSE;
		snprintf((char*)temperature5VDCDCForLCD.messageHandler, 6, "%" PRIi16 "%cC", (int16_t)temperature5VDCDC, DEGREE_SYMBOL_LCD);
		temperature5VDCDCForLCD.size = strlen((char*)temperature5VDCDCForLCD.messageHandler);
		temperature5VDCDCForLCD.messageReadyFLAG = TRUE;
		/*** Board H Bridge temperature ***/
		temperatureHBridgeForLCD.messageReadyFLAG = FALSE;
		snprintf((char*)temperatureHBridgeForLCD.messageHandler, 6, "%" PRIi16 "%cC", (int16_t)temperatureHBridge, DEGREE_SYMBOL_LCD);
		temperatureHBridgeForLCD.size = strlen((char*)temperatureHBridgeForLCD.messageHandler);
		temperatureHBridgeForLCD.messageReadyFLAG = TRUE;
		/*** Board 3V3 DCDC voltage ***/
		voltage3V3ForLCD.messageReadyFLAG = FALSE;
		snprintf((char*)voltage3V3ForLCD.messageHandler, 6, "%01" PRIu16 ".%02" PRIi16 "V", (uint16_t)voltage3V3, (uint16_t)(voltage3V3 * 100) % 100);
		voltage3V3ForLCD.size = strlen((char*)voltage3V3ForLCD.messageHandler);
		voltage3V3ForLCD.messageReadyFLAG = TRUE;
		/*** Board 5V DCDC voltage ***/
		voltage5VForLCD.messageReadyFLAG = FALSE;
		snprintf((char*)voltage5VForLCD.messageHandler, 6, "%01" PRIu16 ".%02" PRIi16 "V", (uint16_t)voltage5V, (uint16_t)(voltage5V * 100) % 100);
		voltage5VForLCD.size = strlen((char*)voltage5VForLCD.messageHandler);
		voltage5VForLCD.messageReadyFLAG = TRUE;
		/*** Board In voltage ***/
		voltageInForLCD.messageReadyFLAG = FALSE;
		snprintf((char*)voltageInForLCD.messageHandler, 7, "%01" PRIu16 ".%02" PRIu16 "V", (uint16_t)voltageIn, (uint16_t)(voltageIn * 100) % 100);
		voltageInForLCD.size = strlen((char*)voltageInForLCD.messageHandler);
		voltageInForLCD.messageReadyFLAG = TRUE;


		/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
		/* Measurements not related directly to sensors, not needing filtering and averaging */

		/* Check AlternatorCharging (TRUE, FALSE) */
		Read_AlternatorCharging();
		/* Check keyState (Out, IgnitionOn, Crank) */
		Read_KeyState();
		/* Check carState (Off, Crank, Idle, Drive) */
		Read_CarState();

		/* Check checkMicroSDPresence (YES, NO) */
		checkMicroSDPresence();
		/* Check USB state (is it present or not) and set SDCard_info */
		checkUSBMSCState();
		/* Check state of input GPIOs */
		checkInBinSignals();

		/* Control the wipers */
		controlWipers();


		if (halfSecond_FLAG)
		{
			temp_mileage = temp_SPEED_counter; /* Because temp_SPEED_counter is in meters/s */
			CarStateInfo.SPEED = temp_SPEED_counter * (SECONDS_IN_ONE_HOUR / METERS_IN_KILOMETER); /* (mileage in meters * 3600 / 1000) */

			temp_mileage = INTEGER_DIVISION_ROUNDING((temp_mileage * 2), 2); /* Thanks to that operation we will obtain a rounded integer after truncating */
			CAR_mileage.totalMileage += temp_mileage; /* Adding the mileage to the global counter */
			CAR_mileage.tripMileage += temp_mileage; /* Adding the mileage to the global counter */
		}

		if (oneSecond_FLAG)
		{
			CarStateInfo.RPM = temp_RPM_counter; /* pulse counter * RPMS in one pulse */
		}

		if (oneSecond_FLAG)
		{
			/* Engine RPM */
			RPMForLCD.messageReadyFLAG = FALSE;
			snprintf((char*)RPMForLCD.messageHandler, 5, "%01" PRIu32, CarStateInfo.RPM);
			RPMForLCD.size = strlen((char*)RPMForLCD.messageHandler);
			RPMForLCD.messageReadyFLAG = TRUE;
			/* Vehicle Speed */
			SPEEDForLCD.messageReadyFLAG = FALSE;
			snprintf((char*)SPEEDForLCD.messageHandler, 4, "%01" PRIu32, (uint32_t)CarStateInfo.SPEED);
			SPEEDForLCD.size = strlen((char*)SPEEDForLCD.messageHandler);
			SPEEDForLCD.messageReadyFLAG = TRUE;
		}

		if (twoSecond_FLAG)
		{
			/* Vehicle total mileage */
			totalMileageForLCD.messageReadyFLAG = FALSE;
			snprintf((char*)totalMileageForLCD.messageHandler, 7, "%01" PRIu32, (CAR_mileage.totalMileage / (uint32_t)METERS_IN_KILOMETER));
			totalMileageForLCD.size = strlen((char*)totalMileageForLCD.messageHandler); /* It will look like that: "123456" */
			totalMileageForLCD.messageReadyFLAG = TRUE;
			/* Vehicle trip mileage */
			tripMileageForLCD.messageReadyFLAG = FALSE;
			snprintf((char*)tripMileageForLCD.messageHandler, 8, "%01" PRIu32 ".%01" PRIu32, (CAR_mileage.tripMileage / (uint32_t)METERS_IN_KILOMETER),
					((CAR_mileage.tripMileage / 100u) % 10u));
			tripMileageForLCD.size = strlen((char*)tripMileageForLCD.messageHandler); /* It will look like that: "12345.x" */
			tripMileageForLCD.messageReadyFLAG = TRUE;

			xQueueSend(Queue_EEPROM_writeHandle, &FRAMData, (TickType_t)0u/*NONE wait time if the queue is full*/);
		}

		if (twoSecond_FLAG)
		{
			if(temperatureHBridge >= ON_BOARD_FAN_HBRIDGE_TEMP_ON_THRESHOLD)
			{
				controlOnBoardFan(ON);
			}
			else if(temperatureHBridge <= ON_BOARD_FAN_HBRIDGE_TEMP_ON_THRESHOLD-3)
			{
				controlOnBoardFan(OFF);
			}
		}

		/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */


		/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
		/***** Iteration counters managing start *****/
		if (tenSeconds_FLAG) /* Managing 10 seconds interval */
		{
			TenSeconds_iteration = ITERATION_1;
		}
		else
		{
			++TenSeconds_iteration;
		}

		if (twoSecond_FLAG) /* Managing 2 seconds interval */
		{
			TwoSeconds_iteration = ITERATION_1;
		}
		else
		{
			++TwoSeconds_iteration;
		}

		if (oneSecond_FLAG) /* Managing 1 second interval */
		{
			OneSecond_iteration = ITERATION_1;
		}
		else
		{
			++OneSecond_iteration;
		}

		if (halfSecond_FLAG) /* Managing 0.5 second interval */
		{
			HalfSecond_iteration = ITERATION_1;
		}
		else
		{
			++HalfSecond_iteration;
		}
		/***** Iteration counters managing end *****/
		/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

		++task_Measure_WDG;
		vTaskDelayUntil(&xLastWakeTime, xFrequency);
	}
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
}




static inline Error_Code calculate_waterTemperature(carTemperature_type *waterTemp)
{
	Error_Code error = NO_ERROR;
	error = calculate_LM35_temperature(waterTemp, CAR_WATER_TEMPERATURE_LM35_ADC_VALUE);
	return error;
}



static inline Error_Code calculate_oilTemperature(carTemperature_type *oilTemp)
{
	Error_Code error = NO_ERROR;
	//TODO
	return error;
}



static inline Error_Code calculate_oilPressure(carOilAnalogPressure_type *oilPressure)
{
	Error_Code error = NO_ERROR;
	// TODO
	return error;
}



static inline Error_Code check_oilBinaryPressure(carOilBinaryPressure_type *oilPressure)
{
	Error_Code error = NO_ERROR;
	/* The Sensor works this way:
	 * Oil pressure OK: sensor is open (no connection to GND) - thus there is +12V on the sensor.
	 * Oil pressure NOK: sensor is tied to the GND - thus there is 0V on the sensor.
	 *
	 * That is why there is no need to over-complicate: HIGH = OK, LOW = NOK
	 */
	*oilPressure = (boolean)HAL_GPIO_ReadPin(GPIO_PORT_OIL_PRESSURE_SENSOR, GPIO_PIN_OIL_PRESSURE_SENSOR);
	return error;
}



static inline Error_Code calculate_mainBattVoltage(carVoltage_type *voltage)
{
	Error_Code error = NO_ERROR;
	error = calculate_voltage(voltage, MAIN_BATTERY_VOLTAGE_ADC_VALUE, MAIN_BATTERY_VOLTAGE_DIVIDER);
	return error;
}



static inline Error_Code calculate_auxBattVoltage(carVoltage_type *voltage)
{
	Error_Code error = NO_ERROR;
	error = calculate_voltage(voltage, AUX_BATTERY_VOLTAGE_ADC_VALUE, AUXILIARY_BATTERY_VOLTAGE_DIVIDER);
	return error;
}



static inline Error_Code calculate_CarfuelLevel(cafFuelLevel_type *fuelLevel)
{
	Error_Code error = NO_ERROR;
	error = calculate_fuelLevel(fuelLevel, FUEL_LEVEL_ADC_VALUE);
	return error;
}



static inline Error_Code calculate_3V3Voltage(boardVoltage_type *voltage)
{
	Error_Code error = NO_ERROR;
	error = calculate_voltage(voltage, BOARD_VOLTAGE_3V3_ADC_VALUE, MEASURE_3V3_VOLTAGE_DIVIDER);
	return error;
}



static inline Error_Code calculate_5VVoltage(boardVoltage_type *voltage)
{
	Error_Code error = NO_ERROR;
	error = calculate_voltage(voltage, BOARD_VOLTAGE_5V_ADC_VALUE, MEASURE_5V_VOLTAGE_DIVIDER);
	return error;
}



static inline Error_Code calculate_VinVoltage(boardVoltage_type *voltage)
{
	Error_Code error = NO_ERROR;
	error = calculate_voltage(voltage, BOARD_VOLTAGE_VIN_ADC_VALUE, MEASURE_VIN_VOLTAGE_DIVIDER);
	return error;
}



static inline Error_Code calculate_3V3DCDCTemperatur(boardTemperature_type *temperature)
{
	Error_Code error = NO_ERROR;
	error = calculate_NTC_temperature(temperature, BOARD_TEMPERATURE_3V3DCDC_ADC_VALUE, &NTC);
	return error;
}



static inline Error_Code calculate_5VDCDCTemperature(boardTemperature_type *temperature)
{
	Error_Code error = NO_ERROR;
	error = calculate_NTC_temperature(temperature, BOARD_TEMPERATURE_5VDCDC_ADC_VALUE, &NTC);
	return error;
}



static inline Error_Code calculate_HBridgeTemperature(boardTemperature_type *temperature)
{
	Error_Code error = NO_ERROR;
	error = calculate_NTC_temperature(temperature, BOARD_TEMPERATURE_HBRIDGE_ADC_VALUE, &NTC);
	return error;
}



/* Check in what state is the key in the car (out, on ignition, cranking) */
static void Read_KeyState(void)
{
	if (GPIO_PIN_SET == HAL_GPIO_ReadPin(KEY_CRANKING_GPIO_Port, KEY_CRANKING_Pin))
	{
		CarStateInfo.keyState = KeyState_Crank;
	}
	else if (GPIO_PIN_SET == HAL_GPIO_ReadPin(KEY_IGNITION_GPIO_Port, KEY_IGNITION_Pin))
	{
		CarStateInfo.keyState = KeyState_IgnitionOn;
	}
	else
	{
		CarStateInfo.keyState = KeyState_Out;
	}
}



/* Check the car state basing on the key and engine status (off, cranking, idling, driving). */
static void Read_CarState(void)
{
	if (KeyState_Out == CarStateInfo.keyState)
	{
		CarStateInfo.carState = CarState_Off;
	}
	/* If key is in crank position. */
	else if (KeyState_Crank == CarStateInfo.keyState)
	{
		CarStateInfo.carState = CarState_Crank;
	}
	/* If key is in ignition state AND engine is in idle/ */
	else if ((KeyState_IgnitionOn == CarStateInfo.keyState) && ((TRUE == CarStateInfo.AlternatorCharging) || (OK == oilPressureValueBinary)))
	{
		CarStateInfo.carState = CarState_Running;
	}
	else
	{
		CarStateInfo.carState = CarState_Off;
	}
}


/* This function reads the pin state which is connected to the 12V line signal from alternator
 * (the same signal as for the icon of lack of charging) */
static void Read_AlternatorCharging(void)
{
	/* Possible states:
	 * LOW - alternator charging NOK
	 * HIGH - alternator charging OK
	 */
	CarStateInfo.AlternatorCharging = (boolean)HAL_GPIO_ReadPin(ALTERNATOR_CHARGING_GPIO_Port, ALTERNATOR_CHARGING_Pin);
}



/* This function checks if the MicroSD card is inserted into the slot. */
static void checkMicroSDPresence(void)
{
	static boolean oldState = FALSE;
	/* "!" because the logic is inverted - when the MicroSD is inserted it shorts the pin to GND. */
	oldState = SDCard_info.isPresent;
	SDCard_info.isPresent = !HAL_GPIO_ReadPin(MICROSD_DETECT_GPIO_Port, MICROSD_DETECT_Pin);

	if((TRUE == oldState) && (FALSE == SDCard_info.isPresent))
	{
		SDCard_info.gotPulledOut = TRUE;
	}


}



/* This function checks the state of the USB Mass Storage Device to set request for SD Card for Dump task */
static void checkUSBMSCState(void)
{
	/* Reads the state of the USB */
	uint8_t USBMSC_state = hUsbDeviceHS.dev_state;
	/* 0x03 - USBD_STATE_CONFIGURED */
	/* 0x04 - USBD_STATE_SUSPENDED */

	if(0x03 == USBMSC_state)
	{
		SDCard_info.cardRequestedByUSB = TRUE;
	}
	else
	{
		SDCard_info.cardRequestedByUSB = FALSE;
	}
}



/* This function controls the state of the FAN - turns it on or off */
static void controlOnBoardFan(boolean state)
{
	/* Possible states:
	 * ON - turn the FAN ON
	 * OFF - turn the FAN OFF
	 */
	HAL_GPIO_WritePin(FAN_BOARD_GPIO_Port, FAN_BOARD_Pin, (GPIO_PinState)state);
}



static void checkInBinSignals(void)
{
	for(uint8_t i=0; i < 8u; ++i)
	{
		inBinSignalsTable[i] = HAL_GPIO_ReadPin(inBinSignalsPortTable[i], inBinSignalsPinTable[i]);
	}
	/* Read also the state of the wipers signal */
	inWipersOnSignal = HAL_GPIO_ReadPin(IN_WIPERS_ON_GPIO_Port, IN_WIPERS_ON_Pin);
}


static void controlWipers(void)
{
	/* Signals to the wipers:
	 * 1. Mass
	 * 2. Switch for slow gear
	 * 3. Switch for fast gear
	 * 4. Constant +12V, engine running and stopping after a bit
	 * 5. +12, engine running constantly
	 *
	 * We are obtaining info from the shifter on the steering wheel
	 * as well as from a switch from panel (0, 1, 2 positions).
	 */

	static TickType_t lastCalledTimerTime = 0;
	int32_t int_tempValue = 0;
	uint32_t uint_tempValue = 0;
	static uint16_t lastCalledTimerWipersTimingValue = 0u;

	uint32_t WIPERS_TIMING_VALUE = WIPERS_TIMING_MINIMUM_IDLE_OFFSET + WIPERS_POTENTIOMETER_ADC_VALUE * 2u;

	HAL_GPIO_WritePin(WIPERS_SIG5_ON_GPIO_Port, WIPERS_SIG5_ON_Pin, RESET);	/* Power for engine OFF */

	/* If the wipers signal is on then proceed */
	if(GPIO_PIN_SET == inWipersOnSignal)
	{
		/* Wipers in timer and slow gear - when switch is in 1 position */
		if((GPIO_PIN_RESET == inBinSignalsTable[WIPERS_SLOW_INPUT_NO]) && (GPIO_PIN_SET == inBinSignalsTable[WIPERS_FAST_INPUT_NO]))
		{
			HAL_GPIO_WritePin(WIPERS_SIG2_ON_GPIO_Port, WIPERS_SIG2_ON_Pin, SET);	/* Slow gear and home switch - it must be ON */
			HAL_GPIO_WritePin(WIPERS_SIG3_ON_GPIO_Port, WIPERS_SIG3_ON_Pin, RESET);	/* Fast gear switch OFF */

			if(pdFALSE == xTimerIsTimerActive(My_Timer_WIPERSHandle))
			{
				HAL_GPIO_WritePin(WIPERS_SIG5_ON_GPIO_Port, WIPERS_SIG5_ON_Pin, SET);	/* Power for engine */
				(void)xTimerChangePeriod(My_Timer_WIPERSHandle, WIPERS_TIMING_VALUE, (TickType_t)0);	/* Change the timer period and Start the timer */
				lastCalledTimerTime = xTaskGetTickCount();	/* Saving when the timer was set to run */
				lastCalledTimerWipersTimingValue = WIPERS_TIMING_VALUE;	/* Saving the value which the timer was started with */
			}
			else /* if the timer is ACTIVE */
			{
				/* calculate difference between the period which the timer was called with and current setting (maybe it has changed in the meantime?) */
				uint_tempValue = (lastCalledTimerWipersTimingValue > WIPERS_TIMING_VALUE) ? (lastCalledTimerWipersTimingValue - WIPERS_TIMING_VALUE) : (WIPERS_TIMING_VALUE - lastCalledTimerWipersTimingValue);
				if(uint_tempValue > WIPERS_TIMING_DELTA_THRESHOLD)	/* If the change is bigger than the minimum threshold then proceed */
				{
					/* Calculate the time which has already passed from starting the timer */
					int_tempValue = WIPERS_TIMING_VALUE - (xTaskGetTickCount() - lastCalledTimerTime);
					uint_tempValue = (int_tempValue > 0) ? int_tempValue : -int_tempValue;

					if(int_tempValue < 0) /* value below 0 means that it passed more time in timer than the set period now and we should run the wipers and new timer */
					{
						HAL_GPIO_WritePin(WIPERS_SIG5_ON_GPIO_Port, WIPERS_SIG5_ON_Pin, SET);	/* Power for engine */
						(void)xTimerChangePeriod(My_Timer_WIPERSHandle, WIPERS_TIMING_VALUE, (TickType_t)0);	/* Change the timer period and Start the timer */
						lastCalledTimerTime = xTaskGetTickCount();	/* Saving when the timer was set to run */
						lastCalledTimerWipersTimingValue = WIPERS_TIMING_VALUE;	/* Saving the value which the timer was started with */
					}
					else	/* value is positive so there passed some time but still some is remaining to run the wipers, so we do not set the WPIERS_SIG5 to SET */
					{
						(void)xTimerChangePeriod(My_Timer_WIPERSHandle, (WIPERS_TIMING_VALUE-uint_tempValue), (TickType_t)0);	/* Change the timer period and Start the timer */
						lastCalledTimerTime = xTaskGetTickCount();	/* Saving when the timer was set to run */
						lastCalledTimerWipersTimingValue = WIPERS_TIMING_VALUE;	/* Saving the value which the timer was started with */
					}
				}
			}
		}
		else /* If there is no more "timer" mode */
		{
			/* Check if there is a timer running - if yes then stop it */
			if(pdFALSE != xTimerIsTimerActive(My_Timer_WIPERSHandle))
			{
				(void)xTimerStop(My_Timer_WIPERSHandle, (TickType_t)0);
			}
		}

		/* Wipers in slow gear - when switch is in 0 position */
		if((GPIO_PIN_SET == inBinSignalsTable[WIPERS_SLOW_INPUT_NO]) && (GPIO_PIN_SET == inBinSignalsTable[WIPERS_FAST_INPUT_NO]))
		{
			HAL_GPIO_WritePin(WIPERS_SIG2_ON_GPIO_Port, WIPERS_SIG2_ON_Pin, SET);	/* Slow gear and home switch - it must be ON */
			HAL_GPIO_WritePin(WIPERS_SIG3_ON_GPIO_Port, WIPERS_SIG3_ON_Pin, RESET);	/* Fast gear switch OFF */
			HAL_GPIO_WritePin(WIPERS_SIG5_ON_GPIO_Port, WIPERS_SIG5_ON_Pin, SET);	/* Power for engine */
		}

		/* Wipers in fast gear - when switch is in 2 position */
		if((GPIO_PIN_SET == inBinSignalsTable[WIPERS_SLOW_INPUT_NO]) && (GPIO_PIN_RESET == inBinSignalsTable[WIPERS_FAST_INPUT_NO]))
		{
			HAL_GPIO_WritePin(WIPERS_SIG2_ON_GPIO_Port, WIPERS_SIG2_ON_Pin, RESET);	/* Slow gear and home switch - it must be OFF for fast gear to work */
			HAL_GPIO_WritePin(WIPERS_SIG3_ON_GPIO_Port, WIPERS_SIG3_ON_Pin, SET);	/* Fast gear switch */
			HAL_GPIO_WritePin(WIPERS_SIG5_ON_GPIO_Port, WIPERS_SIG5_ON_Pin, SET);	/* Power for engine */
		}
	} //if(SET == inWipersOnSignal)
	else	/* If wipers are OFF */
	{
		/* Check if there is a timer running - if yes then stop it */
		if(pdFALSE != xTimerIsTimerActive(My_Timer_WIPERSHandle))
		{
			(void)xTimerStop(My_Timer_WIPERSHandle, (TickType_t)0);
		}

		HAL_GPIO_WritePin(WIPERS_SIG2_ON_GPIO_Port, WIPERS_SIG2_ON_Pin, SET);	/* Slow gear and home switch - it must be always ON */
		HAL_GPIO_WritePin(WIPERS_SIG3_ON_GPIO_Port, WIPERS_SIG3_ON_Pin, RESET);	/* Fast gear switch */
		HAL_GPIO_WritePin(WIPERS_SIG5_ON_GPIO_Port, WIPERS_SIG5_ON_Pin, RESET);	/* Power for engine */
	}
}



void Timer_WIPERS(void const * argument)
{
	HAL_GPIO_WritePin(WIPERS_SIG5_ON_GPIO_Port, WIPERS_SIG5_ON_Pin, SET);	/* Power impulse for engine */
}
