/*
 * Task_LCD.c
 *
 *  Created on: Jan 10, 2021
 *      Author: Wojciech Grzelinski
 */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Includes */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "defines.h"
#include "../../VariousFunctions/Functions.h"
#include "../../EEPROM/EEPROM.h"
#include "../../GPS/GPS_Parsing.h"
#include "../../lcd_hd44780_i2c/lcd_hd44780_i2c.h"
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Defines */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#define ENTER_AreYouSure_MAX_NO_OF_SENT_DATA_PACKETS	2u
#define MAX_POSSIBLE_NUMBER_OF_HOME_SCREENS				5u
#define NUMBER_OF_SCROLLED_LINES	3u
#define LINE1						0u
#define LINE2						1u
#define LINE3						2u
#define STRING_BAR					"bar"
#define STRING_LITERS				"liters"
#define STRING_V					"V"
#define STRING_SEC					"sec"
#define STRING_HOURS				"hours"

typedef struct LCD_board
{
	/* String buffers */
	const char* const name;
	uint8_t nameSize;
	const char* const firstRow;
	uint8_t firstRowSize;
	const char* const secondRow;
	uint8_t secondRowSize;

	/* Pointers to other boards and lists */
	struct LCD_board* previousLayer_ptr;
	struct LCD_board* nextLayer_ptr;
	struct LCD_board* upperLayer_ptr;
	struct LCD_board* lowerLayer_ptr;

	/* Info about the board */
	Enum_Layer const thisLayer;
	void (*RunningFunction)(struct LCD_board* currentBoard);
	void (*EnterFunction)(void);

	/* Controlling value */
	void* value_ptr;
	uint32_t settingsMask;
	Enum_valueType valueType;
	Enum_valueStepSize valueStepSize;
	char* unit;
	uint8_t unitSize;
	const void* const minValue;
	const void* const maxValue;

	/* EEPROM parameters */
	EEPROM_parameters_struct* EEPROMParameters;
	uint16_t EEPROM_memAddress;

} LCD_board;
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#define GET_STR_LENGTH(X) strlen(X)
#define PREPARE_LCD_board(X) \
		X.previousLayer_ptr = NULL;\
		X.nextLayer_ptr 	= NULL;\
		X.upperLayer_ptr 	= NULL;\
		X.lowerLayer_ptr 	= NULL;\
		X.nameSize 			= GET_STR_LENGTH(X.name);\
		X.firstRowSize 		= GET_STR_LENGTH(X.firstRow);\
		X.secondRowSize 	= GET_STR_LENGTH(X.secondRow);\
		X.unitSize 			= GET_STR_LENGTH(X.unit);



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Extern variables */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
extern I2C_HandleTypeDef hi2c2;
extern osMessageQId Queue_EEPROM_readHandle;
extern osMessageQId Queue_EEPROM_writeHandle;

extern EEPROM_parameters_struct EEPROM_car;
extern EEPROM_parameters_struct EEPROM_board;
extern CAR_EEPROM_counters_struct CAR_EEPROM_counters;
extern BOARD_EEPROM_counters_struct BOARD_EEPROM_counters;

extern CAR_mileage_struct CAR_mileage;

extern LCD_parameters_struct LCD;
extern GPS_data_struct GPS;

extern uint8_t tuBylemFLAG;		//TODO: to be deleted finally	(after MicroSd is working)
extern uint8_t TEMPBUFF[100];	//TODO: to be deleted finally (after MicroSd is working)

extern volatile ENCButton_struct ENC_button_menu;
extern volatile int8_t EncoderCounterMenuDiff;

extern LCD_message mainBatteryVoltageValueForLCD;
extern LCD_message auxiliaryBatteryVoltageValueForLCD;
extern LCD_message waterTemperatureValueForLCD;
extern LCD_message totalMileageForLCD;
extern LCD_message tripMileageForLCD;
extern LCD_message RPMForLCD;

extern LCD_message voltage3V3ForLCD;
extern LCD_message voltage5VForLCD;
extern LCD_message voltageInForLCD;
extern LCD_message temperature3V3DCDCForLCD;
extern LCD_message temperature5VDCDCForLCD;

extern waterTempSettings_struct CAR_waterTemp;
extern oilTempSettings_struct CAR_oilTemp;
extern oilPressureSettings_struct CAR_oilPressure;
extern fuelSettings_struct CAR_fuel;
extern batterySettings_struct CAR_mainBattery;
extern batterySettings_struct CAR_auxiliaryBattery;

extern boardVoltagesSettings_struct BOARD_voltage;
extern boardTemperaturesSettings_struct BOARD_temperature;
extern buzzerMainSettings_struct BUZZER_settings;
extern LCDMainSettings_struct LCD_MainSettings;

extern GlobalValuesLimits_struct GlobalValuesLimits;
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Functions prototypes */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
static void ENTER_GoInto(void);
static void ENTER_SaveToEEPROM(void);
static void ENTER_ClearDiagnosticSnapshots(void);
static void ENTER_ClearErrorSnapshots(void);
static void ENTER_AreYouSure(void);

static void RUNNING_ScrollList(struct LCD_board* currentBoard);
static void RUNNING_DesktopLayer(struct LCD_board* currentBoard);
static void RUNNING_GPSLayer(struct LCD_board* currentBoard);
static void RUNNING_CarInfoLayer(struct LCD_board* currentBoard);
static void RUNNING_JarvisInfoLayer(struct LCD_board* currentBoard);
static void RUNNING_Last3Snaps(struct LCD_board* currentBoard);
static void RUNNING_DisplayAndControlValue(struct LCD_board* currentBoard);
static void RUNNING_AreYouSure(struct LCD_board* currentBoard);
static void RUNNING_ClearSnaps(struct LCD_board* currentBoard);
static void RUNNING_ClearTripMileage(struct LCD_board* currentBoard);

static void ScrollForward(LCD_board* displayTable[NUMBER_OF_SCROLLED_LINES], int8_t diff);
static void ScrollBack(LCD_board* displayTable[NUMBER_OF_SCROLLED_LINES], int8_t diff);
static void EEPROMWaitForWriteCheck(EEPROM_data_struct* EEPROMData);
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Local variables */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Buffer of Rows*Columns size for whole display (80 bytes for 4x20) */
uint8_t LCD_buffer[NO_OF_ROWS_IN_LCD][NO_OF_COLUMNS_IN_LCD];
/* Buffer for degree symbol with "C" letter: "*C" */
uint8_t degreeSymbolCharacter[3] = "";

static LCD_board* CurrentBoard_global = NULL;
static LCD_board* HomeScreenBoard = NULL;
static LCD_board* scrollList_currentlyPointedBoard = NULL;
static boolean scrollList_doneOnce = FALSE;
static boolean displayAndControlValue_doneOnce = FALSE;
static boolean last3snaps_doneOnce = FALSE;
static boolean enterAction_save = FALSE;
static boolean EEPROM_Success_Failure_Message = FALSE;



/* LCD boards with its parameters and pointers to others */
static LCD_board LCD_AreYouSure = {.name="<Are You Sure?>",
						.firstRow 			= NULL,
						.secondRow 			= NULL,
						.thisLayer 			= AreYouSure_Layer,
						.RunningFunction 	= RUNNING_AreYouSure,
						.EnterFunction 		= ENTER_AreYouSure,
						.value_ptr 			= NULL,
						.valueType 			= _void_type_,
						.valueStepSize 		= StepNotApplicable,
						.unit 				= NULL,
						.EEPROM_memAddress	= NO_ADDRESS };

static LCD_board LCD_MainMenu = {.name="<Main Menu>",
						.firstRow 			= NULL,
						.secondRow 			= NULL,
						.thisLayer 			= MainMenu_Layer,
						.RunningFunction 	= RUNNING_ScrollList,
						.EnterFunction 		= ENTER_GoInto,
						.value_ptr 			= NULL,
						.valueType 			= _void_type_,
						.valueStepSize 		= StepNotApplicable,
						.unit 				= NULL,
						.EEPROM_memAddress	= NO_ADDRESS };

static LCD_board LCD_Desktop = {.name="<Desktop>",
						.firstRow 			= NULL,
						.secondRow 			= NULL,
						.thisLayer 			= Desktop_Layer,
						.RunningFunction 	= RUNNING_DesktopLayer,
						.EnterFunction 		= NULL,
						.value_ptr 			= NULL,
						.valueType 			= _void_type_,
						.valueStepSize 		= StepNotApplicable,
						.unit 				= NULL,
						.EEPROM_memAddress	= NO_ADDRESS };

static LCD_board LCD_GPS = {.name="<GPS>",
						.firstRow 			= NULL,
						.secondRow 			= NULL,
						.thisLayer 			= GPS_Layer,
						.RunningFunction 	= RUNNING_GPSLayer,
						.EnterFunction 		= NULL,
						.value_ptr 			= NULL,
						.valueType 			= _void_type_,
						.valueStepSize 		= StepNotApplicable,
						.unit 				= NULL,
						.EEPROM_memAddress	= NO_ADDRESS };

static LCD_board LCD_CarInfo = {.name="<Car Information>",
						.firstRow 			= NULL,
						.secondRow 			= NULL,
						.thisLayer 			= CarInfo_Layer,
						.RunningFunction 	= RUNNING_CarInfoLayer,
						.EnterFunction 		= NULL,
						.value_ptr 			= NULL,
						.valueType 			= _void_type_,
						.valueStepSize 		= StepNotApplicable,
						.unit 				= NULL,
						.EEPROM_memAddress	= NO_ADDRESS };

static LCD_board LCD_JarvisInfo = {.name="<Jarvis Info>",
						.firstRow 			= NULL,
						.secondRow 			= NULL,
						.thisLayer 			= JarvisInfo_Layer,
						.RunningFunction 	= RUNNING_JarvisInfoLayer,
						.EnterFunction 		= NULL,
						.value_ptr 			= NULL,
						.valueType 			= _void_type_,
						.valueStepSize 		= StepNotApplicable,
						.unit 				= NULL,
						.EEPROM_memAddress	= NO_ADDRESS };

static LCD_board LCD_Last3DiagSnaps = {.name="<Last 3 Diag Snaps>",
						.firstRow 			= NULL,
						.secondRow 			= NULL,
						.thisLayer 			= Last3Diag_Layer,
						.RunningFunction 	= RUNNING_Last3Snaps,
						.EnterFunction 		= NULL,
						.value_ptr 			= NULL,
						.valueType 			= _void_type_,
						.valueStepSize 		= StepNotApplicable,
						.unit 				= NULL,
						.EEPROM_memAddress	= NO_ADDRESS };

static LCD_board LCD_Last3ErrorSnaps = {.name="<Last 3 Err Snaps>",
						.firstRow 			= NULL,
						.secondRow 			= NULL,
						.thisLayer 			= Last3Err_Layer,
						.RunningFunction 	= RUNNING_Last3Snaps,
						.EnterFunction 		= NULL,
						.value_ptr 			= NULL,
						.valueType 			= _void_type_,
						.valueStepSize 		= StepNotApplicable,
						.unit 				= NULL,
						.EEPROM_memAddress	= NO_ADDRESS };

static LCD_board LCD_CarSettings = {.name="<Car Settings>",
						.firstRow 			= NULL,
						.secondRow 			= NULL,
						.thisLayer 			= CarSettings_Layer,
						.RunningFunction 	= RUNNING_ScrollList,
						.EnterFunction 		= ENTER_GoInto,
						.value_ptr 			= NULL,
						.valueType 			= _void_type_,
						.valueStepSize 		= StepNotApplicable,
						.unit 				= NULL,
						.EEPROM_memAddress	= NO_ADDRESS };

static LCD_board LCD_JarvisSettings = {.name="<Jarvis Settings>",
						.firstRow 			= NULL,
						.secondRow 			= NULL,
						.thisLayer 			= JarvisSettings_Layer,
						.RunningFunction 	= RUNNING_ScrollList,
						.EnterFunction 		= ENTER_GoInto,
						.value_ptr 			= NULL,
						.valueType 			= _void_type_,
						.valueStepSize 		= StepNotApplicable,
						.unit 				= NULL,
						.EEPROM_memAddress	= NO_ADDRESS };

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Car Settings Boards */
static LCD_board LCD_ClearDiagSnaps = {.name="Clear Diag Snaps.",
						.firstRow 			= "Diag. Snaps.:",
						.secondRow 			= "Overflowed?",
						.thisLayer 			= ClearDiagnosticSnapshots,
						.RunningFunction 	= RUNNING_ClearSnaps,
						.EnterFunction 		= ENTER_ClearDiagnosticSnapshots,
						.value_ptr 			= NULL,
						.valueType 			= _void_type_,
						.valueStepSize 		= StepNotApplicable,
						.unit 				= NULL,
						.EEPROM_memAddress	= NO_ADDRESS };

static LCD_board LCD_ClearTripMileage = {.name="Clear Trip Mileage",
						.firstRow 			= "Trip Mileage is:",
						.secondRow 			= NULL,
						.thisLayer 			= ClearTripMileage,
						.RunningFunction 	= RUNNING_ClearTripMileage,
						.EnterFunction 		= NULL,	//TODO: implement calculating and adding mileages, then implement clearing the trip mileage and counters
						.value_ptr 			= NULL,
						.valueType 			= _void_type_,
						.valueStepSize 		= StepNotApplicable,
						.unit 				= NULL,
						.EEPROM_memAddress	= NO_ADDRESS };

static LCD_board LCD_WaterTempSettings = {.name="Water Temp Settings",
						.firstRow 			= NULL,
						.secondRow 			= NULL,
						.thisLayer 			= WaterSettings_Layer,
						.RunningFunction 	= RUNNING_ScrollList,
						.EnterFunction 		= ENTER_GoInto,
						.value_ptr 			= NULL,
						.valueType 			= _void_type_,
						.valueStepSize 		= StepNotApplicable,
						.unit 				= NULL,
						.EEPROM_memAddress	= NO_ADDRESS };

static LCD_board LCD_OilTempSettings = {.name="Oil Temp Settings",
						.firstRow 			= NULL,
						.secondRow 			= NULL,
						.thisLayer 			= OilTempSettings_Layer,
						.RunningFunction 	= RUNNING_ScrollList,
						.EnterFunction 		= ENTER_GoInto,
						.value_ptr 			= NULL,
						.valueType 			= _void_type_,
						.valueStepSize 		= StepNotApplicable,
						.unit 				= NULL,
						.EEPROM_memAddress	= NO_ADDRESS };

static LCD_board LCD_OilPressSettings = {.name="Oil Press Settings",
						.firstRow 			= NULL,
						.secondRow 			= NULL,
						.thisLayer 			= OilPressureSettings_Layer,
						.RunningFunction 	= RUNNING_ScrollList,
						.EnterFunction 		= ENTER_GoInto,
						.value_ptr 			= NULL,
						.valueType 			= _void_type_,
						.valueStepSize 		= StepNotApplicable,
						.unit 				= NULL,
						.EEPROM_memAddress	= NO_ADDRESS };

static LCD_board LCD_FuelSettings = {.name="Fuel Settings",
						.firstRow 			= NULL,
						.secondRow 			= NULL,
						.thisLayer 			= FuelSettings_Layer,
						.RunningFunction 	= RUNNING_ScrollList,
						.EnterFunction 		= ENTER_GoInto,
						.value_ptr 			= NULL,
						.valueType 			= _void_type_,
						.valueStepSize 		= StepNotApplicable,
						.unit 				= NULL,
						.EEPROM_memAddress	= NO_ADDRESS };

static LCD_board LCD_MainBattSettings = {.name="Main Batt Settings",
						.firstRow 			= NULL,
						.secondRow 			= NULL,
						.thisLayer 			= MainBatterySettings_Layer,
						.RunningFunction 	= RUNNING_ScrollList,
						.EnterFunction 		= ENTER_GoInto,
						.value_ptr 			= NULL,
						.valueType 			= _void_type_,
						.valueStepSize 		= StepNotApplicable,
						.unit 				= NULL,
						.EEPROM_memAddress	= NO_ADDRESS };

static LCD_board LCD_AuxBattSettings = {.name="Aux Batt Settings",
						.firstRow 			= NULL,
						.secondRow 			= NULL,
						.thisLayer 			= AuxBatterySettings_Layer,
						.RunningFunction 	= RUNNING_ScrollList,
						.EnterFunction 		= ENTER_GoInto,
						.value_ptr 			= NULL,
						.valueType 			= _void_type_,
						.valueStepSize 		= StepNotApplicable,
						.unit 				= NULL,
						.EEPROM_memAddress	= NO_ADDRESS };

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Jarvis Settings Boards */
static LCD_board LCD_ClearErrorSnap = {.name="Clear Error Snaps",
						.firstRow 			= "Error Snaps.:",
						.secondRow 			= "Overflowed?",
						.thisLayer 			= ClearErrorsSnapshots,
						.RunningFunction 	= RUNNING_ClearSnaps,
						.EnterFunction 		= ENTER_ClearErrorSnapshots,
						.value_ptr 			= NULL,
						.valueType 			= _void_type_,
						.valueStepSize 		= StepNotApplicable,
						.unit 				= NULL,
						.EEPROM_memAddress	= NO_ADDRESS };

static LCD_board LCD_AdjustPolishTime = {.name="Adjust Polish Time",
						.firstRow 			= "Polish Time Zone",
						.secondRow 			= "Adjustment",
						.thisLayer 			= AdjPolishTime,
						.RunningFunction 	= RUNNING_DisplayAndControlValue,
						.EnterFunction 		= ENTER_SaveToEEPROM,
						.value_ptr 			= &(GPS.TimeZoneAdjPoland),
						.valueType 			= _timeHours_type_,
						.valueStepSize 		= StepByOne,
						.unit 				= NULL,
						.EEPROMParameters	= &EEPROM_board,
						.EEPROM_memAddress	= TIME_ZONE_ADJ_POLAND_ADDRESS,
						.minValue			= &(GlobalValuesLimits.polishTimeAdj_min),
						.maxValue			= &(GlobalValuesLimits.polishTimeAdj_max) };

static LCD_board LCD_AdjustTimeZone = {.name="Adjust Time Zone",
						.firstRow 			= "Time Zone Adjustment",
						.secondRow 			= "+- 12 Hours",
						.thisLayer 			= AdjTimeZone,
						.RunningFunction 	= RUNNING_DisplayAndControlValue,
						.EnterFunction 		= ENTER_SaveToEEPROM,
						.value_ptr 			= &(GPS.TimeZoneManualAdj),
						.valueType 			= _timeHours_type_,
						.valueStepSize 		= StepByOne,
						.unit 				= NULL,
						.EEPROMParameters	= &EEPROM_board,
						.EEPROM_memAddress	= TIME_MANUAL_ADJUSTMENT_ADDRESS,
						.minValue			= &(GlobalValuesLimits.timeZoneAdj_min),
						.maxValue			= &(GlobalValuesLimits.timeZoneAdj_max) };

static LCD_board LCD_InternalVoltSett = {.name="Internal Volt Sett",
						.firstRow 			= NULL,
						.secondRow 			= NULL,
						.thisLayer 			= InterVoltSettings_Layer,
						.RunningFunction 	= RUNNING_ScrollList,
						.EnterFunction 		= ENTER_GoInto,
						.value_ptr 			= NULL,
						.valueType 			= _void_type_,
						.valueStepSize 		= StepNotApplicable,
						.unit 				= NULL,
						.EEPROM_memAddress	= NO_ADDRESS };

static LCD_board LCD_InternalTempSett = {.name="Internal Temp Sett",
						.firstRow 			= NULL,
						.secondRow 			= NULL,
						.thisLayer 			= InterTempSettings_Layer,
						.RunningFunction 	= RUNNING_ScrollList,
						.EnterFunction 		= ENTER_GoInto,
						.value_ptr 			= NULL,
						.valueType 			= _void_type_,
						.valueStepSize 		= StepNotApplicable,
						.unit 				= NULL,
						.EEPROM_memAddress	= NO_ADDRESS };

static LCD_board LCD_BuzzerSettings = {.name="Buzzer Settings",
						.firstRow 			= NULL,
						.secondRow 			= NULL,
						.thisLayer 			= BuzzerSettings_Layer,
						.RunningFunction 	= RUNNING_ScrollList,
						.EnterFunction 		= ENTER_GoInto,
						.value_ptr 			= NULL,
						.valueType 			= _void_type_,
						.valueStepSize 		= StepNotApplicable,
						.unit 				= NULL,
						.EEPROM_memAddress	= NO_ADDRESS };

static LCD_board LCD_LCDSettings = {.name="LCD Settings",
						.firstRow 			= NULL,
						.secondRow 			= NULL,
						.thisLayer 			= LCDSettings_Layer,
						.RunningFunction 	= RUNNING_ScrollList,
						.EnterFunction 		= ENTER_GoInto,
						.value_ptr 			= NULL,
						.valueType 			= _void_type_,
						.valueStepSize 		= StepNotApplicable,
						.unit 				= NULL,
						.EEPROM_memAddress	= NO_ADDRESS };

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Water Temperature Settings Boards */
static LCD_board LCD_WaterHighTempWarningThreshold = {.name="High Temp Warn Thr",
						.firstRow 			= "Water High Temp.",
						.secondRow 			= "Warning Threshold",
						.thisLayer 			= WaterHighTempWarningThreshold,
						.RunningFunction 	= RUNNING_DisplayAndControlValue,
						.EnterFunction 		= ENTER_SaveToEEPROM,
						.value_ptr 			= &(CAR_waterTemp.waterHighTempWarningThreshold),
						.valueType 			= _carTemperature_type_,
						.valueStepSize 		= StepByOne,
						.unit 				= NULL,
						.EEPROMParameters	= &EEPROM_car,
						.EEPROM_memAddress	= WATER_HIGH_TEMP_WARNING_THRESHOLD_ADDRESS,
						.minValue			= &(GlobalValuesLimits.waterHighTempWarningThreshold_min),
						.maxValue			= &(GlobalValuesLimits.waterHighTempWarningThreshold_max) };

static LCD_board LCD_WaterHighTempAlarmThreshold = {.name="High Temp Alarm Thr",
						.firstRow 			= "Water High Temp.",
						.secondRow 			= "Alarm Threshold",
						.thisLayer 			= WaterHighTempAlarmThreshold,
						.RunningFunction 	= RUNNING_DisplayAndControlValue,
						.EnterFunction 		= ENTER_SaveToEEPROM,
						.value_ptr 			= &(CAR_waterTemp.waterHighTempAlarmThreshold),
						.valueType 			= _carTemperature_type_,
						.valueStepSize 		= StepByOne,
						.unit 				= NULL,
						.EEPROMParameters	= &EEPROM_car,
						.EEPROM_memAddress	= WATER_HIGH_TEMP_ALARM_THRESHOLD_ADDRESS,
						.minValue			= &(GlobalValuesLimits.waterHighTempAlarmThreshold_min),
						.maxValue			= &(GlobalValuesLimits.waterHighTempAlarmThreshold_max) };

static LCD_board LCD_WaterHighTempFanOnThreshold = {.name="High Temp FanOn Thr",
						.firstRow 			= "Water High Temp.",
						.secondRow 			= "Fan On Threshold",
						.thisLayer 			= WaterHighTempFanOnThreshold,
						.RunningFunction 	= RUNNING_DisplayAndControlValue,
						.EnterFunction 		= ENTER_SaveToEEPROM,
						.value_ptr 			= &(CAR_waterTemp.waterHighTempFanOnThreshold),
						.valueType 			= _carTemperature_type_,
						.valueStepSize 		= StepByOne,
						.unit 				= NULL,
						.EEPROMParameters	= &EEPROM_car,
						.EEPROM_memAddress	= WATER_HIGH_TEMP_FAN_ON_THRESHOLD_ADDRESS,
						.minValue			= &(GlobalValuesLimits.waterHighTempFanOnThreshold_min),
						.maxValue			= &(GlobalValuesLimits.waterHighTempFanOnThreshold_max) };

static LCD_board LCD_WaterHighTempFanOffThreshold = {.name="High Temp FanOff Th",
						.firstRow 			= "Water High Temp.",
						.secondRow 			= "Fan Off Threshold",
						.thisLayer 			= WaterHighTempFanOffThreshold,
						.RunningFunction 	= RUNNING_DisplayAndControlValue,
						.EnterFunction 		= ENTER_SaveToEEPROM,
						.value_ptr 			= &(CAR_waterTemp.waterHighTempFanOffThreshold),
						.valueType 			= _carTemperature_type_,
						.valueStepSize 		= StepByOne,
						.unit 				= NULL,
						.EEPROMParameters	= &EEPROM_car,
						.EEPROM_memAddress	= WATER_HIGH_TEMP_FAN_OFF_THRESHOLD_ADDRESS,
						.minValue			= &(GlobalValuesLimits.waterHighTempFanOffThreshold_min),
						.maxValue			= &(GlobalValuesLimits.waterHighTempFanOffThreshold_max) };

static LCD_board LCD_WaterHighTempWarningOnOff = {.name="Warning On/Off",
						.firstRow 			= "Water High Temp.",
						.secondRow 			= "Warning On/Off",
						.thisLayer 			= WaterTempWarningOn,
						.RunningFunction 	= RUNNING_DisplayAndControlValue,
						.EnterFunction 		= ENTER_SaveToEEPROM,
						.value_ptr 			= &(CAR_waterTemp.allSettings),
						.settingsMask		= 0b00000001,	/* first bit */
						.valueType 			= _boolean_type_,
						.valueStepSize 		= StepByToogling,
						.unit 				= NULL,
						.EEPROMParameters	= &EEPROM_car,
						.EEPROM_memAddress	= WATER_TEMP_ALL_SETTINGS_ADDRESS };

static LCD_board LCD_WaterHighTempAlarmOnOff = {.name="Alarm On/Off",
						.firstRow 			= "Water High Temp.",
						.secondRow 			= "Alarm On/Off",
						.thisLayer 			= WaterTempAlarmOn,
						.RunningFunction 	= RUNNING_DisplayAndControlValue,
						.EnterFunction 		= ENTER_SaveToEEPROM,
						.value_ptr 			= &(CAR_waterTemp.allSettings),
						.settingsMask		= 0b00000010,	/* second bit */
						.valueType 			= _boolean_type_,
						.valueStepSize 		= StepByToogling,
						.unit 				= NULL,
						.EEPROMParameters	= &EEPROM_car,
						.EEPROM_memAddress	= WATER_TEMP_ALL_SETTINGS_ADDRESS };

static LCD_board LCD_WaterHighTempFanControlOnOff = {.name="Fan Control On/Off",
						.firstRow 			= "Water High Temp.",
						.secondRow 			= "Fan Control On/Off",
						.thisLayer 			= WaterFanControlOn,
						.RunningFunction 	= RUNNING_DisplayAndControlValue,
						.EnterFunction 		= ENTER_SaveToEEPROM,
						.value_ptr 			= &(CAR_waterTemp.allSettings),
						.settingsMask		= 0b00000100,	/* third bit */
						.valueType 			= _boolean_type_,
						.valueStepSize 		= StepByToogling,
						.unit 				= NULL,
						.EEPROMParameters	= &EEPROM_car,
						.EEPROM_memAddress	= WATER_TEMP_ALL_SETTINGS_ADDRESS };

static LCD_board LCD_WaterHighTempWarningBuzzerOnOff = {.name="Warn. Buzz. On/Off",
						.firstRow 			= "Water High Temp.",
						.secondRow 			= "Warning Buzzer",
						.thisLayer 			= WaterTempWarningBuzzerOn,
						.RunningFunction 	= RUNNING_DisplayAndControlValue,
						.EnterFunction 		= ENTER_SaveToEEPROM,
						.value_ptr 			= &(CAR_waterTemp.allSettings),
						.settingsMask		= 0b00001000,	/* fourth bit */
						.valueType 			= _boolean_type_,
						.valueStepSize 		= StepByToogling,
						.unit 				= NULL,
						.EEPROMParameters	= &EEPROM_car,
						.EEPROM_memAddress	= WATER_TEMP_ALL_SETTINGS_ADDRESS };

static LCD_board LCD_WaterHighTempAlarmBuzzerOnOff = {.name="Alarm Buzz. On/Off",
						.firstRow 			= "Water High Temp.",
						.secondRow 			= "Alarm Buzzer",
						.thisLayer 			= WaterTempAlarmBuzzerOn,
						.RunningFunction 	= RUNNING_DisplayAndControlValue,
						.EnterFunction 		= ENTER_SaveToEEPROM,
						.value_ptr 			= &(CAR_waterTemp.allSettings),
						.settingsMask		= 0b00010000,	/* fifth bit */
						.valueType 			= _boolean_type_,
						.valueStepSize 		= StepByToogling,
						.unit 				= NULL,
						.EEPROMParameters	= &EEPROM_car,
						.EEPROM_memAddress	= WATER_TEMP_ALL_SETTINGS_ADDRESS };

static LCD_board LCD_WaterHighTempWarningSnapshotOnOff = {.name="Warn. Snap. On/Off",
						.firstRow 			= "Water High Temp.",
						.secondRow 			= "Warning Snapshot",
						.thisLayer 			= WaterTempWarningSnapshotOn,
						.RunningFunction 	= RUNNING_DisplayAndControlValue,
						.EnterFunction 		= ENTER_SaveToEEPROM,
						.value_ptr 			= &(CAR_waterTemp.allSettings),
						.settingsMask		= 0b00100000,	/* sixth bit */
						.valueType 			= _boolean_type_,
						.valueStepSize 		= StepByToogling,
						.unit 				= NULL,
						.EEPROMParameters	= &EEPROM_car,
						.EEPROM_memAddress	= WATER_TEMP_ALL_SETTINGS_ADDRESS };

static LCD_board LCD_WaterHighTempAlarmSnapshotOnOff = {.name="Alarm Snap. On/Off",
						.firstRow 			= "Water High Temp.",
						.secondRow 			= "Alarm Snapshot",
						.thisLayer 			= WaterTempAlarmSnapshotOn,
						.RunningFunction 	= RUNNING_DisplayAndControlValue,
						.EnterFunction 		= ENTER_SaveToEEPROM,
						.value_ptr 			= &(CAR_waterTemp.allSettings),
						.settingsMask		= 0b01000000,	/* seventh bit */
						.valueType 			= _boolean_type_,
						.valueStepSize 		= StepByToogling,
						.unit 				= NULL,
						.EEPROMParameters	= &EEPROM_car,
						.EEPROM_memAddress	= WATER_TEMP_ALL_SETTINGS_ADDRESS };

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Oil Temperature Settings Boards */
static LCD_board LCD_OilHighTempWarningThreshold = {.name="High Temp Warn Thr",
						.firstRow 			= "Oil High Temperature",
						.secondRow 			= "Warning Threshold",
						.thisLayer 			= OilHighTempWarningThreshold,
						.RunningFunction 	= RUNNING_DisplayAndControlValue,
						.EnterFunction 		= ENTER_SaveToEEPROM,
						.value_ptr 			= &(CAR_oilTemp.oilHighTempWarningThreshold),
						.valueType 			= _carTemperature_type_,
						.valueStepSize 		= StepByOne,
						.unit 				= NULL,
						.EEPROMParameters	= &EEPROM_car,
						.EEPROM_memAddress	= OIL_HIGH_TEMP_WARNING_THRESHOLD_ADDRESS,
						.minValue			= &(GlobalValuesLimits.oilHighTempWarningThreshold_min),
						.maxValue			= &(GlobalValuesLimits.oilHighTempWarningThreshold_max) };

static LCD_board LCD_OilHighTempAlarmThreshold = {.name="High Temp Alarm Thr",
						.firstRow 			= "Oil High Temperature",
						.secondRow 			= "Alarm Threshold",
						.thisLayer 			= OilHighTempAlarmThreshold,
						.RunningFunction 	= RUNNING_DisplayAndControlValue,
						.EnterFunction 		= ENTER_SaveToEEPROM,
						.value_ptr 			= &(CAR_oilTemp.oilHighTempAlarmThreshold),
						.valueType 			= _carTemperature_type_,
						.valueStepSize 		= StepByOne,
						.unit 				= NULL,
						.EEPROMParameters	= &EEPROM_car,
						.EEPROM_memAddress	= OIL_HIGH_TEMP_ALARM_THRESHOLD_ADDRESS,
						.minValue			= &(GlobalValuesLimits.oilHighTempAlarmThreshold_min),
						.maxValue			= &(GlobalValuesLimits.oilHighTempAlarmThreshold_max) };

static LCD_board LCD_OilHighTempWarningOnOff = {.name="Warning On/Off",
						.firstRow 			= "Oil High Temperature",
						.secondRow 			= "Warning On/Off",
						.thisLayer 			= OilTempWarningOn,
						.RunningFunction 	= RUNNING_DisplayAndControlValue,
						.EnterFunction 		= ENTER_SaveToEEPROM,
						.value_ptr 			= &(CAR_oilTemp.allSettings),
						.settingsMask		= 0b00000001,	/* first bit */
						.valueType 			= _boolean_type_,
						.valueStepSize 		= StepByToogling,
						.unit 				= NULL,
						.EEPROMParameters	= &EEPROM_car,
						.EEPROM_memAddress	= OIL_TEMP_ALL_SETTINGS_ADDRESS };

static LCD_board LCD_OilHighTempAlarmOnOff = {.name="Alarm On/Off",
						.firstRow 			= "Oil High Temperature",
						.secondRow 			= "Alarm On/Off",
						.thisLayer 			= OilTempAlarmOn,
						.RunningFunction 	= RUNNING_DisplayAndControlValue,
						.EnterFunction 		= ENTER_SaveToEEPROM,
						.value_ptr 			= &(CAR_oilTemp.allSettings),
						.settingsMask		= 0b00000010,	/* second bit */
						.valueType 			= _boolean_type_,
						.valueStepSize 		= StepByToogling,
						.unit 				= NULL,
						.EEPROMParameters	= &EEPROM_car,
						.EEPROM_memAddress	= OIL_TEMP_ALL_SETTINGS_ADDRESS };

static LCD_board LCD_OilHighTempWarningBuzzerOnOff = {.name="Warn. Buzzer On/Off",
						.firstRow 			= "Oil High Temperature",
						.secondRow 			= "Warn. Buzzer On/Off",
						.thisLayer 			= OilTempWarningBuzzerOn,
						.RunningFunction 	= RUNNING_DisplayAndControlValue,
						.EnterFunction 		= ENTER_SaveToEEPROM,
						.value_ptr 			= &(CAR_oilTemp.allSettings),
						.settingsMask		= 0b00000100,	/* third bit */
						.valueType 			= _boolean_type_,
						.valueStepSize 		= StepByToogling,
						.unit 				= NULL,
						.EEPROMParameters	= &EEPROM_car,
						.EEPROM_memAddress	= OIL_TEMP_ALL_SETTINGS_ADDRESS };

static LCD_board LCD_OilHighTempAlarmBuzzerOnOff = {.name="Alarm Buzzer On/Off",
						.firstRow 			= "Oil High Temperature",
						.secondRow 			= "Alarm Buzzer On/Off",
						.thisLayer 			= OilTempAlarmBuzzerOn,
						.RunningFunction 	= RUNNING_DisplayAndControlValue,
						.EnterFunction 		= ENTER_SaveToEEPROM,
						.value_ptr 			= &(CAR_oilTemp.allSettings),
						.settingsMask		= 0b00001000,	/* fourth bit */
						.valueType 			= _boolean_type_,
						.valueStepSize 		= StepByToogling,
						.unit 				= NULL,
						.EEPROMParameters	= &EEPROM_car,
						.EEPROM_memAddress	= OIL_TEMP_ALL_SETTINGS_ADDRESS };

static LCD_board LCD_OilHighTempWarningSnapshotOnOff = {.name="Warn. Snap. On/Off",
						.firstRow 			= "Oil High Temperature",
						.secondRow 			= "Warning Snap. On/Off",
						.thisLayer 			= OilTempWarningSnapshotOn,
						.RunningFunction 	= RUNNING_DisplayAndControlValue,
						.EnterFunction 		= ENTER_SaveToEEPROM,
						.value_ptr 			= &(CAR_oilTemp.allSettings),
						.settingsMask		= 0b00010000,	/* fifth bit */
						.valueType 			= _boolean_type_,
						.valueStepSize 		= StepByToogling,
						.unit 				= NULL,
						.EEPROMParameters	= &EEPROM_car,
						.EEPROM_memAddress	= OIL_TEMP_ALL_SETTINGS_ADDRESS };

static LCD_board LCD_OilHighTempAlarmSnapshotOnOff = {.name="Alarm Snap. On/Off",
						.firstRow 			= "Oil High Temperature",
						.secondRow 			= "Alarm Snap. On/Off",
						.thisLayer 			= OilTempAlarmSpashotOn,
						.RunningFunction 	= RUNNING_DisplayAndControlValue,
						.EnterFunction 		= ENTER_SaveToEEPROM,
						.value_ptr 			= &(CAR_oilTemp.allSettings),
						.settingsMask		= 0b00100000,	/* sixth bit */
						.valueType 			= _boolean_type_,
						.valueStepSize 		= StepByToogling,
						.unit 				= NULL,
						.EEPROMParameters	= &EEPROM_car,
						.EEPROM_memAddress	= OIL_TEMP_ALL_SETTINGS_ADDRESS };

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Oil Pressure Settings Boards */
static LCD_board LCD_OilHighPressureAlarmThreshold = {.name="High Press Alarm Th",
						.firstRow 			= "Oil High Pressure",
						.secondRow 			= "Alarm Threshold",
						.thisLayer 			= OilHighPressureAlarmThreshold,
						.RunningFunction 	= RUNNING_DisplayAndControlValue,
						.EnterFunction 		= ENTER_SaveToEEPROM,
						.value_ptr 			= &(CAR_oilPressure.oilHighPressureAlarmThreshold),
						.valueType 			= _carOilAnalogPressure_type_,
						.valueStepSize 		= StepByOneTen,
						.unit 				= NULL,
						.EEPROMParameters	= &EEPROM_car,
						.EEPROM_memAddress	= OIL_HIGH_PRESSURE_ALARM_THRESHOLD_ADDRESS,
						.minValue			= &(GlobalValuesLimits.oilHighPressureAlarmThreshold_min),
						.maxValue			= &(GlobalValuesLimits.oilHighPressureAlarmThreshold_max) };

static LCD_board LCD_OilLowPressureAlarmThreshold = {.name="Low Press Alarm Thr",
						.firstRow 			= "Oil Low Pressure",
						.secondRow 			= "Alarm Threshold",
						.thisLayer 			= OilLowPressureAlarmThreshold,
						.RunningFunction 	= RUNNING_DisplayAndControlValue,
						.EnterFunction 		= ENTER_SaveToEEPROM,
						.value_ptr 			= &(CAR_oilPressure.oilLowPressureAlarmThreshold),
						.valueType 			= _carOilAnalogPressure_type_,
						.valueStepSize 		= StepByOneTen,
						.unit 				= NULL,
						.EEPROMParameters	= &EEPROM_car,
						.EEPROM_memAddress	= OIL_LOW_PRESSURE_ALARM_THRESHOLD_ADDRESS,
						.minValue			= &(GlobalValuesLimits.oilLowPressureAlarmThreshold_min),
						.maxValue			= &(GlobalValuesLimits.oilLowPressureAlarmThreshold_max) };

static LCD_board LCD_OilPressureAnalogMeasurementOnOff = {.name="Analog Measurement",
						.firstRow 			= "Oil Pressure Analog",
						.secondRow 			= "Measurement On/Off",
						.thisLayer 			= OilPressureAnalogMeasurement,
						.RunningFunction 	= RUNNING_DisplayAndControlValue,
						.EnterFunction 		= ENTER_SaveToEEPROM,
						.value_ptr 			= &(CAR_oilPressure.allSettings),
						.settingsMask		= 0b00000001,	/* first bit */
						.valueType 			= _boolean_type_,
						.valueStepSize 		= StepByToogling,
						.unit 				= NULL,
						.EEPROMParameters	= &EEPROM_car,
						.EEPROM_memAddress	= OIL_PRESSURE_ALL_SETTINGS_ADDRESS };

static LCD_board LCD_OilHighPressureAlarmOnOff = {.name="High Pressure Alarm",
						.firstRow 			= "Oil High Pressure",
						.secondRow 			= "Alarm On/Off",
						.thisLayer 			= OilHighPressureAlarmOn,
						.RunningFunction 	= RUNNING_DisplayAndControlValue,
						.EnterFunction 		= ENTER_SaveToEEPROM,
						.value_ptr 			= &(CAR_oilPressure.allSettings),
						.settingsMask		= 0b00000010,	/* second bit */
						.valueType 			= _boolean_type_,
						.valueStepSize 		= StepByToogling,
						.unit 				= NULL,
						.EEPROMParameters	= &EEPROM_car,
						.EEPROM_memAddress	= OIL_PRESSURE_ALL_SETTINGS_ADDRESS };

static LCD_board LCD_OilLowPressureAlarmOnOff= {.name="Low Pressure Alarm",
						.firstRow 			= "Oil Low Pressure",
						.secondRow 			= "Alarm On/Off",
						.thisLayer 			= OilLowPressureAlarmOn,
						.RunningFunction 	= RUNNING_DisplayAndControlValue,
						.EnterFunction 		= ENTER_SaveToEEPROM,
						.value_ptr 			= &(CAR_oilPressure.allSettings),
						.settingsMask		= 0b00000100,	/* third bit */
						.valueType 			= _boolean_type_,
						.valueStepSize 		= StepByToogling,
						.unit 				= NULL,
						.EEPROMParameters	= &EEPROM_car,
						.EEPROM_memAddress	= OIL_PRESSURE_ALL_SETTINGS_ADDRESS };

static LCD_board LCD_OilPressureAlarmBuzzerOnOff = {.name="Alarm Buzzer On/Off",
						.firstRow 			= "Oil Pressure Alarm",
						.secondRow 			= "Buzzer On/Off",
						.thisLayer 			= OilPressureAlarmBuzzerOn,
						.RunningFunction 	= RUNNING_DisplayAndControlValue,
						.EnterFunction 		= ENTER_SaveToEEPROM,
						.value_ptr 			= &(CAR_oilPressure.allSettings),
						.settingsMask		= 0b00001000,	/* fourth bit */
						.valueType 			= _boolean_type_,
						.valueStepSize 		= StepByToogling,
						.unit 				= NULL,
						.EEPROMParameters	= &EEPROM_car,
						.EEPROM_memAddress	= OIL_PRESSURE_ALL_SETTINGS_ADDRESS };

static LCD_board LCD_OilPressureAlarmSnapshotOnOff = {.name="Alarm Snap. On/Off",
						.firstRow 			= "Oil Pressure Alarm",
						.secondRow 			= "Snapshot On/Off",
						.thisLayer 			= OilPressureAlarmSnapshotOn,
						.RunningFunction 	= RUNNING_DisplayAndControlValue,
						.EnterFunction 		= ENTER_SaveToEEPROM,
						.value_ptr 			= &(CAR_oilPressure.allSettings),
						.settingsMask		= 0b00010000,	/* fifth bit */
						.valueType 			= _boolean_type_,
						.valueStepSize 		= StepByToogling,
						.unit 				= NULL,
						.EEPROMParameters	= &EEPROM_car,
						.EEPROM_memAddress	= OIL_PRESSURE_ALL_SETTINGS_ADDRESS };

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Fuel Settings Boards */
static LCD_board LCD_FuelLowLevelWarningThreshold = {.name="Low Level Threshold",
						.firstRow 			= "Fuel Low Level",
						.secondRow 			= "Warning Threshold",
						.thisLayer 			= FuelLowLevelWarningThreshold,
						.RunningFunction 	= RUNNING_DisplayAndControlValue,
						.EnterFunction 		= ENTER_SaveToEEPROM,
						.value_ptr 			= &(CAR_fuel.fuelLowLevelWarningThreshold),
						.valueType 			= _cafFuelLevel_type_,
						.valueStepSize 		= StepByOne,
						.unit 				= NULL,
						.EEPROMParameters	= &EEPROM_car,
						.EEPROM_memAddress	= FUEL_LOW_LEVEL_WARNING_THRESHOLD_ADDRESS,
						.minValue			= &(GlobalValuesLimits.fuelLowLevelWarningThreshold_min),
						.maxValue			= &(GlobalValuesLimits.fuelLowLevelWarningThreshold_max) };

static LCD_board LCD_FuelLowLevelWarningOnOff = {.name="Lvl Warning On/Off",
						.firstRow 			= "Fuel Low Level",
						.secondRow 			= "Warning On/Off",
						.thisLayer 			= FuelLowLevelWarningOn,
						.RunningFunction 	= RUNNING_DisplayAndControlValue,
						.EnterFunction 		= ENTER_SaveToEEPROM,
						.value_ptr 			= &(CAR_fuel.allSettings),
						.settingsMask		= 0b00000001,	/* first bit */
						.valueType 			= _boolean_type_,
						.valueStepSize 		= StepByToogling,
						.unit 				= NULL,
						.EEPROMParameters	= &EEPROM_car,
						.EEPROM_memAddress	= FUEL_ALL_SETTINGS_ADDRESS };

static LCD_board LCD_FuelLowLevelWarningBuzzerOnOff = {.name="Lvl Buzzer On/Off",
						.firstRow 			= "Fuel Low Level",
						.secondRow 			= "Buzzer On/Off",
						.thisLayer 			= FuelLowLevelWarningBuzzerOn,
						.RunningFunction 	= RUNNING_DisplayAndControlValue,
						.EnterFunction 		= ENTER_SaveToEEPROM,
						.value_ptr 			= &(CAR_fuel.allSettings),
						.settingsMask		= 0b00000010,	/* second bit */
						.valueType 			= _boolean_type_,
						.valueStepSize 		= StepByToogling,
						.unit 				= NULL,
						.EEPROMParameters	= &EEPROM_car,
						.EEPROM_memAddress	= FUEL_ALL_SETTINGS_ADDRESS };

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Main Battery Settings Boards */
static LCD_board LCD_MainBatteryLowVoltageAlarmThreshold = {.name="Low Vol. Alarm Thr.",
						.firstRow 			= "Main Battery Voltage",
						.secondRow 			= "Low Alarm Threshold",
						.thisLayer 			= MainBatteryLowVoltageAlarmThreshold,
						.RunningFunction 	= RUNNING_DisplayAndControlValue,
						.EnterFunction 		= ENTER_SaveToEEPROM,
						.value_ptr 			= &(CAR_mainBattery.batteryLowVoltageAlarmThreshold),
						.valueType 			= _carVoltage_type_,
						.valueStepSize 		= StepByOneHundred,
						.unit 				= NULL,
						.EEPROMParameters	= &EEPROM_car,
						.EEPROM_memAddress	= MAIN_BATTERY_LOW_VOLTAGE_ALARM_THRESHOLD_ADDRESS,
						.minValue			= &(GlobalValuesLimits.batteryLowVoltageAlarmThreshold_min),
						.maxValue			= &(GlobalValuesLimits.batteryLowVoltageAlarmThreshold_max) };

static LCD_board LCD_MainBatteryHighVoltageAlarmThreshold = {.name="High Vol. Alarm Thr",
						.firstRow 			= "Main Battery Voltage",
						.secondRow 			= "High Alarm Threshold",
						.thisLayer 			= MainBatteryHighVoltageAlarmThreshold,
						.RunningFunction 	= RUNNING_DisplayAndControlValue,
						.EnterFunction 		= ENTER_SaveToEEPROM,
						.value_ptr 			= &(CAR_mainBattery.batteryHighVoltageAlarmThreshold),
						.valueType 			= _carVoltage_type_,
						.valueStepSize 		= StepByOneHundred,
						.unit 				= NULL,
						.EEPROMParameters	= &EEPROM_car,
						.EEPROM_memAddress	= MAIN_BATTERY_HIGH_VOLTAGE_ALARM_THRESHOLD_ADDRESS,
						.minValue			= &(GlobalValuesLimits.batteryHighVoltageAlarmThreshold_min),
						.maxValue			= &(GlobalValuesLimits.batteryHighVoltageAlarmThreshold_max) };

static LCD_board LCD_MainBatteryLowVoltageAlarmOnOff = {.name="Low V. Alarm On/Off",
						.firstRow 			= "Main Battery Voltage",
						.secondRow 			= "Low Alarm On/Off",
						.thisLayer 			= MainBatteryLowVoltageAlarmOn,
						.RunningFunction 	= RUNNING_DisplayAndControlValue,
						.EnterFunction 		= ENTER_SaveToEEPROM,
						.value_ptr 			= &(CAR_mainBattery.allSettings),
						.settingsMask		= 0b00000001,	/* first bit */
						.valueType 			= _boolean_type_,
						.valueStepSize 		= StepByToogling,
						.unit 				= NULL,
						.EEPROMParameters	= &EEPROM_car,
						.EEPROM_memAddress	= MAIN_BATTERY_ALL_SETTINGS_ADDRESS };

static LCD_board LCD_MainBatteryHighVoltageAlarmOnOff = {.name="High V Alarm On/Off",
						.firstRow 			= "Main Battery Voltage",
						.secondRow 			= "High Alarm On/Off",
						.thisLayer 			= MainBatteryHighVoltageAlarmOn,
						.RunningFunction 	= RUNNING_DisplayAndControlValue,
						.EnterFunction 		= ENTER_SaveToEEPROM,
						.value_ptr 			= &(CAR_mainBattery.allSettings),
						.settingsMask		= 0b00000010,	/* second bit */
						.valueType 			= _boolean_type_,
						.valueStepSize 		= StepByToogling,
						.unit 				= NULL,
						.EEPROMParameters	= &EEPROM_car,
						.EEPROM_memAddress	= MAIN_BATTERY_ALL_SETTINGS_ADDRESS };

static LCD_board LCD_MainBatteryLowVoltageAlarmBuzzerOnOff = {.name="Low V. Alarm Buzzer",
						.firstRow 			= "Main Battery Low Vol",
						.secondRow 			= "Alarm Buzzer On/Off",
						.thisLayer 			= MainBatteryLowVoltageAlarmBuzzerOn,
						.RunningFunction 	= RUNNING_DisplayAndControlValue,
						.EnterFunction 		= ENTER_SaveToEEPROM,
						.value_ptr 			= &(CAR_mainBattery.allSettings),
						.settingsMask		= 0b00000100,	/* third bit */
						.valueType 			= _boolean_type_,
						.valueStepSize 		= StepByToogling,
						.unit 				= NULL,
						.EEPROMParameters	= &EEPROM_car,
						.EEPROM_memAddress	= MAIN_BATTERY_ALL_SETTINGS_ADDRESS };

static LCD_board LCD_MainBatteryHighVoltageAlarmBuzzerOnOff = {.name="High V Alarm Buzzer",
						.firstRow 			= "Main Battery High V",
						.secondRow 			= "Alarm Buzzer On/Off",
						.thisLayer 			= MainBatteryHighVoltageAlarmBuzzerOn,
						.RunningFunction 	= RUNNING_DisplayAndControlValue,
						.EnterFunction 		= ENTER_SaveToEEPROM,
						.value_ptr 			= &(CAR_mainBattery.allSettings),
						.settingsMask		= 0b00001000,	/* fourth bit */
						.valueType 			= _boolean_type_,
						.valueStepSize 		= StepByToogling,
						.unit 				= NULL,
						.EEPROMParameters	= &EEPROM_car,
						.EEPROM_memAddress	= MAIN_BATTERY_ALL_SETTINGS_ADDRESS };

static LCD_board LCD_MainBatteryLowVoltageSnapshotOnOff = {.name="Low Volt Alarm Snap",
						.firstRow 			= "Main Battery Low V.",
						.secondRow 			= "Alarm Snap. On/Off",
						.thisLayer 			= MainBatteryLowVoltageSnapshotOn,
						.RunningFunction 	= RUNNING_DisplayAndControlValue,
						.EnterFunction 		= ENTER_SaveToEEPROM,
						.value_ptr 			= &(CAR_mainBattery.allSettings),
						.settingsMask		= 0b00010000,	/* fifth bit */
						.valueType 			= _boolean_type_,
						.valueStepSize 		= StepByToogling,
						.unit 				= NULL,
						.EEPROMParameters	= &EEPROM_car,
						.EEPROM_memAddress	= MAIN_BATTERY_ALL_SETTINGS_ADDRESS };

static LCD_board LCD_MainBatteryHighVoltageSnapshotOnOff = {.name="High Vol Alarm Snap",
						.firstRow 			= "Main Battery High V.",
						.secondRow 			= "Alarm Snap. On/Off",
						.thisLayer 			= MainBatteryHighVoltageSnapshotOn,
						.RunningFunction 	= RUNNING_DisplayAndControlValue,
						.EnterFunction 		= ENTER_SaveToEEPROM,
						.value_ptr 			= &(CAR_mainBattery.allSettings),
						.settingsMask		= 0b00100000,	/* sixth bit */
						.valueType 			= _boolean_type_,
						.valueStepSize 		= StepByToogling,
						.unit 				= NULL,
						.EEPROMParameters	= &EEPROM_car,
						.EEPROM_memAddress	= MAIN_BATTERY_ALL_SETTINGS_ADDRESS };

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Auxiliary Battery Settings Boards */
static LCD_board LCD_AuxBatteryLowVoltageAlarmThreshold = {.name="Low Vol. Alarm Thr.",
						.firstRow 			= "Aux Battery Voltage",
						.secondRow 			= "Low Alarm Threshold",
						.thisLayer 			= AuxBatteryLowVoltageAlarmThreshold,
						.RunningFunction 	= RUNNING_DisplayAndControlValue,
						.EnterFunction 		= ENTER_SaveToEEPROM,
						.value_ptr 			= &(CAR_auxiliaryBattery.batteryLowVoltageAlarmThreshold),
						.valueType 			= _carVoltage_type_,
						.valueStepSize 		= StepByOneHundred,
						.unit 				= NULL,
						.EEPROMParameters	= &EEPROM_car,
						.EEPROM_memAddress	= AUXILIARY_BATTERY_LOW_VOLTAGE_ALARM_THRESHOLD_ADDRESS,
						.minValue			= &(GlobalValuesLimits.batteryLowVoltageAlarmThreshold_min),
						.maxValue			= &(GlobalValuesLimits.batteryLowVoltageAlarmThreshold_max) };

static LCD_board LCD_AuxBatteryHighVoltageAlarmThreshold = {.name="High Vol. Alarm Thr",
						.firstRow 			= "Aux Battery Voltage",
						.secondRow 			= "High Alarm Threshold",
						.thisLayer 			= AuxBatteryHighVoltageAlarmThreshold,
						.RunningFunction 	= RUNNING_DisplayAndControlValue,
						.EnterFunction 		= ENTER_SaveToEEPROM,
						.value_ptr 			= &(CAR_auxiliaryBattery.batteryHighVoltageAlarmThreshold),
						.valueType 			= _carVoltage_type_,
						.valueStepSize 		= StepByOneHundred,
						.unit 				= NULL,
						.EEPROMParameters	= &EEPROM_car,
						.EEPROM_memAddress	= AUXILIARY_BATTERY_HIGH_VOLTAGE_ALARM_THRESHOLD_ADDRESS,
						.minValue			= &(GlobalValuesLimits.batteryHighVoltageAlarmThreshold_min),
						.maxValue			= &(GlobalValuesLimits.batteryHighVoltageAlarmThreshold_max) };

static LCD_board LCD_AuxBatteryLowVoltageAlarmOnOff = {.name="Low V. Alarm On/Off",
						.firstRow 			= "Aux Battery Voltage",
						.secondRow 			= "Low Alarm On/Off",
						.thisLayer 			= AuxBatteryLowVoltageAlarmOn,
						.RunningFunction 	= RUNNING_DisplayAndControlValue,
						.EnterFunction 		= ENTER_SaveToEEPROM,
						.value_ptr 			= &(CAR_auxiliaryBattery.allSettings),
						.settingsMask		= 0b00000001,	/* first bit */
						.valueType 			= _boolean_type_,
						.valueStepSize 		= StepByToogling,
						.unit 				= NULL,
						.EEPROMParameters	= &EEPROM_car,
						.EEPROM_memAddress	= AUXILIARY_BATTERY_ALL_SETTINGS_ADDRESS };

static LCD_board LCD_AuxBatteryHighVoltageAlarmOnOff = {.name="High V Alarm On/Off",
						.firstRow 			= "Aux Battery Voltage",
						.secondRow 			= "High Alarm On/Off",
						.thisLayer 			= AuxBatteryHighVoltageAlarmOn,
						.RunningFunction 	= RUNNING_DisplayAndControlValue,
						.EnterFunction 		= ENTER_SaveToEEPROM,
						.value_ptr 			= &(CAR_auxiliaryBattery.allSettings),
						.settingsMask		= 0b00000010,	/* second bit */
						.valueType 			= _boolean_type_,
						.valueStepSize 		= StepByToogling,
						.unit 				= NULL,
						.EEPROMParameters	= &EEPROM_car,
						.EEPROM_memAddress	= AUXILIARY_BATTERY_ALL_SETTINGS_ADDRESS };

static LCD_board LCD_AuxBatteryLowVoltageAlarmBuzzerOnOff = {.name="Low V. Alarm Buzzer",
						.firstRow 			= "Aux Battery Low Vol",
						.secondRow 			= "Alarm Buzzer On/Off",
						.thisLayer 			= AuxBatteryLowVoltageAlarmBuzzerOn,
						.RunningFunction 	= RUNNING_DisplayAndControlValue,
						.EnterFunction 		= ENTER_SaveToEEPROM,
						.value_ptr 			= &(CAR_auxiliaryBattery.allSettings),
						.settingsMask		= 0b00000100,	/* third bit */
						.valueType 			= _boolean_type_,
						.valueStepSize 		= StepByToogling,
						.unit 				= NULL,
						.EEPROMParameters	= &EEPROM_car,
						.EEPROM_memAddress	= AUXILIARY_BATTERY_ALL_SETTINGS_ADDRESS };

static LCD_board LCD_AuxBatteryHighVoltageAlarmBuzzerOnOff = {.name="High V Alarm Buzzer",
						.firstRow 			= "Aux Battery High V",
						.secondRow 			= "Alarm Buzzer On/Off",
						.thisLayer 			= AuxBatteryHighVoltageAlarmBuzzerOn,
						.RunningFunction 	= RUNNING_DisplayAndControlValue,
						.EnterFunction 		= ENTER_SaveToEEPROM,
						.value_ptr 			= &(CAR_auxiliaryBattery.allSettings),
						.settingsMask		= 0b00001000,	/* fourth bit */
						.valueType 			= _boolean_type_,
						.valueStepSize 		= StepByToogling,
						.unit 				= NULL,
						.EEPROMParameters	= &EEPROM_car,
						.EEPROM_memAddress	= AUXILIARY_BATTERY_ALL_SETTINGS_ADDRESS };

static LCD_board LCD_AuxBatteryLowVoltageSnapshotOnOff = {.name="Low Volt Alarm Snap",
						.firstRow 			= "Aux Battery Low V.",
						.secondRow 			= "Alarm Snap. On/Off",
						.thisLayer 			= AuxBatteryLowVoltageSnapshotOn,
						.RunningFunction 	= RUNNING_DisplayAndControlValue,
						.EnterFunction 		= ENTER_SaveToEEPROM,
						.value_ptr 			= &(CAR_auxiliaryBattery.allSettings),
						.settingsMask		= 0b00010000,	/* fifth bit */
						.valueType 			= _boolean_type_,
						.valueStepSize 		= StepByToogling,
						.unit 				= NULL,
						.EEPROMParameters	= &EEPROM_car,
						.EEPROM_memAddress	= AUXILIARY_BATTERY_ALL_SETTINGS_ADDRESS };

static LCD_board LCD_AuxBatteryHighVoltageSnapshotOnOff = {.name="High Vol Alarm Snap",
						.firstRow 			= "Aux Battery High V.",
						.secondRow 			= "Alarm Snap. On/Off",
						.thisLayer 			= AuxBatteryHighVoltageSnapshotOn,
						.RunningFunction 	= RUNNING_DisplayAndControlValue,
						.EnterFunction 		= ENTER_SaveToEEPROM,
						.value_ptr 			= &(CAR_auxiliaryBattery.allSettings),
						.settingsMask		= 0b00100000,	/* sixth bit */
						.valueType 			= _boolean_type_,
						.valueStepSize 		= StepByToogling,
						.unit 				= NULL,
						.EEPROMParameters	= &EEPROM_car,
						.EEPROM_memAddress	= AUXILIARY_BATTERY_ALL_SETTINGS_ADDRESS };

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Internal Voltages Settings Boards */
static LCD_board LCD_Jarvis5VSupplyLowThreshold = {.name="5V Low Threshold",
						.firstRow 			= "5V Supply Voltage",
						.secondRow 			= "Too Low Threshold",
						.thisLayer 			= Supply5VLowThreshold,
						.RunningFunction 	= RUNNING_DisplayAndControlValue,
						.EnterFunction 		= ENTER_SaveToEEPROM,
						.value_ptr 			= &(BOARD_voltage.board5VSupplyLowThreshold),
						.valueType 			= _boardVoltage_type_,
						.valueStepSize 		= StepByOneHundred,
						.unit 				= NULL,
						.EEPROMParameters	= &EEPROM_board,
						.EEPROM_memAddress	= BOARD_5V_SUPPLY_LOW_THRESHOLD_ADDRESS,
						.minValue			= &(GlobalValuesLimits.board5VSupplyLowThreshold_min),
						.maxValue			= &(GlobalValuesLimits.board5VSupplyLowThreshold_max) };

static LCD_board LCD_Jarvis5VSupplyHighThreshold = {.name="5V High Threshold",
						.firstRow 			= "5V Supply Voltage",
						.secondRow 			= "Too High Threshold",
						.thisLayer 			= Supply5VHighThreshold,
						.RunningFunction 	= RUNNING_DisplayAndControlValue,
						.EnterFunction 		= ENTER_SaveToEEPROM,
						.value_ptr 			= &(BOARD_voltage.board5VSupplyHighThreshold),
						.valueType 			= _boardVoltage_type_,
						.valueStepSize 		= StepByOneHundred,
						.unit 				= NULL,
						.EEPROMParameters	= &EEPROM_board,
						.EEPROM_memAddress	= BOARD_5V_SUPPLY_HIGH_THRESHOLD_ADDRESS,
						.minValue			= &(GlobalValuesLimits.board5VSupplyHighThreshold_min),
						.maxValue			= &(GlobalValuesLimits.board5VSupplyHighThreshold_max) };

static LCD_board LCD_Jarvis3V3SupplyLowThreshold = {.name="3V3 Low Threshold",
						.firstRow 			= "3V3 Supply Voltage",
						.secondRow 			= "Too Low Threshold",
						.thisLayer 			= Supply3V3LowThreshold,
						.RunningFunction 	= RUNNING_DisplayAndControlValue,
						.EnterFunction 		= ENTER_SaveToEEPROM,
						.value_ptr 			= &(BOARD_voltage.board3V3SupplyLowThreshold),
						.valueType 			= _boardVoltage_type_,
						.valueStepSize 		= StepByOneHundred,
						.unit 				= NULL,
						.EEPROMParameters	= &EEPROM_board,
						.EEPROM_memAddress	= BOARD_3V3_SUPPLY_LOW_THRESHOLD_ADDRESS,
						.minValue			= &(GlobalValuesLimits.board3V3SupplyLowThreshold_min),
						.maxValue			= &(GlobalValuesLimits.board3V3SupplyLowThreshold_max) };

static LCD_board LCD_Jarvis3V3SupplyHighThreshold = {.name="3V3 High Threshold",
						.firstRow 			= "3V3 Supply Voltage",
						.secondRow 			= "Too High Threshold",
						.thisLayer 			= Supply3V3HighThreshold,
						.RunningFunction 	= RUNNING_DisplayAndControlValue,
						.EnterFunction 		= ENTER_SaveToEEPROM,
						.value_ptr 			= &(BOARD_voltage.board3V3SupplyHighThreshold),
						.valueType 			= _boardVoltage_type_,
						.valueStepSize 		= StepByOneHundred,
						.unit 				= NULL,
						.EEPROMParameters	= &EEPROM_board,
						.EEPROM_memAddress	= BOARD_3V3_SUPPLY_HIGH_THRESHOLD_ADDRESS,
						.minValue			= &(GlobalValuesLimits.board3V3SupplyHighThreshold_min),
						.maxValue			= &(GlobalValuesLimits.board3V3SupplyHighThreshold_max) };

static LCD_board LCD_JarvisVinSupplyLowThreshold = {.name="Vin Low Threshold",
						.firstRow 			= "Vin Supply Voltage",
						.secondRow 			= "Too Low Threshold",
						.thisLayer 			= SupplyVinLowThreshold,
						.RunningFunction 	= RUNNING_DisplayAndControlValue,
						.EnterFunction 		= ENTER_SaveToEEPROM,
						.value_ptr 			= &(BOARD_voltage.boardVinSupplyLowThreshold),
						.valueType 			= _boardVoltage_type_,
						.valueStepSize 		= StepByOneHundred,
						.unit 				= NULL,
						.EEPROMParameters	= &EEPROM_board,
						.EEPROM_memAddress	= BOARD_VIN_SUPPLY_LOW_THRESHOLD_ADDRESS,
						.minValue			= &(GlobalValuesLimits.boardVinSupplyLowThreshold_min),
						.maxValue			= &(GlobalValuesLimits.boardVinSupplyLowThreshold_max) };

static LCD_board LCD_Jarvis3V3SupplyAlarmOnOff = {.name="3V3 Alarm On/Off",
						.firstRow 			= "3V3 Supply Voltage",
						.secondRow 			= "Alarm On/Off",
						.thisLayer 			= Board3V3SupplyAlarmOn,
						.RunningFunction 	= RUNNING_DisplayAndControlValue,
						.EnterFunction 		= ENTER_SaveToEEPROM,
						.value_ptr 			= &(BOARD_voltage.allSettings),
						.settingsMask		= 0b00000001,	/* first bit */
						.valueType 			= _boolean_type_,
						.valueStepSize 		= StepByToogling,
						.unit 				= NULL,
						.EEPROMParameters	= &EEPROM_board,
						.EEPROM_memAddress	= BOARD_VOLTAGE_ALL_SETTINGS_ADDRESS };

static LCD_board LCD_Jarvis3V3SupplyAlarmBuzzerOnOff = {.name="3V3 Alarm Buzzer",
						.firstRow 			= "3V3 Supply Voltage",
						.secondRow 			= "Alarm Buzzer On/Off",
						.thisLayer 			= Board3V3SupplyBuzzerAlarmOn,
						.RunningFunction 	= RUNNING_DisplayAndControlValue,
						.EnterFunction 		= ENTER_SaveToEEPROM,
						.value_ptr 			= &(BOARD_voltage.allSettings),
						.settingsMask		= 0b00000010,	/* second bit */
						.valueType 			= _boolean_type_,
						.valueStepSize 		= StepByToogling,
						.unit 				= NULL,
						.EEPROMParameters	= &EEPROM_board,
						.EEPROM_memAddress	= BOARD_VOLTAGE_ALL_SETTINGS_ADDRESS };

static LCD_board LCD_Jarvis5VSupplyAlarmOnOff = {.name="5V Alarm On/Off",
						.firstRow 			= "5V Supply Voltage",
						.secondRow 			= "Alarm On/Off",
						.thisLayer 			= Board5VSupplyAlarmOn,
						.RunningFunction 	= RUNNING_DisplayAndControlValue,
						.EnterFunction 		= ENTER_SaveToEEPROM,
						.value_ptr 			= &(BOARD_voltage.allSettings),
						.settingsMask		= 0b00000100,	/* third bit */
						.valueType 			= _boolean_type_,
						.valueStepSize 		= StepByToogling,
						.unit 				= NULL,
						.EEPROMParameters	= &EEPROM_board,
						.EEPROM_memAddress	= BOARD_VOLTAGE_ALL_SETTINGS_ADDRESS };

static LCD_board LCD_Jarvis5VSupplyAlarmBuzzerOnOff = {.name="5V Alarm Buzzer",
						.firstRow 			= "5V Supply Voltage",
						.secondRow 			= "Alarm Buzzer On/Off",
						.thisLayer 			= Board5VSupplyBuzzerAlarmOn,
						.RunningFunction 	= RUNNING_DisplayAndControlValue,
						.EnterFunction 		= ENTER_SaveToEEPROM,
						.value_ptr 			= &(BOARD_voltage.allSettings),
						.settingsMask		= 0b00001000,	/* fourth bit */
						.valueType 			= _boolean_type_,
						.valueStepSize 		= StepByToogling,
						.unit 				= NULL,
						.EEPROMParameters	= &EEPROM_board,
						.EEPROM_memAddress	= BOARD_VOLTAGE_ALL_SETTINGS_ADDRESS };

static LCD_board LCD_JarvisVinSupplyAlarmOnOff = {.name="Vin Alarm On/Off",
						.firstRow 			= "Vin Supply Voltage",
						.secondRow 			= "Alarm On/Off",
						.thisLayer 			= BoardVinSupplyAlarmOn,
						.RunningFunction 	= RUNNING_DisplayAndControlValue,
						.EnterFunction 		= ENTER_SaveToEEPROM,
						.value_ptr 			= &(BOARD_voltage.allSettings),
						.settingsMask		= 0b00010000,	/* fifth bit */
						.valueType 			= _boolean_type_,
						.valueStepSize 		= StepByToogling,
						.unit 				= NULL,
						.EEPROMParameters	= &EEPROM_board,
						.EEPROM_memAddress	= BOARD_VOLTAGE_ALL_SETTINGS_ADDRESS };

static LCD_board LCD_JarvisVinSupplyAlarmBuzzerOnOff = {.name="Vin Alarm Buzzer",
						.firstRow 			= "Vin Supply Voltage",
						.secondRow 			= "Alarm Buzzer On/Off",
						.thisLayer 			= BoardVinSupplyBuzzerAlarmOn,
						.RunningFunction 	= RUNNING_DisplayAndControlValue,
						.EnterFunction 		= ENTER_SaveToEEPROM,
						.value_ptr 			= &(BOARD_voltage.allSettings),
						.settingsMask		= 0b00100000,	/* sixth bit */
						.valueType 			= _boolean_type_,
						.valueStepSize 		= StepByToogling,
						.unit 				= NULL,
						.EEPROMParameters	= &EEPROM_board,
						.EEPROM_memAddress	= BOARD_VOLTAGE_ALL_SETTINGS_ADDRESS };

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Internal Temperatures Settings Boards */
static LCD_board LCD_DCDC5VHighTemperatureThreshold = {.name="DCDC 5V H. T. Thr.",
						.firstRow 			= "5V DCDC High",
						.secondRow 			= "Temp. Threshold",
						.thisLayer 			= DCDC5VHighTempThreshold,
						.RunningFunction 	= RUNNING_DisplayAndControlValue,
						.EnterFunction 		= ENTER_SaveToEEPROM,
						.value_ptr 			= &(BOARD_temperature.board5VDCDCTemperatureHighThreshold),
						.valueType 			= _boardTemperature_type_,
						.valueStepSize 		= StepByOne,
						.unit 				= NULL,
						.EEPROMParameters	= &EEPROM_board,
						.EEPROM_memAddress	= BOARD_5V_TEMPERATURE_THRESHOLD_ADDRESS,
						.minValue			= &(GlobalValuesLimits.board5VDCDCTemperatureHighThreshold_min),
						.maxValue			= &(GlobalValuesLimits.board5VDCDCTemperatureHighThreshold_max) };

static LCD_board LCD_DCDC3V3HighTemperatureThreshold = {.name="DCDC 3V3 H. T. Thr.",
						.firstRow 			= "3V3 DCDC High",
						.secondRow 			= "Temp. Threshold",
						.thisLayer 			= DCDC3V3HighTempThreshold,
						.RunningFunction 	= RUNNING_DisplayAndControlValue,
						.EnterFunction 		= ENTER_SaveToEEPROM,
						.value_ptr 			= &(BOARD_temperature.board3V3DCDCTemperatureHighThreshold),
						.valueType 			= _boardTemperature_type_,
						.valueStepSize 		= StepByOne,
						.unit 				= NULL,
						.EEPROMParameters	= &EEPROM_board,
						.EEPROM_memAddress	= BOARD_3V3_TEMPERATURE_THRESHOLD_ADDRESS,
						.minValue			= &(GlobalValuesLimits.board3V3DCDCTemperatureHighThreshold_min),
						.maxValue			= &(GlobalValuesLimits.board3V3DCDCTemperatureHighThreshold_max) };

static LCD_board LCD_HBridgeHighTemperatureThreshold = {.name="H-Bridge H. T. Thr.",
						.firstRow 			= "H-Bridge High",
						.secondRow 			= "Temp. Threshold",
						.thisLayer 			= HBridgeHighTempThreshold,
						.RunningFunction 	= RUNNING_DisplayAndControlValue,
						.EnterFunction 		= ENTER_SaveToEEPROM,
						.value_ptr 			= &(BOARD_temperature.boardHBridgeTemperatureHighThreshold),
						.valueType 			= _boardTemperature_type_,
						.valueStepSize 		= StepByOne,
						.unit 				= NULL,
						.EEPROMParameters	= &EEPROM_board,
						.EEPROM_memAddress	= BOARD_HBRIDGE_TEMPERATURE_THRESHOLD_ADDRESS,
						.minValue			= &(GlobalValuesLimits.boardHBridgeTemperatureHighThreshold_min),
						.maxValue			= &(GlobalValuesLimits.boardHBridgeTemperatureHighThreshold_max) };

static LCD_board LCD_DCDC5VHighTemperatureAlarmOnOff = {.name="DCDC 5V H. T. Alarm",
						.firstRow 			= "5V DCDC High Temp.",
						.secondRow 			= "Alarm On/Off",
						.thisLayer 			= DCDC5VTempHighAlarmOn,
						.RunningFunction 	= RUNNING_DisplayAndControlValue,
						.EnterFunction 		= ENTER_SaveToEEPROM,
						.value_ptr 			= &(BOARD_temperature.allSettings),
						.settingsMask		= 0b00000001,	/* first bit */
						.valueType 			= _boolean_type_,
						.valueStepSize 		= StepByToogling,
						.unit 				= NULL,
						.EEPROMParameters	= &EEPROM_board,
						.EEPROM_memAddress	= BOARD_TEMPERATURE_ALL_SETTINGS_ADDRESS };

static LCD_board LCD_DCDC5VHighTemperatureAlarmBuzzerOnOff = {.name="DCDC 5V Alarm Buzz.",
						.firstRow 			= "5V DCDC High Temp.",
						.secondRow 			= "Alarm Buzzer On/Off",
						.thisLayer 			= DCDC5VTempeHighBuzzAlarmOn,
						.RunningFunction 	= RUNNING_DisplayAndControlValue,
						.EnterFunction 		= ENTER_SaveToEEPROM,
						.value_ptr 			= &(BOARD_temperature.allSettings),
						.settingsMask		= 0b00000010,	/* second bit */
						.valueType 			= _boolean_type_,
						.valueStepSize 		= StepByToogling,
						.unit 				= NULL,
						.EEPROMParameters	= &EEPROM_board,
						.EEPROM_memAddress	= BOARD_TEMPERATURE_ALL_SETTINGS_ADDRESS };

static LCD_board LCD_DCDC3V3HighTemperatureAlarmOnOff = {.name="DCDC 3V3 H. T. Alar",
						.firstRow 			= "3V3 DCDC High Temp.",
						.secondRow 			= "Alarm On/Off",
						.thisLayer 			= DCDC3V3TempHighAlarmOn,
						.RunningFunction 	= RUNNING_DisplayAndControlValue,
						.EnterFunction 		= ENTER_SaveToEEPROM,
						.value_ptr 			= &(BOARD_temperature.allSettings),
						.settingsMask		= 0b00000100,	/* third bit */
						.valueType 			= _boolean_type_,
						.valueStepSize 		= StepByToogling,
						.unit 				= NULL,
						.EEPROMParameters	= &EEPROM_board,
						.EEPROM_memAddress	= BOARD_TEMPERATURE_ALL_SETTINGS_ADDRESS };

static LCD_board LCD_DCDC3V3HighTemperatureAlarmBuzzerOnOff = {.name="DCDC 3V3 Alarm Buzz",
						.firstRow 			= "3V3 DCDC High Temp.",
						.secondRow 			= "Alarm Buzzer On/Off",
						.thisLayer 			= DCDC3V3TempHighBuzzAlarmOn,
						.RunningFunction 	= RUNNING_DisplayAndControlValue,
						.EnterFunction 		= ENTER_SaveToEEPROM,
						.value_ptr 			= &(BOARD_temperature.allSettings),
						.settingsMask		= 0b00001000,	/* fourth bit */
						.valueType 			= _boolean_type_,
						.valueStepSize 		= StepByToogling,
						.unit 				= NULL,
						.EEPROMParameters	= &EEPROM_board,
						.EEPROM_memAddress	= BOARD_TEMPERATURE_ALL_SETTINGS_ADDRESS };

static LCD_board LCD_HBridgeHighTemperatureAlarmOnOff = {.name="H-Bridge H. T. Alar",
						.firstRow 			= "H-Bridge High Temp.",
						.secondRow 			= "Alarm On/Off",
						.thisLayer 			= HBridgeTempHighAlarmOn,
						.RunningFunction 	= RUNNING_DisplayAndControlValue,
						.EnterFunction 		= ENTER_SaveToEEPROM,
						.value_ptr 			= &(BOARD_temperature.allSettings),
						.settingsMask		= 0b00010000,	/* fifth bit */
						.valueType 			= _boolean_type_,
						.valueStepSize 		= StepByToogling,
						.unit 				= NULL,
						.EEPROMParameters	= &EEPROM_board,
						.EEPROM_memAddress	= BOARD_TEMPERATURE_ALL_SETTINGS_ADDRESS, };

static LCD_board LCD_HBridgeHighTemperatureAlarmBuzzerOnOff = {.name="H-Bridge Alarm buzz",
						.firstRow 			= "H-Bridge High Temp.",
						.secondRow 			= "Alarm Buzzer On/Off",
						.thisLayer 			= HBridgeTempHighBuzzAlarmOn,
						.RunningFunction 	= RUNNING_DisplayAndControlValue,
						.EnterFunction 		= ENTER_SaveToEEPROM,
						.value_ptr 			= &(BOARD_temperature.allSettings),
						.settingsMask		= 0b00100000,	/* sixth bit */
						.valueType 			= _boolean_type_,
						.valueStepSize 		= StepByToogling,
						.unit 				= NULL,
						.EEPROMParameters	= &EEPROM_board,
						.EEPROM_memAddress	= BOARD_TEMPERATURE_ALL_SETTINGS_ADDRESS, };

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Buzzer Settings Boards */
static LCD_board LCD_BuzzerMainSwitchOnOff = {.name="Main Buzzer On/Off",
						.firstRow 			= "Buzzer Main Switch",
						.secondRow 			= "All On/Off",
						.thisLayer 			= BuzzerMainSwitch,
						.RunningFunction 	= RUNNING_DisplayAndControlValue,
						.EnterFunction 		= ENTER_SaveToEEPROM,
						.value_ptr 			= &(BUZZER_settings.allSettings),
						.settingsMask		= 0b00000001,	/* first bit */
						.valueType 			= _boolean_type_,
						.valueStepSize 		= StepByToogling,
						.unit 				= NULL,
						.EEPROMParameters	= &EEPROM_board,
						.EEPROM_memAddress	= BOARD_BUZZER_ALL_SETTINGS_ADDRESS };

static LCD_board LCD_BuzzerMainAlarmsSwitchOnOff = {.name="Main Alarms On/Off",
						.firstRow 			= "Buzzer Main Switch",
						.secondRow 			= "All Alarms On/Off",
						.thisLayer 			= BuzzerMainAlarmsSwitch,
						.RunningFunction 	= RUNNING_DisplayAndControlValue,
						.EnterFunction 		= ENTER_SaveToEEPROM,
						.value_ptr 			= &(BUZZER_settings.allSettings),
						.settingsMask		= 0b00000010,	/* second bit */
						.valueType 			= _boolean_type_,
						.valueStepSize 		= StepByToogling,
						.unit 				= NULL,
						.EEPROMParameters	= &EEPROM_board,
						.EEPROM_memAddress	= BOARD_BUZZER_ALL_SETTINGS_ADDRESS };

static LCD_board LCD_BuzzerMainButtonsSwitchOnOff = {.name="Main Buttons On/Off",
						.firstRow 			= "Buzzer Main Switch",
						.secondRow 			= "All Buttons On/Off",
						.thisLayer 			= BuzzerMainButtonsSwitch,
						.RunningFunction 	= RUNNING_DisplayAndControlValue,
						.EnterFunction 		= ENTER_SaveToEEPROM,
						.value_ptr 			= &(BUZZER_settings.allSettings),
						.settingsMask		= 0b00000100,	/* third bit */
						.valueType 			= _boolean_type_,
						.valueStepSize 		= StepByToogling,
						.unit 				= NULL,
						.EEPROMParameters	= &EEPROM_board,
						.EEPROM_memAddress	= BOARD_BUZZER_ALL_SETTINGS_ADDRESS };

static LCD_board LCD_BuzzerWhenShortPressOnOff = {.name="Short Button Buzzer",
						.firstRow 			= "Buzzer Short Button",
						.secondRow 			= "Press On/Off",
						.thisLayer 			= BuzzerWhenShortPress,
						.RunningFunction 	= RUNNING_DisplayAndControlValue,
						.EnterFunction 		= ENTER_SaveToEEPROM,
						.value_ptr 			= &(BUZZER_settings.allSettings),
						.settingsMask		= 0b00001000,	/* fourth bit */
						.valueType 			= _boolean_type_,
						.valueStepSize 		= StepByToogling,
						.unit 				= NULL,
						.EEPROMParameters	= &EEPROM_board,
						.EEPROM_memAddress	= BOARD_BUZZER_ALL_SETTINGS_ADDRESS };

static LCD_board LCD_BuzzerWhenLongPressOnOff = {.name="Long Button Buzzer",
						.firstRow 			= "Buzzer Long Button",
						.secondRow 			= "Press On/Off",
						.thisLayer 			= BuzzerWhenLongPress,
						.RunningFunction 	= RUNNING_DisplayAndControlValue,
						.EnterFunction 		= ENTER_SaveToEEPROM,
						.value_ptr 			= &(BUZZER_settings.allSettings),
						.settingsMask		= 0b00010000,	/* fifth bit */
						.valueType 			= _boolean_type_,
						.valueStepSize 		= StepByToogling,
						.unit 				= NULL,
						.EEPROMParameters	= &EEPROM_board,
						.EEPROM_memAddress	= BOARD_BUZZER_ALL_SETTINGS_ADDRESS };

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* LCD Settings Boards */
static LCD_board LCD_BacklightBrightnessLevel = {.name="Backlight Level",
						.firstRow 			= "Backlight LCD",
						.secondRow 			= "Brightness Level",
						.thisLayer 			= BacklightBrightnessLevel,
						.RunningFunction 	= RUNNING_DisplayAndControlValue,
						.EnterFunction 		= ENTER_SaveToEEPROM,
						.value_ptr 			= &(LCD_MainSettings.backlightLevel),
						.valueType 			= _LCDSettings_type_,
						.valueStepSize 		= StepByOne,
						.unit 				= NULL,
						.EEPROMParameters	= &EEPROM_board,
						.EEPROM_memAddress	= BOARD_LCD_BACKLIGHT_LEVEL_ADDRESS,
						.minValue			= &(GlobalValuesLimits.backlightLevel_min),
						.maxValue			= &(GlobalValuesLimits.backlightLevel_max) };

static LCD_board LCD_SecondsToTurnLCDBacklightOff = {.name="Seconds to LCD Off",
						.firstRow 			= "Seconds to turn LCD",
						.secondRow 			= "Backlight Off",
						.thisLayer 			= SecondsToTurnOffBacklight,
						.RunningFunction 	= RUNNING_DisplayAndControlValue,
						.EnterFunction 		= ENTER_SaveToEEPROM,
						.value_ptr 			= &(LCD_MainSettings.secondsToAutoTurnOffBacklight),
						.valueType 			= _LCDSettings_type_,
						.valueStepSize 		= StepByOne,
						.unit 				= NULL,
						.EEPROMParameters	= &EEPROM_board,
						.EEPROM_memAddress	= BOARD_LCD_SECONDS_TO_AUTO_TURN_OFF_BACKLIGHT_ADDRESS,
						.minValue			= &(GlobalValuesLimits.secondsToAutoTurnOffBacklight_min),
						.maxValue			= &(GlobalValuesLimits.secondsToAutoTurnOffBacklight_max) };

static LCD_board LCD_AutoBacklightOffStartHour = {.name="LCD Off Start Hour",
						.firstRow 			= "Auto Backlight Off",
						.secondRow 			= "Start Hour",
						.thisLayer 			= AutoBacklightOffStartHour,
						.RunningFunction 	= RUNNING_DisplayAndControlValue,
						.EnterFunction 		= ENTER_SaveToEEPROM,
						.value_ptr 			= &(LCD_MainSettings.autoBacklightOffHourStart),
						.valueType 			= _LCDSettings_type_,
						.valueStepSize 		= StepByOne,
						.unit 				= NULL,
						.EEPROMParameters	= &EEPROM_board,
						.EEPROM_memAddress	= BOARD_LCD_AUTO_BACKLIGHT_OFF_HOUR_START_ADDRESS,
						.minValue			= &(GlobalValuesLimits.autoBacklightOffHourStart_min),
						.maxValue			= &(GlobalValuesLimits.autoBacklightOffHourStart_max) };

static LCD_board LCD_AutoBacklightOffEndHour = {.name="LCD Off End Hour",
						.firstRow 			= "Auto Backlight Off",
						.secondRow 			= "End Hour",
						.thisLayer 			= AutoBacklightOffEndHour,
						.RunningFunction 	= RUNNING_DisplayAndControlValue,
						.EnterFunction 		= ENTER_SaveToEEPROM,
						.value_ptr 			= &(LCD_MainSettings.autoBacklightOffHourEnd),
						.valueType 			= _LCDSettings_type_,
						.valueStepSize 		= StepByOne,
						.unit 				= NULL,
						.EEPROMParameters	= &EEPROM_board,
						.EEPROM_memAddress	= BOARD_LCD_AUTO_BACKLIGHT_OFF_HOUR_END_ADDRESS,
						.minValue			= &(GlobalValuesLimits.autoBacklightOffHourEnd_min),
						.maxValue			= &(GlobalValuesLimits.autoBacklightOffHourEnd_max) };

static LCD_board LCD_HomeScreenChoice = {.name="Home Screen",
						.firstRow 			= "Choose The Main",
						.secondRow 			= "(Home) Screen",
						.thisLayer 			= HomeScreen,
						.RunningFunction 	= RUNNING_DisplayAndControlValue,
						.EnterFunction 		= ENTER_SaveToEEPROM,
						.value_ptr 			= &(LCD_MainSettings.homeScreen),
						.valueType 			= _LCD_Enum_Layer_type_,
						.valueStepSize 		= StepByOne,
						.unit 				= NULL,
						.EEPROMParameters	= &EEPROM_board,
						.EEPROM_memAddress	= BOARD_LCD_HOME_SCREEN_ADDRESS,
						.minValue			= &(GlobalValuesLimits.homeScreen_min),
						.maxValue			= &(GlobalValuesLimits.homeScreen_max) };

static LCD_board LCD_AutoHomeReturnTime = {.name="Auto Home Ret. Time",
						.firstRow 			= "Auto Home Return",
						.secondRow 			= "Time",
						.thisLayer 			= AutoHomeReturnTime,
						.RunningFunction 	= RUNNING_DisplayAndControlValue,
						.EnterFunction 		= ENTER_SaveToEEPROM,
						.value_ptr 			= &(LCD_MainSettings.autoHomeReturnTime),
						.valueType 			= _LCDSettings_type_,
						.valueStepSize 		= StepByOne,
						.unit 				= NULL,
						.EEPROMParameters	= &EEPROM_board,
						.EEPROM_memAddress	= BOARD_LCD_AUTO_HOME_RETURN_TIME_ADDRESS,
						.minValue			= &(GlobalValuesLimits.autoHomeReturnTime_min),
						.maxValue			= &(GlobalValuesLimits.autoHomeReturnTime_max) };

static LCD_board LCD_AutoLCDBacklightTurningOffOnOff = {.name="Auto LCD Off On/Off",
						.firstRow 			= "Auto LCD Backlight",
						.secondRow 			= "Turning Off On/Off",
						.thisLayer 			= AutoBacklightOff,
						.RunningFunction 	= RUNNING_DisplayAndControlValue,
						.EnterFunction 		= ENTER_SaveToEEPROM,
						.value_ptr 			= &(LCD_MainSettings.allSettings),
						.settingsMask		= 0b00000001,	/* first bit */
						.valueType 			= _boolean_type_,
						.valueStepSize 		= StepByToogling,
						.unit 				= NULL,
						.EEPROMParameters	= &EEPROM_board,
						.EEPROM_memAddress	= BOARD_LCD_ALL_SETTINGS_ADDRESS };
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* List to search the home screen LCD board */
static const LCD_board* mainScreensList[MAX_POSSIBLE_NUMBER_OF_HOME_SCREENS] = {&LCD_MainMenu, &LCD_Desktop, &LCD_GPS, &LCD_CarInfo, &LCD_JarvisInfo};



void StartLCDTask(void const * argument)
{
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = MY_LCD_TASK_TIME_PERIOD;
	Error_Code error = NO_ERROR;

	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
	/* No better option for making a degree symbol was found so far */
	snprintf((char*)degreeSymbolCharacter, 3, "%cC", DEGREE_SYMBOL_LCD);
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
	/* Setting up the home screen board and setting the first displayed board */
	if(MAX_POSSIBLE_NUMBER_OF_HOME_SCREENS >= (uint8_t)(LCD_MainSettings.homeScreen))
		HomeScreenBoard = (LCD_board*)mainScreensList[(uint8_t)(LCD_MainSettings.homeScreen)];
	else
		HomeScreenBoard = &LCD_Desktop;

	CurrentBoard_global = HomeScreenBoard;
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
// @formatter:off
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
	/* Add units to the proper boards */
	LCD_WaterHighTempWarningThreshold.unit			= (char*)degreeSymbolCharacter;
	LCD_WaterHighTempAlarmThreshold.unit 			= (char*)degreeSymbolCharacter;
	LCD_WaterHighTempFanOnThreshold.unit 			= (char*)degreeSymbolCharacter;
	LCD_WaterHighTempFanOffThreshold.unit 			= (char*)degreeSymbolCharacter;

	LCD_OilHighTempWarningThreshold.unit			= (char*)degreeSymbolCharacter;
	LCD_OilHighTempAlarmThreshold.unit				= (char*)degreeSymbolCharacter;

	LCD_OilHighPressureAlarmThreshold.unit			= STRING_BAR;
	LCD_OilLowPressureAlarmThreshold.unit			= STRING_BAR;

	LCD_FuelLowLevelWarningThreshold.unit			= STRING_LITERS;

	LCD_MainBatteryLowVoltageAlarmThreshold.unit	= STRING_V;
	LCD_MainBatteryHighVoltageAlarmThreshold.unit	= STRING_V;
	LCD_AuxBatteryLowVoltageAlarmThreshold.unit		= STRING_V;
	LCD_AuxBatteryHighVoltageAlarmThreshold.unit	= STRING_V;

	LCD_Jarvis5VSupplyLowThreshold.unit				= STRING_V;
	LCD_Jarvis5VSupplyHighThreshold.unit			= STRING_V;
	LCD_Jarvis3V3SupplyLowThreshold.unit			= STRING_V;
	LCD_Jarvis3V3SupplyHighThreshold.unit			= STRING_V;
	LCD_JarvisVinSupplyLowThreshold.unit			= STRING_V;

	LCD_DCDC5VHighTemperatureThreshold.unit			= (char*)degreeSymbolCharacter;
	LCD_DCDC3V3HighTemperatureThreshold.unit		= (char*)degreeSymbolCharacter;
	LCD_HBridgeHighTemperatureThreshold.unit		= (char*)degreeSymbolCharacter;

	LCD_SecondsToTurnLCDBacklightOff.unit			= STRING_SEC;
	LCD_AutoHomeReturnTime.unit						= STRING_SEC;

	LCD_AdjustPolishTime.unit						= STRING_HOURS;
	LCD_AdjustTimeZone.unit							= STRING_HOURS;
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
	/* Prepare LCD boards to be used - calculate sizes (See Macro definition) */
	PREPARE_LCD_board(LCD_AreYouSure);
	PREPARE_LCD_board(LCD_MainMenu);
	PREPARE_LCD_board(LCD_Desktop);
	PREPARE_LCD_board(LCD_GPS);
	PREPARE_LCD_board(LCD_CarInfo);
	PREPARE_LCD_board(LCD_JarvisInfo);
	PREPARE_LCD_board(LCD_Last3DiagSnaps);
	PREPARE_LCD_board(LCD_Last3ErrorSnaps);
	PREPARE_LCD_board(LCD_CarSettings);
	PREPARE_LCD_board(LCD_JarvisSettings);

	PREPARE_LCD_board(LCD_ClearDiagSnaps);
	PREPARE_LCD_board(LCD_ClearTripMileage);
	PREPARE_LCD_board(LCD_WaterTempSettings);
	PREPARE_LCD_board(LCD_OilTempSettings);
	PREPARE_LCD_board(LCD_OilPressSettings);
	PREPARE_LCD_board(LCD_FuelSettings);
	PREPARE_LCD_board(LCD_MainBattSettings);
	PREPARE_LCD_board(LCD_AuxBattSettings);

	PREPARE_LCD_board(LCD_ClearErrorSnap);
	PREPARE_LCD_board(LCD_AdjustPolishTime);
	PREPARE_LCD_board(LCD_AdjustTimeZone);
	PREPARE_LCD_board(LCD_InternalVoltSett);
	PREPARE_LCD_board(LCD_InternalTempSett);
	PREPARE_LCD_board(LCD_BuzzerSettings);
	PREPARE_LCD_board(LCD_LCDSettings);

	PREPARE_LCD_board(LCD_WaterHighTempWarningThreshold);
	PREPARE_LCD_board(LCD_WaterHighTempAlarmThreshold);
	PREPARE_LCD_board(LCD_WaterHighTempFanOnThreshold);
	PREPARE_LCD_board(LCD_WaterHighTempFanOffThreshold);
	PREPARE_LCD_board(LCD_WaterHighTempWarningOnOff);
	PREPARE_LCD_board(LCD_WaterHighTempAlarmOnOff);
	PREPARE_LCD_board(LCD_WaterHighTempFanControlOnOff);
	PREPARE_LCD_board(LCD_WaterHighTempWarningBuzzerOnOff);
	PREPARE_LCD_board(LCD_WaterHighTempAlarmBuzzerOnOff);
	PREPARE_LCD_board(LCD_WaterHighTempWarningSnapshotOnOff);
	PREPARE_LCD_board(LCD_WaterHighTempAlarmSnapshotOnOff);

	PREPARE_LCD_board(LCD_OilHighTempWarningThreshold);
	PREPARE_LCD_board(LCD_OilHighTempAlarmThreshold);
	PREPARE_LCD_board(LCD_OilHighTempWarningOnOff);
	PREPARE_LCD_board(LCD_OilHighTempAlarmOnOff);
	PREPARE_LCD_board(LCD_OilHighTempWarningBuzzerOnOff);
	PREPARE_LCD_board(LCD_OilHighTempAlarmBuzzerOnOff);
	PREPARE_LCD_board(LCD_OilHighTempWarningSnapshotOnOff);
	PREPARE_LCD_board(LCD_OilHighTempAlarmSnapshotOnOff);

	PREPARE_LCD_board(LCD_OilHighPressureAlarmThreshold);
	PREPARE_LCD_board(LCD_OilLowPressureAlarmThreshold);
	PREPARE_LCD_board(LCD_OilPressureAnalogMeasurementOnOff);
	PREPARE_LCD_board(LCD_OilHighPressureAlarmOnOff);
	PREPARE_LCD_board(LCD_OilLowPressureAlarmOnOff);
	PREPARE_LCD_board(LCD_OilPressureAlarmBuzzerOnOff);
	PREPARE_LCD_board(LCD_OilPressureAlarmSnapshotOnOff);

	PREPARE_LCD_board(LCD_FuelLowLevelWarningThreshold);
	PREPARE_LCD_board(LCD_FuelLowLevelWarningOnOff);
	PREPARE_LCD_board(LCD_FuelLowLevelWarningBuzzerOnOff);

	PREPARE_LCD_board(LCD_MainBatteryLowVoltageAlarmThreshold);
	PREPARE_LCD_board(LCD_MainBatteryHighVoltageAlarmThreshold);
	PREPARE_LCD_board(LCD_MainBatteryLowVoltageAlarmOnOff);
	PREPARE_LCD_board(LCD_MainBatteryHighVoltageAlarmOnOff);
	PREPARE_LCD_board(LCD_MainBatteryLowVoltageAlarmBuzzerOnOff);
	PREPARE_LCD_board(LCD_MainBatteryHighVoltageAlarmBuzzerOnOff);
	PREPARE_LCD_board(LCD_MainBatteryLowVoltageSnapshotOnOff);
	PREPARE_LCD_board(LCD_MainBatteryHighVoltageSnapshotOnOff);

	PREPARE_LCD_board(LCD_AuxBatteryLowVoltageAlarmThreshold);
	PREPARE_LCD_board(LCD_AuxBatteryHighVoltageAlarmThreshold);
	PREPARE_LCD_board(LCD_AuxBatteryLowVoltageAlarmOnOff);
	PREPARE_LCD_board(LCD_AuxBatteryHighVoltageAlarmOnOff);
	PREPARE_LCD_board(LCD_AuxBatteryLowVoltageAlarmBuzzerOnOff);
	PREPARE_LCD_board(LCD_AuxBatteryHighVoltageAlarmBuzzerOnOff);
	PREPARE_LCD_board(LCD_AuxBatteryLowVoltageSnapshotOnOff);
	PREPARE_LCD_board(LCD_AuxBatteryHighVoltageSnapshotOnOff);

	PREPARE_LCD_board(LCD_Jarvis5VSupplyLowThreshold);
	PREPARE_LCD_board(LCD_Jarvis5VSupplyHighThreshold);
	PREPARE_LCD_board(LCD_Jarvis3V3SupplyLowThreshold);
	PREPARE_LCD_board(LCD_Jarvis3V3SupplyHighThreshold);
	PREPARE_LCD_board(LCD_JarvisVinSupplyLowThreshold);
	PREPARE_LCD_board(LCD_Jarvis3V3SupplyAlarmOnOff);
	PREPARE_LCD_board(LCD_Jarvis3V3SupplyAlarmBuzzerOnOff);
	PREPARE_LCD_board(LCD_Jarvis5VSupplyAlarmOnOff);
	PREPARE_LCD_board(LCD_Jarvis5VSupplyAlarmBuzzerOnOff);
	PREPARE_LCD_board(LCD_JarvisVinSupplyAlarmOnOff);
	PREPARE_LCD_board(LCD_JarvisVinSupplyAlarmBuzzerOnOff);

	PREPARE_LCD_board(LCD_DCDC5VHighTemperatureThreshold);
	PREPARE_LCD_board(LCD_DCDC3V3HighTemperatureThreshold);
	PREPARE_LCD_board(LCD_HBridgeHighTemperatureThreshold);
	PREPARE_LCD_board(LCD_DCDC5VHighTemperatureAlarmOnOff);
	PREPARE_LCD_board(LCD_DCDC5VHighTemperatureAlarmBuzzerOnOff);
	PREPARE_LCD_board(LCD_DCDC3V3HighTemperatureAlarmOnOff);
	PREPARE_LCD_board(LCD_DCDC3V3HighTemperatureAlarmBuzzerOnOff);
	PREPARE_LCD_board(LCD_HBridgeHighTemperatureAlarmOnOff);
	PREPARE_LCD_board(LCD_HBridgeHighTemperatureAlarmBuzzerOnOff);

	PREPARE_LCD_board(LCD_BuzzerMainSwitchOnOff);
	PREPARE_LCD_board(LCD_BuzzerMainAlarmsSwitchOnOff);
	PREPARE_LCD_board(LCD_BuzzerMainButtonsSwitchOnOff);
	PREPARE_LCD_board(LCD_BuzzerWhenShortPressOnOff);
	PREPARE_LCD_board(LCD_BuzzerWhenLongPressOnOff);

	PREPARE_LCD_board(LCD_BacklightBrightnessLevel);
	PREPARE_LCD_board(LCD_SecondsToTurnLCDBacklightOff);
	PREPARE_LCD_board(LCD_AutoBacklightOffStartHour);
	PREPARE_LCD_board(LCD_AutoBacklightOffEndHour);
	PREPARE_LCD_board(LCD_HomeScreenChoice);
	PREPARE_LCD_board(LCD_AutoHomeReturnTime);
	PREPARE_LCD_board(LCD_AutoLCDBacklightTurningOffOnOff);
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
	/* Prepare LCD boards to be used - making connections (lists) */
	LCD_MainMenu.lowerLayer_ptr				= &LCD_Desktop;
	LCD_Desktop.upperLayer_ptr 				= &LCD_MainMenu;
	LCD_Desktop.nextLayer_ptr 				= &LCD_GPS;
	LCD_GPS.previousLayer_ptr				= &LCD_Desktop;
	LCD_GPS.upperLayer_ptr 					= &LCD_MainMenu;
	LCD_GPS.nextLayer_ptr 					= &LCD_CarInfo;
	LCD_CarInfo.previousLayer_ptr 			= &LCD_GPS;
	LCD_CarInfo.upperLayer_ptr 				= &LCD_MainMenu;
	LCD_CarInfo.nextLayer_ptr 				= &LCD_JarvisInfo;
	LCD_JarvisInfo.previousLayer_ptr 		= &LCD_CarInfo;
	LCD_JarvisInfo.upperLayer_ptr 			= &LCD_MainMenu;
	LCD_JarvisInfo.nextLayer_ptr 			= &LCD_Last3DiagSnaps;
	LCD_Last3DiagSnaps.previousLayer_ptr 	= &LCD_JarvisInfo;
	LCD_Last3DiagSnaps.upperLayer_ptr 		= &LCD_MainMenu;
	LCD_Last3DiagSnaps.nextLayer_ptr		= &LCD_Last3ErrorSnaps;
	LCD_Last3ErrorSnaps.previousLayer_ptr 	= &LCD_Last3DiagSnaps;
	LCD_Last3ErrorSnaps.upperLayer_ptr 		= &LCD_MainMenu;
	LCD_Last3ErrorSnaps.nextLayer_ptr 		= &LCD_CarSettings;
	LCD_CarSettings.previousLayer_ptr 		= &LCD_Last3ErrorSnaps;
	LCD_CarSettings.lowerLayer_ptr			= &LCD_ClearDiagSnaps;
	LCD_CarSettings.upperLayer_ptr 			= &LCD_MainMenu;
	LCD_CarSettings.nextLayer_ptr 			= &LCD_JarvisSettings;
	LCD_JarvisSettings.lowerLayer_ptr		= &LCD_ClearErrorSnap;
	LCD_JarvisSettings.previousLayer_ptr	= &LCD_Last3ErrorSnaps;
	LCD_JarvisSettings.upperLayer_ptr 		= &LCD_MainMenu;

	/* CarSettings_Layer */
	LCD_ClearDiagSnaps.upperLayer_ptr 		= &LCD_CarSettings;
	LCD_ClearDiagSnaps.nextLayer_ptr 		= &LCD_ClearTripMileage;
	LCD_ClearTripMileage.previousLayer_ptr 	= &LCD_ClearDiagSnaps;
	LCD_ClearTripMileage.upperLayer_ptr 	= &LCD_CarSettings;
	LCD_ClearTripMileage.nextLayer_ptr 		= &LCD_WaterTempSettings;
	LCD_WaterTempSettings.previousLayer_ptr = &LCD_ClearTripMileage;
	LCD_WaterTempSettings.lowerLayer_ptr	= &LCD_WaterHighTempWarningThreshold;
	LCD_WaterTempSettings.upperLayer_ptr 	= &LCD_CarSettings;
	LCD_WaterTempSettings.nextLayer_ptr 	= &LCD_OilTempSettings;
	LCD_OilTempSettings.previousLayer_ptr 	= &LCD_WaterTempSettings;
	LCD_OilTempSettings.lowerLayer_ptr		= &LCD_OilHighTempWarningThreshold;
	LCD_OilTempSettings.upperLayer_ptr 		= &LCD_CarSettings;
	LCD_OilTempSettings.nextLayer_ptr		= &LCD_OilPressSettings;
	LCD_OilPressSettings.previousLayer_ptr 	= &LCD_OilTempSettings;
	LCD_OilPressSettings.upperLayer_ptr 	= &LCD_CarSettings;
	LCD_OilPressSettings.lowerLayer_ptr		= &LCD_OilHighPressureAlarmThreshold;
	LCD_OilPressSettings.nextLayer_ptr 		= &LCD_FuelSettings;
	LCD_FuelSettings.previousLayer_ptr 		= &LCD_OilPressSettings;
	LCD_FuelSettings.upperLayer_ptr 		= &LCD_CarSettings;
	LCD_FuelSettings.lowerLayer_ptr			= &LCD_FuelLowLevelWarningThreshold;
	LCD_FuelSettings.nextLayer_ptr 			= &LCD_MainBattSettings;
	LCD_MainBattSettings.previousLayer_ptr 	= &LCD_FuelSettings;
	LCD_MainBattSettings.upperLayer_ptr		= &LCD_CarSettings;
	LCD_MainBattSettings.lowerLayer_ptr		= &LCD_MainBatteryLowVoltageAlarmThreshold;
	LCD_MainBattSettings.nextLayer_ptr 		= &LCD_AuxBattSettings;
	LCD_AuxBattSettings.previousLayer_ptr 	= &LCD_MainBattSettings;
	LCD_AuxBattSettings.upperLayer_ptr		= &LCD_CarSettings;
	LCD_AuxBattSettings.lowerLayer_ptr		= &LCD_AuxBatteryLowVoltageAlarmThreshold;

	/* JarvisSettings_Layer */
	LCD_ClearErrorSnap.upperLayer_ptr 		= &LCD_JarvisSettings;
	LCD_ClearErrorSnap.nextLayer_ptr 		= &LCD_AdjustPolishTime;
	LCD_AdjustPolishTime.previousLayer_ptr 	= &LCD_ClearErrorSnap;
	LCD_AdjustPolishTime.upperLayer_ptr 	= &LCD_JarvisSettings;
	LCD_AdjustPolishTime.nextLayer_ptr 		= &LCD_AdjustTimeZone;
	LCD_AdjustTimeZone.previousLayer_ptr 	= &LCD_AdjustPolishTime;
	LCD_AdjustTimeZone.upperLayer_ptr 		= &LCD_JarvisSettings;
	LCD_AdjustTimeZone.nextLayer_ptr 		= &LCD_InternalVoltSett;
	LCD_InternalVoltSett.previousLayer_ptr 	= &LCD_AdjustTimeZone;
	LCD_InternalVoltSett.upperLayer_ptr 	= &LCD_JarvisSettings;
	LCD_InternalVoltSett.lowerLayer_ptr		= &LCD_Jarvis5VSupplyLowThreshold;
	LCD_InternalVoltSett.nextLayer_ptr 		= &LCD_InternalTempSett;
	LCD_InternalTempSett.previousLayer_ptr 	= &LCD_InternalVoltSett;
	LCD_InternalTempSett.upperLayer_ptr 	= &LCD_JarvisSettings;
	LCD_InternalTempSett.lowerLayer_ptr		= &LCD_DCDC5VHighTemperatureThreshold;
	LCD_InternalTempSett.nextLayer_ptr 		= &LCD_BuzzerSettings;
	LCD_BuzzerSettings.previousLayer_ptr 	= &LCD_InternalTempSett;
	LCD_BuzzerSettings.upperLayer_ptr 		= &LCD_JarvisSettings;
	LCD_BuzzerSettings.lowerLayer_ptr		= &LCD_BuzzerMainSwitchOnOff;
	LCD_BuzzerSettings.nextLayer_ptr 		= &LCD_LCDSettings;
	LCD_LCDSettings.previousLayer_ptr 		= &LCD_InternalTempSett;
	LCD_LCDSettings.upperLayer_ptr 			= &LCD_JarvisSettings;
	LCD_LCDSettings.lowerLayer_ptr			= &LCD_BacklightBrightnessLevel;

	/* CarSettings_Layer -> WaterSettings_Layer */
	LCD_WaterHighTempWarningThreshold.upperLayer_ptr 		= &LCD_WaterTempSettings;
	LCD_WaterHighTempWarningThreshold.nextLayer_ptr 		= &LCD_WaterHighTempAlarmThreshold;
	LCD_WaterHighTempAlarmThreshold.previousLayer_ptr 		= &LCD_WaterHighTempWarningThreshold;
	LCD_WaterHighTempAlarmThreshold.upperLayer_ptr 			= &LCD_WaterTempSettings;
	LCD_WaterHighTempAlarmThreshold.nextLayer_ptr 			= &LCD_WaterHighTempFanOnThreshold;
	LCD_WaterHighTempFanOnThreshold.previousLayer_ptr 		= &LCD_WaterHighTempAlarmThreshold;
	LCD_WaterHighTempFanOnThreshold.upperLayer_ptr 			= &LCD_WaterTempSettings;
	LCD_WaterHighTempFanOnThreshold.nextLayer_ptr 			= &LCD_WaterHighTempFanOffThreshold;
	LCD_WaterHighTempFanOffThreshold.previousLayer_ptr 		= &LCD_WaterHighTempFanOnThreshold;
	LCD_WaterHighTempFanOffThreshold.upperLayer_ptr 		= &LCD_WaterTempSettings;
	LCD_WaterHighTempFanOffThreshold.nextLayer_ptr 			= &LCD_WaterHighTempWarningOnOff;
	LCD_WaterHighTempWarningOnOff.previousLayer_ptr 		= &LCD_WaterHighTempFanOffThreshold;
	LCD_WaterHighTempWarningOnOff.upperLayer_ptr 			= &LCD_WaterTempSettings;
	LCD_WaterHighTempWarningOnOff.nextLayer_ptr 			= &LCD_WaterHighTempAlarmOnOff;
	LCD_WaterHighTempAlarmOnOff.previousLayer_ptr 			= &LCD_WaterHighTempWarningOnOff;
	LCD_WaterHighTempAlarmOnOff.upperLayer_ptr 				= &LCD_WaterTempSettings;
	LCD_WaterHighTempAlarmOnOff.nextLayer_ptr 				= &LCD_WaterHighTempFanControlOnOff;
	LCD_WaterHighTempFanControlOnOff.previousLayer_ptr 		= &LCD_WaterHighTempAlarmOnOff;
	LCD_WaterHighTempFanControlOnOff.upperLayer_ptr 		= &LCD_WaterTempSettings;
	LCD_WaterHighTempFanControlOnOff.nextLayer_ptr 			= &LCD_WaterHighTempWarningBuzzerOnOff;
	LCD_WaterHighTempWarningBuzzerOnOff.previousLayer_ptr 	= &LCD_WaterHighTempFanControlOnOff;
	LCD_WaterHighTempWarningBuzzerOnOff.upperLayer_ptr 		= &LCD_WaterTempSettings;
	LCD_WaterHighTempWarningBuzzerOnOff.nextLayer_ptr 		= &LCD_WaterHighTempAlarmBuzzerOnOff;
	LCD_WaterHighTempAlarmBuzzerOnOff.previousLayer_ptr 	= &LCD_WaterHighTempWarningBuzzerOnOff;
	LCD_WaterHighTempAlarmBuzzerOnOff.upperLayer_ptr 		= &LCD_WaterTempSettings;
	LCD_WaterHighTempAlarmBuzzerOnOff.nextLayer_ptr 		= &LCD_WaterHighTempWarningSnapshotOnOff;
	LCD_WaterHighTempWarningSnapshotOnOff.previousLayer_ptr = &LCD_WaterHighTempAlarmBuzzerOnOff;
	LCD_WaterHighTempWarningSnapshotOnOff.upperLayer_ptr 	= &LCD_WaterTempSettings;
	LCD_WaterHighTempWarningSnapshotOnOff.nextLayer_ptr 	= &LCD_WaterHighTempAlarmSnapshotOnOff;
	LCD_WaterHighTempAlarmSnapshotOnOff.previousLayer_ptr 	= &LCD_WaterHighTempWarningSnapshotOnOff;
	LCD_WaterHighTempAlarmSnapshotOnOff.upperLayer_ptr 		= &LCD_WaterTempSettings;

	/* CarSettings_Layer -> OilTempSettings_Layer */
	LCD_OilHighTempWarningThreshold.upperLayer_ptr			= &LCD_OilTempSettings;
	LCD_OilHighTempWarningThreshold.nextLayer_ptr			= &LCD_OilHighTempAlarmThreshold;
	LCD_OilHighTempAlarmThreshold.previousLayer_ptr			= &LCD_OilHighTempWarningThreshold;
	LCD_OilHighTempAlarmThreshold.upperLayer_ptr			= &LCD_OilTempSettings;
	LCD_OilHighTempAlarmThreshold.nextLayer_ptr				= &LCD_OilHighTempWarningOnOff;
	LCD_OilHighTempWarningOnOff.previousLayer_ptr			= &LCD_OilHighTempAlarmThreshold;
	LCD_OilHighTempWarningOnOff.upperLayer_ptr				= &LCD_OilTempSettings;
	LCD_OilHighTempWarningOnOff.nextLayer_ptr				= &LCD_OilHighTempAlarmOnOff;
	LCD_OilHighTempAlarmOnOff.previousLayer_ptr				= &LCD_OilHighTempWarningOnOff;
	LCD_OilHighTempAlarmOnOff.upperLayer_ptr				= &LCD_OilTempSettings;
	LCD_OilHighTempAlarmOnOff.nextLayer_ptr					= &LCD_OilHighTempWarningBuzzerOnOff;
	LCD_OilHighTempWarningBuzzerOnOff.previousLayer_ptr		= &LCD_OilHighTempAlarmOnOff;
	LCD_OilHighTempWarningBuzzerOnOff.upperLayer_ptr		= &LCD_OilTempSettings;
	LCD_OilHighTempWarningBuzzerOnOff.nextLayer_ptr			= &LCD_OilHighTempAlarmBuzzerOnOff;
	LCD_OilHighTempAlarmBuzzerOnOff.previousLayer_ptr		= &LCD_OilHighTempWarningBuzzerOnOff;
	LCD_OilHighTempAlarmBuzzerOnOff.upperLayer_ptr			= &LCD_OilTempSettings;
	LCD_OilHighTempAlarmBuzzerOnOff.nextLayer_ptr			= &LCD_OilHighTempWarningSnapshotOnOff;
	LCD_OilHighTempWarningSnapshotOnOff.previousLayer_ptr	= &LCD_OilHighTempAlarmBuzzerOnOff;
	LCD_OilHighTempWarningSnapshotOnOff.upperLayer_ptr		= &LCD_OilTempSettings;
	LCD_OilHighTempWarningSnapshotOnOff.nextLayer_ptr		= &LCD_OilHighTempAlarmSnapshotOnOff;
	LCD_OilHighTempAlarmSnapshotOnOff.previousLayer_ptr		= &LCD_OilHighTempWarningSnapshotOnOff;
	LCD_OilHighTempAlarmSnapshotOnOff.upperLayer_ptr		= &LCD_OilTempSettings;

	/* CarSettings_Layer -> OilPressureSettings_Layer */
	LCD_OilHighPressureAlarmThreshold.upperLayer_ptr		= &LCD_OilPressSettings;
	LCD_OilHighPressureAlarmThreshold.nextLayer_ptr			= &LCD_OilLowPressureAlarmThreshold;
	LCD_OilLowPressureAlarmThreshold.previousLayer_ptr		= &LCD_OilHighPressureAlarmThreshold;
	LCD_OilLowPressureAlarmThreshold.upperLayer_ptr			= &LCD_OilPressSettings;
	LCD_OilLowPressureAlarmThreshold.nextLayer_ptr			= &LCD_OilPressureAnalogMeasurementOnOff;
	LCD_OilPressureAnalogMeasurementOnOff.previousLayer_ptr	= &LCD_OilLowPressureAlarmThreshold;
	LCD_OilPressureAnalogMeasurementOnOff.upperLayer_ptr	= &LCD_OilPressSettings;
	LCD_OilPressureAnalogMeasurementOnOff.nextLayer_ptr		= &LCD_OilHighPressureAlarmOnOff;
	LCD_OilHighPressureAlarmOnOff.previousLayer_ptr			= &LCD_OilPressureAnalogMeasurementOnOff;
	LCD_OilHighPressureAlarmOnOff.upperLayer_ptr			= &LCD_OilPressSettings;
	LCD_OilHighPressureAlarmOnOff.nextLayer_ptr				= &LCD_OilLowPressureAlarmOnOff;
	LCD_OilLowPressureAlarmOnOff.previousLayer_ptr			= &LCD_OilHighPressureAlarmOnOff;
	LCD_OilLowPressureAlarmOnOff.upperLayer_ptr				= &LCD_OilPressSettings;
	LCD_OilLowPressureAlarmOnOff.nextLayer_ptr				= &LCD_OilPressureAlarmBuzzerOnOff;
	LCD_OilPressureAlarmBuzzerOnOff.previousLayer_ptr		= &LCD_OilLowPressureAlarmOnOff;
	LCD_OilPressureAlarmBuzzerOnOff.upperLayer_ptr			= &LCD_OilPressSettings;
	LCD_OilPressureAlarmBuzzerOnOff.nextLayer_ptr			= &LCD_OilPressureAlarmSnapshotOnOff;
	LCD_OilPressureAlarmSnapshotOnOff.previousLayer_ptr		= &LCD_OilPressureAlarmBuzzerOnOff;
	LCD_OilPressureAlarmSnapshotOnOff.upperLayer_ptr		= &LCD_OilPressSettings;

	/* CarSettings_Layer -> FuelSettings_Layer */
	LCD_FuelLowLevelWarningThreshold.upperLayer_ptr			= &LCD_FuelSettings;
	LCD_FuelLowLevelWarningThreshold.nextLayer_ptr			= &LCD_FuelLowLevelWarningOnOff;
	LCD_FuelLowLevelWarningOnOff.previousLayer_ptr			= &LCD_FuelLowLevelWarningThreshold;
	LCD_FuelLowLevelWarningOnOff.upperLayer_ptr				= &LCD_FuelSettings;
	LCD_FuelLowLevelWarningOnOff.nextLayer_ptr				= &LCD_FuelLowLevelWarningBuzzerOnOff;
	LCD_FuelLowLevelWarningBuzzerOnOff.previousLayer_ptr	= &LCD_FuelLowLevelWarningOnOff;
	LCD_FuelLowLevelWarningBuzzerOnOff.upperLayer_ptr		= &LCD_FuelSettings;

	/* CarSettings_Layer -> MainBatterySettings_Layer */
	LCD_MainBatteryLowVoltageAlarmThreshold.upperLayer_ptr		= &LCD_MainBattSettings;
	LCD_MainBatteryLowVoltageAlarmThreshold.nextLayer_ptr		= &LCD_MainBatteryHighVoltageAlarmThreshold;
	LCD_MainBatteryHighVoltageAlarmThreshold.previousLayer_ptr	= &LCD_MainBatteryLowVoltageAlarmThreshold;
	LCD_MainBatteryHighVoltageAlarmThreshold.upperLayer_ptr		= &LCD_MainBattSettings;
	LCD_MainBatteryHighVoltageAlarmThreshold.nextLayer_ptr		= &LCD_MainBatteryLowVoltageAlarmOnOff;
	LCD_MainBatteryLowVoltageAlarmOnOff.previousLayer_ptr		= &LCD_MainBatteryHighVoltageAlarmThreshold;
	LCD_MainBatteryLowVoltageAlarmOnOff.upperLayer_ptr			= &LCD_MainBattSettings;
	LCD_MainBatteryLowVoltageAlarmOnOff.nextLayer_ptr			= &LCD_MainBatteryHighVoltageAlarmOnOff;
	LCD_MainBatteryHighVoltageAlarmOnOff.previousLayer_ptr		= &LCD_MainBatteryLowVoltageAlarmOnOff;
	LCD_MainBatteryHighVoltageAlarmOnOff.upperLayer_ptr			= &LCD_MainBattSettings;
	LCD_MainBatteryHighVoltageAlarmOnOff.nextLayer_ptr			= &LCD_MainBatteryLowVoltageAlarmBuzzerOnOff;
	LCD_MainBatteryLowVoltageAlarmBuzzerOnOff.previousLayer_ptr	= &LCD_MainBatteryHighVoltageAlarmOnOff;
	LCD_MainBatteryLowVoltageAlarmBuzzerOnOff.upperLayer_ptr	= &LCD_MainBattSettings;
	LCD_MainBatteryLowVoltageAlarmBuzzerOnOff.nextLayer_ptr		= &LCD_MainBatteryHighVoltageAlarmBuzzerOnOff;
	LCD_MainBatteryHighVoltageAlarmBuzzerOnOff.previousLayer_ptr= &LCD_MainBatteryLowVoltageAlarmBuzzerOnOff;
	LCD_MainBatteryHighVoltageAlarmBuzzerOnOff.upperLayer_ptr	= &LCD_MainBattSettings;
	LCD_MainBatteryHighVoltageAlarmBuzzerOnOff.nextLayer_ptr	= &LCD_MainBatteryLowVoltageSnapshotOnOff;
	LCD_MainBatteryLowVoltageSnapshotOnOff.previousLayer_ptr	= &LCD_MainBatteryHighVoltageAlarmBuzzerOnOff;
	LCD_MainBatteryLowVoltageSnapshotOnOff.upperLayer_ptr		= &LCD_MainBattSettings;
	LCD_MainBatteryLowVoltageSnapshotOnOff.nextLayer_ptr		= &LCD_MainBatteryHighVoltageSnapshotOnOff;
	LCD_MainBatteryHighVoltageSnapshotOnOff.previousLayer_ptr	= &LCD_MainBatteryLowVoltageSnapshotOnOff;
	LCD_MainBatteryHighVoltageSnapshotOnOff.upperLayer_ptr		= &LCD_MainBattSettings;

	/* CarSettings_Layer -> AuxBatterySettings_Layer */
	LCD_AuxBatteryLowVoltageAlarmThreshold.upperLayer_ptr		= &LCD_AuxBattSettings;
	LCD_AuxBatteryLowVoltageAlarmThreshold.nextLayer_ptr		= &LCD_AuxBatteryHighVoltageAlarmThreshold;
	LCD_AuxBatteryHighVoltageAlarmThreshold.previousLayer_ptr	= &LCD_AuxBatteryLowVoltageAlarmThreshold;
	LCD_AuxBatteryHighVoltageAlarmThreshold.upperLayer_ptr		= &LCD_AuxBattSettings;
	LCD_AuxBatteryHighVoltageAlarmThreshold.nextLayer_ptr		= &LCD_AuxBatteryLowVoltageAlarmOnOff;
	LCD_AuxBatteryLowVoltageAlarmOnOff.previousLayer_ptr		= &LCD_AuxBatteryHighVoltageAlarmThreshold;
	LCD_AuxBatteryLowVoltageAlarmOnOff.upperLayer_ptr			= &LCD_AuxBattSettings;
	LCD_AuxBatteryLowVoltageAlarmOnOff.nextLayer_ptr			= &LCD_AuxBatteryHighVoltageAlarmOnOff;
	LCD_AuxBatteryHighVoltageAlarmOnOff.previousLayer_ptr		= &LCD_AuxBatteryLowVoltageAlarmOnOff;
	LCD_AuxBatteryHighVoltageAlarmOnOff.upperLayer_ptr			= &LCD_AuxBattSettings;
	LCD_AuxBatteryHighVoltageAlarmOnOff.nextLayer_ptr			= &LCD_AuxBatteryLowVoltageAlarmBuzzerOnOff;
	LCD_AuxBatteryLowVoltageAlarmBuzzerOnOff.previousLayer_ptr	= &LCD_AuxBatteryHighVoltageAlarmOnOff;
	LCD_AuxBatteryLowVoltageAlarmBuzzerOnOff.upperLayer_ptr		= &LCD_AuxBattSettings;
	LCD_AuxBatteryLowVoltageAlarmBuzzerOnOff.nextLayer_ptr		= &LCD_AuxBatteryHighVoltageAlarmBuzzerOnOff;
	LCD_AuxBatteryHighVoltageAlarmBuzzerOnOff.previousLayer_ptr	= &LCD_AuxBatteryLowVoltageAlarmBuzzerOnOff;
	LCD_AuxBatteryHighVoltageAlarmBuzzerOnOff.upperLayer_ptr	= &LCD_AuxBattSettings;
	LCD_AuxBatteryHighVoltageAlarmBuzzerOnOff.nextLayer_ptr		= &LCD_AuxBatteryLowVoltageSnapshotOnOff;
	LCD_AuxBatteryLowVoltageSnapshotOnOff.previousLayer_ptr		= &LCD_AuxBatteryHighVoltageAlarmBuzzerOnOff;
	LCD_AuxBatteryLowVoltageSnapshotOnOff.upperLayer_ptr		= &LCD_AuxBattSettings;
	LCD_AuxBatteryLowVoltageSnapshotOnOff.nextLayer_ptr			= &LCD_AuxBatteryHighVoltageSnapshotOnOff;
	LCD_AuxBatteryHighVoltageSnapshotOnOff.previousLayer_ptr	= &LCD_AuxBatteryLowVoltageSnapshotOnOff;
	LCD_AuxBatteryHighVoltageSnapshotOnOff.upperLayer_ptr		= &LCD_AuxBattSettings;

	/* JarvisSettings_Layer -> InterVoltSettings_Layer */
	LCD_Jarvis5VSupplyLowThreshold.upperLayer_ptr				= &LCD_InternalVoltSett;
	LCD_Jarvis5VSupplyLowThreshold.nextLayer_ptr				= &LCD_Jarvis5VSupplyHighThreshold;
	LCD_Jarvis5VSupplyHighThreshold.previousLayer_ptr			= &LCD_Jarvis5VSupplyLowThreshold;
	LCD_Jarvis5VSupplyHighThreshold.upperLayer_ptr				= &LCD_InternalVoltSett;
	LCD_Jarvis5VSupplyHighThreshold.nextLayer_ptr				= &LCD_Jarvis3V3SupplyLowThreshold;
	LCD_Jarvis3V3SupplyLowThreshold.previousLayer_ptr			= &LCD_Jarvis5VSupplyHighThreshold;
	LCD_Jarvis3V3SupplyLowThreshold.upperLayer_ptr				= &LCD_InternalVoltSett;
	LCD_Jarvis3V3SupplyLowThreshold.nextLayer_ptr				= &LCD_Jarvis3V3SupplyHighThreshold;
	LCD_Jarvis3V3SupplyHighThreshold.previousLayer_ptr			= &LCD_Jarvis3V3SupplyLowThreshold;
	LCD_Jarvis3V3SupplyHighThreshold.upperLayer_ptr				= &LCD_InternalVoltSett;
	LCD_Jarvis3V3SupplyHighThreshold.nextLayer_ptr				= &LCD_JarvisVinSupplyLowThreshold;
	LCD_JarvisVinSupplyLowThreshold.previousLayer_ptr			= &LCD_Jarvis3V3SupplyHighThreshold;
	LCD_JarvisVinSupplyLowThreshold.upperLayer_ptr				= &LCD_InternalVoltSett;
	LCD_JarvisVinSupplyLowThreshold.nextLayer_ptr				= &LCD_Jarvis3V3SupplyAlarmOnOff;
	LCD_Jarvis3V3SupplyAlarmOnOff.previousLayer_ptr				= &LCD_JarvisVinSupplyLowThreshold;
	LCD_Jarvis3V3SupplyAlarmOnOff.upperLayer_ptr				= &LCD_InternalVoltSett;
	LCD_Jarvis3V3SupplyAlarmOnOff.nextLayer_ptr					= &LCD_Jarvis3V3SupplyAlarmBuzzerOnOff;
	LCD_Jarvis3V3SupplyAlarmBuzzerOnOff.previousLayer_ptr		= &LCD_Jarvis3V3SupplyAlarmOnOff;
	LCD_Jarvis3V3SupplyAlarmBuzzerOnOff.upperLayer_ptr			= &LCD_InternalVoltSett;
	LCD_Jarvis3V3SupplyAlarmBuzzerOnOff.nextLayer_ptr			= &LCD_Jarvis5VSupplyAlarmOnOff;
	LCD_Jarvis5VSupplyAlarmOnOff.previousLayer_ptr				= &LCD_Jarvis3V3SupplyAlarmBuzzerOnOff;
	LCD_Jarvis5VSupplyAlarmOnOff.upperLayer_ptr					= &LCD_InternalVoltSett;
	LCD_Jarvis5VSupplyAlarmOnOff.nextLayer_ptr					= &LCD_Jarvis5VSupplyAlarmBuzzerOnOff;
	LCD_Jarvis5VSupplyAlarmBuzzerOnOff.previousLayer_ptr		= &LCD_Jarvis5VSupplyAlarmOnOff;
	LCD_Jarvis5VSupplyAlarmBuzzerOnOff.upperLayer_ptr			= &LCD_InternalVoltSett;
	LCD_Jarvis5VSupplyAlarmBuzzerOnOff.nextLayer_ptr			= &LCD_JarvisVinSupplyAlarmOnOff;
	LCD_JarvisVinSupplyAlarmOnOff.previousLayer_ptr				= &LCD_Jarvis5VSupplyAlarmBuzzerOnOff;
	LCD_JarvisVinSupplyAlarmOnOff.upperLayer_ptr				= &LCD_InternalVoltSett;
	LCD_JarvisVinSupplyAlarmOnOff.nextLayer_ptr					= &LCD_JarvisVinSupplyAlarmBuzzerOnOff;
	LCD_JarvisVinSupplyAlarmBuzzerOnOff.previousLayer_ptr		= &LCD_JarvisVinSupplyAlarmOnOff;
	LCD_JarvisVinSupplyAlarmBuzzerOnOff.upperLayer_ptr			= &LCD_InternalVoltSett;

	/* JarvisSettings_Layer -> InterTempSettings_Layer */
	LCD_DCDC5VHighTemperatureThreshold.upperLayer_ptr			= &LCD_InternalTempSett;
	LCD_DCDC5VHighTemperatureThreshold.nextLayer_ptr			= &LCD_DCDC3V3HighTemperatureThreshold;
	LCD_DCDC3V3HighTemperatureThreshold.previousLayer_ptr		= &LCD_DCDC5VHighTemperatureThreshold;
	LCD_DCDC3V3HighTemperatureThreshold.upperLayer_ptr			= &LCD_InternalTempSett;
	LCD_DCDC3V3HighTemperatureThreshold.nextLayer_ptr			= &LCD_HBridgeHighTemperatureThreshold;
	LCD_HBridgeHighTemperatureThreshold.previousLayer_ptr		= &LCD_DCDC3V3HighTemperatureThreshold;
	LCD_HBridgeHighTemperatureThreshold.upperLayer_ptr			= &LCD_InternalTempSett;
	LCD_HBridgeHighTemperatureThreshold.nextLayer_ptr			= &LCD_DCDC5VHighTemperatureAlarmOnOff;
	LCD_DCDC5VHighTemperatureAlarmOnOff.previousLayer_ptr		= &LCD_HBridgeHighTemperatureThreshold;
	LCD_DCDC5VHighTemperatureAlarmOnOff.upperLayer_ptr			= &LCD_InternalTempSett;
	LCD_DCDC5VHighTemperatureAlarmOnOff.nextLayer_ptr			= &LCD_DCDC5VHighTemperatureAlarmBuzzerOnOff;
	LCD_DCDC5VHighTemperatureAlarmBuzzerOnOff.previousLayer_ptr	= &LCD_DCDC5VHighTemperatureAlarmOnOff;
	LCD_DCDC5VHighTemperatureAlarmBuzzerOnOff.upperLayer_ptr	= &LCD_InternalTempSett;
	LCD_DCDC5VHighTemperatureAlarmBuzzerOnOff.nextLayer_ptr		= &LCD_DCDC3V3HighTemperatureAlarmOnOff;
	LCD_DCDC3V3HighTemperatureAlarmOnOff.previousLayer_ptr		= &LCD_DCDC5VHighTemperatureAlarmBuzzerOnOff;
	LCD_DCDC3V3HighTemperatureAlarmOnOff.upperLayer_ptr			= &LCD_InternalTempSett;
	LCD_DCDC3V3HighTemperatureAlarmOnOff.nextLayer_ptr			= &LCD_DCDC3V3HighTemperatureAlarmBuzzerOnOff;
	LCD_DCDC3V3HighTemperatureAlarmBuzzerOnOff.previousLayer_ptr= &LCD_DCDC3V3HighTemperatureAlarmOnOff;
	LCD_DCDC3V3HighTemperatureAlarmBuzzerOnOff.upperLayer_ptr	= &LCD_InternalTempSett;
	LCD_DCDC3V3HighTemperatureAlarmBuzzerOnOff.nextLayer_ptr	= &LCD_HBridgeHighTemperatureAlarmOnOff;
	LCD_HBridgeHighTemperatureAlarmOnOff.previousLayer_ptr		= &LCD_DCDC3V3HighTemperatureAlarmBuzzerOnOff;
	LCD_HBridgeHighTemperatureAlarmOnOff.upperLayer_ptr			= &LCD_InternalTempSett;
	LCD_HBridgeHighTemperatureAlarmOnOff.nextLayer_ptr			= &LCD_HBridgeHighTemperatureAlarmBuzzerOnOff;
	LCD_HBridgeHighTemperatureAlarmBuzzerOnOff.previousLayer_ptr= &LCD_HBridgeHighTemperatureAlarmOnOff;
	LCD_HBridgeHighTemperatureAlarmBuzzerOnOff.upperLayer_ptr	= &LCD_InternalTempSett;

	/* JarvisSettings_Layer -> BuzzerSettings_Layer */
	LCD_BuzzerMainSwitchOnOff.upperLayer_ptr					= &LCD_BuzzerSettings;
	LCD_BuzzerMainSwitchOnOff.nextLayer_ptr						= &LCD_BuzzerMainAlarmsSwitchOnOff;
	LCD_BuzzerMainAlarmsSwitchOnOff.previousLayer_ptr			= &LCD_BuzzerMainSwitchOnOff;
	LCD_BuzzerMainAlarmsSwitchOnOff.upperLayer_ptr				= &LCD_BuzzerSettings;
	LCD_BuzzerMainAlarmsSwitchOnOff.nextLayer_ptr				= &LCD_BuzzerMainButtonsSwitchOnOff;
	LCD_BuzzerMainButtonsSwitchOnOff.previousLayer_ptr			= &LCD_BuzzerMainAlarmsSwitchOnOff;
	LCD_BuzzerMainButtonsSwitchOnOff.upperLayer_ptr				= &LCD_BuzzerSettings;
	LCD_BuzzerMainButtonsSwitchOnOff.nextLayer_ptr				= &LCD_BuzzerWhenShortPressOnOff;
	LCD_BuzzerWhenShortPressOnOff.previousLayer_ptr				= &LCD_BuzzerMainButtonsSwitchOnOff;
	LCD_BuzzerWhenShortPressOnOff.upperLayer_ptr				= &LCD_BuzzerSettings;
	LCD_BuzzerWhenShortPressOnOff.nextLayer_ptr					= &LCD_BuzzerWhenLongPressOnOff;
	LCD_BuzzerWhenLongPressOnOff.previousLayer_ptr				= &LCD_BuzzerWhenShortPressOnOff;
	LCD_BuzzerWhenLongPressOnOff.upperLayer_ptr					= &LCD_BuzzerSettings;

	/* JarvisSettings_Layer -> LCDSettings_Layer */
	LCD_BacklightBrightnessLevel.upperLayer_ptr					= &LCD_LCDSettings;
	LCD_BacklightBrightnessLevel.nextLayer_ptr					= &LCD_SecondsToTurnLCDBacklightOff;
	LCD_SecondsToTurnLCDBacklightOff.previousLayer_ptr			= &LCD_BacklightBrightnessLevel;
	LCD_SecondsToTurnLCDBacklightOff.upperLayer_ptr				= &LCD_LCDSettings;
	LCD_SecondsToTurnLCDBacklightOff.nextLayer_ptr				= &LCD_AutoBacklightOffStartHour;
	LCD_AutoBacklightOffStartHour.previousLayer_ptr				= &LCD_SecondsToTurnLCDBacklightOff;
	LCD_AutoBacklightOffStartHour.upperLayer_ptr				= &LCD_LCDSettings;
	LCD_AutoBacklightOffStartHour.nextLayer_ptr					= &LCD_AutoBacklightOffEndHour;
	LCD_AutoBacklightOffEndHour.previousLayer_ptr				= &LCD_AutoBacklightOffStartHour;
	LCD_AutoBacklightOffEndHour.upperLayer_ptr					= &LCD_LCDSettings;
	LCD_AutoBacklightOffEndHour.nextLayer_ptr					= &LCD_HomeScreenChoice;
	LCD_HomeScreenChoice.previousLayer_ptr						= &LCD_AutoBacklightOffEndHour;
	LCD_HomeScreenChoice.upperLayer_ptr							= &LCD_LCDSettings;
	LCD_HomeScreenChoice.nextLayer_ptr							= &LCD_AutoHomeReturnTime;
	LCD_AutoHomeReturnTime.previousLayer_ptr					= &LCD_HomeScreenChoice;
	LCD_AutoHomeReturnTime.upperLayer_ptr						= &LCD_LCDSettings;
	LCD_AutoHomeReturnTime.nextLayer_ptr						= &LCD_AutoLCDBacklightTurningOffOnOff;
	LCD_AutoLCDBacklightTurningOffOnOff.previousLayer_ptr		= &LCD_AutoHomeReturnTime;
	LCD_AutoLCDBacklightTurningOffOnOff.upperLayer_ptr			= &LCD_LCDSettings;
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
// @formatter:on

	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

	/* Setting " " in the whole buffer */
	memset(LCD_buffer, SPACE_IN_ASCII, (LCD.noOfRowsLCD * LCD.noOfColumnsLCD));

	/** LCD Init (setting number of rows, columns, address, I2C handler **/
	if(TRUE != lcdInit(&hi2c2, LCD.addressLCD, LCD.noOfRowsLCD, LCD.noOfColumnsLCD))
	{
		error = LCD__INIT_FAIL;
		my_error_handler(error);
	}

	// Print text at home position 0,0
	if(TRUE != lcdPrintStr((uint8_t*)"   Hi! My name is", 17u))
	{
		error = LCD__ERROR;
		my_error_handler(error);
	}

	// Set cursor at zero position of line 2
	if(TRUE != lcdSetCursorPosition(0, Row2))
	{
		error = LCD__ERROR;
		my_error_handler(error);
	}

	// Print text at home position 1,0
	if(TRUE != lcdPrintStr((uint8_t*)"       JARVIS", 13u))
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

	// Print text at cursor position 2,0
	if(TRUE != lcdPrintStr((uint8_t*)" !Welcome on board!", 19))
	{
		error = LCD__ERROR;
		my_error_handler(error);
	}

	/************/
	/* For the "hello" text to display for a few seconds */
	vTaskDelay(HELLO_MESSAGE_DISPLAY_TIME);
	/************/

	xLastWakeTime = xTaskGetTickCount();

	/* Infinite loop */
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
	for (;;)
	{
		/** Cleaning the buffer by writing only spaces into it **/
		memset(LCD_buffer, SPACE_IN_ASCII, (LCD.noOfRowsLCD * LCD.noOfColumnsLCD));

		/* Wait with proceeding if there was a "DONE" or "ERROR" message displayed.
		 * Flag EEPROM_Success_Failure_Message is set in EEPROMWaitForWriteCheck(...). */
		if(TRUE == EEPROM_Success_Failure_Message)
		{
			vTaskDelay((TickType_t)ERROR_DONE_DISPLAY_TIME);
			EEPROM_Success_Failure_Message = FALSE;
		}

		/* Calling a function to run on this LCD board (if one exists) */
		if(CurrentBoard_global->RunningFunction) CurrentBoard_global->RunningFunction(CurrentBoard_global);

		if(TRUE == ENC_button_menu.shortPressDetected)
		{
			if(CurrentBoard_global->EnterFunction)
				CurrentBoard_global->EnterFunction();	/* Execute Enter function pointed by current board (if one exists) */
			ENC_button_menu.shortPressDetected = FALSE;
			ENC_button_menu.longPressDetected = FALSE;
			last3snaps_doneOnce = FALSE;	/* Clean the doneOnce Flag for the RUNNING_Last3Snaps */
			scrollList_doneOnce = FALSE;	/* Clean the doneOnce Flag for the ScrollList Function */
		}

		if(TRUE == ENC_button_menu.longPressDetected)
		{
			if(CurrentBoard_global->upperLayer_ptr)
				CurrentBoard_global = CurrentBoard_global->upperLayer_ptr;	/* Go to the upper layer (if one exists) */
			ENC_button_menu.shortPressDetected = FALSE;
			ENC_button_menu.longPressDetected = FALSE;
			displayAndControlValue_doneOnce = FALSE;	/* Clean the doneOnce Flag for the RUNNING_DisplayAndControlValue */
			last3snaps_doneOnce = FALSE;	/* Clean the doneOnce Flag for the RUNNING_Last3Snaps */
			scrollList_doneOnce = FALSE;	/* Clean the doneOnce Flag for the ScrollList Function */
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
}




static void ENTER_GoInto(void)
{
	if(scrollList_currentlyPointedBoard) CurrentBoard_global = scrollList_currentlyPointedBoard;
}



static void ENTER_SaveToEEPROM(void)
{
	enterAction_save = TRUE;
}



static void ENTER_ClearDiagnosticSnapshots(void)
{
	LCD_AreYouSure.upperLayer_ptr = &LCD_ClearDiagSnaps;
	CurrentBoard_global = &LCD_AreYouSure;
}



static void ENTER_ClearErrorSnapshots(void)
{

	LCD_AreYouSure.upperLayer_ptr = &LCD_ClearErrorSnap;
	CurrentBoard_global = &LCD_AreYouSure;
}



static void ENTER_AreYouSure(void)
{
	uint8_t numberOfPacketsToSend = 0u;
	data32bit_union dataToSend[ENTER_AreYouSure_MAX_NO_OF_SENT_DATA_PACKETS] = {0};
	uint16_t dataSize[ENTER_AreYouSure_MAX_NO_OF_SENT_DATA_PACKETS] = {0};
	uint16_t dataAddress[ENTER_AreYouSure_MAX_NO_OF_SENT_DATA_PACKETS] = {0};

	static CREATE_EEPROM_data_struct(EEPROMData);
	EEPROMData.memAddressSize = EEPROM_PAGES_ADDRESS_SIZE;

	switch(LCD_AreYouSure.upperLayer_ptr->thisLayer)
	{
		case ClearDiagnosticSnapshots:
		{
			CAR_EEPROM_counters.diagnosticSnapshotEEPROMIndex = 0u;
			CAR_EEPROM_counters.didTheNumberOfDiagnosticSnapshotsOverflowed = FALSE;
			dataToSend[0].u8bit[0] = CAR_EEPROM_counters.diagnosticSnapshotEEPROMIndex;
			dataToSend[1].u8bit[0] = CAR_EEPROM_counters.didTheNumberOfDiagnosticSnapshotsOverflowed;

			EEPROMData.EEPROMParameters = &EEPROM_car;
			dataSize[0] = sizeof(CAR_EEPROM_counters.diagnosticSnapshotEEPROMIndex);
			dataSize[1] = sizeof(CAR_EEPROM_counters.didTheNumberOfDiagnosticSnapshotsOverflowed);
			dataAddress[0] = TOTAL_SNAPSHOTS_NUMBER_ADDRESS;
			dataAddress[1] = NUMBER_OF_DIAGNOSTIC_SNAPSHOTS_OVERFLOWED_ADDRESS;
			numberOfPacketsToSend = 2u;
			break;
		}
		case ClearErrorsSnapshots:
		{
			BOARD_EEPROM_counters.errorSnapshotEEPROMIndex = 0u;
			BOARD_EEPROM_counters.didTheNumberOfErrorSnapshotsOverflowed = FALSE;
			dataToSend[0].u8bit[0] = BOARD_EEPROM_counters.errorSnapshotEEPROMIndex;
			dataToSend[1].u8bit[0] = BOARD_EEPROM_counters.didTheNumberOfErrorSnapshotsOverflowed;

			EEPROMData.EEPROMParameters = &EEPROM_board;
			dataSize[0] = sizeof(BOARD_EEPROM_counters.errorSnapshotEEPROMIndex);
			dataSize[1] = sizeof(BOARD_EEPROM_counters.didTheNumberOfErrorSnapshotsOverflowed);
			dataAddress[0] = NUMBER_OF_ERROR_SNAPSHOTS;
			dataAddress[1] = NUMBER_OF_ERROR_SNAPSHOTS_OVERFLOWED_ADDRESS;
			numberOfPacketsToSend = 2u;
			break;
		}
		default:
		{
			break;
		}
	}//switch(LCD_AreYouSure.previousLayer_ptr->thisLayer)

	for(uint8_t i = 0; i < numberOfPacketsToSend; ++i)
	{
		EEPROMData.size = dataSize[i];
		EEPROMData.memAddress = dataAddress[i];
		EEPROMData.data = dataToSend[i].u8bit;

		xQueueSend(Queue_EEPROM_writeHandle, &EEPROMData, (TickType_t)100U/*100ms wait time if the queue is full*/);

		EEPROMWaitForWriteCheck(&EEPROMData);
	}

	CurrentBoard_global = LCD_AreYouSure.upperLayer_ptr;
}



static void RUNNING_ScrollList(struct LCD_board* currentBoard)
{
	Error_Code error = NO_ERROR;
	static LCD_board* displayBoards[NUMBER_OF_SCROLLED_LINES] = {NULL};


	error = copy_str_to_buffer(currentBoard->name, (char*)LCD_buffer[Row1], (uint8_t)((20-currentBoard->nameSize)/2), currentBoard->nameSize);
	error = copy_str_to_buffer(">", (char*)LCD_buffer[Row3], 0u, 1u);


	if(FALSE == scrollList_doneOnce)
	{
		for(uint8_t i = 0; i < NUMBER_OF_SCROLLED_LINES; ++i) {displayBoards[i] = NULL;}
		if(currentBoard->lowerLayer_ptr) displayBoards[LINE2] = currentBoard->lowerLayer_ptr;
		if(displayBoards[LINE2]->previousLayer_ptr) displayBoards[LINE1] = displayBoards[LINE2]->previousLayer_ptr;
		if(displayBoards[LINE2]->nextLayer_ptr) displayBoards[LINE3] = displayBoards[LINE2]->nextLayer_ptr;

		scrollList_doneOnce = TRUE;
	}

	if(0 < EncoderCounterMenuDiff)
	{
		ScrollForward(displayBoards, EncoderCounterMenuDiff);
		EncoderCounterMenuDiff = 0;
	}
	else
	{
		if(0 > EncoderCounterMenuDiff)
		{
			ScrollBack(displayBoards, EncoderCounterMenuDiff);
			EncoderCounterMenuDiff = 0;
		}
	}

	scrollList_currentlyPointedBoard = displayBoards[LINE2];

	if(displayBoards[LINE1])
		error = copy_str_to_buffer(displayBoards[LINE1]->name, (char*)LCD_buffer[Row2], 1u, displayBoards[LINE1]->nameSize);
	if(displayBoards[LINE2])
		error = copy_str_to_buffer(displayBoards[LINE2]->name, (char*)LCD_buffer[Row3], 1u, displayBoards[LINE2]->nameSize);
	if(displayBoards[LINE3])
		error = copy_str_to_buffer(displayBoards[LINE3]->name, (char*)LCD_buffer[Row4], 1u, displayBoards[LINE3]->nameSize);

	if(NO_ERROR != error) my_error_handler(error);
}



static void RUNNING_DesktopLayer(struct LCD_board* currentBoard)
{
	Error_Code error = NO_ERROR;

	/*** First Row ***/
		/* Main Battery Voltage */
	if(TRUE == mainBatteryVoltageValueForLCD.messageReadyFLAG)
		error = copy_str_to_buffer((char*)mainBatteryVoltageValueForLCD.messageHandler, (char*)LCD_buffer[Row1], 0, mainBatteryVoltageValueForLCD.size);
	error = copy_str_to_buffer("V", (char*)LCD_buffer[Row1], 5, 1);

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

	if(NO_ERROR != error) my_error_handler(error);

	/*** Second Row ***/
		/* Water temperature */
	error = copy_str_to_buffer("Water: ", (char*)LCD_buffer[Row2], 0, 7);
	if(TRUE == waterTemperatureValueForLCD.messageReadyFLAG)
		error = copy_str_to_buffer((char*)waterTemperatureValueForLCD.messageHandler, (char*)LCD_buffer[Row2], 7, waterTemperatureValueForLCD.size);
	error = copy_str_to_buffer((char*)degreeSymbolCharacter, (char*)LCD_buffer[Row2], 7+waterTemperatureValueForLCD.size, 1);
	error = copy_str_to_buffer("C", (char*)LCD_buffer[Row2], 7+waterTemperatureValueForLCD.size+1, 1);

	if(NO_ERROR != error) my_error_handler(error);

	/*** Third Row ***/
		/* Speed */
	if(TRUE == GPS.forLCD.speed.messageReadyFLAG)
		error = copy_str_to_buffer((char*)GPS.forLCD.speed.messageHandler, (char*)LCD_buffer[Row3], 0, GPS.forLCD.speed.size);
	error = copy_str_to_buffer("km/h", (char*)LCD_buffer[Row3], (GPS.forLCD.speed.size+1), 4);
		/* Total Mileage */
	if(TRUE == totalMileageForLCD.messageReadyFLAG)
		error = copy_str_to_buffer((char*)totalMileageForLCD.messageHandler, (char*)LCD_buffer[Row3], 9, totalMileageForLCD.size);
	error = copy_str_to_buffer("km", (char*)LCD_buffer[Row3], (9+totalMileageForLCD.size+1), 2);

	if(NO_ERROR != error) my_error_handler(error);

	/*** Fourth Row ***/
		/* Engine RPM */
	//TODO
	if(TRUE == RPMForLCD.messageReadyFLAG)
		error = copy_str_to_buffer((char*)RPMForLCD.messageHandler, (char*)LCD_buffer[Row4], 8, RPMForLCD.size);
	error = copy_str_to_buffer("rpm", (char*)LCD_buffer[Row4], 5, 3);
		/* Trip Mileage */
	if(TRUE == tripMileageForLCD.messageReadyFLAG)
		error = copy_str_to_buffer((char*)tripMileageForLCD.messageHandler, (char*)LCD_buffer[Row4], 9, tripMileageForLCD.size);
	error = copy_str_to_buffer("km", (char*)LCD_buffer[Row4], (9+tripMileageForLCD.size+1), 2);

	if(NO_ERROR != error) my_error_handler(error);
}



static void RUNNING_GPSLayer(struct LCD_board* currentBoard)
{
	Error_Code error = NO_ERROR;

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

	if(NO_ERROR != error) my_error_handler(error);

	/*** Second Row ***/
		/* GPS: Latitude */
	error = copy_str_to_buffer("Lat.:", (char*)LCD_buffer[Row2], 0, 5);
	if(TRUE == GPS.forLCD.latitude.messageReadyFLAG)
		error = copy_str_to_buffer((char*)GPS.forLCD.latitude.messageHandler, (char*)LCD_buffer[Row2], 6, GPS.forLCD.latitude.size);
		/* GPS: Latitude Indicator */
	if(TRUE == GPS.forLCD.latitudeIndicator.messageReadyFLAG)
		error = copy_str_to_buffer((char*)GPS.forLCD.latitudeIndicator.messageHandler, (char*)LCD_buffer[Row2], 6+GPS.forLCD.latitude.size+1, GPS.forLCD.latitudeIndicator.size);

	if(NO_ERROR != error) my_error_handler(error);

	/*** Third Row ***/
		/* GPS: Longitude */
	error = copy_str_to_buffer("Lon.:", (char*)LCD_buffer[Row3], 0, 5);
	if(TRUE == GPS.forLCD.longitude.messageReadyFLAG)
		error = copy_str_to_buffer((char*)GPS.forLCD.longitude.messageHandler, (char*)LCD_buffer[Row3], 6, GPS.forLCD.longitude.size);
		/* GPS: Longitude Indicator */
	error = copy_str_to_buffer((char*)GPS.forLCD.longitudeIndicator.messageHandler, (char*)LCD_buffer[Row3], 6+GPS.forLCD.longitude.size+1, GPS.forLCD.longitudeIndicator.size);

	if(NO_ERROR != error) my_error_handler(error);

	/*** Fourth Row ***/
		/* GPS: Altitude */
	error = copy_str_to_buffer("Alt.:", (char*)LCD_buffer[Row4], 0, 5);
	if(TRUE == GPS.forLCD.altitude.messageReadyFLAG)
		error = copy_str_to_buffer((char*)GPS.forLCD.altitude.messageHandler, (char*)LCD_buffer[Row4], 6, GPS.forLCD.altitude.size);
	error = copy_str_to_buffer("m npm", (char*)LCD_buffer[Row4], (6+GPS.forLCD.altitude.size+1), 5);

	if(NO_ERROR != error) my_error_handler(error);
}



static void RUNNING_CarInfoLayer(struct LCD_board* currentBoard)
{
	(void)copy_str_to_buffer("Nothing yet :)", (char*)LCD_buffer[Row2], 3u, 14u);
}



static void RUNNING_JarvisInfoLayer(struct LCD_board* currentBoard)
{
	Error_Code error = NO_ERROR;

	/*** First Row ***/
		/* Vin - input voltage */
	error = copy_str_to_buffer((char*)"Vin: ", (char*)LCD_buffer[Row1], 0u, 5u);
	if(TRUE == voltageInForLCD.messageReadyFLAG)
		error = copy_str_to_buffer((char*)voltageInForLCD.messageHandler, (char*)LCD_buffer[Row1], 5u, voltageInForLCD.size);
		/* clock */
	if((TRUE == GPS.forLCD.hours.messageReadyFLAG) && (TRUE == GPS.forLCD.minutes.messageReadyFLAG))
	{
		error = copy_str_to_buffer((char*)GPS.forLCD.hours.messageHandler, (char*)LCD_buffer[Row1], 15, GPS.forLCD.hours.size);
		error = copy_str_to_buffer(":", (char*)LCD_buffer[Row1], (15+GPS.forLCD.hours.size), 1);
		error = copy_str_to_buffer((char*)GPS.forLCD.minutes.messageHandler, (char*)LCD_buffer[Row1], (15+GPS.forLCD.hours.size+1), GPS.forLCD.minutes.size);
	}

	if(NO_ERROR != error) my_error_handler(error);

	/*** Second Row ***/
		/* 3V3 On Board Voltage and temperature */
	error = copy_str_to_buffer((char*)"3V3: ", (char*)LCD_buffer[Row2], 0u, 5u);
	if(TRUE == voltage3V3ForLCD.messageReadyFLAG)
		error = copy_str_to_buffer((char*)voltage3V3ForLCD.messageHandler, (char*)LCD_buffer[Row2], 5u, voltage3V3ForLCD.size);
	if(TRUE == temperature3V3DCDCForLCD.messageReadyFLAG)
		error = copy_str_to_buffer((char*)temperature3V3DCDCForLCD.messageHandler, (char*)LCD_buffer[Row2], (5u + voltage3V3ForLCD.size + 1u), temperature3V3DCDCForLCD.size);

	/*** Third Row ***/
		/* 5V On Board Voltage and temperature */
	error = copy_str_to_buffer((char*)"5V: ", (char*)LCD_buffer[Row3], 0u, 4u);
	if(TRUE == voltage5VForLCD.messageReadyFLAG)
		error = copy_str_to_buffer((char*)voltage5VForLCD.messageHandler, (char*)LCD_buffer[Row3], 5u, voltage5VForLCD.size);
	if(TRUE == temperature5VDCDCForLCD.messageReadyFLAG)
		error = copy_str_to_buffer((char*)temperature5VDCDCForLCD.messageHandler, (char*)LCD_buffer[Row3], (5u + voltage5VForLCD.size + 1u), temperature5VDCDCForLCD.size);

	if(NO_ERROR != error) my_error_handler(error);
}



static void RUNNING_Last3Snaps(struct LCD_board* currentBoard)
{
	Error_Code error = NO_ERROR;

	static char rowTempBuff[3u][20u];

	boolean isOverflowed = FALSE;
	uint16_t memoryAddressStart = 0u;
	uint8_t snapshotsCounter = 0u;
	uint8_t* clockTimeRead = NULL;

	DiagnosticDataToEEPROM_struct DiagnosticDataRead =
	{ .DiagnosticDataForEEPROM =
		{ .EEPROMParameters = &EEPROM_car,
		.data = DiagnosticDataRead.data,
		.size = MAX_DIAGNOSTIC_SNAPSHOT_SIZE,
		.memAddress = 0u,
		.memAddressSize = EEPROM_PAGES_ADDRESS_SIZE },
	.diag_mess_from_queue =
		{ .snapshotIdentificator = DIAGNOSTICS_OK,
		.value = 0 } };

	ErrorDataToEEPROM_struct ErrorDataRead =
	{ .ErrorDataForEEPROM =
		{ .EEPROMParameters = &EEPROM_board,
		.data = ErrorDataRead.data,
		.size = MAX_ERROR_SNAPSHOT_SIZE,
		.memAddress = 0u,
		.memAddressSize = EEPROM_PAGES_ADDRESS_SIZE	},
	.error_mess_from_queue = NO_ERROR };

	INITIALIZE_EEPROM_data_struct(DiagnosticDataRead.DiagnosticDataForEEPROM);
	INITIALIZE_EEPROM_data_struct(ErrorDataRead.ErrorDataForEEPROM);

	EEPROM_data_struct* localEEPROMData = NULL;

	error = copy_str_to_buffer(currentBoard->name, (char*)LCD_buffer[Row1], (uint8_t)((20-currentBoard->nameSize)/2), currentBoard->nameSize);

	if(NO_ERROR != error) my_error_handler(error);

	if(FALSE == last3snaps_doneOnce)
	{
		memset(rowTempBuff, SPACE_IN_ASCII, (3u * LCD.noOfColumnsLCD));
		switch(currentBoard->thisLayer)
		{
			case Last3Diag_Layer:
			{
				isOverflowed = CAR_EEPROM_counters.didTheNumberOfDiagnosticSnapshotsOverflowed;
				snapshotsCounter = CAR_EEPROM_counters.diagnosticSnapshotEEPROMIndex;
				memoryAddressStart = DIAGNOSTIC_SNAPSHOTS_START_ADDRESS;
				localEEPROMData = &(DiagnosticDataRead.DiagnosticDataForEEPROM);
				clockTimeRead = DiagnosticDataRead.clockTime;
				break;
			}
			case Last3Err_Layer:
			{
				isOverflowed = BOARD_EEPROM_counters.didTheNumberOfErrorSnapshotsOverflowed;
				snapshotsCounter = BOARD_EEPROM_counters.errorSnapshotEEPROMIndex;
				memoryAddressStart = ERROR_SNAPSHOTS_START_ADDRESS;
				localEEPROMData = &(ErrorDataRead.ErrorDataForEEPROM);
				clockTimeRead = ErrorDataRead.clockTime;
				break;
			}
			default:
			{
				break;

			}
		}//switch(currentBoard->thisLayer)


		if(0u != snapshotsCounter)
		{
			uint8_t row = 0u;
			for(uint8_t i = 0; (isOverflowed) ? (i < 3u) : (i < 3u && i <= snapshotsCounter); ++i)
			{
				localEEPROMData->isReady = DATA_NOT_READY;
				localEEPROMData->memAddress = memoryAddressStart + (uint16_t)((uint8_t)(CAR_EEPROM_counters.diagnosticSnapshotEEPROMIndex - i) * EEPROM_PAGE_SIZE);

				xQueueSend(Queue_EEPROM_readHandle, localEEPROMData, (TickType_t)1000U/*1000ms wait time if the queue is full*/);

				if(TRUE == localEEPROMData->isReady)
				{
					error = copy_str_to_buffer((char*)clockTimeRead, rowTempBuff[row], 0u, 8u);
				}
				++row;
			}
		}
		else
		{
			error = copy_str_to_buffer("No Snapshots So Far!", rowTempBuff[1u], 0u, 20u);
		}

		last3snaps_doneOnce = TRUE;
	}//if(FALSE == last3snaps_doneOnce)

	error = copy_str_to_buffer(rowTempBuff[0], (char*)LCD_buffer[Row2], 0u, 20u);
	error = copy_str_to_buffer(rowTempBuff[1], (char*)LCD_buffer[Row3], 0u, 20u);
	error = copy_str_to_buffer(rowTempBuff[2], (char*)LCD_buffer[Row4], 0u, 20u);

	if(NO_ERROR != error) my_error_handler(error);
}



static void RUNNING_DisplayAndControlValue(struct LCD_board* currentBoard)
{
	Error_Code error = NO_ERROR;
	char tempSettingBuffer[10u] = {' '};
	static float tempSetting = 0.0f;
	static boolean tempState = 0;
	static timeHours_type tempTime = 0;
	static Enum_Layer tempLayer;
	uint8_t tempSize = 0u;
	static CREATE_EEPROM_data_struct(EEPROMData);
	static data32bit_union dataToSend = {0};
	EEPROMData.memAddressSize = EEPROM_PAGES_ADDRESS_SIZE;
	EEPROMData.data = dataToSend.u8bit;

	error = copy_str_to_buffer(currentBoard->firstRow, (char*)LCD_buffer[Row1], (uint8_t)((20-currentBoard->firstRowSize)/2), currentBoard->firstRowSize);
	error = copy_str_to_buffer(currentBoard->secondRow, (char*)LCD_buffer[Row2], (uint8_t)((20-currentBoard->secondRowSize)/2), currentBoard->secondRowSize);

	if(NO_ERROR != error) my_error_handler(error);

	switch(currentBoard->valueType)
	{
		case _carTemperature_type_:
		case _carOilAnalogPressure_type_:
		case _cafFuelLevel_type_:
		case _carVoltage_type_:
		case _boardVoltage_type_:
		case _boardTemperature_type_:
		{
			if(FALSE == displayAndControlValue_doneOnce)
			{
				tempSetting = *((float*)(currentBoard->value_ptr));
				displayAndControlValue_doneOnce = TRUE;
			}

			switch(currentBoard->valueStepSize)
			{
				case StepByOne:
				{
					tempSetting += EncoderCounterMenuDiff;
					tempSize = snprintf(tempSettingBuffer, 10u, "%01d ", (uint16_t)tempSetting);
					break;
				}
				case StepByOneTen:
				{
					tempSetting += (float)EncoderCounterMenuDiff/10;
					tempSize = snprintf(tempSettingBuffer, 10u, "%01d.%01d ", (uint16_t)tempSetting, (uint16_t)(tempSetting*10)%10);
					break;
				}
				case StepByOneHundred:
				{
					tempSetting += (float)EncoderCounterMenuDiff/100;
					tempSize = snprintf(tempSettingBuffer, 10u, "%01d.%02d ", (uint16_t)tempSetting, (uint16_t)(tempSetting*100)%100);
					break;
				}
				default:
				{
					break;
				}
			}//switch(currentBoard->valueStepSize)

			if(*((float*)(currentBoard->minValue)) > tempSetting)
			{
				tempSetting = *((float*)(currentBoard->minValue));
			}
			else if(*((float*)(currentBoard->maxValue)) < tempSetting)
			{
				tempSetting = *((float*)(currentBoard->maxValue));
			}

			error = copy_str_to_buffer(tempSettingBuffer, (char*)LCD_buffer[Row3], (uint8_t)(20/2-tempSize), tempSize);
			error = copy_str_to_buffer(currentBoard->unit, (char*)LCD_buffer[Row3], (uint8_t)(20/2), currentBoard->unitSize);

			if(TRUE == enterAction_save)
			{
				*((float*)(currentBoard->value_ptr)) = tempSetting;
				dataToSend.f32bit = tempSetting;
				EEPROMData.size = FLOAT_SIZE;
			}

			break;
		}
		case _boolean_type_:
		case _carOilBinaryPressure_type_:
		{
			if(FALSE == displayAndControlValue_doneOnce)
			{
				tempState = *(uint8_t*)(currentBoard->value_ptr);
				displayAndControlValue_doneOnce = TRUE;
			}

			if(0 != EncoderCounterMenuDiff)
			{
				tempState ^= currentBoard->settingsMask;	//Toggling the bit with encoder
			}

			error = copy_str_to_buffer(((tempState&(currentBoard->settingsMask)) ? "ON " : "OFF"), (char*)LCD_buffer[Row3], (20u-3u)/2u, 3u);

			if(TRUE == enterAction_save)
			{
				*((boolean*)(currentBoard->value_ptr)) = tempState;
				dataToSend.bool8bit[0] = tempState;
				EEPROMData.size = UINT8_T_SIZE;
			}

			break;
		}
		case _timeHours_type_:
		{
			if(FALSE == displayAndControlValue_doneOnce)
			{
				tempTime = *((timeHours_type*)(currentBoard->value_ptr));
				displayAndControlValue_doneOnce = TRUE;
			}

			tempTime += EncoderCounterMenuDiff;
			tempSize = snprintf(tempSettingBuffer, 10u, "%01d ", tempTime);

			if(*((timeHours_type*)(currentBoard->minValue)) > tempTime)
			{
				tempTime = *((timeHours_type*)(currentBoard->minValue));
			}
			else if(*((timeHours_type*)(currentBoard->maxValue)) < tempTime)
			{
				tempTime = *((timeHours_type*)(currentBoard->maxValue));
			}

			error = copy_str_to_buffer(tempSettingBuffer, (char*)LCD_buffer[Row3], (uint8_t)(20/2-tempSize), tempSize);
			error = copy_str_to_buffer(currentBoard->unit, (char*)LCD_buffer[Row3], (uint8_t)(20/2), currentBoard->unitSize);

			if(TRUE == enterAction_save)
			{
				*((timeHours_type*)(currentBoard->value_ptr)) = tempTime;
				dataToSend.timeHours = tempTime;
				EEPROMData.size = INT8_T_SIZE;
			}

			break;
		}
		case _LCD_Enum_Layer_type_:
		{
			if(FALSE == displayAndControlValue_doneOnce)
			{
				tempLayer = *((Enum_Layer*)(currentBoard->value_ptr));
				displayAndControlValue_doneOnce = TRUE;
			}

			tempLayer += EncoderCounterMenuDiff;

			if(*((Enum_Layer*)(currentBoard->minValue)) > tempLayer)
			{
				tempLayer = *((Enum_Layer*)(currentBoard->minValue));
			}
			else if(*((Enum_Layer*)(currentBoard->maxValue)) < tempLayer)
			{
				tempLayer = *((Enum_Layer*)(currentBoard->maxValue));
			}

			error = copy_str_to_buffer(mainScreensList[(uint8_t)tempLayer]->name, (char*)LCD_buffer[Row3], (uint8_t)(20-mainScreensList[(uint8_t)tempLayer]->nameSize)/2, mainScreensList[(uint8_t)tempLayer]->nameSize);

			if(TRUE == enterAction_save)
			{
				*((Enum_Layer*)(currentBoard->value_ptr)) = tempLayer;
				HomeScreenBoard = (LCD_board*)mainScreensList[(uint8_t)tempLayer];
				dataToSend.screenLayer = tempLayer;
				EEPROMData.size = sizeof(tempLayer);
			}

			break;
		}
		case _void_type_:
		default:
		{
			break;
		}
	}//switch(currentBoard->valueType)

	EncoderCounterMenuDiff = 0;

	if(NO_ERROR != error) my_error_handler(error);

	if(TRUE == enterAction_save)
	{
		if(currentBoard->EEPROMParameters)
			EEPROMData.EEPROMParameters = currentBoard->EEPROMParameters;
		else
			error = EEPROM__PARAMETERS_POINTER_IS_NULL;
		if(currentBoard->EEPROM_memAddress)
			EEPROMData.memAddress = currentBoard->EEPROM_memAddress;
		else
			error = EEPROM__ADDRESS_POINTER_IS_NULL;

		if(NO_ERROR != error) my_error_handler(error);

		xQueueSend(Queue_EEPROM_writeHandle, &EEPROMData, (TickType_t)100U/*100ms wait time if the queue is full*/);

		EEPROMWaitForWriteCheck(&EEPROMData);

		enterAction_save = FALSE;
		displayAndControlValue_doneOnce = FALSE;
		if(currentBoard->upperLayer_ptr) {CurrentBoard_global = currentBoard->upperLayer_ptr;}
	}
}



static void RUNNING_AreYouSure(struct LCD_board* currentBoard)
{
	Error_Code error = NO_ERROR;

	error = copy_str_to_buffer(currentBoard->name, (char*)LCD_buffer[Row1], (uint8_t)((20-currentBoard->nameSize)/2), currentBoard->nameSize);
	error = copy_str_to_buffer("Enter to proceed", (char*)LCD_buffer[Row2], 0u, 16u);



	if(NO_ERROR != error) my_error_handler(error);
}



static void RUNNING_ClearSnaps(struct LCD_board* currentBoard)
{
	Error_Code error = NO_ERROR;
	char tempBuff[4u] = {' '};
	uint8_t counter = 0u;
	boolean overflowFlag = FALSE;


	error = copy_str_to_buffer(currentBoard->name, (char*)LCD_buffer[Row1], (uint8_t)((20-currentBoard->nameSize)/2), currentBoard->nameSize);
	error = copy_str_to_buffer(currentBoard->firstRow, (char*)LCD_buffer[Row2], 0u, currentBoard->firstRowSize);
	error = copy_str_to_buffer(currentBoard->secondRow, (char*)LCD_buffer[Row3], 0u, currentBoard->secondRowSize);

	switch(currentBoard->thisLayer)
	{
		case ClearDiagnosticSnapshots:
		{
			counter = CAR_EEPROM_counters.diagnosticSnapshotEEPROMIndex;
			overflowFlag = CAR_EEPROM_counters.didTheNumberOfDiagnosticSnapshotsOverflowed;
			break;
		}
		case ClearErrorsSnapshots:
		{
			counter = BOARD_EEPROM_counters.errorSnapshotEEPROMIndex;
			overflowFlag = BOARD_EEPROM_counters.didTheNumberOfErrorSnapshotsOverflowed;
			break;
		}
		default:
		{
			break;
		}
	}

	snprintf(tempBuff, 4u, "%3d", counter);

	error = copy_str_to_buffer(tempBuff, (char*)LCD_buffer[Row2], currentBoard->firstRowSize+1, 3u);
	error = copy_str_to_buffer(((TRUE == overflowFlag) ? "Yes" : "No "), (char*)LCD_buffer[Row3], currentBoard->secondRowSize+1u, 3u);
	error = copy_str_to_buffer("Click enter to clear", (char*)LCD_buffer[Row4], 0u, 20u);

	if(NO_ERROR != error) my_error_handler(error);
}



static void RUNNING_ClearTripMileage(struct LCD_board* currentBoard)
{
	Error_Code error = NO_ERROR;

	error = copy_str_to_buffer(currentBoard->name, (char*)LCD_buffer[Row1], (uint8_t)((20-currentBoard->nameSize)/2), currentBoard->nameSize);
	error = copy_str_to_buffer(currentBoard->firstRow, (char*)LCD_buffer[Row2], 0u, currentBoard->firstRowSize);

	if(TRUE == tripMileageForLCD.messageReadyFLAG)
		error = copy_str_to_buffer((char*)tripMileageForLCD.messageHandler, (char*)LCD_buffer[Row3], 0u, tripMileageForLCD.size);
	error = copy_str_to_buffer("km", (char*)LCD_buffer[Row3], (tripMileageForLCD.size+1u), 2u);

	error = copy_str_to_buffer("Click enter to clear", (char*)LCD_buffer[Row4], 0u, 20u);

	if(NO_ERROR != error) my_error_handler(error);
}



static void ScrollForward(LCD_board* displayTable[NUMBER_OF_SCROLLED_LINES], int8_t diff)
{
	LCD_board* tempBoardPtr = NULL;

	for(int8_t i = 0; i < diff; ++i)
	{
		if(displayTable[LINE3]) /* Check if line 3 even exists (there might be only 1 or 2 positions in this menu) */
		{
			tempBoardPtr = displayTable[LINE3]; /* save the 3rd line */

			displayTable[LINE1] = displayTable[LINE2]; /* Move second line to first line */
			displayTable[LINE2] = tempBoardPtr;	/* Move third line (from tempBoardPtr) to the second line */

			if(displayTable[LINE3]->nextLayer_ptr) /* Check if next line exists */
			{
				displayTable[LINE3] = displayTable[LINE3]->nextLayer_ptr; /* If exists - write next line to the 3rd one */
			}
			else
			{
				displayTable[LINE3] = NULL;
				break; /* If there is no next line then break the loop and exit */
			}
		}//if(displayTable[LINE3])
	}//for
}



static void ScrollBack(LCD_board* displayTable[NUMBER_OF_SCROLLED_LINES], int8_t diff)
{
	LCD_board* tempBoardPtr = NULL;

	for(int8_t i = 0; i < (-diff); ++i)
	{
		if(displayTable[LINE1]) /* Check if line 1 even exists (there might be only 1 position in this menu) */
		{
			tempBoardPtr = displayTable[LINE1]; /* save the 1st line */

			displayTable[LINE3] = displayTable[LINE2];	/* Move second line to the third line */
			displayTable[LINE2] = tempBoardPtr; /* move first line (from tempBoardPtr) to the second line */

			if(displayTable[LINE1]->previousLayer_ptr) /* Check if previous line exists */
			{
				displayTable[LINE1] = displayTable[LINE1]->previousLayer_ptr;	/* If exists - write previous line to the 1st one */
			}
			else
			{
				displayTable[LINE1] = NULL;
				break; /* If there is no previous line then break the loop and exit */
			}
		}//if(displayTable[LINE1])
	}//for
}



static void EEPROMWaitForWriteCheck(EEPROM_data_struct* EEPROMData)
{
	uint16_t i = 0u;
	boolean EEPROMerrorFlag = FALSE;

	while(DATA_READY != (*EEPROMData->isReadyPtr))
	{
		++i;
		vTaskDelay((TickType_t)1);	/* wait for 1 ms for EEPROM to process that */
		if(MAX_WAIT_TIME_FOR_EEPROM < i)
		{
			EEPROMerrorFlag = TRUE;
			break;
		}
	}

	if(TRUE == EEPROMerrorFlag)
	{
		EEPROMData->data = NULL;
		(void)copy_str_to_buffer("ERROR!", (char*)LCD_buffer[Row4], 7u, 6u);

		my_error_handler(EEPROM__FAILED_TO_WRITE_IN_LCD_TASK);
	}
	else
	{
		(void)copy_str_to_buffer("DONE!", (char*)LCD_buffer[Row4], 7u, 5u);
	}

	EEPROM_Success_Failure_Message = TRUE;
}




