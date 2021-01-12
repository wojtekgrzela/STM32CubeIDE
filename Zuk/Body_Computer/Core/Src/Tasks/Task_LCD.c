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

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Extern variables */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
extern I2C_HandleTypeDef hi2c2;
extern osMessageQId Queue_EEPROM_readHandle;
extern osMessageQId Queue_EEPROM_writeHandle;

extern Enum_Layer HOME_SCREEN;

extern EEPROM_parameters_struct EEPROM_car;
extern EEPROM_parameters_struct EEPROM_board;
extern CAR_EEPROM_counters_struct CAR_EEPROM_counters;
extern BOARD_EEPROM_counters_struct BOARD_EEPROM_counters;

extern CAR_mileage_struct CAR_mileage;

extern LCD_parameters_struct LCD;
extern GPS_data_struct GPS;

extern uint8_t tuBylemFLAG;		//TODO: to be deleted finally	(after MicroSd is working)
extern uint8_t TEMPBUFF[100];	//TODO: to be deleted finally (after MicroSd is working)

extern volatile ENCButton_struct ENC_button;
extern volatile int8_t EncoderCounterDiff;

extern LCD_message mainBatteryVoltageValueForLCD;
extern LCD_message auxiliaryBatteryVoltageValueForLCD;
extern LCD_message waterTemperatureValueForLCD;
extern LCD_message totalMileageForLCD;
extern LCD_message tripMileageForLCD;

extern waterTempSettings_struct CAR_waterTemp;
extern oilTempSettings_struct CAR_oilTemp;
extern oilPressureSettings_struct CAR_oilPressure;
extern fuelSettings_struct CAR_fuel;
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Local variables */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Buffer of Rows*Columns size for whole display (80 bytes for 4x20) */
uint8_t LCD_buffer[NO_OF_ROWS_IN_LCD][NO_OF_COLUMNS_IN_LCD];
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Structures for LCD (screens and layers) */
static LCDBoard LCD_MainMenu =	/* Cannot be constant - can be modified by user (.layerPrevious) */
		{ .name = "Main Menu",
		.nameActualSize = sizeof("Main Menu")-1,
		.layer = MainMenu_Layer,
		.actionForEnter = GoInto_EnterAction,
		.screenType = ScrollList_ScreenType,
		.layerPrevious = Desktop_Layer };

static LCDBoard LCD_YesNo =	/* Cannot be constant - can be modified by user (.layerPrevious) */
		{ .name = "Are you sure?",
		.nameActualSize = sizeof("Are you sure?")-1,
		.layer = YesNo_Layer,
		.actionForEnter = Done_EnterAction,
		.screenType = YesNo_ScreenType,
		.layerPrevious = MainMenu_Layer /* Should be last one opened */ };

static LCDBoard LCD_Ctrl =		/* Cannot be constant - not yet decided //TODO */
		{ .name = "Ctrl",
		.nameActualSize = sizeof("Ctrl")-1,
		.layer = Ctrl_Layer,
		.actionForEnter = Ctrl_EnterAction,
		.screenType = Ctrl_ScreenType,
		.layerPrevious = MainMenu_Layer /* Should be last one opened */ };

static LCDBoard LCD_Alarm =	/* Cannot be constant - not yet decided //TODO */
		{ .name = "!!! ALARM !!!",
		.nameActualSize = sizeof("!!! ALARM !!!")-1,
		.layer = Alarm_Layer,
		.actionForEnter = Alarm_EnterAction,
		.screenType = Alarm_ScreenType,
		.layerPrevious = MainMenu_Layer /* Should be last one opened */ };

/* Lists of sub-menus and sub-sub-menus etc. */
static const LCDBoard LCD_MainMenuList[ ] = {
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
static const uint8_t LCD_MainMenuList_SIZE = sizeof(LCD_MainMenuList)/sizeof(LCDBoard);

static const LCDBoard LCD_CarSettingsList[ ] = {
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
static const uint8_t LCD_CarSettingsList_SIZE = sizeof(LCD_CarSettingsList)/sizeof(LCDBoard);

static const LCDBoard LCD_BoardSettingsList[ ] = {
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
static const uint8_t LCD_BoardSettingsList_SIZE = sizeof(LCD_BoardSettingsList)/sizeof(LCDBoard);

static const LCDBoard LCD_Car_WaterTempSettingsList[ ] = {
			{ .name = "Water H. T. warn.",
			.nameActualSize = sizeof("Water H. T. warn.")-1,
			.layer = WaterHighTempWarningThreshold,
			.actionForEnter = Done_EnterAction,
			.screenType = No_ScreenType,
			.layerPrevious = WaterSettings_Layer },

			{ .name = "Water H. T. alarm",
			.nameActualSize = sizeof("Water H. T. alarm")-1,
			.layer = WaterHighTempAlarmThreshold,
			.actionForEnter = Done_EnterAction,
			.screenType = No_ScreenType,
			.layerPrevious = WaterSettings_Layer },

			{ .name = "Water H. T. FanOn",
			.nameActualSize = sizeof("Water H. T. FanOn")-1,
			.layer = WaterHighTempFanOnThreshold,
			.actionForEnter = Done_EnterAction,
			.screenType = No_ScreenType,
			.layerPrevious = WaterSettings_Layer },

			{ .name = "Water H. T. FanOff",
			.nameActualSize = sizeof("Water H. T. FanOff")-1,
			.layer = WaterHighTempFanOffThreshold,
			.actionForEnter = Done_EnterAction,
			.screenType = No_ScreenType,
			.layerPrevious = WaterSettings_Layer },

			{ .name = "Water T. warn.",
			.nameActualSize = sizeof("Water T. warn.")-1,
			.layer = WaterTempWarningOn,
			.actionForEnter = Done_EnterAction,
			.screenType = No_ScreenType,
			.layerPrevious = WaterSettings_Layer },

			{ .name = "Water T. alarm",
			.nameActualSize = sizeof("Water T. alarm")-1,
			.layer = WaterTempAlarmOn,
			.actionForEnter = Done_EnterAction,
			.screenType = No_ScreenType,
			.layerPrevious = WaterSettings_Layer },

			{ .name = "Water T. Fan ctrl",
			.nameActualSize = sizeof("Water T. Fan ctrl")-1,
			.layer = WaterFanControlOn,
			.actionForEnter = Done_EnterAction,
			.screenType = No_ScreenType,
			.layerPrevious = WaterSettings_Layer },

			{ .name = "Wat. T. warn. Buzz.",
			.nameActualSize = sizeof("Wat. T. warn. Buzz.")-1,
			.layer = WaterTempWarningBuzzerOn,
			.actionForEnter = Done_EnterAction,
			.screenType = No_ScreenType,
			.layerPrevious = WaterSettings_Layer },

			{ .name = "Wat. T. alarm Buzz.",
			.nameActualSize = sizeof("Wat. T. alarm Buzz.")-1,
			.layer = WaterTempAlarmBuzzerOn,
			.actionForEnter = Done_EnterAction,
			.screenType = No_ScreenType,
			.layerPrevious = WaterSettings_Layer },

			{ .name = "Wat. T. warn. snap.",
			.nameActualSize = sizeof("Wat. T. warn. snap.")-1,
			.layer = WaterTempWarningSnapshotOn,
			.actionForEnter = Done_EnterAction,
			.screenType = No_ScreenType,
			.layerPrevious = WaterSettings_Layer },

			{ .name = "Wat. T. alarm snap.",
			.nameActualSize = sizeof("Wat. T. alarm snap.")-1,
			.layer = WaterTempAlarmSnapshotOn,
			.actionForEnter = Done_EnterAction,
			.screenType = No_ScreenType,
			.layerPrevious = WaterSettings_Layer }
	};
static const uint8_t LCD_Car_WaterTempSettingsList_SIZE = sizeof(LCD_Car_WaterTempSettingsList)/sizeof(LCDBoard);

static const LCDBoard LCD_Car_OilTempSettingsList[ ] = {
			{ .name = "Oil H. T. warning",
			.nameActualSize = sizeof("Oil H. T. warning")-1,
			.layer = OilHighTempWarningThreshold,
			.actionForEnter = Ctrl_EnterAction,
			.screenType = No_ScreenType,
			.layerPrevious = OilTempSettings_Layer },

			{ .name = "Oil H. T. alarm",
			.nameActualSize = sizeof("Oil H. T. alarm")-1,
			.layer = OilHighTempAlarmThreshold,
			.actionForEnter = Ctrl_EnterAction,
			.screenType = No_ScreenType,
			.layerPrevious = OilTempSettings_Layer },

			{ .name = "Oil T. warning",
			.nameActualSize = sizeof("Oil T. warning")-1,
			.layer = OilTempWarningOn,
			.actionForEnter = OnOff_EnterAction,
			.screenType = No_ScreenType,
			.layerPrevious = OilTempSettings_Layer },

			{ .name = "Oil T. alarm",
			.nameActualSize = sizeof("Oil T. alarm")-1,
			.layer = OilTempAlarmOn,
			.actionForEnter = OnOff_EnterAction,
			.screenType = No_ScreenType,
			.layerPrevious = OilTempSettings_Layer },

			{ .name = "Oil T. warn. Buzz.",
			.nameActualSize = sizeof("Oil T. warn. Buzz.")-1,
			.layer = OilTempWarningBuzzerOn,
			.actionForEnter = OnOff_EnterAction,
			.screenType = No_ScreenType,
			.layerPrevious = OilTempSettings_Layer },

			{ .name = "Oil T. alarm Buzz.",
			.nameActualSize = sizeof("Oil T. alarm Buzz.")-1,
			.layer = OilTempAlarmBuzzerOn,
			.actionForEnter = OnOff_EnterAction,
			.screenType = No_ScreenType,
			.layerPrevious = OilTempSettings_Layer },

			{ .name = "Oil T. warn. snap.",
			.nameActualSize = sizeof("Oil T. warn. snap.")-1,
			.layer = OilTempWarningSnapshotOn,
			.actionForEnter = OnOff_EnterAction,
			.screenType = No_ScreenType,
			.layerPrevious = OilTempSettings_Layer },

			{ .name = "Oil T. alarm snap.",
			.nameActualSize = sizeof("Oil T. alarm snap.")-1,
			.layer = OilTempAlarmSpashotOn,
			.actionForEnter = OnOff_EnterAction,
			.screenType = No_ScreenType,
			.layerPrevious = OilTempSettings_Layer }
	};
static const uint8_t LCD_Car_OilTempSettingsList_SIZE = sizeof(LCD_Car_OilTempSettingsList)/sizeof(LCDBoard);

static const LCDBoard LCD_Car_OilPressureSettingsList[ ] = {
			{ .name = "Oil H. P. alarm",
			.nameActualSize = sizeof("Oil H. P. alarm")-1,
			.layer = OilHighPressureAlarmThreshold,
			.actionForEnter = Ctrl_EnterAction,
			.screenType = No_ScreenType,
			.layerPrevious = OilPressureSettings_Layer },

			{ .name = "Oil L. P. alarm",
			.nameActualSize = sizeof("Oil L. P. alarm")-1,
			.layer = OilLowPressureAlarmThreshold,
			.actionForEnter = Ctrl_EnterAction,
			.screenType = No_ScreenType,
			.layerPrevious = OilPressureSettings_Layer },

			{ .name = "Oil P. analog",
			.nameActualSize = sizeof("Oil P. analog")-1,
			.layer = OilPressureAnalogMeasurement,
			.actionForEnter = OnOff_EnterAction,
			.screenType = No_ScreenType,
			.layerPrevious = OilPressureSettings_Layer },

			{ .name = "Oil H. P. alarm",
			.nameActualSize = sizeof("Oil H. P. alarm")-1,
			.layer = OilHighPressureAlarmOn,
			.actionForEnter = OnOff_EnterAction,
			.screenType = No_ScreenType,
			.layerPrevious = OilPressureSettings_Layer },

			{ .name = "Oil L. P. alarm",
			.nameActualSize = sizeof("Oil L. P. alarm")-1,
			.layer = OilLowPressureAlarmOn,
			.actionForEnter = OnOff_EnterAction,
			.screenType = No_ScreenType,
			.layerPrevious = OilPressureSettings_Layer },

			{ .name = "Oil P. alarm Buzz.",
			.nameActualSize = sizeof("Oil P. alarm Buzz.")-1,
			.layer = OilPressureAlarmBuzzerOn,
			.actionForEnter = OnOff_EnterAction,
			.screenType = No_ScreenType,
			.layerPrevious = OilPressureSettings_Layer },

			{ .name = "Oil P. alarm snap.",
			.nameActualSize = sizeof("Oil P. alarm snap.")-1,
			.layer = OilPressureAlarmSnapshotOn,
			.actionForEnter = OnOff_EnterAction,
			.screenType = No_ScreenType,
			.layerPrevious = OilPressureSettings_Layer }
	};
static const uint8_t LCD_Car_OilPressureSettingsList_SIZE = sizeof(LCD_Car_OilPressureSettingsList)/sizeof(LCDBoard);

static const LCDBoard LCD_Car_FuelSettingsList[ ] = {
			{ .name = "Fuel Low warn. thr.",
			.nameActualSize = sizeof("Fuel Low warn. thr.")-1,
			.layer = FuelLowLevelWarningThreshold,
			.actionForEnter = Ctrl_EnterAction,
			.screenType = No_ScreenType,
			.layerPrevious = FuelSettings_Layer },

			{ .name = "Fuel Low warn. ON/OFF",
			.nameActualSize = sizeof("Fuel Low warn. ON/OFF")-1,
			.layer = FuelLowLevelWarningOn,
			.actionForEnter = OnOff_EnterAction,
			.screenType = No_ScreenType,
			.layerPrevious = FuelSettings_Layer },

			{ .name = "Fuel Low Buzzer",
			.nameActualSize = sizeof("Fuel Low Buzzer")-1,
			.layer = FuelLowLevelWarningBuzzerOn,
			.actionForEnter = OnOff_EnterAction,
			.screenType = No_ScreenType,
			.layerPrevious = FuelSettings_Layer }
	};
static const uint8_t LCD_Car_FuelSettingsList_SIZE = sizeof(LCD_Car_FuelSettingsList)/sizeof(LCDBoard);

static const LCDBoard LCD_Car_MainBatterySettingsList[ ] = {
			{ .name = "Low V. thres.",
			.nameActualSize = sizeof("Low V. thres.")-1,
			.layer = MainBatteryLowVoltageAlarmThreshold,
			.actionForEnter = Ctrl_EnterAction,
			.screenType = No_ScreenType,
			.layerPrevious = MainBatterySettings_Layer },

			{ .name = "High V. thres.",
			.nameActualSize = sizeof("High V. thres.")-1,
			.layer = MainBatteryHighVoltageAlarmThreshold,
			.actionForEnter = Ctrl_EnterAction,
			.screenType = No_ScreenType,
			.layerPrevious = MainBatterySettings_Layer },

			{ .name = "Low V. alarm",
			.nameActualSize = sizeof("Low V. alarm")-1,
			.layer = MainBatteryLowVoltageAlarmOn,
			.actionForEnter = OnOff_EnterAction,
			.screenType = No_ScreenType,
			.layerPrevious = MainBatterySettings_Layer },

			{ .name = "High V. alarm",
			.nameActualSize = sizeof("High V. alarm")-1,
			.layer = MainBatteryHighVoltageAlarmOn,
			.actionForEnter = OnOff_EnterAction,
			.screenType = No_ScreenType,
			.layerPrevious = MainBatterySettings_Layer },

			{ .name = "Volt. alarm Buzz.",
			.nameActualSize = sizeof("Volt. alarm Buzz.")-1,
			.layer = MainBatteryAlarmBuzzerOn,
			.actionForEnter = OnOff_EnterAction,
			.screenType = No_ScreenType,
			.layerPrevious = MainBatterySettings_Layer },

			{ .name = "L. V. alarm snap.",
			.nameActualSize = sizeof("L. V. alarm snap.")-1,
			.layer = MainBatteryLowVoltageSnapshotOn,
			.actionForEnter = OnOff_EnterAction,
			.screenType = No_ScreenType,
			.layerPrevious = MainBatterySettings_Layer },

			{ .name = "H. V. alarm snap.",
			.nameActualSize = sizeof("H. V. alarm snap.")-1,
			.layer = MainBatteryHighVoltageSnapshotOn,
			.actionForEnter = OnOff_EnterAction,
			.screenType = No_ScreenType,
			.layerPrevious = MainBatterySettings_Layer }
	};
static const uint8_t LCD_Car_MainBatterySettingsList_SIZE = sizeof(LCD_Car_MainBatterySettingsList)/sizeof(LCDBoard);

static const LCDBoard LCD_Car_AuxBatterySettingsList[ ] = {
			{ .name = "Low V. thres.",
			.nameActualSize = sizeof("Low V. thres.")-1,
			.layer = AuxBatteryLowVoltageAlarmThreshold,
			.actionForEnter = Ctrl_EnterAction,
			.screenType = No_ScreenType,
			.layerPrevious = AuxBatterySettings_Layer },

			{ .name = "High V. thres.",
			.nameActualSize = sizeof("High V. thres.")-1,
			.layer = AuxBatteryHighVoltageAlarmThreshold,
			.actionForEnter = Ctrl_EnterAction,
			.screenType = No_ScreenType,
			.layerPrevious = AuxBatterySettings_Layer },

			{ .name = "Low V. alarm",
			.nameActualSize = sizeof("Low V. alarm")-1,
			.layer = AuxBatteryLowVoltageAlarmOn,
			.actionForEnter = OnOff_EnterAction,
			.screenType = No_ScreenType,
			.layerPrevious = AuxBatterySettings_Layer },

			{ .name = "High V. alarm",
			.nameActualSize = sizeof("High V. alarm")-1,
			.layer = AuxBatteryHighVoltageAlarmOn,
			.actionForEnter = OnOff_EnterAction,
			.screenType = No_ScreenType,
			.layerPrevious = AuxBatterySettings_Layer },

			{ .name = "Volt. alarm Buzz.",
			.nameActualSize = sizeof("Volt. alarm Buzz.")-1,
			.layer = AuxBatteryAlarmBuzzerOn,
			.actionForEnter = OnOff_EnterAction,
			.screenType = No_ScreenType,
			.layerPrevious = AuxBatterySettings_Layer },

			{ .name = "L. V. alarm snap.",
			.nameActualSize = sizeof("L. V. alarm snap.")-1,
			.layer = AuxBatteryLowVoltageSnapshotOn,
			.actionForEnter = OnOff_EnterAction,
			.screenType = No_ScreenType,
			.layerPrevious = AuxBatterySettings_Layer },

			{ .name = "H. V. alarm snap.",
			.nameActualSize = sizeof("H. V. alarm snap.")-1,
			.layer = AuxBatteryHighVoltageSnapshotOn,
			.actionForEnter = OnOff_EnterAction,
			.screenType = No_ScreenType,
			.layerPrevious = AuxBatterySettings_Layer }
	};
static const uint8_t LCD_Car_AuxBatterySettingsList_SIZE = sizeof(LCD_Car_AuxBatterySettingsList)/sizeof(LCDBoard);

static const LCDBoard LCD_Board_InternalVoltSettingsList[ ] = {
			{ .name = "5V Low thres.",
			.nameActualSize = sizeof("5V Low thres.")-1,
			.layer = Supply5VLowThreshold,
			.actionForEnter = Ctrl_EnterAction,
			.screenType = No_ScreenType,
			.layerPrevious = InterVoltSettings_Layer },

			{ .name = "5V High thres.",
			.nameActualSize = sizeof("5V High thres.")-1,
			.layer = Supply5VHighThreshold,
			.actionForEnter = Ctrl_EnterAction,
			.screenType = No_ScreenType,
			.layerPrevious = InterVoltSettings_Layer },

			{ .name = "3V3 Low thres.",
			.nameActualSize = sizeof("3V3 Low thres.")-1,
			.layer = Supply3V3LowThreshold,
			.actionForEnter = Ctrl_EnterAction,
			.screenType = No_ScreenType,
			.layerPrevious = InterVoltSettings_Layer },

			{ .name = "3V3 High thres.",
			.nameActualSize = sizeof("3V3 High thres.")-1,
			.layer = Supply3V3HighThreshold,
			.actionForEnter = Ctrl_EnterAction,
			.screenType = No_ScreenType,
			.layerPrevious = InterVoltSettings_Layer },

			{ .name = "VIn Low thres.",
			.nameActualSize = sizeof("VIn Low thres.")-1,
			.layer = SupplyVinLowThreshold,
			.actionForEnter = Ctrl_EnterAction,
			.screenType = No_ScreenType,
			.layerPrevious = InterVoltSettings_Layer }
	};
static const uint8_t LCD_Board_InternalVoltSettingsList_SIZE = sizeof(LCD_Board_InternalVoltSettingsList)/sizeof(LCDBoard);

static const LCDBoard LCD_Board_InternalTempSettingsList[ ] = {
			{ .name = "3V3 DCDC T. thres.",
			.nameActualSize = sizeof("3V3 DCDC T. thres.")-1,
			.layer = DCDC3V3HighTempThreshold,
			.actionForEnter = Ctrl_EnterAction,
			.screenType = No_ScreenType,
			.layerPrevious = InterTempSettings_Layer },

			{ .name = "5V DCDC T. thres.",
			.nameActualSize = sizeof("5V DCDC T. thres.")-1,
			.layer = DCDC5VHighTempThreshold,
			.actionForEnter = Ctrl_EnterAction,
			.screenType = No_ScreenType,
			.layerPrevious = InterTempSettings_Layer }
	};
static const uint8_t LCD_Board_InternalTempSettingsList_SIZE = sizeof(LCD_Board_InternalTempSettingsList)/sizeof(LCDBoard);

static const LCDBoard LCD_Board_BuzzerSettingsList[ ] = {
			{ .name = "Buzzer Main",
			.nameActualSize = sizeof("Buzzer Main")-1,
			.layer = BuzzerMainSwitch,
			.actionForEnter = OnOff_EnterAction,
			.screenType = No_ScreenType,
			.layerPrevious = BuzzerSettings_Layer },

			{ .name = "Buzzer Main alarm",
			.nameActualSize = sizeof("Buzzer Main alarm")-1,
			.layer = BuzzerMainAlarmsSwitch,
			.actionForEnter = OnOff_EnterAction,
			.screenType = No_ScreenType,
			.layerPrevious = BuzzerSettings_Layer },

			{ .name = "Buzzer Main button",
			.nameActualSize = sizeof("Buzzer Main button")-1,
			.layer = BuzzerMainButtonsSwitch,
			.actionForEnter = OnOff_EnterAction,
			.screenType = No_ScreenType,
			.layerPrevious = BuzzerSettings_Layer },

			{ .name = "Buzzer Short press",
			.nameActualSize = sizeof("Buzzer Short press")-1,
			.layer = BuzzerWhenShortPress,
			.actionForEnter = OnOff_EnterAction,
			.screenType = No_ScreenType,
			.layerPrevious = BuzzerSettings_Layer },

			{ .name = "Buzzer Long press",
			.nameActualSize = sizeof("Buzzer Long press")-1,
			.layer = BuzzerWhenLongPress,
			.actionForEnter = OnOff_EnterAction,
			.screenType = No_ScreenType,
			.layerPrevious = BuzzerSettings_Layer }
	};
static const uint8_t LCD_Board_BuzzerSettingsList_SIZE = sizeof(LCD_Board_BuzzerSettingsList)/sizeof(LCDBoard);

static const LCDBoard LCD_Board_LCDSettingsList[ ] = {
			{ .name = "Backlight level",
			.nameActualSize = sizeof("Backlight level")-1,
			.layer = BacklightBrightnessLevel,
			.actionForEnter = Ctrl_EnterAction,
			.screenType = No_ScreenType,
			.layerPrevious = LCDSettings_Layer },

			{ .name = "Sec to off light",
			.nameActualSize = sizeof("Sec to off light")-1,
			.layer = SecondsToTurnOffBacklight,
			.actionForEnter = Ctrl_EnterAction,
			.screenType = No_ScreenType,
			.layerPrevious = LCDSettings_Layer },

			{ .name = "Auto off from",
			.nameActualSize = sizeof("Auto off from")-1,
			.layer = AutoBacklightOffStartHour,
			.actionForEnter = Ctrl_EnterAction,
			.screenType = No_ScreenType,
			.layerPrevious = LCDSettings_Layer },

			{ .name = "Auto off to",
			.nameActualSize = sizeof("Auto off to")-1,
			.layer = AutoBacklightOffEndHour,
			.actionForEnter = Ctrl_EnterAction,
			.screenType = No_ScreenType,
			.layerPrevious = LCDSettings_Layer },

			{ .name = "Home screen",
			.nameActualSize = sizeof("Home screen")-1,
			.layer = HomeScreen,
			.actionForEnter = Ctrl_EnterAction,
			.screenType = No_ScreenType,
			.layerPrevious = LCDSettings_Layer },

			{ .name = "Auto home return",
			.nameActualSize = sizeof("Auto home return")-1,
			.layer = AutoHomeReturnTime,
			.actionForEnter = Ctrl_EnterAction,
			.screenType = No_ScreenType,
			.layerPrevious = LCDSettings_Layer },

			{ .name = "Auto light off",
			.nameActualSize = sizeof("Auto light off")-1,
			.layer = AutoBacklightOff,
			.actionForEnter = OnOff_EnterAction,
			.screenType = No_ScreenType,
			.layerPrevious = LCDSettings_Layer },
	};
static const uint8_t LCD_Board_LCDSettingsList_SIZE = sizeof(LCD_Board_LCDSettingsList)/sizeof(LCDBoard);
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */



static Error_Code ControlValueWithEncoder(const LCDBoard* const currentBoard, boolean* displayErrorFlag, boolean* displayDoneFlag, boolean* doneOnce);



void StartTaskLCD(void const * argument)
{
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = MY_LCD_TASK_TIME_PERIOD;
	Error_Code error = NO_ERROR;

	/* No better option for making a degree symbol was found so far */
	uint8_t degreeSymbolCharacter[2] = "";
	snprintf((char*)degreeSymbolCharacter, 2, "%c", DEGREE_SYMBOL_LCD);
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

	int32_t submenuIterator = 0;

	Enum_Layer currentLayer = HOME_SCREEN;

	boolean displayErrorFlag = FALSE;
	boolean displayDoneFlag = FALSE;

	/******************************************************/
	/* For reading the data from EEPROM and displaying it */
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
	/******************************************************/

	/* Home screen applied - can be changed in settings from LCD */
	LCD_MainMenu.layerPrevious = HOME_SCREEN;

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
	/* For the "hello" text to display for 3 seconds */
	vTaskDelay(HELLO_MESSAGE_DISPLAY_TIME);
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
							*(DiagnosticDataRead.DiagnosticDataForEEPROM.isReadyPtr) = DATA_NOT_READY;
							DiagnosticDataRead.DiagnosticDataForEEPROM.memAddress = DIAGNOSTIC_SNAPSHOTS_START_ADDRESS + (uint16_t)((uint8_t)(CAR_EEPROM_counters.diagnosticSnapshotEEPROMIndex - i)*EEPROM_PAGE_SIZE);
							xQueueSend(Queue_EEPROM_readHandle, &(DiagnosticDataRead.DiagnosticDataForEEPROM), (TickType_t)100U/*100ms wait time if the queue is full*/);

							vTaskDelay((TickType_t)MAX_WAIT_TIME_FOR_EEPROM);	/* The wait time for the response from EEPROM */

							if(DATA_READY == *(DiagnosticDataRead.DiagnosticDataForEEPROM.isReadyPtr))
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
								*(DiagnosticDataRead.DiagnosticDataForEEPROM.isReadyPtr) = DATA_NOT_READY;
								DiagnosticDataRead.DiagnosticDataForEEPROM.memAddress = DIAGNOSTIC_SNAPSHOTS_START_ADDRESS + (uint16_t)((uint8_t)(CAR_EEPROM_counters.diagnosticSnapshotEEPROMIndex - i)*EEPROM_PAGE_SIZE);
								xQueueSend(Queue_EEPROM_readHandle, &(DiagnosticDataRead.DiagnosticDataForEEPROM), (TickType_t)100U/*100ms wait time if the queue is full*/);

								vTaskDelay((TickType_t)MAX_WAIT_TIME_FOR_EEPROM);	/* The wait time for the response from EEPROM */

								if(DATA_READY == *(DiagnosticDataRead.DiagnosticDataForEEPROM.isReadyPtr))
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
							*(ErrorDataRead.ErrorDataForEEPROM.isReadyPtr) = DATA_NOT_READY;
							ErrorDataRead.ErrorDataForEEPROM.memAddress = ERROR_SNAPSHOTS_START_ADDRESS + (uint16_t)((uint8_t)(BOARD_EEPROM_counters.errorSnapshotEEPROMIndex - i)*EEPROM_PAGE_SIZE);
							xQueueSend(Queue_EEPROM_readHandle, &(ErrorDataRead.ErrorDataForEEPROM), (TickType_t)100U/*100ms wait time if the queue is full*/);

							vTaskDelay((TickType_t)MAX_WAIT_TIME_FOR_EEPROM);	/* The wait time for the response from EEPROM */

							if(DATA_READY == *(ErrorDataRead.ErrorDataForEEPROM.isReadyPtr))
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
								*(ErrorDataRead.ErrorDataForEEPROM.isReadyPtr) = DATA_NOT_READY;
								ErrorDataRead.ErrorDataForEEPROM.memAddress = ERROR_SNAPSHOTS_START_ADDRESS + (uint16_t)((uint8_t)(BOARD_EEPROM_counters.errorSnapshotEEPROMIndex - i)*EEPROM_PAGE_SIZE);
								xQueueSend(Queue_EEPROM_readHandle, &(ErrorDataRead.ErrorDataForEEPROM), (TickType_t)100U/*100ms wait time if the queue is full*/);

								vTaskDelay((TickType_t)MAX_WAIT_TIME_FOR_EEPROM);	/* The wait time for the response from EEPROM */

								if(DATA_READY == *(ErrorDataRead.ErrorDataForEEPROM.isReadyPtr))
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

			/* *** *** *** *** *** *** *** *** *** *** *** *** *** *** *//* *** *** *** *** *** *** *** *** *** *** *** *** *** *** */
			/* Car settings list : */
			/* *** *** *** *** *** *** *** *** *** *** *** *** *** *** *//* *** *** *** *** *** *** *** *** *** *** *** *** *** *** */

			case /* CarSettings_Layer -> */ClearDiagnosticSnapshots:
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
			case /* CarSettings_Layer -> */ClearTripMileage:
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
			case /* CarSettings_Layer -> */WaterSettings_Layer:
			{
				error = copy_str_to_buffer("7.3.", (char*)LCD_buffer[Row1], 0, 4);
				error = copy_str_to_buffer(LCD_CarSettingsList[2].name, (char*)LCD_buffer[Row1], 4, LCD_CarSettingsList[2].nameActualSize);

				scroll_list(LCD_Car_WaterTempSettingsList, LCD_Car_WaterTempSettingsList_SIZE, &(LCD_CarSettingsList[2]), &(LCD_buffer[Row1]), (int8_t*)&EncoderCounterDiff, &submenuIterator);

				if(ENC_button.shortPressDetected)
				{
					error = shortButtonPressDetected_LCD(&(LCD_CarSettingsList[2]), LCD_Car_WaterTempSettingsList, &currentLayer, &submenuIterator);
				}

				if(ENC_button.longPressDetected)
				{
					error = longButtonPressDetected_LCD(&LCD_CarSettingsList[2], &currentLayer, &submenuIterator);
				}
				break;
			}

			/* *** *** *** *** *** *** *** *** *** *** *** *** *** *** *//* *** *** *** *** *** *** *** *** *** *** *** *** *** *** */
			/* CarSettings_Layer -> WaterSettings_Layer: */
			/* *** *** *** *** *** *** *** *** *** *** *** *** *** *** *//* *** *** *** *** *** *** *** *** *** *** *** *** *** *** */

			case /* CarSettings_Layer -> WaterSettings_Layer -> */WaterHighTempWarningThreshold:
			{
				static boolean doneOnce = FALSE;
				error = ControlValueWithEncoder(&(LCD_Car_WaterTempSettingsList[0]), &displayErrorFlag, &displayDoneFlag, &doneOnce);

				if(ENC_button.shortPressDetected)
				{
					doneOnce = FALSE;
					error = shortButtonPressDetected_LCD(&(LCD_Car_WaterTempSettingsList[0]), LCD_Car_WaterTempSettingsList, &currentLayer, &submenuIterator);
				}

				if(ENC_button.longPressDetected)
				{
					doneOnce = FALSE;
					error = longButtonPressDetected_LCD(&LCD_Car_WaterTempSettingsList[0], &currentLayer, &submenuIterator);
				}
				break;
			}
			case /* CarSettings_Layer -> WaterSettings_Layer -> */WaterHighTempAlarmThreshold:
			{
				static boolean doneOnce = FALSE;
				error = ControlValueWithEncoder(&(LCD_Car_WaterTempSettingsList[1]), &displayErrorFlag, &displayDoneFlag, &doneOnce);

				if(ENC_button.shortPressDetected)
				{
					doneOnce = FALSE;
					error = shortButtonPressDetected_LCD(&(LCD_Car_WaterTempSettingsList[1]), LCD_Car_WaterTempSettingsList, &currentLayer, &submenuIterator);
				}

				if(ENC_button.longPressDetected)
				{
					doneOnce = FALSE;
					error = longButtonPressDetected_LCD(&LCD_Car_WaterTempSettingsList[1], &currentLayer, &submenuIterator);
				}
				break;
			}
			case /* CarSettings_Layer -> WaterSettings_Layer -> */WaterHighTempFanOnThreshold:
			{
				static boolean doneOnce = FALSE;
				error = ControlValueWithEncoder(&(LCD_Car_WaterTempSettingsList[2]), &displayErrorFlag, &displayDoneFlag, &doneOnce);

				if(ENC_button.shortPressDetected)
				{
					doneOnce = FALSE;
					error = shortButtonPressDetected_LCD(&(LCD_Car_WaterTempSettingsList[2]), LCD_Car_WaterTempSettingsList, &currentLayer, &submenuIterator);
				}

				if(ENC_button.longPressDetected)
				{
					doneOnce = FALSE;
					error = longButtonPressDetected_LCD(&LCD_Car_WaterTempSettingsList[2], &currentLayer, &submenuIterator);
				}
				break;
			}
			case /* CarSettings_Layer -> WaterSettings_Layer -> */WaterHighTempFanOffThreshold:
			{
				static boolean doneOnce = FALSE;
				error = ControlValueWithEncoder(&(LCD_Car_WaterTempSettingsList[3]), &displayErrorFlag, &displayDoneFlag, &doneOnce);

				if(ENC_button.shortPressDetected)
				{
					doneOnce = FALSE;
					error = shortButtonPressDetected_LCD(&(LCD_Car_WaterTempSettingsList[3]), LCD_Car_WaterTempSettingsList, &currentLayer, &submenuIterator);
				}

				if(ENC_button.longPressDetected)
				{
					doneOnce = FALSE;
					error = longButtonPressDetected_LCD(&LCD_Car_WaterTempSettingsList[3], &currentLayer, &submenuIterator);
				}
				break;
			}
			case /* CarSettings_Layer -> WaterSettings_Layer -> */WaterTempWarningOn:
			{
				static boolean doneOnce = FALSE;
				error = ControlValueWithEncoder(&(LCD_Car_WaterTempSettingsList[4]), &displayErrorFlag, &displayDoneFlag, &doneOnce);

				if(ENC_button.shortPressDetected)
				{
					doneOnce = FALSE;
					error = shortButtonPressDetected_LCD(&(LCD_Car_WaterTempSettingsList[4]), LCD_Car_WaterTempSettingsList, &currentLayer, &submenuIterator);
				}

				if(ENC_button.longPressDetected)
				{
					doneOnce = FALSE;
					error = longButtonPressDetected_LCD(&LCD_Car_WaterTempSettingsList[4], &currentLayer, &submenuIterator);
				}
				break;
			}
			case /* CarSettings_Layer -> WaterSettings_Layer -> */WaterTempAlarmOn:
			{
				static boolean doneOnce = FALSE;
				error = ControlValueWithEncoder(&(LCD_Car_WaterTempSettingsList[5]), &displayErrorFlag, &displayDoneFlag, &doneOnce);

				if(ENC_button.shortPressDetected)
				{
					doneOnce = FALSE;
					error = shortButtonPressDetected_LCD(&(LCD_Car_WaterTempSettingsList[5]), LCD_Car_WaterTempSettingsList, &currentLayer, &submenuIterator);
				}

				if(ENC_button.longPressDetected)
				{
					doneOnce = FALSE;
					error = longButtonPressDetected_LCD(&LCD_Car_WaterTempSettingsList[5], &currentLayer, &submenuIterator);
				}
				break;
			}
			case /* CarSettings_Layer -> WaterSettings_Layer -> */WaterFanControlOn:
			{
				static boolean doneOnce = FALSE;
				error = ControlValueWithEncoder(&(LCD_Car_WaterTempSettingsList[6]), &displayErrorFlag, &displayDoneFlag, &doneOnce);

				if(ENC_button.shortPressDetected)
				{
					doneOnce = FALSE;
					error = shortButtonPressDetected_LCD(&(LCD_Car_WaterTempSettingsList[6]), LCD_Car_WaterTempSettingsList, &currentLayer, &submenuIterator);
				}

				if(ENC_button.longPressDetected)
				{
					doneOnce = FALSE;
					error = longButtonPressDetected_LCD(&LCD_Car_WaterTempSettingsList[6], &currentLayer, &submenuIterator);
				}
				break;
			}
			case /* CarSettings_Layer -> WaterSettings_Layer -> */WaterTempWarningBuzzerOn:
			{
				static boolean doneOnce = FALSE;
				error = ControlValueWithEncoder(&(LCD_Car_WaterTempSettingsList[7]), &displayErrorFlag, &displayDoneFlag, &doneOnce);

				if(ENC_button.shortPressDetected)
				{
					doneOnce = FALSE;
					error = shortButtonPressDetected_LCD(&(LCD_Car_WaterTempSettingsList[7]), LCD_Car_WaterTempSettingsList, &currentLayer, &submenuIterator);
				}

				if(ENC_button.longPressDetected)
				{
					doneOnce = FALSE;
					error = longButtonPressDetected_LCD(&LCD_Car_WaterTempSettingsList[7], &currentLayer, &submenuIterator);
				}
				break;
			}
			case /* CarSettings_Layer -> WaterSettings_Layer -> */WaterTempAlarmBuzzerOn:
			{
				static boolean doneOnce = FALSE;
				error = ControlValueWithEncoder(&(LCD_Car_WaterTempSettingsList[8]), &displayErrorFlag, &displayDoneFlag, &doneOnce);

				if(ENC_button.shortPressDetected)
				{
					doneOnce = FALSE;
					error = shortButtonPressDetected_LCD(&(LCD_Car_WaterTempSettingsList[8]), LCD_Car_WaterTempSettingsList, &currentLayer, &submenuIterator);
				}

				if(ENC_button.longPressDetected)
				{
					doneOnce = FALSE;
					error = longButtonPressDetected_LCD(&LCD_Car_WaterTempSettingsList[8], &currentLayer, &submenuIterator);
				}
				break;
			}
			case /* CarSettings_Layer -> WaterSettings_Layer -> */WaterTempWarningSnapshotOn:
			{
				static boolean doneOnce = FALSE;
				error = ControlValueWithEncoder(&(LCD_Car_WaterTempSettingsList[9]), &displayErrorFlag, &displayDoneFlag, &doneOnce);

				if(ENC_button.shortPressDetected)
				{
					doneOnce = FALSE;
					error = shortButtonPressDetected_LCD(&(LCD_Car_WaterTempSettingsList[9]), LCD_Car_WaterTempSettingsList, &currentLayer, &submenuIterator);
				}

				if(ENC_button.longPressDetected)
				{
					doneOnce = FALSE;
					error = longButtonPressDetected_LCD(&LCD_Car_WaterTempSettingsList[9], &currentLayer, &submenuIterator);
				}
				break;
			}
			case /* CarSettings_Layer -> WaterSettings_Layer -> */WaterTempAlarmSnapshotOn:
			{
				static boolean doneOnce = FALSE;
				error = ControlValueWithEncoder(&(LCD_Car_WaterTempSettingsList[10]), &displayErrorFlag, &displayDoneFlag, &doneOnce);

				if(ENC_button.shortPressDetected)
				{
					doneOnce = FALSE;
					error = shortButtonPressDetected_LCD(&(LCD_Car_WaterTempSettingsList[10]), LCD_Car_WaterTempSettingsList, &currentLayer, &submenuIterator);
				}

				if(ENC_button.longPressDetected)
				{
					doneOnce = FALSE;
					error = longButtonPressDetected_LCD(&LCD_Car_WaterTempSettingsList[10], &currentLayer, &submenuIterator);
				}
				break;
			}
			/* *** *** *** *** *** *** *** *** *** *** *** *** *** *** *//* *** *** *** *** *** *** *** *** *** *** *** *** *** *** */


			case /* CarSettings_Layer -> */OilTempSettings_Layer:
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

			/* *** *** *** *** *** *** *** *** *** *** *** *** *** *** *//* *** *** *** *** *** *** *** *** *** *** *** *** *** *** */
			/* CarSettings_Layer -> OilTempSettings_Layer: */
			/* *** *** *** *** *** *** *** *** *** *** *** *** *** *** *//* *** *** *** *** *** *** *** *** *** *** *** *** *** *** */

			case /* CarSettings_Layer -> OilTempSettings_Layer -> */OilHighTempWarningThreshold:
			{
				static boolean doneOnce = FALSE;
				error = ControlValueWithEncoder(&(LCD_Car_OilTempSettingsList[0]), &displayErrorFlag, &displayDoneFlag, &doneOnce);

				if(ENC_button.shortPressDetected)
				{
					doneOnce = FALSE;
					error = shortButtonPressDetected_LCD(&(LCD_Car_OilTempSettingsList[0]), LCD_Car_OilTempSettingsList, &currentLayer, &submenuIterator);
				}

				if(ENC_button.longPressDetected)
				{
					doneOnce = FALSE;
					error = longButtonPressDetected_LCD(&LCD_Car_OilTempSettingsList[0], &currentLayer, &submenuIterator);
				}
				break;
			}
			case /* CarSettings_Layer -> OilTempSettings_Layer -> */OilHighTempAlarmThreshold:
			{
				static boolean doneOnce = FALSE;
				error = ControlValueWithEncoder(&(LCD_Car_OilTempSettingsList[1]), &displayErrorFlag, &displayDoneFlag, &doneOnce);

				if(ENC_button.shortPressDetected)
				{
					doneOnce = FALSE;
					error = shortButtonPressDetected_LCD(&(LCD_Car_OilTempSettingsList[1]), LCD_Car_OilTempSettingsList, &currentLayer, &submenuIterator);
				}

				if(ENC_button.longPressDetected)
				{
					doneOnce = FALSE;
					error = longButtonPressDetected_LCD(&LCD_Car_OilTempSettingsList[1], &currentLayer, &submenuIterator);
				}
				break;
			}
			case /* CarSettings_Layer -> OilTempSettings_Layer -> */OilTempWarningOn:
			{
				static boolean doneOnce = FALSE;
				error = ControlValueWithEncoder(&(LCD_Car_OilTempSettingsList[2]), &displayErrorFlag, &displayDoneFlag, &doneOnce);

				if(ENC_button.shortPressDetected)
				{
					doneOnce = FALSE;
					error = shortButtonPressDetected_LCD(&(LCD_Car_OilTempSettingsList[2]), LCD_Car_OilTempSettingsList, &currentLayer, &submenuIterator);
				}

				if(ENC_button.longPressDetected)
				{
					doneOnce = FALSE;
					error = longButtonPressDetected_LCD(&LCD_Car_OilTempSettingsList[2], &currentLayer, &submenuIterator);
				}
				break;
			}
			case /* CarSettings_Layer -> OilTempSettings_Layer -> */OilTempAlarmOn:
			{
				static boolean doneOnce = FALSE;
				error = ControlValueWithEncoder(&(LCD_Car_OilTempSettingsList[3]), &displayErrorFlag, &displayDoneFlag, &doneOnce);

				if(ENC_button.shortPressDetected)
				{
					doneOnce = FALSE;
					error = shortButtonPressDetected_LCD(&(LCD_Car_OilTempSettingsList[3]), LCD_Car_OilTempSettingsList, &currentLayer, &submenuIterator);
				}

				if(ENC_button.longPressDetected)
				{
					doneOnce = FALSE;
					error = longButtonPressDetected_LCD(&LCD_Car_OilTempSettingsList[3], &currentLayer, &submenuIterator);
				}
				break;
			}
			case /* CarSettings_Layer -> OilTempSettings_Layer -> */OilTempWarningBuzzerOn:
			{
				static boolean doneOnce = FALSE;
				error = ControlValueWithEncoder(&(LCD_Car_OilTempSettingsList[4]), &displayErrorFlag, &displayDoneFlag, &doneOnce);

				if(ENC_button.shortPressDetected)
				{
					doneOnce = FALSE;
					error = shortButtonPressDetected_LCD(&(LCD_Car_OilTempSettingsList[4]), LCD_Car_OilTempSettingsList, &currentLayer, &submenuIterator);
				}

				if(ENC_button.longPressDetected)
				{
					doneOnce = FALSE;
					error = longButtonPressDetected_LCD(&LCD_Car_OilTempSettingsList[4], &currentLayer, &submenuIterator);
				}
				break;
			}
			case /* CarSettings_Layer -> OilTempSettings_Layer -> */OilTempAlarmBuzzerOn:
			{
				static boolean doneOnce = FALSE;
				error = ControlValueWithEncoder(&(LCD_Car_OilTempSettingsList[5]), &displayErrorFlag, &displayDoneFlag, &doneOnce);

				if(ENC_button.shortPressDetected)
				{
					doneOnce = FALSE;
					error = shortButtonPressDetected_LCD(&(LCD_Car_OilTempSettingsList[5]), LCD_Car_OilTempSettingsList, &currentLayer, &submenuIterator);
				}

				if(ENC_button.longPressDetected)
				{
					doneOnce = FALSE;
					error = longButtonPressDetected_LCD(&LCD_Car_OilTempSettingsList[5], &currentLayer, &submenuIterator);
				}
				break;
			}
			case /* CarSettings_Layer -> OilTempSettings_Layer -> */OilTempWarningSnapshotOn:
			{
				static boolean doneOnce = FALSE;
				error = ControlValueWithEncoder(&(LCD_Car_OilTempSettingsList[6]), &displayErrorFlag, &displayDoneFlag, &doneOnce);

				if(ENC_button.shortPressDetected)
				{
					doneOnce = FALSE;
					error = shortButtonPressDetected_LCD(&(LCD_Car_OilTempSettingsList[6]), LCD_Car_OilTempSettingsList, &currentLayer, &submenuIterator);
				}

				if(ENC_button.longPressDetected)
				{
					doneOnce = FALSE;
					error = longButtonPressDetected_LCD(&LCD_Car_OilTempSettingsList[6], &currentLayer, &submenuIterator);
				}
				break;
			}
			case /* CarSettings_Layer -> OilTempSettings_Layer -> */OilTempAlarmSpashotOn:
			{
				static boolean doneOnce = FALSE;
				error = ControlValueWithEncoder(&(LCD_Car_OilTempSettingsList[7]), &displayErrorFlag, &displayDoneFlag, &doneOnce);

				if(ENC_button.shortPressDetected)
				{
					doneOnce = FALSE;
					error = shortButtonPressDetected_LCD(&(LCD_Car_OilTempSettingsList[7]), LCD_Car_OilTempSettingsList, &currentLayer, &submenuIterator);
				}

				if(ENC_button.longPressDetected)
				{
					doneOnce = FALSE;
					error = longButtonPressDetected_LCD(&LCD_Car_OilTempSettingsList[7], &currentLayer, &submenuIterator);
				}
				break;
			}
			/* *** *** *** *** *** *** *** *** *** *** *** *** *** *** *//* *** *** *** *** *** *** *** *** *** *** *** *** *** *** */


			case /* CarSettings_Layer -> */OilPressureSettings_Layer:
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

			/* *** *** *** *** *** *** *** *** *** *** *** *** *** *** *//* *** *** *** *** *** *** *** *** *** *** *** *** *** *** */
			/* CarSettings_Layer -> OilPressureSettings_Layer: */
			/* *** *** *** *** *** *** *** *** *** *** *** *** *** *** *//* *** *** *** *** *** *** *** *** *** *** *** *** *** *** */

			case /* CarSettings_Layer -> OilPressureSettings_Layer -> */OilHighPressureAlarmThreshold:
			{
				static boolean doneOnce = FALSE;
				error = ControlValueWithEncoder(&(LCD_Car_OilPressureSettingsList[0]), &displayErrorFlag, &displayDoneFlag, &doneOnce);

				if(ENC_button.shortPressDetected)
				{
					doneOnce = FALSE;
					error = shortButtonPressDetected_LCD(&(LCD_Car_OilPressureSettingsList[0]), LCD_Car_OilPressureSettingsList, &currentLayer, &submenuIterator);
				}

				if(ENC_button.longPressDetected)
				{
					doneOnce = FALSE;
					error = longButtonPressDetected_LCD(&LCD_Car_OilPressureSettingsList[0], &currentLayer, &submenuIterator);
				}
				break;
			}
			case /* CarSettings_Layer -> OilPressureSettings_Layer -> */OilLowPressureAlarmThreshold:
			{
				static boolean doneOnce = FALSE;
				error = ControlValueWithEncoder(&(LCD_Car_OilPressureSettingsList[1]), &displayErrorFlag, &displayDoneFlag, &doneOnce);

				if(ENC_button.shortPressDetected)
				{
					doneOnce = FALSE;
					error = shortButtonPressDetected_LCD(&(LCD_Car_OilPressureSettingsList[1]), LCD_Car_OilPressureSettingsList, &currentLayer, &submenuIterator);
				}

				if(ENC_button.longPressDetected)
				{
					doneOnce = FALSE;
					error = longButtonPressDetected_LCD(&LCD_Car_OilPressureSettingsList[1], &currentLayer, &submenuIterator);
				}
				break;
			}
			case /* CarSettings_Layer -> OilPressureSettings_Layer -> */OilPressureAnalogMeasurement:
			{
				static boolean doneOnce = FALSE;
				error = ControlValueWithEncoder(&(LCD_Car_OilPressureSettingsList[2]), &displayErrorFlag, &displayDoneFlag, &doneOnce);

				if(ENC_button.shortPressDetected)
				{
					doneOnce = FALSE;
					error = shortButtonPressDetected_LCD(&(LCD_Car_OilPressureSettingsList[2]), LCD_Car_OilPressureSettingsList, &currentLayer, &submenuIterator);
				}

				if(ENC_button.longPressDetected)
				{
					doneOnce = FALSE;
					error = longButtonPressDetected_LCD(&LCD_Car_OilPressureSettingsList[2], &currentLayer, &submenuIterator);
				}
				break;
			}
			case /* CarSettings_Layer -> OilPressureSettings_Layer -> */OilHighPressureAlarmOn:
			{
				static boolean doneOnce = FALSE;
				error = ControlValueWithEncoder(&(LCD_Car_OilPressureSettingsList[3]), &displayErrorFlag, &displayDoneFlag, &doneOnce);

				if(ENC_button.shortPressDetected)
				{
					doneOnce = FALSE;
					error = shortButtonPressDetected_LCD(&(LCD_Car_OilPressureSettingsList[3]), LCD_Car_OilPressureSettingsList, &currentLayer, &submenuIterator);
				}

				if(ENC_button.longPressDetected)
				{
					doneOnce = FALSE;
					error = longButtonPressDetected_LCD(&LCD_Car_OilPressureSettingsList[3], &currentLayer, &submenuIterator);
				}
				break;
			}
			case /* CarSettings_Layer -> OilPressureSettings_Layer -> */OilLowPressureAlarmOn:
			{
				static boolean doneOnce = FALSE;
				error = ControlValueWithEncoder(&(LCD_Car_OilPressureSettingsList[4]), &displayErrorFlag, &displayDoneFlag, &doneOnce);

				if(ENC_button.shortPressDetected)
				{
					doneOnce = FALSE;
					error = shortButtonPressDetected_LCD(&(LCD_Car_OilPressureSettingsList[4]), LCD_Car_OilPressureSettingsList, &currentLayer, &submenuIterator);
				}

				if(ENC_button.longPressDetected)
				{
					doneOnce = FALSE;
					error = longButtonPressDetected_LCD(&LCD_Car_OilPressureSettingsList[4], &currentLayer, &submenuIterator);
				}
				break;
			}
			case /* CarSettings_Layer -> OilPressureSettings_Layer -> */OilPressureAlarmBuzzerOn:
			{
				static boolean doneOnce = FALSE;
				error = ControlValueWithEncoder(&(LCD_Car_OilPressureSettingsList[5]), &displayErrorFlag, &displayDoneFlag, &doneOnce);

				if(ENC_button.shortPressDetected)
				{
					doneOnce = FALSE;
					error = shortButtonPressDetected_LCD(&(LCD_Car_OilPressureSettingsList[5]), LCD_Car_OilPressureSettingsList, &currentLayer, &submenuIterator);
				}

				if(ENC_button.longPressDetected)
				{
					doneOnce = FALSE;
					error = longButtonPressDetected_LCD(&LCD_Car_OilPressureSettingsList[5], &currentLayer, &submenuIterator);
				}
				break;
			}
			case /* CarSettings_Layer -> OilPressureSettings_Layer -> */OilPressureAlarmSnapshotOn:
			{
				static boolean doneOnce = FALSE;
				error = ControlValueWithEncoder(&(LCD_Car_OilPressureSettingsList[6]), &displayErrorFlag, &displayDoneFlag, &doneOnce);

				if(ENC_button.shortPressDetected)
				{
					doneOnce = FALSE;
					error = shortButtonPressDetected_LCD(&(LCD_Car_OilPressureSettingsList[6]), LCD_Car_OilPressureSettingsList, &currentLayer, &submenuIterator);
				}

				if(ENC_button.longPressDetected)
				{
					doneOnce = FALSE;
					error = longButtonPressDetected_LCD(&LCD_Car_OilPressureSettingsList[6], &currentLayer, &submenuIterator);
				}
				break;
			}
			/* *** *** *** *** *** *** *** *** *** *** *** *** *** *** *//* *** *** *** *** *** *** *** *** *** *** *** *** *** *** */


			case /* CarSettings_Layer -> */FuelSettings_Layer:
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

			/* *** *** *** *** *** *** *** *** *** *** *** *** *** *** *//* *** *** *** *** *** *** *** *** *** *** *** *** *** *** */
			/* CarSettings_Layer -> FuelSettings_Layer: */
			/* *** *** *** *** *** *** *** *** *** *** *** *** *** *** *//* *** *** *** *** *** *** *** *** *** *** *** *** *** *** */

			case /* CarSettings_Layer -> FuelSettings_Layer -> */FuelLowLevelWarningThreshold:
			{
				static boolean doneOnce = FALSE;
				error = ControlValueWithEncoder(&(LCD_Car_FuelSettingsList[0]), &displayErrorFlag, &displayDoneFlag, &doneOnce);

				if(ENC_button.shortPressDetected)
				{
					doneOnce = FALSE;
					error = shortButtonPressDetected_LCD(&(LCD_Car_FuelSettingsList[0]), LCD_Car_FuelSettingsList, &currentLayer, &submenuIterator);
				}

				if(ENC_button.longPressDetected)
				{
					doneOnce = FALSE;
					error = longButtonPressDetected_LCD(&LCD_Car_FuelSettingsList[0], &currentLayer, &submenuIterator);
				}
				break;
			}
			case /* CarSettings_Layer -> FuelSettings_Layer -> */FuelLowLevelWarningOn:
			{
				static boolean doneOnce = FALSE;
				error = ControlValueWithEncoder(&(LCD_Car_FuelSettingsList[1]), &displayErrorFlag, &displayDoneFlag, &doneOnce);

				if(ENC_button.shortPressDetected)
				{
					doneOnce = FALSE;
					error = shortButtonPressDetected_LCD(&(LCD_Car_FuelSettingsList[1]), LCD_Car_FuelSettingsList, &currentLayer, &submenuIterator);
				}

				if(ENC_button.longPressDetected)
				{
					doneOnce = FALSE;
					error = longButtonPressDetected_LCD(&LCD_Car_FuelSettingsList[1], &currentLayer, &submenuIterator);
				}
				break;
			}
			case /* CarSettings_Layer -> FuelSettings_Layer -> */FuelLowLevelWarningBuzzerOn:
			{
				static boolean doneOnce = FALSE;
				error = ControlValueWithEncoder(&(LCD_Car_FuelSettingsList[2]), &displayErrorFlag, &displayDoneFlag, &doneOnce);

				if(ENC_button.shortPressDetected)
				{
					doneOnce = FALSE;
					error = shortButtonPressDetected_LCD(&(LCD_Car_FuelSettingsList[2]), LCD_Car_FuelSettingsList, &currentLayer, &submenuIterator);
				}

				if(ENC_button.longPressDetected)
				{
					doneOnce = FALSE;
					error = longButtonPressDetected_LCD(&LCD_Car_FuelSettingsList[2], &currentLayer, &submenuIterator);
				}
				break;
			}
			/* *** *** *** *** *** *** *** *** *** *** *** *** *** *** *//* *** *** *** *** *** *** *** *** *** *** *** *** *** *** */


			case /* CarSettings_Layer -> */MainBatterySettings_Layer:
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

			/* *** *** *** *** *** *** *** *** *** *** *** *** *** *** *//* *** *** *** *** *** *** *** *** *** *** *** *** *** *** */
			/* CarSettings_Layer -> MainBatterySettings_Layer: */
			/* *** *** *** *** *** *** *** *** *** *** *** *** *** *** *//* *** *** *** *** *** *** *** *** *** *** *** *** *** *** */

			case /* CarSettings_Layer -> MainBatterySettings_Layer -> */MainBatteryLowVoltageAlarmThreshold:
			{
				static boolean doneOnce = FALSE;
				error = ControlValueWithEncoder(&(LCD_Car_MainBatterySettingsList[0]), &displayErrorFlag, &displayDoneFlag, &doneOnce);

				if(ENC_button.shortPressDetected)
				{
					doneOnce = FALSE;
					error = shortButtonPressDetected_LCD(&(LCD_Car_MainBatterySettingsList[0]), LCD_Car_MainBatterySettingsList, &currentLayer, &submenuIterator);
				}

				if(ENC_button.longPressDetected)
				{
					doneOnce = FALSE;
					error = longButtonPressDetected_LCD(&LCD_Car_MainBatterySettingsList[0], &currentLayer, &submenuIterator);
				}
				break;
			}
			case /* CarSettings_Layer -> MainBatterySettings_Layer -> */MainBatteryHighVoltageAlarmThreshold:
			{
				static boolean doneOnce = FALSE;
				error = ControlValueWithEncoder(&(LCD_Car_MainBatterySettingsList[1]), &displayErrorFlag, &displayDoneFlag, &doneOnce);

				if(ENC_button.shortPressDetected)
				{
					doneOnce = FALSE;
					error = shortButtonPressDetected_LCD(&(LCD_Car_MainBatterySettingsList[1]), LCD_Car_MainBatterySettingsList, &currentLayer, &submenuIterator);
				}

				if(ENC_button.longPressDetected)
				{
					doneOnce = FALSE;
					error = longButtonPressDetected_LCD(&LCD_Car_MainBatterySettingsList[1], &currentLayer, &submenuIterator);
				}
				break;
			}
			case /* CarSettings_Layer -> MainBatterySettings_Layer -> */MainBatteryLowVoltageAlarmOn:
			{
				static boolean doneOnce = FALSE;
				error = ControlValueWithEncoder(&(LCD_Car_MainBatterySettingsList[2]), &displayErrorFlag, &displayDoneFlag, &doneOnce);

				if(ENC_button.shortPressDetected)
				{
					doneOnce = FALSE;
					error = shortButtonPressDetected_LCD(&(LCD_Car_MainBatterySettingsList[2]), LCD_Car_MainBatterySettingsList, &currentLayer, &submenuIterator);
				}

				if(ENC_button.longPressDetected)
				{
					doneOnce = FALSE;
					error = longButtonPressDetected_LCD(&LCD_Car_MainBatterySettingsList[2], &currentLayer, &submenuIterator);
				}
				break;
			}
			case /* CarSettings_Layer -> MainBatterySettings_Layer -> */MainBatteryHighVoltageAlarmOn:
			{
				static boolean doneOnce = FALSE;
				error = ControlValueWithEncoder(&(LCD_Car_MainBatterySettingsList[3]), &displayErrorFlag, &displayDoneFlag, &doneOnce);

				if(ENC_button.shortPressDetected)
				{
					doneOnce = FALSE;
					error = shortButtonPressDetected_LCD(&(LCD_Car_MainBatterySettingsList[3]), LCD_Car_MainBatterySettingsList, &currentLayer, &submenuIterator);
				}

				if(ENC_button.longPressDetected)
				{
					doneOnce = FALSE;
					error = longButtonPressDetected_LCD(&LCD_Car_MainBatterySettingsList[3], &currentLayer, &submenuIterator);
				}
				break;
			}
			case /* CarSettings_Layer -> MainBatterySettings_Layer -> */MainBatteryAlarmBuzzerOn:
			{
				static boolean doneOnce = FALSE;
				error = ControlValueWithEncoder(&(LCD_Car_MainBatterySettingsList[4]), &displayErrorFlag, &displayDoneFlag, &doneOnce);

				if(ENC_button.shortPressDetected)
				{
					doneOnce = FALSE;
					error = shortButtonPressDetected_LCD(&(LCD_Car_MainBatterySettingsList[4]), LCD_Car_MainBatterySettingsList, &currentLayer, &submenuIterator);
				}

				if(ENC_button.longPressDetected)
				{
					doneOnce = FALSE;
					error = longButtonPressDetected_LCD(&LCD_Car_MainBatterySettingsList[4], &currentLayer, &submenuIterator);
				}
				break;
			}
			case /* CarSettings_Layer -> MainBatterySettings_Layer -> */MainBatteryLowVoltageSnapshotOn:
			{
				static boolean doneOnce = FALSE;
				error = ControlValueWithEncoder(&(LCD_Car_MainBatterySettingsList[5]), &displayErrorFlag, &displayDoneFlag, &doneOnce);

				if(ENC_button.shortPressDetected)
				{
					doneOnce = FALSE;
					error = shortButtonPressDetected_LCD(&(LCD_Car_MainBatterySettingsList[5]), LCD_Car_MainBatterySettingsList, &currentLayer, &submenuIterator);
				}

				if(ENC_button.longPressDetected)
				{
					doneOnce = FALSE;
					error = longButtonPressDetected_LCD(&LCD_Car_MainBatterySettingsList[5], &currentLayer, &submenuIterator);
				}
				break;
			}
			case /* CarSettings_Layer -> MainBatterySettings_Layer -> */MainBatteryHighVoltageSnapshotOn:
			{
				static boolean doneOnce = FALSE;
				error = ControlValueWithEncoder(&(LCD_Car_MainBatterySettingsList[6]), &displayErrorFlag, &displayDoneFlag, &doneOnce);

				if(ENC_button.shortPressDetected)
				{
					doneOnce = FALSE;
					error = shortButtonPressDetected_LCD(&(LCD_Car_MainBatterySettingsList[6]), LCD_Car_MainBatterySettingsList, &currentLayer, &submenuIterator);
				}

				if(ENC_button.longPressDetected)
				{
					doneOnce = FALSE;
					error = longButtonPressDetected_LCD(&LCD_Car_MainBatterySettingsList[6], &currentLayer, &submenuIterator);
				}
				break;
			}
			/* *** *** *** *** *** *** *** *** *** *** *** *** *** *** *//* *** *** *** *** *** *** *** *** *** *** *** *** *** *** */


			case /* CarSettings_Layer -> */AuxBatterySettings_Layer:
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

			/* *** *** *** *** *** *** *** *** *** *** *** *** *** *** *//* *** *** *** *** *** *** *** *** *** *** *** *** *** *** */
			/* CarSettings_Layer -> AuxBatterySettings_Layer: */
			/* *** *** *** *** *** *** *** *** *** *** *** *** *** *** *//* *** *** *** *** *** *** *** *** *** *** *** *** *** *** */

			case /* CarSettings_Layer -> AuxBatterySettings_Layer -> */AuxBatteryLowVoltageAlarmThreshold:
			{
				static boolean doneOnce = FALSE;
				error = ControlValueWithEncoder(&(LCD_Car_AuxBatterySettingsList[0]), &displayErrorFlag, &displayDoneFlag, &doneOnce);

				if(ENC_button.shortPressDetected)
				{
					doneOnce = FALSE;
					error = shortButtonPressDetected_LCD(&(LCD_Car_AuxBatterySettingsList[0]), LCD_Car_AuxBatterySettingsList, &currentLayer, &submenuIterator);
				}

				if(ENC_button.longPressDetected)
				{
					doneOnce = FALSE;
					error = longButtonPressDetected_LCD(&LCD_Car_AuxBatterySettingsList[0], &currentLayer, &submenuIterator);
				}
				break;
			}
			case /* CarSettings_Layer -> AuxBatterySettings_Layer -> */AuxBatteryHighVoltageAlarmThreshold:
			{
				static boolean doneOnce = FALSE;
				error = ControlValueWithEncoder(&(LCD_Car_AuxBatterySettingsList[1]), &displayErrorFlag, &displayDoneFlag, &doneOnce);

				if(ENC_button.shortPressDetected)
				{
					doneOnce = FALSE;
					error = shortButtonPressDetected_LCD(&(LCD_Car_AuxBatterySettingsList[1]), LCD_Car_AuxBatterySettingsList, &currentLayer, &submenuIterator);
				}

				if(ENC_button.longPressDetected)
				{
					doneOnce = FALSE;
					error = longButtonPressDetected_LCD(&LCD_Car_AuxBatterySettingsList[1], &currentLayer, &submenuIterator);
				}
				break;
			}
			case /* CarSettings_Layer -> AuxBatterySettings_Layer -> */AuxBatteryLowVoltageAlarmOn:
			{
				static boolean doneOnce = FALSE;
				error = ControlValueWithEncoder(&(LCD_Car_AuxBatterySettingsList[2]), &displayErrorFlag, &displayDoneFlag, &doneOnce);

				if(ENC_button.shortPressDetected)
				{
					doneOnce = FALSE;
					error = shortButtonPressDetected_LCD(&(LCD_Car_AuxBatterySettingsList[2]), LCD_Car_AuxBatterySettingsList, &currentLayer, &submenuIterator);
				}

				if(ENC_button.longPressDetected)
				{
					doneOnce = FALSE;
					error = longButtonPressDetected_LCD(&LCD_Car_AuxBatterySettingsList[2], &currentLayer, &submenuIterator);
				}
				break;
			}
			case /* CarSettings_Layer -> AuxBatterySettings_Layer -> */AuxBatteryHighVoltageAlarmOn:
			{
				static boolean doneOnce = FALSE;
				error = ControlValueWithEncoder(&(LCD_Car_AuxBatterySettingsList[3]), &displayErrorFlag, &displayDoneFlag, &doneOnce);

				if(ENC_button.shortPressDetected)
				{
					doneOnce = FALSE;
					error = shortButtonPressDetected_LCD(&(LCD_Car_AuxBatterySettingsList[3]), LCD_Car_AuxBatterySettingsList, &currentLayer, &submenuIterator);
				}

				if(ENC_button.longPressDetected)
				{
					doneOnce = FALSE;
					error = longButtonPressDetected_LCD(&LCD_Car_AuxBatterySettingsList[3], &currentLayer, &submenuIterator);
				}
				break;
			}
			case /* CarSettings_Layer -> AuxBatterySettings_Layer -> */AuxBatteryAlarmBuzzerOn:
			{
				static boolean doneOnce = FALSE;
				error = ControlValueWithEncoder(&(LCD_Car_AuxBatterySettingsList[4]), &displayErrorFlag, &displayDoneFlag, &doneOnce);

				if(ENC_button.shortPressDetected)
				{
					doneOnce = FALSE;
					error = shortButtonPressDetected_LCD(&(LCD_Car_AuxBatterySettingsList[4]), LCD_Car_AuxBatterySettingsList, &currentLayer, &submenuIterator);
				}

				if(ENC_button.longPressDetected)
				{
					doneOnce = FALSE;
					error = longButtonPressDetected_LCD(&LCD_Car_AuxBatterySettingsList[4], &currentLayer, &submenuIterator);
				}
				break;
			}
			case /* CarSettings_Layer -> AuxBatterySettings_Layer -> */AuxBatteryLowVoltageSnapshotOn:
			{
				static boolean doneOnce = FALSE;
				error = ControlValueWithEncoder(&(LCD_Car_AuxBatterySettingsList[6]), &displayErrorFlag, &displayDoneFlag, &doneOnce);

				if(ENC_button.shortPressDetected)
				{
					doneOnce = FALSE;
					error = shortButtonPressDetected_LCD(&(LCD_Car_AuxBatterySettingsList[6]), LCD_Car_AuxBatterySettingsList, &currentLayer, &submenuIterator);
				}

				if(ENC_button.longPressDetected)
				{
					doneOnce = FALSE;
					error = longButtonPressDetected_LCD(&LCD_Car_AuxBatterySettingsList[6], &currentLayer, &submenuIterator);
				}
				break;
			}
			case /* CarSettings_Layer -> AuxBatterySettings_Layer -> */AuxBatteryHighVoltageSnapshotOn:
			{
				static boolean doneOnce = FALSE;
				error = ControlValueWithEncoder(&(LCD_Car_AuxBatterySettingsList[7]), &displayErrorFlag, &displayDoneFlag, &doneOnce);

				if(ENC_button.shortPressDetected)
				{
					doneOnce = FALSE;
					error = shortButtonPressDetected_LCD(&(LCD_Car_AuxBatterySettingsList[7]), LCD_Car_AuxBatterySettingsList, &currentLayer, &submenuIterator);
				}

				if(ENC_button.longPressDetected)
				{
					doneOnce = FALSE;
					error = longButtonPressDetected_LCD(&LCD_Car_AuxBatterySettingsList[7], &currentLayer, &submenuIterator);
				}
				break;
			}
			/* *** *** *** *** *** *** *** *** *** *** *** *** *** *** *//* *** *** *** *** *** *** *** *** *** *** *** *** *** *** */

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

			/* *** *** *** *** *** *** *** *** *** *** *** *** *** *** *//* *** *** *** *** *** *** *** *** *** *** *** *** *** *** */
			/* Board settings list : */
			/* *** *** *** *** *** *** *** *** *** *** *** *** *** *** *//* *** *** *** *** *** *** *** *** *** *** *** *** *** *** */

			case /* BoardSettings_Layer -> */ClearErrorsSnapshots:
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
			case /* BoardSettings_Layer -> */AdjTimePoland:
			{
				// TODO dorobic kontrole
				if(ENC_button.longPressDetected)
				{
					error = longButtonPressDetected_LCD(&LCD_BoardSettingsList[0], &currentLayer, &submenuIterator);
				}
				break;
			}
			case /* BoardSettings_Layer -> */AdjTimeZone:
			{
				// TODO dorobic kontrole
				if(ENC_button.longPressDetected)
				{
					error = longButtonPressDetected_LCD(&LCD_BoardSettingsList[0], &currentLayer, &submenuIterator);
				}
				break;
			}
			case /* BoardSettings_Layer -> */InterVoltSettings_Layer:
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

			/* *** *** *** *** *** *** *** *** *** *** *** *** *** *** *//* *** *** *** *** *** *** *** *** *** *** *** *** *** *** */
			/* BoardSettings_Layer -> InterVoltSettings_Layer: */
			/* *** *** *** *** *** *** *** *** *** *** *** *** *** *** *//* *** *** *** *** *** *** *** *** *** *** *** *** *** *** */

			case /* BoardSettings_Layer -> InterVoltSettings_Layer -> */Supply5VLowThreshold:
			{
				static boolean doneOnce = FALSE;
				error = ControlValueWithEncoder(&(LCD_Board_InternalVoltSettingsList[0]), &displayErrorFlag, &displayDoneFlag, &doneOnce);

				if(ENC_button.shortPressDetected)
				{
					doneOnce = FALSE;
					error = shortButtonPressDetected_LCD(&(LCD_Board_InternalVoltSettingsList[0]), LCD_Board_InternalVoltSettingsList, &currentLayer, &submenuIterator);
				}

				if(ENC_button.longPressDetected)
				{
					doneOnce = FALSE;
					error = longButtonPressDetected_LCD(&LCD_Board_InternalVoltSettingsList[0], &currentLayer, &submenuIterator);
				}
				break;
			}
			case /* BoardSettings_Layer -> InterVoltSettings_Layer -> */Supply5VHighThreshold:
			{
				static boolean doneOnce = FALSE;
				error = ControlValueWithEncoder(&(LCD_Board_InternalVoltSettingsList[1]), &displayErrorFlag, &displayDoneFlag, &doneOnce);

				if(ENC_button.shortPressDetected)
				{
					doneOnce = FALSE;
					error = shortButtonPressDetected_LCD(&(LCD_Board_InternalVoltSettingsList[1]), LCD_Board_InternalVoltSettingsList, &currentLayer, &submenuIterator);
				}

				if(ENC_button.longPressDetected)
				{
					doneOnce = FALSE;
					error = longButtonPressDetected_LCD(&LCD_Board_InternalVoltSettingsList[1], &currentLayer, &submenuIterator);
				}
				break;
			}
			case /* BoardSettings_Layer -> InterVoltSettings_Layer -> */Supply3V3LowThreshold:
			{
				static boolean doneOnce = FALSE;
				error = ControlValueWithEncoder(&(LCD_Board_InternalVoltSettingsList[2]), &displayErrorFlag, &displayDoneFlag, &doneOnce);

				if(ENC_button.shortPressDetected)
				{
					doneOnce = FALSE;
					error = shortButtonPressDetected_LCD(&(LCD_Board_InternalVoltSettingsList[2]), LCD_Board_InternalVoltSettingsList, &currentLayer, &submenuIterator);
				}

				if(ENC_button.longPressDetected)
				{
					doneOnce = FALSE;
					error = longButtonPressDetected_LCD(&LCD_Board_InternalVoltSettingsList[2], &currentLayer, &submenuIterator);
				}
				break;
			}
			case /* BoardSettings_Layer -> InterVoltSettings_Layer -> */Supply3V3HighThreshold:
			{
				static boolean doneOnce = FALSE;
				error = ControlValueWithEncoder(&(LCD_Board_InternalVoltSettingsList[3]), &displayErrorFlag, &displayDoneFlag, &doneOnce);

				if(ENC_button.shortPressDetected)
				{
					doneOnce = FALSE;
					error = shortButtonPressDetected_LCD(&(LCD_Board_InternalVoltSettingsList[3]), LCD_Board_InternalVoltSettingsList, &currentLayer, &submenuIterator);
				}

				if(ENC_button.longPressDetected)
				{
					doneOnce = FALSE;
					error = longButtonPressDetected_LCD(&LCD_Board_InternalVoltSettingsList[3], &currentLayer, &submenuIterator);
				}
				break;
			}
			case /* BoardSettings_Layer -> InterVoltSettings_Layer -> */SupplyVinLowThreshold:
			{
				static boolean doneOnce = FALSE;
				error = ControlValueWithEncoder(&(LCD_Board_InternalVoltSettingsList[4]), &displayErrorFlag, &displayDoneFlag, &doneOnce);

				if(ENC_button.shortPressDetected)
				{
					doneOnce = FALSE;
					error = shortButtonPressDetected_LCD(&(LCD_Board_InternalVoltSettingsList[4]), LCD_Board_InternalVoltSettingsList, &currentLayer, &submenuIterator);
				}

				if(ENC_button.longPressDetected)
				{
					doneOnce = FALSE;
					error = longButtonPressDetected_LCD(&LCD_Board_InternalVoltSettingsList[4], &currentLayer, &submenuIterator);
				}
				break;
			}
			/* *** *** *** *** *** *** *** *** *** *** *** *** *** *** *//* *** *** *** *** *** *** *** *** *** *** *** *** *** *** */


			case /* BoardSettings_Layer -> */InterTempSettings_Layer:
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

			/* *** *** *** *** *** *** *** *** *** *** *** *** *** *** *//* *** *** *** *** *** *** *** *** *** *** *** *** *** *** */
			/* BoardSettings_Layer -> InterTempSettings_Layer: */
			/* *** *** *** *** *** *** *** *** *** *** *** *** *** *** *//* *** *** *** *** *** *** *** *** *** *** *** *** *** *** */

			case /* BoardSettings_Layer -> InterTempSettings_Layer -> */DCDC3V3HighTempThreshold:
			{
				static boolean doneOnce = FALSE;
				error = ControlValueWithEncoder(&(LCD_Board_InternalTempSettingsList[0]), &displayErrorFlag, &displayDoneFlag, &doneOnce);

				if(ENC_button.shortPressDetected)
				{
					doneOnce = FALSE;
					error = shortButtonPressDetected_LCD(&(LCD_Board_InternalTempSettingsList[0]), LCD_Board_InternalTempSettingsList, &currentLayer, &submenuIterator);
				}

				if(ENC_button.longPressDetected)
				{
					doneOnce = FALSE;
					error = longButtonPressDetected_LCD(&LCD_Board_InternalTempSettingsList[0], &currentLayer, &submenuIterator);
				}
				break;
			}
			case /* BoardSettings_Layer -> InterTempSettings_Layer -> */DCDC5VHighTempThreshold:
			{
				static boolean doneOnce = FALSE;
				error = ControlValueWithEncoder(&(LCD_Board_InternalTempSettingsList[1]), &displayErrorFlag, &displayDoneFlag, &doneOnce);

				if(ENC_button.shortPressDetected)
				{
					doneOnce = FALSE;
					error = shortButtonPressDetected_LCD(&(LCD_Board_InternalTempSettingsList[1]), LCD_Board_InternalTempSettingsList, &currentLayer, &submenuIterator);
				}

				if(ENC_button.longPressDetected)
				{
					doneOnce = FALSE;
					error = longButtonPressDetected_LCD(&LCD_Board_InternalTempSettingsList[1], &currentLayer, &submenuIterator);
				}
				break;
			}
			/* *** *** *** *** *** *** *** *** *** *** *** *** *** *** *//* *** *** *** *** *** *** *** *** *** *** *** *** *** *** */


			case /* BoardSettings_Layer -> */BuzzerSettings_Layer:
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

			/* *** *** *** *** *** *** *** *** *** *** *** *** *** *** *//* *** *** *** *** *** *** *** *** *** *** *** *** *** *** */
			/* BoardSettings_Layer -> BuzzerSettings_Layer: */
			/* *** *** *** *** *** *** *** *** *** *** *** *** *** *** *//* *** *** *** *** *** *** *** *** *** *** *** *** *** *** */

			case /* BoardSettings_Layer -> BuzzerSettings_Layer -> */BuzzerMainSwitch:
			{
				static boolean doneOnce = FALSE;
				error = ControlValueWithEncoder(&(LCD_Board_BuzzerSettingsList[0]), &displayErrorFlag, &displayDoneFlag, &doneOnce);

				if(ENC_button.shortPressDetected)
				{
					doneOnce = FALSE;
					error = shortButtonPressDetected_LCD(&(LCD_Board_BuzzerSettingsList[0]), LCD_Board_BuzzerSettingsList, &currentLayer, &submenuIterator);
				}

				if(ENC_button.longPressDetected)
				{
					doneOnce = FALSE;
					error = longButtonPressDetected_LCD(&LCD_Board_BuzzerSettingsList[0], &currentLayer, &submenuIterator);
				}
				break;
			}
			case /* BoardSettings_Layer -> BuzzerSettings_Layer -> */BuzzerMainAlarmsSwitch:
			{
				static boolean doneOnce = FALSE;
				error = ControlValueWithEncoder(&(LCD_Board_BuzzerSettingsList[1]), &displayErrorFlag, &displayDoneFlag, &doneOnce);

				if(ENC_button.shortPressDetected)
				{
					doneOnce = FALSE;
					error = shortButtonPressDetected_LCD(&(LCD_Board_BuzzerSettingsList[1]), LCD_Board_BuzzerSettingsList, &currentLayer, &submenuIterator);
				}

				if(ENC_button.longPressDetected)
				{
					doneOnce = FALSE;
					error = longButtonPressDetected_LCD(&LCD_Board_BuzzerSettingsList[1], &currentLayer, &submenuIterator);
				}
				break;
			}
			case /* BoardSettings_Layer -> BuzzerSettings_Layer -> */BuzzerMainButtonsSwitch:
			{
				static boolean doneOnce = FALSE;
				error = ControlValueWithEncoder(&(LCD_Board_BuzzerSettingsList[2]), &displayErrorFlag, &displayDoneFlag, &doneOnce);

				if(ENC_button.shortPressDetected)
				{
					doneOnce = FALSE;
					error = shortButtonPressDetected_LCD(&(LCD_Board_BuzzerSettingsList[2]), LCD_Board_BuzzerSettingsList, &currentLayer, &submenuIterator);
				}

				if(ENC_button.longPressDetected)
				{
					doneOnce = FALSE;
					error = longButtonPressDetected_LCD(&LCD_Board_BuzzerSettingsList[2], &currentLayer, &submenuIterator);
				}
				break;
			}
			case /* BoardSettings_Layer -> BuzzerSettings_Layer -> */BuzzerWhenShortPress:
			{
				static boolean doneOnce = FALSE;
				error = ControlValueWithEncoder(&(LCD_Board_BuzzerSettingsList[3]), &displayErrorFlag, &displayDoneFlag, &doneOnce);

				if(ENC_button.shortPressDetected)
				{
					doneOnce = FALSE;
					error = shortButtonPressDetected_LCD(&(LCD_Board_BuzzerSettingsList[3]), LCD_Board_BuzzerSettingsList, &currentLayer, &submenuIterator);
				}

				if(ENC_button.longPressDetected)
				{
					doneOnce = FALSE;
					error = longButtonPressDetected_LCD(&LCD_Board_BuzzerSettingsList[3], &currentLayer, &submenuIterator);
				}
				break;
			}
			case /* BoardSettings_Layer -> BuzzerSettings_Layer -> */BuzzerWhenLongPress:
			{
				static boolean doneOnce = FALSE;
				error = ControlValueWithEncoder(&(LCD_Board_BuzzerSettingsList[4]), &displayErrorFlag, &displayDoneFlag, &doneOnce);

				if(ENC_button.shortPressDetected)
				{
					doneOnce = FALSE;
					error = shortButtonPressDetected_LCD(&(LCD_Board_BuzzerSettingsList[4]), LCD_Board_BuzzerSettingsList, &currentLayer, &submenuIterator);
				}

				if(ENC_button.longPressDetected)
				{
					doneOnce = FALSE;
					error = longButtonPressDetected_LCD(&LCD_Board_BuzzerSettingsList[4], &currentLayer, &submenuIterator);
				}
				break;
			}
			/* *** *** *** *** *** *** *** *** *** *** *** *** *** *** *//* *** *** *** *** *** *** *** *** *** *** *** *** *** *** */


			case /* BoardSettings_Layer -> */LCDSettings_Layer:
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

			/* *** *** *** *** *** *** *** *** *** *** *** *** *** *** *//* *** *** *** *** *** *** *** *** *** *** *** *** *** *** */
			/* BoardSettings_Layer -> LCDSettings_Layer: */
			/* *** *** *** *** *** *** *** *** *** *** *** *** *** *** *//* *** *** *** *** *** *** *** *** *** *** *** *** *** *** */

			case /* BoardSettings_Layer -> LCDSettings_Layer -> */BacklightBrightnessLevel:
			{
				static boolean doneOnce = FALSE;
				error = ControlValueWithEncoder(&(LCD_Board_LCDSettingsList[0]), &displayErrorFlag, &displayDoneFlag, &doneOnce);

				if(ENC_button.shortPressDetected)
				{
					doneOnce = FALSE;
					error = shortButtonPressDetected_LCD(&(LCD_Board_LCDSettingsList[0]), LCD_Board_LCDSettingsList, &currentLayer, &submenuIterator);
				}

				if(ENC_button.longPressDetected)
				{
					doneOnce = FALSE;
					error = longButtonPressDetected_LCD(&LCD_Board_LCDSettingsList[0], &currentLayer, &submenuIterator);
				}
				break;
			}
			case /* BoardSettings_Layer -> LCDSettings_Layer -> */SecondsToTurnOffBacklight:
			{
				static boolean doneOnce = FALSE;
				error = ControlValueWithEncoder(&(LCD_Board_LCDSettingsList[1]), &displayErrorFlag, &displayDoneFlag, &doneOnce);

				if(ENC_button.shortPressDetected)
				{
					doneOnce = FALSE;
					error = shortButtonPressDetected_LCD(&(LCD_Board_LCDSettingsList[1]), LCD_Board_LCDSettingsList, &currentLayer, &submenuIterator);
				}

				if(ENC_button.longPressDetected)
				{
					doneOnce = FALSE;
					error = longButtonPressDetected_LCD(&LCD_Board_LCDSettingsList[1], &currentLayer, &submenuIterator);
				}
				break;
			}
			case /* BoardSettings_Layer -> LCDSettings_Layer -> */AutoBacklightOffStartHour:
			{
				static boolean doneOnce = FALSE;
				error = ControlValueWithEncoder(&(LCD_Board_LCDSettingsList[24]), &displayErrorFlag, &displayDoneFlag, &doneOnce);

				if(ENC_button.shortPressDetected)
				{
					doneOnce = FALSE;
					error = shortButtonPressDetected_LCD(&(LCD_Board_LCDSettingsList[2]), LCD_Board_LCDSettingsList, &currentLayer, &submenuIterator);
				}

				if(ENC_button.longPressDetected)
				{
					doneOnce = FALSE;
					error = longButtonPressDetected_LCD(&LCD_Board_LCDSettingsList[2], &currentLayer, &submenuIterator);
				}
				break;
			}
			case /* BoardSettings_Layer -> LCDSettings_Layer -> */AutoBacklightOffEndHour:
			{
				static boolean doneOnce = FALSE;
				error = ControlValueWithEncoder(&(LCD_Board_LCDSettingsList[3]), &displayErrorFlag, &displayDoneFlag, &doneOnce);

				if(ENC_button.shortPressDetected)
				{
					doneOnce = FALSE;
					error = shortButtonPressDetected_LCD(&(LCD_Board_LCDSettingsList[3]), LCD_Board_LCDSettingsList, &currentLayer, &submenuIterator);
				}

				if(ENC_button.longPressDetected)
				{
					doneOnce = FALSE;
					error = longButtonPressDetected_LCD(&LCD_Board_LCDSettingsList[3], &currentLayer, &submenuIterator);
				}
				break;
			}
			case /* BoardSettings_Layer -> LCDSettings_Layer -> */HomeScreen:
			{
				static boolean doneOnce = FALSE;
				error = ControlValueWithEncoder(&(LCD_Board_LCDSettingsList[4]), &displayErrorFlag, &displayDoneFlag, &doneOnce);

				if(ENC_button.shortPressDetected)
				{
					doneOnce = FALSE;
					error = shortButtonPressDetected_LCD(&(LCD_Board_LCDSettingsList[4]), LCD_Board_LCDSettingsList, &currentLayer, &submenuIterator);
				}

				if(ENC_button.longPressDetected)
				{
					doneOnce = FALSE;
					error = longButtonPressDetected_LCD(&LCD_Board_LCDSettingsList[4], &currentLayer, &submenuIterator);
				}
				break;
			}
			case /* BoardSettings_Layer -> LCDSettings_Layer -> */AutoHomeReturnTime:
			{
				static boolean doneOnce = FALSE;
				error = ControlValueWithEncoder(&(LCD_Board_LCDSettingsList[5]), &displayErrorFlag, &displayDoneFlag, &doneOnce);

				if(ENC_button.shortPressDetected)
				{
					doneOnce = FALSE;
					error = shortButtonPressDetected_LCD(&(LCD_Board_LCDSettingsList[5]), LCD_Board_LCDSettingsList, &currentLayer, &submenuIterator);
				}

				if(ENC_button.longPressDetected)
				{
					doneOnce = FALSE;
					error = longButtonPressDetected_LCD(&LCD_Board_LCDSettingsList[5], &currentLayer, &submenuIterator);
				}
				break;
			}
			case /* BoardSettings_Layer -> LCDSettings_Layer -> */AutoBacklightOff:
			{
				static boolean doneOnce = FALSE;
				error = ControlValueWithEncoder(&(LCD_Board_LCDSettingsList[6]), &displayErrorFlag, &displayDoneFlag, &doneOnce);

				if(ENC_button.shortPressDetected)
				{
					doneOnce = FALSE;
					error = shortButtonPressDetected_LCD(&(LCD_Board_LCDSettingsList[6]), LCD_Board_LCDSettingsList, &currentLayer, &submenuIterator);
				}

				if(ENC_button.longPressDetected)
				{
					doneOnce = FALSE;
					error = longButtonPressDetected_LCD(&LCD_Board_LCDSettingsList[6], &currentLayer, &submenuIterator);
				}
				break;
			}
			/* *** *** *** *** *** *** *** *** *** *** *** *** *** *** *//* *** *** *** *** *** *** *** *** *** *** *** *** *** *** */


			/* *** *** *** *** *** *** *** *** *** *** *** *** *** *** *//* *** *** *** *** *** *** *** *** *** *** *** *** *** *** */
			/* Actions layers */
			/* *** *** *** *** *** *** *** *** *** *** *** *** *** *** *//* *** *** *** *** *** *** *** *** *** *** *** *** *** *** */

			case YesNo_Layer:
			{
				error = copy_str_to_buffer(LCD_YesNo.name, (char*)LCD_buffer[Row1], 3, LCD_YesNo.nameActualSize);
				error = copy_str_to_buffer("Short press: Yes", (char*)LCD_buffer[Row2], 0, 16);
				error = copy_str_to_buffer("Long press: No", (char*)LCD_buffer[Row3], 0, 14);

				switch(LCD_YesNo.layerPrevious)
				{
					case /* CarSettings_Layer -> */ClearDiagnosticSnapshots:
					{
						if(ENC_button.shortPressDetected)
						{
							CREATE_EEPROM_data_struct(EEPROMData);

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

							if(DATA_READY == *(EEPROMData.isReadyPtr))
							{
								displayDoneFlag = TRUE;
							}
							else
							{
								displayErrorFlag = TRUE;
							}

							error = shortButtonPressDetected_LCD(&LCD_YesNo, LCD_CarSettingsList, &currentLayer, &submenuIterator);
						}

						break;
					}//case /* CarSettings_Layer -> */ClearDiagnosticSnapshots:
					case /* CarSettings_Layer -> */ClearTripMileage:
					{
						if(ENC_button.shortPressDetected)
						{
							CREATE_EEPROM_data_struct(EEPROMData);

							CAR_mileage.tripMileage = 0U;

							EEPROMData.EEPROMParameters = &EEPROM_car;
							EEPROMData.memAddressSize = EEPROM_PAGES_ADDRESS_SIZE;
							EEPROMData.size = UINT32_T_SIZE;

							EEPROMData.memAddress = TRIP_MILEAGE_START_ADDRESS;	//TODO - improve to write in the place we should as there is a table of addresses where the mileage is written!!
							EEPROMData.data = &(CAR_mileage.data[4]);
							xQueueSend(Queue_EEPROM_writeHandle, &EEPROMData, (TickType_t)100U/*100ms wait time if the queue is full*/);

							vTaskDelay((TickType_t)MAX_WAIT_TIME_FOR_EEPROM);	/* wait for xx ms for EEPROM to be able to process that */

							if(DATA_READY == *(EEPROMData.isReadyPtr))
							{
								displayDoneFlag = TRUE;
							}
							else
							{
								displayErrorFlag = TRUE;
							}

							error = shortButtonPressDetected_LCD(&LCD_YesNo, LCD_CarSettingsList, &currentLayer, &submenuIterator);
						}

						break;
					}//case /* CarSettings_Layer -> */ClearTripMileage:
					case /* BoardSettings_Layer -> */ClearErrorsSnapshots:
					{
						if(ENC_button.shortPressDetected)
						{
							CREATE_EEPROM_data_struct(EEPROMData);

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

							if(DATA_READY == *(EEPROMData.isReadyPtr))
							{
								displayDoneFlag = TRUE;
							}
							else
							{
								displayErrorFlag = TRUE;
							}

							error = shortButtonPressDetected_LCD(&LCD_YesNo, LCD_CarSettingsList, &currentLayer, &submenuIterator);
						}

						break;
					}//case /* BoardSettings_Layer -> */ClearErrorsSnapshots:
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
			}//case YesNo_Layer:
			/* *** *** *** *** *** *** *** *** *** *** *** *** *** *** *//* *** *** *** *** *** *** *** *** *** *** *** *** *** *** */
			case Alarm_Layer:
			case Ctrl_Layer:
			default:
			{
				error = LCD__LAYER_CHOICE_FAILURE;
				currentLayer = HOME_SCREEN;
			}
				break;
		}//End of switching between layers



		if(NO_ERROR != error)
		{
			my_error_handler(error);
		}

		if(TRUE == displayErrorFlag)
		{
			error = copy_str_to_buffer("       ERROR!       ", (char*)LCD_buffer[Row4], 0u, 20u);
		}
		if(TRUE == displayDoneFlag)
		{
			error = copy_str_to_buffer("       !DONE!       ", (char*)LCD_buffer[Row4], 0u, 20u);
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

		if((TRUE == displayErrorFlag) || (TRUE == displayDoneFlag))
		{
			vTaskDelay((TickType_t)ERROR_DONE_DISPLAY_TIME);	/* Print "ERROR!" or "!DONE!" for 2 seconds */
			displayErrorFlag = FALSE;
			displayDoneFlag = FALSE;
		}

		vTaskDelayUntil(&xLastWakeTime, xFrequency);
	}
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
}



static Error_Code ControlValueWithEncoder(const LCDBoard* const currentBoard, boolean* displayErrorFlag, boolean* displayDoneFlag, boolean* doneOnce)
{
	Error_Code error = NO_ERROR;
	char tempSettingBuffer[10u] = {' '};

	boolean writeToEEPROM = FALSE;
	CREATE_EEPROM_data_struct(EEPROMData);

	error = copy_str_to_buffer("Enter: save", (char*)LCD_buffer[Row4], 0u, 11u);


	switch(currentBoard->layerPrevious)
	{
		case WaterSettings_Layer:
		{
			static carTemperature_type tempSetting = 0;
			static boolean tempState = 0;
			EEPROMData.EEPROMParameters = &EEPROM_car;
			EEPROMData.memAddressSize = EEPROM_PAGES_ADDRESS_SIZE;

			switch(currentBoard->layer)
			{
				case /* CarSettings_Layer -> WaterSettings_Layer -> */WaterHighTempWarningThreshold:
				{
					error = copy_str_to_buffer("Water High Temp.", (char*)LCD_buffer[Row1], 2u, 16u);
					error = copy_str_to_buffer("Warning Threshold", (char*)LCD_buffer[Row2], 1u, 17u);

					if(FALSE == *doneOnce)
					{
						tempSetting = CAR_waterTemp.waterHighTempWarningThreshold;	//Read the actual value on entry
						*doneOnce = TRUE;
					}

					tempSetting += EncoderCounterDiff;	//Changing the value with encoder
					snprintf(tempSettingBuffer, 7u, "%d %cC", (uint8_t)tempSetting, DEGREE_SYMBOL_LCD);
					error = copy_str_to_buffer(tempSettingBuffer, (char*)LCD_buffer[Row3], 7u, strlen(tempSettingBuffer));

					if(ENC_button.shortPressDetected)
					{
						writeToEEPROM = TRUE;
						CAR_waterTemp.waterHighTempWarningThreshold = tempSetting;

						EEPROMData.size = UINT8_T_SIZE;
						EEPROMData.memAddress = WATER_HIGH_TEMP_WARNING_THRESHOLD_ADDRESS;
						EEPROMData.data = &(CAR_waterTemp.waterHighTempWarningThreshold);
					}

					break;
				}
				case /* CarSettings_Layer -> WaterSettings_Layer -> */WaterHighTempAlarmThreshold:
				{
					error = copy_str_to_buffer("Water High Temp.", (char*)LCD_buffer[Row1], 2u, 16u);
					error = copy_str_to_buffer("Alarm Threshold", (char*)LCD_buffer[Row2], 2u, 15u);

					if(FALSE == *doneOnce)
					{
						tempSetting = CAR_waterTemp.waterHighTempAlarmThreshold;	//Read the actual value on entry
						*doneOnce = TRUE;
					}

					tempSetting += EncoderCounterDiff;	//Changing the value with encoder
					snprintf(tempSettingBuffer, 7u, "%d %cC", (uint8_t)tempSetting, DEGREE_SYMBOL_LCD);
					error = copy_str_to_buffer(tempSettingBuffer, (char*)LCD_buffer[Row3], 7u, strlen(tempSettingBuffer));

					if(ENC_button.shortPressDetected)
					{
						writeToEEPROM = TRUE;
						CAR_waterTemp.waterHighTempAlarmThreshold = tempSetting;

						EEPROMData.size = UINT8_T_SIZE;
						EEPROMData.memAddress = WATER_HIGH_TEMP_ALARM_THRESHOLD_ADDRESS;
						EEPROMData.data = &(CAR_waterTemp.waterHighTempAlarmThreshold);
					}

					break;
				}
				case /* CarSettings_Layer -> WaterSettings_Layer -> */WaterHighTempFanOnThreshold:
				{
					error = copy_str_to_buffer("Water High Temp.", (char*)LCD_buffer[Row1], 2u, 16u);
					error = copy_str_to_buffer("Fan On Threshold", (char*)LCD_buffer[Row2], 2u, 16u);

					if(FALSE == *doneOnce)
					{
						tempSetting = CAR_waterTemp.waterHighTempFanOnThreshold;	//Read the actual value on entry
						*doneOnce = TRUE;
					}

					tempSetting += EncoderCounterDiff;	//Changing the value with encoder
					snprintf(tempSettingBuffer, 7u, "%d %cC", (uint8_t)tempSetting, DEGREE_SYMBOL_LCD);
					error = copy_str_to_buffer(tempSettingBuffer, (char*)LCD_buffer[Row3], 7u, strlen(tempSettingBuffer));

					if(ENC_button.shortPressDetected)
					{
						writeToEEPROM = TRUE;
						CAR_waterTemp.waterHighTempFanOnThreshold = tempSetting;

						EEPROMData.size = UINT8_T_SIZE;
						EEPROMData.memAddress = WATER_HIGH_TEMP_FAN_ON_THRESHOLD_ADDRESS;
						EEPROMData.data = &(CAR_waterTemp.waterHighTempFanOnThreshold);
					}

					break;
				}
				case /* CarSettings_Layer -> WaterSettings_Layer -> */WaterHighTempFanOffThreshold:
				{
					error = copy_str_to_buffer("Water High Temp.", (char*)LCD_buffer[Row1], 2u, 16u);
					error = copy_str_to_buffer("Fan Off Threshold", (char*)LCD_buffer[Row2], 1u, 17u);

					if(FALSE == *doneOnce)
					{
						tempSetting = CAR_waterTemp.waterHighTempFanOffThreshold;	//Read the actual value on entry
						*doneOnce = TRUE;
					}

					tempSetting += EncoderCounterDiff;	//Changing the value with encoder
					snprintf(tempSettingBuffer, 7u, "%d %cC", (uint8_t)tempSetting, DEGREE_SYMBOL_LCD);
					error = copy_str_to_buffer(tempSettingBuffer, (char*)LCD_buffer[Row3], 7u, strlen(tempSettingBuffer));

					if(ENC_button.shortPressDetected)
					{
						writeToEEPROM = TRUE;
						CAR_waterTemp.waterHighTempFanOffThreshold = tempSetting;

						EEPROMData.size = UINT8_T_SIZE;
						EEPROMData.memAddress = WATER_HIGH_TEMP_FAN_OFF_THRESHOLD_ADDRESS;
						EEPROMData.data = &(CAR_waterTemp.waterHighTempFanOffThreshold);
					}

					break;
				}
				case /* CarSettings_Layer -> WaterSettings_Layer -> */WaterTempWarningOn:
				{
					error = copy_str_to_buffer("Water High Temp.", (char*)LCD_buffer[Row1], 2u, 16u);
					error = copy_str_to_buffer("Warning On/Off", (char*)LCD_buffer[Row2], 3u, 14u);

					if(FALSE == *doneOnce)
					{
						tempState = CAR_waterTemp.waterTempWarningOn;	//Read the actual value on entry
						*doneOnce = TRUE;
					}

					if(0 != EncoderCounterDiff)
					{
						tempState ^= 0b1;	//Toggling the bit with encoder
					}

					error = copy_str_to_buffer((tempState ? "ON " : "OFF"), (char*)LCD_buffer[Row3], 9u, 3u);

					if(ENC_button.shortPressDetected)
					{
						writeToEEPROM = TRUE;
						CAR_waterTemp.waterTempWarningOn = tempState;

						EEPROMData.size = UINT8_T_SIZE;
						EEPROMData.memAddress = WATER_TEMP_ALL_SETTINGS_ADDRESS;
						EEPROMData.data = &(CAR_waterTemp.allSettings);
					}

					break;
				}
				case /* CarSettings_Layer -> WaterSettings_Layer -> */WaterTempAlarmOn:
				{
					error = copy_str_to_buffer("Water High Temp.", (char*)LCD_buffer[Row1], 2u, 16u);
					error = copy_str_to_buffer("Alarm On/Off", (char*)LCD_buffer[Row2], 4u, 12u);

					if(FALSE == *doneOnce)
					{
						tempState = CAR_waterTemp.waterTempAlarmOn;	//Read the actual value on entry
						*doneOnce = TRUE;
					}

					if(0 != EncoderCounterDiff)
					{
						tempState ^= 0b1;	//Toggling the bit with encoder
					}

					error = copy_str_to_buffer((tempState ? "ON " : "OFF"), (char*)LCD_buffer[Row3], 9u, 3u);

					if(ENC_button.shortPressDetected)
					{
						writeToEEPROM = TRUE;
						CAR_waterTemp.waterTempAlarmOn = tempState;

						EEPROMData.size = UINT8_T_SIZE;
						EEPROMData.memAddress = WATER_TEMP_ALL_SETTINGS_ADDRESS;
						EEPROMData.data = &(CAR_waterTemp.allSettings);
					}

					break;
				}
				case /* CarSettings_Layer -> WaterSettings_Layer -> */WaterFanControlOn:
				{
					error = copy_str_to_buffer("Water High Temp.", (char*)LCD_buffer[Row1], 2u, 16u);
					error = copy_str_to_buffer("Fan Control On/Off", (char*)LCD_buffer[Row2], 1u, 18u);

					if(FALSE == *doneOnce)
					{
						tempState = CAR_waterTemp.waterFanControlOn;	//Read the actual value on entry
						*doneOnce = TRUE;
					}

					if(0 != EncoderCounterDiff)
					{
						tempState ^= 0b1;	//Toggling the bit with encoder
					}

					error = copy_str_to_buffer((tempState ? "ON " : "OFF"), (char*)LCD_buffer[Row3], 9u, 3u);

					if(ENC_button.shortPressDetected)
					{
						writeToEEPROM = TRUE;
						CAR_waterTemp.waterFanControlOn = tempState;

						EEPROMData.size = UINT8_T_SIZE;
						EEPROMData.memAddress = WATER_TEMP_ALL_SETTINGS_ADDRESS;
						EEPROMData.data = &(CAR_waterTemp.allSettings);
					}

					break;
				}
				case /* CarSettings_Layer -> WaterSettings_Layer -> */WaterTempWarningBuzzerOn:
				{
					error = copy_str_to_buffer("Water High Temp.", (char*)LCD_buffer[Row1], 2u, 16u);
					error = copy_str_to_buffer("Warning Buzzer", (char*)LCD_buffer[Row2], 3u, 14u);

					if(FALSE == *doneOnce)
					{
						tempState = CAR_waterTemp.waterTempWarningBuzzerOn;	//Read the actual value on entry
						*doneOnce = TRUE;
					}

					if(0 != EncoderCounterDiff)
					{
						tempState ^= 0b1;	//Toggling the bit with encoder
					}

					error = copy_str_to_buffer((tempState ? "ON " : "OFF"), (char*)LCD_buffer[Row3], 9u, 3u);

					if(ENC_button.shortPressDetected)
					{
						writeToEEPROM = TRUE;
						CAR_waterTemp.waterTempWarningBuzzerOn = tempState;

						EEPROMData.size = UINT8_T_SIZE;
						EEPROMData.memAddress = WATER_TEMP_ALL_SETTINGS_ADDRESS;
						EEPROMData.data = &(CAR_waterTemp.allSettings);
					}

					break;
				}
				case /* CarSettings_Layer -> WaterSettings_Layer -> */WaterTempAlarmBuzzerOn:
				{
					error = copy_str_to_buffer("Water High Temp.", (char*)LCD_buffer[Row1], 2u, 16u);
					error = copy_str_to_buffer("Alarm Buzzer", (char*)LCD_buffer[Row2], 4u, 12u);

					if(FALSE == *doneOnce)
					{
						tempState = CAR_waterTemp.waterTempAlarmBuzzerOn;	//Read the actual value on entry
						*doneOnce = TRUE;
					}

					if(0 != EncoderCounterDiff)
					{
						tempState ^= 0b1;	//Toggling the bit with encoder
					}

					error = copy_str_to_buffer((tempState ? "ON " : "OFF"), (char*)LCD_buffer[Row3], 9u, 3u);

					if(ENC_button.shortPressDetected)
					{
						writeToEEPROM = TRUE;
						CAR_waterTemp.waterTempAlarmBuzzerOn = tempState;

						EEPROMData.size = UINT8_T_SIZE;
						EEPROMData.memAddress = WATER_TEMP_ALL_SETTINGS_ADDRESS;
						EEPROMData.data = &(CAR_waterTemp.allSettings);
					}

					break;
				}
				case /* CarSettings_Layer -> WaterSettings_Layer -> */WaterTempWarningSnapshotOn:
				{
					error = copy_str_to_buffer("Water High Temp.", (char*)LCD_buffer[Row1], 2u, 16u);
					error = copy_str_to_buffer("Warning Snapshot", (char*)LCD_buffer[Row2], 2u, 16u);

					if(FALSE == *doneOnce)
					{
						tempState = CAR_waterTemp.waterTempWarningSnapshotOn;	//Read the actual value on entry
						*doneOnce = TRUE;
					}

					if(0 != EncoderCounterDiff)
					{
						tempState ^= 0b1;	//Toggling the bit with encoder
					}

					error = copy_str_to_buffer((tempState ? "ON " : "OFF"), (char*)LCD_buffer[Row3], 9u, 3u);

					if(ENC_button.shortPressDetected)
					{
						writeToEEPROM = TRUE;
						CAR_waterTemp.waterTempWarningSnapshotOn = tempState;

						EEPROMData.size = UINT8_T_SIZE;
						EEPROMData.memAddress = WATER_TEMP_ALL_SETTINGS_ADDRESS;
						EEPROMData.data = &(CAR_waterTemp.allSettings);
					}

					break;
				}
				case /* CarSettings_Layer -> WaterSettings_Layer -> */WaterTempAlarmSnapshotOn:
				{
					error = copy_str_to_buffer("Water High Temp.", (char*)LCD_buffer[Row1], 2u, 16u);
					error = copy_str_to_buffer("Alarm Snapshot", (char*)LCD_buffer[Row2], 3u, 14u);

					if(FALSE == *doneOnce)
					{
						tempState = CAR_waterTemp.waterTempAlarmSnapshotOn;	//Read the actual value on entry
						*doneOnce = TRUE;
					}

					if(0 != EncoderCounterDiff)
					{
						tempState ^= 0b1;	//Toggling the bit with encoder
					}

					error = copy_str_to_buffer((tempState ? "ON " : "OFF"), (char*)LCD_buffer[Row3], 9u, 3u);

					if(ENC_button.shortPressDetected)
					{
						writeToEEPROM = TRUE;
						CAR_waterTemp.waterTempAlarmSnapshotOn = tempState;

						EEPROMData.size = UINT8_T_SIZE;
						EEPROMData.memAddress = WATER_TEMP_ALL_SETTINGS_ADDRESS;
						EEPROMData.data = &(CAR_waterTemp.allSettings);
					}

					break;
				}
				default:
				{
					error = LCD__LAYER_CHOICE_FAILURE;
					break;
				}
			}//switch(currentBoard->layer)
			break;
		}//case WaterSettings_Layer:
		/* *** *** *** *** *** *** *** *** *** *** *** *** *** *** *//* *** *** *** *** *** *** *** *** *** *** *** *** *** *** */

		case OilTempSettings_Layer:
		{
			static carTemperature_type tempSetting = 0;
			static boolean tempState = 0;
			EEPROMData.EEPROMParameters = &EEPROM_car;
			EEPROMData.memAddressSize = EEPROM_PAGES_ADDRESS_SIZE;

			switch(currentBoard->layer)
			{
				case /* CarSettings_Layer -> OilTempSettings_Layer -> */OilHighTempWarningThreshold:
				{
					error = copy_str_to_buffer("Oil High Temperature", (char*)LCD_buffer[Row1], 0u, 20u);
					error = copy_str_to_buffer("Warning Threshold", (char*)LCD_buffer[Row2], 1u, 17u);

					if(FALSE == *doneOnce)
					{
						tempSetting = CAR_oilTemp.oilHighTempWarningThreshold;	//Read the actual value on entry
						*doneOnce = TRUE;
					}

					tempSetting += EncoderCounterDiff;	//Changing the value with encoder
					snprintf(tempSettingBuffer, 7u, "%d %cC", (uint8_t)tempSetting, DEGREE_SYMBOL_LCD);
					error = copy_str_to_buffer(tempSettingBuffer, (char*)LCD_buffer[Row3], 7u, strlen(tempSettingBuffer));

					if(ENC_button.shortPressDetected)
					{
						writeToEEPROM = TRUE;
						CAR_oilTemp.oilHighTempWarningThreshold = tempSetting;

						EEPROMData.size = UINT8_T_SIZE;
						EEPROMData.memAddress = OIL_HIGH_TEMP_WARNING_THRESHOLD_ADDRESS;
						EEPROMData.data = &(CAR_oilTemp.oilHighTempWarningThreshold);
					}

					break;
				}
				case /* CarSettings_Layer -> OilTempSettings_Layer -> */OilHighTempAlarmThreshold:
				{
					error = copy_str_to_buffer("Oil High Temperature", (char*)LCD_buffer[Row1], 0u, 20u);
					error = copy_str_to_buffer("Alarm Threshold", (char*)LCD_buffer[Row2], 2u, 15u);

					if(FALSE == *doneOnce)
					{
						tempSetting = CAR_oilTemp.oilHighTempAlarmThreshold;	//Read the actual value on entry
						*doneOnce = TRUE;
					}

					tempSetting += EncoderCounterDiff;	//Changing the value with encoder
					snprintf(tempSettingBuffer, 7u, "%d %cC", (uint8_t)tempSetting, DEGREE_SYMBOL_LCD);
					error = copy_str_to_buffer(tempSettingBuffer, (char*)LCD_buffer[Row3], 7u, strlen(tempSettingBuffer));

					if(ENC_button.shortPressDetected)
					{
						writeToEEPROM = TRUE;
						CAR_oilTemp.oilHighTempAlarmThreshold = tempSetting;

						EEPROMData.size = UINT8_T_SIZE;
						EEPROMData.memAddress = OIL_HIGH_TEMP_ALARM_THRESHOLD_ADDRESS;
						EEPROMData.data = &(CAR_oilTemp.oilHighTempAlarmThreshold);
					}

					break;
				}
				case /* CarSettings_Layer -> OilTempSettings_Layer -> */OilTempWarningOn:
				{
					error = copy_str_to_buffer("Oil High Temperature", (char*)LCD_buffer[Row1], 0u, 20u);
					error = copy_str_to_buffer("Warning On/Off", (char*)LCD_buffer[Row2], 3u, 14u);

					if(FALSE == *doneOnce)
					{
						tempState = CAR_oilTemp.oilTempWarningOn;	//Read the actual value on entry
						*doneOnce = TRUE;
					}

					if(0 != EncoderCounterDiff)
					{
						tempState ^= 0b1;	//Toggling the bit with encoder
					}

					error = copy_str_to_buffer((tempState ? "ON " : "OFF"), (char*)LCD_buffer[Row3], 9u, 3u);

					if(ENC_button.shortPressDetected)
					{
						writeToEEPROM = TRUE;
						CAR_oilTemp.oilTempWarningOn = tempState;

						EEPROMData.size = UINT8_T_SIZE;
						EEPROMData.memAddress = OIL_TEMP_ALL_SETTINGS_ADDRESS;
						EEPROMData.data = &(CAR_oilTemp.allSettings);
					}

					break;
				}
				case /* CarSettings_Layer -> OilTempSettings_Layer -> */OilTempAlarmOn:
				{
					error = copy_str_to_buffer("Oil High Temperature", (char*)LCD_buffer[Row1], 0u, 20u);
					error = copy_str_to_buffer("Alarm On/Off", (char*)LCD_buffer[Row2], 4u, 12u);

					if(FALSE == *doneOnce)
					{
						tempState = CAR_oilTemp.oilTempAlarmOn;	//Read the actual value on entry
						*doneOnce = TRUE;
					}

					if(0 != EncoderCounterDiff)
					{
						tempState ^= 0b1;	//Toggling the bit with encoder
					}

					error = copy_str_to_buffer((tempState ? "ON " : "OFF"), (char*)LCD_buffer[Row3], 9u, 3u);

					if(ENC_button.shortPressDetected)
					{
						writeToEEPROM = TRUE;
						CAR_oilTemp.oilTempAlarmOn = tempState;

						EEPROMData.size = UINT8_T_SIZE;
						EEPROMData.memAddress = OIL_TEMP_ALL_SETTINGS_ADDRESS;
						EEPROMData.data = &(CAR_oilTemp.allSettings);
					}

					break;
				}
				case /* CarSettings_Layer -> OilTempSettings_Layer -> */OilTempWarningBuzzerOn:
				{
					error = copy_str_to_buffer("Oil High Temperature", (char*)LCD_buffer[Row1], 0u, 20u);
					error = copy_str_to_buffer("Warning Buzzer", (char*)LCD_buffer[Row2], 3u, 14u);

					if(FALSE == *doneOnce)
					{
						tempState = CAR_oilTemp.oilTempWarningBuzzerOn;	//Read the actual value on entry
						*doneOnce = TRUE;
					}

					if(0 != EncoderCounterDiff)
					{
						tempState ^= 0b1;	//Toggling the bit with encoder
					}

					error = copy_str_to_buffer((tempState ? "ON " : "OFF"), (char*)LCD_buffer[Row3], 9u, 3u);

					if(ENC_button.shortPressDetected)
					{
						writeToEEPROM = TRUE;
						CAR_oilTemp.oilTempWarningBuzzerOn = tempState;

						EEPROMData.size = UINT8_T_SIZE;
						EEPROMData.memAddress = OIL_TEMP_ALL_SETTINGS_ADDRESS;
						EEPROMData.data = &(CAR_oilTemp.allSettings);
					}

					break;
				}
				case /* CarSettings_Layer -> OilTempSettings_Layer -> */OilTempAlarmBuzzerOn:
				{
					error = copy_str_to_buffer("Oil High Temperature", (char*)LCD_buffer[Row1], 0u, 20u);
					error = copy_str_to_buffer("Alarm Buzzer", (char*)LCD_buffer[Row2], 4u, 12u);

					if(FALSE == *doneOnce)
					{
						tempState = CAR_oilTemp.oilTempAlarmBuzzerOn;	//Read the actual value on entry
						*doneOnce = TRUE;
					}

					if(0 != EncoderCounterDiff)
					{
						tempState ^= 0b1;	//Toggling the bit with encoder
					}

					error = copy_str_to_buffer((tempState ? "ON " : "OFF"), (char*)LCD_buffer[Row3], 9u, 3u);

					if(ENC_button.shortPressDetected)
					{
						writeToEEPROM = TRUE;
						CAR_oilTemp.oilTempAlarmBuzzerOn = tempState;

						EEPROMData.size = UINT8_T_SIZE;
						EEPROMData.memAddress = OIL_TEMP_ALL_SETTINGS_ADDRESS;
						EEPROMData.data = &(CAR_oilTemp.allSettings);
					}

					break;
				}
				case /* CarSettings_Layer -> OilTempSettings_Layer -> */OilTempWarningSnapshotOn:
				{
					error = copy_str_to_buffer("Oil High Temperature", (char*)LCD_buffer[Row1], 0u, 20u);
					error = copy_str_to_buffer("Warning Snapshot", (char*)LCD_buffer[Row2], 2u, 16u);

					if(FALSE == *doneOnce)
					{
						tempState = CAR_oilTemp.oilTempWarningSnapshotOn;	//Read the actual value on entry
						*doneOnce = TRUE;
					}

					if(0 != EncoderCounterDiff)
					{
						tempState ^= 0b1;	//Toggling the bit with encoder
					}

					error = copy_str_to_buffer((tempState ? "ON " : "OFF"), (char*)LCD_buffer[Row3], 9u, 3u);

					if(ENC_button.shortPressDetected)
					{
						writeToEEPROM = TRUE;
						CAR_oilTemp.oilTempWarningSnapshotOn = tempState;

						EEPROMData.size = UINT8_T_SIZE;
						EEPROMData.memAddress = OIL_TEMP_ALL_SETTINGS_ADDRESS;
						EEPROMData.data = &(CAR_oilTemp.allSettings);
					}

					break;
				}
				case /* CarSettings_Layer -> OilTempSettings_Layer -> */OilTempAlarmSpashotOn:
				{
					error = copy_str_to_buffer("Oil High Temperature", (char*)LCD_buffer[Row1], 0u, 20u);
					error = copy_str_to_buffer("Alarm Snapshot", (char*)LCD_buffer[Row2], 3u, 14u);

					if(FALSE == *doneOnce)
					{
						tempState = CAR_oilTemp.oilTempAlarmSnapshotOn;	//Read the actual value on entry
						*doneOnce = TRUE;
					}

					if(0 != EncoderCounterDiff)
					{
						tempState ^= 0b1;	//Toggling the bit with encoder
					}

					error = copy_str_to_buffer((tempState ? "ON " : "OFF"), (char*)LCD_buffer[Row3], 9u, 3u);

					if(ENC_button.shortPressDetected)
					{
						writeToEEPROM = TRUE;
						CAR_oilTemp.oilTempAlarmSnapshotOn = tempState;

						EEPROMData.size = UINT8_T_SIZE;
						EEPROMData.memAddress = OIL_TEMP_ALL_SETTINGS_ADDRESS;
						EEPROMData.data = &(CAR_oilTemp.allSettings);
					}

					break;
				}
				default:
				{
					error = LCD__LAYER_CHOICE_FAILURE;
					break;
				}
			}//switch(currentBoard->layer)
			break;
		}//case OilTempSettings_Layer:
		/* *** *** *** *** *** *** *** *** *** *** *** *** *** *** *//* *** *** *** *** *** *** *** *** *** *** *** *** *** *** */

		case OilPressureSettings_Layer:
		{
			static carOilPressure_type tempSetting = 0;
			static boolean tempState = 0;
			EEPROMData.EEPROMParameters = &EEPROM_car;
			EEPROMData.memAddressSize = EEPROM_PAGES_ADDRESS_SIZE;

			switch(currentBoard->layer)
			{
				case /* CarSettings_Layer -> OilPressureSettings_Layer -> */OilHighPressureAlarmThreshold:
				{
					error = copy_str_to_buffer("Oil High Pressure", (char*)LCD_buffer[Row1], 1u, 17u);
					error = copy_str_to_buffer("Alarm Threshold", (char*)LCD_buffer[Row2], 2u, 15u);

					if(FALSE == *doneOnce)
					{
						tempSetting = CAR_oilPressure.oilHighPressureAlarmThreshold;	//Read the actual value on entry
						*doneOnce = TRUE;
					}

					tempSetting += EncoderCounterDiff;	//Changing the value with encoder
					snprintf(tempSettingBuffer, 8u, "%d psi", (uint8_t)tempSetting);
					error = copy_str_to_buffer(tempSettingBuffer, (char*)LCD_buffer[Row3], 7u, strlen(tempSettingBuffer));

					if(ENC_button.shortPressDetected)
					{
						writeToEEPROM = TRUE;
						CAR_oilPressure.oilHighPressureAlarmThreshold = tempSetting;

						EEPROMData.size = UINT8_T_SIZE;
						EEPROMData.memAddress = OIL_HIGH_PRESSURE_ALARM_THRESHOLD_ADDRESS;
						EEPROMData.data = &(CAR_oilPressure.oilHighPressureAlarmThreshold);
					}

					break;
				}
				case /* CarSettings_Layer -> OilPressureSettings_Layer -> */OilLowPressureAlarmThreshold:
				{
					error = copy_str_to_buffer("Oil Low Pressure", (char*)LCD_buffer[Row1], 2u, 16u);
					error = copy_str_to_buffer("Alarm Threshold", (char*)LCD_buffer[Row2], 2u, 15u);

					if(FALSE == *doneOnce)
					{
						tempSetting = CAR_oilPressure.oilLowPressureAlarmThreshold;	//Read the actual value on entry
						*doneOnce = TRUE;
					}

					tempSetting += EncoderCounterDiff;	//Changing the value with encoder
					snprintf(tempSettingBuffer, 8u, "%d psi", (uint8_t)tempSetting);
					error = copy_str_to_buffer(tempSettingBuffer, (char*)LCD_buffer[Row3], 7u, strlen(tempSettingBuffer));

					if(ENC_button.shortPressDetected)
					{
						writeToEEPROM = TRUE;
						CAR_oilPressure.oilLowPressureAlarmThreshold = tempSetting;

						EEPROMData.size = UINT8_T_SIZE;
						EEPROMData.memAddress = OIL_LOW_PRESSURE_ALARM_THRESHOLD_ADDRESS;
						EEPROMData.data = &(CAR_oilPressure.oilLowPressureAlarmThreshold);
					}

					break;
				}
				case /* CarSettings_Layer -> OilPressureSettings_Layer -> */OilPressureAnalogMeasurement:
				{
					error = copy_str_to_buffer("Oil Pressure", (char*)LCD_buffer[Row1], 4u, 12u);
					error = copy_str_to_buffer("Analog Measurement", (char*)LCD_buffer[Row2], 1u, 18u);

					if(FALSE == *doneOnce)
					{
						tempState = CAR_oilPressure.oilPressureAnalogMeasurement;	//Read the actual value on entry
						*doneOnce = TRUE;
					}

					if(0 != EncoderCounterDiff)
					{
						tempState ^= 0b1;	//Toggling the bit with encoder
					}

					error = copy_str_to_buffer((tempState ? "ON " : "OFF"), (char*)LCD_buffer[Row3], 9u, 3u);

					if(ENC_button.shortPressDetected)
					{
						writeToEEPROM = TRUE;
						CAR_oilPressure.oilPressureAnalogMeasurement = tempState;

						EEPROMData.size = UINT8_T_SIZE;
						EEPROMData.memAddress = OIL_PRESSURE_ALL_SETTINGS_ADDRESS;
						EEPROMData.data = &(CAR_oilPressure.allSettings);
					}

					break;
				}
				case /* CarSettings_Layer -> OilPressureSettings_Layer -> */OilHighPressureAlarmOn:
				{
					error = copy_str_to_buffer("Oil High Pressure", (char*)LCD_buffer[Row1], 1u, 17u);
					error = copy_str_to_buffer("Alarm On/Off", (char*)LCD_buffer[Row2], 4u, 12u);

					if(FALSE == *doneOnce)
					{
						tempState = CAR_oilPressure.oilHighPressureAlarmOn;	//Read the actual value on entry
						*doneOnce = TRUE;
					}

					if(0 != EncoderCounterDiff)
					{
						tempState ^= 0b1;	//Toggling the bit with encoder
					}

					error = copy_str_to_buffer((tempState ? "ON " : "OFF"), (char*)LCD_buffer[Row3], 9u, 3u);

					if(ENC_button.shortPressDetected)
					{
						writeToEEPROM = TRUE;
						CAR_oilPressure.oilHighPressureAlarmOn = tempState;

						EEPROMData.size = UINT8_T_SIZE;
						EEPROMData.memAddress = OIL_PRESSURE_ALL_SETTINGS_ADDRESS;
						EEPROMData.data = &(CAR_oilPressure.allSettings);
					}

					break;
				}
				case /* CarSettings_Layer -> OilPressureSettings_Layer -> */OilLowPressureAlarmOn:
				{
					error = copy_str_to_buffer("Oil Low Pressure", (char*)LCD_buffer[Row1], 2u, 16u);
					error = copy_str_to_buffer("Alarm On/Off", (char*)LCD_buffer[Row2], 4u, 12u);

					if(FALSE == *doneOnce)
					{
						tempState = CAR_oilPressure.oilLowPressureAlarmOn;	//Read the actual value on entry
						*doneOnce = TRUE;
					}

					if(0 != EncoderCounterDiff)
					{
						tempState ^= 0b1;	//Toggling the bit with encoder
					}

					error = copy_str_to_buffer((tempState ? "ON " : "OFF"), (char*)LCD_buffer[Row3], 9u, 3u);

					if(ENC_button.shortPressDetected)
					{
						writeToEEPROM = TRUE;
						CAR_oilPressure.oilLowPressureAlarmOn = tempState;

						EEPROMData.size = UINT8_T_SIZE;
						EEPROMData.memAddress = OIL_PRESSURE_ALL_SETTINGS_ADDRESS;
						EEPROMData.data = &(CAR_oilPressure.allSettings);
					}

					break;
				}
				case /* CarSettings_Layer -> OilPressureSettings_Layer -> */OilPressureAlarmBuzzerOn:
				{
					error = copy_str_to_buffer("Oil Pressure", (char*)LCD_buffer[Row1], 4u, 12u);
					error = copy_str_to_buffer("Alarm Buzzer", (char*)LCD_buffer[Row2], 4u, 12u);

					if(FALSE == *doneOnce)
					{
						tempState = CAR_oilPressure.oilPressureAlarmBuzzerOn;	//Read the actual value on entry
						*doneOnce = TRUE;
					}

					if(0 != EncoderCounterDiff)
					{
						tempState ^= 0b1;	//Toggling the bit with encoder
					}

					error = copy_str_to_buffer((tempState ? "ON " : "OFF"), (char*)LCD_buffer[Row3], 9u, 3u);

					if(ENC_button.shortPressDetected)
					{
						writeToEEPROM = TRUE;
						CAR_oilPressure.oilPressureAlarmBuzzerOn = tempState;

						EEPROMData.size = UINT8_T_SIZE;
						EEPROMData.memAddress = OIL_PRESSURE_ALL_SETTINGS_ADDRESS;
						EEPROMData.data = &(CAR_oilPressure.allSettings);
					}

					break;
				}
				case /* CarSettings_Layer -> OilPressureSettings_Layer -> */OilPressureAlarmSnapshotOn:
				{
					error = copy_str_to_buffer("Oil Pressure", (char*)LCD_buffer[Row1], 4u, 12u);
					error = copy_str_to_buffer("Alarm Snapshot", (char*)LCD_buffer[Row2], 3u, 14u);

					if(FALSE == *doneOnce)
					{
						tempState = CAR_oilPressure.oilPressureAlarmSnapshotOn;	//Read the actual value on entry
						*doneOnce = TRUE;
					}

					if(0 != EncoderCounterDiff)
					{
						tempState ^= 0b1;	//Toggling the bit with encoder
					}

					error = copy_str_to_buffer((tempState ? "ON " : "OFF"), (char*)LCD_buffer[Row3], 9u, 3u);

					if(ENC_button.shortPressDetected)
					{
						writeToEEPROM = TRUE;
						CAR_oilPressure.oilPressureAlarmSnapshotOn = tempState;

						EEPROMData.size = UINT8_T_SIZE;
						EEPROMData.memAddress = OIL_PRESSURE_ALL_SETTINGS_ADDRESS;
						EEPROMData.data = &(CAR_oilPressure.allSettings);
					}

					break;
				}
				default:
				{
					error = LCD__LAYER_CHOICE_FAILURE;
					break;
				}
			}//switch(currentBoard->layer)
			break;
		}//case OilPressureSettings_Layer:
		/* *** *** *** *** *** *** *** *** *** *** *** *** *** *** *//* *** *** *** *** *** *** *** *** *** *** *** *** *** *** */


		case FuelSettings_Layer:
		{
			static cafFuelLevel_type tempSetting = 0;
			static boolean tempState = 0;
			EEPROMData.EEPROMParameters = &EEPROM_car;
			EEPROMData.memAddressSize = EEPROM_PAGES_ADDRESS_SIZE;

			switch(currentBoard->layer)
			{
				case /* CarSettings_Layer -> FuelSettings_Layer -> */FuelLowLevelWarningThreshold:
				{
					error = copy_str_to_buffer("Low Fuel Level", (char*)LCD_buffer[Row1], 3u, 14u);
					error = copy_str_to_buffer("Warning Threshold", (char*)LCD_buffer[Row2], 1u, 17u);

					if(FALSE == *doneOnce)
					{
						tempSetting = CAR_fuel.fuelLowLevelWarningThreshold;	//Read the actual value on entry
						*doneOnce = TRUE;
					}

					tempSetting += EncoderCounterDiff;	//Changing the value with encoder
					snprintf(tempSettingBuffer, 11u, "%d liters", (uint8_t)tempSetting);
					error = copy_str_to_buffer(tempSettingBuffer, (char*)LCD_buffer[Row3], 6u, strlen(tempSettingBuffer));

					if(ENC_button.shortPressDetected)
					{
						writeToEEPROM = TRUE;
						CAR_fuel.fuelLowLevelWarningThreshold = tempSetting;

						EEPROMData.size = UINT8_T_SIZE;
						EEPROMData.memAddress = FUEL_LOW_LEVEL_WARNING_THRESHOLD_ADDRESS;
						EEPROMData.data = &(CAR_fuel.fuelLowLevelWarningThreshold);
					}

					break;
				}
				case /* CarSettings_Layer -> FuelSettings_Layer -> */FuelLowLevelWarningOn:
				{
					error = copy_str_to_buffer("Low Fuel Level", (char*)LCD_buffer[Row1], 3u, 14u);
					error = copy_str_to_buffer("Warning On/Off", (char*)LCD_buffer[Row2], 3u, 14u);

					if(FALSE == *doneOnce)
					{
						tempState = CAR_fuel.lowFuelLevelWarningOn;	//Read the actual value on entry
						*doneOnce = TRUE;
					}

					if(0 != EncoderCounterDiff)
					{
						tempState ^= 0b1;	//Toggling the bit with encoder
					}

					error = copy_str_to_buffer((tempState ? "ON " : "OFF"), (char*)LCD_buffer[Row3], 9u, 3u);

					if(ENC_button.shortPressDetected)
					{
						writeToEEPROM = TRUE;
						CAR_fuel.lowFuelLevelWarningOn = tempState;

						EEPROMData.size = UINT8_T_SIZE;
						EEPROMData.memAddress = FUEL_ALL_SETTINGS_ADDRESS;
						EEPROMData.data = &(CAR_fuel.allSettings);
					}

					break;
				}
				case /* CarSettings_Layer -> FuelSettings_Layer -> */FuelLowLevelWarningBuzzerOn:
				{
					error = copy_str_to_buffer("Low Fuel Level", (char*)LCD_buffer[Row1], 3u, 14u);
					error = copy_str_to_buffer("Warning Buzzer", (char*)LCD_buffer[Row2], 3u, 14u);

					if(FALSE == *doneOnce)
					{
						tempState = CAR_fuel.lowFuelLevelWarningBuzzerOn;	//Read the actual value on entry
						*doneOnce = TRUE;
					}

					if(0 != EncoderCounterDiff)
					{
						tempState ^= 0b1;	//Toggling the bit with encoder
					}

					error = copy_str_to_buffer((tempState ? "ON " : "OFF"), (char*)LCD_buffer[Row3], 9u, 3u);

					if(ENC_button.shortPressDetected)
					{
						writeToEEPROM = TRUE;
						CAR_fuel.lowFuelLevelWarningBuzzerOn = tempState;

						EEPROMData.size = UINT8_T_SIZE;
						EEPROMData.memAddress = FUEL_ALL_SETTINGS_ADDRESS;
						EEPROMData.data = &(CAR_fuel.allSettings);
					}

					break;
				}
				default:
				{
					error = LCD__LAYER_CHOICE_FAILURE;
					break;
				}
			}//switch(currentBoard->layer)
			break;
		}//case FuelSettings_Layer:
		/* *** *** *** *** *** *** *** *** *** *** *** *** *** *** *//* *** *** *** *** *** *** *** *** *** *** *** *** *** *** */


		default:
		{
			break;
		}
	}//switch(currentBoard->layerPrevious)


	if(TRUE == writeToEEPROM)	/* Parameters are set in the switch aboe */
	{
		xQueueSend(Queue_EEPROM_writeHandle, &EEPROMData, (TickType_t)100U/*100ms wait time if the queue is full*/);

		vTaskDelay((TickType_t)MAX_WAIT_TIME_FOR_EEPROM);	/* wait for xx ms for EEPROM to be able to process that */

		if(DATA_READY == (*EEPROMData.isReadyPtr))
		{
			*displayDoneFlag = TRUE;
		}
		else
		{
			*displayErrorFlag = TRUE;
		}
	}

	EncoderCounterDiff = 0;	//Set to 0 after using the value
	return error;
}




