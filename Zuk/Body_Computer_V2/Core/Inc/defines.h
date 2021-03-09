/*
 * defines.h
 *
 *  Created on: Jan 6, 2021
 *      Author: Wojciech Grzelinski
 */

#ifndef INC_DEFINES_H_
#define INC_DEFINES_H_


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
typedef uint8_t boolean;
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#define RUNTIME_STATS_AND_DEBUG

#ifdef RUNTIME_STATS_AND_DEBUG
#define RUNTIME_STATS_TIMER_CONFIG
#define RUNTIME_STATS_QUEUES
#endif
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* TASKS & TIMERS FREQUENCIES */
#define MY_LCD_TASK_TIME_PERIOD				(TickType_t)(200)	/* MINIMUM: 100, for the 4x20 LCD it is needed approximately 80-85ms to send all data */
#define MY_GPS_TASK_TIME_PERIOD				(TickType_t)(1000)
#define MY_EEPROM_TASK_TIME_PERIOD			(TickType_t)(200)	/* Would be good to decrease this number significantly, as it only executes when it has data from a Queue */
#define MY_DUMP_TO_EEPROM_TASK_TIME_PERIOD 	(TickType_t)(1000)
#define MY_DUMP_TO_SDCARD_TASK_TIME_PERIOD	(TickType_t)(1000)
#define MY_TASK_50_MS_TIME_PERIOD			(TickType_t)(50)
#define MY_TASK_MEASURE_TIME_PERIOD			(TickType_t)(250)
#define MY_TASK_500_MS_TIME_PERIOD			(TickType_t)(500)
#define MY_TASK_1000_MS_TIME_PERIOD			(TickType_t)(1000)
#define MY_DIAG_CHECK_TASK_TIME_PERIOD		(TickType_t)(1000)
#define MY_ALARM_CONTROL_TASK_TIME_PERIOD	(TickType_t)(250)

#define DEBOUNCING_TIME_FOR_ENCODER_BUTTON	((uint32_t)(50U))	/* Value in milliseconds */

#define ENC_BUTTON_LONG_PRESS_TIME						((uint32_t)(1000U))		/* Value in milliseconds */
#define CAR_WATER_TEMP_VALUE_CHECK_TIMER_TIME			((TickType_t)(4000))	/* Value in milliseconds */	/* 4 seconds  */
#define CAR_OIL_TEMP_VALUE_CHECK_TIMER_TIME				((TickType_t)(4000))	/* Value in milliseconds */	/* 4 seconds  */
#define CAR_OIL_ANALOG_PRESSURE_VALUE_CHECK_TIMER_TIME	((TickType_t)(3000))	/* Value in milliseconds */	/* 3 seconds  */
#define CAR_OIL_BINARY_PRESSURE_VALUE_CHECK_TIMER_TIME	((TickType_t)(3000))	/* Value in milliseconds */	/* 3 seconds  */
#define CAR_MAIN_BATT_VOLTAGE_VALUE_CHECK_TIMER_TIME	((TickType_t)(5000))	/* Value in milliseconds */	/* 5 seconds  */
#define CAR_AUX_BATT_VOLTAGE_VALUE_CHECK_TIMER_TIME		((TickType_t)(5000))	/* Value in milliseconds */	/* 5 seconds  */
#define CAR_FUEL_LEVEL_VALUE_CHECK_TIMER_TIME			((TickType_t)(30000))	/* Value in milliseconds */	/* 30 seconds */
#define BOARD_3V3_VOLTAGE_VALUE_CHECK_TIMER_TIME		((TickType_t)(2000))	/* Value in milliseconds */	/* 2 seconds  */
#define BOARD_5V_VOLTAGE_VALUE_CHECK_TIMER_TIME			((TickType_t)(2000))	/* Value in milliseconds */	/* 2 seconds  */
#define BOARD_VIN_VOLTAGE_VALUE_CHECK_TIMER_TIME		((TickType_t)(10000))	/* Value in milliseconds */	/* 10 seconds */
#define BOARD_3V3_TEMP_VALUE_CHECK_TIMER_TIME			((TickType_t)(4000))	/* Value in milliseconds */	/* 4 seconds  */
#define BOARD_5V_TEMP_VALUE_CHECK_TIMER_TIME			((TickType_t)(4000))	/* Value in milliseconds */	/* 4 seconds  */

#define ITERATION_INIT		(uint32_t)(0u)
#define ITERATION_1			(uint32_t)(1u)
#define ITERATION_2			(uint32_t)(2u)
#define ITERATION_10		(uint32_t)(10u)

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* FUNCTIONALITIES AND FEATURES */
#define BOARD_TEMPERATURE_MEASUREMENT
#define BOARD_VOLTAGE_MEASUREMENT

#define OIL_PRESSURE_BINARY_SENSOR
#define OIL_PRESSURE_ANALOG_SENSOR

#define GPS_RECEIVING
#ifdef GPS_RECEIVING
	#define GPS_PARSING
	#define GPS_ON_MICROSD_WRITE
#endif
#ifdef GPS_PARSING
	#define GPS_LCD_PRINT
#endif
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* ADCs */
#define RANK_1		((uint8_t)(0))
#define RANK_2		((uint8_t)(1))
#define RANK_3		((uint8_t)(2))
#define RANK_4		((uint8_t)(3))
#define RANK_5		((uint8_t)(4))
#define RANK_6		((uint8_t)(5))
#define RANK_7		((uint8_t)(6))
#define RANK_8		((uint8_t)(7))
#define RANK_9		((uint8_t)(8))
#define RANK_10		((uint8_t)(9))
#define RANK_11		((uint8_t)(10))
#define RANK_12		((uint8_t)(11))
#define RANK_13		((uint8_t)(12))


#define MIN_ADC_VALUE							((uint16_t)(0))
#define MAX_ADC_VALUE							((uint16_t)(4095))
#define MIN_LM35_ADC_VALUE						((int16_t)(-400)) 	/* This value corresponds to the -0.4V under the negative line (~-40*C) */
#define MAX_LM35_ADC_VALUE						((int16_t)(2482)) 	/* This value corresponds to the 2V over the negative line (~+200*C) */
#define REFERENCE_VOLTAGE						((float)(3.3))
#define NO_OF_ADC1_MEASURES						((uint32_t)(13))
#define NO_OF_ADC3_MEASURES						((uint32_t)(12))
#define MEASURE_VIN_VOLTAGE_DIVIDER				((float)(0.175438))		/* R1/(R1+R2) = 1k / (1k + 4k7) = 0.17543859649122807017543859649123 */
#define MEASURE_5V_VOLTAGE_DIVIDER				((float)(0.5))			/* R1/(R1+R2) = 4k7 / (4k7 + 4k7) = 0.50 */
#define MEASURE_3V3_VOLTAGE_DIVIDER				((float)(0.5))			/* R1/(R1+R2) = 4k7 / (4k7 + 4k7) = 0.50 */

#define MAIN_BATTERY_VOLTAGE_DIVIDER			((float)(0.175438))		/* R1/(R1+R2) = 1k / (1k + 4k7) = 0.17543859649122807017543859649123 */
#define AUXILIARY_BATTERY_VOLTAGE_DIVIDER		((float)(0.175438))		/* R1/(R1+R2) = 1k / (1k + 4k7) = 0.17543859649122807017543859649123 */

#define ADC_RESOLUTION_X_REF_VOLTAGE_uint		(uint32_t)(1241)		/* 4095 / 3.3 ~ 1241 */
#define ADC_RESOLUTION_X_REF_VOLTAGE_float		((float)(1240.9091))	/* 4095 / 3.3 ~ 1240.9091 */


#define LM35_1_N_RANK		RANK_7
#define LM35_1_P_RANK		RANK_8
#define LM35_1_N_MEM_TABLE	ADC3Measures
#define LM35_1_P_MEM_TABLE	ADC3Measures
#define LM35_1_N_ADC_VALUE	LM35_1_N_MEM_TABLE[LM35_1_N_RANK]
#define LM35_1_P_ADC_VALUE	LM35_1_P_MEM_TABLE[LM35_1_P_RANK]

#define LM35_2_N_RANK		RANK_9
#define LM35_2_P_RANK		RANK_10
#define LM35_2_N_MEM_TABLE	ADC3Measures
#define LM35_2_P_MEM_TABLE	ADC3Measures
#define LM35_2_N_ADC_VALUE	LM35_2_N_MEM_TABLE[LM35_2_N_RANK]
#define LM35_2_P_ADC_VALUE	LM35_2_P_MEM_TABLE[LM35_2_P_RANK]

#define LM35_3_N_RANK		RANK_11
#define LM35_3_P_RANK		RANK_12
#define LM35_3_N_MEM_TABLE	ADC3Measures
#define LM35_3_P_MEM_TABLE	ADC3Measures
#define LM35_3_N_ADC_VALUE	LM35_3_N_MEM_TABLE[LM35_3_N_RANK]
#define LM35_3_P_ADC_VALUE	LM35_3_P_MEM_TABLE[LM35_3_P_RANK]

#define LM35_4_N_RANK		RANK_9
#define LM35_4_P_RANK		RANK_10
#define LM35_4_N_MEM_TABLE	ADC1Measures
#define LM35_4_P_MEM_TABLE	ADC1Measures
#define LM35_4_N_ADC_VALUE	LM35_4_N_MEM_TABLE[LM35_4_N_RANK]
#define LM35_4_P_ADC_VALUE	LM35_4_P_MEM_TABLE[LM35_4_P_RANK]

#define LM35_5_N_RANK		RANK_11
#define LM35_5_P_RANK		RANK_12
#define LM35_5_N_MEM_TABLE	ADC1Measures
#define LM35_5_P_MEM_TABLE	ADC1Measures
#define LM35_5_N_ADC_VALUE	LM35_5_N_MEM_TABLE[LM35_5_N_RANK]
#define LM35_5_P_ADC_VALUE	LM35_5_P_MEM_TABLE[LM35_5_P_RANK]

#define ACC_POSITION_RANK			RANK_8
#define ACC_POSITION_MEM_TABLE		ADC1Measures
#define ACC_POSITION_ADC_VALUE		ACC_POSITION_MEM_TABLE[ACC_POSTITION_RANK]

#define FUEL_LEVEL_RANK				RANK_7
#define FUEL_LEVEL_MEM_TABLE		ADC1Measures
#define FUEL_LEVEL_ADC_VALUE		FUEL_LEVEL_MEM_TABLE[FUEL_LEVEL_RANK]

#define RESISTANCE_SENSOR_1_RANK		RANK_5
#define RESISTANCE_SENSOR_1_MEM_TABLE	ADC1Measures
#define RESISTANCE_SENSOR_1_ADC_VALUE	RESISTANCE_SENSOR_1_MEM_TABLE[RESISTANCE_SENSOR_1_RANK]

#define RESISTANCE_SENSOR_2_RANK		RANK_6
#define RESISTANCE_SENSOR_2_MEM_TABLE	ADC1Measures
#define RESISTANCE_SENSOR_2_ADC_VALUE	RESISTANCE_SENSOR_2_MEM_TABLE[RESISTANCE_SENSOR_2_RANK]

#define POTENTIOMETER_1_RANK		RANK_3
#define POTENTIOMETER_1_MEM_TABLE	ADC1Measures
#define POTENTIOMETER_1_ADC_VALUE	POTENTIOMETER_1_MEM_TABLE[POTENTIOMETER_1_RANK]

#define POTENTIOMETER_2_RANK		RANK_4
#define POTENTIOMETER_2_MEM_TABLE	ADC1Measures
#define POTENTIOMETER_2_ADC_VALUE	POTENTIOMETER_2_MEM_TABLE[POTENTIOMETER_2_RANK]

#define MAIN_BATTERY_VOLTAGE_RANK		RANK_5
#define MAIN_BATTERY_VOLTAGE_MEM_TABLE	ADC3Measures
#define MAIN_BATTERY_VOLTAGE_ADC_VALUE	MAIN_BATTERY_VOLTAGE_MEM_TABLE[MAIN_BATTERY_VOLTAGE_RANK]

#define AUX_BATTERY_VOLTAGE_RANK		RANK_6
#define AUX_BATTERY_VOLTAGE_MEM_TABLE	ADC3Measures
#define AUX_BATTERY_VOLTAGE_ADC_VALUE	AUX_BATTERY_VOLTAGE_MEM_TABLE[AUX_BATTERY_VOLTAGE_RANK]

#define BOARD_TEMP_1_RANK		RANK_2
#define BOARD_TEMP_1_MEM_TABLE	ADC3Measures
#define BOARD_TEMP_1_ADC_VALUE	BOARD_TEMP_1_MEM_TABLE[BOARD_TEMP_1_RANK]

#define BOARD_TEMP_2_RANK		RANK_3
#define BOARD_TEMP_2_MEM_TABLE	ADC3Measures
#define BOARD_TEMP_2_ADC_VALUE	BOARD_TEMP_2_MEM_TABLE[BOARD_TEMP_2_RANK]

#define BOARD_TEMP_3_RANK		RANK_4
#define BOARD_TEMP_3_MEM_TABLE	ADC3Measures
#define BOARD_TEMP_3_ADC_VALUE	BOARD_TEMP_3_MEM_TABLE[BOARD_TEMP_3_RANK]

#define BOARD_VOLTAGE_3V3_RANK		RANK_2
#define BOARD_VOLTAGE_3V3_MEM_TABLE	ADC1Measures
#define BOARD_VOLTAGE_3V3_ADC_VALUE	BOARD_VOLTAGE_3V3_MEM_TABLE[BOARD_VOLTAGE_3V3_RANK]

#define BOARD_VOLTAGE_5V_RANK		RANK_1
#define BOARD_VOLTAGE_5V_MEM_TABLE	ADC1Measures
#define BOARD_VOLTAGE_5V_ADC_VALUE	BOARD_VOLTAGE_5V_MEM_TABLE[BOARD_VOLTAGE_5V_RANK]

#define BOARD_VOLTAGE_VIN_RANK		RANK_1
#define BOARD_VOLTAGE_VIN_MEM_TABLE	ADC3Measures
#define BOARD_VOLTAGE_VIN_ADC_VALUE	BOARD_VOLTAGE_VIN_MEM_TABLE[BOARD_VOLTAGE_VIN_RANK]


#define CAR_WATER_TEMPERATURE_LM35_ADC_VALUE	(int16_t)(LM35_1_P_ADC_VALUE - LM35_1_N_ADC_VALUE)
#define CAR_OIL_TEMPERATURE_ADC_VALUE			(RESISTANCE_SENSOR_1_ADC_VALUE)
#define CAR_OIL_ANALOG_PRESSURE_ADC_VALUE		(RESISTANCE_SENSOR_2_ADC_VALUE)
#define BOARD_TEMPERATURE_3V3DCDC_ADC_VALUE		BOARD_TEMP_1_ADC_VALUE
#define BOARD_TEMPERATURE_5VDCDC_ADC_VALUE		BOARD_TEMP_2_ADC_VALUE
#define BOARD_TEMPERATURE_HBRIDGE_ADC_VALUE		BOARD_TEMP_3_ADC_VALUE


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Measurements */
#define NO_OF_ENGINE_TEMPERATURE_MEASUREMENTS_ADDED			((uint8_t)(4u))
#define NO_OF_CAR_VOLTAGES_MEASUREMENTS_ADDED				((uint8_t)(4u))
#define NO_OF_FUEL_LEVEL_MEASUREMENTS_ADDED					((uint8_t)(10u))

#define NO_OF_BOARD_TEMPERATURES_MEASUREMENTS_ADDED			((uint8_t)(4u))
#define NO_OF_BOARD_VOLTAGES_MEASUREMENTS_ADDED				((uint8_t)(4u))

#define SECONDS_IN_ONE_HOUR										((uint32_t)(3600))
#define METERS_IN_KILOMETER									((uint32_t)(1000))
#define HOW_MANY_METERS_IN_ONE_PULSE						((float)(10))	/* Value calculated for Buba */
#define HOW_MANY_RPMS_PER_ONE_PULSE							((float)(10))	/* Value calculated for Buba */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Symbols */
#define TRUE									((boolean)(1))
#define True									((boolean)(1))
#define FALSE									((boolean)(0))
#define False									((boolean)(0))
#define NO										((boolean)(0))
#define No										((boolean)(0))
#define NOK										((boolean)(0))
#define YES										((boolean)(1))
#define Yes										((boolean)(1))
#define OK										((boolean)(1))
#define DEGREE_SYMBOL_LCD						((uint8_t)(0b11011111))
#define SPACE_IN_ASCII							((uint8_t)(0x20))
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* EEPROM and FRAM */
#define EEPROM_CAR_ADDRESS							((uint16_t)(0b10100000))
#define EEPROM_BOARD_ADDRESS						((uint16_t)(0b10100010))
#define FRAM_ADDRESS								((uint16_t)(0b10100000))

#define EEPROM_PAGE_SIZE							((uint16_t)(64))
#define EEPROM_PAGES_ADDRESS_SIZE					((uint16_t)(2))
#define FRAM_PAGES_ADDRESS_SIZE						((uint16_t)(1))

#define MAX_DIAGNOSTIC_SNAPSHOT_SIZE				(EEPROM_PAGE_SIZE)
#define MAX_ERROR_SNAPSHOT_SIZE						((uint16_t)(10u))

#define NO_ADDRESS									((uint16_t)(0u))

/* EEPROM CAR ADDRESSES AND SETTINGS */
#define TOTAL_SNAPSHOTS_NUMBER_ADDRESS							((uint16_t)(64))	/* 8 bit counter - 0-255 snapshots */
#define NUMBER_OF_DIAGNOSTIC_SNAPSHOTS_OVERFLOWED_ADDRESS		((uint16_t)(68))	/* indicates if the counter overflowed */
#define DIAGNOSTIC_SNAPSHOTS_START_ADDRESS						((uint16_t)(1024))	/* Max 255 diagnostic snapshots due to 8bit counter (add: 1024 - 17280) */

#define TOTAL_MILEAGE_START_ADDRESS					((uint16_t)(0))	/* 4 bytes value! (uint32) */
#define TRIP_MILEAGE_START_ADDRESS					((uint16_t)(4))	/* 4 bytes value! (uint32) */


#define WATER_HIGH_TEMP_WARNING_THRESHOLD_ADDRESS	((uint16_t)(768))	/* 4 bytes value! (float) */
#define WATER_HIGH_TEMP_ALARM_THRESHOLD_ADDRESS		((uint16_t)(772))	/* 4 bytes value! (float) */
#define WATER_HIGH_TEMP_FAN_ON_THRESHOLD_ADDRESS	((uint16_t)(776))	/* 4 bytes value! (float) */
#define WATER_HIGH_TEMP_FAN_OFF_THRESHOLD_ADDRESS	((uint16_t)(780))	/* 4 bytes value! (float) */
#define WATER_TEMP_ALL_SETTINGS_ADDRESS				((uint16_t)(784))

#define OIL_HIGH_TEMP_WARNING_THRESHOLD_ADDRESS		((uint16_t)(788))	/* 4 bytes value! (float) */
#define OIL_HIGH_TEMP_ALARM_THRESHOLD_ADDRESS		((uint16_t)(792))	/* 4 bytes value! (float) */
#define OIL_TEMP_ALL_SETTINGS_ADDRESS				((uint16_t)(796))

#define MAIN_BATTERY_LOW_VOLTAGE_ALARM_THRESHOLD_ADDRESS		((uint16_t)(800))	/* 4 bytes value! (float) */
#define MAIN_BATTERY_HIGH_VOLTAGE_ALARM_THRESHOLD_ADDRESS		((uint16_t)(804))	/* 4 bytes value! (float) */
#define MAIN_BATTERY_ALL_SETTINGS_ADDRESS						((uint16_t)(808))

#define AUXILIARY_BATTERY_LOW_VOLTAGE_ALARM_THRESHOLD_ADDRESS	((uint16_t)(812))	/* 4 bytes value! (float) */
#define AUXILIARY_BATTERY_HIGH_VOLTAGE_ALARM_THRESHOLD_ADDRESS	((uint16_t)(816))	/* 4 bytes value! (float) */
#define AUXILIARY_BATTERY_ALL_SETTINGS_ADDRESS					((uint16_t)(820))

#define FUEL_LOW_LEVEL_WARNING_THRESHOLD_ADDRESS				((uint16_t)(824))	/* 4 bytes value! (float) */
#define FUEL_ALL_SETTINGS_ADDRESS								((uint16_t)(828))

#define OIL_HIGH_PRESSURE_ALARM_THRESHOLD_ADDRESS				((uint16_t)(832))	/* 4 bytes value! (float) */
#define OIL_LOW_PRESSURE_ALARM_THRESHOLD_ADDRESS				((uint16_t)(836))	/* 4 bytes value! (float) */
#define OIL_PRESSURE_ALL_SETTINGS_ADDRESS						((uint16_t)(840))

/* EEPROM BOARD ADDRESSES AND SETTINGS */
#define NUMBER_OF_ERROR_SNAPSHOTS							((uint16_t)(64))	/* 8 bit counter - 0-255 snapshots */
#define NUMBER_OF_ERROR_SNAPSHOTS_OVERFLOWED_ADDRESS		((uint16_t)(68))	/* indicates if the counter overflowed */
#define ERROR_SNAPSHOTS_START_ADDRESS						((uint16_t)(1024))	/* Max 255 error snapshots due to 8bit counter (add: 1024 - 17280) */

#define HOME_LATITUDE_ADDRESS								((uint16_t)(128))	/* (float) Home GPS coordinates as a float (8 bytes for potential "double" usage) */
#define HOME_LONGITUDE_ADDRESS								((uint16_t)(136))	/* (float) Home GPS coordinates as a float (8 bytes for potential "double" usage) */
#define TIME_ZONE_ADJ_POLAND_ADDRESS						((uint16_t)(144))	/* "1" in winter and "2" in summer (time adjustment for UTC+0) */
#define TIME_MANUAL_ADJUSTMENT_ADDRESS						((uint16_t)(148))	/* "+-12h" in order to be able to adjust to the time zone */

#define BOARD_5V_SUPPLY_LOW_THRESHOLD_ADDRESS					((uint16_t)(256))	/* 4 bytes value! (float) */
#define BOARD_5V_SUPPLY_HIGH_THRESHOLD_ADDRESS					((uint16_t)(260))	/* 4 bytes value! (float) */
#define BOARD_3V3_SUPPLY_LOW_THRESHOLD_ADDRESS					((uint16_t)(264))	/* 4 bytes value! (float) */
#define BOARD_3V3_SUPPLY_HIGH_THRESHOLD_ADDRESS					((uint16_t)(268))	/* 4 bytes value! (float) */
#define BOARD_VIN_SUPPLY_LOW_THRESHOLD_ADDRESS					((uint16_t)(272))	/* 4 bytes value! (float) */
#define BOARD_VOLTAGE_ALL_SETTINGS_ADDRESS						((uint16_t)(276))

#define BOARD_5V_TEMPERATURE_THRESHOLD_ADDRESS					((uint16_t)(280))	/* 4 bytes value! (float) */
#define BOARD_3V3_TEMPERATURE_THRESHOLD_ADDRESS					((uint16_t)(284))	/* 4 bytes value! (float) */
#define BOARD_HBRIDGE_TEMPERATURE_THRESHOLD_ADDRESS				((uint16_t)(288))	/* 4 bytes value! (float) */
#define BOARD_TEMPERATURE_ALL_SETTINGS_ADDRESS					((uint16_t)(292))

#define BOARD_BUZZER_ALL_SETTINGS_ADDRESS						((uint16_t)(300))

#define BOARD_LCD_HOME_SCREEN_ADDRESS							((uint16_t)(320))
#define BOARD_LCD_AUTO_HOME_RETURN_TIME_ADDRESS					((uint16_t)(324))
#define BOARD_LCD_BACKLIGHT_LEVEL_ADDRESS						((uint16_t)(328))
#define BOARD_LCD_SECONDS_TO_AUTO_TURN_OFF_BACKLIGHT_ADDRESS	((uint16_t)(332))
#define BOARD_LCD_AUTO_BACKLIGHT_OFF_HOUR_START_ADDRESS			((uint16_t)(336))
#define BOARD_LCD_AUTO_BACKLIGHT_OFF_HOUR_END_ADDRESS			((uint16_t)(340))
#define BOARD_LCD_ALL_SETTINGS_ADDRESS							((uint16_t)(400))

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* LCD */
#define NO_OF_ROWS_IN_LCD			((uint8_t)(4u))
#define NO_OF_COLUMNS_IN_LCD		((uint8_t)(20u))

#define MAX_WAIT_TIME_FOR_EEPROM	((uint32_t)(500U))
#define ERROR_DONE_DISPLAY_TIME		((TickType_t)(2000U))
#define HELLO_MESSAGE_DISPLAY_TIME	((TickType_t)(2000U))
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* SD CARD */
#define TEST_MESSAGE_FOR_CHECK		("This is a test text.")
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Sizes of variables and types in bytes */
#define UINT8_T_SIZE			((uint8_t)(1u))
#define INT8_T_SIZE				((uint8_t)(1u))
#define UINT16_T_SIZE			((uint8_t)(2u))
#define INT16_T_SIZE			((uint8_t)(2u))
#define UINT32_T_SIZE			((uint8_t)(4u))
#define INT32_T_SIZE			((uint8_t)(4u))
#define UINT64_T_SIZE			((uint8_t)(8u))
#define INT64_T_SIZE			((uint8_t)(8u))
#define FLOAT_SIZE				((uint8_t)(4u))
#define BOOLEAN_SIZE			((uint8_t)(1u))
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* My defines for Pins and Ports */
#define EEPROM_CAR_BLOCK_PORT				(EEPROM_WP1_GPIO_Port)
#define EEPROM_BOARD_BLOCK_PORT				(EEPROM_WP2_GPIO_Port)
#define FRAM_BLOCK_PORT						(FRAM_WP_GPIO_Port)
#define EEPROM_CAR_BLOCK_PIN				(EEPROM_WP1_Pin)
#define EEPROM_BOARD_BLOCK_PIN				(EEPROM_WP2_Pin)
#define FRAM_BLOCK_PIN						(FRAM_WP_Pin)

#define ENC_BUTTON_MENU_Pin 				(ENCODER_1_BUTTON_Pin)
#define ENC_BUTTON_MENU_GPIO_Port 			(ENCODER_1_BUTTON_GPIO_Port)


#define GPIO_PORT_OIL_PRESSURE_SENSOR		(OIL_PRESSURE_BINARY_GPIO_Port)
#define GPIO_PIN_OIL_PRESSURE_SENSOR		(OIL_PRESSURE_BINARY_Pin)


#define DCDC5V_ENABLE_PIN				(DCDC_5V_ENABLE_Pin)
#define DCDC5V_ENABLE_PORT				(DCDC_5V_ENABLE_GPIO_Port)
#define POWER_ON_GPS_PIN				(POWER_ON_GPS_Pin)
#define POWER_ON_GPS_PORT				(POWER_ON_GPS_GPIO_Port)
#define POWER_ON_LCD_PIN				(POWER_ON_LCD_Pin)
#define POWER_ON_LCD_PORT				(POWER_ON_LCD_GPIO_Port)
#define POWER_ON_MICRO_SD_PIN			(POWER_ON_MICROSD_Pin)
#define POWER_ON_MICRO_SD_PORT			(POWER_ON_MICROSD_GPIO_Port)
#define POWER_ON_NODE_MCU_PIN			(POWER_ON_NODEMCU_Pin)
#define POWER_ON_NODE_MCU_PORT			(POWER_ON_NODEMCU_GPIO_Port)
#define POWER_ON_CRUISE_CONTROL_PIN		(POWER_ON_CRUISE_CONTROL_Pin)
#define POWER_ON_CRUISE_CONTROL_PORT	(POWER_ON_CRUISE_CONTROL_GPIO_Port)
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* MACROS */
#define INTEGER_DIVISION_ROUNDING(x, y)	((x + (y - 1)) / (y))
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */



#endif /* INC_DEFINES_H_ */
