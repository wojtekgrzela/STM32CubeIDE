/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "../ErrorCodes/ErrorCodes.h"

#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include <string.h>
#include <math.h>

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Diagnostic codes */
typedef enum
{
	DIAGNOSTICS_OK								= 0,

	WATER_TEMP_HIGH_WARNING						= 10,
	WATER_TEMP_HIGH_ALARM						= 11,

	OIL_TEMP_HIGH_WARNING						= 20,
	OIL_TEMP_HIGH_ALARM							= 21,

	LOW_VOLTAGE_MAIN_BATTERY_ALARM				= 30,
	HIGH_VOLTAGE_MAIN_BATTERY_ALARM				= 31,

	LOW_VOLTAGE_AUXILIARY_BATTERY_ALARM			= 40,
	HIGH_VOLTAGE_AUXILIARY_BATTERY_ALARM		= 41,

	UNKNOWN_ISSUE								= 0xF0F0F0F0
}Diagnostic_Code;
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Diagnostic_Code Snapshot */
typedef struct
{
	Diagnostic_Code snapshotIdentificator;				//4 bytes total
	uint16_t value;										//8 bytes total (2 unused)

}Diagnostic_Snapshot_struct;
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* ENCODER BUTTON STRUCT */
typedef union
{
	uint8_t allFlags;
	struct
	{
		uint8_t shortPressDetected		:1;	/* LSB!!! */
		uint8_t longPressDetected		:1;

		uint8_t longPressInfoForISR		:1;	/* This is an info for the external
												interrupt callback that there
												was a long press detected and
												not to set the short press on */
	};
}ENCButton_struct;
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
typedef struct
{
	uint8_t waterHighTempWarningThreshold;
	uint8_t waterHighTempAlarmThreshold;
	uint8_t waterHighTempFanOnThreshold;
	uint8_t waterHighTempFanOffThreshold;

	union
	{
		uint8_t allSettings;
		struct
		{
			uint8_t waterTempWarningOn			:1;
			uint8_t waterTempAlarmOn			:1;
			uint8_t waterFanControlOn			:1;
			uint8_t waterTempWarningBuzzerOn	:1;
			uint8_t waterTempAlarmBuzzerOn		:1;
			uint8_t waterTempWarningSnapshotOn 	:1;
			uint8_t waterTempAlarmSnapshotOn	:1;
			/* 1 more free bits here */
		};
	};
}waterTempSettings_struct;
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
typedef struct
{
	uint8_t oilHighTempWarningThreshold;
	uint8_t oilHighTempAlarmThreshold;

	union
	{
		uint8_t allSettings;
		struct
		{
			uint8_t oilTempWarningOn			:1;
			uint8_t oilTempAlarmOn				:1;
			uint8_t oilTempWarningBuzzerOn		:1;
			uint8_t oilTempAlarmBuzzerOn		:1;
			uint8_t oilTempWarningSnapshotOn	:1;
			uint8_t oilTempAlarmSnapshotOn		:1;
			/* 2 more free bits here */
		};
	};
}oilTempSettings_struct;
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
typedef struct
{
	float batteryLowVoltageAlarmThreshold;
	float batteryHighVoltageAlarmThreshold;

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
}batterySettings_struct;
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
typedef struct
{
	uint8_t fuelLowLevelWarningThreshold;

	union
	{
		uint8_t allSettings;
		struct
		{
			uint8_t lowFuelLevelWarningOn			:1;
			uint8_t lowFuelLevelWarningBuzzerOn		:1;
		};
	};
}fuelSettings_struct;
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
typedef struct
{
	uint8_t oilHighPressureAlarmThreshold;
	uint8_t oilLowPressureAlarmThreshold;

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
}oilPressureSettings_struct;
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
typedef union
{
	uint8_t data[8];
	struct
	{
		uint32_t totalMileage;
		uint32_t tripMileage;
	};
}CAR_mileage_struct;
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
typedef struct
{
	uint8_t diagnosticSnapshotEEPROMIndex;
	uint8_t didTheNumberOfDiagnosticSnapshotsOverflowed;
	uint8_t mileageEEPROMIndex;
}CAR_EEPROM_counters_struct;
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
typedef enum
{
	MainMenu_Layer			=	0x00,
	Desktop_Layer			=	0x01,
	GPS_Layer				=	0x02,
	CarInfo_Layer			=	0x03,
	JarvisInfo_Layer		=	0x04,
	Last3Diag_Layer			=	0x05,
	Last3Err_Layer			=	0x06,
	CarSettings_Layer		=	0x07,
	BoardSettings_Layer		=	0x08,

	ClearDiagnosticSnapshots	=	0x71,
	ClearTripMileage			=	0x72,
	WaterSettings_Layer			=	0x73,
	OilTempSettings_Layer		=	0x74,
	OilPressureSettings_Layer	=	0x75,
	FuelSettings_Layer			=	0x76,
	MainBatterySettings_Layer	=	0x77,
	AuxBatterySettings_Layer	=	0x78,

	ClearErrorsSnapshots		=	0x81,
	AdjTimePoland				=	0x82,
	AdjTimeZone					=	0x83,
	InterVoltSettings_Layer		=	0x84,
	InterTempSettings_Layer		=	0x85,
	BuzzerSettings_Layer		=	0x86,
	LCDSettings_Layer			=	0x87,

	YesNo_Layer				=	0xF0,
	Ctrl_Layer				=	0xF1,

	Alarm_Layer				=	0xFF
}Enum_Layer;
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
typedef enum
{
	None_EnterAction			=	0x00,
	NotApplicable_EnterAction	=	0x00,
	No_EnterAction				=	0x00,

	GoInto_EnterAction			=	0x01,
	OnOff_EnterAction			=	0x02,
	YesNo_EnterAction			=	0x03,
	Ctrl_EnterAction			=	0x04,

	WinterSummer_EnterAction	=	0x10,

	Alarm_EnterAction			=	0xFF
}Enum_ActionForEnter;
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
typedef enum
{
	None_ScreenType				=	0x00,
	NotApplicable_ScreenType	=	0x00,
	No_ScreenType				=	0x00,

	OneScreen_ScreenType		=	0x01,
	ScrollList_ScreenType		=	0x02,
	YesNo_ScreenType			=	0x03,
	Ctrl_ScreenType				=	0x04,

	Alarm_ScreenType			=	0xFF
}Enum_ScreenType;
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
typedef enum
{
	Row1	=	0x00,
	Row2	=	0x01,
	Row3	=	0x02,
	Row4	=	0x03,
}Enum_LCDRow;
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
typedef struct
{
	char name[21];		/* 20 for string, 21 for '\0', to be able to fill whole line in LCD */
	uint8_t nameActualSize;	/* Actual length of the name */

	Enum_Layer layer;	/* Layer number as enum for cleaner code */
	Enum_ActionForEnter actionForEnter;	/* specifies type of action for short button click */
	Enum_ScreenType screenType;	/* Type of the screen (one board, list to scroll etc.) */

	Enum_Layer layerPrevious;	/* to know where to get back when long press is detected */
}LCDBoard;

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define MEASURE_VIN_Pin GPIO_PIN_0
#define MEASURE_VIN_GPIO_Port GPIOC
#define MEASURE_5V_Pin GPIO_PIN_1
#define MEASURE_5V_GPIO_Port GPIOC
#define BOARD_TEMP_STABILIZER_Pin GPIO_PIN_2
#define BOARD_TEMP_STABILIZER_GPIO_Port GPIOC
#define BOARD_TEMP_DCDC_Pin GPIO_PIN_3
#define BOARD_TEMP_DCDC_GPIO_Port GPIOC
#define LM35_CarWaterTemp_Pin GPIO_PIN_2
#define LM35_CarWaterTemp_GPIO_Port GPIOA
#define BUZZER_Pin GPIO_PIN_2
#define BUZZER_GPIO_Port GPIOB
#define LED_8_Pin GPIO_PIN_10
#define LED_8_GPIO_Port GPIOE
#define LED_7_Pin GPIO_PIN_12
#define LED_7_GPIO_Port GPIOE
#define LED_6_Pin GPIO_PIN_15
#define LED_6_GPIO_Port GPIOE
#define I2C2_SCL_LCD_Pin GPIO_PIN_10
#define I2C2_SCL_LCD_GPIO_Port GPIOB
#define I2C2_SDA_LCD_Pin GPIO_PIN_11
#define I2C2_SDA_LCD_GPIO_Port GPIOB
#define ENC_BUTTON_Pin GPIO_PIN_12
#define ENC_BUTTON_GPIO_Port GPIOB
#define ENC_BUTTON_EXTI_IRQn EXTI15_10_IRQn
#define ENC_A_Pin GPIO_PIN_6
#define ENC_A_GPIO_Port GPIOC
#define ENC_B_Pin GPIO_PIN_7
#define ENC_B_GPIO_Port GPIOC
#define USART1_TX_GPS_Pin GPIO_PIN_9
#define USART1_TX_GPS_GPIO_Port GPIOA
#define USART1_RX_GPS_Pin GPIO_PIN_10
#define USART1_RX_GPS_GPIO_Port GPIOA
#define LED_5_Pin GPIO_PIN_0
#define LED_5_GPIO_Port GPIOD
#define LED_4_Pin GPIO_PIN_1
#define LED_4_GPIO_Port GPIOD
#define LED_3_Pin GPIO_PIN_3
#define LED_3_GPIO_Port GPIOD
#define LED_2_Pin GPIO_PIN_4
#define LED_2_GPIO_Port GPIOD
#define LED_1_Pin GPIO_PIN_7
#define LED_1_GPIO_Port GPIOD
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define I2C1_SCL_EEPROM_Pin GPIO_PIN_6
#define I2C1_SCL_EEPROM_GPIO_Port GPIOB
#define I2C1_SDA_EEPROM_Pin GPIO_PIN_7
#define I2C1_SDA_EEPROM_GPIO_Port GPIOB
#define EEPROM_WP_1_Pin GPIO_PIN_8
#define EEPROM_WP_1_GPIO_Port GPIOB
#define EEPROM_WP_2_Pin GPIO_PIN_9
#define EEPROM_WP_2_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

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
#define MY_EEPROM_TASK_TIME_PERIOD			(TickType_t)(1000)	/* Would be good to decrease this number significantly, as it only executes when it has data from a Queue */
#define MY_DUMP_TO_EEPROM_TASK_TIME_PERIOD 	(TickType_t)(1000)
#define MY_DUMP_TO_SDCARD_TASK_TIME_PERIOD	(TickType_t)(1000)
#define MY_TASK_50_MS_TIME_PERIOD			(TickType_t)(50)
#define MY_TASK_250_MS_TIME_PERIOD			(TickType_t)(250)
#define MY_TASK_500_MS_TIME_PERIOD			(TickType_t)(500)
#define MY_TASK_1000_MS_TIME_PERIOD			(TickType_t)(1000)

#define ENC_BUTTON_LONG_PRESS_TIME			((uint32_t)(1000))	/* Value in milliseconds */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* FUNCTIONALITIES AND FEATURES */
#define BOARD_TEMPERATURE_MEASUREMENT
#define BOARD_VOLTAGE_MEASUREMENT

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
#define MAX_ADC_VALUE							((uint32_t)(4095))
#define REFERENCE_VOLTAGE						((float)(3.3))
#define NO_OF_ADC1_MEASURES						((uint32_t)(1))
#define NO_OF_ADC3_MEASURES						((uint32_t)(4))
#define MEASURE_VIN_VOLTAGE_DIVIDER				((float)(0.16016260))	/* R1/(R1+R2) = 1k/(1k+4.7k) ~ 0.17543859649 BUT after measuring: 0.16016260 */
#define MEASURE_5V_VOLTAGE_DIVIDER				((float)(0.444342629))	/* R1/(R1+R2) = 1.6k/(1.6k+1.6k) = 0.5000000 BUT after measuring: 0.444342629*/

#define ADC_RESOLUTION_X_REF_VOLTAGE_uint		(uint32_t)(1241)	/* 4095 / 3.3 ~ 1241 */
#define ADC_RESOLUTION_X_REF_VOLTAGE_float		(float)(1240.9091)	/* 4095 / 3.3 ~ 1240.9091 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* ADC's measurements */
#define NO_OF_ENGINE_WATER_TEMP_MEASUREMENTS_ADDED			((uint8_t)(4))
#define NO_OF_ENGINE_OIL_TEMP_MEASUREMENTS_ADDED			((uint8_t)(4))
#define NO_OF_ENGINE_OIL_PRESSURE_MEASUREMENTS_ADDED		((uint8_t)(4))
#define NO_OF_MAIN_BATTERY_VOLTAGE_MEASUREMENTS_ADDED		((uint8_t)(4))
#define NO_OF_AUXILIARY_BATTERY_VOLTAGE_MEASUREMENTS_ADDED	((uint8_t)(4))
#define NO_OF_FUEL_LEVEL_MEASUREMENTS_ADDED					((uint8_t)(20))
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Symbols */
#define TRUE									((uint8_t)(1))
#define True									((uint8_t)(1))
#define FALSE									((uint8_t)(0))
#define False									((uint8_t)(0))
#define DEGREE_SYMBOL_LCD						((uint8_t)(0b11011111))
#define SPACE_IN_ASCII							((uint8_t)(0x20))
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* EEPROM */
#define EEPROM_CAR_ADDRESS							((uint16_t)(0b10100000))
#define EEPROM_BOARD_ADDRESS						((uint16_t)(0b10100010))

#define EEPROM_CAR_BLOCK_PORT						(EEPROM_WP_1_GPIO_Port)
#define EEPROM_BOARD_BLOCK_PORT						(EEPROM_WP_2_GPIO_Port)
#define EEPROM_CAR_BLOCK_PIN						(EEPROM_WP_1_Pin)
#define EEPROM_BOARD_BLOCK_PIN						(EEPROM_WP_2_Pin)

#define EEPROM_PAGE_SIZE							((uint16_t)(64))

/* EEPROM CAR ADDRESSES AND SETTINGS */
#define TOTAL_SNAPSHOTS_NUMBER_ADDRESS							((uint16_t)(64))	/* 8 bit counter - 0-255 snapshots */
#define NUMBER_OF_DIAGNOSTIC_SNAPSHOTS_OVERFLOWED_ADDRESS		((uint16_t)(65))	/* indicates if the counter overflowed */

#define TOTAL_MILEAGE_START_ADDRESS					((uint16_t)(128))
#define TRIP_MILEAGE_START_ADDRESS					((uint16_t)(132))
#define CAR_MILEAGE_TABLE_SIZE						((uint8_t)(10))

#define DIAGNOSTIC_SNAPSHOTS_START_ADDRESS			((uint16_t)(1024))	/* Max 255 diagnostic snapshots due to 8bit counter (add: 1024 - 17280) */

#define WATER_HIGH_TEMP_WARNING_THRESHOLD_ADDRESS	((uint16_t)(768))
#define WATER_HIGH_TEMP_ALARM_THRESHOLD_ADDRESS		((uint16_t)(769))
#define WATER_HIGH_TEMP_FAN_ON_THRESHOLD_ADDRESS	((uint16_t)(770))
#define WATER_HIGH_TEMP_FAN_OFF_THRESHOLD_ADDRESS	((uint16_t)(771))
#define WATER_TEMP_ALL_SETTINGS_ADDRESS				((uint16_t)(772))

#define OIL_HIGH_TEMP_WARNING_THRESHOLD_ADDRESS		((uint16_t)(778))
#define OIL_HIGH_TEMP_ALARM_THRESHOLD_ADDRESS		((uint16_t)(779))
#define OIL_TEMP_ALL_SETTINGS_ADDRESS				((uint16_t)(780))

#define MAIN_BATTERY_LOW_VOLTAGE_ALARM_THRESHOLD_ADDRESS		((uint16_t)(788))	/* 4 bytes value! (float) */
#define MAIN_BATTERY_HIGH_VOLTAGE_ALARM_THRESHOLD_ADDRESS		((uint16_t)(792))	/* 4 bytes value! (float) */
#define MAIN_BATTERY_ALL_SETTINGS_ADDRESS						((uint16_t)(796))

#define AUXILIARY_BATTERY_LOW_VOLTAGE_ALARM_THRESHOLD_ADDRESS	((uint16_t)(798))	/* 4 bytes value! (float) */
#define AUXILIARY_BATTERY_HIGH_VOLTAGE_ALARM_THRESHOLD_ADDRESS	((uint16_t)(802))	/* 4 bytes value! (float) */
#define AUXILIARY_BATTERY_ALL_SETTINGS_ADDRESS					((uint16_t)(806))

#define FUEL_LOW_LEVEL_WARNING_THRESHOLD_ADDRESS				((uint16_t)(808))
#define FUEL_ALL_SETTINGS_ADDRESS								((uint16_t)(810))

#define OIL_HIGH_PRESSURE_ALARM_THRESHOLD_ADDRESS				((uint16_t)(818))
#define OIL_LOW_PRESSURE_ALARM_THRESHOLD_ADDRESS				((uint16_t)(819))
#define OIL_PRESSURE_ALL_SETTINGS_ADDRESS						((uint16_t)(820))

/* EEPROM BOARD ADDRESSES AND SETTINGS */
#define NUMBER_OF_ERROR_SNAPSHOTS							((uint16_t)(64))	/* 8 bit counter - 0-255 snapshots */
#define NUMBER_OF_ERROR_SNAPSHOTS_OVERFLOWED_ADDRESS		((uint16_t)(64))	/* indicates if the counter overflowed */

#define HOME_LATITUDE								((uint16_t)(128))	/* (float) Home GPS coordinates as a float (8 bytes for potential "double" usage) */
#define HOME_LONGITUDE								((uint16_t)(136))	/* (float) Home GPS coordinates as a float (8 bytes for potential "double" usage) */
#define TIME_ZONE_ADJ_POLAND						((uint16_t)(144))	/* "1" in winter and "2" in summer (time adjustment for UTC+0 */

#define ERROR_SNAPSHOTS_START_ADDRESS				((uint16_t)(1024))	/* Max 255 error snapshots due to 8bit counter (add: 1024 - 17280) */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* LCD */
#define NO_OF_ROWS_IN_LCD		((uint8_t)(4))
#define NO_OF_COLUMNS_IN_LCD	((uint8_t)(20))
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */



/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
