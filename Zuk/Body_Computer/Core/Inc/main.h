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

#include "defines.h"
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

	Done_EnterAction			=	0x05,

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
typedef uint8_t carTemperature_type;
typedef float carVoltage_type;
typedef uint8_t cafFuelLevel_type;
typedef uint8_t carOilPressure_type;

typedef float boardTemperature_type;
typedef float boardVoltage_type;
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
/* Error_code Snapshot */
typedef struct
{
	Error_Code snapshotIdentificator;					//4 bytes total
	float value;										//8 bytes total
}Error_Snapshot_struct;
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
	carTemperature_type waterHighTempWarningThreshold;
	carTemperature_type waterHighTempAlarmThreshold;
	carTemperature_type waterHighTempFanOnThreshold;
	carTemperature_type waterHighTempFanOffThreshold;

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
	carTemperature_type oilHighTempWarningThreshold;
	carTemperature_type oilHighTempAlarmThreshold;

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
	carVoltage_type batteryLowVoltageAlarmThreshold;
	carVoltage_type batteryHighVoltageAlarmThreshold;

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
	cafFuelLevel_type fuelLowLevelWarningThreshold;

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
	carOilPressure_type oilHighPressureAlarmThreshold;
	carOilPressure_type oilLowPressureAlarmThreshold;

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

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *//* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *//* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *//* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
typedef struct
{
	boardVoltage_type board3V3SupplyLowThreshold;
	boardVoltage_type board3V3SupplyHighThreshold;
	boardVoltage_type board5VSupplyLowThreshold;
	boardVoltage_type board5VSupplyHighThreshold;
	boardVoltage_type boardVinSupplyLowThreshold;

	union
	{
		uint8_t allSettings;
		struct
		{
			uint8_t board3V3SupplyAlarmOn		:1;
			uint8_t board3V3SupplyBuzzerAlarmOn	:1;
			uint8_t board5VSupplyAlarmOn		:1;
			uint8_t board5VSupplyBuzzerAlarmON	:1;
			uint8_t boardVinSupplyAlarmOn		:1;
			uint8_t oardVinSupplyBuzzerAlarmOn	:1;
			/* 2 more free bits here */
		};
	};
}boardVoltagesSettings_struct;
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
typedef struct
{
	boardTemperature_type board5VDCDCTemperatureHighThreshold;
	boardTemperature_type board3V3DCDCTemperatureHighThreshold;

	union
	{
		uint8_t allSettings;
		struct
		{
			uint8_t board5VDCDCTemperatureHighAlarmON			:1;
			uint8_t board5VDCDCTemperatureHighBuzzerAlarmOn		:1;
			uint8_t board3V3DCDCTemperatureHighAlarmOn			:1;
			uint8_t oard3V3DCDCTemperatureHighBuzzerAlarmOn	:1;
			/* 4 more free bits here */
		};
	};
}boardTemperaturesSettings_struct;
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
typedef struct
{
	union
	{
		uint8_t allSettings;
		struct
		{
			uint8_t buzzerMainSwitchOn			:1;
			uint8_t buzzerMainAlarmsSwitchOn	:1;
			uint8_t buzzerMainButtonsSwitchOn	:1;
			uint8_t buzzerWhenShortPressOn		:1;
			uint8_t buzzerWhenLognPressOn		:1;
		};
	};
}buzzerMainSettings_struct;
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
typedef struct
{
	Enum_Layer homeScreen;	// should be 4 bytes
	uint8_t autoHomeReturnTime;
	uint8_t backlightLevel;
	uint8_t secondsToAutoTurnOffBacklight;
	uint8_t autoBacklightOffHourStart;
	uint8_t autoBacklightOffHourEnd;

	union
	{
		uint8_t allSettings;
		struct
		{
			uint8_t autoBacklightOffAtNightOn	:1;
			/* 7 more free bits here */
		};
	};
}LCDMainSettings_struct;
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
typedef struct
{
	uint8_t errorSnapshotEEPROMIndex;
	uint8_t didTheNumberOfErrorSnapshotsOverflowed;
}BOARD_EEPROM_counters_struct;
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
#define OIL_PRESSURE_BINARY_IN_Pin GPIO_PIN_14
#define OIL_PRESSURE_BINARY_IN_GPIO_Port GPIOC
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


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
