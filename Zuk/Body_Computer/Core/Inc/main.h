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
	JarvisSettings_Layer		=	0x08,

		/* CarSettings_Layer */
		ClearDiagnosticSnapshots	=	0x71,
		ClearTripMileage			=	0x72,
		WaterSettings_Layer			=	0x73,
		OilTempSettings_Layer		=	0x74,
		OilPressureSettings_Layer	=	0x75,
		FuelSettings_Layer			=	0x76,
		MainBatterySettings_Layer	=	0x77,
		AuxBatterySettings_Layer	=	0x78,

		/* JarvisSettings_Layer */
		ClearErrorsSnapshots		=	0x81,
		AdjPolishTime				=	0x82,
		AdjTimeZone					=	0x83,
		InterVoltSettings_Layer		=	0x84,
		InterTempSettings_Layer		=	0x85,
		BuzzerSettings_Layer		=	0x86,
		LCDSettings_Layer			=	0x87,

			/* CarSettings_Layer -> WaterSettings_Layer */
			WaterHighTempWarningThreshold	=	0x731,
			WaterHighTempAlarmThreshold		=	0x732,
			WaterHighTempFanOnThreshold		=	0x733,
			WaterHighTempFanOffThreshold	=	0x734,
			WaterTempWarningOn				=	0x735,
			WaterTempAlarmOn				=	0x736,
			WaterFanControlOn				=	0x737,
			WaterTempWarningBuzzerOn		=	0x738,
			WaterTempAlarmBuzzerOn			=	0x739,
			WaterTempWarningSnapshotOn		=	0x73A,
			WaterTempAlarmSnapshotOn		=	0x73B,

			/* CarSettings_Layer -> OilTempSettings_Layer */
			OilHighTempWarningThreshold		=	0x741,
			OilHighTempAlarmThreshold		=	0x742,
			OilTempWarningOn				=	0x743,
			OilTempAlarmOn					=	0x744,
			OilTempWarningBuzzerOn			=	0x745,
			OilTempAlarmBuzzerOn			=	0x746,
			OilTempWarningSnapshotOn		=	0x747,
			OilTempAlarmSpashotOn			=	0x748,

			/* CarSettings_Layer -> OilPressureSettings_Layer */
			OilHighPressureAlarmThreshold	=	0x751,
			OilLowPressureAlarmThreshold	=	0x752,
			OilPressureAnalogMeasurement	=	0x753,
			OilHighPressureAlarmOn			=	0x754,
			OilLowPressureAlarmOn			=	0x755,
			OilPressureAlarmBuzzerOn		=	0x756,
			OilPressureAlarmSnapshotOn		=	0x757,

			/* CarSettings_Layer -> FuelSettings_Layer */
			FuelLowLevelWarningThreshold	=	0x761,
			FuelLowLevelWarningOn			=	0x762,
			FuelLowLevelWarningBuzzerOn		=	0x763,

			/* CarSettings_Layer -> MainBatterySettings_Layer */
			MainBatteryLowVoltageAlarmThreshold		=	0x771,
			MainBatteryHighVoltageAlarmThreshold	=	0x772,
			MainBatteryLowVoltageAlarmOn			=	0x773,
			MainBatteryHighVoltageAlarmOn			=	0x774,
			MainBatteryLowVoltageAlarmBuzzerOn		=	0x775,
			MainBatteryHighVoltageAlarmBuzzerOn		=	0x776,
			MainBatteryLowVoltageSnapshotOn			=	0x777,
			MainBatteryHighVoltageSnapshotOn		=	0x778,

			/* CarSettings_Layer -> AuxBatterySettings_Layer */
			AuxBatteryLowVoltageAlarmThreshold		=	0x781,
			AuxBatteryHighVoltageAlarmThreshold		=	0x782,
			AuxBatteryLowVoltageAlarmOn				=	0x783,
			AuxBatteryHighVoltageAlarmOn			=	0x784,
			AuxBatteryLowVoltageAlarmBuzzerOn		=	0x785,
			AuxBatteryHighVoltageAlarmBuzzerOn		=	0x786,
			AuxBatteryLowVoltageSnapshotOn			=	0x787,
			AuxBatteryHighVoltageSnapshotOn			=	0x788,

			/* JarvisSettings_Layer -> InterVoltSettings_Layer */
			Supply5VLowThreshold		=	0x841,
			Supply5VHighThreshold		=	0x842,
			Supply3V3LowThreshold		=	0x843,
			Supply3V3HighThreshold		=	0x844,
			SupplyVinLowThreshold		=	0x845,
//			Board3V3SupplyAlarmOn		=	0x846,
//			Board3V3SupplyBuzzerAlarmOn	=	0x847,
//			Board5VSupplyAlarmOn		=	0x848,
//			Board5VSupplyBuzzerAlarmON	=	0x849,
//			BoardVinSupplyAlarmOn		=	0x85A,
//			BoardVinSupplyBuzzerAlarmOn	=	0x85B,

			/* JarvisSettings_Layer -> InterTempSettings_Layer */
			DCDC3V3HighTempThreshold	=	0x851,
			DCDC5VHighTempThreshold		=	0x852,

			/* JarvisSettings_Layer -> BuzzerSettings_Layer */
			BuzzerMainSwitch			=	0x861,
			BuzzerMainAlarmsSwitch		=	0x862,
			BuzzerMainButtonsSwitch		=	0x863,
			BuzzerWhenShortPress		=	0x864,
			BuzzerWhenLongPress			=	0x865,

			/* JarvisSettings_Layer -> LCDSettings_Layer */
			BacklightBrightnessLevel	=	0x871,
			SecondsToTurnOffBacklight	=	0x872,
			AutoBacklightOffStartHour	=	0x873,
			AutoBacklightOffEndHour		=	0x874,
			HomeScreen					=	0x875,
			AutoHomeReturnTime			=	0x876,
			AutoBacklightOff			=	0x877,

	AreYouSure_Layer		=	0xF000,
	Ctrl_Layer				=	0xF100,

	Alarm_Layer				=	0xFFFF
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
	_void_type_					=	0,
	_boolean_type_				=	1,

	_carTemperature_type_		=	10,
	_carOilAnalogPressure_type_	=	11,
	_carOilBinaryPressure_type_	=	12,
	_cafFuelLevel_type_			=	13,
	_carVoltage_type_			=	14,

	_boardVoltage_type_			=	20,
	_boardTemperature_type_		=	21
}Enum_valueType;
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
typedef enum
{
	StepNotApplicable	=	0,
	StepByToogling		=	1,
	StepByOne			=	1,
	StepByOneTen		=	10,
	StepByOneHundred	=	100
}Enum_valueStepSize;
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
//typedef enum
//{
//	None_ScreenType				=	0x00,
//	NotApplicable_ScreenType	=	0x00,
//	No_ScreenType				=	0x00,
//
//	OneScreen_ScreenType		=	0x01,
//	ScrollList_ScreenType		=	0x02,
//	YesNo_ScreenType			=	0x03,
//	Ctrl_ScreenType				=	0x04,
//
//	Alarm_ScreenType			=	0xFF
//}Enum_ScreenType;
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
typedef float carTemperature_type;
typedef float carVoltage_type;
typedef float cafFuelLevel_type;
typedef float carOilAnalogPressure_type;
typedef boolean carOilBinaryPressure_type;

typedef float boardTemperature_type;
typedef float boardVoltage_type;
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Diagnostic_Code Snapshot */
typedef struct
{
	Diagnostic_Code snapshotIdentificator;				//4 bytes total
	float value;										//8 bytes total

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
			boolean waterTempWarningOn			:1;
			boolean waterTempAlarmOn			:1;
			boolean waterFanControlOn			:1;
			boolean waterTempWarningBuzzerOn	:1;
			boolean waterTempAlarmBuzzerOn		:1;
			boolean waterTempWarningSnapshotOn 	:1;
			boolean waterTempAlarmSnapshotOn	:1;
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
			boolean oilTempWarningOn			:1;
			boolean oilTempAlarmOn				:1;
			boolean oilTempWarningBuzzerOn		:1;
			boolean oilTempAlarmBuzzerOn		:1;
			boolean oilTempWarningSnapshotOn	:1;
			boolean oilTempAlarmSnapshotOn		:1;
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
			boolean lowVoltageAlarmOn				:1;
			boolean highVoltageAlarmOn				:1;
			boolean lowVoltageAlarmBuzzerOn			:1;
			boolean highVoltageAlarmBuzzerOn		:1;
			boolean lowVoltageAlarmSnapshotOn		:1;
			boolean highVoltageAlarmSnapshotOn		:1;
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
			boolean lowFuelLevelWarningOn			:1;
			boolean lowFuelLevelWarningBuzzerOn		:1;
		};
	};
}fuelSettings_struct;
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
typedef struct
{
	carOilAnalogPressure_type oilHighPressureAlarmThreshold;
	carOilAnalogPressure_type oilLowPressureAlarmThreshold;

	union
	{
		uint8_t allSettings;
		struct
		{
			boolean oilPressureAnalogMeasurement	:1;
			boolean oilHighPressureAlarmOn			:1;
			boolean oilLowPressureAlarmOn			:1;
			boolean oilPressureAlarmBuzzerOn		:1;
			boolean oilPressureAlarmSnapshotOn		:1;
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
