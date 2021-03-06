/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

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
// @formatter:off

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
	JarvisSettings_Layer	=	0x08,

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
			Board3V3SupplyAlarmOn		=	0x846,
			Board3V3SupplyBuzzerAlarmOn	=	0x847,
			Board5VSupplyAlarmOn		=	0x848,
			Board5VSupplyBuzzerAlarmOn	=	0x849,
			BoardVinSupplyAlarmOn		=	0x85A,
			BoardVinSupplyBuzzerAlarmOn	=	0x85B,

			/* JarvisSettings_Layer -> InterTempSettings_Layer */
			DCDC5VHighTempThreshold		=	0x851,
			DCDC3V3HighTempThreshold	=	0x852,
			DCDC5VTempHighAlarmOn		=	0x853,
			DCDC5VTempeHighBuzzAlarmOn	=	0x854,
			DCDC3V3TempHighAlarmOn		=	0x855,
			DCDC3V3TempHighBuzzAlarmOn	=	0x856,

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
	Alarm_Layer				=	0xFFFF
}Enum_Layer;
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
	_boardTemperature_type_		=	21,

	_LCDSettings_type_			=	22,
	_LCD_Enum_Layer_type_		=	23,

	_timeHours_type_			=	24
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

	UNKNOWN_ISSUE								= 0xF0F0
}Enum_DiagnosticCode;
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
typedef enum
{
	KeyState_Error		= 0,
	KeyState_Out 		= 1,
	KeyState_IgnitionOn	= 2,
	KeyState_Crank		= 3
}Enum_KeyState;
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
typedef enum
{
	EngineState_Error	= 0,
	EngineState_Off		= 1,
	EngineState_Crank	= 2,
	EngineState_Idle	= 3,
	EngineState_Work	= 4
}Enum_EngineState;
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
typedef enum
{
	CarState_Error	= 0,
	CarState_Off	= 1,
	CarState_Crank	= 2,
	CarState_Idle	= 3,
	CarState_Drive	= 4
}Enum_CarState;
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

typedef struct
{
	Enum_KeyState keyState;
	Enum_EngineState engineState;
	Enum_CarState carState;

	uint32_t RPM;
	uint32_t SPEED;

	boolean AlternatorCharging :1;
}CarStateinfo_type;

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
typedef float carTemperature_type;
typedef float carVoltage_type;
typedef float cafFuelLevel_type;
typedef float carOilAnalogPressure_type;
typedef boolean carOilBinaryPressure_type;

typedef float boardTemperature_type;
typedef float boardVoltage_type;
typedef uint8_t LCDSettings_type;

typedef int8_t timeHours_type;
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Enum_DiagnosticCode Snapshot */
typedef struct
{
	Enum_DiagnosticCode snapshotIdentificator;			//4 bytes total
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
		uint8_t shortPressDetectedBuzzer	:1;
		uint8_t longPressDetectedBuzzer		:1;
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
	boolean didTheNumberOfDiagnosticSnapshotsOverflowed;
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
			uint8_t boardVinSupplyBuzzerAlarmOn	:1;
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
			uint8_t oard3V3DCDCTemperatureHighBuzzerAlarmOn		:1;
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
	LCDSettings_type autoHomeReturnTime;
	LCDSettings_type backlightLevel;
	LCDSettings_type secondsToAutoTurnOffBacklight;
	LCDSettings_type autoBacklightOffHourStart;
	LCDSettings_type autoBacklightOffHourEnd;

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
	boolean signalBuzzerIndication;
	boolean signalBuzzerDone;
	boolean signalSnapshotIndication;
	boolean signalSnapshotDone;
	boolean* signalSetting1;
	boolean* signalSetting2;
	void (*BuzzerSignal)(void);
}valueSignal_type;
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
typedef struct
{
	const timeHours_type polishTimeAdj_min;
	const timeHours_type polishTimeAdj_max;
	const timeHours_type timeZoneAdj_min;
	const timeHours_type timeZoneAdj_max;

	const carTemperature_type waterHighTempWarningThreshold_min;
	const carTemperature_type waterHighTempWarningThreshold_max;
	const carTemperature_type waterHighTempAlarmThreshold_min;
	const carTemperature_type waterHighTempAlarmThreshold_max;
	const carTemperature_type waterHighTempFanOnThreshold_min;
	const carTemperature_type waterHighTempFanOnThreshold_max;
	const carTemperature_type waterHighTempFanOffThreshold_min;
	const carTemperature_type waterHighTempFanOffThreshold_max;

	const carTemperature_type oilHighTempWarningThreshold_min;
	const carTemperature_type oilHighTempWarningThreshold_max;
	const carTemperature_type oilHighTempAlarmThreshold_min;
	const carTemperature_type oilHighTempAlarmThreshold_max;

	const carOilAnalogPressure_type oilHighPressureAlarmThreshold_min;
	const carOilAnalogPressure_type oilHighPressureAlarmThreshold_max;
	const carOilAnalogPressure_type oilLowPressureAlarmThreshold_min;
	const carOilAnalogPressure_type oilLowPressureAlarmThreshold_max;

	const carVoltage_type batteryLowVoltageAlarmThreshold_min;
	const carVoltage_type batteryLowVoltageAlarmThreshold_max;
	const carVoltage_type batteryHighVoltageAlarmThreshold_min;
	const carVoltage_type batteryHighVoltageAlarmThreshold_max;

	const cafFuelLevel_type fuelLowLevelWarningThreshold_min;
	const cafFuelLevel_type fuelLowLevelWarningThreshold_max;

	const boardVoltage_type board3V3SupplyLowThreshold_min;
	const boardVoltage_type board3V3SupplyLowThreshold_max;
	const boardVoltage_type board3V3SupplyHighThreshold_min;
	const boardVoltage_type board3V3SupplyHighThreshold_max;
	const boardVoltage_type board5VSupplyLowThreshold_min;
	const boardVoltage_type board5VSupplyLowThreshold_max;
	const boardVoltage_type board5VSupplyHighThreshold_min;
	const boardVoltage_type board5VSupplyHighThreshold_max;
	const boardVoltage_type boardVinSupplyLowThreshold_min;
	const boardVoltage_type boardVinSupplyLowThreshold_max;

	const boardTemperature_type board5VDCDCTemperatureHighThreshold_min;
	const boardTemperature_type board5VDCDCTemperatureHighThreshold_max;
	const boardTemperature_type board3V3DCDCTemperatureHighThreshold_min;
	const boardTemperature_type board3V3DCDCTemperatureHighThreshold_max;

	const Enum_Layer homeScreen_min;
	const Enum_Layer homeScreen_max;
	const LCDSettings_type autoHomeReturnTime_min;
	const LCDSettings_type autoHomeReturnTime_max;
	const LCDSettings_type backlightLevel_min;
	const LCDSettings_type backlightLevel_max;
	const LCDSettings_type secondsToAutoTurnOffBacklight_min;
	const LCDSettings_type secondsToAutoTurnOffBacklight_max;
	const LCDSettings_type autoBacklightOffHourStart_min;
	const LCDSettings_type autoBacklightOffHourStart_max;
	const LCDSettings_type autoBacklightOffHourEnd_min;
	const LCDSettings_type autoBacklightOffHourEnd_max;

}GlobalValuesLimits_struct;
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
// @formatter:on
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
#define DCDC_5V_ENABLE_Pin GPIO_PIN_10
#define DCDC_5V_ENABLE_GPIO_Port GPIOI
#define BUZZER_Pin GPIO_PIN_11
#define BUZZER_GPIO_Port GPIOI
#define LCD_I2C2_SDA_Pin GPIO_PIN_0
#define LCD_I2C2_SDA_GPIO_Port GPIOF
#define LCD_I2C2_SCL_Pin GPIO_PIN_1
#define LCD_I2C2_SCL_GPIO_Port GPIOF
#define POWER_ON_LCD_Pin GPIO_PIN_2
#define POWER_ON_LCD_GPIO_Port GPIOF
#define DCDC_3V3_Pin GPIO_PIN_3
#define DCDC_3V3_GPIO_Port GPIOF
#define DCDC_5V_Pin GPIO_PIN_4
#define DCDC_5V_GPIO_Port GPIOF
#define H_BRIDGE_DRIVER_Pin GPIO_PIN_5
#define H_BRIDGE_DRIVER_GPIO_Port GPIOF
#define AUXILIARY_BATTERY_Pin GPIO_PIN_6
#define AUXILIARY_BATTERY_GPIO_Port GPIOF
#define MAIN_BATTERY_Pin GPIO_PIN_7
#define MAIN_BATTERY_GPIO_Port GPIOF
#define LM35_3_N_Pin GPIO_PIN_8
#define LM35_3_N_GPIO_Port GPIOF
#define LM35_3_P_Pin GPIO_PIN_9
#define LM35_3_P_GPIO_Port GPIOF
#define LM35_2_N_Pin GPIO_PIN_10
#define LM35_2_N_GPIO_Port GPIOF
#define LM35_2_P_Pin GPIO_PIN_0
#define LM35_2_P_GPIO_Port GPIOC
#define LM35_1_N_Pin GPIO_PIN_1
#define LM35_1_N_GPIO_Port GPIOC
#define LM35_1_P_Pin GPIO_PIN_2
#define LM35_1_P_GPIO_Port GPIOC
#define LM35_5_N_Pin GPIO_PIN_3
#define LM35_5_N_GPIO_Port GPIOC
#define LM35_5_P_Pin GPIO_PIN_0
#define LM35_5_P_GPIO_Port GPIOA
#define LM35_4_N_Pin GPIO_PIN_1
#define LM35_4_N_GPIO_Port GPIOA
#define LM35_4_P_Pin GPIO_PIN_2
#define LM35_4_P_GPIO_Port GPIOA
#define DIAG_LED_1_Pin GPIO_PIN_2
#define DIAG_LED_1_GPIO_Port GPIOH
#define DIAG_LED_2_Pin GPIO_PIN_3
#define DIAG_LED_2_GPIO_Port GPIOH
#define DIAG_LED_3_Pin GPIO_PIN_4
#define DIAG_LED_3_GPIO_Port GPIOH
#define DIAG_LED_4_Pin GPIO_PIN_5
#define DIAG_LED_4_GPIO_Port GPIOH
#define VOLTAGE_VIN_Pin GPIO_PIN_3
#define VOLTAGE_VIN_GPIO_Port GPIOA
#define POTENTIOMETER_2_Pin GPIO_PIN_4
#define POTENTIOMETER_2_GPIO_Port GPIOA
#define POTENTIOMETER_1_Pin GPIO_PIN_5
#define POTENTIOMETER_1_GPIO_Port GPIOA
#define RESISTANCE_SENSOR_2_Pin GPIO_PIN_6
#define RESISTANCE_SENSOR_2_GPIO_Port GPIOA
#define RESISTANCE_SENSOR_1_Pin GPIO_PIN_7
#define RESISTANCE_SENSOR_1_GPIO_Port GPIOA
#define VOLTAGE_5V_Pin GPIO_PIN_4
#define VOLTAGE_5V_GPIO_Port GPIOC
#define VOLTAGE_3V3_Pin GPIO_PIN_5
#define VOLTAGE_3V3_GPIO_Port GPIOC
#define FUEL_LEVEL_Pin GPIO_PIN_0
#define FUEL_LEVEL_GPIO_Port GPIOB
#define ACC_POSITION_Pin GPIO_PIN_1
#define ACC_POSITION_GPIO_Port GPIOB
#define OIL_PRESSURE_BINARY_Pin GPIO_PIN_12
#define OIL_PRESSURE_BINARY_GPIO_Port GPIOF
#define EEPROM_WP2_Pin GPIO_PIN_10
#define EEPROM_WP2_GPIO_Port GPIOB
#define EEPROM_WP1_Pin GPIO_PIN_11
#define EEPROM_WP1_GPIO_Port GPIOB
#define GPS_I2C3_SCL_Pin GPIO_PIN_7
#define GPS_I2C3_SCL_GPIO_Port GPIOH
#define GPS_I2C3_SDA_Pin GPIO_PIN_8
#define GPS_I2C3_SDA_GPIO_Port GPIOH
#define MICROSD_DETECT_Pin GPIO_PIN_13
#define MICROSD_DETECT_GPIO_Port GPIOB
#define ENCODER_1_BUTTON_Pin GPIO_PIN_8
#define ENCODER_1_BUTTON_GPIO_Port GPIOG
#define ENCODER_1_A_Pin GPIO_PIN_8
#define ENCODER_1_A_GPIO_Port GPIOA
#define ENCODER_1_B_Pin GPIO_PIN_9
#define ENCODER_1_B_GPIO_Port GPIOA
#define POWER_ON_MICROSD_Pin GPIO_PIN_0
#define POWER_ON_MICROSD_GPIO_Port GPIOD
#define POWER_ON_NODEMCU_Pin GPIO_PIN_13
#define POWER_ON_NODEMCU_GPIO_Port GPIOG
#define POWER_ON_GPS_Pin GPIO_PIN_15
#define POWER_ON_GPS_GPIO_Port GPIOG
#define GPS_USART1_TX_Pin GPIO_PIN_6
#define GPS_USART1_TX_GPIO_Port GPIOB
#define GPS_USART1_RX_Pin GPIO_PIN_7
#define GPS_USART1_RX_GPIO_Port GPIOB
#define FRAM_I2C1_SCL_Pin GPIO_PIN_8
#define FRAM_I2C1_SCL_GPIO_Port GPIOB
#define FRAM_I2C1_SDA_Pin GPIO_PIN_9
#define FRAM_I2C1_SDA_GPIO_Port GPIOB
#define FRAM_WP_Pin GPIO_PIN_0
#define FRAM_WP_GPIO_Port GPIOE
#define FAN_BOARD_Pin GPIO_PIN_5
#define FAN_BOARD_GPIO_Port GPIOI
#define POWER_ON_CRUISE_CONTROL_Pin GPIO_PIN_7
#define POWER_ON_CRUISE_CONTROL_GPIO_Port GPIOI
/* USER CODE BEGIN Private defines */


/* My defines for Pins and Ports */
#define EEPROM_CAR_BLOCK_PORT				(EEPROM_WP1_GPIO_Port)
#define EEPROM_BOARD_BLOCK_PORT				(EEPROM_WP2_GPIO_Port)
#define EEPROM_CAR_BLOCK_PIN				(EEPROM_WP1_Pin)
#define EEPROM_BOARD_BLOCK_PIN				(EEPROM_WP2_Pin)

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

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
