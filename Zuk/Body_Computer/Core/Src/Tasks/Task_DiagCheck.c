/*
 * Task_DiagCheck.c
 *
 *  Created on: Jan 15, 2021
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
//#include "../../VariousFunctions/Functions.h"
#include "../../EEPROM/EEPROM.h"
//#include "../../GPS/GPS_Parsing.h"
//#include "../../lcd_hd44780_i2c/lcd_hd44780_i2c.h"
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Defines */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#define RPM_ENGINE_OFF_MAX_THRESHOLD		(uint32_t)(50U)
#define RPM_ENGINE_CRANK_MIN_THRESHOLD		(uint32_t)(50U)
#define RPM_ENGINE_CRANK_MAX_THRESHOLD		(uint32_t)(600U)
#define RPM_ENGINE_IDLE_MIN_THRESHOLD		(uint32_t)(600U)
#define RPM_ENGINE_IDLE_MAX_THRESHOLD		(uint32_t)(1000U)

#define SPEED_ENGINE_IDLE_MAX_THRESHOLD		(uint32_t)(5U)

typedef enum
{
	val_error	= 0,
	val_OK		= 1,
	val_warning	= 2,
	val_alarm	= 3
}Enum_ValueState;

typedef struct
{
	Enum_ValueState carWaterTemp_ValueState;
	Enum_ValueState carOilTemp_ValueState;
	Enum_ValueState carOilAnalogPressure_ValueState;
	Enum_ValueState carOilBinaryPressure_ValueState;
	Enum_ValueState carMainBattVoltage_ValueState;
	Enum_ValueState carAuxBattVoltage_ValueState;
	Enum_ValueState carFuelLevel_ValueState;

	Enum_ValueState board3V3Voltage_ValueState;
	Enum_ValueState board5VVoltage_ValueState;
	Enum_ValueState boardVinVoltage_ValueState;
	Enum_ValueState board3V3Temp_ValueState;
	Enum_ValueState board5VTemp_ValueState;
}Diagnostic_ValueStates_struct;
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Extern variables */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
extern EEPROM_parameters_struct EEPROM_car;
extern EEPROM_parameters_struct EEPROM_board;
extern CAR_EEPROM_counters_struct CAR_EEPROM_counters;
extern BOARD_EEPROM_counters_struct BOARD_EEPROM_counters;

extern CAR_mileage_struct CAR_mileage;

extern volatile ENCButton_struct ENC_button;
extern volatile int8_t EncoderCounterDiff;

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

extern boardVoltagesSettings_struct BOARD_voltage;
extern boardTemperaturesSettings_struct BOARD_temperature;

extern CarStateinfo_type CarStateInfo;
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Local variables */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
static Diagnostic_ValueStates_struct valueStates = {val_OK};
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */



static void Read_KeyState(void);
static void Read_EngineState(void);
static void Read_CarState(void);
static void Read_AlternatorCharging(void);

static void Check_CarWaterTemp();
static void Check_CarOilTemp();
static void Check_CarOilPressure();
static void Check_CarMainBattVoltage();
static void Check_CarAuxBattVoltage();
static void Check_CarFuelLevel();

static void Check_BoardVoltage();
static void Check_BoardTemp();

static inline boolean IsLower(float value, float threshold);
static inline boolean IsHigher(float value, float threshold);



void StartTaskDiagCheck(void const * argument)
{
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = MY_DIAG_CHECK_TASK_TIME_PERIOD;
//	Error_Code error = NO_ERROR;

	CarStateInfo.keyState = KeyState_Out;
	CarStateInfo.engineState = EngineState_Off;
	CarStateInfo.carState = CarState_Off;
	CarStateInfo.AlternatorCharging = FALSE;

	xLastWakeTime = xTaskGetTickCount();

	/* Infinite loop */
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
	for (;;)
	{
		/* Check keyState (Out, IgnitionOn, Crank) */
		Read_KeyState();
		/* Check engineState (Off, Crank, Idle, Work) */
		Read_EngineState();
		/* Check carState (Off, Crank, Idle, Drive) */
		Read_CarState();
		/* Check AlternatorCharging (TRUE, FALSE) */
		Read_AlternatorCharging();

		/* In every car state there might be a different set of checks to perform */
		switch(CarStateInfo.carState)
		{
			case CarState_Off:
			{
				Check_BoardVoltage();
				Check_BoardTemp();
				break;
			}
			case CarState_Crank:
			{
				Check_BoardVoltage();
				Check_BoardTemp();
				break;
			}
			case CarState_Idle:
			{
				Check_CarWaterTemp();
				Check_CarOilTemp();
				Check_CarOilPressure();
				Check_CarMainBattVoltage();
				Check_CarAuxBattVoltage();
				Check_CarFuelLevel();

				Check_BoardVoltage();
				Check_BoardTemp();
				break;
			}
			case CarState_Drive:
			{
				Check_CarWaterTemp();
				Check_CarOilTemp();
				Check_CarOilPressure();
				Check_CarMainBattVoltage();
				Check_CarAuxBattVoltage();
				Check_CarFuelLevel();

				Check_BoardVoltage();
				Check_BoardTemp();
				break;
			}
			default:
			{
				break;
			}
		}









		vTaskDelayUntil(&xLastWakeTime, xFrequency);
	}

}



static void Read_KeyState(void)
{
//	if(GPIO_PIN_SET == HAL_GPIO_ReadPin(KeyCrank_PORT, KeyCrank_PIN))	//TODO
//	{
//		CarStateInfo.keyState = KeyState_Crank;
//	}
//	else if(GPIO_PIN_SET == HAL_GPIO_ReadPin(KeyIgnition_PORT, KeyIgnition_PIN))
//	{
//		CarStateInfo.keyState = KeyState_IgnitionOn;
//	}
//	else
//	{
//		CarStateInfo.keyState = KeyState_Out;
//	}
}



static void Read_EngineState(void)
{
	if(RPM_ENGINE_OFF_MAX_THRESHOLD > CarStateInfo.RPM)
	{
		CarStateInfo.engineState = EngineState_Off;
	}
	else if((RPM_ENGINE_CRANK_MIN_THRESHOLD < CarStateInfo.RPM) &&
			(RPM_ENGINE_CRANK_MAX_THRESHOLD > CarStateInfo.RPM) &&
			(KeyState_Crank == CarStateInfo.keyState))
	{
		CarStateInfo.engineState = EngineState_Crank;
	}
	else if((RPM_ENGINE_IDLE_MIN_THRESHOLD < CarStateInfo.RPM) &&
			(RPM_ENGINE_IDLE_MAX_THRESHOLD > CarStateInfo.RPM) &&
			(SPEED_ENGINE_IDLE_MAX_THRESHOLD > CarStateInfo.SPEED))
	{
		CarStateInfo.engineState = EngineState_Idle;
	}
	else
	{
		CarStateInfo.engineState = EngineState_Work;
	}
}



static void Read_CarState(void)
{
	if(EngineState_Off == CarStateInfo.engineState)
	{
		CarStateInfo.carState = CarState_Off;
	}
	else if((KeyState_Crank == CarStateInfo.keyState) || (EngineState_Crank == CarStateInfo.engineState))
	{
		CarStateInfo.carState = CarState_Crank;
	}
	else if((KeyState_IgnitionOn == CarStateInfo.keyState) && (EngineState_Idle == CarStateInfo.engineState))
	{
		CarStateInfo.carState = CarState_Idle;
	}
	else if((KeyState_IgnitionOn == CarStateInfo.keyState) && (EngineState_Work == CarStateInfo.engineState))
	{
		CarStateInfo.carState = CarState_Drive;
	}
	else
	{
		CarStateInfo.carState = CarState_Error;
	}
}


/* This function reads the pin state which is connected to the 12V line signal from alternator
 * (the same signal as for the icon of lack of charging) */
static void Read_AlternatorCharging(void)
{
//	if(GPIO_PIN_SET == HAL_GPIO_ReadPin(AlternatorCharging_PORT, AlternatorCharging_PIN))	//TODO
//	{
//		CarStateInfo.AlternatorCharging = TRUE;
//	}
//	else
//	{
//		CarStateInfo.AlternatorCharging = FALSE;
//	}
}



/* This function checks whether the current water temperature in the vehicle is below the thresholds. */
static void Check_CarWaterTemp()
{
	if(IsHigher((float)waterTemperatureValue, (float)CAR_waterTemp.waterHighTempAlarmThreshold))
	{
		valueStates.carWaterTemp_ValueState = val_alarm;
	}
	else if(IsHigher((float)waterTemperatureValue, (float)CAR_waterTemp.waterHighTempWarningThreshold))
	{
		valueStates.carWaterTemp_ValueState = val_warning;
	}
	else
	{
		valueStates.carWaterTemp_ValueState = val_OK;
	}
}



/* This function checks whether the current oil temperature in the vehicle is below the thresholds. */
static void Check_CarOilTemp()
{
	if(IsHigher((float)oilTemperatureValue, (float)CAR_oilTemp.oilHighTempAlarmThreshold))
	{
		valueStates.carOilTemp_ValueState = val_alarm;
	}
	else if(IsHigher((float)oilTemperatureValue, (float)CAR_oilTemp.oilHighTempWarningThreshold))
	{
		valueStates.carOilTemp_ValueState = val_warning;
	}
	else
	{
		valueStates.carOilTemp_ValueState = val_OK;
	}
}



/* This function checks whether the current oil pressure in the vehicle is in the thresholds. */
static void Check_CarOilPressure()
{
	if(IsHigher((float)oilPressureValue, (float)CAR_oilPressure.oilHighPressureAlarmThreshold))
	{
		valueStates.carOilAnalogPressure_ValueState = val_alarm;
	}
	else if(IsLower((float)oilPressureValue, (float)CAR_oilPressure.oilLowPressureAlarmThreshold))
	{
		valueStates.carOilAnalogPressure_ValueState = val_alarm;
	}
	else
	{
		valueStates.carOilAnalogPressure_ValueState = val_OK;
	}

	if(FALSE == oilPressureValueBinary)
	{
		valueStates.carOilBinaryPressure_ValueState = val_alarm;
	}
	else
	{
		valueStates.carOilBinaryPressure_ValueState = val_OK;
	}
}



/* This function checks whether the current battery voltage in the vehicle is in the thresholds. */
static void Check_CarMainBattVoltage()
{
	if(IsHigher((float)mainBatteryVoltageValue, (float)CAR_mainBattery.batteryHighVoltageAlarmThreshold))
	{
		valueStates.carMainBattVoltage_ValueState = val_alarm;
	}
	else if(IsLower((float)mainBatteryVoltageValue, (float)CAR_mainBattery.batteryLowVoltageAlarmThreshold))
	{
		valueStates.carMainBattVoltage_ValueState = val_alarm;
	}
	else
	{
		valueStates.carMainBattVoltage_ValueState = val_OK;
	}
}



/* This function checks whether the current battery voltage in the vehicle is in the thresholds. */
static void Check_CarAuxBattVoltage()
{
	if(IsHigher((float)auxiliaryBatteryVoltageValue, (float)CAR_auxiliaryBattery.batteryHighVoltageAlarmThreshold))
	{
		valueStates.carAuxBattVoltage_ValueState = val_alarm;
	}
	else if(IsLower((float)auxiliaryBatteryVoltageValue, (float)CAR_auxiliaryBattery.batteryLowVoltageAlarmThreshold))
	{
		valueStates.carAuxBattVoltage_ValueState = val_alarm;
	}
	else
	{
		valueStates.carAuxBattVoltage_ValueState = val_OK;
	}
}



/* This function checks whether the current fuel level in the vehicle is above the warning thresholds. */
static void Check_CarFuelLevel()
{
	if(IsLower((float)fuelLevelValue, (float)CAR_fuel.fuelLowLevelWarningThreshold))
	{
		valueStates.carFuelLevel_ValueState = val_warning;
	}
	else
	{
		valueStates.carFuelLevel_ValueState = val_OK;
	}
}



/* This function checks whether the current voltages on the board are within the thresholds. */
static void Check_BoardVoltage()
{
	/* 3V3 Check */
	if(IsHigher((float)voltage3V3, (float)BOARD_voltage.board3V3SupplyHighThreshold))
	{
		valueStates.board3V3Voltage_ValueState = val_alarm;
	}
	else if(IsLower((float)voltage3V3, (float)BOARD_voltage.board3V3SupplyLowThreshold))
	{
		valueStates.board3V3Voltage_ValueState = val_alarm;
	}
	else
	{
		valueStates.board3V3Voltage_ValueState = val_OK;
	}

	/* 5V Check */
	if(IsHigher((float)voltage5V, (float)BOARD_voltage.board5VSupplyHighThreshold))
	{
		valueStates.board5VVoltage_ValueState = val_alarm;
	}
	else if(IsLower((float)voltage5V, (float)BOARD_voltage.board5VSupplyLowThreshold))
	{
		valueStates.board5VVoltage_ValueState = val_alarm;
	}
	else
	{
		valueStates.board5VVoltage_ValueState = val_OK;
	}

	/* Vin Check */
	if(IsLower((float)voltageIn, (float)BOARD_voltage.boardVinSupplyLowThreshold))
	{
		valueStates.boardVinVoltage_ValueState = val_alarm;
	}
	else
	{
		valueStates.boardVinVoltage_ValueState = val_OK;
	}

}



/* This function checks whether the current temperatures on the board are within the thresholds. */
static void Check_BoardTemp()
{
	/* 3V3 Check */
	if(IsHigher((float)temperature3V3DCDC, (float)BOARD_temperature.board3V3DCDCTemperatureHighThreshold))
	{
		valueStates.board3V3Temp_ValueState = val_alarm;
	}
	else
	{
		valueStates.board3V3Temp_ValueState = val_OK;
	}

	/* 5V Check */
	if(IsHigher((float)temperature5VDCDC, (float)BOARD_temperature.board5VDCDCTemperatureHighThreshold))
	{
		valueStates.board5VTemp_ValueState = val_alarm;
	}
	else
	{
		valueStates.board5VTemp_ValueState = val_OK;
	}
}



static inline boolean IsLower(float value, float threshold)
{
	if(value <= threshold)
		return TRUE;
	else
		return FALSE;
}
static inline boolean IsHigher(float value, float threshold)
{
	if(value >= threshold)
		return TRUE;
	else
		return FALSE;
}



