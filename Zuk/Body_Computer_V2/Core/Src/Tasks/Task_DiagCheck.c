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
#include "../../EEPROM/EEPROM.h"
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
	val_OK		= 0,
	val_warning	= 1,
	val_alarm	= 2
}Enum_ValueState;

typedef enum
{
	val_notApplicable	= 0,
	val_tooLow			= 1,
	val_tooHigh			= 2
}Enum_valueTooLowHigh;

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

typedef struct
{
	Enum_valueTooLowHigh carOilAnalogPressure_ValueState;
	Enum_valueTooLowHigh carMainBattVoltage_ValueState;
	Enum_valueTooLowHigh carAuxBattVoltage_ValueState;

	Enum_valueTooLowHigh board3V3Voltage_ValueState;
	Enum_valueTooLowHigh board5VVoltage_ValueState;
}Diagnostic_ValueTooLowHigh_struct;
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Extern variables */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
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

extern osTimerId My_Timer_carWaterTempValueCheckHandle;
extern osTimerId My_Timer_carOilTempValueCheckHandle;
extern osTimerId My_Timer_carOilAnalogPressureValueCheckHandle;
extern osTimerId My_Timer_carOilBinaryPressureValueCheckHandle;
extern osTimerId My_Timer_carMainBattVoltageValueCheckHandle;
extern osTimerId My_Timer_carAuxBattVoltageValueCheckHandle;
extern osTimerId My_Timer_carFuelLevelValueCheckHandle;
extern osTimerId My_Timer_board3V3VoltageValueCheckHandle;
extern osTimerId My_Timer_board5VVoltageValueCheckHandle;
extern osTimerId My_Timer_boardVinVoltageValueCheckHandle;
extern osTimerId My_Timer_board3V3TempValueCheckHandle;
extern osTimerId My_Timer_board5VTempValueCheckHandle;

extern valueSignal_type carWaterTemp_warning;
extern valueSignal_type carWaterTemp_alarm;
extern valueSignal_type carOilTemp_warning;
extern valueSignal_type carOilTemp_alarm;
extern valueSignal_type carOilAnalogPressure_alarm;
extern valueSignal_type carOilBinaryPressure_alarm;
extern valueSignal_type carMainBattVoltage_alarm;
extern valueSignal_type carAuxBattVoltage_alarm;
extern valueSignal_type carFuelLevel_warning;

extern valueSignal_type board3V3Voltage_alarm;
extern valueSignal_type board5VVoltage_alarm;
extern valueSignal_type boardVinVoltage_alarm;
extern valueSignal_type board3V3Temp_alarm;
extern valueSignal_type board5VTemp_alarm;
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Local variables */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
static Diagnostic_ValueStates_struct valueStates = {val_OK};
static Diagnostic_ValueTooLowHigh_struct valueStatesLowHigh = {val_notApplicable};
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Functions prototypes */
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

static void CheckValuesStateAndSendAlarmRequests(void);

static Error_Code CheckValueAndTimer(Enum_ValueState valState, osTimerId timerId, TickType_t timerSetting);
static boolean IsLower(float value, float threshold);
static boolean IsHigher(float value, float threshold);
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */



void StartDiagCheckTask(void const * argument)
{
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = MY_DIAG_CHECK_TASK_TIME_PERIOD;

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

		/* Always Check Voltages and Temperatures on the board */
		Check_BoardVoltage();
		Check_BoardTemp();

		/* In every car state there might be a different set of checks to perform */
		switch(CarStateInfo.carState)
		{
			case CarState_Off:
			case CarState_Crank:
			{
				/* Set checked values to OK - we do not want to trigger an alarm here */
				valueStates.carWaterTemp_ValueState = val_OK;
				valueStates.carOilTemp_ValueState = val_OK;
				valueStates.carOilAnalogPressure_ValueState = val_OK;
				valueStates.carOilBinaryPressure_ValueState = val_OK;
				valueStates.carMainBattVoltage_ValueState = val_OK;
				valueStates.carAuxBattVoltage_ValueState = val_OK;
				valueStates.carFuelLevel_ValueState = val_OK;
				break;
			}
			case CarState_Idle:
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
		}//switch(CarStateInfo.carState)

		/* Check the values (if they are OK, in warning or alarm thresholds).
		 * If the values are not OK - start the process of setting an alarm
		 * (for example - start a timer etc.). */
		CheckValuesStateAndSendAlarmRequests();


		vTaskDelayUntil(&xLastWakeTime, xFrequency);
	}
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
}



/* Check in what state is the key in the car (out, on ignition, cranking) */
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



/* Check the state of the engine (off, cranking, idling, working). */
static void Read_EngineState(void)
{
	if(RPM_ENGINE_OFF_MAX_THRESHOLD > CarStateInfo.RPM) /* if RPM are higher than the threshold */
	{
		CarStateInfo.engineState = EngineState_Off;
	}
	else if((RPM_ENGINE_CRANK_MIN_THRESHOLD < CarStateInfo.RPM) &&	/* if RPM are higher than minimal threshold and */
			(RPM_ENGINE_CRANK_MAX_THRESHOLD > CarStateInfo.RPM) &&	/* if RPM are lower than maximum threshold */
			(KeyState_Crank == CarStateInfo.keyState))
	{
		CarStateInfo.engineState = EngineState_Crank;
	}
	else if((RPM_ENGINE_IDLE_MIN_THRESHOLD < CarStateInfo.RPM) &&	/* if RPM are higher than minimal threshold and */
			(RPM_ENGINE_IDLE_MAX_THRESHOLD > CarStateInfo.RPM) &&	/* if RPM are lower than maximum threshold and */
			(SPEED_ENGINE_IDLE_MAX_THRESHOLD > CarStateInfo.SPEED))	/* if speed is lower than maximum threshold */
	{
		CarStateInfo.engineState = EngineState_Idle;
	}
	else
	{
		CarStateInfo.engineState = EngineState_Work;
	}
}



/* Check the car state basing on the key and engine status (off, cranking, idling, driving). */
static void Read_CarState(void)
{
	if(EngineState_Off == CarStateInfo.engineState)
	{
		CarStateInfo.carState = CarState_Off;
	}
	/* If key is in crank position OR engine is cranking. */
	else if((KeyState_Crank == CarStateInfo.keyState) || (EngineState_Crank == CarStateInfo.engineState))
	{
		CarStateInfo.carState = CarState_Crank;
	}
	/* If key is in ignition state AND engine is in idle/ */
	else if((KeyState_IgnitionOn == CarStateInfo.keyState) && (EngineState_Idle == CarStateInfo.engineState))
	{
		CarStateInfo.carState = CarState_Idle;
	}
	/* If key is in Ignition state and engine is in work */
	else if((KeyState_IgnitionOn == CarStateInfo.keyState) && (EngineState_Work == CarStateInfo.engineState))
	{
		CarStateInfo.carState = CarState_Drive;
	}
	/* If no previous option matches then something is wrong (for example the key was taken out while engine running. */
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
		valueStates.carWaterTemp_ValueState = val_alarm;	/* Set value in alarm. */
	}
	else if(IsHigher((float)waterTemperatureValue, (float)CAR_waterTemp.waterHighTempWarningThreshold))
	{
		valueStates.carWaterTemp_ValueState = val_warning;	/* Set value in warning. */
	}
	else
	{
		valueStates.carWaterTemp_ValueState = val_OK;	/* Set value in OK state. */
		carWaterTemp_warning.signalBuzzerIndication = FALSE;	/* Set signal indication to FALSE - turn off alarm as the value is OK. */
		carWaterTemp_alarm.signalBuzzerIndication = FALSE;	/* Set signal indication to FALSE - turn off alarm as the value is OK. */
	}
}



/* This function checks whether the current oil temperature in the vehicle is below the thresholds. */
static void Check_CarOilTemp()
{
	if(IsHigher((float)oilTemperatureValue, (float)CAR_oilTemp.oilHighTempAlarmThreshold))
	{
		valueStates.carOilTemp_ValueState = val_alarm;	/* Set value in alarm. */
	}
	else if(IsHigher((float)oilTemperatureValue, (float)CAR_oilTemp.oilHighTempWarningThreshold))
	{
		valueStates.carOilTemp_ValueState = val_warning;	/* Set value in warning. */
	}
	else
	{
		valueStates.carOilTemp_ValueState = val_OK;	/* Set value in OK state. */
		carOilTemp_warning.signalBuzzerIndication = FALSE;	/* Set signal indication to FALSE - turn off alarm as the value is OK. */
		carOilTemp_alarm.signalBuzzerIndication = FALSE;		/* Set signal indication to FALSE - turn off alarm as the value is OK. */
	}
}



/* This function checks whether the current oil pressure in the vehicle is in the thresholds. */
static void Check_CarOilPressure()
{
	if(IsHigher((float)oilPressureValue, (float)CAR_oilPressure.oilHighPressureAlarmThreshold))
	{
		valueStates.carOilAnalogPressure_ValueState = val_alarm;	/* Set value in alarm. */
		valueStatesLowHigh.carOilAnalogPressure_ValueState = val_tooHigh;	/* Set value too high. */
	}
	else if(IsLower((float)oilPressureValue, (float)CAR_oilPressure.oilLowPressureAlarmThreshold))
	{
		valueStates.carOilAnalogPressure_ValueState = val_alarm;	/* Set value in alarm. */
		valueStatesLowHigh.carOilAnalogPressure_ValueState = val_tooLow;	/* Set value too low. */
	}
	else
	{
		valueStates.carOilAnalogPressure_ValueState = val_OK;	/* Set value in OK state. */
		carOilAnalogPressure_alarm.signalBuzzerIndication = FALSE;	/* Set signal indication to FALSE - turn off alarm as the value is OK. */
	}

	if(FALSE == oilPressureValueBinary)
	{
		valueStates.carOilBinaryPressure_ValueState = val_alarm;	/* Set value in alarm. */
	}
	else
	{
		valueStates.carOilBinaryPressure_ValueState = val_OK;	/* Set value in OK state. */
		carOilBinaryPressure_alarm.signalBuzzerIndication = FALSE;	/* Set signal indication to FALSE - turn off alarm as the value is OK. */
	}
}



/* This function checks whether the current battery voltage in the vehicle is in the thresholds. */
static void Check_CarMainBattVoltage()
{
	if(IsHigher((float)mainBatteryVoltageValue, (float)CAR_mainBattery.batteryHighVoltageAlarmThreshold))
	{
		valueStates.carMainBattVoltage_ValueState = val_alarm;	/* Set value in alarm. */
		valueStatesLowHigh.carMainBattVoltage_ValueState = val_tooHigh;	/* Set value too high. */
	}
	else if(IsLower((float)mainBatteryVoltageValue, (float)CAR_mainBattery.batteryLowVoltageAlarmThreshold))
	{
		valueStates.carMainBattVoltage_ValueState = val_alarm;	/* Set value in alarm. */
		valueStatesLowHigh.carMainBattVoltage_ValueState = val_tooLow;	/* Set value too low. */
	}
	else
	{
		valueStates.carMainBattVoltage_ValueState = val_OK;	/* Set value in OK state. */
		carMainBattVoltage_alarm.signalBuzzerIndication = FALSE;	/* Set signal indication to FALSE - turn off alarm as the value is OK. */
	}
}



/* This function checks whether the current battery voltage in the vehicle is in the thresholds. */
static void Check_CarAuxBattVoltage()
{
	if(IsHigher((float)auxiliaryBatteryVoltageValue, (float)CAR_auxiliaryBattery.batteryHighVoltageAlarmThreshold))
	{
		valueStates.carAuxBattVoltage_ValueState = val_alarm;	/* Set value in alarm. */
		valueStatesLowHigh.carAuxBattVoltage_ValueState = val_tooHigh;	/* Set value too high. */
	}
	else if(IsLower((float)auxiliaryBatteryVoltageValue, (float)CAR_auxiliaryBattery.batteryLowVoltageAlarmThreshold))
	{
		valueStates.carAuxBattVoltage_ValueState = val_alarm;	/* Set value in alarm. */
		valueStatesLowHigh.carAuxBattVoltage_ValueState = val_tooLow;	/* Set value too low. */
	}
	else
	{
		valueStates.carAuxBattVoltage_ValueState = val_OK;	/* Set value in OK state. */
		carAuxBattVoltage_alarm.signalBuzzerIndication = FALSE;	/* Set signal indication to FALSE - turn off alarm as the value is OK. */
	}
}



/* This function checks whether the current fuel level in the vehicle is above the warning thresholds. */
static void Check_CarFuelLevel()
{
	if(IsLower((float)fuelLevelValue, (float)CAR_fuel.fuelLowLevelWarningThreshold))
	{
		valueStates.carFuelLevel_ValueState = val_warning;	/* Set value in warning. */
	}
	else
	{
		valueStates.carFuelLevel_ValueState = val_OK;	/* Set value in OK state. */
		carFuelLevel_warning.signalBuzzerIndication = FALSE;	/* Set signal indication to FALSE - turn off alarm as the value is OK. */
	}
}



/* This function checks whether the current voltages on the board are within the thresholds. */
static void Check_BoardVoltage()
{
	/* 3V3 Check */
	if(IsHigher((float)voltage3V3, (float)BOARD_voltage.board3V3SupplyHighThreshold))
	{
		valueStates.board3V3Voltage_ValueState = val_alarm;	/* Set value in alarm. */
		valueStatesLowHigh.board3V3Voltage_ValueState = val_tooHigh;	/* Set value too high. */
	}
	else if(IsLower((float)voltage3V3, (float)BOARD_voltage.board3V3SupplyLowThreshold))
	{
		valueStates.board3V3Voltage_ValueState = val_alarm;	/* Set value in alarm. */
		valueStatesLowHigh.board3V3Voltage_ValueState = val_tooLow;	/* Set value too low. */
	}
	else
	{
		valueStates.board3V3Voltage_ValueState = val_OK;	/* Set value in OK state. */
		board3V3Voltage_alarm.signalBuzzerIndication = FALSE;		/* Set signal indication to FALSE - turn off alarm as the value is OK. */
	}

	/* 5V Check */
	if(IsHigher((float)voltage5V, (float)BOARD_voltage.board5VSupplyHighThreshold))
	{
		valueStates.board5VVoltage_ValueState = val_alarm;	/* Set value in alarm. */
		valueStatesLowHigh.board5VVoltage_ValueState = val_tooHigh;	/* Set value too high. */
	}
	else if(IsLower((float)voltage5V, (float)BOARD_voltage.board5VSupplyLowThreshold))
	{
		valueStates.board5VVoltage_ValueState = val_alarm;	/* Set value in alarm. */
		valueStatesLowHigh.board5VVoltage_ValueState = val_tooLow;	/* Set value too low. */
	}
	else
	{
		valueStates.board5VVoltage_ValueState = val_OK;	/* Set value in OK state. */
		board5VVoltage_alarm.signalBuzzerIndication = FALSE;	/* Set signal indication to FALSE - turn off alarm as the value is OK. */
	}

	/* Vin Check */
	if(IsLower((float)voltageIn, (float)BOARD_voltage.boardVinSupplyLowThreshold))
	{
		valueStates.boardVinVoltage_ValueState = val_alarm;	/* Set value in alarm. */
	}
	else
	{
		valueStates.boardVinVoltage_ValueState = val_OK;	/* Set value in OK state. */
		boardVinVoltage_alarm.signalBuzzerIndication = FALSE;		/* Set signal indication to FALSE - turn off alarm as the value is OK. */
	}

}



/* This function checks whether the current temperatures on the board are within the thresholds. */
static void Check_BoardTemp()
{
	/* 3V3 Check */
	if(IsHigher((float)temperature3V3DCDC, (float)BOARD_temperature.board3V3DCDCTemperatureHighThreshold))
	{
		valueStates.board3V3Temp_ValueState = val_alarm;	/* Set value in alarm. */
	}
	else
	{
		valueStates.board3V3Temp_ValueState = val_OK;	/* Set value in OK state. */
		board3V3Temp_alarm.signalBuzzerIndication = FALSE;	/* Set signal indication to FALSE - turn off alarm as the value is OK. */
	}

	/* 5V Check */
	if(IsHigher((float)temperature5VDCDC, (float)BOARD_temperature.board5VDCDCTemperatureHighThreshold))
	{
		valueStates.board5VTemp_ValueState = val_alarm;	/* Set value in alarm. */
	}
	else
	{
		valueStates.board5VTemp_ValueState = val_OK;	/* Set value in OK state. */
		board5VTemp_alarm.signalBuzzerIndication = FALSE;		/* Set signal indication to FALSE - turn off alarm as the value is OK. */
	}
}



/* This function calls other function to check the values and timers. */
static void CheckValuesStateAndSendAlarmRequests(void)
{
	Error_Code result = NO_ERROR;

	/* Check if the previous result was OK - if yes then check values' states and turn on timers if needed. */
	if(NO_ERROR == result) result = CheckValueAndTimer(valueStates.carWaterTemp_ValueState, My_Timer_carWaterTempValueCheckHandle, CAR_WATER_TEMP_VALUE_CHECK_TIMER_TIME);
	if(NO_ERROR == result) result = CheckValueAndTimer(valueStates.carOilTemp_ValueState, My_Timer_carOilTempValueCheckHandle, CAR_OIL_TEMP_VALUE_CHECK_TIMER_TIME);
	if(NO_ERROR == result) result = CheckValueAndTimer(valueStates.carOilAnalogPressure_ValueState, My_Timer_carOilAnalogPressureValueCheckHandle, CAR_OIL_ANALOG_PRESSURE_VALUE_CHECK_TIMER_TIME);
	if(NO_ERROR == result) result = CheckValueAndTimer(valueStates.carOilBinaryPressure_ValueState, My_Timer_carOilBinaryPressureValueCheckHandle, CAR_OIL_BINARY_PRESSURE_VALUE_CHECK_TIMER_TIME);
	if(NO_ERROR == result) result = CheckValueAndTimer(valueStates.carMainBattVoltage_ValueState, My_Timer_carMainBattVoltageValueCheckHandle, CAR_MAIN_BATT_VOLTAGE_VALUE_CHECK_TIMER_TIME);
	if(NO_ERROR == result) result = CheckValueAndTimer(valueStates.carAuxBattVoltage_ValueState, My_Timer_carAuxBattVoltageValueCheckHandle, CAR_AUX_BATT_VOLTAGE_VALUE_CHECK_TIMER_TIME);
	if(NO_ERROR == result) result = CheckValueAndTimer(valueStates.carFuelLevel_ValueState, My_Timer_carFuelLevelValueCheckHandle, CAR_FUEL_LEVEL_VALUE_CHECK_TIMER_TIME);

	if(NO_ERROR == result) result = CheckValueAndTimer(valueStates.board3V3Voltage_ValueState, My_Timer_board3V3VoltageValueCheckHandle, BOARD_3V3_VOLTAGE_VALUE_CHECK_TIMER_TIME);
	if(NO_ERROR == result) result = CheckValueAndTimer(valueStates.board5VVoltage_ValueState, My_Timer_board5VVoltageValueCheckHandle, BOARD_5V_VOLTAGE_VALUE_CHECK_TIMER_TIME);
	if(NO_ERROR == result) result = CheckValueAndTimer(valueStates.boardVinVoltage_ValueState, My_Timer_boardVinVoltageValueCheckHandle, BOARD_VIN_VOLTAGE_VALUE_CHECK_TIMER_TIME);
	if(NO_ERROR == result) result = CheckValueAndTimer(valueStates.board3V3Temp_ValueState, My_Timer_board3V3TempValueCheckHandle, BOARD_3V3_TEMP_VALUE_CHECK_TIMER_TIME);
	if(NO_ERROR == result) result = CheckValueAndTimer(valueStates.board5VTemp_ValueState, My_Timer_board5VTempValueCheckHandle, BOARD_5V_TEMP_VALUE_CHECK_TIMER_TIME);

	/* Check if there was an error. */
	if(NO_ERROR != result) my_error_handler(result);
}



/* This function checks if the value is OK.
 * If it is not - it checks if the timer was already triggered.
 * If the value is back to OK state and the timer is going - turns off the timer not to trigger an alarm. */
static Error_Code CheckValueAndTimer(Enum_ValueState valState, osTimerId timerId, TickType_t timerSetting)
{
	Error_Code result = NO_ERROR;
	boolean isTimerActive = !xTimerIsTimerActive(timerId); /* if timer in NOT active (hence !) */

	if(val_OK != valState)	/* If the value is not OK */
	{
		if(isTimerActive)	/* Check if timer is running */
		{
			result = (Error_Code)osTimerStart(timerId, timerSetting);	/* Turn timer ON */
		}
	}
	else	/* If the value is OK */
	{
		if(isTimerActive)	/* Check if timer is running */
		{
			result = (Error_Code)osTimerStop(timerId);	/* Turn timer OFF */
		}
	}

	return result;
}



/* Checks if the value is lower than the given threshold. */
static boolean IsLower(float value, float threshold)
{
	if(value <= threshold)
		return TRUE;
	else
		return FALSE;
}
/* Checks if the value is higher than the given threshold. */
static boolean IsHigher(float value, float threshold)
{
	if(value >= threshold)
		return TRUE;
	else
		return FALSE;
}



/* * * * * Setting alarm indication for Task_AlarmControl * * * * */
/* Function for car water temperature timer. */
void Timer_carWaterTempValueCheck(void const * argument)
{
	/* Check if the value state is alarm */
	if(val_alarm == valueStates.carWaterTemp_ValueState)
	{
		carWaterTemp_alarm.signalBuzzerIndication = TRUE;
	}
	/* If value state is not alarm and the timer time passed - it must have been a warning state */
	else
	{
		carWaterTemp_warning.signalBuzzerIndication = TRUE;
	}
}

/* Function for car oil temperature timer. */
void Timer_carOilTempValueCheck(void const * argument)
{
	/* Check if the value state is alarm */
	if(val_alarm == valueStates.carOilTemp_ValueState)
	{
		carOilTemp_alarm.signalBuzzerIndication = TRUE;
	}
	/* If value state is not alarm and the timer time passed - it must have been a warning state */
	else
	{
		carOilTemp_warning.signalBuzzerIndication = TRUE;
	}
}

/* Function for car oil analog pressure timer. */
void Timer_carOilAnalogPressureValueCheck(void const * argument)
{
	/* Set the alarm indication as the timer time has elapsed. */
//	valueStatesLowHigh
	carOilAnalogPressure_alarm.signalBuzzerIndication = TRUE;
}

/* Function for car oil binary pressure timer. */
void Timer_carOilBinaryPressureValueCheck(void const * argument)
{
	/* Set the alarm indication as the timer time has elapsed. */
	carOilBinaryPressure_alarm.signalBuzzerIndication = TRUE;
}

/* Function for car main battery timer. */
void Timer_carMainBattVoltageValueCheck(void const * argument)
{
	/* Set the alarm indication as the timer time has elapsed. */
	carMainBattVoltage_alarm.signalBuzzerIndication = TRUE;
}

/* Function for car aux battery timer. */
void Timer_carAuxBattVoltageValueCheck(void const * argument)
{
	/* Set the alarm indication as the timer time has elapsed. */
	carAuxBattVoltage_alarm.signalBuzzerIndication = TRUE;
}

/* Function for car fuel level timer. */
void Timer_carFuelLevelValueCheck(void const * argument)
{
	/* Set the warning indication as the timer time has elapsed. */
	carFuelLevel_warning.signalBuzzerIndication = TRUE;
}

/* Function for board 3V3 voltage timer. */
void Timer_board3V3VoltageValueCheck(void const * argument)
{
	/* Set the alarm indication as the timer time has elapsed. */
	board3V3Voltage_alarm.signalBuzzerIndication = TRUE;
}

/* Function for board 5V voltage timer. */
void Timer_board5VVoltageValueCheck(void const * argument)
{
	/* Set the alarm indication as the timer time has elapsed. */
	board5VVoltage_alarm.signalBuzzerIndication = TRUE;
}

/* Function for board Vin voltage timer. */
void Timer_boardVinVoltageValueCheck(void const * argument)
{
	/* Set the alarm indication as the timer time has elapsed. */
	boardVinVoltage_alarm.signalBuzzerIndication = TRUE;
}

/* Function for board 3V3 DCDC temperature timer. */
void Timer_board3V3TempValueCheck(void const * argument)
{
	/* Set the alarm indication as the timer time has elapsed. */
	board3V3Temp_alarm.signalBuzzerIndication = TRUE;
}

/* Function for board 5V DCDC temperature timer. */
void Timer_board5VTempValueCheck(void const * argument)
{
	/* Set the alarm indication as the timer time has elapsed. */
	board5VTemp_alarm.signalBuzzerIndication = TRUE;
}

/* Function for board H Bridge temperature timer. */
void Timer_boardHBridgeTempValueCheck(void const * argument)
{
	/* Set the alarm indication as the timer time has elapsed. */
//	board5VTemp_alarm.signalBuzzerIndication = TRUE;
}
