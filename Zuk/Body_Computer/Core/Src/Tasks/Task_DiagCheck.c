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

extern boardVoltage_type voltage3V3;
extern boardVoltage_type voltage5V;
extern boardVoltage_type voltageIn;
extern boardTemperature_type temperature3V3DCDC;
extern boardTemperature_type temperature5VDCDC;

extern CarStateinfo_type CarStateInfo;
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Local variables */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */



void StartTaskDiagCheck(void const * argument)
{
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = MY_DIAG_CHECK_TASK_TIME_PERIOD;
	Error_Code error = NO_ERROR;

	CarStateInfo.keyState = Key_Out;
	CarStateInfo.engineState = EngineState_Off;
	CarStateInfo.AlternatorCharging = FALSE;

	xLastWakeTime = xTaskGetTickCount();

	/* Infinite loop */
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
	for (;;)
	{
		/* Check keyState (Crank, Ignition, Out) */
//		if(GPIO_PIN_SET == HAL_GPIO_ReadPin(KeyCrank_PORT, KeyCrank_PIN))
//		{
//			CarStateInfo.keyState = Key_Crank;
//		}
//		else if(GPIO_PIN_SET == HAL_GPIO_ReadPin(KeyIgnition_PORT, KeyIgnition_PIN))
//		{
//			CarStateInfo.keyState = Key_IgnitionOn;
//		}
//		else
//		{
//			CarStateInfo.keyState = Key_Out;
//		}

		/* Check engineState (Crank, Idle, Off, Work) */
		if(RPM_ENGINE_OFF_MAX_THRESHOLD > CarStateInfo.RPM)
		{
			CarStateInfo.engineState = EngineState_Off;
		}
		else if((RPM_ENGINE_CRANK_MIN_THRESHOLD < CarStateInfo.RPM) &&
				(RPM_ENGINE_CRANK_MAX_THRESHOLD > CarStateInfo.RPM) &&
				(Key_Crank == CarStateInfo.keyState))
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











		vTaskDelayUntil(&xLastWakeTime, xFrequency);
	}

}


















