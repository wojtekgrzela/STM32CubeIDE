/*
 * Task_250ms.c
 *
 *  Created on: Feb 28, 2021
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
#include "../../lcd_hd44780_i2c/lcd_hd44780_i2c.h"
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Defines And Typedefs */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

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

extern LCD_message mainBatteryVoltageValueForLCD;
extern LCD_message auxiliaryBatteryVoltageValueForLCD;
extern LCD_message waterTemperatureValueForLCD;
extern LCD_message totalMileageForLCD;
extern LCD_message tripMileageForLCD;
extern LCD_message fuelLevelValueForLCD;
extern LCD_message oilPressureValueBinaryForLCD;
extern LCD_message oilPressureValueForLCD;
extern LCD_message oilTemperatureValueForLCD;

extern LCD_message voltage3V3ForLCD;
extern LCD_message voltage5VForLCD;
extern LCD_message voltageInForLCD;
extern LCD_message temperature3V3DCDCForLCD;
extern LCD_message temperature5VDCDCForLCD;

extern volatile uint16_t ADC1Measures[NO_OF_ADC1_MEASURES];
extern volatile uint16_t ADC3Measures[NO_OF_ADC3_MEASURES];

extern CAR_mileage_struct CAR_mileage;

extern volatile ENCButton_struct ENC_button_menu;
extern volatile int8_t EncoderCounterDiff;
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Local variables */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Functions prototypes */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */



void Start250msTask(void const *argument)
{
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = MY_TASK_250_MS_TIME_PERIOD;
	Error_Code error = NO_ERROR;
	uint8_t firstFewRuns = 0u;

	/* Stop My_DiagCheck Task because measurements are not ready yet! */
//	vTaskSuspend(My_DiagCheckHandle);

	/* For calculating the engine parameters (counting the mean value) */
	static volatile carTemperature_type EngineWaterTemperatureTable[NO_OF_ENGINE_WATER_TEMP_MEASUREMENTS_ADDED] =
		{ 0 };
	static volatile carTemperature_type EngineWaterTemperatureMovingAverage[NO_OF_ENGINE_WATER_TEMP_MEASUREMENTS_ADDED] =
		{ 0 };
	static uint8_t i_waterTemp = 0;
	static carTemperature_type tempEngineWaterTemp = 0.0;
	carTemperature_type waterTemperatureValue_beforeMovingAverage = 0.0;
	static uint8_t waterTemperatureValueMessage[4] = "";
	waterTemperatureValueForLCD.messageHandler = waterTemperatureValueMessage;

	static volatile carTemperature_type EngineOilTemperatureTable[NO_OF_ENGINE_OIL_TEMP_MEASUREMENTS_ADDED] =
		{ 0 };
	static volatile carTemperature_type EngineOilTemperatureTableMovingAverage[NO_OF_ENGINE_OIL_TEMP_MEASUREMENTS_ADDED] =
		{ 0 };
	static uint8_t i_oilTemp = 0;
	static carTemperature_type tempEngineOilTemp = 0.0;
	carTemperature_type oilTemperatureValue_beforeMovingAverage = 0.0;
	static uint8_t oilTemperatureValueMessage[4] = "";
	oilTemperatureValueForLCD.messageHandler = oilTemperatureValueMessage;

#ifdef OIL_PRESSURE_ANALOG_SENSOR
	static volatile carOilAnalogPressure_type EngineOilPressureTable[NO_OF_ENGINE_OIL_PRESSURE_MEASUREMENTS_ADDED] = {0};
//	static volatile float EngineOilPressureTableMovingAverage[NO_OF_ENGINE_OIL_PRESSURE_MEASUREMENTS_ADDED] = {0};
	static uint8_t i_oilPressure = 0;
	static carOilAnalogPressure_type tempEngineOilPressure = 0.0;
	static uint8_t oilPressureValueMessage[4] = "";
	oilPressureValueForLCD.messageHandler = oilPressureValueMessage;
#endif

#ifdef OIL_PRESSURE_BINARY_SENSOR
	static uint8_t oilPressureValueBinaryMessage[4] = "";	//OK, NOK
	oilPressureValueBinaryForLCD.messageHandler = oilPressureValueBinaryMessage;
#endif

	static volatile carVoltage_type MainBatteryVoltageValueTable[NO_OF_MAIN_BATTERY_VOLTAGE_MEASUREMENTS_ADDED] =
		{ 0 };
//	static volatile float MainBatteryVoltageValueTableMovingAverage[NO_OF_MAIN_BATTERY_VOLTAGE_MEASUREMENTS_ADDED] = {0};
	static uint8_t i_mainBateryVoltage = 0;
	static carVoltage_type tempMainBatteryVoltage = 0.0;
	static uint8_t mainBatteryVoltageValueMessage[6] = "";
	mainBatteryVoltageValueForLCD.messageHandler = mainBatteryVoltageValueMessage;

	static volatile carVoltage_type AuxiliaryBatteryVoltageValueTable[NO_OF_AUXILIARY_BATTERY_VOLTAGE_MEASUREMENTS_ADDED] =
		{ 0 };
//	static volatile float AuxiliaryBatteryVoltageValueTableMovingAverage[NO_OF_AUXILIARY_BATTERY_VOLTAGE_MEASUREMENTS_ADDED] = {0};
	static uint8_t i_auxBatteryVoltage = 0;
	static carVoltage_type tempAuxBatteryVoltage = 0.0;
	static uint8_t auxiliaryBatteryVoltageValueMessage[6] = "";
	auxiliaryBatteryVoltageValueForLCD.messageHandler = auxiliaryBatteryVoltageValueMessage;

	static volatile cafFuelLevel_type FuelLevelValueTable[NO_OF_FUEL_LEVEL_MEASUREMENTS_ADDED] =
		{ 0 };
	static volatile cafFuelLevel_type FuelLevelValueTableMovingAverage[NO_OF_FUEL_LEVEL_MEASUREMENTS_ADDED] =
		{ 0 };
	static uint8_t i_fuelLevel = 0;
	static cafFuelLevel_type tempFuelLevel = 0.0;
	cafFuelLevel_type fuelLevelValue_beforeMovingAverage = 0.0;
	static uint8_t fuelLevelValueMessage[3] = "";
	fuelLevelValueForLCD.messageHandler = fuelLevelValueMessage;

	xLastWakeTime = xTaskGetTickCount();

	/* Infinite loop */
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
	for (;;)
	{
		/* Water Temperature Value measurement from LM35 */
		if (NO_OF_ENGINE_WATER_TEMP_MEASUREMENTS_ADDED > i_waterTemp)
		{
			error = calculate_LM35_temperature((float*)&tempEngineWaterTemp, ADC1Measures[0]);

			waterTemperatureValue = tempEngineWaterTemp;

			waterTemperatureValueForLCD.messageReadyFLAG = FALSE;
			snprintf((char*)waterTemperatureValueForLCD.messageHandler, 4, "%3" PRIu16, (uint16_t)waterTemperatureValue);
			waterTemperatureValueForLCD.size = strlen(
					(char*)waterTemperatureValueForLCD.messageHandler);
			waterTemperatureValueForLCD.messageReadyFLAG = TRUE;

			tempEngineWaterTemp = 0;
		}

		if (NO_ERROR != error)
		{
			my_error_handler(error);
		}

		/* Oil Temperature Value measurement */
		if (NO_OF_ENGINE_OIL_TEMP_MEASUREMENTS_ADDED > i_oilTemp)
		{
			error = calculate_LM35_temperature((float*)&(EngineOilTemperatureTable[i_oilTemp]),
					ADC1Measures[0]);
			tempEngineOilTemp += EngineOilTemperatureTable[i_oilTemp];
			i_oilTemp += 1;
		}
		else
		{
			oilTemperatureValue_beforeMovingAverage = tempEngineOilTemp
					/ NO_OF_ENGINE_OIL_TEMP_MEASUREMENTS_ADDED;
			EngineOilTemperatureTableMovingAverage[NO_OF_ENGINE_OIL_TEMP_MEASUREMENTS_ADDED - 1] =
					oilTemperatureValue_beforeMovingAverage;
			tempEngineOilTemp = oilTemperatureValue_beforeMovingAverage;

			for (uint8_t j = 0; j < (NO_OF_ENGINE_OIL_TEMP_MEASUREMENTS_ADDED - 1); ++j)
			{
				EngineOilTemperatureTableMovingAverage[j] = EngineOilTemperatureTableMovingAverage[j
						+ 1];
				tempEngineOilTemp += EngineOilTemperatureTableMovingAverage[j];
			}

			oilTemperatureValue = tempEngineOilTemp / NO_OF_ENGINE_OIL_TEMP_MEASUREMENTS_ADDED;

			oilTemperatureValueForLCD.messageReadyFLAG = FALSE;
			snprintf((char*)oilTemperatureValueForLCD.messageHandler, 4, "%3" PRIu16, (uint16_t)oilTemperatureValue);
			oilTemperatureValueForLCD.size = strlen(
					(char*)oilTemperatureValueForLCD.messageHandler);
			oilTemperatureValueForLCD.messageReadyFLAG = TRUE;

			tempEngineOilTemp = 0;
			i_oilTemp = 0;
		}

		if (NO_ERROR != error)
		{
			my_error_handler(error);
		}

		/* Oil Pressure Value measurement */
#ifdef OIL_PRESSURE_ANALOG_SENSOR
		if(NO_OF_ENGINE_OIL_PRESSURE_MEASUREMENTS_ADDED > i_oilPressure)
		{
			error = calculate_EngineOilPressure((float*)&(EngineOilPressureTable[i_oilPressure]), ADC1Measures[0]);
			tempEngineOilPressure += EngineOilPressureTable[i_oilPressure];
			i_oilPressure += 1;
		}
		else
		{
			oilPressureValue = tempEngineOilPressure / NO_OF_ENGINE_OIL_PRESSURE_MEASUREMENTS_ADDED;

			oilPressureValueForLCD.messageReadyFLAG = FALSE;
			snprintf((char*)oilPressureValueForLCD.messageHandler, 4, "%01" PRIu16 ".%01" PRIu16, (uint16_t)oilPressureValue, (uint16_t)(oilPressureValue*10)%10);
			oilPressureValueForLCD.size = strlen((char*)oilPressureValueForLCD.messageHandler);
			oilPressureValueForLCD.messageReadyFLAG = TRUE;

			tempEngineOilPressure = 0;
			i_oilPressure = 0;
		}
#endif

#ifdef OIL_PRESSURE_BINARY_SENSOR
//		if(GPIO_PIN_RESET == HAL_GPIO_ReadPin(GPIO_PORT_OIL_PRESSURE_SENSOR, GPIO_PIN_OIL_PRESSURE_SENSOR))
//		{
//			oilPressureValueBinary = OK;
//			oilPressureValueBinaryForLCD.messageReadyFLAG = FALSE;
//			snprintf((char*)oilPressureValueBinaryForLCD.messageHandler, 4, "OK");
//			oilPressureValueBinaryForLCD.size = strlen((char*)oilPressureValueForLCD.messageHandler);
//			oilPressureValueBinaryForLCD.messageReadyFLAG = TRUE;
//		}
//		else
//		{
//			oilPressureValueBinary = NOK;
//			oilPressureValueBinaryForLCD.messageReadyFLAG = FALSE;
//			snprintf((char*)oilPressureValueBinaryForLCD.messageHandler, 4, "NOK");
//			oilPressureValueBinaryForLCD.size = strlen((char*)oilPressureValueForLCD.messageHandler);
//			oilPressureValueBinaryForLCD.messageReadyFLAG = TRUE;
//		}
#endif

		if (NO_ERROR != error)
		{
			my_error_handler(error);
		}

		/* Main Battery Voltage Value measurement */
		if (NO_OF_MAIN_BATTERY_VOLTAGE_MEASUREMENTS_ADDED > i_mainBateryVoltage)
		{
			for (uint8_t j = 1; j < (NO_OF_MAIN_BATTERY_VOLTAGE_MEASUREMENTS_ADDED); ++j)
			{
				MainBatteryVoltageValueTable[j] = MainBatteryVoltageValueTable[j - 1];
				tempMainBatteryVoltage += MainBatteryVoltageValueTable[j];
			}

			error = calculate_voltage((float*)&(MainBatteryVoltageValueTable[0u]), ADC1Measures[0],
					MAIN_BATTERY_VOLTAGE_DIVIDER);
			tempMainBatteryVoltage += MainBatteryVoltageValueTable[0u];
			mainBatteryVoltageValue = tempMainBatteryVoltage
					/ NO_OF_MAIN_BATTERY_VOLTAGE_MEASUREMENTS_ADDED;

			mainBatteryVoltageValueForLCD.messageReadyFLAG = FALSE;
			snprintf((char*)mainBatteryVoltageValueForLCD.messageHandler, 6, "%02" PRIu16 ".%02" PRIu16, (uint16_t)mainBatteryVoltageValue, (uint16_t)(mainBatteryVoltageValue*100)%100);
			mainBatteryVoltageValueForLCD.size = strlen(
					(char*)mainBatteryVoltageValueForLCD.messageHandler);
			mainBatteryVoltageValueForLCD.messageReadyFLAG = TRUE;

			i_mainBateryVoltage += 1;
			tempMainBatteryVoltage = 0;
		}
		else
		{
			tempMainBatteryVoltage = 0;
			i_mainBateryVoltage = 0;
		}

		if (NO_ERROR != error)
		{
			my_error_handler(error);
		}

		/* Auxiliary Battery Voltage Value measurement */
		if (NO_OF_AUXILIARY_BATTERY_VOLTAGE_MEASUREMENTS_ADDED > i_auxBatteryVoltage)
		{
			for (uint8_t j = 1; j < (NO_OF_AUXILIARY_BATTERY_VOLTAGE_MEASUREMENTS_ADDED); ++j)
			{
				AuxiliaryBatteryVoltageValueTable[j] = AuxiliaryBatteryVoltageValueTable[j - 1];
				tempAuxBatteryVoltage += AuxiliaryBatteryVoltageValueTable[j];
			}

			error = calculate_voltage((float*)&(AuxiliaryBatteryVoltageValueTable[0u]),
					ADC1Measures[0], AUXILIARY_BATTERY_VOLTAGE_DIVIDER);
			tempAuxBatteryVoltage += AuxiliaryBatteryVoltageValueTable[0u];
			auxiliaryBatteryVoltageValue = tempAuxBatteryVoltage
					/ NO_OF_AUXILIARY_BATTERY_VOLTAGE_MEASUREMENTS_ADDED;

			auxiliaryBatteryVoltageValueForLCD.messageReadyFLAG = FALSE;
			snprintf((char*)auxiliaryBatteryVoltageValueForLCD.messageHandler, 6, "%02" PRIu16 ".%02" PRIu16, (uint16_t)auxiliaryBatteryVoltageValue, (uint16_t)(auxiliaryBatteryVoltageValue*100)%100);
			auxiliaryBatteryVoltageValueForLCD.size = strlen(
					(char*)auxiliaryBatteryVoltageValueForLCD.messageHandler);
			auxiliaryBatteryVoltageValueForLCD.messageReadyFLAG = TRUE;

			i_auxBatteryVoltage += 1;
			tempAuxBatteryVoltage = 0;
		}
		else
		{
			tempAuxBatteryVoltage = 0;
			i_auxBatteryVoltage = 0;
		}

		if (NO_ERROR != error)
		{
			my_error_handler(error);
		}

		/* Fuel Level Value measurement */
		if (NO_OF_FUEL_LEVEL_MEASUREMENTS_ADDED > i_fuelLevel)
		{
			error = calculate_fuelLevel((float*)&(FuelLevelValueTable[i_fuelLevel]),
					ADC1Measures[0]);
			tempFuelLevel += FuelLevelValueTable[i_fuelLevel];
			i_fuelLevel += 1;
		}
		else
		{
			fuelLevelValue_beforeMovingAverage = tempFuelLevel
					/ NO_OF_FUEL_LEVEL_MEASUREMENTS_ADDED;
			FuelLevelValueTableMovingAverage[NO_OF_FUEL_LEVEL_MEASUREMENTS_ADDED - 1] =
					fuelLevelValue_beforeMovingAverage;
			tempFuelLevel = fuelLevelValue_beforeMovingAverage;

			for (uint8_t j = 0; j < (NO_OF_FUEL_LEVEL_MEASUREMENTS_ADDED - 1); ++j)
			{
				FuelLevelValueTableMovingAverage[j] = FuelLevelValueTableMovingAverage[j + 1];
				tempFuelLevel += FuelLevelValueTableMovingAverage[j];
			}

			fuelLevelValue = tempFuelLevel / NO_OF_FUEL_LEVEL_MEASUREMENTS_ADDED;

			fuelLevelValueForLCD.messageReadyFLAG = FALSE;
			snprintf((char*)fuelLevelValueForLCD.messageHandler, 4, "%3" PRIu16, (uint16_t)fuelLevelValue);
			fuelLevelValueForLCD.size = strlen((char*)fuelLevelValueForLCD.messageHandler);
			fuelLevelValueForLCD.messageReadyFLAG = TRUE;

			tempFuelLevel = 0;
			i_fuelLevel = 0;
		}

		if (NO_ERROR != error)
		{
			my_error_handler(error);
		}

		/* Release My_DiagCheck Task after some measurements first */
//		(NUMBER_OF_MEASUREMENTS_BEFORE_DIAGNOSTIC <= firstFewRuns) ?
//				vTaskResume(My_DiagCheckHandle) : ++firstFewRuns;

		vTaskDelayUntil(&xLastWakeTime, xFrequency);
	}
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
}
