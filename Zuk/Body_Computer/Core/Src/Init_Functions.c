/*
 * Init_Functions.c
 *
 *  Created on: May 16, 2020
 *      Author: Dell
 */


#include "Init_Functions.h"


/**
 * [WARNING] !!!THIS FUNCTIONS ERASES ALL DATA PERMANENTLY!!!
 * A function that writes 0x00 to every byte of the EEPROM (32k x 8).
 *
 * @param EEPROMParameters: a pointer to the car or board EEPROM
 * 			parameters struct.
 * @retval Error_Code: gives a value of error if one occurs
 */
Error_Code EraseWholeEEPROM(EEPROM_parameters_struct * EEPROMParameters)
{
	Error_Code error = NO_ERROR;
	uint8_t data[64] = {0};
	EEPROM_data_struct EEPROMData = {.EEPROMParameters = EEPROMParameters, .data = data, .memAddress = 0, .memAddressSize = 2, .size = 64};

	for(uint16_t i=0; i<512; ++i)
	{
		EEPROMData.memAddress += 64*i;
		error = WriteEEPROM(EEPROMData.EEPROMParameters, &EEPROMData);
		if(NO_ERROR != error)
		{
			break;
		}
		HAL_Delay(5);
	}

	return error;
}
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */


/**
 * A function that reads all the values saved in EEPROM at start
 * of the BodyComputer (settings, counters etc.)
 *
 * @param void
 * @retval Error_Code: gives a value of error if one occurs
 */
Error_Code InitVariablesFromEEPROMCar(void)
{
	Error_Code error = NO_ERROR;
	EEPROM_data_struct EEPROMData;
	EEPROMData.EEPROMParameters = &EEPROM_car;
	EEPROMData.memAddressSize = 2u;

	union data16bit_union
	{
		uint8_t d8bit[2];
		uint16_t d16bit;
	}data16bit = {};

	uint32_t tempTotalMileage = 0;
	uint32_t tempTripMileage = 0;
	uint8_t tempIndex = 0;

	/** Diagnostic Snapshot EEPROM Index **/
	if(NO_ERROR == error)
	{
		EEPROMData.data = &(CAR_EEPROM_counters.diagnosticSnapshotEEPROMIndex);
		EEPROMData.memAddress = TOTAL_SNAPSHOTS_NUMBER_ADDRESS;
		EEPROMData.size = 1u;
		error = ReadEEPROM(EEPROMData.EEPROMParameters, &EEPROMData);
		HAL_Delay(1);
	}

	/** Diagnostic Snapshot EEPROM Index Overflow **/
	if(NO_ERROR == error)
	{
		EEPROMData.data = &(CAR_EEPROM_counters.didTheNumberOfDiagnosticSnapshotsOverflowed);
		EEPROMData.memAddress = NUMBER_OF_DIAGNOSTIC_SNAPSHOTS_OVERFLOWED_ADDRESS;
		EEPROMData.size = 1u;
		error = ReadEEPROM(EEPROMData.EEPROMParameters, &EEPROMData);
		HAL_Delay(1);
	}

	/** Total Mileage and Trip Mileage, Mileage EEPROM Index **/
	if(NO_ERROR == error)
	{
		EEPROMData.data = CAR_mileage.data;
		EEPROMData.size = 8u;

		for(uint8_t i=0; i< CAR_MILEAGE_TABLE_SIZE; ++i)
		{
			EEPROMData.memAddress = TOTAL_MILEAGE_START_ADDRESS + (i * EEPROM_PAGE_SIZE);
			error = ReadEEPROM(EEPROMData.EEPROMParameters, &EEPROMData);
			HAL_Delay(1);

			if(NO_ERROR != error)
			{
				break;
			}

			if (CAR_mileage.totalMileage > tempTotalMileage)
			{
				tempTotalMileage = CAR_mileage.totalMileage;
				tempTripMileage = CAR_mileage.tripMileage;
				tempIndex = i;
			}
		}
	}
	if(NO_ERROR == error)
	{
		CAR_mileage.totalMileage = tempTotalMileage;
		CAR_mileage.tripMileage = tempTripMileage;
		CAR_EEPROM_counters.mileageEEPROMIndex = tempIndex;
	}

	/** Water Temperature Settings **/
	if(NO_ERROR == error)
	{
		EEPROMData.data = &(CAR_waterTemp.waterHighTempWarningThreshold);
		EEPROMData.memAddress = WATER_HIGH_TEMP_WARNING_THRESHOLD_ADDRESS;
		EEPROMData.size = 1u;
		error = ReadEEPROM(EEPROMData.EEPROMParameters, &EEPROMData);
		HAL_Delay(1);

		EEPROMData.data = &(CAR_waterTemp.waterHighTempAlarmThreshold);
		EEPROMData.memAddress = WATER_HIGH_TEMP_ALARM_THRESHOLD_ADDRESS;
		EEPROMData.size = 1u;
		error = ReadEEPROM(EEPROMData.EEPROMParameters, &EEPROMData);
		HAL_Delay(1);

		EEPROMData.data = &(CAR_waterTemp.waterHighTempFanOnThreshold);
		EEPROMData.memAddress = WATER_HIGH_TEMP_FAN_ON_THRESHOLD_ADDRESS;
		EEPROMData.size = 1u;
		error = ReadEEPROM(EEPROMData.EEPROMParameters, &EEPROMData);
		HAL_Delay(1);

		EEPROMData.data = &(CAR_waterTemp.waterHighTempFanOffThreshold);
		EEPROMData.memAddress = WATER_HIGH_TEMP_FAN_OFF_THRESHOLD_ADDRESS;
		EEPROMData.size = 1u;
		error = ReadEEPROM(EEPROMData.EEPROMParameters, &EEPROMData);
		HAL_Delay(1);

		EEPROMData.data = &(CAR_waterTemp.allSettings);
		EEPROMData.memAddress = WATER_TEMP_ALL_SETTINGS_ADDRESS;
		EEPROMData.size = 1u;
		error = ReadEEPROM(EEPROMData.EEPROMParameters, &EEPROMData);
		HAL_Delay(1);
	}

	/** Oil Temperature Settings **/
	if(NO_ERROR == error)
	{
		EEPROMData.data = &(CAR_oilTemp.oilHighTempWarningThreshold);
		EEPROMData.memAddress = OIL_HIGH_TEMP_WARNING_THRESHOLD_ADDRESS;
		EEPROMData.size = 1u;
		error = ReadEEPROM(EEPROMData.EEPROMParameters, &EEPROMData);
		HAL_Delay(1);

		EEPROMData.data = &(CAR_oilTemp.oilHighTempAlarmThreshold);
		EEPROMData.memAddress = OIL_HIGH_TEMP_ALARM_THRESHOLD_ADDRESS;
		EEPROMData.size = 1u;
		error = ReadEEPROM(EEPROMData.EEPROMParameters, &EEPROMData);
		HAL_Delay(1);

		EEPROMData.data = &(CAR_oilTemp.allSettings);
		EEPROMData.memAddress = OIL_TEMP_ALL_SETTINGS_ADDRESS;
		EEPROMData.size = 1u;
		error = ReadEEPROM(EEPROMData.EEPROMParameters, &EEPROMData);
		HAL_Delay(1);
	}

	/** Main Battery Settings **/
	if(NO_ERROR == error)
	{
		EEPROMData.data = data16bit.d8bit;
		EEPROMData.memAddress = MAIN_BATTERY_LOW_VOLTAGE_ALARM_THRESHOLD_ADDRESS;
		EEPROMData.size = 1u;
		error = ReadEEPROM(EEPROMData.EEPROMParameters, &EEPROMData);
		HAL_Delay(1);
		CAR_mainBattery.batteryLowVoltageAlarmThreshold = data16bit.d16bit;

		EEPROMData.data = data16bit.d8bit;
		EEPROMData.memAddress = MAIN_BATTERY_HIGH_VOLTAGE_ALARM_THRESHOLD_ADDRESS;
		EEPROMData.size = 1u;
		error = ReadEEPROM(EEPROMData.EEPROMParameters, &EEPROMData);
		HAL_Delay(1);
		CAR_mainBattery.batteryHighVoltageAlarmThreshold = data16bit.d16bit;

		EEPROMData.data = data16bit.d8bit;
		EEPROMData.memAddress = MAIN_BATTERY_ALL_SETTINGS_ADDRESS;
		EEPROMData.size = 1u;
		error = ReadEEPROM(EEPROMData.EEPROMParameters, &EEPROMData);
		HAL_Delay(1);
		CAR_mainBattery.allSettings = data16bit.d16bit;
	}

	/** Auxiliary Battery Settings **/
	if(NO_ERROR == error)
	{
		EEPROMData.data = data16bit.d8bit;
		EEPROMData.memAddress = AUXILIARY_BATTERY_LOW_VOLTAGE_ALARM_THRESHOLD_ADDRESS;
		EEPROMData.size = 1u;
		error = ReadEEPROM(EEPROMData.EEPROMParameters, &EEPROMData);
		HAL_Delay(1);
		CAR_auxiliaryBattery.batteryLowVoltageAlarmThreshold = data16bit.d16bit;

		EEPROMData.data = data16bit.d8bit;
		EEPROMData.memAddress = AUXILIARY_BATTERY_HIGH_VOLTAGE_ALARM_THRESHOLD_ADDRESS;
		EEPROMData.size = 1u;
		error = ReadEEPROM(EEPROMData.EEPROMParameters, &EEPROMData);
		HAL_Delay(1);
		CAR_auxiliaryBattery.batteryLowVoltageAlarmThreshold = data16bit.d16bit;

		EEPROMData.data = data16bit.d8bit;
		EEPROMData.memAddress = AUXILIARY_BATTERY_ALL_SETTINGS_ADDRESS;
		EEPROMData.size = 1u;
		error = ReadEEPROM(EEPROMData.EEPROMParameters, &EEPROMData);
		HAL_Delay(1);
		CAR_auxiliaryBattery.batteryLowVoltageAlarmThreshold = data16bit.d16bit;
	}

	/** Fuel settings **/
	if(NO_ERROR == error)
	{
		EEPROMData.data = &(CAR_fuel.fuelLowLevelWarningThreshold);
		EEPROMData.memAddress = FUEL_LOW_LEVEL_WARNING_THRESHOLD_ADDRESS;
		EEPROMData.size = 1u;
		error = ReadEEPROM(EEPROMData.EEPROMParameters, &EEPROMData);
		HAL_Delay(1);

		EEPROMData.data = &(CAR_fuel.allSettings);
		EEPROMData.memAddress = FUEL_ALL_SETTINGS_ADDRESS;
		EEPROMData.size = 1u;
		error = ReadEEPROM(EEPROMData.EEPROMParameters, &EEPROMData);
		HAL_Delay(1);
	}

	/** Oil Pressure settings **/
	if(NO_ERROR == error)
	{
		EEPROMData.data = &(CAR_oilPressure.oilHighPressureAlarmThreshold);
		EEPROMData.memAddress = OIL_HIGH_PRESSURE_ALARM_THRESHOLD_ADDRESS;
		EEPROMData.size = 1u;
		error = ReadEEPROM(EEPROMData.EEPROMParameters, &EEPROMData);
		HAL_Delay(1);

		EEPROMData.data = &(CAR_oilPressure.oilLowPressureAlarmThreshold);
		EEPROMData.memAddress = OIL_LOW_PRESSURE_ALARM_THRESHOLD_ADDRESS;
		EEPROMData.size = 1u;
		error = ReadEEPROM(EEPROMData.EEPROMParameters, &EEPROMData);
		HAL_Delay(1);

		EEPROMData.data = &(CAR_oilPressure.allSettings);
		EEPROMData.memAddress = OIL_PRESSURE_ALL_SETTINGS_ADDRESS;
		EEPROMData.size = 1u;
		error = ReadEEPROM(EEPROMData.EEPROMParameters, &EEPROMData);
		HAL_Delay(1);
	}



	return error;
}
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */













