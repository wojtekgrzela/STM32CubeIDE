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
 * A function that reads all the values saved in Car EEPROM at start
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

	data32bit_union data_union = {0};

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

	/** Main Battery Settings **/
	if(NO_ERROR == error)
	{
		EEPROMData.data = data_union.u8bit;
		EEPROMData.memAddress = MAIN_BATTERY_LOW_VOLTAGE_ALARM_THRESHOLD_ADDRESS;
		EEPROMData.size = 4u;
		error = ReadEEPROM(EEPROMData.EEPROMParameters, &EEPROMData);
		HAL_Delay(1);
		CAR_mainBattery.batteryLowVoltageAlarmThreshold = data_union.f32bit;

		EEPROMData.data = data_union.u8bit;
		EEPROMData.memAddress = MAIN_BATTERY_HIGH_VOLTAGE_ALARM_THRESHOLD_ADDRESS;
		EEPROMData.size = 4u;
		error = ReadEEPROM(EEPROMData.EEPROMParameters, &EEPROMData);
		HAL_Delay(1);
		CAR_mainBattery.batteryHighVoltageAlarmThreshold = data_union.f32bit;

		EEPROMData.data = &(CAR_mainBattery.allSettings);
		EEPROMData.memAddress = MAIN_BATTERY_ALL_SETTINGS_ADDRESS;
		EEPROMData.size = 1u;
		error = ReadEEPROM(EEPROMData.EEPROMParameters, &EEPROMData);
		HAL_Delay(1);
	}

	/** Auxiliary Battery Settings **/
	if(NO_ERROR == error)
	{
		EEPROMData.data = data_union.u8bit;
		EEPROMData.memAddress = AUXILIARY_BATTERY_LOW_VOLTAGE_ALARM_THRESHOLD_ADDRESS;
		EEPROMData.size = 4u;
		error = ReadEEPROM(EEPROMData.EEPROMParameters, &EEPROMData);
		HAL_Delay(1);
		CAR_auxiliaryBattery.batteryLowVoltageAlarmThreshold = data_union.f32bit;

		EEPROMData.data = data_union.u8bit;
		EEPROMData.memAddress = AUXILIARY_BATTERY_HIGH_VOLTAGE_ALARM_THRESHOLD_ADDRESS;
		EEPROMData.size = 4u;
		error = ReadEEPROM(EEPROMData.EEPROMParameters, &EEPROMData);
		HAL_Delay(1);
		CAR_auxiliaryBattery.batteryHighVoltageAlarmThreshold = data_union.f32bit;

		EEPROMData.data = &(CAR_auxiliaryBattery.allSettings);
		EEPROMData.memAddress = AUXILIARY_BATTERY_ALL_SETTINGS_ADDRESS;
		EEPROMData.size = 1u;
		error = ReadEEPROM(EEPROMData.EEPROMParameters, &EEPROMData);
		HAL_Delay(1);
	}


	return error;
}
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */



/**
 * A function that reads all the values saved in Board EEPROM at start
 * of the BodyComputer (settings, counters etc.)
 *
 * @param void
 * @retval Error_Code: gives a value of error if one occurs
 */
Error_Code InitVariablesFromEEPROMBoard(void)
{
	Error_Code error = NO_ERROR;
	EEPROM_data_struct EEPROMData;
	EEPROMData.EEPROMParameters = &EEPROM_board;
	EEPROMData.memAddressSize = 2u;

	data32bit_union data_union = {0};


	/** Error Snapshot EEPROM Index **/
	if(NO_ERROR == error)
	{
		EEPROMData.data = &(BOARD_EEPROM_counters.errorSnapshotEEPROMIndex);
		EEPROMData.memAddress = NUMBER_OF_ERROR_SNAPSHOTS;
		EEPROMData.size = 1u;
		error = ReadEEPROM(EEPROMData.EEPROMParameters, &EEPROMData);
		HAL_Delay(1);
	}

	/** Error Snapshot EEPROM Index Overflow **/
	if(NO_ERROR == error)
	{
		EEPROMData.data = &(BOARD_EEPROM_counters.didTheNumberOfErrorSnapshotsOverflowed);
		EEPROMData.memAddress = NUMBER_OF_ERROR_SNAPSHOTS_OVERFLOWED_ADDRESS;
		EEPROMData.size = 1u;
		error = ReadEEPROM(EEPROMData.EEPROMParameters, &EEPROMData);
		HAL_Delay(1);
	}

	/** GPS and Time settings **/
	if(NO_ERROR == error)
	{
		EEPROMData.data = data_union.u8bit;
		EEPROMData.memAddress = HOME_LATITUDE_ADDRESS;
		EEPROMData.size = 4u;
		error = ReadEEPROM(EEPROMData.EEPROMParameters, &EEPROMData);
		HAL_Delay(1);
		GPS.homeLatitude = data_union.f32bit;

		EEPROMData.data = data_union.u8bit;
		EEPROMData.memAddress = HOME_LONGITUDE_ADDRESS;
		EEPROMData.size = 4u;
		error = ReadEEPROM(EEPROMData.EEPROMParameters, &EEPROMData);
		HAL_Delay(1);
		GPS.homeLongitude= data_union.f32bit;

		EEPROMData.data = &(GPS.TimeZoneAdjPoland);
		EEPROMData.memAddress = TIME_ZONE_ADJ_POLAND_ADDRESS;
		EEPROMData.size = 1u;
		error = ReadEEPROM(EEPROMData.EEPROMParameters, &EEPROMData);
		HAL_Delay(1);

		EEPROMData.data = &TimeZoneManualAdj;
		EEPROMData.memAddress = TIME_MANUAL_ADJUSTMENT_ADDRESS;
		EEPROMData.size = 1u;
		error = ReadEEPROM(EEPROMData.EEPROMParameters, &EEPROMData);
		HAL_Delay(1);
	}

	/** Board voltages settings **/
	if(NO_ERROR == error)
	{
		EEPROMData.data = data_union.u8bit;
		EEPROMData.memAddress = BOARD_5V_SUPPLY_LOW_THRESHOLD_ADDRESS;
		EEPROMData.size = 4u;
		error = ReadEEPROM(EEPROMData.EEPROMParameters, &EEPROMData);
		HAL_Delay(1);
		BOARD_voltage.board5VSupplyLowThreshold = data_union.f32bit;

		EEPROMData.data = data_union.u8bit;
		EEPROMData.memAddress = BOARD_5V_SUPPLY_HIGH_THRESHOLD_ADDRESS;
		EEPROMData.size = 4u;
		error = ReadEEPROM(EEPROMData.EEPROMParameters, &EEPROMData);
		HAL_Delay(1);
		BOARD_voltage.board5VSupplyHighThreshold = data_union.f32bit;

		EEPROMData.data = data_union.u8bit;
		EEPROMData.memAddress = BOARD_3V3_SUPPLY_LOW_THRESHOLD_ADDRESS;
		EEPROMData.size = 4u;
		error = ReadEEPROM(EEPROMData.EEPROMParameters, &EEPROMData);
		HAL_Delay(1);
		BOARD_voltage.board3V3SupplyLowThreshold = data_union.f32bit;

		EEPROMData.data = data_union.u8bit;
		EEPROMData.memAddress = BOARD_3V3_SUPPLY_HIGH_THRESHOLD_ADDRESS;
		EEPROMData.size = 4u;
		error = ReadEEPROM(EEPROMData.EEPROMParameters, &EEPROMData);
		HAL_Delay(1);
		BOARD_voltage.board3V3SupplyHighThreshold = data_union.f32bit;

		EEPROMData.data = data_union.u8bit;
		EEPROMData.memAddress = BOARD_VIN_SUPPLY_LOW_THRESHOLD_ADDRESS;
		EEPROMData.size = 4u;
		error = ReadEEPROM(EEPROMData.EEPROMParameters, &EEPROMData);
		HAL_Delay(1);
		BOARD_voltage.boardVinSupplyLowThreshold = data_union.f32bit;

		EEPROMData.data = &(BOARD_voltage.allSettings);
		EEPROMData.memAddress = BOARD_VOLTAGE_ALL_SETTINGS_ADDRESS;
		EEPROMData.size = 1u;
		error = ReadEEPROM(EEPROMData.EEPROMParameters, &EEPROMData);
		HAL_Delay(1);
	}

	/** Board temperatures settings **/
	if(NO_ERROR == error)
	{
		EEPROMData.data = data_union.u8bit;
		EEPROMData.memAddress = BOARD_5V_TEMPERATURE_THRESHOLD_ADDRESS;
		EEPROMData.size = 4u;
		error = ReadEEPROM(EEPROMData.EEPROMParameters, &EEPROMData);
		HAL_Delay(1);
		BOARD_temperature.board5VDCDCTemperatureHighThreshold = data_union.f32bit;

		EEPROMData.data = data_union.u8bit;
		EEPROMData.memAddress = BOARD_3V3_TEMPERATURE_THRESHOLD_ADDRESS;
		EEPROMData.size = 4u;
		error = ReadEEPROM(EEPROMData.EEPROMParameters, &EEPROMData);
		HAL_Delay(1);
		BOARD_temperature.board3V3DCDCTemperatureHighThreshold = data_union.f32bit;

		EEPROMData.data = &(BOARD_temperature.allSettings);
		EEPROMData.memAddress = BOARD_TEMPERATURE_ALL_SETTINGS_ADDRESS;
		EEPROMData.size = 1u;
		error = ReadEEPROM(EEPROMData.EEPROMParameters, &EEPROMData);
		HAL_Delay(1);
	}

	/** Board buzzer settings **/
	if(NO_ERROR == error)
	{
		EEPROMData.data = &(BUZZER_settings.allSettings);
		EEPROMData.memAddress = BOARD_BUZZER_ALL_SETTINGS_ADDRESS;
		EEPROMData.size = 1u;
		error = ReadEEPROM(EEPROMData.EEPROMParameters, &EEPROMData);
		HAL_Delay(1);
	}

	/** Board LCD settings **/
	if(NO_ERROR == error)
	{
		EEPROMData.data = data_union.u8bit;
		EEPROMData.memAddress = BOARD_LCD_HOME_SCREEN_ADDRESS;
		EEPROMData.size = 4u;
		error = ReadEEPROM(EEPROMData.EEPROMParameters, &EEPROMData);
		HAL_Delay(1);
		LCD_MainSettings.homeScreen = data_union.u32bit;

		EEPROMData.data = &(LCD_MainSettings.autoHomeReturnTime);
		EEPROMData.memAddress = BOARD_LCD_AUTO_HOME_RETURN_TIME_ADDRESS;
		EEPROMData.size = 1u;
		error = ReadEEPROM(EEPROMData.EEPROMParameters, &EEPROMData);
		HAL_Delay(1);

		EEPROMData.data = &(LCD_MainSettings.backlightLevel);
		EEPROMData.memAddress = BOARD_LCD_BACKLIGHT_LEVEL_ADDRESS;
		EEPROMData.size = 1u;
		error = ReadEEPROM(EEPROMData.EEPROMParameters, &EEPROMData);
		HAL_Delay(1);

		EEPROMData.data = &(LCD_MainSettings.secondsToAutoTurnOffBacklight);
		EEPROMData.memAddress = BOARD_LCD_SECONDS_TO_AUTO_TURN_OFF_BACKLIGHT_ADDRESS;
		EEPROMData.size = 1u;
		error = ReadEEPROM(EEPROMData.EEPROMParameters, &EEPROMData);
		HAL_Delay(1);

		EEPROMData.data = &(LCD_MainSettings.autoBacklightOffHourStart);
		EEPROMData.memAddress = BOARD_LCD_AUTO_BACKLIGHT_OFF_HOUR_START_ADDRESS;
		EEPROMData.size = 1u;
		error = ReadEEPROM(EEPROMData.EEPROMParameters, &EEPROMData);
		HAL_Delay(1);

		EEPROMData.data = &(LCD_MainSettings.autoBacklightOffHourEnd);
		EEPROMData.memAddress = BOARD_LCD_AUTO_BACKLIGHT_OFF_HOUR_END_ADDRESS;
		EEPROMData.size = 1u;
		error = ReadEEPROM(EEPROMData.EEPROMParameters, &EEPROMData);
		HAL_Delay(1);

		EEPROMData.data = &(LCD_MainSettings.allSettings);
		EEPROMData.memAddress = BOARD_LCD_ALL_SETTINGS_ADDRESS;
		EEPROMData.size = 1u;
		error = ReadEEPROM(EEPROMData.EEPROMParameters, &EEPROMData);
		HAL_Delay(1);
	}


		return error;
	}







