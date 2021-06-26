/*
 * Functions.c
 *
 *  Created on: May 1, 2020
 *      Author: Wojciech Grzelinski
 */


#include "Functions.h"


#define FUEL_LEVEL_ADC_ERROR_THRESHOLD		((uint16_t)(4000))	/* When disconnected / broken cable the value goes above 4000 */
#define FUEL_EQUATION_COEFF1	((float)(-0.0346f))	/* a: y = a*x + b */
#define FUEL_EQUATION_COEFF2	((float)(110.536f)) /* b: y = a*x + b */


/**
 * A function that copies the given string (source) to the buffer (destiny)
 * but from a certain point in the buffer(startPosition).
 *
 * @param source: a pointer to the string you want to copy
 * @param destiny: a pointer to the buffer you copy the string to
 * @param startPosition: position in buffer where you want to begin the
 * 			copied string to be saved
 * @param sourceLength: length of the string you want to copy
 * @retval Error_Code: gives a value of error if one occurs
 */
Error_Code copy_str_to_buffer(const char *const source, char* destiny, const uint16_t startPosition, const uint16_t sourceLength)
{
	Error_Code error = NO_ERROR;

	for(uint16_t i=0; i<(sourceLength); ++i)
	{
	  destiny[i + startPosition] = source[i];
	}

	return error;
}
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */



/**
 * A function that copies the given part of buffer (source) beginning from
 * the starting point (starPosition) of a given length (dataLength) to a
 * string (destiny).
 *
 * @param source: a pointer to the buffer you want to copy from
 * @param destiny: a pointer to the string you copy to
 * @param startPosition: position in buffer where you want to begin the
 * 			copying from
 * @param dataLength: length of the data from buffer you want to copy
 * @retval Error_Code: gives a value of error if one occurs
 */
Error_Code copy_buffer_to_str(const char *const source, char* destiny, const uint16_t startPosition, const uint16_t dataLength)
{
	Error_Code error = NO_ERROR;

	for(uint16_t i=0; i<(dataLength); ++i)
	{
	  destiny[i] = source[i+startPosition];
	}

	return error;
}
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */



/**
 * A function that calculates the temperature basing on given ADC value
 * and NTC parameters. It return the temperature with no decimal points
 * (pure integer value)
 *
 * @param temperature: a pointer to the temperature it will calculate
 * @param ADC_value: a value from ADC
 * @param NTC: a pointer to the NTC structure with NTC parameters
 * @retval Error_Code: gives a value of error if one occurs
 */
Error_Code calculate_NTC_temperature(boardTemperature_type *temperature, const uint16_t ADC_value, const NTC_parameters_struct *const NTC)
{
	/*********************************************************************************************************
	 * 					Beta * T25									beta_x_T25
	 * 	T = ------------------------------------	=	--------------------------------------
	 * 				  V3.3
	 * 			Rgnd(------- - 1)
	 * 				  Vmeas											(ADC_max - ADC_meas) / 10
	 * 		ln(------------------) * T25 + Beta			ln(coeff * (-------------------------))* T25 + Beta
	 * 				   R25													ADC_meas
	 *
	 * 	where coeff = 1 instead of 0.1 (DO NOT USE FLOAT) so after calculation we have to divide by "10"
	 *********************************************************************************************************/

	Error_Code error = NO_ERROR;

	if((MIN_ADC_VALUE > ADC_value) || (MAX_ADC_VALUE < ADC_value))
	{
		error = ADC__VALUE_INCORRECT;
	}
	else
	{
		float R = NTC->Rgnd * (((float)(MAX_ADC_VALUE) / (float)ADC_value) - 1.0f);

		float step2 = log((R / NTC->R25));

		*temperature = (((NTC->beta_x_T25) / ((step2 * NTC->T25) + NTC->Beta)) - 273);
	}

	return error;
}
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */



/**
 * A function that calculates the temperature basing on given ADC value
 * read from LM35 termometer.
 *
 * @param temperature: a pointer to the temperature it will calculate
 * @param ADC_value: a value from ADC
 * @retval Error_Code: gives a value of error if one occurs
 */
Error_Code calculate_LM35_temperature(float *temperature, const int16_t ADC_value)
{
	Error_Code error = NO_ERROR;
	static const float LM35_TEMPERATURE_TO_VOLTAGE_RATIO = 100.0f;

	if((MIN_LM35_ADC_VALUE > ADC_value) || (MAX_LM35_ADC_VALUE < ADC_value))
	{
		error = ADC__VALUE_INCORRECT;
	}
	else
	{
		*temperature = (((float)ADC_value) / ADC_RESOLUTION_X_REF_VOLTAGE_float) * LM35_TEMPERATURE_TO_VOLTAGE_RATIO;
	}

	return error;
}
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */



/**
 * A function that calculates the oil pressure basing on given ADC value
 * read a sensor.
 *
 * @param oilPressure: a pointer to the oil pressure it will calculate
 * @param ADC_value: a value from ADC
 * @retval Error_Code: gives a value of error if one occurs
 */
Error_Code calculate_EngineOilPressure(float *oilPressure, const uint16_t ADC_value)
{
	Error_Code error = NO_ERROR;

	if((MIN_ADC_VALUE > ADC_value) || (MAX_ADC_VALUE < ADC_value))
	{
		error = ADC__VALUE_INCORRECT;
	}

	*oilPressure = 1.0;	//TODO

	return error;
}
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */



/**
 * A function that calculates the voltage basing on ADC value and voltage
 * divider multiplied by 10.000 (for better accuracy).
 *
 * @param result: a pointer to the result value (voltage)
 * @param measure: the measured ADC value
 * @param voltageDivider: the used voltage divider
 * @retval Error_Code: gives a value of error if one occurs
 */
Error_Code calculate_voltage(float *result, const uint16_t measure, const float voltageDivider)
{
	Error_Code error = NO_ERROR;

	if((MIN_ADC_VALUE > measure) || (MAX_ADC_VALUE < measure))
	{
		error = ADC__VALUE_INCORRECT;
	}
	else
	{
		*result = ((float)(measure) / ADC_RESOLUTION_X_REF_VOLTAGE_float) / voltageDivider;
	}

	return error;
}
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */



/**
 * A function that calculates the fuel level based on the reading
 * from the sensor in the tank.
 *
 * @param result: a pointer to the result value (voltage)
 * @param measure: the measured ADC value
 * @retval Error_Code: gives a value of error if one occurs
 */
Error_Code calculate_fuelLevel(cafFuelLevel_type *result, const uint16_t measure)
{
	Error_Code error = NO_ERROR;

	if((MIN_ADC_VALUE > measure) || (MAX_ADC_VALUE < measure))
	{
		error = ADC__VALUE_INCORRECT;
	}
	else
	{
		if(FUEL_LEVEL_ADC_ERROR_THRESHOLD < measure)
		{
			*result = 99.0f;
			error = ADC__VALUE_INCORRECT;
		}
		else
		{
			/* Equation coefficients calculated in Matlab basing on measurements during vehicle refilling */
			*result = (cafFuelLevel_type)measure * (FUEL_EQUATION_COEFF1) + (FUEL_EQUATION_COEFF2);
		}
	}

	return error;
}
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */



/**
 * A function that enables 5V DCDC converter.
 *
 * @retval Error_Code: gives a value of error if one occurs
 */
Error_Code enable_5VDCDC(void)
{
	Error_Code error = NO_ERROR;
	HAL_GPIO_WritePin(DCDC5V_ENABLE_PORT, DCDC5V_ENABLE_PIN, SET);
	return error;
}
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */



/**
 * A function that disables 5V DCDC converter.
 *
 * @retval Error_Code: gives a value of error if one occurs
 */
Error_Code disable_5VDCDC(void)
{
	Error_Code error = NO_ERROR;
	HAL_GPIO_WritePin(DCDC5V_ENABLE_PORT, DCDC5V_ENABLE_PIN, RESET);
	return error;
}
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */



/**
 * A function that turns on power for GPS.
 *
 * @retval Error_Code: gives a value of error if one occurs
 */
Error_Code turnOnPower_GPS(void)
{
	Error_Code error = NO_ERROR;
	HAL_GPIO_WritePin(POWER_ON_GPS_PORT, POWER_ON_GPS_PIN, SET);
	return error;
}
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */



/**
 * A function that turns off power for GPS.
 *
 * @retval Error_Code: gives a value of error if one occurs
 */
Error_Code turnOffPower_GPS(void)
{
	Error_Code error = NO_ERROR;
	HAL_GPIO_WritePin(POWER_ON_GPS_PORT, POWER_ON_GPS_PIN, RESET);
	return error;
}
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */



/**
 * A function that turns on power for LCD.
 *
 * @retval Error_Code: gives a value of error if one occurs
 */
Error_Code turnOnPower_LCD(void)
{
	Error_Code error = NO_ERROR;
	HAL_GPIO_WritePin(POWER_ON_LCD_PORT, POWER_ON_LCD_PIN, SET);
	return error;
}
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */



/**
 * A function that turns off power for LCD.
 *
 * @retval Error_Code: gives a value of error if one occurs
 */
Error_Code turnOffPower_LCD(void)
{
	Error_Code error = NO_ERROR;
	HAL_GPIO_WritePin(POWER_ON_LCD_PORT, POWER_ON_LCD_PIN, RESET);
	return error;
}
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */



/**
 * A function that turns on power for MicroSD Card.
 *
 * @retval Error_Code: gives a value of error if one occurs
 */
Error_Code turnOnPower_MicroSD(void)
{
	Error_Code error = NO_ERROR;
	HAL_GPIO_WritePin(POWER_ON_MICRO_SD_PORT, POWER_ON_MICRO_SD_PIN, SET);
	return error;
}
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */



/**
 * A function that turns off power for MicroSD Card.
 *
 * @retval Error_Code: gives a value of error if one occurs
 */
Error_Code turnOffPower_MicroSD(void)
{
	Error_Code error = NO_ERROR;
	HAL_GPIO_WritePin(POWER_ON_MICRO_SD_PORT, POWER_ON_MICRO_SD_PIN, RESET);
	return error;
}
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */



/**
 * A function that turns on power for Node MCU.
 *
 * @retval Error_Code: gives a value of error if one occurs
 */
Error_Code turnOnPower_NodeMCU(void)
{
	Error_Code error = NO_ERROR;
	HAL_GPIO_WritePin(POWER_ON_NODE_MCU_PORT, POWER_ON_NODE_MCU_PIN, SET);
	return error;
}
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */



/**
 * A function that turns off power for Node MCU.
 *
 * @retval Error_Code: gives a value of error if one occurs
 */
Error_Code turnOffPower_NodeMCU(void)
{
	Error_Code error = NO_ERROR;
	HAL_GPIO_WritePin(POWER_ON_NODE_MCU_PORT, POWER_ON_NODE_MCU_PIN, RESET);
	return error;
}
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */



/**
 * A function that turns on power for Cruise Control Driver (H bridge).
 *
 * @retval Error_Code: gives a value of error if one occurs
 */
Error_Code turnOnPower_CruiseControl(void)
{
	Error_Code error = NO_ERROR;
	HAL_GPIO_WritePin(POWER_ON_CRUISE_CONTROL_PORT, POWER_ON_CRUISE_CONTROL_PIN, SET);
	return error;
}
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */



/**
 * A function that turns off power for Cruise Control Driver (H bridge).
 *
 * @retval Error_Code: gives a value of error if one occurs
 */
Error_Code turnOffPower_CruiseControl(void)
{
	Error_Code error = NO_ERROR;
	HAL_GPIO_WritePin(POWER_ON_CRUISE_CONTROL_PORT, POWER_ON_CRUISE_CONTROL_PIN, RESET);
	return error;
}
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */



/**
 * [WARNING] No error value returned! Use with caution and check potential
 * 				error yourself!!!
 *
 * A function that compares two strings (str1 and str2). The str1 is
 * compared from the beginning but you can set the staring point of the
 * str2 (start). Only the given number of character (size) is compared.
 *
 * @param str1: a pointer to the string to compare
 * @param str2: a pointer to the string to compare (for this string
 * 			you set the starting point)
 * @param start: a starting point for the str2
 * @param size: a number of characters to compare
 * @retval result: the result of the comparison (true/false)
 */
uint8_t compare_two_strings(const char *const str1/*The short one*/,
		const char *const str2/*The one you compare to*/,
		const uint16_t start /*Start position of the comparison*/,
		const uint16_t size /*Number of characters to compare*/)
{
	uint8_t result = TRUE;

	for (uint16_t i = 0; i < (size); ++i)
	{
		if (str1[i] != str2[i + start])
		{
			result = FALSE;
			break;
		}
	}

	return result;
}
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */



/**
 * [WARNING] No error value returned! Use with caution and check potential
 * 				error yourself!!!
 *
 * A function that finds the first occurrence of the given character
 * (symbol) checking from the starting point (start) in the string (str)
 *
 * @param symbol: the symbol you want to find
 * @param str: the string to look in
 * @param start: the starting position in the string
 * @retval index: the index in the string where the symbol is located
 */
int16_t find_nearest_symbol(const char symbol, const char *const str,
		const uint16_t start /*start position in string to look for the symbol*/)
{
	int16_t index = -1;
	uint16_t strLength = strlen(str);

	for (uint16_t i = start; i < strLength; ++i)
	{
		if (str[i] == symbol)
		{
			index = i;
			break;
		}
	}

	return index;
}
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */








