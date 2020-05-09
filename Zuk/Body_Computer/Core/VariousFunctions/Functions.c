/*
 * Functions.c
 *
 *  Created on: May 1, 2020
 *      Author: Wojciech Grzelinski
 */


#include "Functions.h"



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
Error_Code copy_str_to_buffer(const char *const source, char* destiny, const uint8_t startPosition, const uint8_t sourceLength)
{
	Error_Code error = NO_ERROR;

	for(uint8_t i=0; i<(sourceLength); ++i)
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
Error_Code copy_buffer_to_str(const char *const source, char* destiny, const uint8_t startPosition, const uint8_t dataLength)
{
	Error_Code error = NO_ERROR;

	for(uint8_t i=0; i<(dataLength); ++i)
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
Error_Code calculate_NTC_temperature(int16_t *temperature, const uint16_t ADC_value, const NTC_parameters_struct *const NTC)
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
	int32_t coeff = 1000;

	if((MIN_ADC_VALUE >= ADC_value) || (MAX_ADC_VALUE < ADC_value))
	{
		error = ADC__VALUE_INCORRECT;
	}
	else
	{
		int32_t R = NTC->Rgnd * (((MAX_ADC_VALUE * coeff) / (uint32_t)ADC_value) - 1 * coeff);

		int32_t step2 = log(((float)(R / coeff) / NTC->R25)) * coeff;

		*temperature = (int16_t)((((NTC->beta_x_T25) * coeff) / ((step2 * NTC->T25) + (NTC->Beta * coeff))) - 273);
	}

	return error;
}
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */



/**
 * A function that calculates the voltage basing on ADC value and voltage
 * divider multiplied by 10.000 (for better accuracy).
 *
 * @param result: a pointer to the result value (voltage)
 * @param measure: the measured ADC value
 * @param voltageDivider_x10000: the used voltage divider multiplied by
 * 			10.000 for better accuracy
 * @retval Error_Code: gives a value of error if one occurs
 */
Error_Code calculate_voltage(uint16_t *result, const uint16_t measure, const uint32_t voltageDivider_x10000)
{
	Error_Code error = NO_ERROR;
	uint16_t coeff = 100;	/* every "10^1" more is one digit after comma more*/

	if((0 == measure) || (4095 < measure))
	{
		error = ADC__VALUE_INCORRECT;
	}
	else
	{
		*result = (((uint32_t)(measure * 10000 * coeff) / 1240) / voltageDivider_x10000);
	}

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
		const uint8_t start /*Start position of the comparison*/,
		const uint8_t size /*Number of characters to compare*/)
{
	uint8_t result = TRUE;

	for (uint8_t i = 0; i < (size); ++i)
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
		const uint8_t start /*start position in string to look for the symbol*/)
{
	int16_t index = -1;
	uint8_t strLength = strlen(str);

	for (uint8_t i = start; i < strLength; ++i)
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

