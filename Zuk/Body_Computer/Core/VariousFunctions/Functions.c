/*
 * Functions.c
 *
 *  Created on: May 1, 2020
 *      Author: Wojciech Grzelinski
 */


#include "Functions.h"


extern volatile ENCButton_struct ENC_button;

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
 * A function that calculates the temperature basing on given ADC value
 * read from LM35 termometer.
 *
 * @param temperature: a pointer to the temperature it will calculate
 * @param ADC_value: a value from ADC
 * @retval Error_Code: gives a value of error if one occurs
 */
Error_Code calculate_LM35_temperature(float *temperature, const uint16_t ADC_value)
{
	Error_Code error = NO_ERROR;

	if((MIN_ADC_VALUE > ADC_value) || (MAX_ADC_VALUE < ADC_value))
	{
		error = ADC__VALUE_INCORRECT;
	}
	else
	{
		*temperature = (((float)ADC_value) / ADC_RESOLUTION_X_REF_VOLTAGE_float) * 100;
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

	if((MIN_ADC_VALUE >= measure) || (MAX_ADC_VALUE < measure))
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
Error_Code calculate_fuelLevel(float *result, const uint16_t measure)
{
	Error_Code error = NO_ERROR;

	if((MIN_ADC_VALUE >= measure) || (MAX_ADC_VALUE < measure))
	{
		error = ADC__VALUE_INCORRECT;
	}
	else
	{
		*result = 45.0;	//TODO - value just for reference
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



/**
 * A function that makes a scroll list on an LCD
 * display (ecrolling menu).
 *
 * @param boardList: the list to scroll through
 * @param boardListSize: the length of the list
 * @param currentBoard: the board from where the function is called
 * @param BufferForLCD: the main buffer that will be sent to the LCD
 * @param ENCDiffptr: the pointer to the number of encoder's rotations
 * @param externalIterator: the pointer to the iterator from calling
 * 			function to pass the info of the currently chosen submenu
 * @retval error: the error code
 */
Error_Code scroll_list(LCDBoard* boardList, uint8_t boardListSize, LCDBoard* currentBoard, uint8_t BufferForLCD[NO_OF_ROWS_IN_LCD][NO_OF_COLUMNS_IN_LCD], int8_t* ENCDiffptr, int32_t* externalIterator)
{
	Error_Code error = NO_ERROR;
	uint8_t doneFLAG = FALSE;
	static int32_t iterator = 0;
	int8_t ENCDiff = *ENCDiffptr;

	if(0 > *externalIterator)
	{
		iterator = 0;
	}

	if((0 == boardListSize) && (FALSE == doneFLAG))
	{
		error = copy_str_to_buffer("Nothing here", (char*)BufferForLCD[Row3], 1, boardList[iterator].nameActualSize);
		doneFLAG = TRUE;
	}

	if((1 == boardListSize) && (FALSE == doneFLAG))
	{
		iterator = 0;
		error = copy_str_to_buffer(">", (char*)BufferForLCD[Row3], 0, 1);
		error = copy_str_to_buffer(boardList[iterator].name, (char*)BufferForLCD[Row3], 1, boardList[iterator].nameActualSize);
		doneFLAG = TRUE;
	}

	if(FALSE == doneFLAG)
	{
		iterator += ENCDiff;
	}

	if((0 > (iterator - 1)) && (FALSE == doneFLAG))
	{
		iterator = 0;
		error = copy_str_to_buffer(">", (char*)BufferForLCD[Row3], 0, 1);
		error = copy_str_to_buffer(boardList[iterator].name, (char*)BufferForLCD[Row3], 1, boardList[iterator].nameActualSize);

		if((boardListSize - 1) >= (iterator + 1))
		{
			error = copy_str_to_buffer(boardList[iterator+1].name, (char*)BufferForLCD[Row4], 1, boardList[iterator+1].nameActualSize);
		}
		doneFLAG = TRUE;
	}

	if(((boardListSize - 1) <= iterator) && (FALSE == doneFLAG))
	{
		iterator = boardListSize-1;
		error = copy_str_to_buffer(boardList[iterator-1].name, (char*)BufferForLCD[Row2], 1, boardList[iterator-1].nameActualSize);
		error = copy_str_to_buffer(">", (char*)BufferForLCD[Row3], 0, 1);
		error = copy_str_to_buffer(boardList[iterator].name, (char*)BufferForLCD[Row3], 1, boardList[iterator].nameActualSize);
		doneFLAG = TRUE;
	}

	if(FALSE == doneFLAG)
	{
		error = copy_str_to_buffer(boardList[iterator-1].name, (char*)BufferForLCD[Row2], 1, boardList[iterator-1].nameActualSize);
		error = copy_str_to_buffer(">", (char*)BufferForLCD[Row3], 0, 1);
		error = copy_str_to_buffer(boardList[iterator].name, (char*)BufferForLCD[Row3], 1, boardList[iterator].nameActualSize);
		error = copy_str_to_buffer(boardList[iterator+1].name, (char*)BufferForLCD[Row4], 1, boardList[iterator+1].nameActualSize);
		doneFLAG = TRUE;
	}

	*ENCDiffptr = 0;
	*externalIterator = iterator;

	return error;
}
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */


/**
 * A function that makes a scroll list on an LCD
 * display (ecrolling menu).
 *
 * @param boardList: the list to scroll through
 * @param boardListSize: the length of the list
 * @retval error: the error code
 */
Error_Code shortButtonPressDetected_LCD(LCDBoard* currentBoard, LCDBoard* boardList, Enum_Layer* layerSwitchVariable, int32_t* submenuIterator)
{
	Error_Code error = NO_ERROR;

	switch(currentBoard->actionForEnter)
	{
		case YesNo_EnterAction:
		{
			ENC_button.longPressDetected = FALSE;
			ENC_button.shortPressDetected = FALSE;
			*layerSwitchVariable = YesNo_Layer;
			*submenuIterator = -1;
			break;
		}
		case GoInto_EnterAction:
		{
			ENC_button.longPressDetected = FALSE;
			ENC_button.shortPressDetected = FALSE;
			*layerSwitchVariable = boardList[*submenuIterator].layer;
			*submenuIterator = -1;
			break;
		}
		case OnOff_EnterAction:	//TODO
		{
			ENC_button.longPressDetected = FALSE;
			ENC_button.shortPressDetected = FALSE;
			break;
		}
		case Ctrl_EnterAction:	//TODO
		{
			ENC_button.longPressDetected = FALSE;
			ENC_button.shortPressDetected = FALSE;
			break;
		}
		case Done_EnterAction:
		{
			ENC_button.longPressDetected = FALSE;
			ENC_button.shortPressDetected = FALSE;
			*layerSwitchVariable = currentBoard->layerPrevious;
			break;
		}
		case 0x00:	/* None_EnterAction / NotApplicable_EnterAction / No_EnterAction */
		default:
		{
			ENC_button.longPressDetected = FALSE;
			ENC_button.shortPressDetected = FALSE;
			break;
		}
	}

	return error;
}
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */



/**
 * A function that makes a scroll list on an LCD
 * display (scrolling menu).
 *
 * @param boardList: the list to scroll through
 * @param boardListSize: the length of the list
 * @retval error: the error code
 */
Error_Code longButtonPressDetected_LCD(LCDBoard* currentBoard, Enum_Layer* layerSwitchVariable, int32_t* submenuIterator)
{
	Error_Code error = NO_ERROR;

	ENC_button.longPressDetected = FALSE;
	ENC_button.shortPressDetected = FALSE;
	*layerSwitchVariable = currentBoard->layerPrevious;
	*submenuIterator = -1;

	return error;
}
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */



