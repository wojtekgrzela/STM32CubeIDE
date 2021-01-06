/*
 * Functions.h
 *
 *  Created on: May 1, 2020
 *      Author: Wojciech Grzelinski
 */


#ifndef INC_FUNCTIONS_H_
#define INC_FUNCTIONS_H_


#include "main.h"
#include "../ErrorCodes/ErrorCodes.h"

#include <string.h>


#ifndef MIN_ADC_VALUE
	#define MIN_ADC_VALUE	0
#endif
#ifndef MAX_ADC_VALUE
	#define MAX_ADC_VALUE	4095
#endif


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
typedef struct
{
	int32_t R25;
	int32_t Rgnd;
	int32_t Beta;
	int32_t T25;

	int32_t beta_x_T25;
} NTC_parameters_struct;
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */


Error_Code copy_str_to_buffer(const char *const source, char* destiny, const uint8_t startPosition, const uint8_t sourceLength);
Error_Code copy_buffer_to_str(const char *const source, char* destiny, const uint8_t startPosition, const uint8_t dataLength);
Error_Code calculate_NTC_temperature(int16_t *temperature, const uint16_t ADC_value, const NTC_parameters_struct *const NTC);
Error_Code calculate_LM35_temperature(float *temperature, const uint16_t ADC_value);
Error_Code calculate_EngineOilPressure(float *oilPressure, const uint16_t ADC_value);
Error_Code calculate_voltage(float *result, const uint16_t measure, const float voltageDivider);
Error_Code calculate_fuelLevel(float *result, const uint16_t measure);

uint8_t compare_two_strings(const char *const str1/*The short one*/,
		const char *const str2/*The one you compare to*/,
		const uint8_t start /*Start position of the comparison*/,
		const uint8_t size /*Number of characters to compare*/);
int16_t find_nearest_symbol(const char symbol, const char *const str,
		const uint8_t start /*start position in string to look for the symbol*/);

Error_Code scroll_list(LCDBoard* boardList, uint8_t boardListSize, LCDBoard* currentBoard, uint8_t BufferForLCD[NO_OF_ROWS_IN_LCD][NO_OF_COLUMNS_IN_LCD], int8_t* ENCDiffptr, int32_t* externalIterator);
Error_Code shortButtonPressDetected_LCD(LCDBoard* currentBoard, LCDBoard* boardList, Enum_Layer* layerSwitchVariable, int32_t* submenuIterator);
Error_Code longButtonPressDetected_LCD(LCDBoard* currentBoard, Enum_Layer* layerSwitchVariable, int32_t* submenuIterator);
#endif /* INC_FUNCTIONS_H_ */
