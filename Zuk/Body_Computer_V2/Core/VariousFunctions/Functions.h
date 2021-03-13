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
	float R25;
	float Rgnd;
	float Beta;
	float T25;

	float beta_x_T25;
} NTC_parameters_struct;
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */


Error_Code copy_str_to_buffer(const char *const source, char* destiny, const uint8_t startPosition, const uint8_t sourceLength);
Error_Code copy_buffer_to_str(const char *const source, char* destiny, const uint8_t startPosition, const uint8_t dataLength);
Error_Code calculate_NTC_temperature(boardTemperature_type *temperature, const uint16_t ADC_value, const NTC_parameters_struct *const NTC);
Error_Code calculate_LM35_temperature(float *temperature, const int16_t ADC_value);
Error_Code calculate_EngineOilPressure(float *oilPressure, const uint16_t ADC_value);
Error_Code calculate_voltage(float *result, const uint16_t measure, const float voltageDivider);
Error_Code calculate_fuelLevel(float *result, const uint16_t measure);

Error_Code enable_5VDCDC(void);
Error_Code disable_5VDCDC(void);
Error_Code turnOnPower_GPS(void);
Error_Code turnOffPower_GPS(void);
Error_Code turnOnPower_LCD(void);
Error_Code turnOffPower_LCD(void);
Error_Code turnOnPower_MicroSD(void);
Error_Code turnOffPower_MicroSD(void);
Error_Code turnOnPower_NodeMCU(void);
Error_Code turnOffPower_NodeMCU(void);
Error_Code turnOnPower_CruiseControl(void);
Error_Code turnOffPower_CruiseControl(void);

uint8_t compare_two_strings(const char *const str1/*The short one*/,
		const char *const str2/*The one you compare to*/,
		const uint8_t start /*Start position of the comparison*/,
		const uint8_t size /*Number of characters to compare*/);
int16_t find_nearest_symbol(const char symbol, const char *const str,
		const uint8_t start /*start position in string to look for the symbol*/);

#endif /* INC_FUNCTIONS_H_ */
