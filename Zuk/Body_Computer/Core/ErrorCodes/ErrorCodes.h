/*
 * ErrorCodes.h
 *
 *  Created on: 7 maj 2020
 *      Author: Wojciech Grzelinski
 */


#ifndef INC_ERRORCODES_H_
#define INC_ERRORCODES_H_


typedef enum
{
	NO_ERROR										= 0,

	/*** HAL errors for compliance with HAL functions ***/
	HAL__OK__										= 0,
	HAL__ERROR__									= 1,
	HAL__BUSY__										= 2,
	HAL__TIMEOUT__									= 3,

	/*** OS errors ***/
	OS__STARTING_TIMER_FAILED,
	OS__STOPPING_TIMER_FAILED,

	/*** EEPROM ERRORS ***/
	EEPROM__TRIED_TO_WRITE_WHILE_LOCKED,

	/*** ADCs ERRORS ***/
	ADC__VALUE_INCORRECT,

	/*** STRING ERRORS ***/
	STRING__EMPTY,

	/*** GPS ERRORS ***/
	GPS__NO_SYMBOL_FOUND,
	GPS__INVALID_DATA_START,

	/*** UNKNOWN / UNDEFINED ERROR ***/
	UNKNOWN_ERROR									= -1
}Error_Code;


#endif /* INC_ERRORCODES_H_ */
