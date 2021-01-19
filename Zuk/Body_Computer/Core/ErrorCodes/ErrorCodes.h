/*
 * ErrorCodes.h
 *
 *  Created on: 7 maj 2020
 *      Author: Wojciech Grzelinski
 */


#ifndef INC_ERRORCODES_H_
#define INC_ERRORCODES_H_

//#include "main.h"

typedef enum
{
	NO_ERROR										= 0,

	/*** HAL errors for compliance with HAL functions ***/
	HAL__OK__										= 0,
	HAL__ERROR__									= 1,
	HAL__BUSY__										= 2,
	HAL__TIMEOUT__									= 3,

	/*** OS errors ***/
	OS__OK											= 0,
		/* original OS errors */
	OS__OsEventSignal								= 0x08,		//8
	OS__osEventMessage								= 0x10,		//16
	OS__osEventMail									= 0x20,		//32
	OS__osEventTimeout								= 0x40,		//64
	OS__osErrorParameter							= 0x80,		//128
	OS__osErrorResource								= 0x81,		//129
	OS__osErrorTimeoutResource						= 0xC1,		//193
	OS__osErrorISR									= 0x82,		//130
	OS__osErrorISRRecursive							= 0x83,		//131
	OS__osErrorPriority								= 0x84,		//132
	OS__osErrorNoMemory								= 0x85,		//133
	OS__osErrorValue								= 0x86,		//134
	OS__osErrorOS									= 0xFF,		//255
	OS__os_status_reserved							= 0x7FFFFFFF,//2147483647
		/* my OS errors */
	OS__MAIN_WHILE_LOOP_REACHED						= 2001,
	OS__STARTING_TIMER_FAILED						= 2002,
	OS__STOPPING_TIMER_FAILED						= 2003,

	/*** FreeRTOS errors ***/
	FREERTOS__QUEUE_SEND_ERROR,

	/*** EEPROM errors ***/
	EEPROM__TRIED_TO_WRITE_WHILE_LOCKED				= 4001,
	EEPROM__WHOLE_MEMORY_ERASE_FAIL					= 4002,
	EEPROM__SIZE_TO_WRITE_IS_ZERO					= 4003,
	EEPROM__PARAMETERS_POINTER_IS_NULL				= 4004,
	EEPROM__ADDRESS_POINTER_IS_NULL					= 4005,
	EEPROM__FAILED_TO_WRITE_IN_LCD_TASK				= 4006,

	/*** ADCs errors ***/
	ADC__VALUE_INCORRECT							= 5001,
	ADC__CALIBRATION_FAIL							= 5002,

	/*** TIMERs errors ***/
	TIM__ENCODER_START_FAIL							= 6001,
	TIM__PWM_START_FAIL								= 6002,

	/*** LCD & String errors ***/
	LCD__INIT_FAIL									= 7001,
	LCD__ERROR										= 7002,
	LCD__LAYER_CHOICE_FAILURE						= 7003,
	STRING__EMPTY									= 7010,

	/*** GPS errors ***/
	GPS__NO_SYMBOL_FOUND							= 8001,
	GPS__INVALID_DATA_START							= 8002,

	/** SD Card errors **/
	SDCARD__INITIAL_READ_FAILED						= 9001,

	/*** UNKNOWN / UNDEFINED ERROR ***/
	UNKNOWN_ERROR									= 0x8FFFFFFF
}Error_Code;


void my_error_handler(Error_Code error);
//char* get_error_description(Error_Code error);

#endif /* INC_ERRORCODES_H_ */
