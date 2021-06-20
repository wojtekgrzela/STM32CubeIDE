/*
 * ErrorCodes.c
 *
 *  Created on: 12 paź 2020
 *      Author: Dell
 */


#include "ErrorCodes.h"
#include "main.h"

#ifndef TRUE
#define TRUE	(uint8_t)(1)
#endif

extern boolean EXT_LCDReInitRequest;

void my_error_handler(Error_Code error)
{
	static uint32_t debugCounter = 0;	//A counter of errors for debugging purposes

	switch (error)
	{
		case 0:		/*NO_ERROR / HAL__OK__ / OS__OK */ //Not an error - everything is OK
		{
			break;
		}

		case HAL__ERROR__:	//There should be no error from HAL library allowed
		case HAL__BUSY__:
		case HAL__TIMEOUT__:
		{
			while(TRUE) {}
			break;
		}

		case OS__STARTING_TIMER_FAILED:		//Such an error will make the system unresponsive / non-operational
		case OS__STOPPING_TIMER_FAILED:
		{
			while(TRUE) {}
			break;
		}

		case FREERTOS__QUEUE_SEND_ERROR:	//Serious system operating problem
		{
			while(TRUE) {}
			break;
		}

		case ADC__VALUE_INCORRECT:			//Not a real issue for now
		{
			++debugCounter;
			while(TRUE) {}
			break;
		}

		case STRING__EMPTY:					//Not a real issue for now
		{
			++debugCounter;
			while(TRUE) {}
			break;
		}

		case GPS__NO_SYMBOL_FOUND:			//Not a real issue for now
		case GPS__INVALID_DATA_START:
		{
			++debugCounter;
			break;
		}

		case SDCARD__INIT_FAILED:	//Non-critical error but the usage of card must be stopped - done in the code
		{
			++debugCounter;
			while(TRUE) {}
			break;
		}

		case UNKNOWN_ERROR:					//Due to not knowing the error it is better to stop the program
		{
			while(TRUE) {}
			break;
		}

		case LCD__ERROR:
		{
			EXT_LCDReInitRequest = TRUE;
		}

		default:							//By default the uC should be stopped
		{
			while(TRUE) {}
			break;
		}
	}
}
