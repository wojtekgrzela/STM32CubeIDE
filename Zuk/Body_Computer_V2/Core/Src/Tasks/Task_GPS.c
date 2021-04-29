/*
 * Task_GPS.c
 *
 *  Created on: 11 sty 2021
 *      Author: Wojciech Grzelinski
 */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Includes */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "defines.h"
#include "../../VariousFunctions/Functions.h"
#include "../../GPS/GPS_Parsing.h"
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Defines */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Extern variables */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
extern UART_HandleTypeDef huart1;
extern GPS_data_struct GPS;
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Local variables */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Functions prototypes */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
static float floatModulo(float a, float b);
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */



void StartGPSTask(void const * argument)
{
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = MY_GPS_TASK_TIME_PERIOD;
	Error_Code error = NO_ERROR;

#ifdef GPS_LCD_PRINT
	timeHours_type tempHour = '\0';
	uint8_t tempbuffer[3u] = {'a'};

	GPS.forLCD.hours.messageHandler 				= GPS.message_buffers.hours;
	GPS.forLCD.minutes.messageHandler 				= GPS.message_buffers.minutes;
	GPS.forLCD.seconds.messageHandler 				= GPS.message_buffers.seconds;
	GPS.forLCD.clock.messageHandler 				= GPS.message_buffers.clock;
	GPS.forLCD.day.messageHandler					= GPS.message_buffers.day;
	GPS.forLCD.month.messageHandler					= GPS.message_buffers.month;
	GPS.forLCD.year.messageHandler					= GPS.message_buffers.year;
	GPS.forLCD.date.messageHandler					= GPS.message_buffers.date;
	GPS.forLCD.latitude.messageHandler 				= GPS.message_buffers.latitude;
	GPS.forLCD.latitudeIndicator.messageHandler 	= GPS.message_buffers.latitudeIndicator;
	GPS.forLCD.longitude.messageHandler 			= GPS.message_buffers.longitude;
	GPS.forLCD.longitudeIndicator.messageHandler 	= GPS.message_buffers.longitudeIndicator;
	GPS.forLCD.status.messageHandler				= GPS.message_buffers.fixMessage;
	GPS.forLCD.satellitesUsed.messageHandler		= GPS.message_buffers.satellitesUsed;
	GPS.forLCD.altitude.messageHandler				= GPS.message_buffers.altitude;
	GPS.forLCD.speed.messageHandler					= GPS.message_buffers.speed;

	GPS.forLCD.hours.size 				= 2u;
	GPS.forLCD.minutes.size 			= 2u;
	GPS.forLCD.seconds.size 			= 2u;
	GPS.forLCD.clock.size 				= 8u;
	GPS.forLCD.day.size					= 2u;
	GPS.forLCD.month.size				= 2u;
	GPS.forLCD.year.size				= 4u;
	GPS.forLCD.date.size				= 10u;
	GPS.forLCD.latitude.size			= 10u;
	GPS.forLCD.latitudeIndicator.size	= 1u;
	GPS.forLCD.longitude.size			= 11u;
	GPS.forLCD.longitudeIndicator.size	= 1u;
#endif

	/* Turns on receiving from GPS */
	HAL_UART_Receive_IT(&huart1, (uint8_t *)&(GPS.receivedByte), 1u);

	xLastWakeTime = xTaskGetTickCount();
	/* Infinite loop */
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
	for (;;)
	{
#ifdef GPS_PARSING
		if (TRUE == GPS.DataReady)
		{
			error = parse_GPS_data(&GPS);
		}

		if(NO_ERROR != error)
		{
			my_error_handler(error);
		}
#endif

#ifdef GPS_LCD_PRINT
		if(TRUE == GPS.TimeReady)
		{
			error = copy_buffer_to_str((char*)GPS.rawData.UTC, (char*)tempbuffer, 0, 2);
			tempHour = (timeHours_type)atoi((char*)tempbuffer);
			tempHour += GPS.TimeZoneAdjPoland;
			tempHour += GPS.TimeZoneManualAdj;

			if(24 <= tempHour)
			{
				tempHour -= 24;
			}

			if(0 > tempHour)
			{
				tempHour += 24;
			}

			if(10 <= tempHour)
			{
				itoa(tempHour, (char*)tempbuffer, 10/*decimal system*/);
			}
			else
			{
				tempbuffer[0] = '0';
				itoa(tempHour, (char*)&tempbuffer[1], 10);
			}

			GPS.forLCD.hours.messageReadyFLAG = FALSE;
			error = copy_buffer_to_str((char*)tempbuffer, (char*)GPS.message_buffers.hours, 0u, GPS.forLCD.hours.size);
			GPS.forLCD.hours.messageReadyFLAG = TRUE;

			GPS.forLCD.minutes.messageReadyFLAG = FALSE;
			error = copy_buffer_to_str((char*)GPS.rawData.UTC, (char*)GPS.message_buffers.minutes, 2u, GPS.forLCD.minutes.size);
			GPS.forLCD.minutes.messageReadyFLAG = TRUE;

			GPS.forLCD.seconds.messageReadyFLAG = FALSE;
			error = copy_buffer_to_str((char*)GPS.rawData.UTC, (char*)GPS.message_buffers.seconds, 4u, GPS.forLCD.seconds.size);
			GPS.forLCD.seconds.messageReadyFLAG = TRUE;

			GPS.hoursInNumber = (timeHours_type)atoi((char*)tempbuffer);
		}
		else
		{
			GPS.forLCD.hours.messageReadyFLAG = FALSE;
			GPS.forLCD.minutes.messageReadyFLAG = FALSE;
			GPS.forLCD.seconds.messageReadyFLAG = FALSE;

			for(uint i=0; i<2; ++i)
			{
				GPS.message_buffers.hours[i] = 'x';
				GPS.message_buffers.minutes[i] = 'x';
				GPS.message_buffers.seconds[i] = 'x';
			}

			GPS.forLCD.hours.messageReadyFLAG = TRUE;
			GPS.forLCD.minutes.messageReadyFLAG = TRUE;
			GPS.forLCD.seconds.messageReadyFLAG = TRUE;

			GPS.hoursInNumber = -1;
		}

		GPS.forLCD.clock.messageReadyFLAG = FALSE;
		GPS.message_buffers.clock[2] = ':';
		GPS.message_buffers.clock[5] = ':';
		error = copy_str_to_buffer((char*)GPS.message_buffers.hours, (char*)GPS.message_buffers.clock, 0u, GPS.forLCD.hours.size);
		error = copy_str_to_buffer((char*)GPS.message_buffers.minutes, (char*)GPS.message_buffers.clock, 3u, GPS.forLCD.minutes.size);
		error = copy_str_to_buffer((char*)GPS.message_buffers.seconds, (char*)GPS.message_buffers.clock, 6u, GPS.forLCD.seconds.size);
		GPS.forLCD.clock.messageReadyFLAG = TRUE;

		if(NO_ERROR != error)
		{
			my_error_handler(error);
		}

		if(TRUE == GPS.DateReady)
		{
			GPS.forLCD.day.messageReadyFLAG = FALSE;
			error = copy_buffer_to_str((char*)GPS.rawData.Day, (char*)GPS.message_buffers.day, 0u, GPS.forLCD.day.size);
			GPS.forLCD.day.messageReadyFLAG = TRUE;

			GPS.forLCD.month.messageReadyFLAG = FALSE;
			error = copy_buffer_to_str((char*)GPS.rawData.Month, (char*)GPS.message_buffers.month, 0u, GPS.forLCD.month.size);
			GPS.forLCD.month.messageReadyFLAG = TRUE;

			GPS.forLCD.year.messageReadyFLAG = FALSE;
			error = copy_buffer_to_str((char*)GPS.rawData.Year, (char*)GPS.message_buffers.year, 0u, GPS.forLCD.year.size);
			GPS.forLCD.year.messageReadyFLAG = TRUE;
		}
		else
		{
			GPS.forLCD.day.messageReadyFLAG = FALSE;
			GPS.forLCD.month.messageReadyFLAG = FALSE;
			GPS.forLCD.year.messageReadyFLAG = FALSE;

			for(uint i=0; i<2; ++i)
			{
				GPS.message_buffers.day[i] = 'x';
				GPS.message_buffers.month[i] = 'x';
				GPS.message_buffers.year[i] = 'x';
				GPS.message_buffers.year[i+2] = 'x';
			}

			GPS.forLCD.day.messageReadyFLAG = TRUE;
			GPS.forLCD.month.messageReadyFLAG = TRUE;
			GPS.forLCD.year.messageReadyFLAG = TRUE;
		}

		GPS.forLCD.date.messageReadyFLAG = FALSE;
		GPS.message_buffers.date[2] = '_';
		GPS.message_buffers.date[5] = '_';
		error = copy_str_to_buffer((char*)GPS.message_buffers.day, (char*)GPS.message_buffers.date, 0u, GPS.forLCD.day.size);
		error = copy_str_to_buffer((char*)GPS.message_buffers.month, (char*)GPS.message_buffers.date, 3u, GPS.forLCD.month.size);
		error = copy_str_to_buffer((char*)GPS.message_buffers.year, (char*)GPS.message_buffers.date, 6u, GPS.forLCD.year.size);
		GPS.forLCD.date.messageReadyFLAG = TRUE;

		if(NO_ERROR != error)
		{
			my_error_handler(error);
		}

		if(TRUE == GPS.Fix)
		{
			float tempVal = 0.0;
			float tempPartVal = 0.0;
			uint8_t tempBuffer[20u] = {SPACE_IN_ASCII};
			uint16_t tempSize = 0;

			GPS.forLCD.latitude.messageReadyFLAG = FALSE;
			GPS.forLCD.latitudeIndicator.messageReadyFLAG = FALSE;
			tempVal = (float)atof((const char*)GPS.rawData.Latitude);
			tempPartVal = floatModulo(tempVal, 100.0f);
			tempVal = tempVal + (tempPartVal / 60.0f);
			tempSize = snprintf((char*)tempBuffer, 11u, "%01" PRIu32 ".%06" PRIu32, (uint32_t)tempVal, ((uint32_t)(tempVal * 1000000u) % 1000000u));
			error = copy_str_to_buffer((char*)tempBuffer, (char*)GPS.message_buffers.latitude, 0u, tempSize);
			error = copy_str_to_buffer((char*)GPS.rawData.LatitudeIndicator, (char*)GPS.message_buffers.latitudeIndicator, 0u, GPS.forLCD.latitudeIndicator.size);
			GPS.forLCD.latitude.messageReadyFLAG = TRUE;
			GPS.forLCD.latitudeIndicator.messageReadyFLAG = TRUE;

			GPS.forLCD.longitude.messageReadyFLAG = FALSE;
			GPS.forLCD.longitudeIndicator.messageReadyFLAG = FALSE;
			tempVal = (float)atof((const char*)GPS.rawData.Longitude);
			tempPartVal = floatModulo(tempVal, 100.0f);
			tempVal = tempVal + (tempPartVal / 60.0f);
			tempSize = snprintf((char*)tempBuffer, 12u, "%01" PRIu32 ".%06" PRIu32, (uint32_t)tempVal, ((uint32_t)(tempVal * 1000000u) % 1000000u));
			error = copy_str_to_buffer((char*)tempBuffer, (char*)GPS.message_buffers.longitude, 0u, tempSize);
			error = copy_str_to_buffer((char*)GPS.rawData.LongitudeIndicator, (char*)GPS.message_buffers.longitudeIndicator, 0u, GPS.forLCD.longitudeIndicator.size);
			GPS.forLCD.longitude.messageReadyFLAG = TRUE;
			GPS.forLCD.longitudeIndicator.messageReadyFLAG = TRUE;


			GPS.forLCD.status.messageReadyFLAG = FALSE;
			GPS.forLCD.status.size = 5u;	//5u - Because "Fixed" has 5 letters
			error = copy_str_to_buffer("Fixed", (char*)GPS.message_buffers.fixMessage, 0u, GPS.forLCD.status.size);
			GPS.forLCD.status.messageReadyFLAG = TRUE;

			GPS.forLCD.satellitesUsed.messageReadyFLAG = FALSE;
			error = copy_str_to_buffer((char*)GPS.rawData.SatellitesUsed, (char*)GPS.message_buffers.satellitesUsed, 0u, GPS.forLCD.satellitesUsed.size);
			GPS.forLCD.satellitesUsed.messageReadyFLAG = TRUE;

			GPS.forLCD.altitude.messageReadyFLAG = FALSE;
			GPS.forLCD.altitude.size = strlen((char*)GPS.rawData.Altitude);
			error = copy_str_to_buffer((char*)GPS.rawData.Altitude, (char*)GPS.message_buffers.altitude, 0u, GPS.forLCD.altitude.size);
			GPS.forLCD.altitude.messageReadyFLAG = TRUE;

			GPS.forLCD.speed.messageReadyFLAG = FALSE;
			GPS.forLCD.speed.size = find_nearest_symbol('.', (char*)GPS.rawData.Speed, 0u);
			error = copy_str_to_buffer((char*)GPS.rawData.Speed, (char*)GPS.message_buffers.speed, 0u, GPS.forLCD.speed.size);
			GPS.forLCD.speed.messageReadyFLAG = TRUE;
		}
		else
		{
			GPS.forLCD.latitude.messageReadyFLAG = FALSE;
			GPS.forLCD.latitudeIndicator.messageReadyFLAG = FALSE;
			error = copy_str_to_buffer("xxxx.xxxxx", (char*)GPS.message_buffers.latitude, 0u, GPS.forLCD.latitude.size);
			error = copy_str_to_buffer("X", (char*)GPS.message_buffers.latitudeIndicator, 0u, GPS.forLCD.latitudeIndicator.size);
			GPS.forLCD.latitude.messageReadyFLAG = TRUE;
			GPS.forLCD.latitudeIndicator.messageReadyFLAG = TRUE;

			GPS.forLCD.longitude.messageReadyFLAG = FALSE;
			GPS.forLCD.longitudeIndicator.messageReadyFLAG = FALSE;
			error = copy_str_to_buffer("xxxxx.xxxxx", (char*)GPS.message_buffers.longitude, 0u, GPS.forLCD.longitude.size);
			error = copy_str_to_buffer("X", (char*)GPS.message_buffers.longitudeIndicator, 0u, GPS.forLCD.longitudeIndicator.size);
			GPS.forLCD.longitude.messageReadyFLAG = TRUE;
			GPS.forLCD.longitudeIndicator.messageReadyFLAG = TRUE;


			GPS.forLCD.status.messageReadyFLAG = FALSE;
			GPS.forLCD.status.size = 5u;	//5u - Because "Fixed" has 5 letters
			error = copy_str_to_buffer("NoFix", (char*)GPS.message_buffers.fixMessage, 0u, GPS.forLCD.status.size);
			GPS.forLCD.status.messageReadyFLAG = TRUE;

			GPS.forLCD.satellitesUsed.messageReadyFLAG = FALSE;
			error = copy_str_to_buffer((char*)GPS.rawData.SatellitesUsed, (char*)GPS.message_buffers.satellitesUsed, 0u, GPS.forLCD.satellitesUsed.size);
			GPS.forLCD.satellitesUsed.messageReadyFLAG = TRUE;

			GPS.forLCD.altitude.messageReadyFLAG = FALSE;
			GPS.forLCD.altitude.size = 5u;
			error = copy_str_to_buffer("xxx.x", (char*)GPS.message_buffers.altitude, 0u, GPS.forLCD.altitude.size);
			GPS.forLCD.altitude.messageReadyFLAG = TRUE;

			GPS.forLCD.speed.messageReadyFLAG = FALSE;
			GPS.forLCD.speed.size = 3u;
			error = copy_str_to_buffer("xxx", (char*)GPS.message_buffers.speed, 0u, GPS.forLCD.speed.size);
			GPS.forLCD.speed.messageReadyFLAG = TRUE;
		}

		if(NO_ERROR != error)
		{
			my_error_handler(error);
		}
#endif

		vTaskDelayUntil(&xLastWakeTime, xFrequency);
	}
}



static float floatModulo(float a, float b)
{
	float result = a;

	if (a < 0)
	{
		a = -a;
	}
	if (b < 0)
	{
		b = -b;
	}

	while (result > b)
	{
		result -= b;
	}

	return result;
}
