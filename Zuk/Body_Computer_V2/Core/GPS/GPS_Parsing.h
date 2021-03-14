/*
 * GPS_Parsing.h
 *
 *  Created on: May 1, 2020
 *      Author: Wojciech Grzelinski
 */


#ifndef INC_GPS_PARSING_H_
#define INC_GPS_PARSING_H_


#include "main.h"
#include "../ErrorCodes/ErrorCodes.h"


#define GPS__USING_LCD

#ifndef GPS_BUFFER_SIZE
#define GPS_BUFFER_SIZE		(uint8_t)(150)
#endif

#define NUMBER_OF_R_AND_N_SIGNS		(uint8_t)(3)

#ifdef GPS__USING_LCD
#include "../lcd_hd44780_i2c/lcd_hd44780_i2c.h"
#endif


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
typedef struct
{
	volatile uint8_t receivedByte;
	volatile uint8_t GPS_buffer[GPS_BUFFER_SIZE];
	timeHours_type TimeZoneAdjPoland;			/* 1 in Winter and 2 in Summer (it will be added to the clock to get proper time */
	timeHours_type TimeZoneManualAdj;		/* for manual adjusting the time zone in clock */

	float homeLatitude;
	float homeLongitude;

	struct message_buffers
	{
		uint8_t hours[2];
		uint8_t minutes[2];
		uint8_t seconds[2];
		uint8_t clock[8];

		uint8_t day[2];
		uint8_t month[2];
		uint8_t year[4];
		uint8_t date[10];

		uint8_t latitude[10];
		uint8_t latitudeIndicator[1];
		uint8_t longitude[11];
		uint8_t longitudeIndicator[1];
		uint8_t fixMessage[5];			/* Because original status is a number, and we want noFix or Fixed */
		uint8_t satellitesUsed[2];
		uint8_t altitude[6];
		uint8_t speed[3];
	}message_buffers;

	struct rawData
	{
		uint8_t UTC[7];					/* UTC time */

		uint8_t Day[3];
		uint8_t Month[3];
		uint8_t Year[3];

		uint8_t Latitude[11];
		uint8_t LatitudeIndicator[2];	/* N-North, S-South */
		uint8_t Longitude[12];
		uint8_t LongitudeIndicator[2];	/* E-East, W-West */
		uint8_t Status[2];				/* 0-No Fix, 1-2D/3D, 2-DGNSS, 4-Fixed RTK, 5-Float RTK, 6-Dead Reckoning */
		uint8_t SatellitesUsed[3];		/* Number of satellites used for navigation */
		uint8_t Altitude[7];			/* Meters above sea level */

		uint8_t Speed[8];				/* Speed over ground in km/h, format: xyz.abc */
	}rawData;

#ifdef GPS__USING_LCD
	struct forLCD
	{
		LCD_message hours;
		LCD_message minutes;
		LCD_message seconds;
		LCD_message clock;

		LCD_message day;
		LCD_message month;
		LCD_message year;
		LCD_message date;

		LCD_message latitude;
		LCD_message latitudeIndicator;
		LCD_message longitude;
		LCD_message longitudeIndicator;
		LCD_message status;
		LCD_message satellitesUsed;
		LCD_message altitude;
		LCD_message speed;
	}forLCD;
#endif

	uint8_t DataReady	:1;				/* Set when all data is received from GPS and ready to be processed */
	uint8_t TimeReady	:1;				/* Set when GPS received the time (do not have to mean, that there is a FIX!!! */
	uint8_t DateReady	:1;				/* Set when GPS received the date (do not have to mean, that there is a FIX or time ready!!! */
	uint8_t Fix			:1;				/* Set when there is fix (set in GPS_Parsing) */
}GPS_data_struct;
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */


Error_Code parse_GPS_data(GPS_data_struct* const GPS);
Error_Code track_GPS_movement(const GPS_data_struct* const GPS);


__weak Error_Code save_GPS_point(const GPS_data_struct* const GPS);

#endif /* INC_GPS_PARSING_H_ */
