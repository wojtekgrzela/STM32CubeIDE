/*
 * GPS_Parsing.c
 *
 *  Created on: May 1, 2020
 *      Author: Wojciech Grzelinski
 */


#include "GPS_Parsing.h"
#include "../VariousFunctions/Functions.h"

#include <math.h>


#define RADIUS_OF_THE_EARTH				((float)(6372795.0))

#define MINIMUM_SPEED_TO_RECORD			((float)(4.0))
#define MINIMUM_ANGLE_TO_RECORD			((float)(4.0))
#define MAX_SPEED_IN_TRAFFIC_JAM		((float)(2.0))
#define MAX_DISTANCE_IN_TRAFFIC_JAM		((int32_t)(50))
#define MIN_ALTITUDE_CHANGE				((float)(20.0))

#define MAX_MESSAGES_NUMBER				((uint8_t)(7u))

static float calculate_Course(float latitudeStart, float longitudeStart, float latitudeEnd, float longitudeEnd);
static float calculate_Distance(float latitudeStart, float longitudeStart, float latitudeEnd, float longitudeEnd);
static inline float to_Radians(const float decimalDegree);
static inline float to_Degrees(const float radians);
static inline float pow2(float var);

/**
 * A function that copies the data from GPS buffer from UART and
 * performs parsing on it. There is parsing for GPVTG and GPGGA
 * implemented. Information is then cut put into buffers.
 *
 * @param GPS: a pointer to a structure with multiple parameters,
 * buffers and pointers (look in GPS_parsing.h)
 * @retval Error_Code: gives a value of error if one occurs
 */
Error_Code parse_GPS_data(GPS_data_struct* const GPS)
{
	uint8_t GPS_message[GPS_BUFFER_SIZE] = {0};
	int16_t GPS_messages_indexes[MAX_MESSAGES_NUMBER+1u] = {-1};	/* +1 because index 0 always must be "0" */
	int16_t index1 = -1;
	int16_t index2 = -1;
	int16_t tempIndex = -1;
	int16_t GPGGAIndex = -1;
	int16_t GPVTGIndex = -1;
	int16_t GPRMCIndex = -1;
	int16_t indexDiff = 0;
	Error_Code error = NO_ERROR;

	for(int i=0; i<GPS_BUFFER_SIZE; ++i)
	{
		GPS_message[i] = GPS->GPS_buffer[i];
	}

	memset((uint8_t *)(GPS->GPS_buffer), 0, GPS_BUFFER_SIZE);
	GPS->DataReady = RESET;

	for(uint8_t i=1u; i<MAX_MESSAGES_NUMBER+1u; ++i)	/* Starting from 1 because index 0 always must be "0" */
	{
		GPS_messages_indexes[i] = find_nearest_symbol('\n', (const char* const)GPS_message, GPS_messages_indexes[i-1]+1);
	}

	for(uint8_t i=0; i<MAX_MESSAGES_NUMBER+1u; ++i)
	{
		if (TRUE == compare_two_strings("$GPGGA", (const char* const )GPS_message,
				GPS_messages_indexes[i]+1 /*Start position of the comparison*/,
						6 /*Number of characters to compare*/))
		{
			GPGGAIndex = GPS_messages_indexes[i] + 1;
		}

		if (TRUE == compare_two_strings("$GPVTG", (const char* const )GPS_message,
				GPS_messages_indexes[i]+1 /*Start position of the comparison*/,
						6 /*Number of characters to compare*/))
		{
			GPVTGIndex = GPS_messages_indexes[i] + 1;
		}

		if (TRUE == compare_two_strings("$GPRMC", (const char* const )GPS_message,
				GPS_messages_indexes[i]+1 /*Start position of the comparison*/,
						6 /*Number of characters to compare*/))
		{
			GPRMCIndex = GPS_messages_indexes[i] + 1;
		}
	}


	/* FIRST PART OF THE MESSAGE */
	if(-1 != GPGGAIndex)
	{
		/* FINDING UTC (time) START */
		if(NO_ERROR == error)
		{
			index1 = find_nearest_symbol(',', (const char* const)GPS_message, GPGGAIndex);
			if(-1 == index1)
			{
				error = GPS__NO_SYMBOL_FOUND;
			}
			else
			{
				index2 = find_nearest_symbol(',', (const char* const)GPS_message, index1+1);
				if(-1 == index2)
				{
					error = GPS__NO_SYMBOL_FOUND;
				}
			}
			if(NO_ERROR == error)
			{
				indexDiff = index2 - index1;
				if(indexDiff >1)
				{
					error = copy_buffer_to_str((char*)GPS_message, (char*)GPS->rawData.UTC, index1+1, (indexDiff-1)-3/*-3 because we ignore 3 letters there!!*/);
					GPS->TimeReady = 1;
				}
				else
				{
					GPS->TimeReady = 0;
				}
			}
		}
		/* FINDING UTC (time) END */

		/* FINDING Latitude START */
		if(NO_ERROR == error)
		{
			index1 = index2;

			index2 = find_nearest_symbol(',', (const char* const)GPS_message, index1+1);
			if(-1 == index2)
			{
				error = GPS__NO_SYMBOL_FOUND;
			}

			if(NO_ERROR == error)
			{
				indexDiff = index2 - index1;
				if(indexDiff >1)
				{
					error = copy_buffer_to_str((char*)GPS_message, (char*)GPS->rawData.Latitude, index1+1, indexDiff-1);
				}
			}
		}
		/* FINDING Latitude END */

		/* FINDING Latitude Indicator START */
		if(NO_ERROR == error)
		{
			index1 = index2;

			index2 = find_nearest_symbol(',', (const char* const)GPS_message, index1+1);
			if(-1 == index2)
			{
				error = GPS__NO_SYMBOL_FOUND;
			}

			if(FALSE == error)
			{
				indexDiff = index2 - index1;
				if(indexDiff >1)
				{
					error = copy_buffer_to_str((char*)GPS_message, (char*)GPS->rawData.LatitudeIndicator, index1+1, indexDiff-1);
				}
			}
		}
		/* FINDING Latitude Indicator END */

		/* FINDING Longitude START */
		if(NO_ERROR == error)
		{
			index1 = index2;

			index2 = find_nearest_symbol(',', (const char* const)GPS_message, index1+1);
			if(-1 == index2)
			{
				error = GPS__NO_SYMBOL_FOUND;
			}

			if(NO_ERROR == error)
			{
				indexDiff = index2 - index1;
				if(indexDiff >1)
				{
					error = copy_buffer_to_str((char*)GPS_message, (char*)GPS->rawData.Longitude, index1+1, indexDiff-1);
				}
			}
		}
		/* FINDING Longitude END */

		/* FINDING Longitude Indicator START */
		if(NO_ERROR == error)
		{
			index1 = index2;

			index2 = find_nearest_symbol(',', (const char* const)GPS_message, index1+1);
			if(-1 == index2)
			{
				error = GPS__NO_SYMBOL_FOUND;
			}

			if(NO_ERROR == error)
			{
				indexDiff = index2 - index1;
				if(indexDiff >1)
				{
					error = copy_buffer_to_str((char*)GPS_message, (char*)GPS->rawData.LongitudeIndicator, index1+1, indexDiff-1);
				}
			}
		}
		/* FINDING Longitude Indicator END */

		/* FINDING Status START */
		if(NO_ERROR == error)
		{
			index1 = index2;

			index2 = find_nearest_symbol(',', (const char* const)GPS_message, index1+1);
			if(-1 == index2)
			{
				error = GPS__NO_SYMBOL_FOUND;
			}

			if(NO_ERROR == error)
			{
				indexDiff = index2 - index1;
				if(indexDiff >1)
				{
					error = copy_buffer_to_str((char*) GPS_message,(char*) GPS->rawData.Status, index1 + 1, indexDiff - 1);
					if (('1' == GPS->rawData.Status[0])
							|| ('2' == GPS->rawData.Status[0])
							|| ('3' == GPS->rawData.Status[0])
							|| ('4' == GPS->rawData.Status[0])
							|| ('5' == GPS->rawData.Status[0])
							|| ('6' == GPS->rawData.Status[0]))
					{
						GPS->Fix = 1;
					}
					else
					{
						GPS->Fix = 0;
					}
				}
			}
		}
		/* FINDING Status END */

		/* FINDING Satellites Used START */
		if(NO_ERROR == error)
		{
			index1 = index2;

			index2 = find_nearest_symbol(',', (const char* const)GPS_message, index1+1);
			if(-1 == index2)
			{
				error = GPS__NO_SYMBOL_FOUND;
			}

			if(NO_ERROR == error)
			{
				indexDiff = index2 - index1;
				if(indexDiff >1)
				{
					error = copy_buffer_to_str((char*)GPS_message, (char*)GPS->rawData.SatellitesUsed, index1+1, indexDiff-1);
				}
			}
		}
		/* FINDING Satellites Used END */

		//IGNORING Horizontal Dilution of Precision START
		index1 = index2;
		index2 = find_nearest_symbol(',', (const char* const)GPS_message, index1+1);
		if(-1 == index2)
		{
			error = GPS__NO_SYMBOL_FOUND;
		}
		//IGNORING Horizontal Dilution of Precision END

		/* FINDING Altitude START */
		if(NO_ERROR == error)
		{
			index1 = index2;

			index2 = find_nearest_symbol(',', (const char* const)GPS_message, index1+1);
			if(-1 == index2)
			{
				error = GPS__NO_SYMBOL_FOUND;
			}

			if(NO_ERROR == error)
			{
				indexDiff = index2 - index1;
				if(indexDiff >1)
				{
					error = copy_buffer_to_str((char*)GPS_message, (char*)GPS->rawData.Altitude, index1+1, indexDiff-1);
				}
			}
		}
		/* FINDING Altitude END */

		//IGNORING the rest of the information: Unit, Geoid Separation, Age of diff corrections and ID of Reference Station
	}
	else
	{
		error = GPS__INVALID_DATA_START;
	}


	/* SECOND PART OF THE MESSAGE */
	if(-1 != GPVTGIndex)
	{
		/* IGNORING COG, Unit, COG, Unit, SOG, Unit START */
		tempIndex = GPVTGIndex;
		for(uint8_t i=0; i<6; ++i)
		{
			if(NO_ERROR == error)
			{
				index1 = find_nearest_symbol(',', (const char* const)GPS_message, tempIndex);
				if(-1 == index1)
				{
					error = GPS__NO_SYMBOL_FOUND;
				}
				else
				{
					index2 = find_nearest_symbol(',', (const char* const)GPS_message, index1+1);
					if(-1 == index2)
					{
						error = GPS__NO_SYMBOL_FOUND;
					}
					else
					{
						tempIndex = index2;
					}
				}
			}
		}
		/* IGNORING COG END */

		/* FINDING Speed START */
		if(NO_ERROR == error)
		{
			index1 = index2;

			index2 = find_nearest_symbol(',', (const char* const)GPS_message, index1+1);
			if(-1 == index2)
			{
				error = GPS__NO_SYMBOL_FOUND;
			}

			if(FALSE == error)
			{
				indexDiff = index2 - index1;
				if(indexDiff >1)
				{
					error = copy_buffer_to_str((char*)GPS_message, (char*)GPS->rawData.Speed, index1+1, indexDiff-1);
				}
			}
		}
		/* FINDING Speed END */
		//IGNORING the rest of the message: no useful information for us
	}
	else
	{
		error = GPS__INVALID_DATA_START;
	}


	/* Third PART OF THE MESSAGE */
	if(-1 != GPRMCIndex)
	{
		/* IGNORING UTC time, position status, lat, lat dir, lon, lon dir, speed Kn, track true, START */
		tempIndex = GPRMCIndex;
		for(uint8_t i=0; i<8; ++i)
		{
			if(NO_ERROR == error)
			{
				index1 = find_nearest_symbol(',', (const char* const)GPS_message, tempIndex);
				if(-1 == index1)
				{
					error = GPS__NO_SYMBOL_FOUND;
				}
				else
				{
					index2 = find_nearest_symbol(',', (const char* const)GPS_message, index1+1);
					if(-1 == index2)
					{
						error = GPS__NO_SYMBOL_FOUND;
					}
					else
					{
						tempIndex = index2;
					}
				}
			}
		}
		/* IGNORING UTC time END */

		/* FINDING day START */
		if(NO_ERROR == error)
		{
			index2 = find_nearest_symbol(',', (const char* const)GPS_message, index1+1);
			if(-1 == index2)
			{
				error = GPS__NO_SYMBOL_FOUND;
			}

			if(FALSE == error)
			{
				indexDiff = index2 - index1;
				if(indexDiff >1)
				{
					char tempStringBuff[5] = {'\0'};
					error = copy_buffer_to_str((char*)GPS_message, (char*)GPS->rawData.Day, index1+1u, 2u);
					error = copy_buffer_to_str((char*)GPS_message, (char*)GPS->rawData.Month, index1+1u+2u, 2u);
					error = copy_buffer_to_str((char*)"20", (char*)(&(tempStringBuff[0])), 0u, 2u);
					error = copy_buffer_to_str((char*)GPS_message, (char*)(&(tempStringBuff[2])), index1+1u+4u, 2u);

					error = copy_buffer_to_str((char*)tempStringBuff, (char*)GPS->rawData.Year, 4u, 2u);
				}
			}
		}
		/* FINDING day END */

		/* FINDING month START */
		if(NO_ERROR == error)
		{
			index1 = index2;
			index2 = find_nearest_symbol(',', (const char* const)GPS_message, index1+1);
			if(-1 == index2)
			{
				error = GPS__NO_SYMBOL_FOUND;
			}

			if(FALSE == error)
			{
				indexDiff = index2 - index1;
				if(indexDiff >1)
				{
					error = copy_buffer_to_str((char*)GPS_message, (char*)GPS->rawData.Month, index1+1, indexDiff-1);
				}
			}
		}
		/* FINDING month END */

		/* FINDING year START */
		if(NO_ERROR == error)
		{
			index1 = index2;
			index2 = find_nearest_symbol(',', (const char* const)GPS_message, index1+1);
			if(-1 == index2)
			{
				error = GPS__NO_SYMBOL_FOUND;
			}

			if(FALSE == error)
			{
				indexDiff = index2 - index1;
				if(indexDiff >1)
				{
					error = copy_buffer_to_str((char*)GPS_message, (char*)GPS->rawData.Year, index1+1, indexDiff-1);
					GPS->DateReady = 1;
				}
				else
				{
					GPS->DateReady = 0;
				}
			}
		}
		/* FINDING year END */
		//IGNORING the rest of the message: no useful information for us
	}
	else
	{
		error = GPS__INVALID_DATA_START;
	}


	return error;
}
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */



/**
 * A function that checks if the vehicle is moving and if the
 * movement is within given parameters it gives a point to
 * save (for example on a SD card etc.).
 *
 * @param GPS: a pointer to a structure with multiple parameters,
 * buffers and pointers (look in GPS_parsing.h)
 * @retval boolean: TRUE if the position should be saved, FALSE if not
 */
boolean track_GPS_movement(const GPS_data_struct* const GPS)
{
	boolean Write_FLAG = 0;

	int32_t distance = 0;
	float angle = 0;
	float course = 0.0;
	float speed = 0.0;
	float altitudeDiff = 0.0;

	static float courseOld = 0.0;
	static float latitudeOld = 0.0;
	static float longitudeOld = 0.0;
	static float altitudeOld = 0.0;

	float latitude = atoff((char*)(GPS->rawData.Latitude));
	float longitude = atoff((char*)(GPS->rawData.Longitude));
	float altitude = atoff((char*)(GPS->rawData.Altitude));

	course = calculate_Course(latitudeOld, longitudeOld, latitude, longitude);
	speed = atoff((char*)(GPS->rawData.Speed));
	altitudeDiff = altitude - altitudeOld;

	if (0 > altitudeDiff)
	{
		altitudeDiff *= (-1);
	}

	if(course > courseOld)
	{
		angle = course - courseOld;
	}
	else
	{
		angle = 360 - course + courseOld;
	}

	if( ((MINIMUM_SPEED_TO_RECORD < speed) && (MINIMUM_ANGLE_TO_RECORD < angle)) ||
			(MIN_ALTITUDE_CHANGE < altitudeDiff) )
	{
		Write_FLAG = TRUE;
	}

	if(MAX_SPEED_IN_TRAFFIC_JAM > speed)
	{
		distance = (int32_t)calculate_Distance(latitude, longitude, latitudeOld, longitudeOld);

		if(MAX_DISTANCE_IN_TRAFFIC_JAM < distance)
		{
			Write_FLAG = TRUE;
		}
	}

	if(TRUE == Write_FLAG)
	{
		courseOld = course;
		latitudeOld = latitude;
		longitudeOld = longitude;
		altitudeOld = altitude;
	}

	return Write_FLAG;
}
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */



/**
 * A STATIC function that calculates the course in degrees, where
 * North=0, West=270 from lat and lon position 1 to 2, where
 * both lat and lon are decimal degrees
 *
 * Based on: TinyGPSPlus
 * https://github.com/mikalhart/TinyGPSPlus
 *
 * @param latitudeStart: starting latitude
 * @param longitudeStart: starting longitude
 * @param latitudeEnd: ending latitude
 * @param longitudeEnd: ending longitude
 * @retval float: returns course angle
 */
static float calculate_Course(float latitudeStart, float longitudeStart, float latitudeEnd, float longitudeEnd)
{
	float diffLongitude = to_Radians(longitudeEnd - longitudeStart);

	latitudeStart = to_Radians(latitudeStart);
	latitudeEnd = to_Radians(latitudeEnd);

	float angle1 = sin(diffLongitude) * cos(latitudeEnd);
	float angle2 = sin(latitudeStart) * cos(latitudeEnd) * cos(diffLongitude);

	angle2 = cos(latitudeStart) * sin(latitudeEnd) - latitudeEnd;
	angle2 = atan2(angle1, angle2);

	if( 0.0 > angle2)
	{
		angle2 += M_TWOPI;
	}

	return to_Degrees(angle2);
}
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */



/**
 * A STATIC function that calculates the distance in meters between
 * two position, both as signed decimal degrees lat and lon.
 * According to the TinyGPSPlus lib - it uses great-circle
 * distance computation for hypothetical sphere of radius of
 * 6372795 meters. Rounding might be up to 0.5% wrong due to
 * the Earth not being an exact sphere.
 *
 * Based on: TinyGPSPlus
 * https://github.com/mikalhart/TinyGPSPlus
 *
 * @param latitudeStart: starting latitude
 * @param longitudeStart: starting longitude
 * @param latitudeEnd: ending latitude
 * @param longitudeEnd: ending longitude
 * @retval float: returns distance in meters
 */
static float calculate_Distance(float latitudeStart, float longitudeStart, float latitudeEnd, float longitudeEnd)
{
	float delta = to_Radians(longitudeStart - longitudeEnd);
	float sindiffLongitude = sin(delta);
	float cosDiffLongitude = cos(delta);

	latitudeStart = to_Radians(latitudeStart);
	latitudeEnd = to_Radians(latitudeEnd);

	float sinLatitudeStart = sin(latitudeStart);
	float cosLatitudeStart = cos(latitudeStart);
	float sinLatitudeEnd = sin(latitudeEnd);
	float cosLatitudeEnd = cos(latitudeEnd);

	delta = (cosLatitudeStart * cosLatitudeEnd) - (sinLatitudeStart * cosLatitudeEnd * cosDiffLongitude);
	delta = pow2(delta);
	delta += pow2(cosLatitudeEnd * sindiffLongitude);
	delta = sqrtf(delta);

	float denom = (sinLatitudeStart * sinLatitudeEnd) + (cosLatitudeStart * cosLatitudeEnd * cosDiffLongitude);

	delta = atan2f(delta, denom);

	return (delta * RADIUS_OF_THE_EARTH);
}
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */



/**
 * A STATIC function: converts decimal degrees
 * into radians.
 *
 * Based on: TinyGPSPlus
 * https://github.com/mikalhart/TinyGPSPlus
 *
 * @param decimalDegree: degrees in decimal
 * @retval float: returns radians
 */
static inline float to_Radians(const float decimalDegree)
{
	return (decimalDegree * M_PI / 180.0);
}
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */



/**
 *  A STATIC function: converts radians
 *  into decimal degrees
 *
 * Based on: TinyGPSPlus
 * https://github.com/mikalhart/TinyGPSPlus
 *
 * @param radians: angle in radians
 * @retval float: returns decimal degrees
 */
static inline float to_Degrees(const float radians)
{
	return (radians * 180.0 / M_PI);
}
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */



/**
 *  A STATIC function: returns variable squared
 *
 * Based on: TinyGPSPlus
 * https://github.com/mikalhart/TinyGPSPlus
 *
 * @param var: any value
 * @retval float: returns var to the power of 2
 */
static inline float pow2(float var)
{
	return (var*var);
}
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */


