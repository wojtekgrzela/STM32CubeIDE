/*
 * GPS_Parsing.c
 *
 *  Created on: May 1, 2020
 *      Author: Wojciech Grzelinski
 */


#include "GPS_Parsing.h"
#include "../VariousFunctions/Functions.h"



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
	int16_t index1 = -1;
	int16_t index2 = -1;
	int16_t tempIndex = -1;
	int16_t nIndex = -1;
	int16_t GPGGAIndex = -1;
	int16_t GPVTGIndex = -1;
	int16_t indexDiff = 0;
	Error_Code error = NO_ERROR;

	for(int i=0; i<GPS_BUFFER_SIZE; ++i)
	{
		GPS_message[i] = GPS->GPS_buffer[i];
	}

	memset((uint8_t *)(GPS->GPS_buffer), 0, GPS_BUFFER_SIZE);
	GPS->DataReady = RESET;

	nIndex = find_nearest_symbol('\n', (const char* const)GPS_message, 0);

	if (TRUE
			== compare_two_strings("$GPGGA", (const char* const ) GPS_message,
					0 /*Start position of the comparison*/,
					6 /*Number of characters to compare*/))
	{
		GPGGAIndex = 0;
	}
	else
	{
		if (TRUE
				== compare_two_strings("$GPGGA",
						(const char* const ) GPS_message,
						nIndex + 1 /*Start position of the comparison*/,
						6 /*Number of characters to compare*/))
		{
			GPGGAIndex = nIndex + 1;
		}
	}

	if (TRUE
			== compare_two_strings("$GPVTG", (const char* const ) GPS_message,
					0 /*Start position of the comparison*/,
					6 /*Number of characters to compare*/))
	{
		GPVTGIndex = 0;
	}
	else
	{
		if (TRUE
				== compare_two_strings("$GPVTG",
						(const char* const ) GPS_message,
						nIndex + 1 /*Start position of the comparison*/,
						6 /*Number of characters to compare*/))
		{
			GPVTGIndex = nIndex + 1;
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
					error = copy_buffer_to_str((char*) GPS_message,(char*) GPS->rawData.status, index1 + 1, indexDiff - 1);
					if (('1' == GPS->rawData.status[0])
							|| ('2' == GPS->rawData.status[0])
							|| ('3' == GPS->rawData.status[0])
							|| ('4' == GPS->rawData.status[0])
							|| ('5' == GPS->rawData.status[0])
							|| ('6' == GPS->rawData.status[0]))
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

	return error;
}
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */


