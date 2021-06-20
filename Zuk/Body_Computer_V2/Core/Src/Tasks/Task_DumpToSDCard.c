/*
 * Task_DumpToSDCard.c
 *
 *  Created on: Feb 28, 2021
 *      Author: Wojciech Grzelinski
 */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Includes */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "defines.h"
#include "../../VariousFunctions/Functions.h"
#include "../../GPS/GPS_Parsing.h"
#include "fatfs.h"
#include "sdio.h"
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Defines And Typedefs */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#define MAIN_PATH				("")			/* Main path to the disk (that is no path - we write in the main folder) */
#define FILE_NAME_LENGTH		((uint8_t)(15))	/* For example: "01.12.2020.csv" - 14 signs and \0 = 15 */
#define FILE_EXTERNSION_STRING	(".csv")		/* This will be added to the date as a file name, for example: "01.12.2021" + ".csv" */
#define STRINGS_DELIMITER		(",")			/* This is used for csv data */
#define STRINGS_ENDING			("\n\r\0")		/* This is used to finish a string */

#define MAX_LENGTH_CSV_HEADER_TABLE		(128u)
#define MAX_LENGTH_CSV_DATA_TABLE		(128u)
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Extern variables */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
extern SDCard_info_struct SDCard_info;

extern GPS_data_struct GPS;

extern LCD_message RPMForLCD;
extern LCD_message SPEEDForLCD;
extern LCD_message waterTemperatureValueForLCD;

extern boolean EXT_saveSpecialGPSPointDone;
extern boolean EXT_saveSpecialGPSPoint;
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Local variables */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
static FIL fileObj;
static boolean InitialIteration = TRUE;
static uint8_t CSVDataTable[MAX_LENGTH_CSV_DATA_TABLE] = "";
static uint8_t CSVHeadersTable[MAX_LENGTH_CSV_HEADER_TABLE] =
		"Time,Latitude,LatitudeIndicator,Longitude,LongitudeIndicator,Altitude,SpeedByGPS,SatelitesUsed,Speed,RPM,CoolantTemp";

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Functions prototypes */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
static inline void addCSVDelimiter(uint8_t* CSVDataTable, uint16_t* dataBufferPosition);
static inline void addCSVLineEnd(uint8_t* CSVDataTable, uint16_t* dataBufferPosition);

static Error_Code unmountIfNecessary(void);
static Error_Code mountIfNecessary(void);
static Error_Code initializeIfNecessary(void);
static void prepareDataToSave(uint16_t* dataBufferPosition);
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

uint32_t savedBytes = 0u;

void StartDumpToSDCardTask(void const *argument)
{
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = MY_DUMP_TO_SDCARD_TASK_TIME_PERIOD;
	Error_Code error = NO_ERROR;
	FRESULT result = FR_OK;

	static uint8_t fileName[FILE_NAME_LENGTH] = {SPACE_IN_ASCII};
	uint16_t dataBufferPosition = 0u;

	/* Turn on power for MicroSD */
	(void)turnOnPower_MicroSD();

	if(TRUE == SDCard_info.isPresent)
	{
		SDCard_info.isPresentFromBeginning = TRUE;	/* The card is present from the beginning. */
		result = BSP_SD_Init();	/* Initialize the card because it is present. */
	}
	else
	{
		SDCard_info.isPresentFromBeginning = FALSE;	/* The card is not present from the beginning. */
		SDCard_info.needsToBeInitialized = TRUE;	/* Set the flag to indicate that there is an initialization needed. */
	}

	if(FR_OK != result)
	{
		error = SDCARD__INIT_FAILED;
		my_error_handler(error);
	}

	xLastWakeTime = xTaskGetTickCount();
	/* Infinite loop */
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
	for (;;)
	{
		result = FR_OK;
		/* In case the card is NOT present and IS mounted - unmount it immediately. */
		error = unmountIfNecessary();
		if(NO_ERROR != error)
		{
			my_error_handler(error);
		}
		/* Check if SDCard is present and needs to be initialized. */
		error = initializeIfNecessary();
		if(NO_ERROR != error)
		{
			my_error_handler(error);
		}
		/* In case the card IS present and IS initialized but is NOT mounted - mount it immediately. */
		error = mountIfNecessary();
		if(NO_ERROR != error)
		{
			my_error_handler(error);
		}

		if(TRUE == SDCard_info.cardRequestedByUSB)
		{
			SDCard_info.canBePassedToUSB = TRUE;
		}

		/* Check if the SDCard is present and is initialized and is mounted - if yes then proceed to writing data. */
		if((TRUE == SDCard_info.isPresent) && (FALSE == SDCard_info.needsToBeInitialized) && (TRUE == SDCard_info.isMounted) && (FALSE == SDCard_info.cardRequestedByUSB))
		{
			if((TRUE == GPS.DateReady) && (TRUE == GPS.forLCD.clock.messageReadyFLAG))
			{
				error = copy_str_to_buffer((char*)GPS.forLCD.date.messageHandler, (char*)fileName, 0u, GPS.forLCD.date.size);
				error = copy_str_to_buffer(FILE_EXTERNSION_STRING, (char*)fileName, GPS.forLCD.date.size, sizeof(FILE_EXTERNSION_STRING));

				/* If it is the initialization iteration then do some checks. */
				if(TRUE == InitialIteration)
				{
					/* This function tries to create a new file. If the file already exists - it fails.
					 * This way we can get information if the file is or is not already created and
					 * add or not add the headers to it.
					 */
					result = f_open(&fileObj, (char*)fileName, (FA_CREATE_NEW | FA_WRITE));

					/* If the file already exists then try opening in append mode. */
					if(FR_EXIST == result)
					{
						/* FA_OPEN_APPEND - opens the file if it is existing. If not, a new file will be created.
						 * The read/write pointer is set end of the file.
						 */
						result = f_open(&fileObj, (char*)fileName, (FA_OPEN_APPEND | FA_WRITE));

						/* If the result is no OK, then catch the error and go to the error handler. */
						if(FR_OK != result)
						{
							error = SDCARD__WRITING_FAILED;
							my_error_handler(error);
						}
						else
						{
							/* If opened correctly then close it. */
							result = f_close(&fileObj);
						}
					}else if(FR_OK == result)/* If the files does not exist already, but the result is OK then write */
					{
						/* If the file is newly created - add the data headers in the file. */
						result = f_write(&fileObj, (char*)CSVHeadersTable, sizeof(CSVHeadersTable), (UINT*)&savedBytes);

						/* If the result is not OK or nothing was saved to the file - catch the error. */
						if((FR_OK != result) || (0 == savedBytes))
						{
							error = SDCARD__WRITING_FAILED;
							my_error_handler(error);
						}
						else
						{
							/* Close the file after writing */
							result = f_close(&fileObj);
							if(FR_OK != result)
							{
								error = SDCARD__FILE_CLOSE_FAILED;
								my_error_handler(error);
							}
						}
					}
					else	/* No file could be opened or created. */
					{
						error = SDCARD__FILE_OPEN_FAILED;
						my_error_handler(error);
					}
					/* Initialization finished. */
					InitialIteration = FALSE;
				}

				/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
				/* Here is the process of copying data to a buffer which will be written to the file */
				dataBufferPosition = 0u;	/* Clearing the data position value */
				prepareDataToSave(&dataBufferPosition);
				/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

				/* Check if GPS point should be saved and do it if so */
				if(TRUE == track_GPS_movement(&GPS))
				{
					/* FA_OPEN_APPEND - opens the file if it is existing. If not, a new file will be created.
					 * The read/write pointer is set end of the file.
					 */
					result = f_open(&fileObj, (char*)fileName, (FA_OPEN_APPEND | FA_WRITE));
					if (FR_OK != result)
					{
						error = SDCARD__FILE_OPEN_FAILED;
						my_error_handler(error);
					}
					else
					{
						result = f_write(&fileObj, (char*)CSVDataTable, dataBufferPosition, (UINT*)&savedBytes);
						if (FR_OK != result)
						{
							error = SDCARD__WRITING_FAILED;
							my_error_handler(error);
						}

						/* Close the file after writing to it. */
						result = f_close(&fileObj);
						if (FR_OK != result)
						{
							error = SDCARD__FILE_CLOSE_FAILED;
							my_error_handler(error);
						}//(FR_OK != result)
					}
				}//if(TRUE == track_GPS_movement(&GPS))

				/* Write GPS data, parameters data, etc. */
				if(TRUE == EXT_saveSpecialGPSPoint)
				{
					EXT_saveSpecialGPSPoint = FALSE;

					/* FA_OPEN_APPEND - opens the file if it is existing. If not, a new file will be created.
					 * The read/write pointer is set end of the file.
					 */
					result = f_open(&fileObj, "SpecialPoints.csv", (FA_OPEN_APPEND | FA_WRITE));
					if (FR_OK != result)
					{
						error = SDCARD__FILE_OPEN_FAILED;
						my_error_handler(error);
					}
					else
					{
						result = f_write(&fileObj, (char*)CSVDataTable, dataBufferPosition, (UINT*)&savedBytes);
						if (FR_OK != result)
						{
							error = SDCARD__WRITING_FAILED;
							my_error_handler(error);
						}

						/* Close the file after writing to it. */
						result = f_close(&fileObj);
						if (FR_OK != result)
						{
							error = SDCARD__FILE_CLOSE_FAILED;
							my_error_handler(error);
						}//(FR_OK != result)
					}

					EXT_saveSpecialGPSPointDone = TRUE;
				}//if(TRUE == EXT_saveSpecialGPSPoint)
			}
		}

		vTaskDelayUntil(&xLastWakeTime, xFrequency);
	}
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
}



/* This function adds a delimiter to the data buffer in the specified position and increments it */
static inline void addCSVDelimiter(uint8_t* CSVDataTable, uint16_t* dataBufferPosition)
{
	copy_str_to_buffer((char*)STRINGS_DELIMITER, (char*)CSVDataTable, *dataBufferPosition, 1u/*size of a ","*/);
	++(*dataBufferPosition);
}



/* This function adds an ending to the data buffer in the specified position and increments it */
static inline void addCSVLineEnd(uint8_t* CSVDataTable, uint16_t* dataBufferPosition)
{
	copy_str_to_buffer((char*)STRINGS_ENDING, (char*)CSVDataTable, *dataBufferPosition, 3u/*size of a "\n\r\0"*/);
	*dataBufferPosition += 3u;
}



/* This function checks the state of the card and unmounts it if necessary. */
static Error_Code unmountIfNecessary(void)
{
	Error_Code error = NO_ERROR;
	FRESULT result = FR_OK;

	/* The card is NOT present and mounted OR card is present, mounted and requested by USB.*/
	if(((FALSE == SDCard_info.isPresent) && (TRUE == SDCard_info.isMounted)) ||
			((TRUE == SDCard_info.isPresent) && (TRUE == SDCard_info.isMounted) && (TRUE == SDCard_info.cardRequestedByUSB)))
	{
		/* Passing NULL parameter means: unmount (if the SDCard was previously mounted it must be unmounted first. */
		result = f_mount(NULL, MAIN_PATH, UNMOUNT_PARAMETER);
		if (FR_OK == result)
		{
			SDCard_info.isMounted = FALSE;
			/* Initialization needed. */
			InitialIteration = TRUE;
		}
		else
		{
			error = SDCARD__UNMOUNTING_FAILED;
		}
	}

	return error;
}



/* This function checks the state of the card and mounts it if necessary. */
static Error_Code mountIfNecessary(void)
{
	Error_Code error = NO_ERROR;
	FRESULT result = FR_OK;

	/* The card is present, initialized and NOT mounted and NOT requested by USB */
	if((TRUE == SDCard_info.isPresent) && (FALSE == SDCard_info.needsToBeInitialized) && (FALSE == SDCard_info.isMounted) && (FALSE == SDCard_info.cardRequestedByUSB))
	{
		result = f_mount(&SDFatFS, MAIN_PATH, MOUNT_IMMEDIATELY);
		if (FR_OK == result)
		{
			SDCard_info.isMounted = TRUE;
		}
		else
		{
			error = SDCARD__MOUNTING_FAILED;
		}
	}

	return error;
}



/* This function checks if SDCard is present and needs to be initialized - if yes then it makes whole process. */
static Error_Code initializeIfNecessary(void)
{
	Error_Code error = NO_ERROR;
	FRESULT result = FR_OK;

	/* Check if the SDCard is present and needs initialization - if yes then do it. */
	if ((TRUE == SDCard_info.isPresent) && (TRUE == SDCard_info.needsToBeInitialized))
	{
		if ((FR_OK == result) && (TRUE == SDCard_info.isMounted))
		{
			/* Passing NULL parameter means: unmount (if the SDCard was previously mounted it must be unmounted first. */
			result = f_mount(NULL, MAIN_PATH, UNMOUNT_PARAMETER);
			if (FR_OK == result)
			{
				SDCard_info.isMounted = FALSE;
			}
		}
		if (FR_OK == result)
		{
			/* Now the Card can be initialized */
			result = BSP_SD_Init();
			if (FR_OK == result)
			{
				SDCard_info.needsToBeInitialized = FALSE;
			}
		}
		if (FR_OK == result)
		{
			/* Now the Card can be mounted */
			result = f_mount(&SDFatFS, MAIN_PATH, MOUNT_IMMEDIATELY);
			if (FR_OK == result)
			{
				SDCard_info.isMounted = TRUE;
				/* Initialization needed. */
				InitialIteration = TRUE;
			}
		}

		if (FR_OK != result)
		{
			/* If something went wrong - call error handler */
			error = SDCARD__INIT_FAILED;
		}
	}

	return error;
}



static void prepareDataToSave(uint16_t* dataBufferPosition)
{
	if(TRUE == GPS.forLCD.clock.messageReadyFLAG) /* Saving clock */
	{
		copy_str_to_buffer((char*)GPS.forLCD.clock.messageHandler, (char*)CSVDataTable, *dataBufferPosition, GPS.forLCD.clock.size);
		dataBufferPosition += GPS.forLCD.clock.size;
	}
	addCSVDelimiter(CSVDataTable, dataBufferPosition);
	if(TRUE == GPS.forLCD.latitude.messageReadyFLAG) /* Saving latitude */
	{
		copy_str_to_buffer((char*)GPS.forLCD.latitude.messageHandler, (char*)CSVDataTable, *dataBufferPosition, GPS.forLCD.latitude.size);
		dataBufferPosition += GPS.forLCD.latitude.size;
	}
	addCSVDelimiter(CSVDataTable, dataBufferPosition);
	if(TRUE == GPS.forLCD.latitudeIndicator.messageReadyFLAG) /* Saving latitude indicator */
	{
		copy_str_to_buffer((char*)GPS.forLCD.latitudeIndicator.messageHandler, (char*)CSVDataTable, *dataBufferPosition, GPS.forLCD.latitudeIndicator.size);
		dataBufferPosition += GPS.forLCD.latitudeIndicator.size;
	}
	addCSVDelimiter(CSVDataTable, dataBufferPosition);
	if(TRUE == GPS.forLCD.longitude.messageReadyFLAG) /* Saving longitude */
	{
		copy_str_to_buffer((char*)GPS.forLCD.longitude.messageHandler, (char*)CSVDataTable, *dataBufferPosition, GPS.forLCD.longitude.size);
		dataBufferPosition += GPS.forLCD.longitude.size;
	}
	addCSVDelimiter(CSVDataTable, dataBufferPosition);
	if(TRUE == GPS.forLCD.longitudeIndicator.messageReadyFLAG) /* Saving longitude indicator */
	{
		copy_str_to_buffer((char*)GPS.forLCD.longitudeIndicator.messageHandler, (char*)CSVDataTable, *dataBufferPosition, GPS.forLCD.longitudeIndicator.size);
		dataBufferPosition += GPS.forLCD.longitudeIndicator.size;
	}
	addCSVDelimiter(CSVDataTable, dataBufferPosition);
	if(TRUE == GPS.forLCD.altitude.messageReadyFLAG) /* Saving altitude */
	{
		copy_str_to_buffer((char*)GPS.forLCD.altitude.messageHandler, (char*)CSVDataTable, *dataBufferPosition, GPS.forLCD.altitude.size);
		dataBufferPosition += GPS.forLCD.altitude.size;
	}
	addCSVDelimiter(CSVDataTable, dataBufferPosition);
	if(TRUE == GPS.forLCD.speed.messageReadyFLAG) /* Saving speed from GPS */
	{
		copy_str_to_buffer((char*)GPS.forLCD.speed.messageHandler, (char*)CSVDataTable, *dataBufferPosition, GPS.forLCD.speed.size);
		dataBufferPosition += GPS.forLCD.speed.size;
	}
	addCSVDelimiter(CSVDataTable, dataBufferPosition);
	if(TRUE == GPS.forLCD.satellitesUsed.messageReadyFLAG) /* Saving number of used satellites */
	{
		copy_str_to_buffer((char*)GPS.forLCD.satellitesUsed.messageHandler, (char*)CSVDataTable, *dataBufferPosition, GPS.forLCD.satellitesUsed.size);
		dataBufferPosition += GPS.forLCD.satellitesUsed.size;
	}
	addCSVDelimiter(CSVDataTable, dataBufferPosition);
	if(TRUE == SPEEDForLCD.messageReadyFLAG) /* Saving speed calculated from a sensor */
	{
		copy_str_to_buffer((char*)SPEEDForLCD.messageHandler, (char*)CSVDataTable, *dataBufferPosition, SPEEDForLCD.size);
		dataBufferPosition += SPEEDForLCD.size;
	}
	addCSVDelimiter(CSVDataTable, dataBufferPosition);
	if(TRUE == RPMForLCD.messageReadyFLAG) /* Saving RPM calculated */
	{
		copy_str_to_buffer((char*)RPMForLCD.messageHandler, (char*)CSVDataTable, *dataBufferPosition, RPMForLCD.size);
		dataBufferPosition += RPMForLCD.size;
	}
	addCSVDelimiter(CSVDataTable, dataBufferPosition);
	if(TRUE == waterTemperatureValueForLCD.messageReadyFLAG) /* Saving coolant temperature */
	{
		copy_str_to_buffer((char*)waterTemperatureValueForLCD.messageHandler, (char*)CSVDataTable, *dataBufferPosition, waterTemperatureValueForLCD.size);
		dataBufferPosition += waterTemperatureValueForLCD.size;
	}

	addCSVLineEnd(CSVDataTable, dataBufferPosition); /* Line ending added */
}




