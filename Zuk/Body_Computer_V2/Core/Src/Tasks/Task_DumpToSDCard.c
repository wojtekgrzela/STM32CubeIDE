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
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Extern variables */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
extern SDCard_info_struct SDCard_info;

extern GPS_data_struct GPS;
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Local variables */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
//static FATFS SD_FATFS;
static FIL fileObj;
static FRESULT *my_global_result = NULL;
uint32_t count = 0;
uint32_t lastVal = 0;
uint32_t dupa = 0;

uint8_t TEMPBUFF[20];
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Functions prototypes */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
static Error_Code unmountIfNecessary(void);
static Error_Code mountIfNecessary(void);
static Error_Code initializeIfNecessary(void);
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

uint32_t savedBytes = 0u;

void StartDumpToSDCardTask(void const *argument)
{
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = MY_DUMP_TO_SDCARD_TASK_TIME_PERIOD;
	Error_Code error = NO_ERROR;
	FRESULT result = FR_OK;

	static boolean FirstIteration = TRUE;
	static uint8_t fileName[FILE_NAME_LENGTH] = {SPACE_IN_ASCII};
	static uint8_t tempFileName[] = "tempF.csv";
	my_global_result = &result;


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
//			if((TRUE == GPS.DateReady) && (TRUE == GPS.forLCD.clock.messageReadyFLAG))
//			{
//				error = copy_str_to_buffer((char*)GPS.forLCD.date.messageHandler, (char*)fileName, 0u, GPS.forLCD.date.size);
//				error = copy_str_to_buffer(FILE_EXTERNSION_STRING, (char*)fileName, GPS.forLCD.date.size, sizeof(FILE_EXTERNSION_STRING));
				/* FA_OPEN_APPEND - opens the file if it is existing. If not, a new file will be created.
				 * The read/write pointer is set end of the file.
				 */
				f_open(&fileObj, "plik.txt", (FA_OPEN_APPEND | FA_WRITE));
				if (FR_OK != result)
				{
					continue;
				}

				f_write(&fileObj, "zawartosc pliku\n", 15, (UINT*)&savedBytes);

				if (FR_OK != result)
				{

				}

				result = f_close(&fileObj);


				if (FR_OK != result)
				{
					continue;
				}
		}

		dupa++;

		vTaskDelayUntil(&xLastWakeTime, xFrequency);
	}
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
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





