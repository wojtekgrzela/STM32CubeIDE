/*
 * Task_EEPROM.c
 *
 *  Created on: Jan 11, 2021
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
#include "../../EEPROM/EEPROM.h"
#include "Init_Functions.h"
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Defines */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#ifdef RUNTIME_STATS_QUEUES
	#define QUEUE_EEPROM_WRITE_NAME		"EEPROM_WRITE"
	#define QUEUE_EEPROM_READ_NAME		"EEPROM_READ"
#endif
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Extern variables */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
extern osMessageQId Queue_EEPROM_readHandle;
extern osMessageQId Queue_EEPROM_writeHandle;
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Local variables */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */



void StartEEPROMTask(void const * argument)
{
  /* USER CODE BEGIN StartTaskEEPROM */

	TickType_t xLastWakeTime;
	const TickType_t xFrequency = MY_EEPROM_TASK_TIME_PERIOD;
	Error_Code error = NO_ERROR;

	CREATE_EEPROM_data_struct(EEPROMData);
	uint8_t localBuffer[64] = {0};

#ifdef RUNTIME_STATS_QUEUES
	vQueueAddToRegistry(Queue_EEPROM_writeHandle, QUEUE_EEPROM_WRITE_NAME);
	vQueueAddToRegistry(Queue_EEPROM_readHandle, QUEUE_EEPROM_READ_NAME);
#endif

	/*** EEPROM INIT SEQUENCE ***/

	/* EEPROM Car initialization: */
	error = InitVariablesFromEEPROMCar();
	if(NO_ERROR != error) my_error_handler(error);

	/* EEPROM Board initialization: */
	error = InitVariablesFromEEPROMBoard();
	if(NO_ERROR != error) my_error_handler(error);

	/* FRAM initialization: */
	error = InitVariablesFromFRAM();
	if(NO_ERROR != error) my_error_handler(error);

	xLastWakeTime = xTaskGetTickCount();
	/* Infinite loop */
	for (;;)
	{
		if (pdTRUE == xQueueReceive(Queue_EEPROM_readHandle, &EEPROMData, (TickType_t)0))
		{
			*(EEPROMData.isReadyPtr) = DATA_NOT_READY;

			error = ReadEEPROM(EEPROMData.EEPROMParameters, &EEPROMData);
			if (NO_ERROR == error)
			{
				*(EEPROMData.isReadyPtr) = DATA_READY;    //TODO: mem error dump
			}
			else
			{
				my_error_handler(error);
			}
		}
		else
		{
			if (pdTRUE == xQueueReceive(Queue_EEPROM_writeHandle, &EEPROMData, (TickType_t)0))
			{
				*(EEPROMData.isReadyPtr) = DATA_NOT_READY;
				*(EEPROMData.dataIsCopiedPtr) = DATA_NOT_COPIED;
				if (0u == EEPROMData.size)
				{
					error = EEPROM__SIZE_TO_WRITE_IS_ZERO;
				}
				else
				{
					for (uint8_t i = 0; i < EEPROMData.size; ++i)
					{
						localBuffer[i] = EEPROMData.data[i];
					}
					EEPROMData.data = localBuffer;
					*(EEPROMData.dataIsCopiedPtr) = DATA_COPIED;
					(void)UnlockEEPROM(EEPROMData.EEPROMParameters);
					error = WriteEEPROM(EEPROMData.EEPROMParameters, &EEPROMData);
				}

				if (NO_ERROR == error)
				{
					*(EEPROMData.isReadyPtr) = DATA_READY;    //TODO: mem error dump
				}
				else
				{
					my_error_handler(error);
				}
				vTaskDelay(5);
				(void)LockEEPROM(EEPROMData.EEPROMParameters);
			}
		}

		vTaskDelayUntil(&xLastWakeTime, xFrequency);
	}
	/* USER CODE END StartTaskEEPROM */
}
