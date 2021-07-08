/*
 * Task_DumpToEEPROM.c
 *
 *  Created on: Jan 11, 2021
 *      Author: Dell
 */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Includes */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "defines.h"
#include "../../EEPROM/EEPROM.h"
#include "../../GPS/GPS_Parsing.h"
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Defines */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#ifdef RUNTIME_STATS_QUEUES
	#define QUEUE_DIAGNOSTIC_DUMP_NAME	"DIAGNOSTIC_DUMP"
	#define QUEUE_ERROR_DUMP_NAME		"ERROR_DUMP"
#endif
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Extern variables */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
extern osMessageQId Queue_error_snapshot_dumpHandle;
extern osMessageQId Queue_diagnostic_snapshot_dumpHandle;

extern EEPROM_parameters_struct EEPROM_car;
extern EEPROM_parameters_struct EEPROM_board;

extern GPS_data_struct GPS;

extern WDGCounter task_DumpToEEPROM_WDG;
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Local variables */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */



void StartDumpToEEPROMTask(void const * argument)
{
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = MY_DUMP_TO_EEPROM_TASK_TIME_PERIOD;
	Error_Code error = NO_ERROR;

	DiagnosticDataToEEPROM_struct DiagnosticDataToSend =
	{ .DiagnosticDataForEEPROM =
		{ .EEPROMParameters = &EEPROM_car,
		.data = DiagnosticDataToSend.data,
		.size = MAX_DIAGNOSTIC_SNAPSHOT_SIZE,
		.memAddress = 0u,
		.memAddressSize = EEPROM_PAGES_ADDRESS_SIZE },
	.diag_mess_from_queue =
		{ .snapshotIdentificator = DIAGNOSTICS_OK,
		.value = 0 } };

	INITIALIZE_EEPROM_data_struct(DiagnosticDataToSend.DiagnosticDataForEEPROM);

	ErrorDataToEEPROM_struct ErrorDataToSend =
	{ .ErrorDataForEEPROM.EEPROMParameters = &EEPROM_board,
	.error_mess_from_queue = NO_ERROR };

	INITIALIZE_EEPROM_data_struct(ErrorDataToSend.ErrorDataForEEPROM);

#ifdef RUNTIME_STATS_QUEUES
	vQueueAddToRegistry(Queue_diagnostic_snapshot_dumpHandle, QUEUE_DIAGNOSTIC_DUMP_NAME);
	vQueueAddToRegistry(Queue_error_snapshot_dumpHandle, QUEUE_ERROR_DUMP_NAME);
#endif

	xLastWakeTime = xTaskGetTickCount();

  /* Infinite loop */
  for(;;)
  {
	  if(pdTRUE == xQueueReceive(Queue_diagnostic_snapshot_dumpHandle, &(DiagnosticDataToSend.diag_mess_from_queue), (TickType_t)0))
	  {
		  *(DiagnosticDataToSend.DiagnosticDataForEEPROM.isReadyPtr) = DATA_NOT_READY;
		  DiagnosticDataToSend.longitudeIndicator = (TRUE == GPS.Fix) ? GPS.rawData.LongitudeIndicator[0] : 'X';
		  DiagnosticDataToSend.latitudeIndicator = (TRUE == GPS.Fix) ? GPS.rawData.LatitudeIndicator[0] : 'X';

		  for(uint8_t i = 0; i<11; ++i)
		  {
			  if(8>i)
			  {
				  DiagnosticDataToSend.clockTime[i] = (TRUE == GPS.TimeReady) ? GPS.forLCD.clock.messageHandler[i] : 'x';
			  }

			  if(10>i)
			  {
				  DiagnosticDataToSend.latitude[i] = (TRUE == GPS.Fix) ? GPS.rawData.Latitude[i] : 'y';
			  }

			  DiagnosticDataToSend.longitude[i] = (TRUE == GPS.Fix) ? GPS.rawData.Longitude[i] : 'z';
		  }

		  error = WriteEEPROM(DiagnosticDataToSend.DiagnosticDataForEEPROM.EEPROMParameters, &(DiagnosticDataToSend.DiagnosticDataForEEPROM));

		  if(NO_ERROR == error)
		  {
			  *(DiagnosticDataToSend.DiagnosticDataForEEPROM.isReadyPtr) = DATA_READY; //TODO
		  }
		  else
		  {
			  my_error_handler(error);
		  }

	  }
	  else
	  {
		  if(pdTRUE == xQueueReceive(Queue_error_snapshot_dumpHandle, &(ErrorDataToSend.error_mess_from_queue), (TickType_t)0))
		  {
			  *(ErrorDataToSend.ErrorDataForEEPROM.isReadyPtr) = DATA_NOT_READY;

			  for(uint8_t i = 0; i<11; ++i)
			  {
				  ErrorDataToSend.clockTime[i] = (TRUE == GPS.TimeReady) ? GPS.forLCD.clock.messageHandler[i] : 'x';
			  }

			  error = WriteEEPROM(ErrorDataToSend.ErrorDataForEEPROM.EEPROMParameters, &(ErrorDataToSend.ErrorDataForEEPROM));
			  if(NO_ERROR == error)
			  {
				  *(ErrorDataToSend.ErrorDataForEEPROM.isReadyPtr) = DATA_READY; //TODO
			  }
			  else
			  {
				  my_error_handler(error);
			  }
			  vTaskDelay((TickType_t)5u);
		  }
	  }

	  ++task_DumpToEEPROM_WDG;
	  vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}
