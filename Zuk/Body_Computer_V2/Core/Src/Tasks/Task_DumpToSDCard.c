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
#include "fatfs.h"
#include "sdio.h"
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Defines And Typedefs */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Extern variables */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Local variables */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

uint8_t tuBylemFLAG = 0;
uint8_t TEMPBUFF[100];
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Functions prototypes */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */



void StartDumpToSDCardTask(void const *argument)
{
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = MY_DUMP_TO_SDCARD_TASK_TIME_PERIOD;
	Error_Code error = NO_ERROR;

	FRESULT result = FR_OK;

	char ReadBuffer[100] = "";
	uint32_t NoOfReadBytes = 0;

	/* Check if SD card is ready to be used */
	while (HAL_SD_STATE_READY != hsd.State)
	{
		vTaskDelay((TickType_t)100);
	}

	result = BSP_SD_Init();

	if (FR_OK == result)
	{
		result = f_mount(&SDFatFS, "", 1);
	}
	if (FR_OK == result)
	{
		result = f_open(&SDFile, "test.txt", FA_READ);
	}
	if (FR_OK == result)
	{
		result = f_read(&SDFile, ReadBuffer, 20, (UINT*)&NoOfReadBytes);
	}
	if (FR_OK == result)
	{
		result = f_close(&SDFile);
	}

	if (TRUE != compare_two_strings(ReadBuffer, TEST_MESSAGE_FOR_CHECK, 0u, 20u))
	{
		error = SDCARD__INITIAL_READ_FAILED;
		my_error_handler(error);

		/* Stop the task now, because if the card is not read correctly then do not proceed! */
//		vTaskSuspend(My_DumpToSDCardHandle);
	}

	error = copy_str_to_buffer(ReadBuffer, (char*)TEMPBUFF, 0, 20);
	tuBylemFLAG = 1;

	xLastWakeTime = xTaskGetTickCount();

	/* Infinite loop */
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
	for (;;)
	{
		vTaskDelayUntil(&xLastWakeTime, xFrequency);
	}
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
}
