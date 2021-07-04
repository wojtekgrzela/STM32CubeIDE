/*
 * Task_500ms.c
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
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Defines And Typedefs */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#define ITERATION_1		((uint16_t)(1u))
#define ITERATION_2		((uint16_t)(2u))
#define ITERATION_4		((uint16_t)(4u))
#define ITERATION_6		((uint16_t)(6u))

#define MAX_NO_OF_ITERATIONS_TO_FULLY_OPERATIONAL	((uint32_t)(8))

#define task_50ms_counter_MIN	((WDGCounter)(8))	/* Task period: 50ms */
#define task_AlarmControl_MIN	((WDGCounter)(1))	/* Task period: 250ms */
#define task_CruiseControl_MIN	((WDGCounter)(1))	/* Task period: 500ms */
#define task_DiagCheck_MIN		((WDGCounter)(1))	/* Task period: 1000ms */
#define task_DumpToEEPROM_MIN	((WDGCounter)(1))	/* Task period: 1000ms */
#define task_DumpToSDCard_MIN	((WDGCounter)(1))	/* Task period: 1000ms */
#define task_EEPROM_MIN			((WDGCounter)(2))	/* Task period: 200ms */
#define task_GPS_MIN			((WDGCounter)(1))	/* Task period: 1000ms */
#define task_LCD_MIN			((WDGCounter)(1))	/* Task period: 200ms - but max used delay is 2000ms */
#define task_Measure_MIN		((WDGCounter)(1))	/* Task period: 250ms */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Extern variables */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
extern IWDG_HandleTypeDef hiwdg;
extern boolean SYSTEM_IS_UP_FLAG;
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Local variables */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
WDGCounter task_50ms_counter_WDG = 0u;
WDGCounter task_AlarmControl_WDG = 0u;
WDGCounter task_CruiseControl_WDG = 0u;
WDGCounter task_DiagCheck_WDG = 0u;
WDGCounter task_DumpToEEPROM_WDG = 0u;
WDGCounter task_DumpToSDCard_WDG = 0u;
WDGCounter task_EEPROM_WDG = 0u;
WDGCounter task_GPS_WDG = 0u;
WDGCounter task_LCD_WDG = 0u;
WDGCounter task_Measure_WDG = 0u;

static boolean oneSecond_FLAG = FALSE;
static boolean twoSecond_FLAG = FALSE;
static boolean threeSecond_FLAG = FALSE;
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Functions prototypes */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */



void Start500msTask(void const *argument)
{
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = MY_TASK_500_MS_TIME_PERIOD;

	static uint32_t IterationCounter = 0u;
	static uint16_t IterationCounter_1sec = 0u;
	static uint16_t IterationCounter_2sec = 0u;
	static uint16_t IterationCounter_3sec = 0u;

	static boolean task_50ms_FAILED = FALSE;
	static boolean task_AlarmControl_FAILED = FALSE;
	static boolean task_CruiseControl_FAILED = FALSE;
	static boolean task_DiagCheck_FAILED = FALSE;
	static boolean task_DumpToEEPROM_FAILED = FALSE;
	static boolean task_DumpToSDCard_FAILED = FALSE;
	static boolean task_EEPROM_FAILED = FALSE;
	static boolean task_GPS_FAILED = FALSE;
	static boolean task_LCD_FAILED = FALSE;
	static boolean task_Measure_FAILED = FALSE;

	(void)HAL_IWDG_Refresh(&hiwdg);

	xLastWakeTime = xTaskGetTickCount();

	/* Infinite loop */
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
	for (;;)
	{
		/* Check if system is up or not yet */
		if(TRUE == SYSTEM_IS_UP_FLAG)
		{
			(ITERATION_2 == IterationCounter_1sec) ? (oneSecond_FLAG = TRUE) : (oneSecond_FLAG = FALSE);		/* If 1 second passed - set flag ON */
			(ITERATION_4 == IterationCounter_2sec) ? (twoSecond_FLAG = TRUE) : (twoSecond_FLAG = FALSE);		/* If 2 seconds passed - set flag ON */
			(ITERATION_6 == IterationCounter_3sec) ? (threeSecond_FLAG = TRUE) : (threeSecond_FLAG = FALSE);	/* If 3 seconds passed - set flag ON */

			/* Tasks checked every 500ms: */
			(task_50ms_counter_MIN <= task_50ms_counter_WDG) ? (task_50ms_FAILED = FALSE) : (task_50ms_FAILED = TRUE);
			task_50ms_counter_WDG = 0;
			(task_AlarmControl_MIN <= task_AlarmControl_WDG) ? (task_AlarmControl_FAILED = FALSE) : (task_AlarmControl_FAILED = TRUE);
			task_AlarmControl_WDG = 0;
			(task_EEPROM_MIN <= task_EEPROM_WDG) ? (task_EEPROM_FAILED = FALSE) : (task_EEPROM_FAILED = TRUE);
			task_EEPROM_WDG = 0;
			(task_Measure_MIN <= task_Measure_WDG) ? (task_Measure_FAILED = FALSE) : (task_Measure_FAILED = TRUE);
			task_Measure_WDG = 0;

			if(TRUE == oneSecond_FLAG)	/* Tasks checked every 1000ms: */
			{
				(task_CruiseControl_MIN <= task_CruiseControl_WDG) ? (task_CruiseControl_FAILED = FALSE) : (task_CruiseControl_FAILED = TRUE);
				task_CruiseControl_WDG = 0;
			}
			if(TRUE == twoSecond_FLAG)	/* Tasks checked every 2000ms: */
			{
				(task_DiagCheck_MIN <= task_DiagCheck_WDG) ? (task_DiagCheck_FAILED = FALSE) : (task_DiagCheck_FAILED = TRUE);
				task_DiagCheck_WDG = 0;
				(task_DumpToEEPROM_MIN <= task_DiagCheck_WDG) ? (task_DumpToEEPROM_FAILED = FALSE) : (task_DumpToEEPROM_FAILED = TRUE);
				task_DiagCheck_WDG = 0;
				(task_DumpToSDCard_MIN <= task_DiagCheck_WDG) ? (task_DumpToSDCard_FAILED = FALSE) : (task_DumpToSDCard_FAILED = TRUE);
				task_DiagCheck_WDG = 0;
				(task_GPS_MIN <= task_GPS_WDG) ? (task_GPS_FAILED = FALSE) : (task_GPS_FAILED = TRUE);
				task_GPS_WDG = 0;
			}
			if(TRUE == threeSecond_FLAG)/* Tasks checked every 3000ms: */
			{
				(task_LCD_MIN <= task_LCD_WDG) ? (task_LCD_FAILED = FALSE) : (task_LCD_FAILED = TRUE);
				task_LCD_WDG = 0;
			}

			/* If system is fully up - check flags */
			if ((FALSE == task_50ms_FAILED) && (FALSE == task_AlarmControl_FAILED) && (FALSE == task_EEPROM_FAILED)
					&& (FALSE == task_Measure_FAILED) && (FALSE == task_CruiseControl_FAILED) && (FALSE == task_DiagCheck_FAILED)
					&& (FALSE == task_DumpToEEPROM_FAILED) && (FALSE == task_DumpToSDCard_FAILED) && (FALSE == task_GPS_FAILED)
					&& (FALSE == task_LCD_FAILED))
			{
				/* If no FAILURE flag is TRUE then reset IWDG */
				(void)HAL_IWDG_Refresh(&hiwdg);
			}

			if(TRUE == oneSecond_FLAG)
			{
				IterationCounter_1sec = ITERATION_1;
			}
			else
			{
				++IterationCounter_1sec;
			}
			if(TRUE == twoSecond_FLAG)
			{
				IterationCounter_2sec = ITERATION_1;
			}
			else
			{
				++IterationCounter_2sec;
			}
			if(TRUE == threeSecond_FLAG)
			{
				IterationCounter_3sec = ITERATION_1;
			}
			else
			{
				++IterationCounter_3sec;
			}
		}
		else
		{
			/* If system is not fully up yet - reset if iteration number NOT exceeded */
			if(MAX_NO_OF_ITERATIONS_TO_FULLY_OPERATIONAL >= IterationCounter)
			{
				(void)HAL_IWDG_Refresh(&hiwdg);
			}
		}


		HAL_GPIO_TogglePin(DIAG_LED_1_GPIO_Port, DIAG_LED_1_Pin);
		HAL_GPIO_TogglePin(DIAG_LED_2_GPIO_Port, DIAG_LED_2_Pin);
		HAL_GPIO_TogglePin(DIAG_LED_3_GPIO_Port, DIAG_LED_3_Pin);
		HAL_GPIO_TogglePin(DIAG_LED_4_GPIO_Port, DIAG_LED_4_Pin);


		++IterationCounter;
		vTaskDelayUntil(&xLastWakeTime, xFrequency);
	}
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
}
