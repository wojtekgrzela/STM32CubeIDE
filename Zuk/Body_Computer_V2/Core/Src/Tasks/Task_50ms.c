/*
 * Task_50ms.c
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

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Extern variables */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern volatile int8_t EncoderCounterMenuDiff;
extern volatile int8_t EncoderCounterCruiseDiff;
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Local variables */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Functions prototypes */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */



void Start50msTask(void const *argument)
{
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = MY_TASK_50_MS_TIME_PERIOD;

	volatile uint16_t TIM1CounterReadout = 0;
	volatile uint16_t TIM3CounterReadout = 0;
	static uint16_t previousCounterValue1 = 0;
	static uint16_t previousCounterValue3 = 0;
	volatile int32_t tempDiff1 = 0;
	volatile int32_t tempDiff3 = 0;

	xLastWakeTime = xTaskGetTickCount();
	/* Infinite loop */
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
	for (;;)
	{
		/* Menu Encoder counter reading */
		TIM1CounterReadout = __HAL_TIM_GET_COUNTER(&htim1);
		tempDiff1 = TIM1CounterReadout - previousCounterValue1;

		if ((4 <= tempDiff1) || (-4 >= tempDiff1))
		{
			tempDiff1 /= 4;
			EncoderCounterMenuDiff += (int8_t)tempDiff1;
			previousCounterValue1 = TIM1CounterReadout;
		}

		/* Cruise Control Encoder counter reading */
		TIM3CounterReadout = __HAL_TIM_GET_COUNTER(&htim3);
		tempDiff3 = TIM3CounterReadout - previousCounterValue3;

		if((4 <= tempDiff3) || (-4 >= tempDiff3))
		{
			tempDiff3 /= 4;
			EncoderCounterCruiseDiff = TIM3CounterReadout;
			previousCounterValue3 = TIM3CounterReadout;
		}

		vTaskDelayUntil(&xLastWakeTime, xFrequency);
	}
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
}
