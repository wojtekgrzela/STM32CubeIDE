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
#include "../../lcd_hd44780_i2c/lcd_hd44780_i2c.h"
#include "../../PID/PID.h"
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Defines And Typedefs */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#define P_REGULATOR_GAIN_ADC	((float)(5.0))
#define I_REGULATOR_GAIN_ADC	((float)(0.5))
#define D_REGULATOR_GAIN_ADC	((float)(1.0))

/* TIMERs defines */
#define PWM_TIMER_CRUISE_CONTROL			(TIM9)
#define PWM_RESOLUTION_CRUISE_CONTROL		(PWM_TIMER_CRUISE_CONTROL->ARR)
#define PWM_PULSE_CRUISE_CONTROL			(PWM_TIMER_CRUISE_CONTROL->CCR1)
#define MIN_PULSE							(uint16_t)(0)
#define MAX_PULSE							(uint16_t)(100)

/* H-BRIDGE defines */
#define DECREASE_ENG_TERMINAL_PORT	(IN2_CNTRL_ENGINE_GPIO_Port)
#define DECREASE_ENG_TERMINAL_PIN	(IN2_CNTRL_ENGINE_Pin)
#define INCREASE_ENG_TERMINAL_PORT	(IN1_CNTRL_ENGINE_GPIO_Port)
#define INCREASE_ENG_TERMINAL_PIN	(IN1_CNTRL_ENGINE_Pin)

#define MAX_CONTROL_SIGNAL			((uint16_t)(100))
#define MIN_CONTROL_SIGNAL			((uint16_t)(0))

#define SHUTDOWN_HIGH_RPM			((uint32_t)(3800))
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Extern variables */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern volatile int8_t EncoderCounterMenuDiff;
extern volatile int8_t EncoderCounterCruiseDiff;

extern volatile uint16_t ADCDesiredAccelerationValue;
extern volatile CruiseControlParameters_struct cruiseControlParam;
extern CarStateinfo_type CarStateInfo;
extern volatile ENCButton_struct ENC_button_cruise;

extern boolean EXT_boardChangeRequest;
extern LCD_board *EXT_boardPtr;
extern LCD_board LCD_CruiseControl;
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Local variables */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Functions prototypes */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
static inline void Set_Electromagnes_On(void);
static inline void Set_Electromagnes_Off(void);
void Complete_Shutdown(void);
static void Set_Direction(Enum_EngineDirection direction);
static void Encoder_CruiseControl_Button_Function(void);
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */



void Start50msTask(void const *argument)
{
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = MY_TASK_50_MS_TIME_PERIOD;

	static boolean previousState = OFF;
	volatile int16_t ADCErrorVal = 0;
	PIDparameters_t PID_param =
		{ 	.P = P_REGULATOR_GAIN_ADC,
			.I = I_REGULATOR_GAIN_ADC,
			.D = D_REGULATOR_GAIN_ADC,
			.dt = MY_TASK_50_MS_TIME_PERIOD };
	float PID_result = 0.0;

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
		TIM1CounterReadout = __HAL_TIM_GET_COUNTER(&htim1); /* Reading the counter value */
		tempDiff1 = TIM1CounterReadout - previousCounterValue1; /* Calculating the diff between previous value and the current one */

		if ((4 <= tempDiff1) || (-4 >= tempDiff1))
		{
			tempDiff1 /= 4;	/* Dividing by 4 because we react for all 4 edges for one scroll */
			EncoderCounterMenuDiff += (int8_t)tempDiff1;	/* Casting for int8_t allows to get the proper value */
			previousCounterValue1 = TIM1CounterReadout;
		}

		/* Cruise Control Encoder counter reading */
		TIM3CounterReadout = __HAL_TIM_GET_COUNTER(&htim3); /* Reading the counter value */
		tempDiff3 = TIM3CounterReadout - previousCounterValue3; /* Calculating the diff between previous value and the current one */

		if((4 <= tempDiff3) || (-4 >= tempDiff3))
		{
			tempDiff3 /= 4;	/* Dividing by 4 because we react for all 4 edges for one scroll */
			EncoderCounterCruiseDiff = TIM3CounterReadout;	/* Casting for int8_t allows to get the proper value */
			previousCounterValue3 = TIM3CounterReadout;
		}

		/* Checks the state of the button of the cruise control Encoder and reacts for it */
		Encoder_CruiseControl_Button_Function();

		/* Check if the RPM is not too high when the cruise control is ON - if yes then shut it down */
		if((ON == cruiseControlParam.state) && (SHUTDOWN_HIGH_RPM < CarStateInfo.RPM))
		{
			/* Shutdown the cruise control */
			Complete_Shutdown();
		}

		if(ON == cruiseControlParam.state)
		{
			if(OFF == previousState)
			{
				/* Turn on cruise control */
				Set_Electromagnes_On();

				/* If in previous state cruise control was off - turn on the engine for a little bit with full power
				 * to stretch the line just a bit
				 */
				PWM_PULSE_CRUISE_CONTROL = MAX_PULSE;

				/* Set the pointer to the Cruise Control Board */
				EXT_boardPtr = &LCD_CruiseControl;

				/* Set the change request flag to TRUE */
				EXT_boardChangeRequest = TRUE;
			}
			else
			{
				/* Set PWM duty cycle to min value which is 0 - turn off the engine */
				PWM_PULSE_CRUISE_CONTROL = MIN_PULSE;
				/* Read current ADC value for accelerator */
				CarStateInfo.ADC_AcceleratorValue = ACC_POSITION_ADC_VALUE;

				/* Calculate the error between the desired and the actual position */
				ADCErrorVal = ADCDesiredAccelerationValue - CarStateInfo.ADC_AcceleratorValue;

				/* Calculate the result (PWM signal width) */
				PID_result = RunPIDController((float)ADCErrorVal, &PID_param);

				/* If the result is negative - make it positive and set direction to decrease the throttle */
				if(0.0f > PID_result)
				{
					PID_result *= (-1);	/* Make the value positive */
					Set_Direction(DECREASE);
				}
				else
				{
					/* If the result is positive - set the direction to increase the throttle */
					Set_Direction(INCREASE);
				}

				/* If the signal is bigger than the maximum allowed PWM duty cycle - set it to the max */
				if(MAX_CONTROL_SIGNAL < PID_result)
				{
					PID_result = MAX_CONTROL_SIGNAL;
				}

				/* Write the calculated PWM duty cycle to the registers */
				PWM_PULSE_CRUISE_CONTROL = (uint16_t)PID_result;
			}

			/* Set the previous State variable to ON - because the cruise control is on */
			previousState = ON;
		}
		else
		{
			/* Make shure everything is off */
			Complete_Shutdown();
			/* Set the previous State variable to OFF - because the cruise control is off */
			previousState = OFF;
		}

		vTaskDelayUntil(&xLastWakeTime, xFrequency);
	}
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
}



/* This function turns the electromagnetic clutch ON */
static inline void Set_Electromagnes_On(void)
{
	HAL_GPIO_WritePin(EN_ELECTRO_CLUTCH_GPIO_Port, EN_ELECTRO_CLUTCH_Pin, SET);
}



/* This function turns the electromagnetic clutch OFF */
static inline void Set_Electromagnes_Off(void)
{
	HAL_GPIO_WritePin(EN_ELECTRO_CLUTCH_GPIO_Port, EN_ELECTRO_CLUTCH_Pin, RESET);
}



/* This function shuts down completely cruise control */
void Complete_Shutdown(void)
{
	cruiseControlParam.state = OFF;
	Set_Electromagnes_Off();
	Set_Direction(NOTHING);
	PWM_PULSE_CRUISE_CONTROL = MIN_PULSE;	/* Min pulse is 0 - turning off completely */
}



/* This function sets direction in which the cruise contorl engine should rotate */
static void Set_Direction(Enum_EngineDirection direction)
{
	switch (direction)
	{
		case DECREASE:
		{
			HAL_GPIO_WritePin(INCREASE_ENG_TERMINAL_PORT, INCREASE_ENG_TERMINAL_PIN, RESET);
			HAL_GPIO_WritePin(DECREASE_ENG_TERMINAL_PORT, DECREASE_ENG_TERMINAL_PIN, SET);
			break;
		}
		case INCREASE:
		{
			HAL_GPIO_WritePin(INCREASE_ENG_TERMINAL_PORT, INCREASE_ENG_TERMINAL_PIN, SET);
			HAL_GPIO_WritePin(DECREASE_ENG_TERMINAL_PORT, DECREASE_ENG_TERMINAL_PIN, RESET);
			break;
		}
		case NOTHING:
		default:
		{
			HAL_GPIO_WritePin(INCREASE_ENG_TERMINAL_PORT, INCREASE_ENG_TERMINAL_PIN, RESET);
			HAL_GPIO_WritePin(DECREASE_ENG_TERMINAL_PORT, DECREASE_ENG_TERMINAL_PIN, RESET);
			break;
		}
	}
}



static void Encoder_CruiseControl_Button_Function(void)
{
	if(TRUE == ENC_button_cruise.longPressDetected)	/* In case a long press is detected - change mode of the cruise control */
	{
		if(CONSTANT_SPEED == cruiseControlParam.mode)
		{
			cruiseControlParam.mode = CONSTANT_RPM; /* If it was a constant speed - set constant RPM */
		}
		else
		{
			cruiseControlParam.mode = CONSTANT_SPEED; /* If it was a constant RPM - set constant speed */
		}

		/* Clear short and long press detections */
		ENC_button_cruise.shortPressDetected = FALSE;
		ENC_button_cruise.longPressDetected = FALSE;
	}
	else if(TRUE == ENC_button_cruise.shortPressDetected)
	{
		if(ON == cruiseControlParam.state)
		{
			/* Shutdown the cruise control completely */
			Complete_Shutdown();
		}
		else
		{
			/* Set the state to ON and switch ON the electromagnetic clutch */
			cruiseControlParam.state = ON;
			Set_Electromagnes_On();
		}

		/* Clear short and long press detections */
		ENC_button_cruise.shortPressDetected = FALSE;
		ENC_button_cruise.longPressDetected = FALSE;
	}
}
