/*
 * Task_1000ms.c
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
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Defines And Typedefs */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#define P_REGULATOR_GAIN			((float)(1.0))
#define I_REGULATOR_GAIN			((float)(0.5))
#define D_REGULATOR_GAIN			((float)(1.0))
#define DT_SAMPLING_RERIOD			((float)(1000))		/* in ms */

/* ADC defines */
#define MAX_ADC_ACCELERATION_READOUT	((uint32_t)(2000))	//TODO: max what an ADC can read as a max gas pedal position
#define MIN_ADC_ACCELERATION_READOUT	((uint32_t)(1000))	//TODO: min what an ADC can read as a min gas pedal position
#define ADC_VALUE_MULTIPLER				((uint32_t)(1))		// in case the control values were too strong for the ADC values, the values can be adjusted
#define ADC_ERROR_GAIN					((int32_t)(1))

/* RPM sefines */
#define MAX_ALLOWED_RPM				((uint32_t)(3500))
#define MIN_ALLOWED_RPM				((uint32_t)(1200))
#define SHUTDOWN_HIGH_RPM			((uint32_t)(3700))
#define ENCODER_RPM_SETTING_STEP	((uint32_t)(10))
#define IMPULSES_PER_ONE_ENGINE_REVOLUTION	((uint32_t)(12))

/* TIMERs defines */
#define PWM_TIMER					(TIM3)
#define PWM_RESOLUTION				(PWM_TIMER->ARR)
#define PWM_PULSE					(PWM_TIMER->CCR3)

/* H-BRIDGE defines */
#define DECREASE_ENG_TERMINAL_PORT	(IN2_ENG_GPIO_Port)
#define DECREASE_ENG_TERMINAL_PIN	(IN2_ENG_Pin)
#define INCREASE_ENG_TERMINAL_PORT	(IN1_ENG_GPIO_Port)
#define INCREASE_ENG_TERMINAL_PIN	(IN1_ENG_Pin)

/* REGULATING constants */
#define MAX_CONTROL_SIGNAL			((int32_t)(100))
#define MIN_CONTROL_SIGNAL			((int32_t)(-100))


typedef enum {
	NOTHING = 						(0),
	DECREASE = 						(-1),
	INCREASE = 						(1)
}Enum_EngineDirection;

typedef enum {
	CONSTANT_SPEED = 0,
	CONSTANT_RPM   = 1
}Enum_CruiseMode;

typedef struct {
	boolean state;
	Enum_CruiseMode mode;
	uint32_t wantedSpeed;
	uint32_t wantedRPM;
	int32_t error;

}CruiseControlParameters_struct;
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Extern variables */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
extern CarStateinfo_type CarStateInfo;
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Local variables */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
static volatile CruiseControlParameters_struct cruiseControlParam = {0};
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Functions prototypes */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
static inline void Set_Electromagnes_On(void);
static inline void Set_Electromagnes_On(void);
void Complete_Shutdown(void);
static void Set_Direction(Enum_EngineDirection direction);
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */



void StartCruiseCntrlTask(void const *argument)
{
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = MY_TASK_1000_MS_TIME_PERIOD;
	Error_Code error = NO_ERROR;
	volatile float controlValue = 0.0;


	xLastWakeTime = xTaskGetTickCount();

	/* Infinite loop */
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
	for (;;)
	{
		if(CONSTANT_SPEED == cruiseControlParam.mode)
		{
			cruiseControlParam.error = cruiseControlParam.wantedSpeed - CarStateInfo.SPEED;
		}
		else
		{
			cruiseControlParam.error = cruiseControlParam.wantedRPM - CarStateInfo.RPM;
		}

		ACC_POSITION_ADC_VALUE;

		if (ON == cruiseControlParam.state)
		{
			PWM_PULSE = 0;
			Set_Electromagnes_On();
			controlValue = RunPIDController((float)cruiseControlParam.error);

			if (MIN_CONTROL_SIGNAL > (int32_t)controlValue)
			{
				controlValue = MIN_CONTROL_SIGNAL;
			}
			if (MAX_CONTROL_SIGNAL < (int32_t)controlValue)
			{
				controlValue = MAX_CONTROL_SIGNAL;
			}

			ADCDesiredAccelerationValue = ADC_value + controlValue;
		}
		else
		{
			Complete_Shutdown();
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
static inline void Set_Electromagnes_On(void)
{
	HAL_GPIO_WritePin(EN_ELECTRO_CLUTCH_GPIO_Port, EN_ELECTRO_CLUTCH_Pin, RESET);
}



/* This function shuts down completely cruise control */
void Complete_Shutdown(void)
{
	cruiseControlParam.state = OFF;
	Set_Electromagnes_Off();
	Set_Direction(NOTHING);
	PWM_PULSE = 0u;
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
		{
			HAL_GPIO_WritePin(INCREASE_ENG_TERMINAL_PORT, INCREASE_ENG_TERMINAL_PIN, RESET);
			HAL_GPIO_WritePin(DECREASE_ENG_TERMINAL_PORT, DECREASE_ENG_TERMINAL_PIN, RESET);
			break;
		}
		default:
		{
			HAL_GPIO_WritePin(INCREASE_ENG_TERMINAL_PORT, INCREASE_ENG_TERMINAL_PIN, RESET);
			HAL_GPIO_WritePin(DECREASE_ENG_TERMINAL_PORT, DECREASE_ENG_TERMINAL_PIN, RESET);
			break;
		}
	}
}



static void Encoder_Button_Function(void)
{
	if(TRUE == ENC_button_cruise.shortPressDetected)
	{
		if(ON == cruiseControlParam.state)
		{
			cruiseControlParam.state = OFF;
			Set_Electromagnes_Off();
		}
		else
		{
			cruiseControlParam.state = ON;
			Set_Electromagnes_On();
		}
	}
}






