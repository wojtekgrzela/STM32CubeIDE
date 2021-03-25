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
#include "../../PID/PID.h"
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Defines And Typedefs */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#define P_REGULATOR_GAIN_SPEED			((float)(1.0))
#define I_REGULATOR_GAIN_SPEED			((float)(0.5))
#define D_REGULATOR_GAIN_SPEED			((float)(1.0))

#define P_REGULATOR_GAIN_RPM			((float)(1.0))
#define I_REGULATOR_GAIN_RPM			((float)(0.5))
#define D_REGULATOR_GAIN_RPM			((float)(1.0))

/* ADC defines */
#define MAX_ADC_ACCELERATION_READOUT	((uint32_t)(2000))	//TODO: max what an ADC can read as a max gas pedal position
#define MIN_ADC_ACCELERATION_READOUT	((uint32_t)(1000))	//TODO: min what an ADC can read as a min gas pedal position

/* RPM sefines */
#define MAX_ALLOWED_RPM				((uint32_t)(3500))
#define MIN_ALLOWED_RPM				((uint32_t)(1200))
#define SHUTDOWN_HIGH_RPM			((uint32_t)(3700))
#define ENCODER_RPM_SETTING_STEP	((uint32_t)(10))
#define IMPULSES_PER_ONE_ENGINE_REVOLUTION	((uint32_t)(12))



/* REGULATING constants */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Extern variables */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
extern CarStateinfo_type CarStateInfo;
extern volatile ENCButton_struct ENC_button_cruise;
extern volatile uint16_t ADC1Measures[NO_OF_ADC1_MEASURES];
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Local variables */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
volatile CruiseControlParameters_struct cruiseControlParam = {0};	/* NOT static on purpose */
volatile uint16_t ADCDesiredAccelerationValue = 0;	/* NOT static on purpose */

static PIDparameters_t PID_param_SPEED =
	{ 	.P = P_REGULATOR_GAIN_SPEED,
		.I = I_REGULATOR_GAIN_SPEED,
		.D = D_REGULATOR_GAIN_SPEED,
		.dt = MY_TASK_CRUISE_CONTROL_TIME_PERIOD };
static PIDparameters_t PID_param_RPM =
		{ 	.P = P_REGULATOR_GAIN_RPM,
			.I = I_REGULATOR_GAIN_RPM,
			.D = D_REGULATOR_GAIN_RPM,
			.dt = MY_TASK_CRUISE_CONTROL_TIME_PERIOD };
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Functions prototypes */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
extern void Complete_Shutdown(void);
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */



void StartCruiseCntrlTask(void const *argument)
{
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = MY_TASK_CRUISE_CONTROL_TIME_PERIOD;

	volatile float controlValue = 0.0;
	volatile int32_t ADCtempDesiredValue = 0;

	PIDparameters_t *PID_param_ptr = NULL;

	xLastWakeTime = xTaskGetTickCount();

	/* Infinite loop */
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
	for (;;)
	{
		if (ON == cruiseControlParam.state)
		{
			/* Check the mode of the cruise control */
			if(CONSTANT_SPEED == cruiseControlParam.mode)
			{
				/* If the mode is constant speed - use constant speed parameters and error */
				PID_param_ptr = &PID_param_SPEED;
				cruiseControlParam.error = cruiseControlParam.wantedSpeed - CarStateInfo.SPEED;
			}
			else
			{
				/* If the mode is constant RPM - use constant RPM parameters and error */
				PID_param_ptr = &PID_param_RPM;
				cruiseControlParam.error = cruiseControlParam.wantedRPM - CarStateInfo.RPM;
			}

			/* Save to the temporary variable current desired ADC value from accelerator position */
			ADCtempDesiredValue = ADCDesiredAccelerationValue;

			/* calculate the control value */
			controlValue = RunPIDController((float)cruiseControlParam.error, PID_param_ptr);

			/* Add the calculated value to the temporary variable with desired ADC value */
			ADCtempDesiredValue += (int32_t)controlValue;

			/* Check if the temporary desired value is within sensible limits and correct it if necessary */
			if (MIN_ADC_ACCELERATION_READOUT > ADCtempDesiredValue)
			{
				/* Set the minimum value when the desired was even lower */
				ADCtempDesiredValue = MIN_ADC_ACCELERATION_READOUT;
			}
			else if (MAX_ADC_ACCELERATION_READOUT < ADCtempDesiredValue)
			{
				/* Set the maximum value when the desired was even higher */
				ADCtempDesiredValue = MAX_ADC_ACCELERATION_READOUT;
			}

			/* Write the newly calculated temporary desired value to the exported desired value */
			ADCDesiredAccelerationValue = (uint16_t)ADCtempDesiredValue;
		}
		else
		{
			Complete_Shutdown();
		}


		vTaskDelayUntil(&xLastWakeTime, xFrequency);
	}
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
}





