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
#define P_REGULATOR_GAIN_SPEED_1			((float)(3.0))
#define I_REGULATOR_GAIN_SPEED_1			((float)(0.001))
#define D_REGULATOR_GAIN_SPEED_1			((float)(2.0))

#define P_REGULATOR_GAIN_SPEED_2			((float)(3.0))
#define I_REGULATOR_GAIN_SPEED_2			((float)(0.001))
#define D_REGULATOR_GAIN_SPEED_2			((float)(2.0))

#define P_REGULATOR_GAIN_SPEED_3			((float)(4.0))
#define I_REGULATOR_GAIN_SPEED_3			((float)(0.0001))
#define D_REGULATOR_GAIN_SPEED_3			((float)(2.0))

#define P_REGULATOR_GAIN_SPEED_4			((float)(4.0))
#define I_REGULATOR_GAIN_SPEED_4			((float)(0.5))
#define D_REGULATOR_GAIN_SPEED_4			((float)(6.0))

#define SPEED_THRESHOLD_1			((float)(100.0))
#define SPEED_THRESHOLD_2			((float)(85.0))
#define SPEED_THRESHOLD_3			((float)(70.0))
#define SPEED_THRESHOLD_4			(MIN_ALLOWED_SPEED)

#define P_REGULATOR_GAIN_RPM			((float)(1.0))
#define I_REGULATOR_GAIN_RPM			((float)(0.5))
#define D_REGULATOR_GAIN_RPM			((float)(1.0))

/* ADC defines */
#define MAX_ADC_ACCELERATION_READOUT	((int32_t)(2900))	//TODO: max what an ADC can read as a max gas pedal position
#define MIN_ADC_ACCELERATION_READOUT	((int32_t)(540))	//TODO: min what an ADC can read as a min gas pedal position



/* REGULATING constants */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Extern variables */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
extern CarStateinfo_type CarStateInfo;
extern volatile ENCButton_struct ENC_button_cruise;
extern volatile uint16_t ADC1Measures[NO_OF_ADC1_MEASURES];
extern volatile int8_t EncoderCounterCruiseDiff;
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Local variables */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
volatile CruiseControlParameters_struct cruiseControlParam = {0};	/* NOT static on purpose */
volatile uint16_t ADCDesiredAccelerationValue = 1400;	/* NOT static on purpose */

static PIDparameters_t PID_param_SPEED =
	{ 	.P = P_REGULATOR_GAIN_SPEED_1,
		.I = I_REGULATOR_GAIN_SPEED_1,
		.D = D_REGULATOR_GAIN_SPEED_1,
		.dt = MY_TASK_CRUISE_CONTROL_TIME_PERIOD };
static PIDparameters_t PID_param_RPM =
		{ 	.P = P_REGULATOR_GAIN_RPM,
			.I = I_REGULATOR_GAIN_RPM,
			.D = D_REGULATOR_GAIN_RPM,
			.dt = MY_TASK_CRUISE_CONTROL_TIME_PERIOD };

LCD_message Wanted_RPMForLCD =
	{ 	.messageHandler = NULL,
		.size = 0,
		.messageReadyFLAG = 0 };
LCD_message Wanted_SPEEDForLCD =
	{ 	.messageHandler = NULL,
		.size = 0,
		.messageReadyFLAG = 0 };
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Functions prototypes */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
static void Encoder_CruiseControl_Counter(void);
static void valuesToStrings(void);

extern void Complete_Shutdown(void);
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

float CALCULATEDPosition = 0.0;
float globalCntlValue;
float globalTempDesired;

void StartCruiseCntrlTask(void const *argument)
{
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = MY_TASK_CRUISE_CONTROL_TIME_PERIOD;

	volatile float controlValue = 0.0;
	volatile int32_t ADCtempDesiredValue = 0;

	PIDparameters_t *PID_param_ptr = NULL;

	static uint8_t Wanted_RPMMessage[5]	= "";
	static uint8_t Wanted_SPEEDMessage[4] = "";
	Wanted_RPMForLCD.messageHandler = Wanted_RPMMessage;
	Wanted_SPEEDForLCD.messageHandler = Wanted_SPEEDMessage;

	cruiseControlParam.wantedRPM = MIN_ALLOWED_RPM;
	cruiseControlParam.wantedSpeed = MIN_ALLOWED_SPEED;

	xLastWakeTime = xTaskGetTickCount();

	/* Infinite loop */
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
	for (;;)
	{
		/* Update the setting of SPEED or RPM with encoder difference value */
		Encoder_CruiseControl_Counter();
		/* Translate values to strings for displaying */
		valuesToStrings();

		if (ON == cruiseControlParam.state)
		{
			HAL_GPIO_WritePin(POWER_ON_CRUISE_CONTROL_PORT, POWER_ON_CRUISE_CONTROL_Pin, SET);
			/* Check the mode of the cruise control */
			if(CONSTANT_SPEED == cruiseControlParam.mode)
			{
				if (cruiseControlParam.wantedSpeed >= SPEED_THRESHOLD_1)
				{
					PID_param_SPEED.P = P_REGULATOR_GAIN_SPEED_1;
					PID_param_SPEED.I = I_REGULATOR_GAIN_SPEED_1;
					PID_param_SPEED.D = D_REGULATOR_GAIN_SPEED_1;
				}
				else if(cruiseControlParam.wantedSpeed >= SPEED_THRESHOLD_2)
				{
					PID_param_SPEED.P = P_REGULATOR_GAIN_SPEED_2;
					PID_param_SPEED.I = I_REGULATOR_GAIN_SPEED_2;
					PID_param_SPEED.D = D_REGULATOR_GAIN_SPEED_2;
				}
				else if(cruiseControlParam.wantedSpeed >= SPEED_THRESHOLD_3)
				{
					PID_param_SPEED.P = P_REGULATOR_GAIN_SPEED_3;
					PID_param_SPEED.I = I_REGULATOR_GAIN_SPEED_3;
					PID_param_SPEED.D = D_REGULATOR_GAIN_SPEED_3;
				}
				else
				{
					PID_param_SPEED.P = P_REGULATOR_GAIN_SPEED_4;
					PID_param_SPEED.I = I_REGULATOR_GAIN_SPEED_4;
					PID_param_SPEED.D = D_REGULATOR_GAIN_SPEED_4;
				}
				/* If the mode is constant speed - use constant speed parameters and error */
				PID_param_ptr = &PID_param_SPEED;
				cruiseControlParam.error = (float)cruiseControlParam.wantedSpeed - (float)CarStateInfo.SPEED;
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
			globalCntlValue = controlValue;
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
			globalTempDesired = ADCtempDesiredValue;
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



/* This function checks if there was an action with encoder for cruise control and updates the wanted RPM or SPEED accordingly */
static void Encoder_CruiseControl_Counter(void)
{
	int32_t tempWantedRPM = cruiseControlParam.wantedRPM;
	int32_t tempWantedSpeed = cruiseControlParam.wantedSpeed;

	if(0 != EncoderCounterCruiseDiff)
	{
		if(CONSTANT_RPM == cruiseControlParam.mode)
		{
			/* Calculating wanted RPM to a temporary variable */
			tempWantedRPM += EncoderCounterCruiseDiff * ENCODER_RPM_SETTING_STEP; /* Changing by "20" RPM */

			/* Checking if the variable is within some limits (MIN and MAX values) */
			if(MIN_ALLOWED_RPM > tempWantedRPM)
			{
				tempWantedRPM = MIN_ALLOWED_RPM;
			}
			else if(MAX_ALLOWED_RPM < tempWantedRPM)
			{
				tempWantedRPM = MAX_ALLOWED_RPM;
			}

			/* Writing the temporary value to the global one after check */
			cruiseControlParam.wantedRPM = (uint32_t)tempWantedRPM;
		}
		else
		{
			/* Calculating wanted speed to a temporary variable */
			tempWantedSpeed += EncoderCounterCruiseDiff * ENCODER_SPEED_SETTING_STEP; /* Changing by "1" km/h */

			/* Checking if the variable is within some limits (MIN and MAX values) */
			if(MIN_ALLOWED_SPEED > tempWantedSpeed)
			{
				tempWantedSpeed = MIN_ALLOWED_SPEED;
			}
			else if(MAX_ALLOWED_SPEED < tempWantedSpeed)
			{
				tempWantedSpeed = MAX_ALLOWED_SPEED;
			}

			/* Writing the temporary value to the global one after check */
			cruiseControlParam.wantedSpeed = (uint32_t)tempWantedSpeed;
		}
		/* Clear the difference value after usage */
		EncoderCounterCruiseDiff = 0;
	}
}



/* This function is to have the wanted parameters (speed and RPM) translated to strings for displaying */
static void valuesToStrings(void)
{
	/* Engine Wanted RPM */
	Wanted_RPMForLCD.messageReadyFLAG = FALSE;
	snprintf((char*)Wanted_RPMForLCD.messageHandler, 5, "%01" PRIu32, cruiseControlParam.wantedRPM);
	Wanted_RPMForLCD.size = strlen((char*)Wanted_RPMForLCD.messageHandler);
	Wanted_RPMForLCD.messageReadyFLAG = TRUE;
	/* Vehicle Wanted Speed */
	Wanted_SPEEDForLCD.messageReadyFLAG = FALSE;
	snprintf((char*)Wanted_SPEEDForLCD.messageHandler, 4, "%01" PRIu32, cruiseControlParam.wantedSpeed);
	Wanted_SPEEDForLCD.size = strlen((char*)Wanted_SPEEDForLCD.messageHandler);
	Wanted_SPEEDForLCD.messageReadyFLAG = TRUE;
}
