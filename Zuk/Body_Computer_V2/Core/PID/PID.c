/*
 * PID.c
 *
 *  Created on: Nov 15, 2020
 *      Author: Wojciech Grzelinski
 */



#include "PID.h"



static inline float integrate(float error, PIDparameters_t *PIDparam);
static inline float differentiate(float error, PIDparameters_t *PIDparam);



/**
 * A function that calculates the control signal from PID controller.
 *
 * @param error: the controlled value error
 * @param *PIDparam: a pointer to the PID parameters struct
 * @retval float: calculated steering value
 */
float RunPIDController(float error, PIDparameters_t *PIDparam)
{
	static float i_res = 0.0;
	i_res += integrate(error, PIDparam);

	if (PIDparam->I_low_limit > i_res)
	{
		i_res = PIDparam->I_low_limit;
	}
	if(PIDparam->I_high_limit < i_res)
	{
		i_res = PIDparam->I_high_limit;
	}

	float d_res = differentiate(error, PIDparam);

	return ((PIDparam->P * error) + (((PIDparam->I) * i_res) / 1000) + ((PIDparam->D * d_res) * 1000));
}



/**
 * A function that integrates the error (I part of the PID).
 * Integration method: trapezoidal rule
 *
 * @param error: the controlled value error
 * @param *PIDparam: a pointer to the PID parameters struct
 * @retval float: the result of integration
 */
static inline float integrate(float error, PIDparameters_t *PIDparam)
{
	static float errorPrevious = 0;
	float y = (((errorPrevious - error) / 2) * PIDparam->dt);
	errorPrevious = error;

	return y;
}



/**
 * A function that differentiates the error (D part of the PID).
 *
 * @param error: the controlled value error
 * @param *PIDparam: a pointer to the PID parameters struct
 * @retval float: the result of differentiation
 */
static inline float differentiate(float error, PIDparameters_t *PIDparam)
{
	static float errorPrevious = 0;
	float y = ((error - errorPrevious) / PIDparam->dt);
	errorPrevious = error;

	return y;
}
