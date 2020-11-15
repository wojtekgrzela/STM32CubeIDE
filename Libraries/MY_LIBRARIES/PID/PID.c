/*
 * PID.c
 *
 *  Created on: Nov 15, 2020
 *      Author: Wojciech Grzelinski
 */


#include "PID.h"

static inline float integrate(float error);
static inline float differentiate(float error);


typedef struct PIDparameters_t
{
	float P;	// Proportional gain
	float I;	// Integrating gain
	float D;	// Differentiating gain

	float dt;	// sampling period in milliseconds
}PIDparameters_t;


static PIDparameters_t PIDparam = {1.0};



/**
 * A function that sets the parameters of PID controller.
 * Default values are: 1.0, 1.0, 1.0, 1.0;
 *
 * @param P: Proportional gain value
 * @param I: Integrating gain value
 * @param D: Differentiating gain value
 * @param dt: sampling period
 * @retval void
 */
void WritePIDParameters(float P, float I, float D, float dt/*in ms*/)
{
	PIDparam.P = P;
	PIDparam.I = I;
	PIDparam.D = D;
	PIDparam.dt = dt;
}



/**
 * A function that calculates the control signal from PID controller.
 *
 * @param error: the controlled value error
 * @retval float: calculated steering value
 */
float RunPIDController(float error)
{
	float i_res = integrate(error);
	float d_res = differentiate(error);

	return ( PIDparam.P * ( error + (((1 / PIDparam.I) * i_res) / 1000) + ((PIDparam.D * d_res) * 1000) ) );
}



/**
 * A function that integrates the error (I part of the PID).
 * Integration method: trapezoidal rule
 *
 * @param error: the controlled value error
 * @retval float: the result of integration
 */
static inline float integrate(float error)
{
	static float errorPrevious = 0;
	float y = (((errorPrevious - error) / 2) * PIDparam.dt);
	errorPrevious = error;

	return y;
}



/**
 * A function that differentiates the error (D part of the PID).
 *
 * @param error: the controlled value error
 * @retval float: the result of differentiation
 */
static inline float integrate(float error)
{
	static float errorPrevious = 0;
	float y = ((error - errorPrevious) / PIDparam.dt);
	errorPrevious = error;

	return y;
}
