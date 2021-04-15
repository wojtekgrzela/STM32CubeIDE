/*
 * PID.h
 *
 *  Created on: Nov 15, 2020
 *      Author: Wojciech Grzelinski
 */

#ifndef INC_PID_H_
#define INC_PID_H_



typedef struct PIDparameters_t
{
	float P;	// Proportional gain
	float I;	// Integrating gain
	float D;	// Differentiating gain

	float dt;	// sampling period in milliseconds
}PIDparameters_t;



float RunPIDController(float error, PIDparameters_t *PIDparam);



#endif /* INC_PID_H_ */
