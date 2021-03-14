/*
 * PID.h
 *
 *  Created on: Nov 15, 2020
 *      Author: Wojciech Grzelinski
 */

#ifndef INC_PID_H_
#define INC_PID_H_


void WritePIDParameters(float P, float I, float D, float dt);
float RunPIDController(float error);


#endif /* INC_PID_H_ */
