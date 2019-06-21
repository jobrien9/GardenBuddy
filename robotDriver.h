/*
 * robotDriver.h
 *
 *  Created on: Apr 6, 2019
 *      Author: jj
 */

#ifndef ROBOTDRIVER_H_
#define ROBOTDRIVER_H_

#define PWM_PERIOD 100

void pwmInterrupt();

void rightMotorLow();

void leftMotorLow();

void rightMotorForward();

void rightMotorBackward();

//set the GPIO pin high
void rightMotorPWM();

//set the GPIO pin high
void leftMotorPWM();

void leftMotorForward();

void leftMotorBackward();

void actuateMotor(struct Motor *motor);

#endif /* ROBOTDRIVER_H_ */
