/*
 * readEncoder.h
 *
 *  Created on: Apr 6, 2019
 *      Author: jj
 */

#include "main.h"

#ifndef READENCODER_H_
#define READENCODER_H_

#define RIGHT_ENCODER_MEM ADC_MEM0
#define LEFT_ENCODER_MEM ADC_MEM3

//in meters
#define QUARTER_CIRCUMFERENCE 0.04082

void PID_controller(struct Motor *motor);
void encoderISR();
float readADC(int memoryLocation);


#endif /* READENCODER_H_ */
