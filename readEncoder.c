/*
 * readEncoder.c
 *
 *  Created on: Apr 6, 2019
 *      Author: jj
 */

#include "readEncoder.h"
#include "math.h"
#include "main.h"
#include "constants.h"
#include "driverlib.h"
#include "stdio.h"
#include "robotDriver.h"

extern struct Motor leftMotor;
extern struct Motor rightMotor;

int rightEncoderCount = 0;
float previousRight = 0;
int leftEncoderCount = 0;
float previousLeft = 0;

float rightEncoderValue;
float leftEncoderValue;
float controllerOutput;
float errorTerm;
float u;


bool isFirstCycle = true;

//meters per second
float secondsElapsed = 0;
//this is used to keep track of the time that's passed for the proportional controller
float previousRightSecondsElapsed = 0;
float previousRightMetersTraveled = 0;
float previousLeftSecondsElapsed = 0;
float previousLeftMetersTraveled = 0;
float Kp = 10;


//test arrays
int speedArray[1000];
float outputArray[1000];
int bufferPointer = 0;

void encoderISR(){
    Timer_A_clearInterruptFlag(TIMER_A2_BASE);
    Timer_A_clearCaptureCompareInterrupt(TIMER_A2_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0);
    isFirstCycle = false;
    secondsElapsed += 0.01;//100hz each time
    //read from the adc
    ADC14_toggleConversionTrigger();
    rightEncoderValue = readADC(RIGHT_ENCODER_MEM);
    leftEncoderValue = readADC(LEFT_ENCODER_MEM);

    //detect the rising edge of the encoder to detect a 1/4 turn
    //skip the first time this happens to avoid noise associated with startup
    if (!isFirstCycle){
        if(rightEncoderValue - previousRight > 200){
            PID_controller(&rightMotor);
        }

        if (leftEncoderValue - previousLeft > 200){
            PID_controller(&leftMotor);
        }
    }

    //track the last encoder value to figure out when the rising edge occurs
    previousRight = rightEncoderValue;
    previousLeft = leftEncoderValue;
}

void PID_controller(struct Motor *motor){
    //the encoder returns 4 times per rotation, so add 1/4 of the wheel's circumference
    motor->metersTraveled += QUARTER_CIRCUMFERENCE;
    //if secondsElapsed is 0 for some reason, it's a problem
    if(secondsElapsed == 0 ){
        return;
    }
    if (motor->ID == LEFT_MOTOR_ID){
        controllerOutput = (motor->metersTraveled - previousRightMetersTraveled)/(secondsElapsed -previousRightSecondsElapsed);
    }else{
        controllerOutput = (motor->metersTraveled - previousLeftMetersTraveled)/(secondsElapsed -previousLeftSecondsElapsed);
    }
    //find the error term
    errorTerm = motor->setPointSpeed - controllerOutput;
    //multiply by Kp
    u = Kp * errorTerm;

    //create the new PWM cycle
    motor->dutyCycle += PWM_PERIOD * (u/100);
    //uncomment this to debug Proportional controller
    //speedArray[bufferPointer] = motor->dutyCycle;
    //outputArray[bufferPointer++] = controllerOutput;
    //don't let the duty cycle exceed 100%, and keep it above 25% to avoid it from dropping ridiculously slow
    if (motor->dutyCycle > DUTY_CYCLE_FAST){
        motor->dutyCycle = DUTY_CYCLE_FAST;
    }else if (motor->dutyCycle < DUTY_CYCLE_SLOW){
        motor->dutyCycle = DUTY_CYCLE_SLOW;
    }
    if (motor->ID == LEFT_MOTOR_ID){
        //at each break, keep track of the previous seconds count
        previousLeftSecondsElapsed = secondsElapsed;
        previousLeftMetersTraveled = motor->metersTraveled;
    }else{
        previousRightSecondsElapsed = secondsElapsed;
        previousRightMetersTraveled = motor->metersTraveled;
    }
}

//read from the ADC
float readADC(int memoryLocation){
    if (ADC14_isBusy()){
        readADC(memoryLocation);
    }else{
        return (float) ADC14_getResult(memoryLocation);
    }
    return;
}
