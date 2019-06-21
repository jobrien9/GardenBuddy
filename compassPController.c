/*
 * compassPController.c
 *
 *  Created on: Apr 17, 2019
 *      Author: jj
 */

#include "main.h"
#include "robotDriver.h"
#include "constants.h"
#include "driverlib.h"

//robotDegrees from main.c
extern volatile float robotDegrees;
extern volatile int currentState; //from main.c
extern float originalRobotDegrees;
extern volatile struct Motor rightMotor;
extern volatile struct Motor leftMotor;

float seconds = 0;
float degreesPerSecond = 0;
float lastDegrees = 0;
float lastDegreesPerSecond = 0;

const float SECONDS_PER_INTERRUPT = 0.5;

const float KpCompass = 3;

//ISR to implement proportional control for when the robot turns
void compassProportion(){
    Timer_A_clearInterruptFlag(TIMER_A3_BASE);
    Timer_A_clearCaptureCompareInterrupt(TIMER_A3_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0);
    //every time this is called, add half a second
    seconds += SECONDS_PER_INTERRUPT;
    //on the first one, set the starting point
    if (lastDegrees == 0){
        lastDegrees = originalRobotDegrees;
    }
    degreesPerSecond = (robotDegrees - lastDegrees)/SECONDS_PER_INTERRUPT;
    float errorTerm = (rightMotor.setPointSpeed) - degreesPerSecond;

    float u = KpCompass * errorTerm;
    float dutyCycle = PWM_PERIOD * (u/100);

    if (dutyCycle > DUTY_CYCLE_FAST){
        dutyCycle = DUTY_CYCLE_FAST;
    }else if (dutyCycle < DUTY_CYCLE_SLOW){
        dutyCycle = DUTY_CYCLE_SLOW;
    }

    //keep the motors in sync
    rightMotor.dutyCycle = dutyCycle;
    leftMotor.dutyCycle = dutyCycle;

    lastDegreesPerSecond = degreesPerSecond;
}
