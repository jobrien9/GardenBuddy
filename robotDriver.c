/*
 * robotDriver.c
 *
 *  Created on: Apr 6, 2019
 *      Author: jj
 */
#include "driverlib.h"
#include "main.h"
#include "robotDriver.h"
#include "stdbool.h"
#include "constants.h"

//count interrupts per second
int interruptCount = 0;

extern struct Motor leftMotor;
extern struct Motor rightMotor;

//ISR for the PWM Timer
void pwmInterrupt(){
    Timer_A_clearInterruptFlag(TIMER_A1_BASE);
    Timer_A_clearCaptureCompareInterrupt(TIMER_A1_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0);

    //reset the interrupt count every PWM period
    if (interruptCount++ >= PWM_PERIOD){
        interruptCount = 0; //reset back to 0
    }else {
        //set direction
        if (rightMotor.directionChange){
            if (rightMotor.direction == FORWARD){
                rightMotorForward();
            }else if(rightMotor.direction == BACKWARD){
                rightMotorBackward();
            }
            rightMotor.directionChange = false;
        }

        if (leftMotor.directionChange){
            if (leftMotor.direction == FORWARD){
                leftMotorForward();
            }else if(leftMotor.direction == BACKWARD){
                leftMotorBackward();
            }
            leftMotor.directionChange = false;
        }


        //speed should be the duty cycle
        if (interruptCount <= rightMotor.dutyCycle){
            //check for direction and move
            actuateMotor(&rightMotor);
            rightMotor.isCurrentlyMoving = true;
            //rightMotorPWM();
        }else if (rightMotor.isCurrentlyMoving){
            rightMotorLow();
            rightMotor.isCurrentlyMoving = false;
            //rightMotor.directionChange = true;
        }
        if (interruptCount <= leftMotor.dutyCycle){
            actuateMotor(&leftMotor);
            leftMotor.isCurrentlyMoving = true;
        }else if (leftMotor.isCurrentlyMoving){
            leftMotorLow();
            leftMotor.isCurrentlyMoving= false;
        }
    }
}

//decide whether or not to move the motor and in which direction
void actuateMotor(struct Motor *motor){
    //check if the motor is currently moving or not to avoid constantly turning the GPIO pins on when not needed
    if (motor->direction != STOP){
            if (!motor->isCurrentlyMoving){
                //determine which motor should move
                if(motor->ID == LEFT_MOTOR_ID){
                    leftMotorPWM();
                }else{
                    rightMotorPWM();
                }
                motor->isCurrentlyMoving = true;
            }
    } else {
        if (motor->ID == LEFT_MOTOR_ID) {
            leftMotorLow();
        } else{
            rightMotorLow();
        }
        motor->isCurrentlyMoving = false;
    }
}

void rightMotorLow(){
    GPIO_setOutputLowOnPin(GPIO_PORT_P6, GPIO_PIN7);
    //GPIO_setOutputLowOnPin(GPIO_PORT_P6, GPIO_PIN6);
    //GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN6);
}

void leftMotorLow(){
    GPIO_setOutputLowOnPin(GPIO_PORT_P6, GPIO_PIN4);
}

void rightMotorForward(){
    GPIO_setOutputLowOnPin(GPIO_PORT_P6, GPIO_PIN6);
    GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN6);
}

void rightMotorBackward(){
    GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN6);
    GPIO_setOutputHighOnPin(GPIO_PORT_P6, GPIO_PIN6);
}

//set the GPIO pin high
void rightMotorPWM(){
    GPIO_setOutputHighOnPin(GPIO_PORT_P6, GPIO_PIN7);
}

//set the GPIO pin high
void leftMotorPWM(){
    GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN3);
}

void leftMotorForward(){
    GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN4);
    GPIO_setOutputHighOnPin(GPIO_PORT_P6, GPIO_PIN4);
}

void leftMotorBackward(){
    GPIO_setOutputLowOnPin(GPIO_PORT_P6, GPIO_PIN4);
    GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN4);
}
