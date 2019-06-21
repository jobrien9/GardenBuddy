/*
 * main.h
 *
 *  Created on: Mar 18, 2019
 *      Author: root
 */

#include "stdbool.h"


#ifndef MAIN_H_
#define MAIN_H_

#define TIMER_A_PRIORITY 1
#define UART_PRIORITY 0
#define COMPASS_PRIORITY 0
#define PWM_PRIORITY 2
#define ENCODER_PRIORITY 5
#define COMPASS_PID_PRIORITY 4

#define SET_POINT_SPEED 0.25

//http://www.longitudestore.com/how-big-is-one-gps-degree.html
//roughly 111 km per degree of long/lat
#define METERS_PER_DEGREE 111000

#define MOTOR_TIMER_PERIOD 3 //100 kHz - Max Switching frequency of TB6612FNG is 100khz
#define ENCODER_TIMER_PERIOD 468 //100 Hz

// +/- this number of degrees means that we are pointing in the right direction
#define DEGREES_THRESHOLD 5

void GPSCommSetup();
void setClock(double frequency);
void timerSetup();
float findDistance();
void driverBoardInit();
void adcInit();
void arduinoCommSetup();
double directionCalculator();
bool isSufficientDegrees();
void disableMotors();

struct Coordinates{
    float longitude;
    float latitude;
};

struct Phone{
    struct Coordinates location;
    //number of degrees the robot will have to move to face the phone
    double degreesFromRobot;
    double distanceFromRobot;
};

struct Motor {
    int ID;
    int direction; //0 for neutral, 1 for forward, -1 for backward
    float dutyCycle; //duty cycle
    float setPointSpeed; //meters per second for drive; degrees per second for turn
    float metersTraveled;
    bool isCurrentlyMoving;
    bool directionChange; //to save on computation, set this to true if the direction needs to be toggled
};

#endif /* MAIN_H_ */
