/*
 * constants.h
 *
 *  Created on: Mar 18, 2019
 *      Author: root
 */

#ifndef CONSTANTS_H_
#define CONSTANTS_H_

//these addresses found at  https://github.com/adafruit/Adafruit_LSM303/blob/master/Adafruit_LSM303.h
#define LSM303_ADDRESS_ACCEL          (0x32 >> 1)         // 0011001x
#define LSM303_ADDRESS_MAG            (0x3C >> 1)         // 0011110x

//the duty cycle to determine how fast the robot moves
#define DUTY_CYCLE_NORMAL 75
#define DUTY_CYCLE_FAST 100
#define DUTY_CYCLE_SLOW 50

//motor directions
#define FORWARD 1
#define STOP 0
#define BACKWARD -1

#define RIGHT_MOTOR_ID 1
#define LEFT_MOTOR_ID 2

//in meters
#define WHEEL_DIAMETER 0.052

#endif /* CONSTANTS_H_ */
