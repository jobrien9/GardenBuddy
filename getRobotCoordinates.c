/*
 * getRobotCoordinates.c
 *
 *  Created on: Mar 31, 2019
 *      Author: jj
 */

#include "main.h"
#include "stdbool.h"
#include "driverlib.h"
#include "incomingGPS.h"
#include "stdio.h"
#include "getRobotCoordinates.h"
#include "math.h"

int sampleCount = 0;
int longitudeDenom = 0;
int latitudeDenom = 0;
float averageLatitude =0 ;
float averageLongitude = 0;

//variable from main.c
extern bool isTraveling;
extern struct Coordinates robotLocation;

//interrupt that should be triggered every second to determine the robot's location
void robotInterrupt(){
    Timer_A_clearCaptureCompareInterrupt(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0);
    Timer_A_clearInterruptFlag(TIMER_A0_BASE);
    sampleCount++;
    //sample a couple times and average to make sure that the location is accurate
    if (sampleCount < 10){
        float* longitude = (float*)LONGITUDE_ADDRESS;
        float* latitude = (float*)LATITUDE_ADDRESS;
        if (*latitude != 0 && !isnan(*latitude)){
            latitudeDenom++;
            averageLatitude = averageLatitude+ *latitude;
        }
        if (*longitude !=0 && !isnan(*longitude)){
            longitudeDenom++;
            averageLongitude = averageLongitude+ *longitude;
        }
    }else{
        averageLatitude = averageLatitude/latitudeDenom;
        averageLongitude = averageLongitude/longitudeDenom;
        sampleCount = 0;
        latitudeDenom = 0;
        longitudeDenom = 0;
        //retry until we get a real latitude
        if (!isinf(averageLatitude) && !isinf(averageLongitude)){
            Timer_A_stopTimer(TIMER_A0_BASE);
            robotLocation.latitude = extractCoordinates(averageLatitude);
            robotLocation.longitude = extractCoordinates(averageLongitude);
            //uncomment this out when debugging
            //robotLocation.latitude = 33.777464;
            //robotLocation.longitude = -84.397508;
            printf("latitude %f \r\n", robotLocation.latitude);
            printf("longitude %f \r\n", robotLocation.longitude);
        }
    }
}

//gps module returns coordinates in format of DD.MMSS
float extractCoordinates(float coordinates){
    int degrees = (int) coordinates;
    int minutes = (int) ((coordinates - degrees) *100);
    float seconds = roundf((coordinates - degrees - (minutes/100.0)) * 100000)/10;

    return degrees + (minutes/60.0) + (seconds/6000.0);
}
