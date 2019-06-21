/*
 * arduinoReader.c
 *
 *  Created on: Apr 16, 2019
 *      Author: jj
 */

#include "driverlib.h"
//incomingGPS has some serial comm stuff we need
#include "incomingGPS.h"
#include "arduinoReader.h"
#include "main.h"
#include "stdlib.h"

char arduinoBuffer[100];
char incomingChar;
int arduinoBufferPointer = 0;

//location of the phone as declared on the main.c file
extern struct Coordinates phoneCoordinates;

//direction which the robot is facing in degrees
extern float robotDegrees;


//this ISR reads from the Arduino serially
void readArduino(){
    UART_clearInterruptFlag(EUSCI_A3_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG);

  //read character
  incomingChar = UART_receiveData(EUSCI_A3_BASE);

  //compass degrees come first, then waypoint coordinates
  //syntax <DEGREES>,LAT$LONG\r\n
  if (incomingChar == COMMA){
      float degreeMeasurement = atof(arduinoBuffer);

      //some incoming degrees were garbled - do this to avoid any junk data
      if(degreeMeasurement < -1 || degreeMeasurement > 1){
          robotDegrees = degreeMeasurement;
      }else{
          robotDegrees = 5; //have it point north if this happens
      }

      clearArray(arduinoBuffer);
      arduinoBufferPointer = 0;
  }else if (incomingChar == DOLLAR_SIGN){ //this is a weird syntax, but it works
      phoneCoordinates.latitude = atof(arduinoBuffer);
      clearArray(arduinoBuffer);
      arduinoBufferPointer = 0;
  }else if (incomingChar == LINE_CHARACTER){
      phoneCoordinates.longitude = atof(arduinoBuffer);
      clearArray(arduinoBuffer);
      arduinoBufferPointer=0;
  }else{
      arduinoBuffer[arduinoBufferPointer++] = incomingChar;
  }
}
