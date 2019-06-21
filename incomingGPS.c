/*
 * incomingGPS.c
 *
 *  Created on: Mar 18, 2019
 *      Author: jobrien
 */

//Found GPS Info here: https://www.gpsinformation.org/dale/nmea.htm

#include "driverlib.h"
#include "uart.h"
#include "incomingGPS.h"
#include "stdio.h"
#include "stdlib.h"

//GPS - 3.3V input, GND, Port3, Pin2

//todo: look into moving these out of global variables
char listenBuffer[BUFFER_SIZE];
char latitudeBuffer[COORDINATES_SIZE];
char longitudeBuffer[COORDINATES_SIZE];
float latitude_incoming;
float longitude_incoming;
int bufferCursor = 0;
char incomingGPSChar;

void GPSInterrupt(){
    UART_clearInterruptFlag(EUSCI_A2_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG);

    //read character
    incomingGPSChar = (char) UART_receiveData(EUSCI_A2_BASE);


    //each GPS sentence begins with a dollar sign - look for GPGGA
    if (incomingGPSChar == DOLLAR_SIGN || bufferCursor >= BUFFER_SIZE){
        //count how many characters we are into the GPS sentence
        //todo: make this more elegant
         if (listenBuffer[2] == 'G' && listenBuffer[3] == 'G' && listenBuffer[4] == 'A'){
             //loop through CSV to find long and lat
             int i =0;
             int commaCount = 0;
             //use the value buffer to build the current value (i.e. latitude or longitude)
             int valueBuffer = 0;
             char currentCharacter;
             for(i = 0; i < bufferCursor; i++){
                 currentCharacter = listenBuffer[i];
                 //comma 2 = Latitude
                 if (currentCharacter == ','){
                      commaCount++;
                      //reset back to zero
                      valueBuffer = 0;
                 } else if (commaCount == 2){
                     latitudeBuffer[valueBuffer] = currentCharacter;
                     valueBuffer++;
                 }else if (commaCount == 3){ //this CSV value shows N/S
                     //return the latitude as a float and divide by 100 because of the way the GPS module stores data
                     latitude_incoming = atof(latitudeBuffer)/100;
                     if (currentCharacter == 'S'){
                        latitude_incoming = latitude_incoming * -1;
                     }
                 }else if (commaCount == 4){
                     longitudeBuffer[valueBuffer] = currentCharacter;
                     valueBuffer++;
                 }else if (commaCount ==5){
                     longitude_incoming = atof(longitudeBuffer)/100;
                     if (currentCharacter == 'W'){
                         longitude_incoming = longitude_incoming * -1;
                     }
                 }
             }
         }
       //save to Flash
         if (longitude_incoming != 0){
             writeToFlash(longitude_incoming, LONGITUDE_ADDRESS, LONGITUDE_SECTOR);
         }

         if (latitude_incoming != 0){
             writeToFlash(latitude_incoming, LATITUDE_ADDRESS, LATITUDE_SECTOR);
         }
       //reset Buffer
       clearArray(listenBuffer);
       bufferCursor = 0;
    }else{
        listenBuffer[bufferCursor++] = incomingGPSChar;
    }
}


//helper function to clear an array
void clearArray(char *arrayToClear){
    int i;
    for (i = 0; i < sizeof(arrayToClear); i++){
        arrayToClear[i] = '\0';
    }
}

//manages writing the float array to flash memory
void writeToFlash(float value, int addressToWrite, uint32_t sector){
    //unlock sector
    if (!FlashCtl_unprotectSector(FLASH_MAIN_MEMORY_SPACE_BANK1, sector)){
        printf("Sector was not able to be unprotected!\r\n");
    }
    //erase sector
    if (!FlashCtl_eraseSector(addressToWrite)){
        printf("Sector erase failed!\r\n");
    }
    //write each float to memory
    if (!FlashCtl_programMemory(&value, (void*) addressToWrite, sizeof(value))){
        printf("Write failed!\r\n");
    }

    //lock sector
    if(!FlashCtl_protectSector(FLASH_MAIN_MEMORY_SPACE_BANK1, sector)){
        printf("Protect failed!\r\n");
    }
}
