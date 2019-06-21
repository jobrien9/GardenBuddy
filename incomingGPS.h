/*
 * incomingGPS.h
 *
 *  Created on: Mar 18, 2019
 *      Author: root
 */

#ifndef INCOMINGGPS_H_
#define INCOMINGGPS_H_

#define BUFFER_SIZE 1000
#define COORDINATES_SIZE 15
#define LINE_CHARACTER 13
#define DOLLAR_SIGN 36

//flash memory locations for coordinates
#define LONGITUDE_ADDRESS 0x0003F000
#define LONGITUDE_SECTOR FLASH_SECTOR31
#define LATITUDE_ADDRESS  0x0003E068
#define LATITUDE_SECTOR FLASH_SECTOR30

void GPSInterrupt();

void clearArray(char *arrayToClear);
void writeToFlash(float value, int addressToWrite, uint32_t sector);

#endif /* INCOMINGGPS_H_ */
