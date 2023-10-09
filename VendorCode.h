/*
 * VendorCode.h
 *
 *  Created on: 06-Jul-2022
 *      Author: aditayagarg
 */

#ifndef ARDUINOABSTRACTIONS_VENDORCODE_H_
#define ARDUINOABSTRACTIONS_VENDORCODE_H_

#include <stdint.h>

bool vendorInit();
bool setBaudRateUSB(int baudRate);
int printChar(char);
int readChar();

//////////I2C
///

bool deInitI2C(int port);
bool initI2C(int port);
bool setI2CFrequency(int port, uint32_t frequency);
int i2cRecvBuffer(int port, uint8_t address, uint8_t* buffer,int buffLen, int timeout = 1000);
int i2cTrasBuffer(int port, uint8_t address, uint8_t* buffer,int buffLen, int timeout = 1000);

/////////////////////////

//////////UART
///

bool deInitUART(int port);
bool initUART(int port, uint8_t parity, uint8_t dataBitLength, uint8_t stopBitLength, int baudRate);
int uartRecvBuffer(int port, uint8_t* buffer,int buffLen, int timeout = 1000);
int uartTrasBuffer(int port, uint8_t* buffer,int buffLen, int timeout = 1000);

/////////////////////////

#endif /* ARDUINOABSTRACTIONS_VENDORCODE_H_ */
