/*
 * Arduino.h
 *
 *  Created on: 20-Jun-2022
 *      Author: aditayagarg
 */

#ifndef ARDUINOABSTRACTIONS_ARDUINO_H_
#define ARDUINOABSTRACTIONS_ARDUINO_H_


#include "EFR32Variant.h"
#include "ArduinoMain.h"
#include "ArduinoCore-API/api/Common.h"
#include "ArduinoCore-API/api/itoa.h"
#include "ArduinoCore-API/api/HardwareSerial.h"
#include "StandarLibrary.h"

typedef uint8_t byte;
#define ARDUINO 100

typedef void (*InterruptCallback)(void);

/*
void delay(uint32_t);
unsigned long millis();
unsigned long micros();

void pinMode(int pinNumber, PinMode pinMode);
void digitalWrite(int pinNumber, PinStatus status);
PinStatus digitalRead(int pinNumber);

void attachInterrupt(int,InterruptCallback,PinStatus);
void detachInterrupt(int);
*/
int digitalPinToInterrupt(int);



#endif /* ARDUINOABSTRACTIONS_ARDUINO_H_ */
