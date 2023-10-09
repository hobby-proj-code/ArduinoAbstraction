/*
 * Uart.h
 *
 *  Created on: 28-Jun-2022
 *      Author: aditayagarg
 */

#ifndef ARDUINOABSTRACTIONS_USBUART_H_
#define ARDUINOABSTRACTIONS_USBUART_H_

#include <ArduinoAbstractions/ArduinoCore-API/api/HardwareSerial.h>
//#include <ArduinoAbstractions/VendorTypes.h>

class UsbUart: public arduino::HardwareSerial
{
  int peekChar = -1;
public:
  virtual void begin(unsigned long);
  virtual void begin(unsigned long baudrate, uint16_t config);
  virtual void end();
  virtual int available(void);
  virtual int peek(void);
  virtual int read(void);
  virtual void flush(void);
  virtual size_t write(uint8_t);
  virtual operator bool();
};

#endif /* ARDUINOABSTRACTIONS_USBUART_H_ */
