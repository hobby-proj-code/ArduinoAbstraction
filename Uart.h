/*
 * Uart.h
 *
 *  Created on: 10-Mar-2023
 *      Author: aditayagarg
 */

#ifndef ARDUINOABSTRACTIONS_UART_H_
#define ARDUINOABSTRACTIONS_UART_H_


#include "ArduinoCore-API/api/HardwareSerial.h"
//#include <ArduinoAbstractions/VendorTypes.h>
#define UART_BUFFER_SIZE 100
enum UartPort
{
  Uart0Port, //DebugPort
  Uart1Port //ExternalPort
};

class Uart: public arduino::HardwareSerial
{
  UartPort port;
  unsigned long baudrate = 115200;
  uint16_t config = SERIAL_8N1;
  bool status = false;

  uint8_t bufferRead[UART_BUFFER_SIZE]{};
  int filledBufferRead = 0;
  int currPosRead = -1;
  bool fillReadBuffer();
public:
  Uart(UartPort portParm);
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


#endif /* ARDUINOABSTRACTIONS_UART_H_ */
