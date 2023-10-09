/*
 * Uart.cpp
 *
 *  Created on: 28-Jun-2022
 *      Author: aditayagarg
 */

#include <ArduinoAbstractions/UsbUart.h>
#include <ArduinoAbstractions/VendorCode.h>

void UsbUart::begin(unsigned long rate)
{
  setBaudRateUSB(rate);
}

void UsbUart::begin(unsigned long baudrate, uint16_t config)
{

}

void UsbUart::end()
{

}

int UsbUart::available(void)
{
  return 0;
}

int UsbUart::peek(void)
{
  if(peekChar < 0){peekChar = readChar();}
  return peekChar;
}

int UsbUart::read(void)
{
  if(peekChar >= 0)
  {
      int toRet = peekChar;
      peekChar = -1;
      return toRet;
  }
  return readChar();
}

void UsbUart::flush(void)
{
}

size_t UsbUart::write(uint8_t character)
{
  return printChar((char)character);
}

UsbUart::operator bool()
{
  return false;
}
