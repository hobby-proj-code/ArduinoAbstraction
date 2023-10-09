/*
 * Uart.cpp
 *
 *  Created on: 10-Mar-2023
 *      Author: aditayagarg
 */


#include "Uart.h"
#include "VendorCode.h"

Uart::Uart(UartPort portParam): port(portParam)
{

}

void Uart::begin(unsigned long rate)
{
  baudrate = rate;
  status = initUART(port, 0, 8, 1, baudrate);
}

void Uart::begin(unsigned long baudrate, uint16_t config)
{
  Uart::baudrate = baudrate;
  Uart::config = config;
  status = initUART(port, 0, 8, 1, baudrate);
}

void Uart::end()
{
  deInitUART(port);
}

int Uart::available(void)
{
  if(filledBufferRead - currPosRead <= 0){fillReadBuffer();}
  return filledBufferRead - currPosRead > 0 ? filledBufferRead - currPosRead : 0;
}

int Uart::peek(void)
{
  if(filledBufferRead - currPosRead <= 0){fillReadBuffer();}
  if(currPosRead < filledBufferRead && currPosRead >= 0)
  {
      return bufferRead[currPosRead];
  }
  return -1;
}

int Uart::read(void)
{
  if(filledBufferRead - currPosRead <= 0){fillReadBuffer();}
  if(currPosRead < filledBufferRead && currPosRead >= 0)
  {
      return bufferRead[currPosRead++];
  }
  return -1;
}

bool Uart::fillReadBuffer()
{
  if(filledBufferRead - currPosRead > 0)
  {
    return true;
  }
  else
  {
      filledBufferRead = uartRecvBuffer(port, bufferRead, UART_BUFFER_SIZE, 100);
      currPosRead = 0;
  }
}

void Uart::flush(void)
{
}

size_t Uart::write(uint8_t character)
{
  return uartTrasBuffer(port, &character, 1);
}

Uart::operator bool()
{
  return status;
}
