/*
 * I2C.cpp
 *
 *  Created on: 30-Sep-2022
 *      Author: aditayagarg
 */

#include <ArduinoAbstractions/VendorCode.h>
#include <Wire.h>

I2CCommon::I2CCommon(I2CPort port)
{
  I2CCommon::port = port;
}

void I2CCommon::end()
{
  deInitI2C(port);
}

void I2CCommon::setWireTimeout(int timeout, bool reset_on_timeout)
{
  I2CCommon::timeout = timeout;
  I2CCommon::resetOnTimeout = reset_on_timeout;
}

void I2CCommon::setClock(uint32_t freq)
{
  setI2CFrequency(port,freq);
}


void I2CCommon::begin()
{
  initI2C(port);
}

int I2CMasterRead::available(void)
{
  return filledBuffer - currPos;
}

int I2CMasterRead::peek(void)
{
  if(currPos < filledBuffer && currPos >= 0)
  {
      return buffer[currPos];
  }
  return -1;
}

int I2CMasterRead::read(void)
{
  if(currPos < filledBuffer && currPos >= 0)
  {
      return buffer[currPos++];
  }
  return -1;
}

size_t I2CMasterWrite::write(uint8_t character)
{
  if(++currPos >= I2C_BUFFER_SIZE)
  {
      return 0;
  }
  buffer[currPos] = character;
  return 1;
}

void I2CMasterWrite::beginTransmission(uint8_t address)
{
  I2CMasterWrite::address = address;
  currPos = -1;
}

uint8_t I2CMasterWrite::endTransmission(bool stopBit)
{
  int ret = 4; //oter error
  if(currPos >= I2C_BUFFER_SIZE ){ret = 1;} //data too long to fit into buffer
  else
  {
      if(i2cTrasBuffer(port, address, buffer, currPos + 1, timeout) == (currPos+1))
      {
          ret = 0;
      }
  }
  I2CMasterWrite::address = 0x00;
  currPos = -1;
  return ret;
}

uint8_t I2CMasterWrite::endTransmission(void)
{
  return endTransmission(false);
}

size_t I2CMasterRead::requestFrom(uint8_t address, size_t len, bool stopBit)
{
  filledBuffer = len > I2C_BUFFER_SIZE ? I2C_BUFFER_SIZE : len;
  memset(buffer,0,filledBuffer);
  filledBuffer = i2cRecvBuffer(port, address, buffer, filledBuffer, timeout);
  if(filledBuffer > 0){currPos = 0;}
  return filledBuffer;
}

size_t I2CMasterRead::requestFrom(uint8_t address, size_t len)
{
  return requestFrom(address,len,true);
}

void I2CSlave::onReceive(void(*) (int))
{
}

void I2CSlave::onRequest(void(*) (void))
{
}

void I2CSlave::begin(uint8_t address)
{
  I2CCommon::begin();
}



