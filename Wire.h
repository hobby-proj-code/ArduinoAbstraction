/*
 * I2C.h
 *
 *  Created on: 30-Sep-2022
 *      Author: aditayagarg
 */

#ifndef ARDUINOABSTRACTIONS_WIRE_H_
#define ARDUINOABSTRACTIONS_WIRE_H_

#include "ArduinoCore-API/api/HardwareI2C.h"
//#include <ArduinoAbstractions/VendorTypes.h>

enum I2CPort
{
  I2C0Port,
  I2C1Port
};

#define I2C_BUFFER_SIZE 100

class I2CCommon: virtual public arduino::HardwareI2C
{
protected:
  I2CPort port;
  int timeout = 1000;
  bool resetOnTimeout = false;
public:
  I2CCommon(I2CPort port);
  virtual void end();
  void setWireTimeout(int timeout = 1000, bool reset_on_timeout = false);
  virtual void begin();

  virtual void setClock(uint32_t freq);
};

class I2CMasterWrite: virtual public I2CCommon
{
  uint8_t address = 0x00;
  uint8_t buffer[I2C_BUFFER_SIZE]{};
  int currPos = -1;
public:
  I2CMasterWrite(I2CPort port): I2CCommon(port){}
  virtual void beginTransmission(uint8_t address);
  virtual uint8_t endTransmission(bool stopBit);
  virtual uint8_t endTransmission(void);

  //print
  virtual size_t write(uint8_t);
};

class I2CMasterRead: virtual public I2CCommon
{
  uint8_t buffer[I2C_BUFFER_SIZE]{};
  int filledBuffer = 0;
  int currPos = -1;
public:
  I2CMasterRead(I2CPort port): I2CCommon(port){}

  virtual size_t requestFrom(uint8_t address, size_t len, bool stopBit);
  virtual size_t requestFrom(uint8_t address, size_t len);

  //stream
  virtual int available();
  virtual int read();
  virtual int peek();
};

class I2CSlave: virtual public I2CCommon
{
public:
  I2CSlave(I2CPort port): I2CCommon(port){}
  virtual void begin(uint8_t address);

  virtual void onReceive(void(*)(int));
  virtual void onRequest(void(*)(void));
};


class I2C: virtual public I2CMasterRead, virtual public I2CMasterWrite, virtual public I2CSlave
{
public:
    I2C(I2CPort port): I2CMasterRead(port), I2CMasterWrite(port), I2CSlave(port), I2CCommon(port) {}
    using I2CCommon::begin;
    using I2CCommon::setClock;
    using I2CCommon::end;
    using I2CCommon::setWireTimeout;
};

#endif /* ARDUINOABSTRACTIONS_WIRE_H_ */
