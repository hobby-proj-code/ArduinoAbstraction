/*
 * SPI.h
 *
 *  Created on: 18-Nov-2022
 *      Author: aditayagarg
 */

#ifndef ARDUINOABSTRACTIONS_SPI_H_
#define ARDUINOABSTRACTIONS_SPI_H_

#include <ArduinoAbstractions/ArduinoCore-API/api/HardwareSPI.h>

enum SPIPort
{
  SPI0Port,
  SPI1Port
};

typedef arduino::SPISettings SPISettings;


class SPINonInterrupt: virtual public arduino::HardwareSPI
{
  protected:
  SPIPort port;
  SPISettings settings;
  public:
  SPINonInterrupt(SPIPort portParm): port(portParm){}

  virtual void begin(); //SPI vendor INIT
  virtual void end();   //SPI vendor De-init
  virtual void beginTransaction(SPISettings settings);
  virtual void endTransaction(void);

  virtual uint8_t transfer(uint8_t data);
  virtual uint16_t transfer16(uint16_t data);
  virtual void transfer(void *buf, size_t count);

  virtual void attachInterrupt(){}
  virtual void detachInterrupt(){}

  virtual void usingInterrupt(int interruptNumber){}
  virtual void notUsingInterrupt(int interruptNumber){}
  virtual ~SPINonInterrupt() { }

  void setBitOrder(BitOrder order);
  void setDataMode(uint8_t uc_mode);
  void setClockDivider(uint8_t uc_div);
};


#endif /* ARDUINOABSTRACTIONS_SPI_H_ */
