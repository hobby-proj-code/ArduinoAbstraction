/*
 * SPI.cpp
 *
 *  Created on: 18-Nov-2022
 *      Author: aditayagarg
 */

#include "SPI.h"

void SPINonInterrupt::beginTransaction (SPISettings settings)
{
  SPINonInterrupt::settings = settings;
}

void SPINonInterrupt::begin ()
{
}

void SPINonInterrupt::end ()
{
}

void SPINonInterrupt::endTransaction (void)
{
}

uint8_t SPINonInterrupt::transfer (uint8_t data)
{
  transfer(&data,sizeof(data));
  return data;
}

uint16_t SPINonInterrupt::transfer16 (uint16_t data)
{
  transfer(&data,sizeof(data));
  return data;
}

void SPINonInterrupt::transfer (void *buf, size_t count)
{
}


void SPINonInterrupt::setBitOrder(BitOrder order)
{}

void SPINonInterrupt::setDataMode(uint8_t uc_mode)
{}

void SPINonInterrupt::setClockDivider(uint8_t uc_div)
{}
