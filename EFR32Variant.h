/*
 * EFR32Variant.h
 *
 *  Created on: 20-Jun-2022
 *      Author: aditayagarg
 */

#ifndef ARDUINOABSTRACTIONS_EFR32VARIANT_H_
#define ARDUINOABSTRACTIONS_EFR32VARIANT_H_

#define XG24EK2703B 1

#include "UsbUart.h"
#include "Wire.h"
#ifdef XG24DK2601B
#include "xg24dk2601b.h"
#endif
#ifdef XG24EK2703B
#include "xg24ek2703b.h"
#endif
#include "SPI.h"
#include "Uart.h"

extern UsbUart Serial;
extern I2C Wire;
extern SPINonInterrupt SPI;
extern Uart Serial1;

#endif /* ARDUINOABSTRACTIONS_EFR32VARIANT_H_ */
