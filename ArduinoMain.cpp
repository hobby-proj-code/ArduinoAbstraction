/*
 * ArduinoMain.cpp
 *
 *  Created on: 06-Jul-2022
 *      Author: aditayagarg
 */

#include "VendorCode.h"
#include "ArduinoMain.h"

int main(void)
{
  vendorInit();
  setup();

  for(;;)
  {
      loop();
  }
  return 0;
}
