/*
 * ArduinoMain.cpp
 *
 *  Created on: 06-Jul-2022
 *      Author: aditayagarg
 */

#include <ArduinoAbstractions/VendorCode.h>
#include <ArduinoAbstractions/ArduinoMain.h>

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
