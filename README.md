# ArduinoAbstraction
Arduino wrapper for other platforms. Using this wrapper developper can port their Arduino code to supported platforms with none or small modifications. Also, Arduino libraries can easily be used in supported platforms with small or no modification

# Supported platforms
It supports below platforms
* [Simplicity studio (from Silicon Labs)](https://www.silabs.com/developers/simplicity-studio)

# Tested against
It is tested aginst below hardware
* [efr32xg24-dev-kit](https://www.silabs.com/development-tools/wireless/efr32xg24-dev-kit?tab=overview)
* [efr32xg24-explorer-kit](https://www.silabs.com/development-tools/wireless/efr32xg24-explorer-kit?tab=overview)

# Setup guide for Silicon Studio
Follow below steps to start using ArduinoAbstractions in Simplicity studio. Refer [Example](https://github.com/hobby-proj-code/SiStArduinoAbstraction) which uses Arduino SI7021  
1. Copy ArduinoAbstraction folder to project folder. For Git, this repo can be added as submodule also. Refer below sample for setting up Git as submodule.

   ```bash
   git submodule add https://github.com/hobby-proj-code/ArduinoAbstraction ./ArduinoAbstractionExample/
   ```
2. For new project, "main" function is implemented within ArduinoAbstraction. Developer needs to implement "setup" and "loop" function as per Arduino. Developer can migrate their "main" function to loop and their set-up functionality to "setup" function. If it is a porting from existing Arduino project then no changes should be required.
3. Add below software components in Simplicity studio using "*.slcp" to the project.

   ```bash
   Services->IO Stream->IO Stream: EUSART 
     * Before adding verify that EUSART0_RX and EUSART0_TX
     * Name the instance as vcom
   Services->Sleep Timer
   Platform->Peripheral->I2C
   ```
4. Go to project properties and edit "C/C++ Build->Settings". Select "[All configurations]" and add include path "${workspace_loc:/${ProjName}/ArduinoAbstractions}" to C and C++ compiler setting.
5. Again go to project properties and edit "C/C++ Build->Settings". Select "[All configurations]" and add Preprocessor define "ARDUINO=101" to C and C++ compiler setting.
6. Include "Arduino.h" in file where "setup" and "loop" function are implemented. To use Arduino functionality in other files include "Arduino.h".
