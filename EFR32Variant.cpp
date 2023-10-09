#include <ArduinoAbstractions/EFR32Variant.h>

//static sl_iostream_uart_t sl_iostream_vcom;
UsbUart Serial;
I2C Wire(I2CPort::I2C0Port);
SPINonInterrupt SPI(SPIPort::SPI0Port);
Uart Serial1(UartPort::Uart1Port);
