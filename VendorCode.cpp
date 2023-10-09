/*
 * VendorCode.cpp
 *
 *  Created on: 06-Jul-2022
 *      Author: aditayagarg
 */

#include <ArduinoAbstractions/VendorCode.h>
#include <ArduinoAbstractions/Arduino.h>


#define _GPIOINT_IF_EVEN_MASK ((_GPIO_IF_MASK) & 0x55555555UL)
#define _GPIOINT_IF_ODD_MASK  ((_GPIO_IF_MASK) & 0xAAAAAAAAUL)
#define INTERRUPT_UNAVAILABLE 0xFF

#include <sl_iostream_uart.h>

extern "C"
{
  #include <sl_system_init.h>
  #include <sl_iostream_eusart_vcom_config.h>
  #include <em_eusart.h>
  #include <sl_sleeptimer.h>
  #include <em_gpio.h>
  #include <em_cmu.h>
  #include <em_i2c.h>
  //GPIOINT - GPIO interrupt package
  //#include <gpiointerrupt.h>
  //#include <sl_udelay.h>
}

void errorBlock(const char* errorMessage)
{
  Serial.println(errorMessage);
  while(1);
}

int printChar(char charac)
{
  sl_iostream_putchar(SL_IOSTREAM_STDOUT,charac);
  return 1;
}

int readChar()
{
  int charac = -1;
  sl_iostream_getchar(SL_IOSTREAM_STDOUT,(char*)&charac);
  return charac;
}

bool setBaudRateUSB(int baudRate)
{
  //EUSART_BaudrateSet(eusart, init->refFreq, init->baudrate);
  EUSART_BaudrateSet(SL_IOSTREAM_EUSART_VCOM_PERIPHERAL, 0, baudRate);
  EUSART_Enable(SL_IOSTREAM_EUSART_VCOM_PERIPHERAL, eusartEnable);
  return true;
}

bool vendorInit()
{
  sl_system_init();

  //GPIOINT_Init();
  if (CORE_NvicIRQDisabled(GPIO_ODD_IRQn)) {
      NVIC_ClearPendingIRQ(GPIO_ODD_IRQn);
      NVIC_EnableIRQ(GPIO_ODD_IRQn);
    }
    if (CORE_NvicIRQDisabled(GPIO_EVEN_IRQn)) {
      NVIC_ClearPendingIRQ(GPIO_EVEN_IRQn);
      NVIC_EnableIRQ(GPIO_EVEN_IRQn);
    }

  sl_sleeptimer_init();

  return true;
}

void delay(unsigned long milliseconds)
{
  unsigned long stMilliSec = millis();
  while(millis() - stMilliSec < milliseconds && millis() >= stMilliSec);
  //sl_sleeptimer_delay_millisecond(milliseconds);
}

unsigned long millis()
{
  uint64_t millis = 0;
  sl_sleeptimer_tick64_to_ms(sl_sleeptimer_get_tick_count64(),&millis);
  return millis;//RTCDRV_TicksToMsec32(RTCDRV_GetWallClockTicks32());
}

unsigned long micros()
{
  uint64_t micros = 0;
  sl_sleeptimer_tick64_to_ms(sl_sleeptimer_get_tick_count64(),&micros);
  return micros;//RTCDRV_TicksToMsec32(RTCDRV_GetWallClockTicks32());
}

GPIO_Port_TypeDef GetPort(int pinNumber)
{
  GPIO_Port_TypeDef numericPinToQFNPortMap[]{GPIO_Port_TypeDef::gpioPortC,GPIO_Port_TypeDef::gpioPortB,
      GPIO_Port_TypeDef::gpioPortA,GPIO_Port_TypeDef::gpioPortD};
  return numericPinToQFNPortMap[(int)((pinNumber-1)/12)];
}

int GetPortPin(int pinNumber)
{
  int numericPinToPortPinMap[]{
      0,1,2,3,4,5,6,7,8,9,-1,-1,
      -1,-1,-1,-1,-1,3,2,1,0,-1,-1,-1,
      -1,-1,-1,0,1,2,3,4,5,6,7,-1,
      -1,-1,-1,-1,-1,-1,5,4,3,2,1,0
  };
  return numericPinToPortPinMap[pinNumber-1];
}

GPIO_Mode_TypeDef GetArduinoToEFRMode(PinMode pinMode)
{
  GPIO_Mode_TypeDef arduinoToEFRModeMap[]{
      GPIO_Mode_TypeDef::gpioModeInput, //PinMode::INPUT
      GPIO_Mode_TypeDef::gpioModePushPull, //PinMode::OUTPUT
      GPIO_Mode_TypeDef::gpioModeInputPullFilter, //PinMode::INPUT_PULLUP
      GPIO_Mode_TypeDef::gpioModeInputPullFilter //PinMode::INPUT_PULLDOWN
  };
  return arduinoToEFRModeMap[(int)pinMode];
}

void pinMode(pin_size_t pinNumber, PinMode pinMode)
{
  GPIO_PinModeSet(GetPort(pinNumber),GetPortPin(pinNumber),GetArduinoToEFRMode(pinMode),
                  PinMode::INPUT_PULLUP == pinMode ? 1 : 0);
}

void digitalWrite(pin_size_t pinNumber, PinStatus status)
{
  if(status == PinStatus::LOW)
  {
      GPIO_PinOutClear(GetPort(pinNumber),GetPortPin(pinNumber));
  }
  else if(status == PinStatus::HIGH)
  {
      GPIO_PinOutSet(GetPort(pinNumber),GetPortPin(pinNumber));
  }
  else if(status == PinStatus::CHANGE)
  {
      GPIO_PinOutToggle(GetPort(pinNumber),GetPortPin(pinNumber));
  }
}

PinStatus digitalRead(int pinNumber)
{
  return GPIO_PinInGet(GetPort(pinNumber),GetPortPin(pinNumber)) == 0 ?
      PinStatus::LOW : PinStatus::HIGH;
}

int digitalPinToInterrupt(int pin)
{return pin;}

int InterruptNumberToPinNumberMap[] =
    {-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1
    ,-1,-1,-1,-1};
InterruptCallback InterruptNumberCallbacks[sizeof(InterruptNumberToPinNumberMap)]{};


void attachInterrupt(pin_size_t pinNumber, voidFuncPtr callback, PinStatus status)
{
  //https://www.silabs.com/documents/public/application-notes/an0012-efm32-gpio.pdf
  //In MG24 total 16 Interrupts (Interrupt number) is available, so at maximum 16 GPIO can be used for interrupt
  //It seems not an issue but there is a restriction that each interrupt number can be applied to certain GPIO
  //For easy understanding, consider 0 pin on each port can be bound to interrupt number zero and so on.
  //  As MG24 have 12 pins package (not 16) so interrupt number get reduced to 12 and only
  //    one pin among each port can be used at a time, also among 12 max 8 can be used as GPIO so
  //    usable interrupt number is reduced to 8
  //  Some MCU (MG24 one of them) has handled it to some extent by grouping pins and any of the GPIO pin in a group
  //    can bound to any of interrupt number in group. So Pin zero of multiple ports can bound to different interrupt
  //    number providing some remedy
  //attach function to interrupt before configuring

  //somehow gpiointerrupt.h APIs are crashing and going into default handler
  if(status == PinStatus::HIGH || status == PinStatus::LOW)
  {
      errorBlock("Status not supported for interrupt");
  }

  uint8_t interruptNumber = InterruptNumberToPinNumberMap[GetPortPin(pinNumber)] != -1 ?
      INTERRUPT_UNAVAILABLE : GetPortPin(pinNumber);
  if(interruptNumber != INTERRUPT_UNAVAILABLE)
  {
      InterruptNumberToPinNumberMap[interruptNumber] = interruptNumber;
      uint32_t flag = 1<<interruptNumber;
      InterruptNumberCallbacks[interruptNumber] = callback;
      GPIO_IntDisable(flag);
      GPIO_ExtIntConfig(GetPort(pinNumber),GetPortPin(pinNumber),interruptNumber,
                    status == PinStatus::RISING ? true : false,
                    status == PinStatus::FALLING ? true : false,
                    false);
      GPIO_IntClear(flag);
      GPIO_IntEnable(flag);
  }
  else
  {
      errorBlock("Failed to get interrupt number");
  }
}

void detachInterrupt(int pinNumber)
{
  uint8_t interruptNumber = INTERRUPT_UNAVAILABLE;
  for(int interruptNumCounter = 0; interruptNumCounter < sizeof(InterruptNumberToPinNumberMap); interruptNumCounter++)
  {
      if(InterruptNumberToPinNumberMap[interruptNumCounter] == pinNumber)
      {
        interruptNumber = interruptNumCounter;
        break;
      }
  }
  if(interruptNumber != INTERRUPT_UNAVAILABLE)
  {
      InterruptNumberToPinNumberMap[interruptNumber] = -1;
      InterruptNumberCallbacks[interruptNumber] = nullptr;
      uint32_t flag = 1<<interruptNumber;
      GPIO_IntDisable(flag);
  }
}

enum InterruptHandlerType
{
  GPIO_EVEN_IRQ,
  GPIO_ODD_IRQ
};

void irqHandler (InterruptHandlerType intHandType)
{
  if (intHandType == InterruptHandlerType::GPIO_EVEN_IRQ
      || intHandType == InterruptHandlerType::GPIO_ODD_IRQ)
    {
      uint32_t mask =
          intHandType == InterruptHandlerType::GPIO_EVEN_IRQ ?
              _GPIOINT_IF_EVEN_MASK : _GPIOINT_IF_ODD_MASK;
      uint32_t iflags;

      /* Get all even interrupts. */
      iflags = GPIO_IntGetEnabled () & mask;
      GPIO_IntClear (iflags);
      for (int interruptNumCounter = 0;
          interruptNumCounter < sizeof(InterruptNumberToPinNumberMap);
          interruptNumCounter++)
        {
          if ((iflags & 1 << interruptNumCounter) != 0
              && InterruptNumberCallbacks[interruptNumCounter] != nullptr)
            {
              (*InterruptNumberCallbacks[interruptNumCounter]) ();
            }
        }
    }
}

void GPIO_EVEN_IRQHandler(void)
{
  irqHandler(InterruptHandlerType::GPIO_EVEN_IRQ);
}

/***************************************************************************//**
* @brief
*   GPIO ODD interrupt handler. Interrupt handler clears all IF odd flags and
*   call the dispatcher passing the flags which triggered the interrupt.
*
******************************************************************************/
void GPIO_ODD_IRQHandler(void)
{
  irqHandler(InterruptHandlerType::GPIO_ODD_IRQ);
}


bool deInitI2C(int port)
{
  if(port != I2CPort::I2C0Port &&
        port != I2CPort::I2C1Port)
  {errorBlock("I2C Port Incorrect");}

  GPIO_Port_TypeDef i2cPortSCA = gpioPortC;
  int i2cPinSCA = 4;
  GPIO_Port_TypeDef i2cPortSDA = gpioPortC;
  int i2cPinSDA = 5;
  I2C_TypeDef* i2cType = I2C0;
  CMU_Clock_TypeDef clock = cmuClock_I2C0;
  int routeArray = 0;
  I2C_Init_TypeDef i2cInit;

  if(port == I2CPort::I2C0Port)
  {
     i2cPortSCA = gpioPortC;
     i2cPinSCA = 4;
     i2cPortSDA = gpioPortC;
     i2cPinSDA = 5;
     clock = cmuClock_I2C0;
     routeArray = 0;
     i2cType = I2C0;
  }
  else
  {
     errorBlock("I2C1 not configured");
  }


  //I2C_Reset
  I2C_Reset(i2cType);
  GPIO_PinModeSet(i2cPortSCA, i2cPinSCA, gpioModeDisabled, 0);
  GPIO_PinModeSet(i2cPortSDA, i2cPinSDA, gpioModeDisabled, 0);

  CMU_ClockEnable(clock, false);
}

bool initI2C(int port)
{
  if(port != I2CPort::I2C0Port &&
      port != I2CPort::I2C1Port)
  {errorBlock("I2C Port Incorrect");}

  //I2C pin and port
  GPIO_Port_TypeDef i2cPortSCA = gpioPortC;
  int i2cPinSCA = 4;
  GPIO_Port_TypeDef i2cPortSDA = gpioPortC;
  int i2cPinSDA = 5;
  I2C_TypeDef* i2cType = I2C0;
  CMU_Clock_TypeDef clock = cmuClock_I2C0;
  int routeArray = 0;
  I2C_Init_TypeDef i2cInit;

  if(port == I2CPort::I2C0Port)
  {
      i2cPortSCA = gpioPortC;
      i2cPinSCA = 4;
      i2cPortSDA = gpioPortC;
      i2cPinSDA = 5;
      clock = cmuClock_I2C0;
      routeArray = 0;
      i2cType = I2C0;
  }
  else
  {
     errorBlock("I2C1 not configured");
  }


  //enable GPIO clock for it to work
  //should be already enabled but do it fo safety
  CMU_ClockEnable(cmuClock_GPIO, true);


  //enable clock if not already
  CMU_ClockEnable(clock, true);

  //set pin mode of SDA SCL
  GPIO_PinModeSet(i2cPortSCA, i2cPinSCA, gpioModeWiredAndPullUp, 1);
  GPIO_PinModeSet(i2cPortSDA, i2cPinSDA, gpioModeWiredAndPullUp, 1);

  //set all slave devices into known state
  for (int i = 0; i < 9; i++) {
      GPIO_PinOutClear(i2cPortSCA, i2cPinSCA);
      delay(100);
      GPIO_PinOutSet(i2cPortSCA, i2cPinSCA);
      delay(100);
   }

  GPIO->I2CROUTE[routeArray].ROUTEEN = GPIO_I2C_ROUTEEN_SDAPEN | GPIO_I2C_ROUTEEN_SCLPEN;
  GPIO->I2CROUTE[routeArray].SCLROUTE = (uint32_t)((i2cPinSCA << _GPIO_I2C_SCLROUTE_PIN_SHIFT)
                               | (i2cPortSCA << _GPIO_I2C_SCLROUTE_PORT_SHIFT));
  GPIO->I2CROUTE[routeArray].SDAROUTE = (uint32_t)((i2cPinSDA << _GPIO_I2C_SDAROUTE_PIN_SHIFT)
                               | (i2cPortSDA << _GPIO_I2C_SDAROUTE_PORT_SHIFT));

  //init i2c
  i2cInit.enable = true;
  i2cInit.master = true; /* master mode only */
  i2cInit.freq = I2C_FREQ_STANDARD_MAX;
  i2cInit.refFreq = 0;
  i2cInit.clhr = I2C_ClockHLR_TypeDef::i2cClockHLRStandard;

  I2C_Init(i2cType, &i2cInit);
  return true;
}

bool setI2CFrequency(int port, uint32_t frequency)
{
  if(port != I2CPort::I2C0Port &&
        port != I2CPort::I2C1Port)
  {errorBlock("I2C Port Incorrect");}

  I2C_TypeDef* i2cType = I2C0;

    if(port == I2CPort::I2C0Port)
    {
        i2cType = I2C0;
    }
    else
    {
        i2cType = I2C1;
    }

    I2C_BusFreqSet(i2cType,0, frequency, I2C_ClockHLR_TypeDef::i2cClockHLRStandard);
    return true;
}

int i2cRecvBuffer(int port, uint8_t address, uint8_t* buffer,int buffLen, int timeout)
{
  if(buffLen == 0)
  {
      errorBlock("usage fault not allowed");
  }
  I2C_TransferReturn_TypeDef ret;
  I2C_TransferSeq_TypeDef seq;
  //uint8_t dummy[1];
  // Do a polled transfer
  seq.addr = address << 1;
  seq.buf[0].data = buffer;
  seq.buf[0].len = buffLen;
  //seq.buf[1].data = dummy;
  //seq.buf[1].len = 0;
  seq.flags = I2C_FLAG_READ;
  auto stMillis = millis();
  ret = I2C_TransferInit(port == I2CPort::I2C0Port ? I2C0 : I2C1, &seq);
  while (ret == i2cTransferInProgress)// && millis() - stMillis < timeout)
  {
    ret = I2C_Transfer(I2C0);
  }
  return ret == i2cTransferDone ? buffLen : 0;
}

int i2cTrasBuffer(int port, uint8_t address, uint8_t* buffer,int buffLen, int timeout)
{
  I2C_TransferReturn_TypeDef ret;
  I2C_TransferSeq_TypeDef seq;
  //uint8_t dummy[1];
  // Do a polled transfer
  seq.addr = address << 1;
  seq.buf[0].data = buffer;
  seq.buf[0].len = buffLen;
  //seq.buf[1].data = dummy;
  //seq.buf[1].len = 0;
  seq.flags = I2C_FLAG_WRITE;
  auto stMillis = millis();
  ret = I2C_TransferInit(port == I2CPort::I2C0Port ? I2C0 : I2C1, &seq);
  while (ret == i2cTransferInProgress)// && millis() - stMillis < timeout)
  {
    ret = I2C_Transfer(I2C0);
  }
  return ret == i2cTransferDone ? buffLen : (buffLen == 0 ? -1 : 0);
}

bool deInitUART(int port)
{
  if(/*port != UartPort::Uart0Port &&*/
        port != UartPort::Uart1Port)
  {errorBlock("UART Port Incorrect");}

  EUSART_TypeDef* uart = (EUSART1);
  CMU_Clock_TypeDef clock = cmuClock_EUSART1;
  int routeArray = 1;
  //PB03 - GPIO - EXP11
  //PD02 - GPIO - EXP13
  GPIO_Port_TypeDef uartTXPort = gpioPortB;
  int uartTXPin = 3;
  GPIO_Port_TypeDef uartRXPort = gpioPortD;
  int uartRXPin = 2;

  EUSART_Reset(uart);
  GPIO_PinModeSet(uartTXPort, uartTXPin, gpioModeDisabled, 0);
  GPIO_PinModeSet(uartRXPort, uartRXPin, gpioModeDisabled, 0);

  CMU_ClockEnable(clock, false);
}

bool initUART(int port, uint8_t parity, uint8_t dataBitLength, uint8_t stopBitLength, int baudRate)
{
  if(/*port != UartPort::Uart0Port &&*/
        port != UartPort::Uart1Port)
  {errorBlock("UART Port Incorrect");}
  if(parity != 0 || dataBitLength != 8 || stopBitLength != 1)
  {errorBlock("UART Port Config Incorrect");}


  EUSART_UartInit_TypeDef uartInit = EUSART_UART_INIT_DEFAULT_HF;
  EUSART_TypeDef* uart = (EUSART1);
  CMU_Clock_TypeDef clock = cmuClock_EUSART1;
  int routeArray = 1;
  //PB03 - GPIO - EXP11
  //PD02 - GPIO - EXP13
  GPIO_Port_TypeDef uartTXPort = GetPort(UART1_TX);
  int uartTXPin = GetPortPin(UART1_TX);
  GPIO_Port_TypeDef uartRXPort = GetPort(UART1_RX);
  int uartRXPin = GetPortPin(UART1_RX);

  uartInit.baudrate = baudRate;
  //enable GPIO clock for it to work
  //should be already enabled but do it fo safety
  CMU_ClockEnable(cmuClock_GPIO, true);

  //enable clock if not already
  CMU_ClockEnable(clock, true);

  //set pin mode of SDA SCL
  GPIO_PinModeSet(uartTXPort, uartTXPin, gpioModePushPull, 1);//tx
  GPIO_PinModeSet(uartRXPort, uartRXPin, gpioModeInputPull, 1);//rx

  GPIO->EUSARTROUTE[routeArray].RXROUTE   = ((uartRXPort << _GPIO_EUSART_RXROUTE_PORT_SHIFT) | (uartRXPin  << _GPIO_EUSART_RXROUTE_PIN_SHIFT));
  GPIO->EUSARTROUTE[routeArray].TXROUTE   = ((uartTXPort << _GPIO_EUSART_TXROUTE_PORT_SHIFT) | (uartTXPin  << _GPIO_EUSART_TXROUTE_PIN_SHIFT));
  GPIO->EUSARTROUTE[routeArray].ROUTEEN   = GPIO_EUSART_ROUTEEN_RXPEN | GPIO_EUSART_ROUTEEN_TXPEN;

  //em_eusart.c
  EUSART_UartInitHf(uart,&uartInit);
  EUSART_Enable(uart, eusartEnable);
  return true;
}


int uartRecvBuffer(int port, uint8_t* buffer,int buffLen, int timeout)
{
  if((/*port != UartPort::Uart0Port &&*/
          port != UartPort::Uart1Port) || buffLen <= 0)
  {errorBlock("UART Port Incorrect");}
  EUSART_TypeDef* uart = (EUSART1);
  int charReceived = 0;
  auto stMillis = millis();
  while((millis() - stMillis < timeout) && (buffLen - charReceived) <= 0)
  {
      if(uart->STATUS & EUSART_STATUS_RXFL)
      {
          *(buffer+charReceived++) = (uint8_t)uart->RXDATA;
      }
  }

  return charReceived;
}


int uartTrasBuffer(int port, uint8_t* buffer,int buffLen, int timeout)
{
  if((/*port != UartPort::Uart0Port &&*/
            port != UartPort::Uart1Port) || buffLen <= 0)
    {errorBlock("UART Incorrect");}
  //Serial.println((char*)buffer);
    EUSART_TypeDef* uart = (EUSART1);
    //quick bug fix unknow issue
    //EUSART_Tx(uart,*(buffer));
    for(int counter = 0; counter < buffLen; counter++)
    {
        //Serial.println("iterating");
        EUSART_Tx(uart,*(buffer+counter));
    }
    return buffLen;
}


