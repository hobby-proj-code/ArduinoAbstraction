/*
 * xg24dk2601b.h
 *
 *  Created on: 25-Aug-2022
 *      Author: aditayagarg
 */

#ifndef ARDUINOABSTRACTIONS_XG24DK2601B_H_
#define ARDUINOABSTRACTIONS_XG24DK2601B_H_


//https://www.silabs.com/documents/public/user-guides/ug524-brd2601b-user-guide.pdf
#define BUTTON_0    (18U)          //PB02
#define BUTTON_1    (19U)          //PB03

#define LED_RED     (46U)          //PD02
#define LED_GREEN   (32U)          //PA04
#define LED_BLUE    (21U)          //PB00

#define I2C_SENSOR_ENABLE (10U)       //PC09

///might be wrong recheck
#define UART1_RX 46U     //PD02
#define UART1_TX 18U     //PB03

#endif /* ARDUINOABSTRACTIONS_XG24DK2601B_H_ */
