/*
 * USART_prv.h
 *
 *  Created on: Oct 31, 2024
 *      Author: user
 */

#ifndef USART_PRV_H_
#define USART_PRV_H_

#define NUMBER_OF_USART_PERIPH        6
#define FLAG_ID_NUMBERS               10

#define Fck                           16000000UL
#define BAUDRATE_FAC_MASK             1000U
#define FRACTION_FAC_MASK             160U
#define GET_ROUND_MASK                500UL

#define USART_EN_MASK                 1
#define TRANSIMTTED_DATA_MASK         0x000001FF

#define TX_EN_MASK                    1
#define RX_EN_MASK                    1
#define CTS_EN_MASK                   1
#define RTS_EN_MASK                   1

#define EMPTY_TXDR                    1
#define READY_RXDR                    1
#define TRAS_COMPLETED				  1
#define GETVAL_MASK				      0x01


#endif /* USART_PRV_H_ */
