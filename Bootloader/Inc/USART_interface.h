/*
 * USART_interface.h
 *
 *  Created on: Oct 31, 2024
 *      Author: user
 */

#ifndef USART_INTERFACE_H_
#define USART_INTERFACE_H_

#include "ErrType.h"

typedef enum{

	PARITY_ERROR_FLAG=0,
	FRAMING_ERROR_FLAG,
	NOISE_DETECTED_FLAG,
	OVERRUN_ERROR_FLAG,
	IDLE_LINE_DETECTED_ERROR,
	READ_DATA_R_NOT_EMPTY_FLAG,
	TRANSMISSION_COMPLETE_FLAG,
	TRANSMIT_DATA_R_EMPTY_FLAG,
	LIN_BREAK_DETECTION_FLAG,
	CTS_FLAG


}USART_FlagID_t;

typedef enum{
	USART_PE=0,
	USART_FE,
	USART_NF,
	USART_ORE,
	USART_IDLE,
	USART_RXNE,
	USART_TC,
	USART_TXE,
	USART_LBD,
	USART_CTS
}USART_SR_t;

typedef enum{
	USART_DR0=0
}USART_DR_t;

typedef enum{
	USART_DIV_FRACTION =0,
	USART_DIV_MANTISSA=4
}USART_BRR_t;

/*
 * Control register 1
 * @USART_CR1_t
 * */
typedef enum{

	USART_SBK=0,	/*Send break			- rw*/
	USART_RWU,		/*Receiver wakeup	    - rw*/
	USART_RE,		/*Receiver enable		- rw*/
	USART_TE,		/*Transmitter enable  	- rw*/
	USART_IDLEIE,	/*IDLE interrupt enable	- rw*/
	USART_RXNEIE,	/*RXNE interrupt enable - rw*/
	USART_TCIE,		/*Transmission complete interrupt enable	- rw*/
	USART_TXEIE,	/*TXE interrupt enable  - rw*/
	USART_PEIE,		/*PE interrupt enable	- rw*/
	USART_PS,		/*Parity selection  	- rw*/
	USART_PCE,		/*Parity control enable	- rw*/
	USART_WAKE,		/*Wakeup method  		- rw*/
	USART_M,		/*Word length			- rw*/
	USART_UE,		/*USART enable  		- rw*/
	USART_OVER8=15	/*Oversampling mode		- rw*/

}USART_CR1_t;

/*
 * Control register 2
 * @USART_CR2_t
 * */
typedef enum{

	USART_ADD0=0,		/*Address of the USART node[3:0]			- rw*/
	USART_LBDL=5,		/*lin break detection length	    		- rw*/
	USART_LBDIE=6,		/*LIN break detection interrupt enable		- rw*/
	USART_LBCL=8,		/*Last bit clock pulse  					- rw*/
	USART_CPHA,			/*Clock phase								- rw*/
	USART_CPOL,			/*Clock polarity 							- rw*/
	USART_CLKEN,		/*Clock enable								- rw*/
	USART_STOP0,		/*STOP bits  								- rw*/
	USART_LINEN=14,		/*LIN mode enable							- rw*/

}USART_CR2_t;

/*
 * Control register 3
 * @USART_CR3_t
 * */
typedef enum{

	USART_EIE=0,		/*Error interrupt enable					- rw*/
	USART_IREN,			/*IrDA mode enable	    					- rw*/
	USART_IRLP,			/*IrDA low-power							- rw*/
	USART_HDSEL,		/*Half-duplex selection 					- rw*/
	USART_NACK,			/*Smartcard NACK enable						- rw*/
	USART_SCEN,			/*Smartcard mode enable 					- rw*/
	USART_DMAR,			/*DMA enable receiver						- rw*/
	USART_DMAT,			/*DMA enable transmitter  					- rw*/
	USART_RTSE,			/*RTS enable								- rw*/
	USART_CTSE,			/*CTS enable								- rw*/
	USART_CTSIE,		/*CTS interrupt enable						- rw*/
	USART_ONEBIT		/*One sample bit method enable				- rw*/

}USART_CR3_t;

/*
 * Guard time and prescaler register
 * @USART_GTPR_t
 * */
typedef enum{

	USART_PSC0=0,		/*Prescaler value[7:0]						- rw*/
	USART_GT0=8			/*Guard time value[7:0]    					- rw*/

}USART_GTPR_t;

/*
 * @brief: USART mode (in transmitter , receiver mode or both modes)
 * @USART_Mode_t
 * */
typedef enum{

	USART_RX=0,			/*Rx Only*/
	USART_TX,			/*Tx Only*/
	USART_RXTX			/*Both Rx and Tx*/

}USART_Mode_t;

/*
 * @brief: USART number of stop bits 0.5, 1, 1.5 or 2 bits
 * @USART_StopBits_t
 * */
typedef enum{

	USART_ONE_STOP_BITS,		/*one stop bit*/
	USART_0P5_STOP_BIT,			/*zero point five stop bit*/
	USART_TWO_STOP_BITS,		/*two stop bits*/
	USART_1P5_STOP_BITS			/*one point five stop bits*/


}USART_StopBits_t;

/*
 * @brief: word length (8 or 9 bits)
 * @USART_WordLength_t
 * */
typedef enum{

	USART_EIGHT_BIT=0,		/*eight bits*/
	USART_NINE_BITS			/*nine bits*/

}USART_WordLength_t;

/*
 * @brief: Parity options
 * @USART_Parity_t
 * */
typedef enum{

	USART_NO_PARITY=0,		/*no parity bit*/
	USART_PARITY			/*with parity bit*/

}USART_Parity_t;

/*
 * @brief: Hardware flow control on/off
 * @USART_HWFlowControl_t
 * */
typedef enum{

	USART_HW_FLOW_CONTROL_ON=0,		/*Hardware flow control on*/
	USART_HW_FLOW_CONTROL_OFF		/*Hardware flow control off*/

}USART_HWFlowControl_t;

/*
 * @brief: USART peripheral number (from 1 to 6)
 * @USART_UsartNumber_t
 * */
typedef enum{

	USART_USART1=0,		/*USART1*/
	USART_USART2,		/*USART2*/
	USART_USART3,		/*USART3*/
	USART_USART4,		/*USART4*/
	USART_USART5,		/*USART5*/
	USART_USART6		/*USART6*/

}USART_UsartNumber_t;

/*
 * @brief: Oversampling option
 * @USART_OverSampling_t
 * */
typedef enum{

	USART_OVER16_=0,		/*Over16*/
	USART_OVER8_		    /*Over8*/

}USART_OverSampling_t;

/*
 * @brief: Synchronous mode
 * @USART_SynchMode_t
 * */
typedef enum{

	USART_ASYNCH=0,		/*Synchronous*/
	USART_SYNCH		/*Asynchronous*/

}USART_SynchMode_t;


/********************************************************************************
 ********************* ### Configurations structure ### *************************
 ********************************************************************************/
/*
 * @brief a structure consists of USART configurations
 * @USART_ConfigReg_t
 * */
typedef struct{

	USART_Mode_t			USART_MODE;
	uint32_t				USART_BAUDRATE;
	USART_StopBits_t		USART_STOPBITS;
	USART_WordLength_t		USART_WORDLENGTH;
	USART_Parity_t			USART_PARITYBIT;
	USART_HWFlowControl_t	USART_HWFLOWCONTROL;
	USART_UsartNumber_t		USART_USARTNUMBER;
	USART_OverSampling_t	USART_OVERSAMPLINGMODE;
	USART_SynchMode_t		USART_SYNCHMODE;

}USART_ConfigReg_t;
/********************************************************************************/

/********************************************************************************
 ********************** ### Functions Declarations ### **************************
 ********************************************************************************/
/*
 * @fn:USART_ReadFlag
 * @brief:Read interrupts Flag
 * @param:Copy_USARTNum
 * @param:Copy_FlagID			(@USART_UsartNumber_t)
 * @param:Copy_FlagVal			(@USART_FlagID_t)
 * @retval: Local_ErrorStatus	(@ErrorState_t)
 * 						USART error status
 * */
ErrorState_t USART_ReadFlag(USART_UsartNumber_t Copy_USARTNum,USART_FlagID_t Copy_FlagID, uint8_t* Copy_FlagVal);
/****************INITIALIZATION**************/
/*
 * @fn:	USART_Init
 * @brief:	to initialize the USART configurations
 * @param:	Copy_ConfigReg 		(enum @USART_ConfigReg_t)
 * 						a pointer to structure consists of the USART configurations
 * @retval: Copy_ErrorStatus	(@ErrorState_t)
 * 						USART error status
 * */
ErrorState_t USART_Init(const USART_ConfigReg_t* Copy_ConfigReg);

				/****************POLLing**************/

/*
 * @fn:	USART_Tx
 * @brief:	transmit data with polling
 * @param:	Copy_ConfigReg 		(enum @USART_ConfigReg_t)
 * 						a pointer to structure consists of the USART configurations
 * @param: Copy_TransmitingData  (uint8_t)
 * @retval: Local_ErrorStatus	(@ErrorState_t)
 * 						USART error status
 * */
ErrorState_t USART_Tx(const USART_ConfigReg_t* Copy_ConfigReg , uint8_t Copy_TransmitingData);

/*
 * @fn:	USART_Rx
 * @brief:	receive data with polling
 * @param:	Copy_ConfigReg 		(enum @USART_ConfigReg_t)
 * 						a pointer to structure consists of the USART configurations
 * @param: Copy_ReceivingData   (uint8_t*)
 * @retval: Local_ErrorStatus	(@ErrorState_t)
 * 						USART error status
 * */
ErrorState_t USART_Rx(const USART_ConfigReg_t* Copy_ConfigReg, uint8_t* Copy_ReceivingData);

				/***************INTERRUPT*************/

/*
 * @fn:	USART_Tx_IT
 * @brief:	transmit data with interrupt notification
 * @param:	Copy_ConfigReg 		(enum @USART_ConfigReg_t)
 * 						a pointer to structure consists of the USART configurations
 * @retval: Copy_ErrorStatus	(@ErrorState_t)
 * 						USART error status
 * */
ErrorState_t USART_Tx_IT(USART_UsartNumber_t Copy_USARTNum, uint8_t Copy_TransmitingData);

/*
 * @fn:	USART_Rx_IT
 * @brief:	receive data with interrupt notification
 * @param:	Copy_ConfigReg 		(enum @USART_ConfigReg_t)
 * 						a pointer to structure consists of the USART configurations
 * @retval: Copy_ErrorStatus	(@ErrorState_t)
 * 						USART error status
 * */
ErrorState_t USART_Rx_IT(USART_UsartNumber_t Copy_USARTNum, uint16_t* Copy_ReceivingData);


				/******************DMA****************/

/*
 * @fn:	USART_Tx_DMA
 * @brief:	transmit data with DMA
 * @param:	Copy_ConfigReg 		(enum @USART_ConfigReg_t)
 * 						a pointer to structure consists of the USART configurations
 * @retval: Copy_ErrorStatus	(@ErrorState_t)
 * 						USART error status
 * */
ErrorState_t USART_Tx_DMA(const USART_ConfigReg_t* Copy_ConfigReg);

/*
 * @fn:	USART_Rx_DMA
 * @brief:	receive data with DMA
 * @param:	Copy_ConfigReg 		(enum @USART_ConfigReg_t)
 * 						a pointer to structure consists of the USART configurations
 * @retval: Copy_ErrorStatus	(@ErrorState_t)
 * 						USART error status
 * */
ErrorState_t USART_Rx_DMA(const USART_ConfigReg_t* Copy_ConfigReg);

/*
 * @fn:	USART_CallBack
 * @brief:	Set the call back function
 * @param:	Copy_USARTNum 		(enum USART_UsartNumber_t)
 * @param:	Copy_FlagID 		(enum USART_FlagID_t)
 * @param:	Copy_CallBackFunc 	pointer void function
 * @retval: Copy_ErrorStatus	(@ErrorState_t)
 * 						USART error status
 * */
ErrorState_t USART_CallBack(USART_UsartNumber_t Copy_USARTNum,USART_FlagID_t Copy_FlagID, void (*Copy_CallBackFunc)(void));
void USART_IRQHandler(USART_UsartNumber_t Copy_USARTNum);



#endif /* USART_INTERFACE_H_ */
