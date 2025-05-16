/*
 * USART_prog.c
 *
 *  Created on: Oct 31, 2024
 *      Author: user
 */


#include <stdint.h>
#include "Stm32F446xx.h"
#include "USART_interface.h"
#include "USART_prv.h"
#include "ErrType.h"

static USART_Reg_t USARTx[NUMBER_OF_USART_PERIPH]={USART1,USART2,USART3,USART4,USART5,USART6};

static void(*G_CallBackFunc[NUMBER_OF_USART_PERIPH][FLAG_ID_NUMBERS])(void)={NULL};

static uint16_t *Global_ReceiveData[NUMBER_OF_USART_PERIPH]={NULL};

static uint8_t Global_TransimittedData[NUMBER_OF_USART_PERIPH]={0};

/*
 * @fn:USART_ReadFlag
 * @brief:Read interrupts Flag
 * @param:Copy_USARTNum
 * @param:Copy_FlagID			(@USART_UsartNumber_t)
 * @param:Copy_FlagVal			(@USART_FlagID_t)
 * @retval: Local_ErrorStatus	(@ErrorState_t)
 * 						USART error status
 * */
ErrorState_t USART_ReadFlag(USART_UsartNumber_t Copy_USARTNum,USART_FlagID_t Copy_FlagID, uint8_t* Copy_FlagVal)
{
	ErrorState_t Local_ErrorState = OK;
	if(Copy_FlagVal==NULL)
	{
		Local_ErrorState = NULL_PTR_ERR;

	}

	if((Copy_USARTNum>=USART1)&&(Copy_USARTNum<=USART6)&&(Copy_FlagID>=TRANSMIT_DATA_R_EMPTY_FLAG)&&(Copy_FlagID<=READ_DATA_R_NOT_EMPTY_FLAG))
	{
		*Copy_FlagVal = ( ((USARTx[Copy_USARTNum].USART_SR) >> Copy_FlagID) & 0x01 ) ;
	}
	else
	{
		Local_ErrorState=NOK;
	}
	return Local_ErrorState;
}
/****************INITIALIZATION**************/
/*
 * @fn:	USART_Init
 * @brief:	to initialize the USART configurations
 * @param:	Copy_ConfigReg 		(enum @USART_ConfigReg_t)
 * 						a pointer to structure consists of the USART configurations
 * @retval: Copy_ErrorStatus	(@ErrorState_t)
 * 						USART error status
 * */
ErrorState_t USART_Init(const USART_ConfigReg_t* Copy_ConfigReg)
{
	ErrorState_t Local_ErrorState = OK;
	uint32_t Local_u32USARTDIV=0;
	uint16_t Local_u16Mantissa=0;
	uint8_t Local_u8FractionPart=0;

	if(Copy_ConfigReg==NULL)
	{
		Local_ErrorState = NULL_PTR_ERR;
		return Local_ErrorState;
	}

	if( (Copy_ConfigReg->USART_MODE >= USART_RX) && (Copy_ConfigReg->USART_MODE <= USART_RXTX)
			&& (Copy_ConfigReg->USART_STOPBITS >= USART_ONE_STOP_BITS) && (Copy_ConfigReg->USART_STOPBITS <= USART_1P5_STOP_BITS)
			&& (Copy_ConfigReg->USART_WORDLENGTH >= USART_EIGHT_BIT) && (Copy_ConfigReg->USART_WORDLENGTH <= USART_NINE_BITS)
			&& (Copy_ConfigReg->USART_PARITYBIT >= USART_NO_PARITY) && (Copy_ConfigReg->USART_PARITYBIT <= USART_PARITY)
			&& (Copy_ConfigReg->USART_HWFLOWCONTROL >= USART_HW_FLOW_CONTROL_ON) && (Copy_ConfigReg->USART_HWFLOWCONTROL <= USART_HW_FLOW_CONTROL_OFF)
			&& (Copy_ConfigReg->USART_USARTNUMBER >= USART_USART1) && (Copy_ConfigReg->USART_USARTNUMBER <= USART_USART6)
			&& (Copy_ConfigReg->USART_OVERSAMPLINGMODE >= USART_OVER16_) && (Copy_ConfigReg->USART_OVERSAMPLINGMODE <= USART_OVER8_)
			&& (Copy_ConfigReg->USART_SYNCHMODE >= USART_ASYNCH) && (Copy_ConfigReg->USART_SYNCHMODE <= USART_SYNCH))
	{
		/*Make control register 1 is on reset value */
		USARTx[Copy_ConfigReg->USART_USARTNUMBER].USART_CR1 &=~0xFFFFFFFF;
		/*Make control register 2 is on reset value */
		USARTx[Copy_ConfigReg->USART_USARTNUMBER].USART_CR2 &=~0xFFFFFFFF;
		/*Make control register 3 is on reset value */
		USARTx[Copy_ConfigReg->USART_USARTNUMBER].USART_CR3 &=~0xFFFFFFFF;

		USARTx[Copy_ConfigReg->USART_USARTNUMBER].USART_CR1 |=((Copy_ConfigReg->USART_STOPBITS)<<USART_STOP0);

		USARTx[Copy_ConfigReg->USART_USARTNUMBER].USART_CR2 |=((Copy_ConfigReg->USART_WORDLENGTH)<<USART_M);

		USARTx[Copy_ConfigReg->USART_USARTNUMBER].USART_CR3 |=((Copy_ConfigReg->USART_PARITYBIT)<<USART_PCE);

		if(Copy_ConfigReg->USART_HWFLOWCONTROL ==USART_HW_FLOW_CONTROL_ON)
		{
			USARTx[Copy_ConfigReg->USART_USARTNUMBER].USART_CR3 |=(CTS_EN_MASK<<USART_CTSE);

			USARTx[Copy_ConfigReg->USART_USARTNUMBER].USART_CR3 |=(RTS_EN_MASK<<USART_RTSE);
		}
		USARTx[Copy_ConfigReg->USART_USARTNUMBER].USART_CR1 |=((Copy_ConfigReg->USART_OVERSAMPLINGMODE)<<USART_OVER8);

		switch(Copy_ConfigReg->USART_OVERSAMPLINGMODE)
		{
		case USART_OVER16_ :
			Local_u32USARTDIV = ((Fck / (8u*2u*(Copy_ConfigReg->USART_BAUDRATE))) * BAUDRATE_FAC_MASK ) ;
		case USART_OVER8_ :
			Local_u32USARTDIV = ((Fck / (8u*(Copy_ConfigReg->USART_BAUDRATE))) * BAUDRATE_FAC_MASK ) ;
		}
		Local_u16Mantissa 	 = (uint16_t)( Local_u32USARTDIV / BAUDRATE_FAC_MASK ) ;
		Local_u8FractionPart = (uint8_t)( (((( Local_u32USARTDIV % BAUDRATE_FAC_MASK) * FRACTION_FAC_MASK ) + GET_ROUND_MASK ) / BAUDRATE_FAC_MASK ) ) ;

		USARTx[Copy_ConfigReg->USART_USARTNUMBER].USART_BRR |= (Local_u8FractionPart<<USART_DIV_FRACTION);
		USARTx[Copy_ConfigReg->USART_USARTNUMBER].USART_BRR |= (Local_u16Mantissa<<USART_DIV_MANTISSA);

		/*8- Synch/Asynch*/
		USARTx[Copy_ConfigReg->USART_USARTNUMBER].USART_CR2 |= ((Copy_ConfigReg->USART_SYNCHMODE) << USART_CLKEN);
		/*9- USART enable*/
		USARTx[Copy_ConfigReg->USART_USARTNUMBER].USART_CR1 |= (USART_EN_MASK << USART_UE);
		/*1-mode Rx, Tx or both */
		switch(Copy_ConfigReg->USART_MODE)
		{
		case USART_RX 	:
			/*Enable Receiver*/
			USARTx[Copy_ConfigReg->USART_USARTNUMBER].USART_CR1 |= ( TX_EN_MASK << USART_RE);
			break;
		case USART_TX 	:
			/*Enable transmitter*/
			USARTx[Copy_ConfigReg->USART_USARTNUMBER].USART_CR1 |= ( TX_EN_MASK << USART_TE);
			break;
		case USART_RXTX :
			/*Enable Receiver*/
			USARTx[Copy_ConfigReg->USART_USARTNUMBER].USART_CR1 |= ( TX_EN_MASK << USART_RE);
			/*Enable transmitter*/
			USARTx[Copy_ConfigReg->USART_USARTNUMBER].USART_CR1 |= ( TX_EN_MASK << USART_TE);
			break;
		default: Local_ErrorState = NOK; break;
		}
	}
	else
	{
		Local_ErrorState =NOK;
	}
	return Local_ErrorState;
}

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
ErrorState_t USART_Tx(const USART_ConfigReg_t* Copy_ConfigReg , uint8_t Copy_TransmitingData)
{
	ErrorState_t Local_ErrorState =OK ;

	while((((USARTx[Copy_ConfigReg->USART_USARTNUMBER].USART_SR)>>USART_TXE)& GETVAL_MASK) != EMPTY_TXDR);

	USARTx[Copy_ConfigReg->USART_USARTNUMBER].USART_DR |= Copy_TransmitingData ;

    while((((USARTx[Copy_ConfigReg->USART_USARTNUMBER].USART_SR)>>USART_TC)&GETVAL_MASK) != TRAS_COMPLETED);

    return Local_ErrorState;
}

/*
 * @fn:	USART_Rx
 * @brief:	receive data with polling
 * @param:	Copy_ConfigReg 		(enum @USART_ConfigReg_t)
 * 						a pointer to structure consists of the USART configurations
 * @param: Copy_ReceivingData   (uint8_t*)
 * @retval: Local_ErrorStatus	(@ErrorState_t)
 * 						USART error status
 * */
ErrorState_t USART_Rx(const USART_ConfigReg_t* Copy_ConfigReg, uint8_t* Copy_ReceivingData)
{
	ErrorState_t Local_ErrorState = OK ;

	if(Copy_ReceivingData != NULL)
	{
		while((((USARTx[Copy_ConfigReg->USART_USARTNUMBER].USART_SR)>>USART_RXNE)&GETVAL_MASK) !=READY_RXDR )
		{

		}
		*Copy_ReceivingData = (uint8_t)USARTx[Copy_ConfigReg->USART_USARTNUMBER].USART_DR ;
	}
	else
	{
		Local_ErrorState = NULL_PTR_ERR;
	}
	return Local_ErrorState  ;
}

				/***************INTERRUPT*************/

/*
 * @fn:	USART_Tx_IT
 * @brief:	transmit data with interrupt notification
 * @param:	Copy_ConfigReg 		(enum @USART_ConfigReg_t)
 * 						a pointer to structure consists of the USART configurations
 * @retval: Copy_ErrorStatus	(@ErrorState_t)
 * 						USART error status
 * */
ErrorState_t USART_Tx_IT(USART_UsartNumber_t Copy_USARTNum, uint8_t Copy_TransmitingData)
{
	ErrorState_t Local_ErrorState = OK ;

	if((Copy_USARTNum >=USART_USART1)&&(Copy_USARTNum<=USART_USART6))
	{
		USARTx[Copy_USARTNum].USART_SR &=~(1<<USART_TC);
		USARTx[Copy_USARTNum].USART_CR1 |=(1<<USART_TXEIE);
		USARTx[Copy_USARTNum].USART_CR1 |=(1<<USART_TCIE);
		Global_TransimittedData[Copy_USARTNum]=Copy_TransmitingData;
	}
	else
	{
		Local_ErrorState = NOK;
	}
	return Local_ErrorState  ;
}

/*
 * @fn:	USART_Rx_IT
 * @brief:	receive data with interrupt notification
 * @param:	Copy_ConfigReg 		(enum @USART_ConfigReg_t)
 * 						a pointer to structure consists of the USART configurations
 * @retval: Copy_ErrorStatus	(@ErrorState_t)
 * 						USART error status
 * */
ErrorState_t USART_Rx_IT(USART_UsartNumber_t Copy_USARTNum, uint16_t* Copy_ReceivingData)
{
	ErrorState_t Local_ErrorState = OK ;
	if(Copy_ReceivingData == NULL)
	{
		Local_ErrorState = NULL_PTR_ERR ;
	}
	else
	{
		if((Copy_USARTNum >=USART_USART1)&&(Copy_USARTNum<=USART_USART6))
		{
			USARTx[Copy_USARTNum].USART_CR1 |=(1<<USART_RXNEIE);
			USARTx[Copy_USARTNum].USART_SR &=~(1<<USART_TC);
			Global_ReceiveData[Copy_USARTNum]=Copy_ReceivingData;
		}
		else
		{
			Local_ErrorState = NOK;
		}
	}
	return Local_ErrorState  ;
}


				/******************DMA****************/

/*
 * @fn:	USART_Tx_DMA
 * @brief:	transmit data with DMA
 * @param:	Copy_ConfigReg 		(enum @USART_ConfigReg_t)
 * 						a pointer to structure consists of the USART configurations
 * @retval: Copy_ErrorStatus	(@ErrorState_t)
 * 						USART error status
 * */
ErrorState_t USART_Tx_DMA(const USART_ConfigReg_t* Copy_ConfigReg)
{
	ErrorState_t Local_ErrorState = OK ;

	if(Copy_ConfigReg == NULL)
	{
		Local_ErrorState = NULL_PTR_ERR ;
	}
	else
	{
		USARTx[Copy_ConfigReg->USART_USARTNUMBER].USART_CR3 |=(1<<USART_DMAT);
	}

	return Local_ErrorState  ;
}

/*
 * @fn:	USART_Rx_DMA
 * @brief:	receive data with DMA
 * @param:	Copy_ConfigReg 		(enum @USART_ConfigReg_t)
 * 						a pointer to structure consists of the USART configurations
 * @retval: Copy_ErrorStatus	(@ErrorState_t)
 * 						USART error status
 * */
ErrorState_t USART_Rx_DMA(const USART_ConfigReg_t* Copy_ConfigReg)
{
	ErrorState_t Local_ErrorState = OK ;

		if(Copy_ConfigReg == NULL)
		{
			Local_ErrorState = NULL_PTR_ERR ;
		}
		else
		{
			USARTx[Copy_ConfigReg->USART_USARTNUMBER].USART_CR3 |=(1<<USART_DMAR);
		}

		return Local_ErrorState  ;
}

/*
 * @fn:	USART_CallBack
 * @brief:	Set the call back function
 * @param:	Copy_USARTNum 		(enum USART_UsartNumber_t)
 * @param:	Copy_FlagID 		(enum USART_FlagID_t)
 * @param:	Copy_CallBackFunc 	pointer void function
 * @retval: Copy_ErrorStatus	(@ErrorState_t)
 * 						USART error status
 * */
ErrorState_t USART_CallBack(USART_UsartNumber_t Copy_USARTNum,USART_FlagID_t Copy_FlagID, void (*Copy_CallBackFunc)(void))
{
	ErrorState_t Local_ErrorStatus = OK ;
	if(Copy_CallBackFunc != NULL)
	{
		G_CallBackFunc[Copy_USARTNum][Copy_FlagID] = Copy_CallBackFunc ;
	}
	else
	{
		Local_ErrorStatus = NULL_PTR_ERR ;
	}
	return Local_ErrorStatus ;
}
void USART_IRQHandler(USART_UsartNumber_t Copy_USARTNum)
{
	uint8_t Local_FlagVal=0;

		/*Reading (Read data register not empty) flag*/
		USART_ReadFlag(Copy_USARTNum , READ_DATA_R_NOT_EMPTY_FLAG, &Local_FlagVal);
		if(Local_FlagVal == 1)
		{

			/*Clear flag*/
			USARTx[Copy_USARTNum].USART_SR &=~ (1<<USART_RXNE);
			/* (Read data register not empty) flag is set*/
			*Global_ReceiveData[Copy_USARTNum] = USARTx[Copy_USARTNum].USART_DR ;

			/*Disable (Read data register not empty) interrupt*/
			USARTx[Copy_USARTNum].USART_CR1 &=~ (1<<USART_RXNEIE);

			/*if callback function is valid call it*/
			if(G_CallBackFunc[Copy_USARTNum][READ_DATA_R_NOT_EMPTY_FLAG] != NULL)
			{
				G_CallBackFunc[Copy_USARTNum][READ_DATA_R_NOT_EMPTY_FLAG]();
			}

		}

		Local_FlagVal=0;
		/*Reading Transmit data register empty Flag*/
		USART_ReadFlag(Copy_USARTNum , TRANSMIT_DATA_R_EMPTY_FLAG, &Local_FlagVal);
		if(Local_FlagVal == 1)
		{
			/* (Transmit data register empty) flag is set*/
			USARTx[Copy_USARTNum].USART_DR |= ( TRANSIMTTED_DATA_MASK & Global_TransimittedData[Copy_USARTNum]);
			/*Disable (Transmit data register empty) interrupt*/
			USARTx[Copy_USARTNum].USART_CR1 &=~ (1<<USART_TXEIE);
			/*if callback function is valid call it*/
			if(G_CallBackFunc[Copy_USARTNum][TRANSMIT_DATA_R_EMPTY_FLAG] != NULL)
			{
				G_CallBackFunc[Copy_USARTNum][TRANSMIT_DATA_R_EMPTY_FLAG]();
			}
		}

		Local_FlagVal=0;
		/*Reading Transmission Complete Flag*/
		USART_ReadFlag(Copy_USARTNum , TRANSMISSION_COMPLETE_FLAG, &Local_FlagVal);
		if(Local_FlagVal == 1)
		{
			/*Clear flag*/
			USARTx[Copy_USARTNum].USART_SR &=~ (1<<USART_TC);
			/* (Transmission Complete Flag) flag is set*/
			/*Disable (Transmission Complete) interrupt*/
			USARTx[Copy_USARTNum].USART_CR1 &=~ (1<<USART_TCIE);
			/*if callback function is valid call it*/
			if(G_CallBackFunc[Copy_USARTNum][TRANSMISSION_COMPLETE_FLAG] != NULL)
			{
				G_CallBackFunc[Copy_USARTNum][TRANSMISSION_COMPLETE_FLAG]();
			}

		}

}

/*USART 1 HANDLER*/
void USART1_IRQHandler(void)
{
	USART_IRQHandler(USART_USART1);
}

/*USART 2 HANDLER*/
void USART2_IRQHandler(void)
{
	USART_IRQHandler(USART_USART2);
}

/*USART 3 HANDLER*/
void USART3_IRQHandler(void)
{
	USART_IRQHandler(USART_USART3);
}

/*USART 4 HANDLER*/
void UART4_IRQHandler(void)
{
	USART_IRQHandler(USART_USART4);
}

/*USART 5 HANDLER*/
void UART5_IRQHandler(void)
{
	USART_IRQHandler(USART_USART5);
}

/*USART 6 HANDLER*/
void USART6_IRQHandler(void)
{
	USART_IRQHandler(USART_USART6);
}

