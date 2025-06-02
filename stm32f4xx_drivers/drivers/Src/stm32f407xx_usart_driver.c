/*
 * stm32f407xx_usart_driver.c
 *
 *  Created on: May 26, 2025
 *      Author: hugo-juarez
 */

#include "stm32f407xx_usart_driver.h"
#include "stm32f407xx_rcc_driver.h"


/******************************************
 *              Peripherals
 ******************************************/

// --- Peripheral Clock ---
void USART_PCLKControl(USARTx_RegDef_t *pUSARTx, uint8_t state){
	if(state == ENABLE){
		if(pUSARTx == USART1){
			USART1_PCLK_EN();
		} else if(pUSARTx == USART2){
			USART2_PCLK_EN();
		} else if(pUSARTx == USART3){
			USART3_PCLK_EN();
		} else if(pUSARTx == UART4){
			UART4_PCLK_EN();
		} else if(pUSARTx == UART5){
			UART5_PCLK_EN();
		} else if(pUSARTx == USART6){
			USART6_PCLK_EN();
		}
	} else {
		if(pUSARTx == USART1){
			USART1_PCLK_DI();
		} else if(pUSARTx == USART2){
			USART2_PCLK_DI();
		} else if(pUSARTx == USART3){
			USART3_PCLK_DI();
		} else if(pUSARTx == UART4){
			UART4_PCLK_DI();
		} else if(pUSARTx == UART5){
			UART5_PCLK_DI();
		} else if(pUSARTx == USART6){
			USART6_PCLK_DI();
		}
	}
}

// --- Peripheral EN/DI ---
void USART_PeripheralCtrl(USARTx_RegDef_t *pUSARTx, uint8_t state){
	if(state == ENABLE){
		pUSARTx->CR1 |= (1 << USART_CR1_UE);
	} else{
		pUSARTx->CR1 &= ~(1 << USART_CR1_UE);
	}
}


/******************************************
 *            Init/De-Init
 ******************************************/

// --- Init ---
void USART_Init(USARTx_Handle_t *pUSARTHandle){
	uint32_t tempreg = 0;

	//Enable Peripheral Clock
	USART_PCLKControl(pUSARTHandle->pUSARTx, ENABLE);

	//************ CR1 Config ************

	//USART Mode
	if(pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_TX){
		tempreg |= (1 << USART_CR1_TE);
	} else if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_RX){
		tempreg |= (1 << USART_CR1_RE);
	} else if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_TXRX){
		tempreg |= ((1 << USART_CR1_TE) | (1 << USART_CR1_RE));
	}

	//USART WordLength
	tempreg |= (pUSARTHandle->USART_Config.USART_WordLength << USART_CR1_M);

	//USART Parity
	if(pUSARTHandle->USART_Config.USART_Parity == USART_PARITY_EVEN){
		tempreg |= (1 << USART_CR1_PCE);
	} else if (pUSARTHandle->USART_Config.USART_Parity == USART_PARITY_ODD){
		tempreg |= (1 << USART_CR1_PCE);
		tempreg |= (1 << USART_CR1_PS);
	}

	//Load Values to CR1
	pUSARTHandle->pUSARTx->CR1 = tempreg;

	//************ CR2 Config ************

	tempreg = 0;

	//USART StopBits
	tempreg |= (pUSARTHandle->USART_Config.USART_StopBits << USART_CR2_STOP);

	//Load Values to CR2
	pUSARTHandle->pUSARTx->CR2 = tempreg;

	//************ CR3 Config ************

	tempreg = 0;

	//USART HWFlowControl
	if(pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS){
		tempreg |= (1 << USART_CR3_CTSE);
	} else if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_RTS){
		tempreg |= (1 << USART_CR3_RTSE);
	} else if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS_RTS){
		tempreg |= ((1 << USART_CR3_CTSE) | (1 << USART_CR3_RTSE));
	}

	//Load Values to CR3
	pUSARTHandle->pUSARTx->CR3 = tempreg;

	//************ Baud Rate Config ************
	USART_SetBaudRate(pUSARTHandle->pUSARTx, pUSARTHandle->USART_Config.USART_Baud);

}

// --- De-Init ---
void USART_DeInit(USARTx_RegDef_t *pUSARTx){
	if(pUSARTx == USART1){
		USART1_REG_RESET();
	} else if(pUSARTx == USART2){
		USART2_REG_RESET();
	} else if(pUSARTx == USART3){
		USART3_REG_RESET();
	} else if(pUSARTx == UART4){
		UART4_REG_RESET();
	} else if(pUSARTx == UART5){
		UART5_REG_RESET();
	} else if(pUSARTx == USART6){
		USART6_REG_RESET();
	}
}

// --- Baud Rate Configuration ---
void USART_SetBaudRate(USARTx_RegDef_t *pUSARTx, uint32_t baudRate){
	//Temporal register to load into baud rate config
	uint32_t tempreg = 0;

	//Get the peripheral clock
	uint32_t PCLKx;

	if(pUSARTx == USART1 || pUSARTx == USART6){
		//This are located in APB2
		PCLKx = RCC_GetPCLK2Value();
	} else {
		//The rest of USART are in APB1
		PCLKx = RCC_GetPCLK1Value();
	}

	//Getx USARTDIV value depending on oversampling
	uint32_t usartdiv;
	if(pUSARTx->CR1 & (1 << USART_CR1_OVER8)){
		//If oversampling is 8
		usartdiv = ((25 * PCLKx) / (2 * baudRate)) ;
	} else {
		//Oversampling is 16
		usartdiv = ((25 * PCLKx) / (4 * baudRate)) ;
	}

	//Calculate Mantissa
	uint32_t M_part = usartdiv/100;
	tempreg |= M_part << 4;

	//Get fraction part
	uint32_t F_part = (usartdiv - (M_part * 100));

	if(pUSARTx->CR1 & (1 << USART_CR1_OVER8)){
		//Oversampling = 8
		F_part = ((F_part * 8 + 50)/100) & 0x7;
	} else {
		//Oversampling = 16
		F_part = ((F_part * 16 + 50)/100) & 0xF;
	}

	tempreg |= F_part;

	//Load into BRR register
	pUSARTx->BRR = tempreg;

}

/******************************************
 *     Send And Receive Data (Polling)
 ******************************************/

// --- Send Data ---
void USART_SendData(USARTx_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t len){

	for(uint32_t i=0; i<len; i++){

		// Wait till TXE
		while(!USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_TXE));

		if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS){
			//******** 9-BIT Communication ********

			//On a 9 bit communication the DR gets loaded 2 bytes but mask to only 9 bits
			uint16_t *pData = (uint16_t *)pTxBuffer;
			pUSARTHandle->pUSARTx->DR = (*pData & 0x01FF);

			if(pUSARTHandle->USART_Config.USART_Parity == USART_PARITY_DISABLE){
				//No parity is used therefore we are sending 9 bytes so we are sending more than 1 byte
				pTxBuffer++;
			}

			pTxBuffer++;


		} else {
			//******** 8-BIT Communication ********

			//8 bit data transfer
			pUSARTHandle->pUSARTx->DR = *pTxBuffer;

			//No need to check for parity bit since we either load 7 bits or the 8 gets overwritten

			//Increase buffer address
			pTxBuffer++;
		}

	}

	//Wait till communication finishes
	while(!USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_TC));

}

// --- Receive Data ---
void USART_ReceiveData(USARTx_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t len){
	for(uint32_t i = 0; i < len; i++){

		//Wait till data is received RXNE
		while(!USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_RXNE));

		if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS){
			//******** 9-BIT Communication ********

			//If using parity bit we have 8 bits of data if not we have 9 bits
			if(pUSARTHandle->USART_Config.USART_Parity == USART_PARITY_DISABLE){

				//If we are on 9bit and we are not using a parity bit then the 9 bits are data
				*((uint16_t*) pRxBuffer) = (pUSARTHandle->pUSARTx->DR & 0x01FF);

				pRxBuffer++;
				pRxBuffer++;

			}else{

				//If we are using parity bit with 9 bits only 8 are data
				*pRxBuffer = (pUSARTHandle->pUSARTx->DR & 0xFF);
				pRxBuffer++;
			}


		} else {
			//******** 8-BIT Communication ********
			//If using parity bit we have 7 bits of data if not we have 8 bits
			if(pUSARTHandle->USART_Config.USART_Parity == USART_PARITY_DISABLE){
				//If we are on 8bit and we are not using a parity bit then the 8 bits are data
				*pRxBuffer = (pUSARTHandle->pUSARTx->DR & 0xFF);
			}else{
				//If we are using parity bit with 8 bits only 7 are data
				*pRxBuffer = (pUSARTHandle->pUSARTx->DR & 0x7F);
			}

			//Increment buffer
			pRxBuffer++;

		}
	}
}


/******************************************
 *    Send And Receive Data (Interrupt)
 ******************************************/

// --- Send Data ---
uint8_t USART_SendDataIT(USARTx_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t len){

	uint8_t txstate = pUSARTHandle->TxBusyState;

	if(txstate != USART_BUSY_IN_TX){
		pUSARTHandle->TxBusyState = USART_BUSY_IN_TX;
		pUSARTHandle->TxLen = len;
		pUSARTHandle->pTxBuffer = pTxBuffer;

		//Enable TXE intterupt
		pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_TXEIE);

		//Enable TC interrupt
		pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_TCIE);
	}

	return txstate;
}

// --- Receive Data ---
uint8_t USART_ReceiveDataIT(USARTx_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t len){

	uint8_t rxstate = pUSARTHandle->RxBusyState;

	if(rxstate != USART_BUSY_IN_RX){
		pUSARTHandle->RxBusyState = USART_BUSY_IN_RX;
		pUSARTHandle->RxLen = len;
		pUSARTHandle->pRxBuffer = pRxBuffer;

		//Enable RXNE intterupt
		pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_RXNEIE);
	}
	return rxstate;
}


/******************************************
 *              Flags
 ******************************************/

// --- Get Flag ---
uint8_t USART_GetFlagStatus(USARTx_RegDef_t *pUSARTx, uint32_t flag){
	if(pUSARTx->SR & flag){
		return SET;
	} else {
		return RESET;
	}
}


// --- Clear Flag ---
void USART_ClearFlag(USARTx_RegDef_t *pUSARTx, uint32_t flag){
	pUSARTx->SR &= ~flag;
}


/******************************************
 * 		  Interrupt Configuration
 ******************************************/

// --- EN/DI Interrupt ---
void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t state){
	uint8_t temp1 = IRQNumber / 32;
	uint8_t temp2 = IRQNumber % 32;

	if(state == ENABLE){
		NVIC->ISER[temp1] = (1 << temp2);
	} else {
		NVIC->ICER[temp1] = (1 << temp2);
	}
}


// --- Priority Interrupt ---
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority){
	NVIC->IPR[IRQNumber] = (IRQPriority << (8 - NO_PR_BITS_IMPLEMENTED));
}

/******************************************
 *          Interrupt Handle
 ******************************************/

// --- Interrupt Handlers Prototype definition ---
static void USART_HandleTXEInterrupt(USARTx_Handle_t *pUSARTHandle);
static void USART_HandleTCInterrupt(USARTx_Handle_t *pUSARTHandle);
static void USART_HandleRXNEInterrupt(USARTx_Handle_t *pUSARTHandle);

// --- Interrupt Handling ---
void USART_IRQHandling(USARTx_Handle_t *pUSARTHandle){

	//******** TXE Interrupt ********
	uint8_t temp1 = USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_TXE);
	uint32_t temp2 = pUSARTHandle->pUSARTx->CR1 & USART_FLAG_TXEIE;
	uint8_t temp3 = pUSARTHandle->TxBusyState;

	if( temp1 && temp2 && temp3 ){
		USART_HandleTXEInterrupt(pUSARTHandle);
	}

	//******** TC Interrupt ********
	temp1 = USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_TC);
	temp2 = pUSARTHandle->pUSARTx->CR1 & USART_FLAG_TCIE;
	temp3 = pUSARTHandle->TxBusyState;

	if( temp1 && temp2 &&temp3 ){
		USART_HandleTCInterrupt(pUSARTHandle);
	}

	//******** RXNE Interrupt ********
	temp1 = USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_RXNE);
	temp2 = pUSARTHandle->pUSARTx->CR1 & USART_FLAG_RXNEIE;
	temp3 = pUSARTHandle->RxBusyState;

	if( temp1 && temp2 && temp3 ){
		USART_HandleRXNEInterrupt(pUSARTHandle);
	}

	//******** CTS Interrupt ********
	temp1 = USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_CTS);
	temp2 = pUSARTHandle->pUSARTx->CR3 & USART_FLAG_CTSIE;

	if(temp1 && temp2){
		//Clear CTS Flag
		USART_ClearFlag(pUSARTHandle->pUSARTx, USART_FLAG_CTS);

		//Callback Event
		USART_ApplicationEventCallback(pUSARTHandle, USART_EV_CTS);
	}

	//******** IDLE Interrupt ********
	temp1 = USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_IDLE);
	temp2 = pUSARTHandle->pUSARTx->CR1 & USART_FLAG_IDLEIE;

	if(temp1 && temp2){
		//Clear CTS Flag
		USART_ClearFlag(pUSARTHandle->pUSARTx, USART_FLAG_IDLE);

		//Callback Event
		USART_ApplicationEventCallback(pUSARTHandle, USART_EV_IDLE);
	}

	//******** ORE Interrupt ********
	temp1 = USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_ORE);
	temp2 = pUSARTHandle->pUSARTx->CR1 & USART_FLAG_RXNEIE;

	if(temp1 && temp2){

		//Cleaning of this event should happen from App Callback

		//Callback Event
		USART_ApplicationEventCallback(pUSARTHandle, USART_EV_ORE);
	}

	//******** Error Interrupt ********
	temp1 = pUSARTHandle->pUSARTx->CR3 & USART_FLAG_EIE;

	if(temp1){

		// Framing Error
		temp2 = USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_FE);

		if(temp2){
			USART_ApplicationEventCallback(pUSARTHandle, USART_ER_FE);
		}

		// Noise Detect
		temp2 = USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_NF);

		if(temp2){
			USART_ApplicationEventCallback(pUSARTHandle, USART_ER_NF);
		}

		//Overrun Error
		temp2 = USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_ORE);

		if(temp2){
			USART_ApplicationEventCallback(pUSARTHandle, USART_ER_ORE);
		}
	}

}

// --- USART TXE Interrput Handler ---
void USART_HandleTXEInterrupt(USARTx_Handle_t *pUSARTHandle){

	//Check If USART is on transmission mode

	//If there is nothing to transmit don't do nothing
	if(pUSARTHandle->TxLen <=0){
		return;
	}

	if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS){
		//******** 9-BIT Communication ********

		//On a 9 bit communication the DR gets loaded 2 bytes but mask to only 9 bits
		uint16_t *pData = (uint16_t *) pUSARTHandle->pTxBuffer;
		pUSARTHandle->pUSARTx->DR = (*pData & 0x01FF);

		if(pUSARTHandle->USART_Config.USART_Parity == USART_PARITY_DISABLE){
			//No parity is used therefore we are sending 9 bytes so we are sending more than 1 byte
			pUSARTHandle->pTxBuffer++;
		}

		pUSARTHandle->pTxBuffer++;


	} else {
		//******** 8-BIT Communication ********

		//8 bit data transfer
		pUSARTHandle->pUSARTx->DR = *pUSARTHandle->pTxBuffer;

		//No need to check for parity bit since we either load 7 bits or the 8 gets overwritten

		//Increase buffer address
		pUSARTHandle->pTxBuffer++;
	}

	//Decrement TxLen Count
	pUSARTHandle->TxLen--;


}

// --- USART TC Interrupt Handler ---
void USART_HandleTCInterrupt(USARTx_Handle_t *pUSARTHandle){

	//If there are still data to transmit do nothing
	if(pUSARTHandle->TxLen > 0){
		return;
	}

	//Data transmited entirely therefore tx was completed

	//Close Send Communication
	USART_CloseSendData(pUSARTHandle);

	//Callback TX Completed
	USART_ApplicationEventCallback(pUSARTHandle, USART_EV_TX_CMPLT);
}

// --- USART RXE Interrupt Handler ---
void USART_HandleRXNEInterrupt(USARTx_Handle_t *pUSARTHandle){

	//We should only read when len > 0
	if(pUSARTHandle->RxLen <=0){
		return;
	}

	if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS){
		//******** 9-BIT Communication ********

		//If using parity bit we have 8 bits of data if not we have 9 bits
		if(pUSARTHandle->USART_Config.USART_Parity == USART_PARITY_DISABLE){

			//If we are on 9bit and we are not using a parity bit then the 9 bits are data
			*((uint16_t*) pUSARTHandle->pRxBuffer) = (pUSARTHandle->pUSARTx->DR & 0x01FF);

			pUSARTHandle->pRxBuffer++;
			pUSARTHandle->pRxBuffer++;

		}else{

			//If we are using parity bit with 9 bits only 8 are data
			*(pUSARTHandle->pRxBuffer) = (pUSARTHandle->pUSARTx->DR & 0xFF);
			pUSARTHandle->pRxBuffer++;
		}


	} else {
		//******** 8-BIT Communication ********
		//If using parity bit we have 7 bits of data if not we have 8 bits
		if(pUSARTHandle->USART_Config.USART_Parity == USART_PARITY_DISABLE){
			//If we are on 8bit and we are not using a parity bit then the 8 bits are data
			*(pUSARTHandle->pRxBuffer) = (pUSARTHandle->pUSARTx->DR & 0xFF);
		}else{
			//If we are using parity bit with 8 bits only 7 are data
			*(pUSARTHandle->pRxBuffer) = (pUSARTHandle->pUSARTx->DR & 0x7F);
		}

		//Increment buffer
		pUSARTHandle->pRxBuffer++;
	}

	pUSARTHandle->RxLen--;

	//Reception Done
	if(pUSARTHandle->RxLen == 0){

		//Close Receive Communication
		USART_CloseReceiveData(pUSARTHandle);

		//Send Application Callback
		USART_ApplicationEventCallback(pUSARTHandle, USART_EV_RX_CMPLT);
	}
}

/******************************************
 *      Close Communication Handlers
 ******************************************/

// --- Close Send Data ---
void USART_CloseSendData(USARTx_Handle_t *pUSARTHandle){
	pUSARTHandle->TxBusyState = USART_FREE;
	pUSARTHandle->TxLen = 0;
	pUSARTHandle->pTxBuffer = NULL;

	//Disable TXE intterupt
	pUSARTHandle->pUSARTx->CR1 &= ~(1 << USART_CR1_TXEIE);

	//Disable TC interrupt
	pUSARTHandle->pUSARTx->CR1 &= ~(1 << USART_CR1_TCIE);
}

// --- Close Receive Data ---
void USART_CloseReceiveData(USARTx_Handle_t *pUSARTHandle){
	pUSARTHandle->RxBusyState = USART_FREE;
	pUSARTHandle->RxLen = 0;
	pUSARTHandle->pRxBuffer = NULL;

	//Enable RXNE intterupt
	pUSARTHandle->pUSARTx->CR1 &= ~(1 << USART_CR1_RXNEIE);
}

/******************************************
 * 		  	   Other APIs
 ******************************************/

void USART_ClearErrorFlag(USARTx_RegDef_t *pUSARTx){
	uint32_t temp;

	//Read from SR
	temp = pUSARTx->SR;

	//Read from DR
	temp = pUSARTx->DR;

	(void) temp;
}


/******************************************
 * 		  Application Callback
 ******************************************/

__weak void USART_ApplicationEventCallback(USARTx_Handle_t *pUSARTHandle, uint8_t event){

}

