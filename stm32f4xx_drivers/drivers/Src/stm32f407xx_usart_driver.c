/*
 * stm32f407xx_usart_driver.c
 *
 *  Created on: May 26, 2025
 *      Author: hugo-juarez
 */

#include "stm32f407xx_usart_driver.h"

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

	} else{

	}
}


/******************************************
 *              Flags
 ******************************************/

// --- Get Flag ---
uint8_t USART_GetFlagStatus(USARTx_RegDef_t *pUSARTx, uint8_t flag){
	if(pUSARTx->SR & flag){
		return SET;
	} else {
		return RESET;
	}
}


// --- Clear Flag ---
void USART_ClearFlag(USARTx_RegDef_t *pUSARTx, uint8_t flag){
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


