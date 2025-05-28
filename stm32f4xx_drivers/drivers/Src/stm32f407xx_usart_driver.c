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


