/*
 * sm32f407xx_usart_driver.h
 *
 *  Created on: May 26, 2025
 *      Author: hugo-juarez
 */

#ifndef INC_STM32F407XX_USART_DRIVER_H_
#define INC_STM32F407XX_USART_DRIVER_H_

#include "stm32f407xx.h"

/******************************************
 *              CONFIGURATION
 ******************************************/

// --- Configuration Structure ---

typedef struct {
	uint8_t USART_Mode;
	uint32_t USART_Baud;
	uint8_t USART_StopBits;
	uint8_t USART_Length;
	uint8_t USART_Parity;
	uint8_t USART_HWFlowControl;
} USARTx_Config_t;

// --- Handle Structure ---

typedef struct {
	USARTx_RegDef_t *pUSARTx;
	USARTx_Config_t USART_Config;
} USART_Handle_t;

/******************************************
 *              	APIS
 ******************************************/

// --- Peripheral  ---
void USART_PCLKControl(USARTx_RegDef_t *pUSARTx, uint8_t state);
void USART_PeripheralCtrl(USARTx_RegDef_t *pUSARTx, uint8_t state);

// --- Flags ---
uint8_t USART_GetFlagStatus(USARTx_RegDef_t *pUSARTx, uint8_t flag);
void USART_ClearFlag(USARTx_RegDef_t *pUSARTx, uint8_t flag);

// --- Interrupt Configuration ---
void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t state);
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);

#endif /* INC_STM32F407XX_USART_DRIVER_H_ */
