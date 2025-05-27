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
} USARTx_Handle_t;

/******************************************
 *              	APIS
 ******************************************/

// --- Peripheral  ---
void USART_PCLKControl(USARTx_RegDef_t *pUSARTx, uint8_t state);
void USART_PeripheralCtrl(USARTx_RegDef_t *pUSARTx, uint8_t state);

// --- Init and De-init
void USART_Init(USARTx_Handle_t *pUSARTHandle);
void USART_DeInit(USARTx_RegDef_t *pUSARTx);

// --- Data Send And Receive ---
void USART_SendData(USARTx_RegDef_t *pUSARTx, uint8_t *pTxBuffer, uint32_t len);
void USART_ReceiveData(USARTx_RegDef_t *pUSARTx, uint8_t *pRxBuffer, uint32_t len);
void USART_SendDataIT(USARTx_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t len);
void USART_ReceiveDataIT(USARTx_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t len);

// --- Flags ---
uint8_t USART_GetFlagStatus(USARTx_RegDef_t *pUSARTx, uint8_t flag);
void USART_ClearFlag(USARTx_RegDef_t *pUSARTx, uint8_t flag);

// --- Interrupt Configuration And Handling ---
void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t state);
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void USART_IRQHandling(USARTx_Handle_t *pUSARTHandle);

// --- Application Callback
void USART_ApplicationEventCallback(USARTx_Handle_t *pUSARTHandle, uint8_t event);

#endif /* INC_STM32F407XX_USART_DRIVER_H_ */
