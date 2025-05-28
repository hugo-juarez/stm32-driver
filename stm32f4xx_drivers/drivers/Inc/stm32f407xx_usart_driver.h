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
	uint8_t USART_WordLength;
	uint8_t USART_Parity;
	uint8_t USART_HWFlowControl;
} USARTx_Config_t;

// --- Handle Structure ---
typedef struct {
	USARTx_RegDef_t *pUSARTx;
	USARTx_Config_t USART_Config;
} USARTx_Handle_t;

// --- @USART_Mode ---
#define USART_MODE_ONLY_TX				0
#define USART_MODE_ONLY_RX				1
#define USART_MODE_TXRX					2

// --- @USART_Baud ---
#define USART_STD_BAUD_1200				1200
#define USART_STD_BAUD_2400				2400
#define USART_STD_BAUD_9600				9600
#define USART_STD_BAUD_19200			19200
#define USART_STD_BAUD_38400			38400
#define USART_STD_BAUD_57600			57600
#define USART_STD_BAUD_115200			115200
#define USART_STD_BAUD_230400			230400
#define USART_STD_BAUD_460800			460800
#define USART_STD_BAUD_921600			921600
#define USART_STD_BAUD_2M				2000000
#define USART_STD_BAUD_3M				3000000

// --- @USART_StopBits ---
#define USART_STOPBITS_1				0
#define USART_STOPBITS_0_5				1
#define USART_STOPBITS_2				2
#define USART_STOPBITS_1_5				3

// --- @USART_WordLength ---
#define USART_WORDLEN_8BITS				0
#define USART_WORDLEN_9BITS				1

// --- @USART_Parity ---
#define USART_PARITY_DISABLE			0
#define USART_PARITY_EVEN				1
#define USART_PARITY_ODD				2

// --- @USART_HWFlowControl ---
#define USART_HW_FLOW_CTRL_NONE			0
#define USART_HW_FLOW_CTRL_CTS			1
#define USART_HW_FLOW_CTRL_RTS			2
#define USART_HW_FLOW_CTRL_CTS_RTS		3


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
