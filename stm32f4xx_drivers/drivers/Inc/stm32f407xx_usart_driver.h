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
	uint8_t 	USART_Mode;
	uint32_t 	USART_Baud;
	uint8_t 	USART_StopBits;
	uint8_t 	USART_WordLength;
	uint8_t 	USART_Parity;
	uint8_t 	USART_HWFlowControl;
} USARTx_Config_t;

// --- Handle Structure ---
typedef struct {
	USARTx_RegDef_t 	*pUSARTx;
	USARTx_Config_t 	USART_Config;
	uint8_t				TxBusyState;
	uint32_t			TxLen;
	uint8_t				*pTxBuffer;
	uint8_t				RxBusyState;
	uint32_t			RxLen;
	uint8_t				*pRxBuffer;
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
 *              	Flags
 ******************************************/

// --- SR Flags ---
#define USART_FLAG_TXE					(1 << USART_SR_TXE)
#define USART_FLAG_RXNE					(1 << USART_SR_RXNE)
#define USART_FLAG_TC					(1 << USART_SR_TC)
#define USART_FLAG_CTS					(1 << USART_SR_CTS)
#define USART_FLAG_IDLE					(1 << USART_SR_IDLE)
#define USART_FLAG_ORE					(1 << USART_SR_ORE)
#define USART_FLAG_FE					(1 << USART_SR_FE)
#define USART_FLAG_NF					(1 << USART_SR_NF)

// --- CR1 Flags ---
#define USART_FLAG_TXEIE				(1 << USART_CR1_TXEIE)
#define USART_FLAG_RXNEIE				(1 << USART_CR1_RXNEIE)
#define USART_FLAG_TCIE					(1 << USART_CR1_TCIE)
#define USART_FLAG_IDLEIE				(1 << USART_CR1_IDLEIE)
#define USART_FLAG_PEIE					(1 << USART_CR1_PEIE)

// --- CR2 Flags ---
#define USART_FLAG_LBDIE				(1 << USART_CR2_LBDIE)

// --- CR3 Flags ---
#define USART_FLAG_CTSIE				(1 << USART_CR3_CTSIE)
#define USART_FLAG_EIE					(1 << USART_CR3_EIE)

// --- Interrupt Flags ---
#define USART_FREE						0
#define USART_BUSY_IN_TX				1
#define USART_BUSY_IN_RX				1

// --- Application Events ---
#define USART_EV_TX_CMPLT				1
#define USART_EV_RX_CMPLT				2
#define USART_EV_CTS					3
#define USART_EV_IDLE					4
#define USART_EV_ORE					5

// --- Application Errors ---
#define USART_ER_FE						6
#define USART_ER_NF						7
#define USART_ER_ORE					8

/******************************************
 *              	APIS
 ******************************************/

// --- Peripheral  ---
void USART_PCLKControl(USARTx_RegDef_t *pUSARTx, uint8_t state);
void USART_PeripheralCtrl(USARTx_RegDef_t *pUSARTx, uint8_t state);

// --- Init and De-init
void USART_Init(USARTx_Handle_t *pUSARTHandle);
void USART_DeInit(USARTx_RegDef_t *pUSARTx);
void USART_SetBaudRate(USARTx_RegDef_t *pUSARTx, uint32_t baudRate);

// --- Data Send And Receive ---
void USART_SendData(USARTx_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t len);
void USART_ReceiveData(USARTx_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t len);
uint8_t USART_SendDataIT(USARTx_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t len);
uint8_t USART_ReceiveDataIT(USARTx_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t len);

// --- Flags ---
uint8_t USART_GetFlagStatus(USARTx_RegDef_t *pUSARTx, uint32_t flag);
void USART_ClearFlag(USARTx_RegDef_t *pUSARTx, uint32_t flag);

// --- Interrupt Configuration And Handling ---
void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t state);
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void USART_IRQHandling(USARTx_Handle_t *pUSARTHandle);

// --- Interrupt Communication Closing ---
void USART_CloseSendData(USARTx_Handle_t *pUSARTHandle);
void USART_CloseReceiveData(USARTx_Handle_t *pUSARTHandle);

// --- Application Callback ---
void USART_ApplicationEventCallback(USARTx_Handle_t *pUSARTHandle, uint8_t event);

// --- Other APIs ---
void USART_ClearErrorFlag(USARTx_RegDef_t *pUSARTx);

#endif /* INC_STM32F407XX_USART_DRIVER_H_ */
