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


#endif /* INC_STM32F407XX_USART_DRIVER_H_ */
