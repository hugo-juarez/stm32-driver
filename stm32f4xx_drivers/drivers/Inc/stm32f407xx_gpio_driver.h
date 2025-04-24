/*
 * stm32f407xx_gpio_driver.h
 *
 *  Created on: Apr 24, 2025
 *      Author: hugo-juarez
 */

#ifndef INC_STM32F407XX_GPIO_DRIVER_H_
#define INC_STM32F407XX_GPIO_DRIVER_H_

#include "stm32f407xx.h"
#include <stdint.h>

typedef struct
{
	uint8_t GPIOx_PinNumber;
	uint8_t GPIOx_PinMode;
	uint8_t GPIOx_PinSpeed;
	uint8_t GPIOx_PinPuPdControl;
	uint8_t GPIOx_PinOPType;
	uint8_t GPIOx_PinAltFunMode;
} GPIOx_PinConfig_t;


typedef struct
{
	GPIOx_RegDef_t* pGPIOx;
	GPIOx_PinConfig_t GPIOx_PinConfig;

} GPIOx_Handle_t;

/*****************************************************************
 * 							APIs!!!!!!
 *****************************************************************/

// PCLK Control
void GPIOx_PCLKControl(GPIOx_RegDef_t* pGPIOx, uint8_t state);

// Init-DeInit
void GPIOx_Init(GPIOx_Handle_t* pGPIOHandle);
void GPIOx_DeInit(GPIOx_RegDef_t* pGPIOx);

// Read
uint8_t GPIOx_ReadPin(GPIOx_RegDef_t* pGPIOx, uint8_t pin);
uint16_t GPIOx_ReadPort(GPIOx_RegDef_t* pGPIOx);

// Write
void GPIOx_WritePin(GPIOx_RegDef_t* pGPIOx, uint8_t pin, uint8_t val);
void GPIOx_WritePort(GPIOx_RegDef_t* pGPIOx, uint16_t val);
void GPIOx_TogglePin(GPIOx_RegDef_t* pGPIOx, uint8_t pin);

// Interrupt
void GPIOx_IRQConfig(uint8_t IRQNumber, uint8_t IRQPrio, uint8_t state);
void GPIOx_IRQHandling(uint8_t pin);



#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */
