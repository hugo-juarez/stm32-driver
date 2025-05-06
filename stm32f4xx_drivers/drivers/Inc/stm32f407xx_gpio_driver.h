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
	uint8_t GPIOx_PinNumber;			//Possible values @GPIOx_PIN_NO
	uint8_t GPIOx_PinMode;				//Possible values @GPIOx_MODES
	uint8_t GPIOx_PinSpeed;				//Possible values @GPIO_OSPEED
	uint8_t GPIOx_PinPuPdControl;		//Possible values @GPIO_PULL_UP_DOWN_REG
	uint8_t GPIOx_PinOType;			//Possible values @GPIO_OUTPUT_TYPES
	uint8_t GPIOx_PinAltFunMode;
} GPIOx_PinConfig_t;


typedef struct
{
	GPIOx_RegDef_t* pGPIOx;
	GPIOx_PinConfig_t GPIOx_PinConfig;

} GPIOx_Handle_t;

//
#define GPIOx_PIN_NO_0			0
#define GPIOx_PIN_NO_1			1
#define GPIOx_PIN_NO_2			2
#define GPIOx_PIN_NO_3			3
#define GPIOx_PIN_NO_4			4
#define GPIOx_PIN_NO_5			5
#define GPIOx_PIN_NO_6			6
#define GPIOx_PIN_NO_7			7
#define GPIOx_PIN_NO_8			8
#define GPIOx_PIN_NO_9			9
#define GPIOx_PIN_NO_10			10
#define GPIOx_PIN_NO_11			11
#define GPIOx_PIN_NO_12			12
#define GPIOx_PIN_NO_13			13
#define GPIOx_PIN_NO_14			14
#define GPIOx_PIN_NO_15			15

// GPIOx_MODES
#define GPIOx_MODE_IN		 	0
#define GPIOx_MODE_OUT			1
#define GPIOx_MODE_ALTFN		2
#define GPIOx_MODE_ANALOG		3
#define GPIOx_MODE_IT_FT		4
#define GPIOx_MODE_IT_RT		5
#define GPIOx_MODE_IT_RFT		6

// GPIO_OUTPUT_TYPES
#define GPIOx_OUT_TYPE_PP		0
#define GPIOx_OUT_TYPE_OD		1

// GPIO_OSPEED
#define GPIOx_OSPEED_LOW		0
#define GPIOx_OSPEED_MID		1
#define GPIOx_OSPEED_FAST		2
#define GPIOx_OSPEED_HIGH		3

// GPIO_PULL_UP_DOWN_REG
#define GPIOx_PUPDR_NONE		0
#define GPIOx_PUPDR_UP			1
#define GPIOx_PUPDR_DOWN		2

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
void GPIOx_IRQInterruptConfig(uint8_t IRQNumber, uint8_t state);
void GPIOx_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void GPIOx_IRQHandling(uint8_t pin);



#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */
