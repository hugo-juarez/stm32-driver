/*
 * stm32f407xx_gpio_driver.c
 *
 *  Created on: Apr 24, 2025
 *      Author: hugo-juarez
 */

#include "stm32f407xx_gpio_driver.h"


/**************** PCLK Control ****************
 *
 * @fn				- GPIOx_PCLKControl
 *
 * @brief			- This functions enables or disables the peripheral clock for the given GPIO
 *
 * @param[in]		- base address of the GPIO peripheral
 * @param[in]		- ENABLE or DISABLE macros
 * @param[in]		-
 *
 * @return			- none
 *
 * @note			- none
 *
 */

void GPIOx_PCLKControl(GPIOx_RegDef_t* pGPIOx, uint8_t state)
{
	if(state == ENABLE)
	{
		if(pGPIOx == GPIOA)
			GPIOA_PCLK_EN();

		else if(pGPIOx == GPIOB)
			GPIOB_PCLK_EN();

		else if(pGPIOx == GPIOC)
			GPIOC_PCLK_EN();

		else if(pGPIOx == GPIOD)
			GPIOD_PCLK_EN();

		else if(pGPIOx == GPIOE)
			GPIOE_PCLK_EN();

		else if(pGPIOx == GPIOF)
			GPIOF_PCLK_EN();

		else if(pGPIOx == GPIOG)
			GPIOG_PCLK_EN();

		else if(pGPIOx == GPIOH)
			GPIOH_PCLK_EN();

		else if(pGPIOx == GPIOI)
			GPIOI_PCLK_EN();

		else if(pGPIOx == GPIOJ)
			GPIOJ_PCLK_EN();

		else if(pGPIOx == GPIOK)
			GPIOK_PCLK_EN();

	} else {

		if(pGPIOx == GPIOA)
			GPIOA_PCLK_DI();

		else if(pGPIOx == GPIOB)
			GPIOB_PCLK_DI();

		else if(pGPIOx == GPIOC)
			GPIOC_PCLK_DI();

		else if(pGPIOx == GPIOD)
			GPIOD_PCLK_DI();

		else if(pGPIOx == GPIOE)
			GPIOE_PCLK_DI();

		else if(pGPIOx == GPIOF)
			GPIOF_PCLK_DI();

		else if(pGPIOx == GPIOG)
			GPIOG_PCLK_DI();

		else if(pGPIOx == GPIOH)
			GPIOH_PCLK_DI();

		else if(pGPIOx == GPIOI)
			GPIOI_PCLK_DI();

		else if(pGPIOx == GPIOJ)
			GPIOJ_PCLK_DI();

		else if(pGPIOx == GPIOK)
			GPIOK_PCLK_DI();
	}
}

/**************** GPIO INIT ****************
 *
 * @fn				- GPIOx_Init
 *
 * @brief			- This functions initilize the registers of a GPIO to a set value by the user
 *
 * @param[in]		- handler with pointer of GPIO base address and values to set the registers
 * @param[in]		-
 * @param[in]		-
 *
 * @return			- none
 *
 * @note			- none
 *
 */

void GPIOx_Init(GPIOx_Handle_t* pGPIOxHandle){

	GPIOx_PCLKControl(pGPIOxHandle->pGPIOx, ENABLE);

	uint8_t pin = pGPIOxHandle->GPIOx_PinConfig.GPIOx_PinNumber;


	//1 . configure the mode of gpio pin
	if(pGPIOxHandle->GPIOx_PinConfig.GPIOx_PinMode <= 3){ //Is not an Interrupt
		pGPIOxHandle->pGPIOx->MODER &= ~(3 << 2*pin);
		pGPIOxHandle->pGPIOx->MODER |= (pGPIOxHandle->GPIOx_PinConfig.GPIOx_PinMode << 2*pin);
	} else{

		// 1. configure risign or falling edge

		if(pGPIOxHandle->GPIOx_PinConfig.GPIOx_PinMode == GPIOx_MODE_IT_FT){
			EXTI->FTSR |= (1 << pin);
			EXTI->RTSR &= ~(1 << pin);
		}else if(pGPIOxHandle->GPIOx_PinConfig.GPIOx_PinMode == GPIOx_MODE_IT_RT){
			EXTI->FTSR &= ~(1 << pin);
			EXTI->RTSR |= (1 << pin);
		} else if(pGPIOxHandle->GPIOx_PinConfig.GPIOx_PinMode == GPIOx_MODE_IT_RFT){
			EXTI->FTSR |= (1 << pin);
			EXTI->RTSR |= (1 << pin);
		}

		// 2. configure GPIO port
		uint8_t temp1 = pin / 4;
		uint8_t temp2 = pin % 4;

		uint8_t port = GPIO_BASEADDR_TO_PORT(pGPIOxHandle->pGPIOx);

		SYSCFG_PCLK_EN();

		SYSCFG->EXTICR[temp1] &= ~(0xF << temp2*4);
		SYSCFG->EXTICR[temp1] |= (port << temp2*4);

		// 3. Unmask EXTIx pin
		EXTI->IMR |= (1 << pin);


	}
	//2 . configure speed
	pGPIOxHandle->pGPIOx->OSPEEDR &= ~(3 << 2*pin);
	pGPIOxHandle->pGPIOx->OSPEEDR |= (pGPIOxHandle->GPIOx_PinConfig.GPIOx_PinSpeed << 2*pin);

	//3 . configure the PU/PD settings
	pGPIOxHandle->pGPIOx->PUPDR &= ~(3 << 2*pin);
	pGPIOxHandle->pGPIOx->PUPDR |= (pGPIOxHandle->GPIOx_PinConfig.GPIOx_PinPuPdControl << 2*pin);

	//4 . configure the output type
	pGPIOxHandle->pGPIOx->OTYPER &= ~(1 << pin);
	pGPIOxHandle->pGPIOx->OTYPER |= (pGPIOxHandle->GPIOx_PinConfig.GPIOx_PinOType << pin);

	//5 . configure the alternate functionality
	if(!(pGPIOxHandle->GPIOx_PinConfig.GPIOx_PinMode == GPIOx_MODE_ALTFN))
		return;

	uint8_t offset, index;

	index = pin / 8;
	offset = 4 * (pin % 8);

	pGPIOxHandle->pGPIOx->AFRx[index] &= ~(0xF << offset);
	pGPIOxHandle->pGPIOx->AFRx[index] |= (pGPIOxHandle->GPIOx_PinConfig.GPIOx_PinAltFunMode << offset);
}

/**************** GPIO DEINIT ****************
 *
 * @fn				- GPIOx_DeInit
 *
 * @brief			- This function resets the default values of the GPIO register
 *
 * @param[in]		- GPIO base address
 * @param[in]		-
 * @param[in]		-
 *
 * @return			- none
 *
 * @note			- none
 *
 */

void GPIOx_DeInit(GPIOx_RegDef_t* pGPIOx){
	if(pGPIOx == GPIOA)
		GPIOA_REG_RESET();

	else if(pGPIOx == GPIOB)
		GPIOB_REG_RESET();

	else if(pGPIOx == GPIOC)
		GPIOC_REG_RESET();

	else if(pGPIOx == GPIOD)
		GPIOD_REG_RESET();

	else if(pGPIOx == GPIOE)
		GPIOE_REG_RESET();

	else if(pGPIOx == GPIOF)
		GPIOF_REG_RESET();

	else if(pGPIOx == GPIOG)
		GPIOG_REG_RESET();

	else if(pGPIOx == GPIOH)
		GPIOH_REG_RESET();

	else if(pGPIOx == GPIOI)
		GPIOI_REG_RESET();

	else if(pGPIOx == GPIOJ)
		GPIOJ_REG_RESET();

	else if(pGPIOx == GPIOK)
		GPIOK_REG_RESET();
}

/**************** READ PIN ****************
 *
 * @fn				- GPIOx_ReadPin
 *
 * @brief			- This function reads the value at the pin
 *
 * @param[in]		- GPIO base address
 * @param[in]		- PIN number
 * @param[in]		-
 *
 * @return			- INPUT value at PINx 1/0
 *
 * @note			- none
 *
 */
uint8_t GPIOx_ReadPin(GPIOx_RegDef_t* pGPIOx, uint8_t pin){
	return (uint8_t) ((pGPIOx->IDR >> pin) & 0x1);
}

/**************** READ PORT ****************
 *
 * @fn				- GPIOx_ReadPort
 *
 * @brief			- This function reads the value at the 15 GPIO port pins
 *
 * @param[in]		- GPIO base address
 * @param[in]		-
 * @param[in]		-
 *
 * @return			- 16 BIT value where each bit represents an input of the port
 *
 * @note			- none
 *
 */
uint16_t GPIOx_ReadPort(GPIOx_RegDef_t* pGPIOx){
	return (uint16_t) pGPIOx->IDR;
}

/**************** WRITE PIN ****************
 *
 * @fn				- GPIOx_WritePin
 *
 * @brief			- This function writes the value at the pin in the selected port
 *
 * @param[in]		- GPIO base address
 * @param[in]		- PIN number
 * @param[in]		- Value
 *
 * @return			- none
 *
 * @note			- none
 *
 */
void GPIOx_WritePin(GPIOx_RegDef_t* pGPIOx, uint8_t pin, uint8_t val){
	if(val == SET){
		pGPIOx->ODR |= (1 << pin);
	}else{
		pGPIOx->ODR &= ~(1 << pin);
	}
}

/**************** WRITE PORT ****************
 *
 * @fn				- GPIOx_WritePort
 *
 * @brief			- This function writes the value at the selected port
 *
 * @param[in]		- GPIO base address
 * @param[in]		- Value
 * @param[in]		-
 *
 * @return			- none
 *
 * @note			- none
 *
 */
void GPIOx_WritePort(GPIOx_RegDef_t* pGPIOx, uint16_t val){
	pGPIOx->ODR &= ~(0xFFFF << 0);
	pGPIOx->ODR |= val;
}

/**************** TOGGLE PIN ****************
 *
 * @fn				- GPIOx_TogglePin
 *
 * @brief			- This function toggles the value at the selected pin
 *
 * @param[in]		- GPIO base address
 * @param[in]		- PIN
 * @param[in]		-
 *
 * @return			- none
 *
 * @note			- none
 *
 */
void GPIOx_TogglePin(GPIOx_RegDef_t* pGPIOx, uint8_t pin){
	pGPIOx->ODR ^= (1 << pin);
}

// Interrupt
void GPIOx_IRQInterruptConfig(uint8_t IRQNumber, uint8_t state){

	uint8_t temp1 = IRQNumber / 32;
	uint8_t temp2 = IRQNumber % 32;

	if(state == ENABLE){
		NVIC->ISER[temp1] |= (1 << temp2);
	} else {
		NVIC->ICER[temp1] |= (1 << temp2);
	}
}

void GPIOx_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority){

	NVIC->IPR[IRQNumber] = (IRQPriority << (8 - NO_PR_BITS_IMPLEMENTED));

}

void GPIOx_IRQHandling(uint8_t pin){

	//Clear PR register on EXTI since it is not done automatically to allow for next interrupt
	if(EXTI->PR & (1 << pin)){
		//clear
		EXTI->PR |= (1<<pin);
	}
}
