/*
 * led_button.c
 *
 *  Created on: Apr 26, 2025
 *      Author: hugo-juarez
 */

#include "stm32f407xx.h"
#include "stm32f407xx_gpio_driver.h"
#include <stdint.h>

int main(void){

	//	BUTTON CONFIG
	GPIOx_Handle_t GPIOButton;

	GPIOButton.pGPIOx = GPIOA;

	GPIOButton.GPIOx_PinConfig.GPIOx_PinNumber = GPIOx_PIN_NO_0;
	GPIOButton.GPIOx_PinConfig.GPIOx_PinMode = GPIOx_MODE_IN;
	GPIOButton.GPIOx_PinConfig.GPIOx_PinPuPdControl = GPIOx_PUPDR_NONE;

	//	LED CONFIG
	GPIOx_Handle_t GPIOLed;

	GPIOLed.pGPIOx = GPIOD;

	GPIOLed.GPIOx_PinConfig.GPIOx_PinNumber = GPIOx_PIN_NO_12;
	GPIOLed.GPIOx_PinConfig.GPIOx_PinMode = GPIOx_MODE_OUT;
	GPIOLed.GPIOx_PinConfig.GPIOx_PinOType = GPIOx_OUT_TYPE_PP;
	GPIOLed.GPIOx_PinConfig.GPIOx_PinPuPdControl = GPIOx_PUPDR_NONE;
	GPIOLed.GPIOx_PinConfig.GPIOx_PinSpeed = GPIOx_OSPEED_FAST;

	//	ENABLE CLOCKS
	GPIOx_PCLKControl(GPIOA, ENABLE);
	GPIOx_PCLKControl(GPIOD, ENABLE);

	//	SETTING INIT VALUES
	GPIOx_Init(&GPIOButton);
	GPIOx_Init(&GPIOLed);

	for(;;){
		uint8_t btn = GPIOx_ReadPin( GPIOA, GPIOx_PIN_NO_0);
		GPIOx_WritePin( GPIOD, GPIOx_PIN_NO_12, btn);
	}

	return 0;
}
