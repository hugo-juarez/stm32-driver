/*
 * led_toggle.c
 *
 *  Created on: Apr 24, 2025
 *      Author: hugo-juarez
 */

#include "stm32f407xx.h"
#include "stm32f407xx_gpio_driver.h"

void sleep(void);

int main(void){

	GPIOx_Handle_t GPIOLed;

	GPIOLed.pGPIOx = GPIOD;

	GPIOLed.GPIOx_PinConfig.GPIOx_PinNumber = GPIOx_PIN_NO_12; //GreenLed
	GPIOLed.GPIOx_PinConfig.GPIOx_PinMode = GPIOx_MODE_OUT;
	GPIOLed.GPIOx_PinConfig.GPIOx_PinSpeed = GPIOx_OSPEED_FAST;
	GPIOLed.GPIOx_PinConfig.GPIOx_PinOType = GPIOx_OUT_TYPE_PP;
	GPIOLed.GPIOx_PinConfig.GPIOx_PinPuPdControl = GPIOx_PUPDR_NONE;

	GPIOx_PCLKControl(GPIOD, ENABLE);

	GPIOx_Init( &GPIOLed );

	for(;;){
		GPIOx_TogglePin(GPIOD, GPIOx_PIN_NO_12);
		sleep();
	}

	return 0;
}

void sleep(void){
	for(int i=0; i<4000000; i++);
}
