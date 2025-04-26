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


	// GPIO PUSH PULL CONFIG
	GPIOx_Handle_t GPIOLed;

	GPIOLed.pGPIOx = GPIOD;

	GPIOLed.GPIOx_PinConfig.GPIOx_PinNumber = GPIOx_PIN_NO_12; //GreenLed
	GPIOLed.GPIOx_PinConfig.GPIOx_PinMode = GPIOx_MODE_OUT;
	GPIOLed.GPIOx_PinConfig.GPIOx_PinSpeed = GPIOx_OSPEED_FAST;
	GPIOLed.GPIOx_PinConfig.GPIOx_PinOType = GPIOx_OUT_TYPE_PP;
	GPIOLed.GPIOx_PinConfig.GPIOx_PinPuPdControl = GPIOx_PUPDR_NONE;

	// GPIO OPEN DRAIN CONFIG
	GPIOx_Handle_t GPIOLedOD;

	GPIOLedOD.pGPIOx = GPIOD;

	GPIOLedOD.GPIOx_PinConfig.GPIOx_PinNumber = GPIOx_PIN_NO_13; //GreenLed
	GPIOLedOD.GPIOx_PinConfig.GPIOx_PinMode = GPIOx_MODE_OUT;
	GPIOLedOD.GPIOx_PinConfig.GPIOx_PinSpeed = GPIOx_OSPEED_FAST;
	GPIOLedOD.GPIOx_PinConfig.GPIOx_PinOType = GPIOx_OUT_TYPE_OD;
	GPIOLedOD.GPIOx_PinConfig.GPIOx_PinPuPdControl = GPIOx_PUPDR_UP;

	//Clock enable and inits

	GPIOx_PCLKControl(GPIOD, ENABLE);

	GPIOx_Init( &GPIOLed );
	GPIOx_Init( &GPIOLedOD );

	for(;;){
		GPIOx_TogglePin(GPIOD, GPIOx_PIN_NO_12);
		GPIOx_TogglePin(GPIOD, GPIOx_PIN_NO_13);
		sleep();
	}

	return 0;
}

void sleep(void){
	for(int i=0; i<4000000; i++);
}
