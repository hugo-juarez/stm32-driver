/*
 * button_interrupt.c
 *
 *  Created on: Apr 27, 2025
 *      Author: hugo-juarez
 */

#include "stm32f407xx.h"
#include "stm32f407xx_gpio_driver.h"
#include <string.h>

void delay(void);

int main(void){
	// GPIO LED Confg
	GPIOx_Handle_t GPIOLed;
	memset(&GPIOLed, 0, sizeof(GPIOLed)); //Initialize all member values to 0

	GPIOLed.pGPIOx = GPIOD;

	GPIOLed.GPIOx_PinConfig.GPIOx_PinNumber = GPIOx_PIN_NO_12; //GreenLed
	GPIOLed.GPIOx_PinConfig.GPIOx_PinMode = GPIOx_MODE_OUT;
	GPIOLed.GPIOx_PinConfig.GPIOx_PinSpeed = GPIOx_OSPEED_FAST;
	GPIOLed.GPIOx_PinConfig.GPIOx_PinOType = GPIOx_OUT_TYPE_PP;
	GPIOLed.GPIOx_PinConfig.GPIOx_PinPuPdControl = GPIOx_PUPDR_NONE;

	//GPIO Interrupt Button Config
	GPIOx_Handle_t GPIOBtn;
	memset(&GPIOBtn, 0, sizeof(GPIOBtn)); //Initialize all member values to 0

	GPIOBtn.pGPIOx = GPIOA;

	GPIOBtn.GPIOx_PinConfig.GPIOx_PinNumber = GPIOx_PIN_NO_0; //USR PIN
	GPIOBtn.GPIOx_PinConfig.GPIOx_PinMode = GPIOx_MODE_IT_FT;
	GPIOBtn.GPIOx_PinConfig.GPIOx_PinPuPdControl = GPIOx_PUPDR_NONE;


	//	ENABLE CLOCKS
	GPIOx_PCLKControl(GPIOA, ENABLE);
	GPIOx_PCLKControl(GPIOD, ENABLE);

	//	SETTING INIT VALUES
	GPIOx_Init(&GPIOBtn);
	GPIOx_Init(&GPIOLed);

	// Set IRQ Configuration
	GPIOx_IRQInterruptConfig(IRQ_NO_EXTI0, ENABLE);

	for(;;);

	return 0;

}


//ISR FOR EXTI0
void EXTI0_IRQHandler(void){

	delay();
	//Resets Pending Register
	GPIOx_IRQHandling(0);
	GPIOx_TogglePin(GPIOD, GPIOx_PIN_NO_12);
}

//~200ms delay
void delay(void){
	for(uint32_t i=0; i<500000/2; i++);
}
