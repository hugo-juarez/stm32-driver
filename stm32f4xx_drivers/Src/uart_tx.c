/*
 * uart_tx.c
 *
 *  Created on: May 29, 2025
 *      Author: hugo-juarez
 */

#include "stm32f407xx.h"
#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx_usart_driver.h"
#include <string.h>

USARTx_Handle_t UART2Handle;
char message[] = "Hello from the STM32!\n";
uint8_t buttonPressed = 0;

static void delay(void){
	for(uint32_t i=0; i<500000/2; i++);
}

void UART2_GPIOInit(void){
	GPIOx_Handle_t UART2Gpio;

	UART2Gpio.pGPIOx = GPIOA;
	UART2Gpio.GPIOx_PinConfig.GPIOx_PinOType = GPIOx_OUT_TYPE_PP;
	UART2Gpio.GPIOx_PinConfig.GPIOx_PinSpeed = GPIOx_OSPEED_FAST;
	UART2Gpio.GPIOx_PinConfig.GPIOx_PinPuPdControl = GPIOx_PUPDR_NONE;

	UART2Gpio.GPIOx_PinConfig.GPIOx_PinMode = GPIOx_MODE_ALTFN;
	UART2Gpio.GPIOx_PinConfig.GPIOx_PinAltFunMode = 0x7;

	//USART2_TX
	UART2Gpio.GPIOx_PinConfig.GPIOx_PinNumber = GPIOx_PIN_NO_2;
	GPIOx_Init(&UART2Gpio);

	//USART2_RX
	UART2Gpio.GPIOx_PinConfig.GPIOx_PinNumber = GPIOx_PIN_NO_3;
	GPIOx_Init(&UART2Gpio);
}

void UART2_Init(void){

	UART2Handle.pUSARTx = USART2;
	UART2Handle.USART_Config.USART_Mode = USART_MODE_ONLY_TX;
	UART2Handle.USART_Config.USART_Baud = USART_STD_BAUD_115200;
	UART2Handle.USART_Config.USART_StopBits = USART_STOPBITS_1;
	UART2Handle.USART_Config.USART_WordLength = USART_WORDLEN_8BITS;
	UART2Handle.USART_Config.USART_Parity = USART_PARITY_DISABLE;
	UART2Handle.USART_Config.USART_HWFlowControl = USART_HW_FLOW_CTRL_NONE;

	USART_Init(&UART2Handle);
}


void GPIO_BtnConfig(void){

	GPIOx_Handle_t UserBtn;

	UserBtn.pGPIOx = GPIOA;
	UserBtn.GPIOx_PinConfig.GPIOx_PinNumber = GPIOx_PIN_NO_0;
	UserBtn.GPIOx_PinConfig.GPIOx_PinMode = GPIOx_MODE_IT_FT;
	UserBtn.GPIOx_PinConfig.GPIOx_PinOType = GPIOx_OUT_TYPE_PP;
	UserBtn.GPIOx_PinConfig.GPIOx_PinPuPdControl = GPIOx_PUPDR_NONE;
	UserBtn.GPIOx_PinConfig.GPIOx_PinSpeed = GPIOx_OSPEED_FAST;
	UserBtn.GPIOx_PinConfig.GPIOx_PinAltFunMode = 0;

	//Enable GPIOB Peripheral
	GPIOx_PCLKControl(GPIOA, ENABLE);

	//Initialize BTN interrupt
	GPIOx_Init(&UserBtn);

	//Enable Interrupt in NVIC
	GPIOx_IRQInterruptConfig(IRQ_NO_EXTI0, ENABLE);

}

int main(){

	//Configure BTN
	GPIO_BtnConfig();

	//Configure GPIO pins to ALT MODE UART TX
	UART2_GPIOInit();

	//Configure UART peripheral
	UART2_Init();

	//Enable UART Peripheral
	USART_PeripheralCtrl(USART2, ENABLE);

	for(;;){

		//Enable Interrupt Button
		GPIOx_IRQInterruptConfig(IRQ_NO_EXTI0, ENABLE);

		//Wait for button pressed
		while(!buttonPressed);

		//Disable Interrupt Button
		GPIOx_IRQInterruptConfig(IRQ_NO_EXTI0, DISABLE);

		USART_SendData(&UART2Handle, (uint8_t*) message, strlen(message));

		buttonPressed = 0;

	}

	return 0;
}

void EXTI0_IRQHandler(void){
	delay();
	//Reset Pending BIt
	GPIOx_IRQHandling(GPIOx_PIN_NO_0);
	buttonPressed = 1;

}
