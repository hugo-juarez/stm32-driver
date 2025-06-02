/*
 * uart_case.c
 *
 *  Created on: Jun 2, 2025
 *      Author: hugo-juarez
 */
#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx_usart_driver.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h>

// --- Macros ---
#define	MAX_BUFFER			256

// --- Global Variables ---
USARTx_Handle_t UART2Handle;
uint8_t i = 0;
char *msg[3] = {"hi do you copy 123","How You Doin?", "Today is Monday !"};
uint8_t rxBuffer[MAX_BUFFER];
uint8_t rxCmplt = RESET;

// --- Btn Delay ---
static void delay(void){
	for(uint32_t i=0; i<500000/2; i++);
}

// --- UART GPIO Init ---
void UART_GPIOInit(void){
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

// --- UART Init ---
void UART_Init(void){
	UART2Handle.pUSARTx = USART2;
	UART2Handle.USART_Config.USART_Mode = USART_MODE_TXRX;
	UART2Handle.USART_Config.USART_Baud = USART_STD_BAUD_115200;
	UART2Handle.USART_Config.USART_StopBits = USART_STOPBITS_1;
	UART2Handle.USART_Config.USART_WordLength = USART_WORDLEN_8BITS;
	UART2Handle.USART_Config.USART_Parity = USART_PARITY_DISABLE;
	UART2Handle.USART_Config.USART_HWFlowControl = USART_HW_FLOW_CTRL_NONE;

	USART_Init(&UART2Handle);
}

// --- BTN Config ---
void GPIO_BtnConfig(void){

	GPIOx_Handle_t UserBtn;

	UserBtn.pGPIOx = GPIOA;
	UserBtn.GPIOx_PinConfig.GPIOx_PinNumber = GPIOx_PIN_NO_0;
	UserBtn.GPIOx_PinConfig.GPIOx_PinMode = GPIOx_MODE_IN;
	UserBtn.GPIOx_PinConfig.GPIOx_PinOType = GPIOx_OUT_TYPE_PP;
	UserBtn.GPIOx_PinConfig.GPIOx_PinPuPdControl = GPIOx_PUPDR_NONE;
	UserBtn.GPIOx_PinConfig.GPIOx_PinSpeed = GPIOx_OSPEED_FAST;
	UserBtn.GPIOx_PinConfig.GPIOx_PinAltFunMode = 0;

	//Enable GPIOA Peripheral
	GPIOx_PCLKControl(GPIOA, ENABLE);

	//Initialize BTN interrupt
	GPIOx_Init(&UserBtn);

}


int main(void){

	//Config BTN
	GPIO_BtnConfig();

	//UART GPIO
	UART_GPIOInit();

	//UART Config
	UART_Init();

	//Enable UART Interrupt
	USART_IRQInterruptConfig(IRQ_NO_USART2, ENABLE);

	//Enable UART peripheral
	USART_PeripheralCtrl(USART2, ENABLE);

	printf("Application Started!\n");
	printf("====================\n");

	for(;;){

		//Read BTN input
		while(!GPIOx_ReadPin(GPIOA, GPIOx_PIN_NO_0));

		//Delay for de-bounce of button
		delay();

		//Select which message we are reading and account for going over it
		i = i % 3;

		//Enable interrupt for recieving data
		while(USART_ReceiveDataIT(&UART2Handle, rxBuffer, strlen(msg[i])) != USART_FREE);

		//Enable interrupt for sending data
		USART_SendData(&UART2Handle, (uint8_t*) msg[i],  strlen(msg[i]));

		printf("Transmitted: %s\n", msg[i]);

		//Wait for Rx to complete
		while(rxCmplt != SET);

		//Adding ending string byte
		rxBuffer[strlen(msg[i])] = '\0';

		printf("Received: %s\n", rxBuffer);

		//Reset flag
		rxCmplt = RESET;

		//Move index
		i++;

	}

	return 0;
}

// --- Pass Handling Function to Interrupt ---
void USART2_IRQHandler(void){
	USART_IRQHandling(&UART2Handle);
}

// --- Callback Function ---
void USART_ApplicationEventCallback(USARTx_Handle_t *pUSARTHandle, uint8_t event){
	if (event == USART_EV_RX_CMPLT){
		rxCmplt = SET;
	}
}
