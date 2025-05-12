/*
 * spi_message_rx_it.c
 *
 *  Created on: May 6, 2025
 *      Author: hugo-juarez
 */

#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx_spi_driver.h"
#include <stdio.h>

#define MAX_LEN 500

//SPI2 Global Handle
SPIx_Handle_t SPI2Handle;

uint8_t slvRDY = 0;
uint8_t done = 0;
char readBuffer[MAX_LEN];
uint8_t readData = 0;

void SPI2_GPIOInit(){
	GPIOx_Handle_t SPIPins;

	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIOx_PinConfig.GPIOx_PinMode = GPIOx_MODE_ALTFN;
	SPIPins.GPIOx_PinConfig.GPIOx_PinOType = GPIOx_OUT_TYPE_PP;
	SPIPins.GPIOx_PinConfig.GPIOx_PinPuPdControl = GPIOx_PUPDR_NONE;
	SPIPins.GPIOx_PinConfig.GPIOx_PinSpeed = GPIOx_OSPEED_FAST;
	SPIPins.GPIOx_PinConfig.GPIOx_PinAltFunMode = 5;

	//Enable GPIOB Peripheral
	GPIOx_PCLKControl(GPIOB, ENABLE);

	//SPI2 NSS Pin
	SPIPins.GPIOx_PinConfig.GPIOx_PinNumber = GPIOx_PIN_NO_12;
	GPIOx_Init(&SPIPins);

	//SPI2 SCK Pin
	SPIPins.GPIOx_PinConfig.GPIOx_PinNumber = GPIOx_PIN_NO_13;
	GPIOx_Init(&SPIPins);

	//SPI2 MISO
	SPIPins.GPIOx_PinConfig.GPIOx_PinNumber = GPIOx_PIN_NO_14;
	GPIOx_Init(&SPIPins);

	//SPI2 MOSI
	SPIPins.GPIOx_PinConfig.GPIOx_PinNumber = GPIOx_PIN_NO_15;
	GPIOx_Init(&SPIPins);
}

void SPI2_Init(){

	SPI2Handle.pSPIx = SPI2;
	SPI2Handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2Handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2Handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2Handle.SPIConfig.SPI_SSM = SPI_SSM_DI;
	SPI2Handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8;
	SPI2Handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2Handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;

	SPIx_PCLKControl(SPI2, ENABLE);

	SPIx_Init(&SPI2Handle);
}

void SlaveRDY_GPIOInit(){
	GPIOx_Handle_t SlaveRDYPin;

	SlaveRDYPin.pGPIOx = GPIOD;
	SlaveRDYPin.GPIOx_PinConfig.GPIOx_PinNumber = GPIOx_PIN_NO_6;
	SlaveRDYPin.GPIOx_PinConfig.GPIOx_PinMode = GPIOx_MODE_IT_FT;
	SlaveRDYPin.GPIOx_PinConfig.GPIOx_PinOType = GPIOx_OUT_TYPE_PP;
	SlaveRDYPin.GPIOx_PinConfig.GPIOx_PinPuPdControl = GPIOx_PUPDR_NONE;
	SlaveRDYPin.GPIOx_PinConfig.GPIOx_PinSpeed = GPIOx_OSPEED_FAST;
	SlaveRDYPin.GPIOx_PinConfig.GPIOx_PinAltFunMode = 0;

	GPIOx_Init(&SlaveRDYPin);
}


int main(void){

	//Dummy data for fetching info
	uint8_t dummy = 0xff;

	//Init GPIO pins
	SPI2_GPIOInit();
	SlaveRDY_GPIOInit();

	//Configure SPI2 communication
	SPI2_Init();
	SPI_SSOEConfig(SPI2, ENABLE);

	//Enable SPI2 interrupts
	SPIx_IRQInterruptConfig(IRQ_NO_SPI2, ENABLE);

	for(;;){
		printf("Waiting on Slave Signal\n");
		//Enable SLV Interrupts
		GPIOx_IRQInterruptConfig(IRQ_NO_EXTI9_5, ENABLE);
		while(!slvRDY);
		printf("Slave Signal Received\n");
		//Disable them while SPI transmission is being made
		GPIOx_IRQInterruptConfig(IRQ_NO_EXTI9_5, DISABLE);

		//Enable SPI peripheral
		SPI_PeripheralCtrl(SPI2, ENABLE);

		printf("Receiving Data\n");
		//Receive Data until we reach the end of the character '\0'
		while(!done){
			while(SPI_SendDataIT(&SPI2Handle, &dummy, 1) == SPI_BUSY_IN_TX );
			while(SPI_ReceiveDataIT(&SPI2Handle, &readData, 1) == SPI_BUSY_IN_RX);
		}

		//Wait for SPI is not busy in communication
		while( (SPI2->SR & (1 << 7)) );

		//Disable SPI
		SPI_PeripheralCtrl(SPI2, DISABLE);


		//print Result
		printf("Received = %s\n", readBuffer);

		//Reset Done Bit From last Run
		done = 0;

		//Reset Interupt
		slvRDY = 0;
	}


	return 0;
}

//SlaveRDY Interrupt
void EXTI9_5_IRQHandler(void){
	GPIOx_IRQHandling(GPIOx_PIN_NO_6);
	slvRDY = 1;
}

//SPI2 Interrupt
void SPI2_IRQHandler(void){
	SPIx_IRQHandling(&SPI2Handle);
}

//Interrupt Handler
void SPI_ApplicationEventCallback(SPIx_Handle_t* pHandle,uint8_t event){
	static uint32_t i = 0;

	if(event == SPI_EVENT_RX_CMPLT){
		readBuffer[i++] = readData;
		if(readData == '\0' || (i == MAX_LEN)){
			done = 1;
			//Make sure last element is \0
			readBuffer[i - 1] = '\0';
			i=0;
		}
	}
}
