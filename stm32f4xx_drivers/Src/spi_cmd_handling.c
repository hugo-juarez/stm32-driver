/*
 * spi_cmd_handling.c
 *
 *  Created on: May 4, 2025
 *      Author: hugo-juarez
 */

#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx_spi_driver.h"

//Command controls
#define COMMAND_LED_CTRL            0x50
#define COMMAND_SENSOR_READ         0x51
#define COMMAND_LED_READ			0x52
#define COMMAND_PRINT	            0x53
#define COMMAND_ID_READ 	        0x54

//ACK or NACK
#define NACK 						0xA5
#define ACK 						0xF5

//Arguments
#define LED_PIN						9
#define LED_ON						1

//Initializations
void SPI2_GPIOInit(void);
void SPI2_Inits(void);
void GPIO_BtnConfig(void);

//SPI API
uint8_t SPI_VerifyResponse(uint8_t ack_byte);

uint8_t btn_is_pressed = 0;

int main(void){
	//dummy byte
	uint8_t dummy_read = 0xff;
	uint8_t dummy_write = 0xff;

	//SPI2 GPIO Pins Initialization
	SPI2_GPIOInit();

	//Button Config
	GPIO_BtnConfig();

	//SPI2 Config Initialization
	SPI2_Inits();

	//Enable SSOE meaning using NSS pin
	SPI_SSOEConfig(SPI2, ENABLE);

	for(;;){
		if(!btn_is_pressed)
			continue;

		//Enable SPI
		SPI_PeripheralCtrl(SPI2, ENABLE);

		//1. CMD_LED_CTRL <pin no(1)> <value>
		uint8_t command_code = COMMAND_LED_CTRL;
		uint8_t args[2];
		uint8_t ack_byte;
		SPI_SendData(SPI2, &command_code, 1);
		//To clear RXNE
		SPI_ReceiveData(SPI2, &dummy_read, 1);

		//Fetching Response from Slave
		SPI_SendData(SPI2, &dummy_write, 1);
		SPI_ReceiveData(SPI2, &ack_byte, 1);

		if (SPI_VerifyResponse(ack_byte)){
			//If ACK send arguments
			args[0] = LED_PIN;
			args[1] = LED_ON;

			//Send Arguments
			SPI_SendData(SPI2, args, 2);
		}

		//Wait for SPI is not busy in communication
		while( (SPI2->SR & (1 << 7)) );

		//Disable SPI
		SPI_PeripheralCtrl(SPI2, DISABLE);

		btn_is_pressed = 0;

	}


	return 0;
}

void SPI2_GPIOInit(void){
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

void SPI2_Inits(void){

	SPIx_Handle_t SPI2Config;

	SPI2Config.pSPIx = SPI2;
	SPI2Config.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2Config.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2Config.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2Config.SPIConfig.SPI_SSM = SPI_SSM_DI;
	SPI2Config.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8;
	SPI2Config.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2Config.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;

	SPIx_PCLKControl(SPI2, ENABLE);

	SPIx_Init(&SPI2Config);

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

uint8_t SPI_VerifyResponse(uint8_t ack_byte){
	if(ack_byte == ACK)
		return 1;
	else
		return 0;
}


void delay(void){
	for(uint32_t i=0; i<500000/2; i++);
}

void EXTI0_IRQHandler(void){
	delay();
	//Reset Pending BIt
	GPIOx_IRQHandling(GPIOx_PIN_NO_0);
	btn_is_pressed = 1;
}

