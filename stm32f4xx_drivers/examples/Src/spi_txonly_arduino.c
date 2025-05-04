/*
 * spi_txonly_arduino.c
 *
 *  Created on: May 3, 2025
 *      Author: hugo-juarez
 */

#include "stm32f407xx_spi_driver.h"
#include "stm32f407xx_gpio_driver.h"
#include <string.h>


/* PB14 -> SPI2_MISO
 * PB15 -> SPI2_MOSI
 * PB13 -> SPI2_SCLK
 * PB12 -> SPI2_NSS
 * ALT Function Mode = 5
 */

uint8_t btnIsPressed = 0;

void SPI2_GPIOInit(void){
	GPIOx_Handle_t SPIPins;

	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIOx_PinConfig.GPIOx_PinMode = GPIOx_MODE_ALTFN;
	SPIPins.GPIOx_PinConfig.GPIOx_PinAltFunMode = 5;
	SPIPins.GPIOx_PinConfig.GPIOx_PinOType = GPIOx_OUT_TYPE_PP;
	SPIPins.GPIOx_PinConfig.GPIOx_PinPuPdControl = GPIOx_PUPDR_NONE;
	SPIPins.GPIOx_PinConfig.GPIOx_PinSpeed = GPIOx_OSPEED_FAST;

	//SCLK
	SPIPins.GPIOx_PinConfig.GPIOx_PinNumber = GPIOx_PIN_NO_13;
	GPIOx_Init(&SPIPins);

	//MOSI
	SPIPins.GPIOx_PinConfig.GPIOx_PinNumber = GPIOx_PIN_NO_15;
	GPIOx_Init(&SPIPins);

//	//MISO
//	SPIPins.GPIOx_PinConfig.GPIOx_PinNumber = GPIOx_PIN_NO_14;
//	GPIOx_Init(&SPIPins);
//
	//NSS
	SPIPins.GPIOx_PinConfig.GPIOx_PinNumber = GPIOx_PIN_NO_12;
	GPIOx_Init(&SPIPins);
}

void SPI2_Inits(void){
	SPIx_Handle_t SPI2Handle;

	SPI2Handle.pSPIx = SPI2;
	SPI2Handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2Handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2Handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8;
	SPI2Handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2Handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2Handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2Handle.SPIConfig.SPI_SSM = SPI_SSM_DI;

	SPIx_Init(&SPI2Handle);
}

void GPIO_BtnInit (void){
	//GPIO Interrupt Button Config
	GPIOx_Handle_t GPIOBtn;
	memset(&GPIOBtn, 0, sizeof(GPIOBtn)); //Initialize all member values to 0

	GPIOBtn.pGPIOx = GPIOA;

	GPIOBtn.GPIOx_PinConfig.GPIOx_PinNumber = GPIOx_PIN_NO_0; //USR PIN
	GPIOBtn.GPIOx_PinConfig.GPIOx_PinMode = GPIOx_MODE_IT_FT;
	GPIOBtn.GPIOx_PinConfig.GPIOx_PinPuPdControl = GPIOx_PUPDR_NONE;


	//	ENABLE CLOCKS
	GPIOx_PCLKControl(GPIOA, ENABLE);

	//	SETTING INIT VALUES
	GPIOx_Init(&GPIOBtn);

	// Set IRQ Configuration
	GPIOx_IRQInterruptConfig(IRQ_NO_EXTI0, ENABLE);
}

void delay(void){
	for(uint32_t i=0; i<500000/2; i++);
}

void EXTI0_IRQHandler(void){

	delay();
	//Resets Pending Register
	GPIOx_IRQHandling(GPIOx_PIN_NO_0);

	btnIsPressed = 1;


}

int main(void){

	char data[] = "Hello world";

	//Initialize GPIO pins
	SPI2_GPIOInit();

	//Initialize Btn
	GPIO_BtnInit();

	//Initialize SPI2 peripheral parameters
	SPI2_Inits();

	//SSOE Enable for one master communication
	SPI_SSOEConfig(SPI2, ENABLE);


	for(;;){
		if(!btnIsPressed)
			continue;

		//Enable SPI peripheral
		SPI_PeripheralCtrl(SPI2, ENABLE);

		//Send Length Info
		uint8_t dataLen = strlen(data);
		SPI_SendData(SPI2, &dataLen, 1);

		//Send Data
		SPI_SendData(SPI2, (uint8_t*)data, strlen(data));

		//Check that the SPI is not busy in communication
		while( (SPI2->SR & (1 << 7)) );

		//Disable SPI Peripheral
		SPI_PeripheralCtrl(SPI2, DISABLE);

		//Reset interrupt flag
		btnIsPressed = 0;
	}


	return 0;
}
