/*
 * i2c_master_tx_testing.c
 *
 *  Created on: May 12, 2025
 *      Author: hugo-juarez
 */


#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx_i2c_driver.h"
#include <stdint.h>
#include <string.h>

//Macros
#define MY_ADDR				(0x61)
#define SLAVE_ADDR			(0x68)

//Global Variables
I2Cx_Handle_t I2C1_Handle;
uint8_t buttonPressed = 0;

static void delay(void){
	for(uint32_t i=0; i<500000/2; i++);
}

/*
 * PB6 -> SCL
 * PB9 -> SDA
 */

void I2C1_GPIOInit(void){

	GPIOx_Handle_t I2C1Pins;

	I2C1Pins.pGPIOx = GPIOB;
	I2C1Pins.GPIOx_PinConfig.GPIOx_PinMode = GPIOx_MODE_ALTFN;
	I2C1Pins.GPIOx_PinConfig.GPIOx_PinOType = GPIOx_OUT_TYPE_OD;
	I2C1Pins.GPIOx_PinConfig.GPIOx_PinPuPdControl = GPIOx_PUPDR_NONE;
	I2C1Pins.GPIOx_PinConfig.GPIOx_PinAltFunMode = 4;
	I2C1Pins.GPIOx_PinConfig.GPIOx_PinSpeed = GPIOx_OSPEED_FAST;

	//SCL
	I2C1Pins.GPIOx_PinConfig.GPIOx_PinNumber = GPIOx_PIN_NO_6;
	GPIOx_Init(&I2C1Pins);

	//SDA
	I2C1Pins.GPIOx_PinConfig.GPIOx_PinNumber = GPIOx_PIN_NO_7;
	GPIOx_Init(&I2C1Pins);

}

void I2C1_Inits(void){

	I2C1_Handle.pI2C = I2C1;
	I2C1_Handle.I2C_Config.I2C_ACKControl = I2C_ACK_EN;
	I2C1_Handle.I2C_Config.I2C_DeviceAddress = MY_ADDR;
	I2C1_Handle.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;
	I2C1_Handle.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM;

	I2Cx_Init(&I2C1_Handle);

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

int main(void){

	//GPIO Btn Config
	GPIO_BtnConfig();

	//Init Pins
	I2C1_GPIOInit();

	//I2C Peripheral Configuration
	I2C1_Inits();

	//Enable I2C1 Clock Peripheral
	I2C_PeripheralCtrl(I2C1, ENABLE);

	//Sending some data into the slave
	uint8_t some_data[] = "We are testing I2C master Tx\n";

	for(;;){

		//Enable Interrupt Button
		GPIOx_IRQInterruptConfig(IRQ_NO_EXTI0, ENABLE);

		//Wait for button pressed
		while(!buttonPressed);

		//Enable Interrupt Button
		GPIOx_IRQInterruptConfig(IRQ_NO_EXTI0, DISABLE);

		//Send Data
		I2C_MasterSendData(&I2C1_Handle, some_data, strlen((char *)some_data), SLAVE_ADDR);

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
