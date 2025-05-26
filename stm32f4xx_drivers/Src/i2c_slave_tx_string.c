/*
 * i2c_slave_tx_string.c
 *
 *  Created on: May 25, 2025
 *      Author: hugo-juarez
 */

#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx_i2c_driver.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

//Macros
#define SLAVE_ADDR			(0x68)
#define MAX_LEN				300

//Global Variables
I2Cx_Handle_t I2C1_Handle;
uint8_t Tx_buf[32] = "STM32 Slave mode testing";

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
	I2C1_Handle.I2C_Config.I2C_DeviceAddress = SLAVE_ADDR;
	I2C1_Handle.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;
	I2C1_Handle.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM;

	I2C_Init(&I2C1_Handle);

}

int main(void){

	//I2C Pin Init
	I2C1_GPIOInit();

	//I2C Peripheral config
	I2C1_Inits();

	//I2C IRQ config
	I2C_IRQInterruptConfig(IRQ_NO_I2C1_EV, ENABLE);
	I2C_IRQInterruptConfig(IRQ_NO_I2C1_ER, ENABLE);

	I2C_SlaveCallbackEvents(I2C1_Handle.pI2C, ENABLE);

	//Enable I2C Peripheral
	I2C_PeripheralCtrl(I2C1, ENABLE);

	//Enable ACK bit
	I2C_SetACK(I2C1, ENABLE);

	for(;;);

	return 0;
}

void I2C1_EV_IRQHandler(void){
	I2C_EV_IRQHandling(&I2C1_Handle);
}

void I2C1_ER_IRQHandler(void){
	I2C_ER_IRQHandling(&I2C1_Handle);
}

void I2C_ApplicationEventCallback(I2Cx_Handle_t* pHandle,uint8_t event){
	static uint8_t command_code = 0;
	static uint8_t i = 0;

	if(event == I2C_EV_DATA_REQ){
		//Master is reading data. Slave needs to send it
		if(command_code == 0x51){

			I2C_SlaveSendData(I2C1_Handle.pI2C, strlen((char *)Tx_buf));

		} else if (command_code == 0x52){

			//Send Contents of Tx_buf
			I2C_SlaveSendData(I2C1_Handle.pI2C, Tx_buf[i++]);

		}

	} else if(event == I2C_EV_DATA_RCV){
		//Master writes to slave. Slave needs to read it.
		command_code = I2C_SlaveReceiveData(I2C1_Handle.pI2C);

	} else if(event == I2C_ER_AF){
		//On slave TX indicates master sent NACK so slave needs to stop sending data.
		command_code = 0xff;
		i = 0;

	} else if(event == I2C_EV_STOP){
		//On slave RX indicates master has ended communication with the slave.

	}
}






