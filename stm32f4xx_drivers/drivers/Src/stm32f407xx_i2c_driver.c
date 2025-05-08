/*
 * stm32f407xx_i2c_driver.c
 *
 *  Created on: May 8, 2025
 *      Author: hugo-juarez
 */

#include "stm32f407xx_i2c_driver.h"


static uint32_t RCC_GetPLLOutputClock(void){
	return 16;
}//To be implemented

static uint32_t RCC_GetPCLK1Value(void){

	uint16_t AHB_Prescaler[8] = {2,4,8,16,64,128,256,512};
	uint16_t APB_Prescaler[4] = {2,4,8,16};
	uint32_t pclk1;
	//Check which is the value of sysclock

	uint8_t clcksrc = (RCC->CFGR >> 2) & 0x3;
	uint32_t systemclk;
	uint32_t ahb;
	uint32_t apb;

	if(clcksrc == 0){ //HSI
		systemclk = 16000000;
	} else if(clcksrc == 1){//External
		systemclk = 8000000;
	} else if(clcksrc == 2){
		systemclk = RCC_GetPLLOutputClock();
	}

	//Now we need to know the AHB prescalaer
	uint32_t temp;

	temp = (RCC->CFGR >> 4) & 0xf;

	if(temp < 8){
		ahb = 1;
	} else{
		ahb = AHB_Prescaler[temp - 8];
	}

	//Finally we are only missing the APB1 preescaler

	temp = (RCC->CFGR >> 10) & 0x7;

	if(temp < 4){
		apb = 1;
	} else{
		apb = APB_Prescaler[temp - 4];
	}

	pclk1 = (systemclk/ahb)/apb;

	return pclk1;
}

//PCLK Control
void I2Cx_PCLKControl(I2Cx_RegDef_t* pI2Cx, uint8_t state){
	if(state == ENABLE){
		if(pI2Cx == I2C1){
			I2C1_PCLK_EN();
		} else if(pI2Cx == I2C2){
			I2C2_PCLK_EN();
		} else if(pI2Cx == I2C3){
			I2C3_PCLK_EN();
		}
	} else{
		if(pI2Cx == I2C1){
			I2C1_PCLK_DI();
		} else if(pI2Cx == I2C2){
			I2C2_PCLK_DI();
		} else if(pI2Cx == I2C3){
			I2C3_PCLK_DI();
		}
	}
}

//Init and Deinit

void I2Cx_Init(I2Cx_Handle_t* pI2CHandle){
	uint32_t tempreg = 0;
	uint32_t pclk = 0;

	//ACK Control Bit
	tempreg |= pI2CHandle->I2C_Config.I2C_ACKControl << I2C_CR1_ACK;
	pI2CHandle->pI2C->CR1 = tempreg;

	//FREQ Register
	pclk = RCC_GetPCLK1Value();
	tempreg = pclk/1000000;
	pI2CHandle->pI2C->CR2 = (tempreg & 0x3f);

	//Own Address Register
	tempreg = pI2CHandle->I2C_Config.I2C_DeviceAddress << 1;
	tempreg |= 1 << 14;
	pI2CHandle->pI2C->OAR1 = tempreg;

	uint16_t crrValue= 0;
	tempreg = 0;

	//CCR
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM){
		//Standard Mode
		crrValue = pclk/(2*pI2CHandle->I2C_Config.I2C_SCLSpeed);
		tempreg |= (crrValue & 0xfff);
	} else{
		//Fast Mode
		tempreg |= (1<<15);
		tempreg |= pI2CHandle->I2C_Config.I2C_FMDutyCycle << 14;
		//Check DUTY
		if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2){
			crrValue = pclk/(3*pI2CHandle->I2C_Config.I2C_SCLSpeed);
		} else {
			crrValue = pclk/(25*pI2CHandle->I2C_Config.I2C_SCLSpeed);
		}
		tempreg |= (crrValue & 0xfff);
	}

	pI2CHandle->pI2C->CCR = tempreg;

	//Configure TRISE
}

void I2Cx_DeInit(I2Cx_RegDef_t* pI2Cx){
	if(pI2Cx == I2C1){
		I2C1_REG_RESET();
	} else if(pI2Cx == I2C2){
		I2C2_REG_RESET();
	} else if(pI2Cx == I2C3){
		I2C3_REG_RESET();
	}
}

// IRQ Configuration and ISR handling
void I2Cx_IRQInterruptConfig(uint8_t IRQNumber, uint8_t state){
	uint8_t temp1 = IRQNumber / 32;
	uint8_t temp2 = IRQNumber % 32;

	if(state == ENABLE){
		NVIC->ISER[temp1] |= (1 << temp2);
	} else {
		NVIC->ICER[temp1] |= (1 << temp2);
	}
}
void I2Cx_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority){
	NVIC->IPR[IRQNumber] = (IRQPriority << (8 - NO_PR_BITS_IMPLEMENTED));
}

// Other peripheral control APIs
void I2C_PeripheralCtrl(I2Cx_RegDef_t* pI2Cx, uint8_t state){
	if(state == ENABLE){
		pI2Cx->CR1 |= (1 << I2C_CR1_PE);
	} else{
		pI2Cx->CR1 &= ~(1 << I2C_CR1_PE); //Reset to 0
	}

}
