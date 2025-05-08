/*
 * stm32f407xx_i2c_driver.c
 *
 *  Created on: May 8, 2025
 *      Author: hugo-juarez
 */

#include "stm32f407xx_i2c_driver.h"

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
