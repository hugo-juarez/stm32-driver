/*
 * stm32f407xx_i2c_driver.c
 *
 *  Created on: May 8, 2025
 *      Author: hugo-juarez
 */

#include "stm32f407xx_i2c_driver.h"

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

void I2Cx_DeInit(I2Cx_RegDef_t* pI2Cx){
	if(pI2Cx == I2C1){
		I2C1_REG_RESET();
	} else if(pI2Cx == I2C2){
		I2C2_REG_RESET();
	} else if(pI2Cx == I2C3){
		I2C3_REG_RESET();
	}
}
