/*
 * stm32f407xx_i2c_driver.h
 *
 *  Created on: May 8, 2025
 *      Author: hugo-juarez
 */

#ifndef INC_STM32F407XX_I2C_DRIVER_H_
#define INC_STM32F407XX_I2C_DRIVER_H_

#include "stm32f407xx.h"

//Configuration Structure
typedef struct {
	uint32_t I2C_SCLSpeed;
	uint32_t I2C_DeviceAddress;
	uint32_t I2C_ACKControl;
	uint32_t I2C_FMDutyCycle;
}I2Cx_Config_t;

//Handle Struct

typedef struct{
	I2Cx_RegDef_t* pI2C;
	I2Cx_Config_t I2C_Config;
} I2Cx_Handle_t;

//MACROS
//@I2C_SCLSpeed
#define I2C_SCL_SPEED_SM			100000
#define I2C_SCL_SPEED_FM2K			200000
#define I2C_SCL_SPEED_FM4K			400000

//@I2C_ACKControl
#define I2C_ACK_EN					1
#define I2C_ACK_DI					0

//@I2C_FMDutyCycle
#define I2C_FM_DUTY_2				0
#define I2C_FM_DUTY_16_9			1

/*****************************************************************
 * 							APIs!!!!!!
 *****************************************************************/

// PCLK Control
void I2Cx_PCLKControl(I2Cx_RegDef_t* pI2Cx, uint8_t state);

// Init-DeInit
void I2Cx_Init(I2Cx_Handle_t* pI2CHandle);
void I2Cx_DeInit(I2Cx_RegDef_t* pI2Cx);

// Data Send and Receive


// IRQ Configuration and ISR handling
void I2Cx_IRQInterruptConfig(uint8_t IRQNumber, uint8_t state);
void I2Cx_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);

// Other peripheral control APIs
void I2C_PeripheralCtrl(I2Cx_RegDef_t* pI2Cx, uint8_t state);

//Calback Function for Application
void I2C_ApplicationEventCallback(I2Cx_Handle_t* pHandle,uint8_t event);


#endif /* INC_STM32F407XX_I2C_DRIVER_H_ */
