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
	I2Cx_RegDef_t	*pI2C;
	I2Cx_Config_t 	I2C_Config;
	uint8_t			*pTxBuffer;		//Store the app TX buffer address
	uint8_t			*pRxBuffer;		//Store the app RX buffer address
	uint32_t		TxLen;			//Store TX Len
	uint32_t		RxLen;			//Store RX Len
	uint8_t			TxRxState;		//Store communication state
	uint8_t			DevAddr;		//Store slave/device address
	uint32_t		RxSize;			//Store RX Size
	uint32_t		RS;				//Store repeated start value
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

//FLAGS
#define I2C_FLAG_SB					(1 << I2C_SR1_SB)
#define I2C_FLAG_BTF				(1 << I2C_SR1_BTF)
#define I2C_FLAG_TXE				(1 << I2C_SR1_TXE)
#define I2C_FLAG_RXNE				(1 << I2C_SR1_RXNE)
#define I2C_FLAG_ADDR				(1 << I2C_SR1_ADDR)
#define I2C_FLAG_BERR				(1 << I2C_SR1_BERR)
#define I2C_FLAG_ARLO				(1 << I2C_SR1_ARLO)
#define I2C_FLAG_AF					(1 << I2C_SR1_AF)
#define I2C_FLAG_OVR				(1 << I2C_SR1_OVR)
#define I2C_FLAG_TIMEOUT			(1 << I2C_SR1_TIMEOUT)

//Select Read Or Write for address write up
#define WRITE						0
#define READ						1

//Repeated Start
#define I2C_ENABLE_RS				0
#define I2C_DISABLE_RS				1

//I2C Application States
#define I2C_READY					0
#define I2C_BUSY_IN_RX				1
#define I2C_BUSY_IN_TX				2

//I2C Application Events
#define I2C_EV_STOP					0
#define I2C_EV_TX_CMPLT				1
#define I2C_EV_RX_CMPLT				2

//I2C Error Events
#define I2C_ER_BERR					3
#define I2C_ER_ARLO					4
#define I2C_ER_AF					5
#define I2C_ER_OVR					6
#define I2C_ER_TIMEOUT				7


/*****************************************************************
 * 							APIs!!!!!!
 *****************************************************************/

// PCLK Control
void I2C_PCLKControl(I2Cx_RegDef_t* pI2Cx, uint8_t state);

// Init-DeInit
void I2C_Init(I2Cx_Handle_t* pI2CHandle);
void I2C_DeInit(I2Cx_RegDef_t* pI2Cx);

// Data Send and Receive
void I2C_MasterSendData(I2Cx_Handle_t* pI2CHandle, uint8_t* pTxBuffer, uint32_t len, uint8_t slaveAddr,  uint8_t repeatedStart);
void I2C_MasterReceiveData(I2Cx_Handle_t* pI2CHandle, uint8_t* pRxBuffer, uint32_t len, uint8_t slaveAddr,  uint8_t repeatedStart);

// Data Send and Receive IT
uint8_t I2C_MasterSendDataIT(I2Cx_Handle_t* pI2CHandle, uint8_t* pTxBuffer, uint32_t len, uint8_t slaveAddr,  uint8_t repeatedStart);
uint8_t I2C_MasterReceiveDataIT(I2Cx_Handle_t* pI2CHandle, uint8_t* pRxBuffer, uint32_t len, uint8_t slaveAddr,  uint8_t repeatedStart);
void I2C_CloseSendData(I2Cx_Handle_t *pI2CHandle);
void I2C_CloseReceiveData(I2Cx_Handle_t *pI2CHandle);
void I2C_GenerateStopCondition(I2Cx_RegDef_t* pI2C);

// IRQ Configuration and ISR handling
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t state);
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void I2C_EV_IRQHandling(I2Cx_Handle_t* pI2CHandle);
void I2C_ER_IRQHandling(I2Cx_Handle_t* pI2CHandle);

// Other peripheral control APIs
void I2C_PeripheralCtrl(I2Cx_RegDef_t* pI2Cx, uint8_t state);

//Calback Function for Application
void I2C_ApplicationEventCallback(I2Cx_Handle_t* pHandle,uint8_t event);


#endif /* INC_STM32F407XX_I2C_DRIVER_H_ */
