/*
 * stm32f407xx_spi_driver.h
 *
 *  Created on: May 2, 2025
 *      Author: hugo-juarez
 */

#ifndef INC_STM32F407XX_SPI_DRIVER_H_
#define INC_STM32F407XX_SPI_DRIVER_H_

#include "stm32f407xx.h"

// Configuration Structure for SPIx peripheral
typedef struct {
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_SclkSpeed;
	uint8_t SPI_DFF;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;
} SPIx_Config_t;

// Handle Struct

typedef struct{
	SPIx_RegDef_t* pSPIx;
	SPIx_Config_t SPIConfig;
}SPIx_Handle_t;

// @SPI_DeviceMode

#define SPI_DEVICE_MODE_MASTER					1
#define SPI_DEVICE_MODE_SLAVE					0

// @SPI_BusConfig

#define SPI_BUS_CONFIG_FD						1
#define SPI_BUS_CONFIG_HD						2
#define SPI_BUS_CONFIG_S_TXONLY					3
#define SPI_BUS_CONFIG_S_RXONLY					4

// @SPI_SclkSpeed

#define SPI_SCLK_SPEED_DIV2						0
#define SPI_SCLK_SPEED_DIV4						1
#define SPI_SCLK_SPEED_DIV8						2
#define SPI_SCLK_SPEED_DIV16					3
#define SPI_SCLK_SPEED_DIV32					4
#define SPI_SCLK_SPEED_DIV64					5
#define SPI_SCLK_SPEED_DIV128					6
#define SPI_SCLK_SPEED_DIV256					7

// @SPI_DFF
#define SPI_DFF_8BITS							0
#define SPI_DFF_16BITS							1

// @SPI_CPOL

#define SPI_CPOL_LOW							0
#define SPI_CPOL_HIGH							1

// @SPI_CPHA

#define SPI_CPHA_LOW							0
#define SPI_CPHA_HIGH							1

// @SPI_SSM

#define SPI_SSM_DI								0
#define SPI_SSM_EN								1

/*****************************************************************
 * 							APIs!!!!!!
 *****************************************************************/

// PCLK Control
void SPIx_PCLKControl(SPIx_RegDef_t* pSPIx, uint8_t state);

// Init-DeInit
void SPIx_Init(SPIx_Handle_t* pSPIHandle);
void SPIx_DeInit(SPIx_RegDef_t* pSPIx);

// Data Send and Receive
void SPI_SendData(SPIx_RegDef_t* pSPIx, uint8_t* pTxBuffer, uint32_t len);
void SPI_ReceiveData(SPIx_RegDef_t* pSPIx, uint8_t* pRxBuffer, uint32_t len);

// IRQ Configuration and ISR handling
void SPIx_IRQInterruptConfig(uint8_t IRQNumber, uint8_t state);
void SPIx_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPIx_IRQHandling(SPIx_Handle_t *pHandle);

// Other peripheral control APIs


#endif /* INC_STM32F407XX_SPI_DRIVER_H_ */
