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

#endif /* INC_STM32F407XX_SPI_DRIVER_H_ */
