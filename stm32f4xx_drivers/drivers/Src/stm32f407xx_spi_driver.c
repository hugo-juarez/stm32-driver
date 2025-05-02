/*
 * stm32f407xx_spi_driver.c
 *
 *  Created on: May 2, 2025
 *      Author: hugo-juarez
 */

#include "stm32f407xx_spi_driver.h"

/**************** PCLK Control ****************
 *
 * @fn				- SPIx_PCLKControl
 *
 * @brief			- This functions enables or disables the peripheral clock for the given SPI
 *
 * @param[in]		- base address of the SPI peripheral
 * @param[in]		- ENABLE or DISABLE macros
 * @param[in]		-
 *
 * @return			- none
 *
 * @note			- none
 *
 */

void SPIx_PCLKControl(SPIx_RegDef_t* pSPIx, uint8_t state){
	if(state == ENABLE){
		if(pSPIx == SPI1)
			SPI1_PCLK_EN();
		else if(pSPIx == SPI2)
			SPI2_PCLK_EN();
		else if(pSPIx == SPI3)
			SPI3_PCLK_EN();
	}else{
		if(pSPIx == SPI1)
			SPI1_PCLK_DI();
		else if(pSPIx == SPI2)
			SPI2_PCLK_DI();
		else if(pSPIx == SPI3)
			SPI3_PCLK_DI();
	}
}

/**************** SPI INIT ****************
 *
 * @fn				- SPIx_Init
 *
 * @brief			- This functions initilize the registers of a SPI to a set value by the user
 *
 * @param[in]		- handler with pointer of SPI base address and values to set the registers
 * @param[in]		-
 * @param[in]		-
 *
 * @return			- none
 *
 * @note			- none
 *
 */
void SPIx_Init(SPIx_Handle_t* pSPIHandle){

}

/**************** SPI DEINIT ****************
 *
 * @fn				- SPIx_DeInit
 *
 * @brief			- This function resets the default values of the SPI register
 *
 * @param[in]		- SPI base address
 * @param[in]		-
 * @param[in]		-
 *
 * @return			- none
 *
 * @note			- none
 *
 */

void SPIx_DeInit(SPIx_RegDef_t* pSPIx){
	if(pSPIx == SPI1)
		SPI1_REG_RESET();
	else if(pSPIx == SPI2)
		SPI2_REG_RESET();
	else if(pSPIx == SPI3)
		SPI3_REG_RESET();
}
