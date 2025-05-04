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
	//Debbuging tip if the peripheral is not in master it won't produce the sclk
	SPIx_PCLKControl(pSPIHandle->pSPIx, ENABLE);

	uint32_t tempreg = 0;

	// Set Master or Slave mode
	tempreg |= (pSPIHandle->SPIConfig.SPI_DeviceMode << 2);

	// BUS Mode
	switch(pSPIHandle->SPIConfig.SPI_BusConfig){
		case SPI_BUS_CONFIG_FD:
			tempreg &= ~(1 << 15);
			break;
		case SPI_BUS_CONFIG_HD:
			tempreg |= (1 << 15);
			break;
		case SPI_BUS_CONFIG_S_RXONLY:
			tempreg &= ~(1 << 15);
			tempreg |= (1 << 10);
			break;
		default:
			break;
	}

	//SPI Clock Speed
	tempreg |= (pSPIHandle->SPIConfig.SPI_SclkSpeed << 3);

	// Data Frame Format
	tempreg |= (pSPIHandle->SPIConfig.SPI_DFF << 11);

	// CPOL
	tempreg |= (pSPIHandle->SPIConfig.SPI_CPOL << 1);

	// CPHA
	tempreg |= (pSPIHandle->SPIConfig.SPI_CPHA << 0);

	// SSM
	tempreg |= (pSPIHandle->SPIConfig.SPI_SSM << 9);

	pSPIHandle->pSPIx->CR[0] = tempreg;

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

/**************** SPI SendData ****************
 *
 * @fn				- SPI_SendData (Blocking/Polling Call)
 *
 * @brief			- This function transmits data of size len into Shift register for it to be reached on the outside world
 *
 * @param[in]		- SPI base address
 * @param[in]		- Data to be transmitted
 * @param[in]		- Number of bytes transmitted
 *
 * @return			- none
 *
 * @note			- none
 *
 */
void SPI_SendData(SPIx_RegDef_t* pSPIx, uint8_t* pTxBuffer, uint32_t len){

	while(len > 0){

		//Check for TXE flag - Transmitter buffer is empty
		while( ! ( pSPIx->SR & (1 << 1) ));

		//Check if 8bit or 16bit
		if( (pSPIx->CR[0] & (1 << 11) ) ){
			//16 bit
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			len--;
			(uint16_t*)pTxBuffer++;
		} else {
			//8 bit
			pSPIx->DR = *pTxBuffer;
			pTxBuffer++;
		}
		len--;
	}

}

/**************** SPI Peripheral Ctrl ****************
 *
 * @fn				- SPI_PeripheralCtrl
 *
 * @brief			- Enables or disables SPI peripheral for starting communication or stop for configuration
 *
 * @param[in]		- SPI base address
 * @param[in]		- ENABLE or DISABLE
 * @param[in]		-
 *
 * @return			- none
 *
 * @note			- none
 *
 */

void SPI_PeripheralCtrl(SPIx_RegDef_t* pSPIx, uint8_t state){
	if(state == ENABLE){
		pSPIx->CR[0] |= (1 << 6);
	}else{
		pSPIx->CR[0] &= ~(1 << 6);
	}
}

void SPI_SSIConfig(SPIx_RegDef_t* pSPIx, uint8_t state) {
	if(state == ENABLE){
		pSPIx->CR[0] |= (1 << 8);
	}else{
		pSPIx->CR[0] &= ~(1 << 8);
	}
}

void SPI_SSOEConfig(SPIx_RegDef_t* pSPIx, uint8_t state) {
	if(state == ENABLE){
		pSPIx->CR[1] |= (1 << 2);
	}else{
		pSPIx->CR[1] &= ~(1 << 2);
	}
}


