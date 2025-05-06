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

/**************** SPI ReceiveData ****************
 *
 * @fn				- SPI_ReceiveData (Blocking/Polling Call)
 *
 * @brief			- This function receives data of size len comming from the outside world
 *
 * @param[in]		- SPI base address
 * @param[in]		- Receiving data buffer array
 * @param[in]		- Length of data received
 *
 * @return			- none
 *
 * @note			- none
 *
 */

void SPI_ReceiveData(SPIx_RegDef_t* pSPIx, uint8_t* pRxBuffer, uint32_t len){
	while(len > 0){

		//Check for RXNE flag - Receiver buffer is not empty
		while( ! ( pSPIx->SR & (1 << 0) ));

		//Check if 8bit or 16bit
		if( (pSPIx->CR[0] & (1 << 11) ) ){
			//16 bit
			//Load data from DR into RxBuffer address
			*((uint16_t*)pRxBuffer) = pSPIx->DR;
			len--;
			(uint16_t*)pRxBuffer++;
		} else {
			//8 bit
			*pRxBuffer = pSPIx->DR;
			pRxBuffer++;
		}
		len--;
	}
}

/**************** SPI SendData Interrupt ****************
 *
 * @fn				- SPI_SendDataIT (Non-Blocking/Interrupt Call)
 *
 * @brief			- This function transmits data of size len into Shift register for it to be reached on the outside world
 *
 * @param[in]		- SPI handle address
 * @param[in]		- Data to be transmitted
 * @param[in]		- Number of bytes transmitted
 *
 * @return			- TxState
 *
 * @note			- none
 *
 */
uint8_t SPI_SendDataIT(SPIx_Handle_t* pSPIHandle, uint8_t* pTxBuffer, uint32_t len){

	uint8_t state = pSPIHandle->TxState;

	if(state == SPI_BUSY_IN_TX){
		return state;
	}
	//1. Save TxBuffer and Len in global vairables
	pSPIHandle->pTxBuffer = pTxBuffer;
	pSPIHandle->TxLen = len;

	//2. Mask the SPI state as bussy in transmission so that no other code can take over same SPI peripheral
	//until the transmision is over
	pSPIHandle->TxState = SPI_BUSY_IN_TX;

	//3. Enable TXEIE control bit to get interrupt whenever TXE flag is set in SR
	pSPIHandle->pSPIx->CR[1] |= (1 << 7);

	return state;
}

/**************** SPI ReceiveData Interrupt ****************
 *
 * @fn				- SPI_ReceiveDataIT (Non-Blocking/Interrupt Call)
 *
 * @brief			- This function receives data of size len comming from the outside world
 *
 * @param[in]		- SPI handle address
 * @param[in]		- Receiving data buffer array
 * @param[in]		- Length of data received
 *
 * @return			- none
 *
 * @note			- none
 *
 */

uint8_t SPI_ReceiveDataIT(SPIx_Handle_t* pSPIHandle, uint8_t* pRxBuffer, uint32_t len){
	uint8_t state = pSPIHandle->RxState;

	if(state == SPI_BUSY_IN_RX){
		return state;
	}
	//1. Save TxBuffer and Len in global vairables
	pSPIHandle->pRxBuffer = pRxBuffer;
	pSPIHandle->RxLen = len;

	//2. Mask the SPI state as bussy in transmission so that no other code can take over same SPI peripheral
	//until the transmision is over
	pSPIHandle->RxState = SPI_BUSY_IN_RX;

	//3. Enable TXEIE control bit to get interrupt whenever TXE flag is set in SR
	pSPIHandle->pSPIx->CR[1] |= (1 << 6);

	return state;
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

void SPIx_IRQInterruptConfig(uint8_t IRQNumber, uint8_t state){
	uint8_t temp1 = IRQNumber / 32;
	uint8_t temp2 = IRQNumber % 32;

	if(state == ENABLE){
		NVIC->ISER[temp1] |= (1 << temp2);
	} else {
		NVIC->ICER[temp1] |= (1 << temp2);
	}
}

void SPIx_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority){
	NVIC->IPR[IRQNumber] = (IRQPriority << (8 - NO_PR_BITS_IMPLEMENTED));
}

//Private Helper Function for SPI Interrupt Handle
static void spi_txe_interrupt_handle(SPIx_Handle_t *pHandle);
static void spi_rxne_interrupt_handle(SPIx_Handle_t *pHandle);
static void spi_ovr_interrupt_handle(SPIx_Handle_t *pHandle);

void SPIx_IRQHandling(SPIx_Handle_t *pHandle){

	//Check for TXE
	uint8_t temp1 = pHandle->pSPIx->SR & (1 << 1); //TXE Flag
	uint8_t temp2 = pHandle->pSPIx->CR[1] & (1 << 7); //TXEIE Interrupt Enable

	if(temp1 & temp2){
		//Handle TXE
		spi_txe_interrupt_handle(pHandle);
	}

	//Check for RXNE
	temp1 = pHandle->pSPIx->SR & (1 << 0); //RXNE Flag
	temp2 = pHandle->pSPIx->CR[1] & (1 << 6); //RXNEIE Interrupt Enable

	if(temp1 & temp2){
		//Handle RXNE
		spi_rxne_interrupt_handle(pHandle);
	}

	//Check for OVR
	temp1 = pHandle->pSPIx->SR & (1 << 6); //OVR Flag
	temp2 = pHandle->pSPIx->CR[1] & (1 << 5); //ERROR Interrupt Enable

	if(temp1 & temp2){
		//Handle OVR
		spi_ovr_interrupt_handle(pHandle);
	}
}

static void spi_txe_interrupt_handle(SPIx_Handle_t *pHandle){

	//Check if 8bit or 16bit
	if( (pHandle->pSPIx->CR[0] & (1 << 11) ) ){
		//16 bit
		pHandle->pSPIx->DR = *((uint16_t*)pHandle->pTxBuffer);
		pHandle->TxLen--;
		(uint16_t*)pHandle->pTxBuffer++;
	} else {
		//8 bit
		pHandle->pSPIx->DR = *pHandle->pTxBuffer;
		pHandle->pTxBuffer++;
	}
	pHandle->TxLen--;

	if(!pHandle->TxLen){
		//If TXLen is 0, the we close the spi transmission and inform the application that
		//Tx is over
		SPI_CloseTransmission(pHandle);
		SPI_ApplicationEventCallback(pHandle, SPI_EVENT_TX_CMPLT);
	}


}
static void spi_rxne_interrupt_handle(SPIx_Handle_t *pHandle){

	//Check if 8bit or 16bit
	if( (pHandle->pSPIx->CR[0] & (1 << 11) ) ){
		//16 bit
		//Load data from DR into RxBuffer address
		*((uint16_t*)pHandle->pRxBuffer) = pHandle->pSPIx->DR;
		pHandle->RxLen--;
		(uint16_t*)pHandle->pRxBuffer++;
	} else {
		//8 bit
		*pHandle->pRxBuffer = pHandle->pSPIx->DR;
		pHandle->pRxBuffer++;
	}
	pHandle->RxState--;

	if(!pHandle->RxLen){
		//If RXLen is 0, the we close the spi transmission and inform the application that
		//Rx is over
		SPI_CloseReception(pHandle);
		SPI_ApplicationEventCallback(pHandle, SPI_EVENT_RX_CMPLT);
	}
}

static void spi_ovr_interrupt_handle(SPIx_Handle_t *pHandle){

	if(pHandle->TxState != SPI_BUSY_IN_TX){
		SPI_ClearOVRFlag(pHandle);
	}

	SPI_ApplicationEventCallback(pHandle, SPI_EVENT_OVR_ERR);
}

//Callback function
__weak void SPI_ApplicationEventCallback(SPIx_Handle_t* pHandle,uint8_t event){
	//This is a weak implementation of a callback function that the user can use in the application
}


//Clear OVR Flag
void SPI_ClearOVRFlag(SPIx_Handle_t *pHandle){
	uint8_t temp;
	temp = pHandle->pSPIx->DR; //Read from DR
	temp = pHandle->pSPIx->SR; //Read from Flags
	(void)temp;
}

//Transmision Close
void SPI_CloseTransmission(SPIx_Handle_t *pHandle){
	pHandle->pSPIx->CR[1] &= ~(1 << 7);
	pHandle->pTxBuffer = NULL;
	pHandle->TxLen = 0;
	pHandle->TxState = SPI_READY;
}
void SPI_CloseReception(SPIx_Handle_t *pHandle){
	pHandle->pSPIx->CR[1] &= ~(1 << 6); //Reset RXNEIE interrupt flag
	pHandle->pRxBuffer = NULL;
	pHandle->RxLen = 0;
	pHandle->RxState = SPI_READY;
}
