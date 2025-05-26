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
void I2C_PCLKControl(I2Cx_RegDef_t* pI2Cx, uint8_t state){
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

void I2C_Init(I2Cx_Handle_t* pI2CHandle){

	I2C_PCLKControl(pI2CHandle->pI2C, ENABLE);

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
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM){
		//Standard Mode
		tempreg = ( (pclk*1000)/1000000000U ) + 1;
	} else{
		//Fast Mode
		tempreg = ( (pclk*300)/1000000000U ) + 1;
	}

	pI2CHandle->pI2C->TRISE = (tempreg & 0x3F);

}

void I2C_DeInit(I2Cx_RegDef_t* pI2Cx){
	if(pI2Cx == I2C1){
		I2C1_REG_RESET();
	} else if(pI2Cx == I2C2){
		I2C2_REG_RESET();
	} else if(pI2Cx == I2C3){
		I2C3_REG_RESET();
	}
}

//DATA READ AND WRITE PRIVATE FUNCTION

static void I2C_GenerateStartCondition(I2Cx_RegDef_t* pI2C){
	pI2C->CR1 |= (1 << I2C_CR1_START);
}

void I2C_GenerateStopCondition(I2Cx_RegDef_t* pI2C){
	pI2C->CR1 |= (1 << I2C_CR1_STOP);
}

void I2C_SetACK(I2Cx_RegDef_t* pI2C, uint8_t status){
	if(status == ENABLE){
		pI2C->CR1 |= (1 << I2C_CR1_ACK);
	}else{
		pI2C->CR1 &= ~(1 << I2C_CR1_ACK);
	}
}

static void I2C_ExecuteAddressPhase(I2Cx_RegDef_t* pI2C, uint8_t slaveAddr, uint8_t mode){
	slaveAddr = slaveAddr << 1;
	if(mode == READ){
		slaveAddr |= (1 << 0);
	}else{
		slaveAddr &= ~(1 << 0);
	}
	pI2C->DR = slaveAddr;
}

static void I2C_ClearADDRFlag(I2Cx_Handle_t* pI2CHandle){
	//check for device mode: master/slave
	uint32_t dummy_read;

	if((pI2CHandle->pI2C->SR2 & (1 << I2C_SR2_MSL)) && pI2CHandle->TxRxState == I2C_BUSY_IN_RX && pI2CHandle->RxSize == 1){
		//Master mode, BUSYINRX=TRUE, RxLen=1

		//Disable ACK
		I2C_SetACK(pI2CHandle->pI2C, DISABLE);

	}

	//Clear ADDR FLAG
	dummy_read = pI2CHandle->pI2C->SR1;
	dummy_read = pI2CHandle->pI2C->SR2;
	(void) dummy_read;

}

static void I2C_ClearStopFlag(I2Cx_RegDef_t* pI2C){
	uint32_t dummyRead;
	//Read SR1
	dummyRead = pI2C->SR1;
	//Read SR2
	dummyRead = pI2C->SR2;

	(void) dummyRead;
}

static void I2C_MasterHandleTXEInterrupt(I2Cx_Handle_t* pI2CHandle){

	if(pI2CHandle->TxLen > 0){

		//Load data into DR
		pI2CHandle->pI2C->DR = *(pI2CHandle->pTxBuffer);

		//Decrement TxLen
		pI2CHandle->TxLen--;

		//Incremenet buffer address
		pI2CHandle->pTxBuffer++;

	}
}

static void I2C_MasterHandleRXNEInterrupt(I2Cx_Handle_t* pI2CHandle){
	if(pI2CHandle->RxSize == 1)
	{
		if(pI2CHandle->RS == I2C_DISABLE_RS)
			I2C_GenerateStopCondition(pI2CHandle->pI2C);

		*pI2CHandle->pRxBuffer = pI2CHandle->pI2C->DR;
		pI2CHandle->RxLen--;

	} else if(pI2CHandle->RxSize > 1)
	{

		if(pI2CHandle->RxLen == 2){

			//Disable ACK
			I2C_SetACK(pI2CHandle->pI2C, DISABLE);

			if(pI2CHandle->RS == I2C_DISABLE_RS)
				I2C_GenerateStopCondition(pI2CHandle->pI2C);
		}

		*pI2CHandle->pRxBuffer = pI2CHandle->pI2C->DR;
		pI2CHandle->pRxBuffer++;
		pI2CHandle->RxLen--;

	}

	if(pI2CHandle->RxLen == 0){
		//Close I2C data reception and notify the application

		//1. Close the I2C Rx
		I2C_CloseReceiveData(pI2CHandle);

		//2. Notify the application
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_RX_CMPLT);
	}
}

/************************************************************************************
 * 							MASTER SEND/RECEIVE DATA
 ************************************************************************************/

// Data Send and Receive
void I2C_MasterSendData(I2Cx_Handle_t* pI2CHandle, uint8_t* pTxBuffer, uint32_t len, uint8_t slaveAddr, uint8_t repeatedStart){

	//Generate START condition
	I2C_GenerateStartCondition(pI2CHandle->pI2C);

	//Confirm Start generation is complete
	while(!(pI2CHandle->pI2C->SR1 & I2C_FLAG_SB));

	//SendAddress o the slave with r/nw bit set to w(0)
	I2C_ExecuteAddressPhase(pI2CHandle->pI2C, slaveAddr, WRITE);

	//Confirm address phase is completed so checking ADDR flag in SR1
	while(!(pI2CHandle->pI2C->SR1 & I2C_FLAG_ADDR));

	//Clear ADDR flag
	I2C_ClearADDRFlag(pI2CHandle);

	//Send data until len becomes 0:
	while(len > 0){
		while(!(pI2CHandle->pI2C->SR1 & I2C_FLAG_TXE)); //Wait till TxE flag is set emaning the buffer is empty
		pI2CHandle->pI2C->DR = *pTxBuffer;
		pTxBuffer++;
		len--;
	}

	//When Len is 0 wait for TXE=1 and BTF=1 meanst that next transmission can start
	while(!(pI2CHandle->pI2C->SR1 & I2C_FLAG_TXE));
	while(!(pI2CHandle->pI2C->SR1 & I2C_FLAG_BTF));

	//Generate STOP condition
	if(repeatedStart == I2C_DISABLE_RS){
		I2C_GenerateStopCondition(pI2CHandle->pI2C);
	}

}

void I2C_MasterReceiveData(I2Cx_Handle_t* pI2CHandle, uint8_t* pRxBuffer, uint32_t len, uint8_t slaveAddr, uint8_t repeatedStart){
	//Generate STARt Condition
	I2C_GenerateStartCondition(pI2CHandle->pI2C);

	//Confirm Start generation is complete
	while(!(pI2CHandle->pI2C->SR1 & I2C_FLAG_SB));

	//SendAddress o the slave with r/nw bit set to r(1)
	I2C_ExecuteAddressPhase(pI2CHandle->pI2C, slaveAddr, READ);

	//Confirm address phase is completed so checking ADDR flag in SR1
	while(!(pI2CHandle->pI2C->SR1 & I2C_FLAG_ADDR));

	if(len == 1){
		//Disable ACK send no response
		I2C_SetACK(pI2CHandle->pI2C, DISABLE);

		//Clear ADDR flag
		I2C_ClearADDRFlag(pI2CHandle);

		//Wait for RXNE=1 Data is in buffer.
		while(!(pI2CHandle->pI2C->SR1 & I2C_FLAG_RXNE));

		//Generate STOP condition
		if(repeatedStart == I2C_DISABLE_RS){
			I2C_GenerateStopCondition(pI2CHandle->pI2C);
		}

		//Read data
		*pRxBuffer = (uint8_t) (pI2CHandle->pI2C->DR & 0xFF);

	} else{
		//Clear ADDR flag
		I2C_ClearADDRFlag(pI2CHandle);

		while(len > 0){
			//Wait for RXNE=1 Data is in buffer.
			while(!(pI2CHandle->pI2C->SR1 & I2C_FLAG_RXNE));

			if(len == 2){
				//Disable ACK send no response
				I2C_SetACK(pI2CHandle->pI2C, DISABLE);

				//Generate STOP condition
				if(repeatedStart == I2C_DISABLE_RS){
					I2C_GenerateStopCondition(pI2CHandle->pI2C);
				}

			}

			//Read data
			*pRxBuffer = (uint8_t) (pI2CHandle->pI2C->DR & 0xFF);
			pRxBuffer++;
			len--;
		}
	}

	//Re-enable ACK
	I2C_SetACK(pI2CHandle->pI2C, ENABLE);


}

/************************************************************************************
 * 							SLAVE SEND/RECEIVE DATA
 ************************************************************************************/

void I2C_SlaveSendData(I2Cx_RegDef_t* pI2C, uint8_t data){
	pI2C->DR = data;
}
uint8_t I2C_SlaveReceiveData(I2Cx_RegDef_t* pI2C){
	return pI2C->DR;
}

void I2C_SlaveCallbackEvents(I2Cx_RegDef_t* pI2C, uint8_t event){
	if(event == ENABLE){
		pI2C->CR2 |= (1 << I2C_CR2_ITEVTEN);
		pI2C->CR2 |= (1 << I2C_CR2_ITBUFEN);
		pI2C->CR2 |= (1 << I2C_CR2_ITERREN);
	}else {
		pI2C->CR2 &= ~(1 << I2C_CR2_ITEVTEN);
		pI2C->CR2 &= ~(1 << I2C_CR2_ITBUFEN);
		pI2C->CR2 &= ~(1 << I2C_CR2_ITERREN);
	}
}



/************************************************************************************
 * 							PERIPEHRAL CTRL
 ************************************************************************************/

void I2C_PeripheralCtrl(I2Cx_RegDef_t* pI2Cx, uint8_t state){
	if(state == ENABLE){
		pI2Cx->CR1 |= (1 << I2C_CR1_PE);
	}else{
		pI2Cx->CR1 &= ~(1 << I2C_CR1_PE);
	}
}

/************************************************************************************
 * 							IRQ CONFIG/PRIO
 ************************************************************************************/

void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t state){
	uint8_t temp1 = IRQNumber / 32;
	uint8_t temp2 = IRQNumber % 32;

	if(state == ENABLE){
		NVIC->ISER[temp1] = (1 << temp2);
	} else {
		NVIC->ICER[temp1] = (1 << temp2);
	}
}
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority){
	NVIC->IPR[IRQNumber] = (IRQPriority << (8 - NO_PR_BITS_IMPLEMENTED));
}

/*
 *  Data Send and Receive
 *  With INTERRUPTS
 */

uint8_t I2C_MasterSendDataIT(I2Cx_Handle_t* pI2CHandle, uint8_t* pTxBuffer, uint32_t len, uint8_t slaveAddr,  uint8_t repeatedStart){

	uint8_t busystate = pI2CHandle->TxRxState;

	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX) ){

		pI2CHandle->TxRxState = I2C_BUSY_IN_TX;
		pI2CHandle->pTxBuffer = pTxBuffer;
		pI2CHandle->TxLen = len;
		pI2CHandle->DevAddr = slaveAddr;
		pI2CHandle->RS = repeatedStart;

		//Enable Buffer interrupt ITBUFEN
		pI2CHandle->pI2C->CR2 |= (1 << I2C_CR2_ITBUFEN);

		//Enable Event interrupt ITEVTEN
		pI2CHandle->pI2C->CR2 |= (1 << I2C_CR2_ITEVTEN);

		//Enable Error interrupt ITERREN
		pI2CHandle->pI2C->CR2 |= (1 << I2C_CR2_ITERREN);

		//Generate START Condition
		I2C_GenerateStartCondition(pI2CHandle->pI2C);
	}

	return busystate;
}

uint8_t I2C_MasterReceiveDataIT(I2Cx_Handle_t* pI2CHandle, uint8_t* pRxBuffer, uint32_t len, uint8_t slaveAddr,  uint8_t repeatedStart){

	uint8_t busystate = pI2CHandle->TxRxState;

	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX) ){

		pI2CHandle->TxRxState = I2C_BUSY_IN_RX;
		pI2CHandle->pRxBuffer = pRxBuffer;
		pI2CHandle->RxLen = len;
		pI2CHandle->RxSize = len;
		pI2CHandle->DevAddr = slaveAddr;
		pI2CHandle->RS = repeatedStart;

		//Generate START Condition
		I2C_GenerateStartCondition(pI2CHandle->pI2C);

		//Enable Buffer interrupt ITBUFEN
		pI2CHandle->pI2C->CR2 |= (1 << I2C_CR2_ITBUFEN);

		//Enable Event interrupt ITEVTEN
		pI2CHandle->pI2C->CR2 |= (1 << I2C_CR2_ITEVTEN);

		//Enable Error interrupt ITERREN
		pI2CHandle->pI2C->CR2 |= (1 << I2C_CR2_ITERREN);
	}

	return busystate;

}

//IRQ Handling APIs

void I2C_EV_IRQHandling(I2Cx_Handle_t* pI2CHandle){

	/*
	 * Interrupt handling by event can happen for different reasons so first let's
	 * decode and handle for each interrupt
	 */

	uint32_t temp1, temp2, temp3;

	//First let's check that the ITEVTEN and ITBUFEN flag is enabled
	temp1 = pI2CHandle->pI2C->CR2 & (1 << I2C_CR2_ITEVTEN);
	temp2 = pI2CHandle->pI2C->CR2 & (1 << I2C_CR2_ITBUFEN);

	//1. Handle interrupt generated by SB event
	//Note:	When master mode: Start condition generated
	//		When slave mode : This block will not be executed
	temp3 = pI2CHandle->pI2C->SR1 & (1 << I2C_SR1_SB);

	if(temp1 && temp3){
		//SB Flag is set
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX){
			I2C_ExecuteAddressPhase(pI2CHandle->pI2C, pI2CHandle->DevAddr, READ);
		} else if (pI2CHandle->TxRxState == I2C_BUSY_IN_TX){
			I2C_ExecuteAddressPhase(pI2CHandle->pI2C, pI2CHandle->DevAddr, WRITE);
		}

	}

	//2. Handle interrupt generated by ADDR event
	//Note:	When master mode :	Address is sent
	//		When slave mode	 :	Address matched with own address

	temp3 = pI2CHandle->pI2C->SR1 & (1 << I2C_SR1_ADDR);

	if(temp1 && temp3){
		//ADDR Flag is set
		I2C_ClearADDRFlag(pI2CHandle);
	}

	//3. Handle interrupt generated by BTF event

	temp3 = pI2CHandle->pI2C->SR1 & (1 << I2C_SR1_BTF);

	if(temp1 && temp3){
		//BTF Flag is set
		if (pI2CHandle->TxRxState == I2C_BUSY_IN_TX &&
				(pI2CHandle->pI2C->SR1 & (1 << I2C_SR1_TXE)) &&
				pI2CHandle->TxLen == 0){
			//BTF, TXE = 1
			//1. Generate stop condition
			if(pI2CHandle->RS == I2C_DISABLE_RS)
				I2C_GenerateStopCondition(pI2CHandle->pI2C);

			//2. Reset all the member elements of the handle structure
			I2C_CloseSendData(pI2CHandle);

			//3. Notify the application about transmission complete
			I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_TX_CMPLT);

		}
	}

	//4. Handle interrupt generated by STOPF event
	//Note:	When master mode :	This won't trigger on master mode
	//		When slave mode	 :	Stop condition is detected on the bus by the slave after an acknowledge
	temp3 = pI2CHandle->pI2C->SR1 & (1 << I2C_SR1_STOPF);

	if(temp1 && temp3){
		//STOPF Flag is set

		//Clear Flag
		I2C_ClearStopFlag(pI2CHandle->pI2C);

		//Notify call back function
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_STOP);
	}

	//5. Handle interrupt generated by RxNE event
	temp3 = pI2CHandle->pI2C->SR1 & (1 << I2C_SR1_RXNE);

	if(temp1 && temp2 && temp3){

		//RxNE Flag is set
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX){

			I2C_MasterHandleRXNEInterrupt(pI2CHandle);

		} else {
			//Check master is on write mode
			if( !(pI2CHandle->pI2C->SR2 & (1 << I2C_SR2_TRA)) ){
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_RCV);
			}
		}
	}

	//6. Handle interrupt generated by TxE event
	temp3 = pI2CHandle->pI2C->SR1 & (1 << I2C_SR1_TXE);

	if(temp1 && temp2 && temp3){
		//TxE Flag is set
		//Check we are on TX Mode, Master Mode and we have data to transmit:

		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX && (pI2CHandle->pI2C->SR2 & (1 << I2C_SR2_MSL)) ){
			//Master
			I2C_MasterHandleTXEInterrupt(pI2CHandle);
		} else {
			//Slave
			//Checks if it is in transmitter mode
			if(pI2CHandle->pI2C->SR2 & (1 << I2C_SR2_TRA)){
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_REQ);
			}
		}
	}

}

void I2C_ER_IRQHandling(I2Cx_Handle_t* pI2CHandle){

	uint32_t temp1, temp2;

	temp2 = (pI2CHandle->pI2C->CR2) & (1 << I2C_CR2_ITERREN);

	/*********************************************************
	 * 						BUS ERROR
	 *********************************************************/

	temp1 = (pI2CHandle->pI2C->SR1) & (1 << I2C_SR1_BERR);

	if(temp1 && temp2){

		//Clear bus error
		pI2CHandle->pI2C->SR1 &= ~(1 << I2C_SR1_BERR);

		//Notify application
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ER_BERR);

	}

	/*********************************************************
	 * 				ARBITRATION LOST ERROR
	 *********************************************************/

	temp1 = (pI2CHandle->pI2C->SR1) & (1 << I2C_SR1_ARLO);

	if(temp1 && temp2){

		//Clear error
		pI2CHandle->pI2C->SR1 &= ~(1 << I2C_SR1_ARLO);

		//Notify
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ER_ARLO);

	}

	/*********************************************************
	 *	 				ACK FAILURE ERROR
	 *********************************************************/

	temp1 = (pI2CHandle->pI2C->SR1) & (1 << I2C_SR1_AF);

	if(temp1 && temp2){

		//Clear error
		pI2CHandle->pI2C->SR1 &= ~(1 << I2C_SR1_AF);

		//Notify
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ER_AF);

	}

	/*********************************************************
	 *	 				OVER/UNDER-RUN ERROR
	 *********************************************************/

	temp1 = (pI2CHandle->pI2C->SR1) & (1 << I2C_SR1_OVR);

	if(temp1 && temp2){

		//Clear error
		pI2CHandle->pI2C->SR1 &= ~(1 << I2C_SR1_OVR);

		//Notify
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ER_OVR);

	}

	/*********************************************************
	 *	 				TIMEOUT ERROR
	 *********************************************************/

	temp1 = (pI2CHandle->pI2C->SR1) & (1 << I2C_SR1_TIMEOUT);

	if(temp1 && temp2){

		//Clear error
		pI2CHandle->pI2C->SR1 &= ~(1 << I2C_SR1_TIMEOUT);

		//Notify
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ER_TIMEOUT);

	}

}

void I2C_CloseSendData(I2Cx_Handle_t *pI2CHandle){

	//Disable ITBUFEN
	pI2CHandle->pI2C->CR2 &= ~(1 << I2C_CR2_ITBUFEN);

	//Disable ITEVTEN
	pI2CHandle->pI2C->CR2 &= ~(1 << I2C_CR2_ITEVTEN);

	//Reset Handle
	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pTxBuffer = NULL;
	pI2CHandle->TxLen = 0;

}

void I2C_CloseReceiveData(I2Cx_Handle_t *pI2CHandle){

	//Disable ITBUFEN
	pI2CHandle->pI2C->CR2 &= ~(1 << I2C_CR2_ITBUFEN);

	//Disable ITEVTEN
	pI2CHandle->pI2C->CR2 &= ~(1 << I2C_CR2_ITEVTEN);

	//Reset Handle
	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pRxBuffer = NULL;
	pI2CHandle->RxLen = 0;
	pI2CHandle->RxSize = 0;

	//Reenable ACK
	I2C_SetACK(pI2CHandle->pI2C, ENABLE);

}

__weak void I2C_ApplicationEventCallback(I2Cx_Handle_t* pHandle,uint8_t event){

}




