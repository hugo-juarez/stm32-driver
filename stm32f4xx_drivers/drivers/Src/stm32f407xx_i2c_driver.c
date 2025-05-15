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

void I2Cx_Init(I2Cx_Handle_t* pI2CHandle){

	I2Cx_PCLKControl(pI2CHandle->pI2C, ENABLE);

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

void I2Cx_DeInit(I2Cx_RegDef_t* pI2Cx){
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

static void I2C_GenerateStopCondition(I2Cx_RegDef_t* pI2C){
	pI2C->CR1 |= (1 << I2C_CR1_STOP);
}

static void I2C_SetACK(I2Cx_RegDef_t* pI2C, uint8_t status){
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

static void I2C_ClearADDRFlag(I2Cx_RegDef_t* pI2C){
	uint32_t dummyRead;
	//Read SR1
	dummyRead = pI2C->SR1;
	//Read SR2
	dummyRead = pI2C->SR2;

	(void) dummyRead;
}


// Data Send and Receive
void I2C_MasterSendData(I2Cx_Handle_t* pI2CHandle, uint8_t* pTxBuffer, uint32_t len, uint8_t slaveAddr){

	//Generate START condition
	I2C_GenerateStartCondition(pI2CHandle->pI2C);

	//Confirm Start generation is complete
	while(!(pI2CHandle->pI2C->SR1 & I2C_FLAG_SB));

	//SendAddress o the slave with r/nw bit set to w(0)
	I2C_ExecuteAddressPhase(pI2CHandle->pI2C, slaveAddr, WRITE);

	//Confirm address phase is completed so checking ADDR flag in SR1
	while(!(pI2CHandle->pI2C->SR1 & I2C_FLAG_ADDR));

	//Clear ADDR flag
	I2C_ClearADDRFlag(pI2CHandle->pI2C);

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
	I2C_GenerateStopCondition(pI2CHandle->pI2C);

}

void I2C_MasterReceiveData(I2Cx_Handle_t* pI2CHandle, uint8_t* pRxBuffer, uint32_t len, uint8_t slaveAddr){
	//Generate STARt Condition
	I2C_GenerateStartCondition(pI2CHandle->pI2C);

	//Confirm Start generation is complete
	while(!(pI2CHandle->pI2C->SR1 & I2C_FLAG_SB));

	//SendAddress o the slave with r/nw bit set to r(1)
	I2C_ExecuteAddressPhase(pI2CHandle->pI2C, slaveAddr, WRITE);

	//Confirm address phase is completed so checking ADDR flag in SR1
	while(!(pI2CHandle->pI2C->SR1 & I2C_FLAG_ADDR));

	if(len == 1){
		//Disable ACK send no response
		I2C_SetACK(pI2CHandle->pI2C, DISABLE);

		//Clear ADDR flag
		I2C_ClearADDRFlag(pI2CHandle->pI2C);

		//Wait for RXNE=1 Data is in buffer.
		while(!(pI2CHandle->pI2C->SR1 & I2C_FLAG_RXNE));

		//Generate STOP condition
		I2C_GenerateStopCondition(pI2CHandle->pI2C);

		//Read data
		*pRxBuffer = (uint8_t) (pI2CHandle->pI2C->DR & 0xFF);
	} else{
		//Clear ADDR flag
		I2C_ClearADDRFlag(pI2CHandle->pI2C);

		while(len > 0){
			//Wait for RXNE=1 Data is in buffer.
			while(!(pI2CHandle->pI2C->SR1 & I2C_FLAG_RXNE));

			if(len == 2){
				//Disable ACK send no response
				I2C_SetACK(pI2CHandle->pI2C, DISABLE);

				//Generate STOP condition
				I2C_GenerateStopCondition(pI2CHandle->pI2C);

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

//Peripheral Control
void I2C_PeripheralCtrl(I2Cx_RegDef_t* pI2Cx, uint8_t state){
	if(state == ENABLE){
		pI2Cx->CR1 |= (1 << I2C_CR1_PE);
	}else{
		pI2Cx->CR1 &= ~(1 << I2C_CR1_PE);
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
