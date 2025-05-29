/*
 * stm32f407xx_rcc_driver.c
 *
 *  Created on: May 29, 2025
 *      Author: hugo-juarez
 */

#include "stm32f407xx_rcc_driver.h"

uint16_t AHB_Prescaler[8] = {2,4,8,16,64,128,256,512};
uint16_t APB_Prescaler[4] = {2,4,8,16};


uint32_t RCC_GetPLLOutputClock(void){
	return 16;
}//To be implemented

uint32_t RCC_GetPCLK1Value(void){

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

uint32_t RCC_GetPCLK2Value(void){

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

	temp = (RCC->CFGR >> 13) & 0x7;

	if(temp < 4){
		apb = 1;
	} else{
		apb = APB_Prescaler[temp - 4];
	}

	pclk1 = (systemclk/ahb)/apb;

	return pclk1;
}
