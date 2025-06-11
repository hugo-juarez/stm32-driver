/*
 * ds1307.c
 *
 *  Created on: Jun 5, 2025
 *      Author: hugo-juarez
 */

#include "ds1307.h"
#include <string.h>
#include <stdint.h>

// --- Helper Function Prototypes ---
static void ds1307_i2c_pin_config(void);
static void ds1307_i2c_config(void);
static void ds1307_write(uint8_t value, uint8_t reg_addr);
static uint8_t ds1307_read(uint8_t reg_addr);
static uint8_t binary_to_bcd(uint8_t x);
static uint8_t bcd_to_binary(uint8_t x);

// --- Global Variables ---
I2Cx_Handle_t g_ds1307I2CHandle;

/******************************************
 *              	Init
 ******************************************/

uint8_t ds1307_init(void){
	//Initialize I2C Pins
	ds1307_i2c_pin_config();

	//Initialize I2C Peripheral
	ds1307_i2c_config();

	//Enabel the I2C Peripheral
	I2C_PeripheralCtrl(g_ds1307I2CHandle.pI2C, ENABLE);

	//Make Clock Halt = 0 Initialize Time Count
	ds1307_write(0, DS1307_ADDR_SEC);

	//Read back clock halt bit
	uint8_t clock_state = ds1307_read(DS1307_ADDR_SEC);

	/*
	 * CH = 1 ; Init Failed
	 * CH = 0 ; Init Success
	 */

	return clock_state >> 7 & 0x1;
}


/******************************************
 *        	 Set/Get Current Time
 ******************************************/
// --- Set Current Time ---
void ds1307_set_current_time(RTC_time_t *pRTCTime){

	uint8_t seconds, hrs;

	//Seconds
	seconds = binary_to_bcd(pRTCTime->seconds);
	seconds &= ~(1 << 7);
	ds1307_write(seconds, DS1307_ADDR_SEC);

	//Minutes
	ds1307_write(binary_to_bcd(pRTCTime->minutes), DS1307_ADDR_MIN);

	//Hours
	hrs = binary_to_bcd(pRTCTime->hours);

	if(pRTCTime->time_format == TIME_FORMAT_24HRS){
		//24 HR Format
		hrs &= ~(1 << 6);
	} else {
		//12 HR format
		hrs |= (1 << 6);

		//PM OR AM
		hrs = (pRTCTime->time_format == TIME_FORMAT_12HRS_PM) ? hrs | (1 << 5) : hrs & ~(1 << 5);
	}

	ds1307_write(hrs, DS1307_ADDR_HRS);
}

// --- Get Current Time ---
void ds1307_get_current_time(RTC_time_t *pRTCTime){

	uint8_t seconds, hrs;

	//Seconds
	seconds = ds1307_read(DS1307_ADDR_SEC);
	seconds &= ~(1 << 7);
	pRTCTime->seconds = bcd_to_binary(seconds);

	//Minutes
	pRTCTime->minutes = bcd_to_binary(ds1307_read(DS1307_ADDR_MIN));

	//Hours
	hrs = ds1307_read(DS1307_ADDR_HRS);

	if(hrs & (1 << 6)){
		//12 HR format
		//AM or PM
		pRTCTime->time_format = (hrs & (1 << 5)) ? TIME_FORMAT_12HRS_PM : TIME_FORMAT_12HRS_AM;

	} else{
		//24 Hour format
		pRTCTime->time_format = TIME_FORMAT_24HRS;
	}

	//Clear bits
	hrs &= ~(3<<5);

	pRTCTime->hours = bcd_to_binary(hrs);
}


/******************************************
 *        	 Set/Get Current Date
 ******************************************/
// --- Set Current Date ---
void ds1307_set_current_date(RTC_date_t *pRTCDate){

	//Day
	ds1307_write(binary_to_bcd(pRTCDate->day), DS1307_ADDR_DAY);

	//Date
	ds1307_write(binary_to_bcd(pRTCDate->date), DS1307_ADDR_DATE);

	//Month
	ds1307_write(binary_to_bcd(pRTCDate->month), DS1307_ADDR_MONTH);

	//Year
	ds1307_write(binary_to_bcd(pRTCDate->year), DS1307_ADDR_YEAR);

}

// --- Get Current Date ---
void ds1307_get_current_date(RTC_date_t *pRTCDate){

	//Day
	pRTCDate->day = bcd_to_binary(ds1307_read(DS1307_ADDR_DAY));

	//Date
	pRTCDate->date = bcd_to_binary(ds1307_read(DS1307_ADDR_DATE));

	//Month
	pRTCDate->month = bcd_to_binary(ds1307_read(DS1307_ADDR_MONTH));

	//Year
	pRTCDate->year = bcd_to_binary(ds1307_read(DS1307_ADDR_YEAR));

}

/******************************************
 *        	 Helper Functions
 ******************************************/

// --- I2C Pin Initialization ---
static void ds1307_i2c_pin_config(void){

	GPIOx_Handle_t i2c_sda, i2c_scl;

	// Set initial values to 0
	memset(&i2c_sda, 0, sizeof(i2c_sda));
	memset(&i2c_scl, 0, sizeof(i2c_scl));

	/*
	 * I2C1_SCL ==> PB6
	 * I2C1_SDA ==> PB7
	 */

	// SDA PIN Config
	i2c_sda.pGPIOx = DS1307_I2C_GPIO_PORT;
	i2c_sda.GPIOx_PinConfig.GPIOx_PinAltFunMode = 0x4;
	i2c_sda.GPIOx_PinConfig.GPIOx_PinMode = GPIOx_MODE_ALTFN;
	i2c_sda.GPIOx_PinConfig.GPIOx_PinNumber = DS1307_I2C_SDA_PIN;
	i2c_sda.GPIOx_PinConfig.GPIOx_PinOType = GPIOx_OUT_TYPE_OD;
	i2c_sda.GPIOx_PinConfig.GPIOx_PinPuPdControl = DS1307_I2C_PUPD;
	i2c_sda.GPIOx_PinConfig.GPIOx_PinSpeed = GPIOx_OSPEED_FAST;

	GPIOx_Init(&i2c_sda);

	// SCL PIN Config
	i2c_scl.pGPIOx = DS1307_I2C_GPIO_PORT;
	i2c_scl.GPIOx_PinConfig.GPIOx_PinAltFunMode = 0x4;
	i2c_scl.GPIOx_PinConfig.GPIOx_PinMode = GPIOx_MODE_ALTFN;
	i2c_scl.GPIOx_PinConfig.GPIOx_PinNumber = DS1307_I2C_SCL_PIN;
	i2c_scl.GPIOx_PinConfig.GPIOx_PinOType = GPIOx_OUT_TYPE_OD;
	i2c_scl.GPIOx_PinConfig.GPIOx_PinPuPdControl = DS1307_I2C_PUPD;
	i2c_scl.GPIOx_PinConfig.GPIOx_PinSpeed = GPIOx_OSPEED_FAST;

	GPIOx_Init(&i2c_scl);

}

// --- I2C Peripheral Initialization ---
static void ds1307_i2c_config(void){
	g_ds1307I2CHandle.pI2C = DS1307_I2C;
	g_ds1307I2CHandle.I2C_Config.I2C_SCLSpeed = DS1307_I2C_SPEED;
	g_ds1307I2CHandle.I2C_Config.I2C_ACKControl = I2C_ACK_EN;

	I2C_Init(&g_ds1307I2CHandle);
}

// --- DS1307 Write ---
static void ds1307_write(uint8_t value, uint8_t reg_addr){
	uint8_t tx[2];
	tx[0] = reg_addr;
	tx[1] = value;
	I2C_MasterSendData(&g_ds1307I2CHandle, tx, 2, DS1307_I2C_ADDR, DISABLE);
}

// --- DS1307 Read ---
static uint8_t ds1307_read(uint8_t reg_addr){
	uint8_t data;
	//Sending reg address we wanna read at
	I2C_MasterSendData(&g_ds1307I2CHandle, &reg_addr, 1, DS1307_I2C_ADDR, ENABLE);

	//Read Data
	I2C_MasterReceiveData(&g_ds1307I2CHandle, &data, 1, DS1307_I2C_ADDR, DISABLE);

	return data;

}

// --- Binary to BCD ---
static uint8_t binary_to_bcd(uint8_t x){
	uint8_t m = x/10;
	uint8_t n = x%10;

	return (m << 4) | n;
}

static uint8_t bcd_to_binary(uint8_t x){
	uint8_t m = (x >> 4) * 10;
	uint8_t n = x & 0xF;

	return m + n;
}

