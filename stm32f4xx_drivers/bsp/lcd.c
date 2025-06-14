/*
 * lcd.c
 *
 *  Created on: Jun 5, 2025
 *      Author: hugo-juarez
 */

#include "lcd.h"
#include <string.h>

// --- Helper function prototypes ---
static void write_4_bits(uint8_t val);

/******************************************
 *        	 	LCD Init
 ******************************************/

void lcd_init(void){
	//Configure GPIO pins
	GPIOx_Handle_t lcd_signal;

	memset(&lcd_signal, 0, sizeof(lcd_signal));

	lcd_signal.pGPIOx = LCD_GPIO_PORT;
	lcd_signal.GPIOx_PinConfig.GPIOx_PinMode = GPIOx_MODE_OUT;
	lcd_signal.GPIOx_PinConfig.GPIOx_PinSpeed = GPIOx_OSPEED_FAST;
	lcd_signal.GPIOx_PinConfig.GPIOx_PinOType = GPIOx_OUT_TYPE_PP;
	lcd_signal.GPIOx_PinConfig.GPIOx_PinPuPdControl = GPIOx_PUPDR_NONE;

	//RS Pin
	lcd_signal.GPIOx_PinConfig.GPIOx_PinNumber = LCD_GPIO_RS;
	GPIOx_Init(&lcd_signal);

	//RW Pin
	lcd_signal.GPIOx_PinConfig.GPIOx_PinNumber = LCD_GPIO_RW;
	GPIOx_Init(&lcd_signal);

	//EN Pin
	lcd_signal.GPIOx_PinConfig.GPIOx_PinNumber = LCD_GPIO_EN;
	GPIOx_Init(&lcd_signal);

	//D4 Pin
	lcd_signal.GPIOx_PinConfig.GPIOx_PinNumber = LCD_GPIO_D4;
	GPIOx_Init(&lcd_signal);

	//D5 Pin
	lcd_signal.GPIOx_PinConfig.GPIOx_PinNumber = LCD_GPIO_D5;
	GPIOx_Init(&lcd_signal);

	//D6 Pin
	lcd_signal.GPIOx_PinConfig.GPIOx_PinNumber = LCD_GPIO_D6;
	GPIOx_Init(&lcd_signal);

	//D7 Pin
	lcd_signal.GPIOx_PinConfig.GPIOx_PinNumber = LCD_GPIO_D7;
	GPIOx_Init(&lcd_signal);

	//Reset all pins to 0
	GPIOx_WritePin(LCD_GPIO_PORT, LCD_GPIO_RS, RESET);
	GPIOx_WritePin(LCD_GPIO_PORT, LCD_GPIO_RW, RESET);
	GPIOx_WritePin(LCD_GPIO_PORT, LCD_GPIO_EN, RESET);
	GPIOx_WritePin(LCD_GPIO_PORT, LCD_GPIO_D4, RESET);
	GPIOx_WritePin(LCD_GPIO_PORT, LCD_GPIO_D5, RESET);
	GPIOx_WritePin(LCD_GPIO_PORT, LCD_GPIO_D6, RESET);
	GPIOx_WritePin(LCD_GPIO_PORT, LCD_GPIO_D7, RESET);

	//LCD Initialization

	//40 ms delay
	mdelay(40);

	//RS = 0 For LCD Command
	GPIOx_WritePin(LCD_GPIO_PORT, LCD_GPIO_RS, RESET);

	//RW=0 Writigin to LCD
	GPIOx_WritePin(LCD_GPIO_PORT, LCD_GPIO_RW, RESET);

	//Write initialization
	write_4_bits(0x3);

	//5 ms delay
	mdelay(5);

	//Write initialization
	write_4_bits(0x3);

	//150 us delay
	udelay(150);

	//Write initialization
	write_4_bits(0x3);

	//Method is 4 bit
	write_4_bits(0x2);
}

/******************************************
 *        	 Helper Functions
 ******************************************/

// --- Write 4 Bits ---
static void write_4_bits(uint8_t val){
	GPIOx_WritePin(LCD_GPIO_PORT, LCD_GPIO_D4, ((val >> 0) & 0x1));
	GPIOx_WritePin(LCD_GPIO_PORT, LCD_GPIO_D5, ((val >> 1) & 0x1));
	GPIOx_WritePin(LCD_GPIO_PORT, LCD_GPIO_D6, ((val >> 2) & 0x1));
	GPIOx_WritePin(LCD_GPIO_PORT, LCD_GPIO_D7, ((val >> 3) & 0x1));

	lcd_enable();
}

// --- Send Command ---
static void lcd_send_command(uint8_t cmd){
	//RS=0 For sending commands
	GPIOx_WritePin(LCD_GPIO_PORT, LCD_GPIO_RS, RESET);

	//RW=0 Writing commands
	GPIOx_WritePin(LCD_GPIO_PORT, LCD_GPIO_RW, RESET);

	//Sending higher nibble
	write_4_bits(cmd >> 4);

	//Sending lower nibble
	write_4_bits(cmd & 0xF);
}

// --- LCD Enable ---
static void lcd_enable(void){
	//Hight to low in Enable line
	GPIOx_WritePin(LCD_GPIO_PORT, LCD_GPIO_EN, SET);
	udelay(10);
	GPIOx_WritePin(LCD_GPIO_PORT, LCD_GPIO_EN, RESET);
	mdelay(100);
}

