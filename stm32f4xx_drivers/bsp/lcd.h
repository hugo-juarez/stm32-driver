/*
 * lcd.h
 *
 *  Created on: Jun 5, 2025
 *      Author: hugo-juarez
 */

#ifndef LCD_H_
#define LCD_H_

#include "stm32f407xx.h"

/******************************************
 *              CONFIGURATION
 ******************************************/

// --- Peripheral Config ---
#define LCD_GPIO_PORT			GPIOD
#define LCD_GPIO_RS				GPIOx_PIN_NO_0
#define LCD_GPIO_RW				GPIOx_PIN_NO_1
#define LCD_GPIO_EN				GPIOx_PIN_NO_2
#define LCD_GPIO_D4				GPIOx_PIN_NO_3
#define LCD_GPIO_D5				GPIOx_PIN_NO_4
#define LCD_GPIO_D6				GPIOx_PIN_NO_5
#define LCD_GPIO_D7				GPIOx_PIN_NO_6

/******************************************
 *              	APIS
 ******************************************/

void lcd_init(void);
void lcd_send_command(uint8_t cmd);

#endif /* LCD_H_ */
