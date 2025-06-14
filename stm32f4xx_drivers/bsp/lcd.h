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
#define LCD_GPIO_PORT					GPIOD
#define LCD_GPIO_RS				        GPIOx_PIN_NO_0
#define LCD_GPIO_RW				        GPIOx_PIN_NO_1
#define LCD_GPIO_EN				        GPIOx_PIN_NO_2
#define LCD_GPIO_D4				        GPIOx_PIN_NO_3
#define LCD_GPIO_D5				        GPIOx_PIN_NO_4
#define LCD_GPIO_D6				        GPIOx_PIN_NO_5
#define LCD_GPIO_D7				        GPIOx_PIN_NO_6

// --- Commands ---
#define LCD_CMD_DIS_CLEAR				0x01
#define LCD_CMD_DIS_RETURN_HOME			0x02
#define LCD_CMD_IN_ADD					0x06
#define LCD_CMD_DION_CURON				0x0E
#define LCD_CMD_4DL_2N_5X8F				0x28

/******************************************
 *              	APIS
 ******************************************/

void lcd_init(void);
void lcd_send_command(uint8_t cmd);
void lcd_send_char(uint8_t data);
void lcd_send_string(char *message);
void lcd_display_clear(void);
void lcd_return_home(void);
void lcd_set_cursor(uint8_t row, uint8_t col);

#endif /* LCD_H_ */
