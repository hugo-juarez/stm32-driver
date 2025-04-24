/*
 * stm32f407xx.h
 *
 *  Created on: Apr 23, 2025
 *      Author: hugo-juarez
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include <stdint.h>

#define __vo volatile

//Generic macros
#define ENABLE				1
#define DISABLE				0
#define SET					ENABLE
#define RESET				DISABLE

/***********************************************************/
/******************** BASE ADDRESSES ***********************/
/***********************************************************/

// MEMORY ADDRESSES

#define FLASH_BASEADDR				0x08000000UL
#define SRAM1_BASEADDR				0x20000000UL
#define SRAM2_BASEADDR				0x2001C000UL
#define ROM_BASEADDR				0x1FFF0000UL //ROM Base address is the System Memory
#define SRAM						SRAM1_BASEADDR

// PERIPHER ADDRESSES

#define PERIPH_BASEADDR				0X40000000UL
#define APB1_BASEADDR				PERIPH_BASEADDR //APB run in a lower clock speed than AHB
#define APB2_BASEADDR				0X40010000UL
#define AHB1_BASEADDR				0x40015000UL
#define AHB2_BASEADDR				0x50000000UL

// AHB1 ADDRESSES

#define GPIOA_BASEADDR				(AHB1_BASEADDR + 0x0000)
#define GPIOB_BASEADDR				(AHB1_BASEADDR + 0x0400)
#define GPIOC_BASEADDR				(AHB1_BASEADDR + 0x0800)
#define GPIOD_BASEADDR				(AHB1_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR				(AHB1_BASEADDR + 0x1000)
#define GPIOF_BASEADDR				(AHB1_BASEADDR + 0x1400)
#define GPIOG_BASEADDR				(AHB1_BASEADDR + 0x1800)
#define GPIOH_BASEADDR				(AHB1_BASEADDR + 0x1C00)
#define GPIOI_BASEADDR				(AHB1_BASEADDR + 0x2000)
#define GPIOJ_BASEADDR				(AHB1_BASEADDR + 0x2400)
#define GPIOK_BASEADDR				(AHB1_BASEADDR + 0x2800)

#define RCC_BASEADDR				(AHB1_BASEADDR + 0x3800)

// APB1 ADDRESSES

#define I2C1_BASEADDR				(APB1_BASEADDR + 0x5400)
#define I2C2_BASEADDR				(APB1_BASEADDR + 0x5800)
#define I2C3_BASEADDR				(APB1_BASEADDR + 0x5C00)

#define SPI2_BASEADDR				(APB1_BASEADDR + 0x3800)
#define SPI3_BASEADDR				(APB1_BASEADDR + 0x3C00)

#define USART2_BASEADDR				(APB1_BASEADDR + 0x4400)
#define USART3_BASEADDR				(APB1_BASEADDR + 0x4800)
#define UART4_BASEADDR				(APB1_BASEADDR + 0x4C00)
#define UART5_BASEADDR				(APB1_BASEADDR + 0x5000)

// APB2 ADDRESSES

#define EXTI_BASEADDR				(APB2_BASEADDR + 0x3C00)
#define SYSCFG_BASEADDR				(APB2_BASEADDR + 0x3800)
#define SPI1_BASEADDR				(APB2_BASEADDR + 0x3000)

#define USART1_BASEADDR				(APB2_BASEADDR + 0x1000)
#define USART6_BASEADDR				(APB2_BASEADDR + 0x1400)

/***********************************************************/
/************************** RCC ****************************/
/***********************************************************/

#define RCC							((RCC_RefDef_t*)RCC_BASEADDR)

typedef struct {
	__vo uint32_t CR;					//RCC clock control register
	__vo uint32_t PLLCFGR;				//RCC PLL configuration register
	__vo uint32_t CFGR;					//RCC clock configuration register
	__vo uint32_t CIR;					//RCC clock interrupt register
	__vo uint32_t AHB1RSTR;				//RCC AHB1 peripheral reset register
	__vo uint32_t AHB2RSTR;				//RCC AHB2 peripheral reset register
	__vo uint32_t AHB3RSTR;				//RCC AHB3 peripheral reset register
	uint32_t RESERVED0;					//
	__vo uint32_t APB1RSTR;				//RCC APB1 peripheral reset register
	__vo uint32_t APB2RSTR;				//RCC APB2 peripheral reset register
	uint32_t RESERVED1[2];				//
	__vo uint32_t AHB1ENR;				//RCC AHB1 peripheral clock enable register
	__vo uint32_t AHB2ENR;				//RCC AHB2 peripheral clock enable register
	__vo uint32_t AHB3ENR;				//RCC AHB3 peripheral clock enable register
	uint32_t RESERVED2;					//
	__vo uint32_t APB1ENR;				//RCC APB1 peripheral clock enable register
	__vo uint32_t APB2ENR;				//RCC APB2 peripheral clock enable register
	uint32_t RESERVED3[2];				//
	__vo uint32_t AHB1LPENR;			//RCC AHB1 peripheral clock enable in low power mode register
	__vo uint32_t AHB2LPENR;			//RCC AHB2 peripheral clock enable in low power mode register
	__vo uint32_t AHB3LPENR;			//RCC AHB3 peripheral clock enable in low power mode register
	uint32_t RESERVED4;					//
	__vo uint32_t APB1LPENR;			//RCC APB1 peripheral clock enable in low power mode register
	__vo uint32_t APB2LPENR;			//RCC APB2 peripheral clock enable in low power mode register
	uint32_t RESERVED5[2];				//
	__vo uint32_t BDCR;					//RCC Backup domain control register
	__vo uint32_t CSR;					//RCC clock control & status register
	uint32_t RESERVED6[2];				//
	__vo uint32_t SSCGR;				//RCC spread spectrum clock generation register
	__vo uint32_t PLLI2SCFGR;			//RCC PLLI2S configuration register
} RCC_RefDef_t;

/***********************************************************/
/************************* GPIO ****************************/
/***********************************************************/

#define GPIOA						((GPIOx_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB						((GPIOx_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC						((GPIOx_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD						((GPIOx_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE						((GPIOx_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF						((GPIOx_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG						((GPIOx_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH						((GPIOx_RegDef_t*)GPIOH_BASEADDR)
#define GPIOI						((GPIOx_RegDef_t*)GPIOI_BASEADDR)
#define GPIOJ						((GPIOx_RegDef_t*)GPIOJ_BASEADDR)
#define GPIOK						((GPIOx_RegDef_t*)GPIOK_BASEADDR)


typedef struct{
	__vo uint32_t MODER;				//GPIO port mode register
	__vo uint32_t OTYPER;				//GPIO port output type register
	__vo uint32_t OSPEEDR;				//GPIO port output speed register
	__vo uint32_t PUPDR;				//GPIO port pull-up/pull-down register
	__vo uint32_t IDR;					//GPIO port input data register
	__vo uint32_t ODR;					//GPIO port output data register
	__vo uint32_t BSRR;					//GPIO port bit set/reset register
	__vo uint32_t LCKR;					//GPIO port configuration lock register
	__vo uint32_t AFRx[2];				//GPIO alternate function low/high register
} GPIOx_RegDef_t;

/***********************************************************/
/**************** PERIPHERAL CLOCK EN/DI *******************/
/***********************************************************/

// ENABLE GPIOx PERIPHERALS
#define GPIOA_PCLK_EN()				(RCC->AHB1ENR |= 1 << 0)
#define GPIOB_PCLK_EN()				(RCC->AHB1ENR |= 1 << 1)
#define GPIOC_PCLK_EN()				(RCC->AHB1ENR |= 1 << 2)
#define GPIOD_PCLK_EN()				(RCC->AHB1ENR |= 1 << 3)
#define GPIOE_PCLK_EN()				(RCC->AHB1ENR |= 1 << 4)
#define GPIOF_PCLK_EN()				(RCC->AHB1ENR |= 1 << 5)
#define GPIOG_PCLK_EN()				(RCC->AHB1ENR |= 1 << 6)
#define GPIOH_PCLK_EN()				(RCC->AHB1ENR |= 1 << 7)
#define GPIOI_PCLK_EN()				(RCC->AHB1ENR |= 1 << 8)
#define GPIOJ_PCLK_EN()				(RCC->AHB1ENR |= 1 << 9)
#define GPIOK_PCLK_EN()				(RCC->AHB1ENR |= 1 << 10)

// ENABLE I2Cx PERIPHERALS
#define I2C1_PCLK_EN()				(RCC->APB1ENR |= 1 << 21)
#define I2C2_PCLK_EN()				(RCC->APB1ENR |= 1 << 22)
#define I2C3_PCLK_EN()				(RCC->APB1ENR |= 1 << 23)

// ENABLE SPI PERIPHERALS
#define SPI1_PCLK_EN()				(RCC->APB2ENR |= 1 << 12)
#define SPI2_PCLK_EN()				(RCC->APB1ENR |= 1 << 14)
#define SPI3_PCLK_EN()				(RCC->APB1ENR |= 1 << 15)

// ENABLE USART PERIPHERALS
#define USART1_PCLK_EN() 			(RCC->APB2ENR |= 1 << 4)
#define USART2_PCLK_EN()			(RCC->APB1ENR |= 1 << 17)
#define USART3_PCLK_EN()			(RCC->APB1ENR |= 1 << 18)
#define UART4_PCLK_EN()				(RCC->APB1ENR |= 1 << 19)
#define UART5_PCLK_EN()				(RCC->APB1ENR |= 1 << 20)
#define USART6_PCLK_EN() 			(RCC->APB2ENR |= 1 << 5)

// ENABLE SYSCFG PERIPHERALS
#define SYSCFG_PCLK_EN()			(RCC->APB2ENR |= 1 << 14)

// DISABLE GPIOx PERIPHERALS
#define GPIOA_PCLK_DI()				(RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI()				(RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI()				(RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI()				(RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI()				(RCC->AHB1ENR &= ~(1 << 4))
#define GPIOF_PCLK_DI()				(RCC->AHB1ENR &= ~(1 << 5))
#define GPIOG_PCLK_DI()				(RCC->AHB1ENR &= ~(1 << 6))
#define GPIOH_PCLK_DI()				(RCC->AHB1ENR &= ~(1 << 7))
#define GPIOI_PCLK_DI()				(RCC->AHB1ENR &= ~(1 << 8))
#define GPIOJ_PCLK_DI()				(RCC->AHB1ENR &= ~(1 << 9))
#define GPIOK_PCLK_DI()				(RCC->AHB1ENR &= ~(1 << 10))

// DISABLE I2Cx PERIPHERALS
#define I2C1_PCLK_DI()				(RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI()				(RCC->APB1ENR &= ~(1 << 22))
#define I2C3_PCLK_DI()				(RCC->APB1ENR &= ~(1 << 23))

// DISABLE SPI PERIPHERALS
#define SPI1_PCLK_DI()				(RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DI()				(RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DI()				(RCC->APB1ENR &= ~(1 << 15))

// DISABLE USART PERIPHERALS
#define USART1_PCLK_DI() 			(RCC->APB2ENR &= ~(1 << 4))
#define USART2_PCLK_DI()			(RCC->APB1ENR &= ~(1 << 17))
#define USART3_PCLK_DI()			(RCC->APB1ENR &= ~(1 << 18))
#define UART4_PCLK_DI()				(RCC->APB1ENR &= ~(1 << 19))
#define UART5_PCLK_DI()				(RCC->APB1ENR &= ~(1 << 20))
#define USART6_PCLK_DI() 			(RCC->APB2ENR &= ~(1 << 5))

// DISABLE SYSCFG PERIPHERALS
#define SYSCFG_PCLK_DI()			(RCC->APB2ENR &= ~(1 << 14))

#endif /* INC_STM32F407XX_H_ */
