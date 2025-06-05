/*
 * stm32f407xx.h
 *
 *  Created on: Apr 23, 2025
 *      Author: hugo-juarez
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include <stdint.h>
#include <stddef.h>

#define __vo volatile
#define __weak __attribute__((weak))

//Generic macros
#define ENABLE				1
#define DISABLE				0
#define SET					ENABLE
#define RESET				DISABLE


/*********************************** ARM CORTEX M4 REGISTERS ***********************************/

/***********************************************************/
/************************* NVIC ****************************/
/***********************************************************/

#define NVIC_BASEADDR				0xE000E100UL
#define NVIC						((NVIC_RefDef_t*)NVIC_BASEADDR)
#define NO_PR_BITS_IMPLEMENTED		4	//Number of priority pins implement

typedef struct {
	__vo uint32_t ISER[8];
	uint32_t RESERVED0[24];
	__vo uint32_t ICER[8];
	uint32_t RESERVED1[24];
	__vo uint32_t ISPR[8];
	uint32_t RESERVED2[24];
	__vo uint32_t ICPR[8];
	uint32_t RESERVED3[24];
	__vo uint32_t IABR[8];
	uint32_t RESERVED4[56];
	__vo uint8_t IPR[240];
}NVIC_RefDef_t;

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
#define AHB1_BASEADDR				0x40020000UL
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
/************************** SPI ****************************/
/***********************************************************/

#define SPI1						((SPIx_RegDef_t*)SPI1_BASEADDR)
#define SPI2						((SPIx_RegDef_t*)SPI2_BASEADDR)
#define SPI3						((SPIx_RegDef_t*)SPI3_BASEADDR)

typedef struct {
	__vo uint32_t CR[2];				//SPI control register
	__vo uint32_t SR;					//SPI status register
	__vo uint32_t DR;					//SPI data register
	__vo uint32_t CRCPR;				//SPI CRC polynomial register
	__vo uint32_t RXCRCR;				//RX CRC register
	__vo uint32_t TXCRCR;				//TX CRC register
	__vo uint32_t I2SCFGR;				//SPI_I2S configuration register
	__vo uint32_t I2SPR;				//SPI_I2S prescaler register
} SPIx_RegDef_t;

/***********************************************************/
/************************** I2C ****************************/
/***********************************************************/

#define I2C1						((I2Cx_RegDef_t*)I2C1_BASEADDR)
#define I2C2						((I2Cx_RegDef_t*)I2C2_BASEADDR)
#define I2C3						((I2Cx_RegDef_t*)I2C3_BASEADDR)


typedef struct{
	__vo uint32_t CR1;					//I2C Control register
	__vo uint32_t CR2;					//I2C Control register
	__vo uint32_t OAR1;					//I2C Own address register
	__vo uint32_t OAR2;					//I2C Own address register
	__vo uint32_t DR;					//I2C Data register
	__vo uint32_t SR1;					//I2C Status register
	__vo uint32_t SR2;					//I2C Status register
	__vo uint32_t CCR;					//I2C Clock control register
	__vo uint32_t TRISE;				//I2C TRISE register
	__vo uint32_t FLTR;					//I2C FLTR register
}I2Cx_RegDef_t;

//Bit Defintion

#define I2C_CR1_PE					0
#define I2C_CR1_SMBUS				1
#define I2C_CR1_SMBTYPE				3
#define I2C_CR1_ENARP				4
#define I2C_CR1_ENPEC				5
#define I2C_CR1_ENGC				6
#define I2C_CR1_NOSTRETCH			7
#define I2C_CR1_START				8
#define I2C_CR1_STOP				9
#define I2C_CR1_ACK					10
#define I2C_CR1_POS					11
#define I2C_CR1_PEC					12
#define I2C_CR1_ALERT				13
#define I2C_CR1_SWRST				15

#define I2C_CR2_FREQ				0
#define I2C_CR2_ITERREN				8
#define I2C_CR2_ITEVTEN				9
#define I2C_CR2_ITBUFEN				10
#define I2C_CR2_DMAEN				11
#define I2C_CR2_LAST				12

#define I2C_SR1_SB					0
#define I2C_SR1_ADDR				1
#define I2C_SR1_BTF					2
#define I2C_SR1_ADD10				3
#define I2C_SR1_STOPF				4
#define I2C_SR1_RXNE				6
#define I2C_SR1_TXE					7
#define I2C_SR1_BERR				8
#define I2C_SR1_ARLO				9
#define I2C_SR1_AF					10
#define I2C_SR1_OVR					11
#define I2C_SR1_PECERR				12
#define I2C_SR1_TIMEOUT				14
#define I2C_SR1_SMBALERT			15

#define I2C_SR2_MSL					0
#define I2C_SR2_BUSY				1
#define I2C_SR2_TRA					2
#define I2C_SR2_GENCALL				4
#define I2C_SR2_SMBDEFAULT			5
#define I2C_SR2_SMBHOST				6
#define I2C_SR2_DUALF				7
#define I2C_SR2_PEC					8

#define I2C_CCR_CCR					0
#define I2C_CCR_DUTY				14
#define I2C_CCR_FS					15

/***********************************************************/
/************************* USART ***************************/
/***********************************************************/

// --- Peripherals Base Addresses ---
#define USART1						((USARTx_RegDef_t*) USART1_BASEADDR)
#define USART2						((USARTx_RegDef_t*) USART2_BASEADDR)
#define USART3						((USARTx_RegDef_t*) USART3_BASEADDR)
#define UART4						((USARTx_RegDef_t*) UART4_BASEADDR)
#define UART5						((USARTx_RegDef_t*) UART5_BASEADDR)
#define USART6						((USARTx_RegDef_t*) USART6_BASEADDR)


// --- Register Definition ---
typedef struct{
	__vo uint32_t SR;					//USART Status register
	__vo uint32_t DR;					//USART Data register
	__vo uint32_t BRR;					//USART Baud rate register
	__vo uint32_t CR1;					//USART Control register 1
	__vo uint32_t CR2;					//USART Control register 2
	__vo uint32_t CR3;					//USART Control register 3
	__vo uint32_t GTPR;					//USART Guard time and pre-scaler register
} USARTx_RegDef_t;


// --- Bit Definition ---
#define USART_SR_PE					0
#define USART_SR_FE					1
#define USART_SR_NF					2
#define USART_SR_ORE				3
#define USART_SR_IDLE				4
#define USART_SR_RXNE				5
#define USART_SR_TC					6
#define USART_SR_TXE				7
#define USART_SR_LBD				8
#define USART_SR_CTS				9

#define USART_BRR_DIV_FRACTION		0
#define USART_BRR_DIV_MANTISSA		4

#define USART_CR1_SBK				0
#define USART_CR1_RWU				1
#define USART_CR1_RE				2
#define USART_CR1_TE				3
#define USART_CR1_IDLEIE			4
#define USART_CR1_RXNEIE			5
#define USART_CR1_TCIE				6
#define USART_CR1_TXEIE				7
#define USART_CR1_PEIE				8
#define USART_CR1_PS				9
#define USART_CR1_PCE				10
#define USART_CR1_WAKE				11
#define USART_CR1_M					12
#define USART_CR1_UE				13
#define USART_CR1_OVER8				15

#define USART_CR2_ADD				0
#define USART_CR2_LBDL				5
#define USART_CR2_LBDIE				6
#define USART_CR2_LBCL				8
#define USART_CR2_CPHA				9
#define USART_CR2_CPOL				10
#define USART_CR2_CLKEN				11
#define USART_CR2_STOP				12
#define USART_CR2_LINEN				14

#define USART_CR3_EIE				0
#define USART_CR3_IREN				1
#define USART_CR3_IRLP				2
#define USART_CR3_HDSEL				3
#define USART_CR3_NACK				4
#define USART_CR3_SCEN				5
#define USART_CR3_DMAR				6
#define USART_CR3_DMAT				7
#define USART_CR3_RTSE				8
#define USART_CR3_CTSE				9
#define USART_CR3_CTSIE				10
#define USART_CR3_ONEBIT			11

#define USART_GTPR_PSC				0
#define USART_GTPR_GT				8

/***********************************************************/
/************************* EXTI ****************************/
/***********************************************************/

#define EXTI						((EXTIx_RegDef_t*)EXTI_BASEADDR)

typedef struct {
	__vo uint32_t IMR;					//Interrupt Mask Register
	__vo uint32_t EMR;					//Event Mask Register
	__vo uint32_t RTSR;					//Rising trigger selection register
	__vo uint32_t FTSR;					//Falling trigger selection register
	__vo uint32_t SWIER;				//Software interrupt event register
	__vo uint32_t PR;					//Pending Register

} EXTIx_RegDef_t;

/***********************************************************/
/************************ SYSCFG ***************************/
/***********************************************************/

#define SYSCFG						((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

typedef struct {
	__vo uint32_t MEMRMP;				//Memory remap register
	__vo uint32_t PMC;					//Peripheral mode configuration register
	__vo uint32_t EXTICR[4];			//External interrupt configuration registers EXT0:3 per register
	uint32_t RESERVED[2];
	__vo uint32_t CMPCR;				//Compensation cell control register
} SYSCFG_RegDef_t;

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

/***********************************************************/
/********************* REGISTER RESET **********************/
/***********************************************************/

#define GPIOA_REG_RESET()			do{ (RCC->AHB1RSTR |= 1 << 0);	(RCC->AHB1RSTR &= ~(1 << 0)); } while(0)
#define GPIOB_REG_RESET()			do{ (RCC->AHB1RSTR |= 1 << 1);	(RCC->AHB1RSTR &= ~(1 << 1)); } while(0)
#define GPIOC_REG_RESET()			do{ (RCC->AHB1RSTR |= 1 << 2);	(RCC->AHB1RSTR &= ~(1 << 2)); } while(0)
#define GPIOD_REG_RESET()			do{ (RCC->AHB1RSTR |= 1 << 3);	(RCC->AHB1RSTR &= ~(1 << 3)); } while(0)
#define GPIOE_REG_RESET()			do{ (RCC->AHB1RSTR |= 1 << 4);	(RCC->AHB1RSTR &= ~(1 << 4)); } while(0)
#define GPIOF_REG_RESET()			do{ (RCC->AHB1RSTR |= 1 << 5);	(RCC->AHB1RSTR &= ~(1 << 5)); } while(0)
#define GPIOG_REG_RESET()			do{ (RCC->AHB1RSTR |= 1 << 6);	(RCC->AHB1RSTR &= ~(1 << 6)); } while(0)
#define GPIOH_REG_RESET()			do{ (RCC->AHB1RSTR |= 1 << 7);	(RCC->AHB1RSTR &= ~(1 << 7)); } while(0)
#define GPIOI_REG_RESET()			do{ (RCC->AHB1RSTR |= 1 << 8);	(RCC->AHB1RSTR &= ~(1 << 8)); } while(0)
#define GPIOJ_REG_RESET()			do{ (RCC->AHB1RSTR |= 1 << 9);	(RCC->AHB1RSTR &= ~(1 << 9)); } while(0)
#define GPIOK_REG_RESET()			do{ (RCC->AHB1RSTR |= 1 << 10);	(RCC->AHB1RSTR &= ~(1 << 10)); } while(0)

#define SPI1_REG_RESET()			do{ (RCC->APB2RSTR |= 1 << 12);	(RCC->APB2RSTR &= ~(1 << 12)); } while(0)
#define SPI2_REG_RESET()			do{ (RCC->APB1RSTR |= 1 << 14);	(RCC->APB1RSTR &= ~(1 << 14)); } while(0)
#define SPI3_REG_RESET()			do{ (RCC->APB1RSTR |= 1 << 15);	(RCC->APB1RSTR &= ~(1 << 15)); } while(0)

#define I2C1_REG_RESET()			do{ (RCC->APB1RSTR |= 1 << 21);	(RCC->APB1RSTR &= ~(1 << 21)); } while(0)
#define I2C2_REG_RESET()			do{ (RCC->APB1RSTR |= 1 << 22);	(RCC->APB1RSTR &= ~(1 << 22)); } while(0)
#define I2C3_REG_RESET()			do{ (RCC->APB1RSTR |= 1 << 23);	(RCC->APB1RSTR &= ~(1 << 23)); } while(0)

#define USART1_REG_RESET()			do{ (RCC->APB2RSTR |= 1 << 4);	(RCC->APB2RSTR &= ~(1 << 4)); } while(0)
#define USART2_REG_RESET()			do{ (RCC->APB1RSTR |= 1 << 17);	(RCC->APB1RSTR &= ~(1 << 17)); } while(0)
#define USART3_REG_RESET()			do{ (RCC->APB1RSTR |= 1 << 18);	(RCC->APB1RSTR &= ~(1 << 18)); } while(0)
#define UART4_REG_RESET()			do{ (RCC->APB1RSTR |= 1 << 19);	(RCC->APB1RSTR &= ~(1 << 19)); } while(0)
#define UART5_REG_RESET()			do{ (RCC->APB1RSTR |= 1 << 20);	(RCC->APB1RSTR &= ~(1 << 20)); } while(0)
#define USART6_REG_RESET()			do{ (RCC->APB2RSTR |= 1 << 5);	(RCC->APB2RSTR &= ~(1 << 5)); } while(0)

/***********************************************************/
/********************* PORT CODE FUNC **********************/
/***********************************************************/

#define GPIO_BASEADDR_TO_PORT(x)	(	(x == GPIOA) ? 0 :\
										(x == GPIOB) ? 1 :\
										(x == GPIOC) ? 2 :\
										(x == GPIOD) ? 3 :\
										(x == GPIOE) ? 4 :\
										(x == GPIOF) ? 5 :\
										(x == GPIOG) ? 6 :\
										(x == GPIOH) ? 7 :\
										(x == GPIOI) ? 8 : 0	)

/***********************************************************/
/************************* IRQ NO **************************/
/***********************************************************/

#define IRQ_NO_EXTI0				6
#define IRQ_NO_EXTI1				7
#define IRQ_NO_EXTI2				8
#define IRQ_NO_EXTI3				9
#define IRQ_NO_EXTI4				10
#define IRQ_NO_EXTI9_5				23
#define IRQ_NO_EXTI15_10			40

#define IRQ_NO_SPI1					35
#define IRQ_NO_SPI2					36
#define IRQ_NO_SPI3					51

#define IRQ_NO_I2C1_EV				31
#define IRQ_NO_I2C1_ER				32

#define IRQ_NO_USART2				38

/***********************************************************/
/************************* DRIVER **************************/
/***********************************************************/
#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx_i2c_driver.h"
#include "stm32f407xx_spi_driver.h"
#include "stm32f407xx_usart_driver.h"

#endif /* INC_STM32F407XX_H_ */
