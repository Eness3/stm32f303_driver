/*
 * stm32f303xx.h
 *
 *  Created on: Jul 10, 2020
 *      Author: Enes Catlioglu
 */

#ifndef INC_STM32F303XX_H_
#define INC_STM32F303XX_H_


#include<stdint.h>



#define __vo volatile

/**********************************************************************
 *  				    Base Addresses of Flash, SRAM and ROM
 */

#define FLASH_BASEADDR		0x08000000U // Flash Base Address
#define SRAM_BASEADDR		0x20000000U // SRAM Base Address
#define ROM_BASEADDR		0x1FFFD800U // ROM Base Address

//---------------------------------------------------------------------

/**********************************************************************
 * 							Bus Base Addresses
 */

#define APB1PERIPH_BASEADDR 		0x40000000U
#define APB2PERIPH_BASEADDR 		0x40010000U

#define AHB1PERIPH_BASEADDR			0x40020000U
#define AHB2PERIPH_BASEADDR			0x48000000U
#define AHB3PERIPH_BASEADDR			0x50000000U

//---------------------------------------------------------------------

/**********************************************************************
 * 						APB1 Peripheral Base Address
 */

#define TIM2_BASEADDR				APB1PERIPH_BASEADDR
#define TIM3_BASEADDR				(APB1PERIPH_BASEADDR +0x0400U)
#define	TIM6_BASEADDR				(APB1PERIPH_BASEADDR +0x1000U)
#define TIM7_BASEADDR				(APB1PERIPH_BASEADDR +0x1400U)
#define RTC_BASEADDR				(APB1PERIPH_BASEADDR +0x2800U)
#define WWDG_BASEADDR				(APB1PERIPH_BASEADDR +0x2C00U)
#define IWDG_BASEADDR				(APB1PERIPH_BASEADDR +0x3000U)
#define USRT2_BASEADDR				(APB1PERIPH_BASEADDR +0x4400U)
#define USRT3_BASEADDR				(APB1PERIPH_BASEADDR +0x4800U)
#define I2C1_BASEADDR				(APB1PERIPH_BASEADDR +0x5400U)
#define DAC1_BASEADDR				(APB1PERIPH_BASEADDR +0x7400U)
#define DAC2_BASEADDR				(APB1PERIPH_BASEADDR +0x9800U)

//---------------------------------------------------------------------

/**********************************************************************
 * 						APB2 Peripheral Base Address
 */

#define TIM1_BASEADDR				(APB2PERIPH_BASEADDR +0x2C00U)
#define SPI1_BASEADRR	 			(APB2PERIPH_BASEADDR +0x3000U)
#define USART1_BASEADRR	 			(APB2PERIPH_BASEADDR +0x3800U)
#define EXTI_BASEADRR	 			(APB2PERIPH_BASEADDR +0x0400U)

//---------------------------------------------------------------------


/***********************************************************************
 * 						AHB1 Peripheral Base Address
 */

#define RCC_BASEADDR	 (AHB1PERIPH_BASEADDR +0x1000U)

//----------------------------------------------------------------------

/***********************************************************************
 * 						AHB2 Peripheral Base Address
 */

#define GPIOA_BASEADRR  (AHB2PERIPH_BASEADDR)
#define GPIOB_BASEADRR  (AHB2PERIPH_BASEADDR + 0X0400U)
#define GPIOC_BASEADRR	(AHB2PERIPH_BASEADDR + 0X0800U)
#define GPIOD_BASEADRR	(AHB2PERIPH_BASEADDR + 0X0C00U)
#define GPIOF_BASEADRR	(AHB2PERIPH_BASEADDR + 0X01400U)

//-----------------------------------------------------------------------

/************************************************************************
 * 						GPIO Register Structure
 */

typedef struct {
	__vo uint32_t MODER;       // Mode Register Offset : 0
	__vo uint32_t OTYPER;	   // Output Type Register Offset  : 0x04
	__vo uint32_t OSPEEDR;	   // Output Speed Register Offset : 0x08
	__vo uint32_t PUPDR;       // Pull-up pull-down register
	__vo uint32_t IDR;		   // Input data register
	__vo uint32_t ODR;		   // Output data register
	__vo uint32_t BSRR;        // Bit set reset register
	__vo uint32_t LCKR;        // Lock register
	__vo uint32_t AFR[2];      // Alternate Function low-high register AFR[0]:low  AFR[1]:High
	__vo uint32_t BRR;		   // Bit reset register
}GPIO_RegDef_t;

//-------------------------------------------------------------------------

/**************************************************************************
 * 						GPIO Base Addresses
 */

#define  GPIOA		((GPIO_RegDef_t*)GPIOA_BASEADRR)
#define  GPIOB		((GPIO_RegDef_t*)GPIOB_BASEADRR)
#define  GPIOC		((GPIO_RegDef_t*)GPIOC_BASEADRR)
#define  GPIOD		((GPIO_RegDef_t*)GPIOD_BASEADRR)

//-----------------------------------------------------------------------------

/******************************************************************************
 * 					RCC Register Structure
 */
typedef struct{
	__vo uint32_t CR;
	__vo uint32_t CFGR;
	__vo uint32_t CIR;
	__vo uint32_t APB2RSTR;
	__vo uint32_t APB1RSTR;
	__vo uint32_t AHBENR;
	__vo uint32_t APB2ENR;
	__vo uint32_t APB1ENR;
	__vo uint32_t BDCR;
	__vo uint32_t CSR;
	__vo uint32_t AHBRSTR;
	__vo uint32_t CFGR2;
	__vo uint32_t CFGR3;
}Rcc_RegDef_t;

//----------------------------------------------------------------------------

/*****************************************************************************
 * 							RCC Base Address
 */
#define RCC  ((Rcc_RegDef_t*)RCC_BASEADDR)

//----------------------------------------------------------------------------


/*****************************************************************************
 * 						GPIOx Clock Enable Macros
 */

#define GPIOA_CLK_EN()   (RCC->AHBENR |= (1<<17))  // GPIOA ENABLE
#define GPIOB_CLK_EN()   (RCC->AHBENR |= (1<<18))  // GPIOB ENABLE
#define GPIOC_CLK_EN()   (RCC->AHBENR |= (1<<19))  // GPIOC ENABLE
#define GPIOD_CLK_EN()   (RCC->AHBENR |= (1<<20))  // GPIOD ENABLE

//---------------------------------------------------------------------------

/****************************************************************************
 *						I2C Clock Enable Macros
 */
#define I2C1_CLK_EN()   (RCC->APB1ENR |=(1<<21))
#define I2C2_CLK_EN()   (RCC->APB1ENR |=(1<<22))

//---------------------------------------------------------------------------

/****************************************************************************
 *						SPI Clock Enable Macro
 */
#define SPI1_CLK_EN()   (RCC->APB2ENR |=(1<<12))

//---------------------------------------------------------------------------

/****************************************************************************
 *						USART Clock Enable Macros
 */
#define USRT1_CLK_EN()   (RCC->APB2ENR |=(1<<14))
#define USRT2_CLK_EN()   (RCC->APB1ENR |=(1<<17))
#define USRT3_CLK_EN()   (RCC->APB1ENR |=(1<<18))

//---------------------------------------------------------------------------

/****************************************************************************
 *					GPIO Clock Disable Macros
 */
#define GPIOA_CLK_DI()     (RCC->AHBENR &= ~(1<<17))
#define	GPIOB_CLK_DI()	   (RCC->AHBENR &= ~(1<<18))
#define	GPIOC_CLK_DI()	   (RCC->AHBENR &= ~(1<<19))
#define	GPIOD_CLK_DI()     (RCC->AHBENR &= ~(1<<20))
#define	GPIOF_CLK_DI()     (RCC->AHBENR &= ~(1<<22))

/*****************************************************************************
 *					I2C Clock Disable Macros
 */
#define I2C1_CLK_DI()   (RCC->APB1ENR &= ~(1<<21))
#define I2C2_CLK_DI()   (RCC->APB1ENR &= ~(1<<22))

//----------------------------------------------------------------------------

/*****************************************************************************
 *					SPI Clock Disable Macro
 */
#define SPI1_CLK_DI()  (RCC->APB2ENR &= ~(1<<12))

//----------------------------------------------------------------------------

/*****************************************************************************
 *				   USART Clock Disable Macros
 */
#define USRT1_CLK_DI()    (RCC->APB2ENR &= ~(1<<14))
#define USRT2_CLK_DI()    (RCC->APB1ENR &= ~(1<<17))
#define USRT3_CLK_DI()    (RCC->APB1ENR &= ~(1<<18))

//----------------------------------------------------------------------------


















#endif /* INC_STM32F303XX_H_ */
