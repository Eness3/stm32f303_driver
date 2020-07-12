/*
 * stm32f303xx.h
 *
 *  Created on: Jul 10, 2020
 *      Author: Enes Catlioglu
 */

#ifndef INC_STM32F303XX_H_
#define INC_STM32F303XX_H_


/*
 *  Base Addresses of Flash, SRAM and ROM
 */

#define FLASH_BASEADDR		0x08000000U // Flash Base Address
#define SRAM_BASEADDR		0x20000000U // SRAM Base Address
#define ROM_BASEADDR		0x1FFFD800U // ROM Base Address

//********************************************************************

/*
 * Bus Base Addresses
 */

#define APB1PERIPH_BASEADDR 		0x40000000U
#define APB2PERIPH_BASEADDR 		0x40010000U

#define AHB1PERIPH_BASEADDR			0x40020000U
#define AHB2PERIPH_BASEADDR			0x48000000U
#define AHB3PERIPH_BASEADDR			0x50000000U

//********************************************************************

/*
 * APB1 Peripheral Base Address
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

//********************************************************************

/*
 * APB2 Peripheral Base Address
 */

#define TIM1_BASEADDR				(APB2PERIPH_BASEADDR +0x2C00U)
#define SPI1_BASEADRR	 			(APB2PERIPH_BASEADDR +0x3000U)
#define USART1_BASEADRR	 			(APB2PERIPH_BASEADDR +0x3800U)
#define EXTI_BASEADRR	 			(APB2PERIPH_BASEADDR +0x0400U)

//********************************************************************


/*
 * AHB1 Peripheral Base Address
 */

#define RCC_BASEADRR	 (AHB1PERIPH_BASEADDR +0x1000U)

//**********************************************************************

/*
 * AHB2 Peripheral Base Address
 */

#define GPIOA_BASEADRR  (AHB2PERIPH_BASEADDR)
#define GPIOB_BASEADRR  (AHB2PERIPH_BASEADDR + 0X0400U)
#define GPIOC_BASEADRR	(AHB2PERIPH_BASEADDR + 0X0800U)
#define GPIOD_BASEADRR	(AHB2PERIPH_BASEADDR + 0X0C00U)
#define GPIOF_BASEADRR	(AHB2PERIPH_BASEADDR + 0X01400U)

//***********************************************************************















#endif /* INC_STM32F303XX_H_ */
