/*
 * stm32f303xx_gpio_driver.h
 *
 *  Created on: 12 Jul 2020
 *      Author: Enes Catlioglu
 */

#ifndef INC_STM32F303XX_GPIO_DRIVER_H_
#define INC_STM32F303XX_GPIO_DRIVER_H_

#include "stm32f303xx.h"



typedef struct{

	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinSpeed;
	uint8_t GPIO_PinMode;          /* For possible pin modes @GPIO_Possible_Pin_Modes -> */
	uint8_t GPIO_PinPuPdCont;
	uint8_t GPIO_PinOtyp;		   /* For possible output types @GPIO_Possible_Output_Types -> */
	uint8_t GPIO_PinAltFunMode;

}GPIO_PinConfig_t;

/***********************************************************************
 * 						GPIO Pin Numbers
 */

#define GPIO_PIN_NO_0		0
#define GPIO_PIN_NO_1		1
#define GPIO_PIN_NO_2		2
#define GPIO_PIN_NO_3		3
#define GPIO_PIN_NO_4		4
#define GPIO_PIN_NO_5		5
#define GPIO_PIN_NO_6		6
#define GPIO_PIN_NO_7		7
#define GPIO_PIN_NO_8		8
#define GPIO_PIN_NO_9		9
#define GPIO_PIN_NO_10		10
#define GPIO_PIN_NO_11		11
#define GPIO_PIN_NO_12		12
#define GPIO_PIN_NO_13		13
#define GPIO_PIN_NO_14		14
#define GPIO_PIN_NO_15		15

//---------------------------------------------------------------------


/*
 * @GPIO_Possible_Pin_Modes
 */
#define GPIO_MODE_IN  		0
#define GPIO_MODE_OU		1
#define GPIO_MODE_ALTFM    	2	// Alternate Function Mode
#define GPIO_MODE_ANALOG 	3
#define GPIO_MODE_IT_FT	   	4   // Falling         Edge Interrupt
#define GPIO_MODE_IT_RT	   	5   // Rising          Edge Interrupt
#define GPIO_MODE_IT_RT_FT	6   // Rising-Falling  Edge Interrupt


/*********************************************************************
 * 				@GPIO_Possible_Output_Types
 */
#define GPIO_OP_TYPE_PP   0
#define GPIO_OP_TYPE_OD   1

//--------------------------------------------------------------------

/**********************************************************************
 * 				GPIO Possible Output Speed
 */
#define GPIO_OP_SPEED_LS 0
#define GPIO_OP_SPEED_MS 1
#define GPIO_OP_SPEED_HS 3
//--------------------------------------------------------------------

/**********************************************************************
 * 				GPIO Pull-up Pull-down
 */
#define GPIO_NO_PUPD    0
#define GPIO_PU		    1
#define GPIO_PD		    2
//--------------------------------------------------------------------

/********************************************************
 * 				GPIO Handle Structure
 */

typedef struct{

	GPIO_RegDef_t *pGPIOx; // Hold GPIO ports base address which the pin belongs
	GPIO_PinConfig_t GPIO_PinConfig;

}GPIO_Handle_t;

//-------------------------------------------------------

/**************************************************************************************

							API's Supported in This Driver

***************************************************************************************/

/*
 * Peripheral Clock Setup
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);


/*
 * Init and Deinit GPIO
 */
void GPIO_Init   (GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit (GPIO_RegDef_t *pGPIOx);


/*
 * Data Read and Write
 */
uint8_t   GPIO_ReadFromInputPin  (GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t  GPIO_ReadFromInputPort (GPIO_RegDef_t *pGPIOx);
void 	  GPIO_WriteToOutpuPin   (GPIO_RegDef_t *pGPIOx, uint8_t PinNumber,uint8_t Value);
void	  GPIO_WriteToOutpuPort  (GPIO_RegDef_t *pGPIOx, uint16_t Value);
void	  GPIO_ToggleOutputPin   (GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/*
 * IRQ Configration and ISR Handling
 */

void GPIO_IRQConfig    (uint8_t IRQNumber, uint8_t IRQPriority,uint8_t EnorDi);
void GPIO_IRQHandling  (uint8_t PinNumber);



























#endif /* INC_STM32F303XX_GPIO_DRIVER_H_ */
