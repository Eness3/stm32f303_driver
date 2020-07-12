/*
 * stm32f303xx_gpio_driver.c
 *
 *  Created on: 12 Jul 2020
 *      Author: Enes Catlioglu
 */

#include "stm32f303xx_gpio_driver.h"

//*******************************************************************************************
	 	 	 	 	 /*Peripheral Clock Setup*/
 /*
 * @Fn               : GPIO_PeriClockControl
 *
 * @Brief			 : This function enable or disable clock for given GPIO port
 *
 * @Param[in]	     : Base address of the GPIO port
 * @Param[in]		 : ENABLE or DISABLE Macros
 * @Param[in]		 :
 *
 * @Return		     : none
 *
 * @Note			 : none
 *
 */

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_CLK_EN();
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_CLK_EN();
		}
		else if(pGPIOx == GPIOC)
		{
			GPIOC_CLK_EN();
		}
		else if(pGPIOx == GPIOD)
		{
			GPIOD_CLK_EN();
		}

	}

	else if(EnorDi == DISABLE)
		{
			if(pGPIOx == GPIOA)
			{
				GPIOA_CLK_DI();
			}
			else if(pGPIOx == GPIOB)
			{
				GPIOB_CLK_DI();
			}
			else if(pGPIOx == GPIOC)
			{
				GPIOC_CLK_DI();
			}
			else if(pGPIOx == GPIOD)
			{
				GPIOD_CLK_DI();
			}

		}
}

//------------------------------------------------------------------------------------------


//******************************************************************************************

     	 /* Inıt GPIO */
/*
* @Fn               : GPIO_Init
*
* @Brief			 : This function initialize GPIO
*
* @Param[in]	     : Base address of the GPIO port
* @Param[in]		 :
* @Param[in]		 :
*
* @Return		     : none
*
* @Note			     : none
*
*/
void GPIO_Init   (GPIO_Handle_t *pGPIOHandle)
{


}

//--------------------------------------------------------------------------------------------












































}
