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
void GPIO_Init (GPIO_Handle_t *pGPIOHandle)
{
	//1. Configuration the mode of pin
		uint32_t temp =0;


		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode < 4){

		temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		pGPIOHandle->pGPIOx->MODER &= ~(3<< (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER |= temp;
		}

		else{
				// Interrupt
		};
		temp = 0;

	//2. Speed Configuration

		temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		pGPIOHandle->pGPIOx->OSPEEDR &= ~(3<<(2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->OSPEEDR |= temp;

		temp = 0;

	//3. Pupd Configuration
		temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdCont << (2* pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		pGPIOHandle->pGPIOx->PUPDR &= ~(3<< (2* pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->PUPDR |= temp;
		temp = 0;

	//4. Output Type Configuration
		temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinOtyp << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		pGPIOHandle->pGPIOx->OTYPER &= ~(1<< (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->OTYPER |= temp;
		temp = 0;

	//5. Alternate Function Configuration
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFM)
		{
			uint8_t temp1, temp2;
			temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber/8;
			temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber%8;
			pGPIOHandle->pGPIOx->AFR[temp1] &= (15<< pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << 4*temp2);
			pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << 4*temp2);
		}



}

//--------------------------------------------------------------------------------------------

/*
 *********************************************************************************************
 */
void GPIO_DeInit (GPIO_RegDef_t *pGPIOx)
{
	if(pGPIOx == GPIOA)
			{
				GPIOA_REG_RST();
			}
			else if(pGPIOx == GPIOB)
			{
				GPIOB_REG_RST();
			}
			else if(pGPIOx == GPIOC)
			{
				GPIOC_REG_RST();
			}
			else if(pGPIOx == GPIOD)
			{
				GPIOD_REG_RST();
			}

}

//---------------------------------------------------------------------------------------------


/**********************************************************************************************
 * 								Data Read
 */
uint8_t   GPIO_ReadFromInputPin  (GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;
	value = (uint8_t)((pGPIOx->IDR >>PinNumber) & 1);
	return value;
}


uint16_t  GPIO_ReadFromInputPort (GPIO_RegDef_t *pGPIOx)
{
		uint16_t value;
		value = (uint16_t)pGPIOx->IDR;
		return value;
}

//----------------------------------------------------------------------------------------------


/**********************************************************************************************
 * 								Data Write
 */

void GPIO_WriteToOutpuPin   (GPIO_RegDef_t *pGPIOx, uint8_t PinNumber,uint8_t Value)
{
	if(Value == GPIO_PIN_SET)
	{
		pGPIOx->ODR |= (1<<PinNumber);
	}
	else
	{
		pGPIOx->ODR &= ~(1<<PinNumber);
	}
}


void GPIO_WriteToOutpuPort  (GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
		pGPIOx->ODR  = Value;
}

//----------------------------------------------------------------------------------------------

void GPIO_ToggleOutputPin   (GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	    pGPIOx->ODR ^= (1 << PinNumber);
}































