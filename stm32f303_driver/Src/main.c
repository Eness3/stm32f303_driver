



#include"stm32f303xx_gpio_driver.h"

int main(void)
{
	GPIOB_CLK_EN();
	GPIO_Handle_t LedHandle;

	LedHandle.pGPIOx = GPIOB;
	LedHandle.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_3;
	LedHandle.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OU;

	GPIO_Init(&LedHandle);

	GPIO_WriteToOutpuPin(GPIOB,GPIO_PIN_NO_3,1);


	for(;;);
	return 0;
}
