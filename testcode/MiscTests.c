/*
 * MiscTests.c
 *
 *  Created on: 22-Mar-2024
 *      Author: romsh
 */


#include "stm32f407xx_gpio_drv.h"

void delay(int count)
{
	for(__vo int i = 0;i<count;i++);

}

int main(void)
{

	GPIO_Handle_t led;

	led.GPIOPinConfig.GPIO_PinNum = PIN8;
	led.GPIOPinConfig.GPIOmode	=	OUTPUT_MODE;
	led.GPIOPinConfig.GPIOOutputType = OP_TYPE_PP;
	led.GPIOPinConfig.GPIOspeed = LOW_SPEED;
	led.GPIOPinConfig.GPIOPullUpPullDown = NO_PU_NO_PD;
	led.GPIOPinConfig.GPIOPort = PORTC;
	led.pGPIO = GPIOC;

	GPIO_Init(&led);


	while(1)
	{
		GPIO_ToggleOutputPin(GPIOC, PIN8);
		delay(5000000);
		//output voltage of a GPIO is 3.17V approx.

	}
}
