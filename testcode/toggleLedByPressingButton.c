/*
 * toggleLedByPressingButton.c
 *
 *  Created on: 19-Jan-2024
 *      Author: romsh
 */


	/*
	 * P2: WAP to toggle on board LED whenever the on board button is pressed
	 */
#include "stm32f407xx_gpio_drv.h"
#include "stm32f407Header.h"

void delay(uint16_t count)
{
	for(__vo uint16_t i = 0;i<count;i++);

}

int main(void)
{
	__vo uint8_t button_push  = RESET;
	__vo uint8_t ButtonBounce = FALSE;
	GPIO_Handle_t led[4];
	GPIO_Handle_t button;

	//for green LED
	led[0].GPIOPinConfig.GPIO_PinNum = PIN12;
	led[0].GPIOPinConfig.GPIOmode	=	OUTPUT_MODE;
	led[0].GPIOPinConfig.GPIOOutputType = OP_TYPE_PP;
	led[0].GPIOPinConfig.GPIOspeed = LOW;
	led[0].GPIOPinConfig.GPIOPullUpPullDown = NO_PU_NO_PD;
	led[0].pGPIO = GPIOD;

	button.GPIOPinConfig.GPIO_PinNum = PIN0;
	button.GPIOPinConfig.GPIOmode    = INPUT_MODE;
	button.GPIOPinConfig.GPIOPullUpPullDown = NO_PU_NO_PD;
	button.GPIOPinConfig.GPIOspeed   = LOW;
	button.GPIOPinConfig.GPIOOutputType = OP_TYPE_PP; //though not needed but code is written in a way that requires a place holder
	button.pGPIO = GPIOA;

	//unable the GPIOD clock
	GPIO_PeriClkCtrl(GPIOD,ENABLE);

	//unable the clock to GPIOA
	GPIO_PeriClkCtrl(GPIOA,ENABLE);

	//initialize the GPIOD pin 12
	GPIO_Init(&led[0]);

	//initialize GPIOA pin 0
	GPIO_Init(&button);


	/* Loop forever */
	while(1)
	{
		button_push = GPIO_readFrmInputPin(GPIOA, PIN0);
		ButtonBounce = button_push;
		//add a timer delay and not a for loop, a for loop is highly unreliable
		delay(100U);
		button_push = GPIO_readFrmInputPin(GPIOA, PIN0);
		if(ButtonBounce==button_push)
		{
			//button is pressed
			if(button_push==TRUE)
			{
				GPIO_ToggleOutputPin(GPIOD,PIN12);
			}
		}

//		if(GPIO_readFrmInputPin(GPIOA, PIN0) == SET)
//		{
//			delay(1000);
//			GPIO_ToggleOutputPin(GPIOD,PIN12);
//		}
	}
}
