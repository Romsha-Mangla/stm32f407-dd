/*
 * ExtButtonExtLed.c
 *
 *  Created on: 28-Jan-2024
 *      Author: romsh
 */


#include "stm32f407xx_gpio_drv.h"

GPIO_Handle_t led[1];
GPIO_Handle_t button;
InterruptHandle button_int;

void delay(uint16_t count)
{
	for(__vo uint16_t i = 0;i<count;i++);

}

int main(void)
{
//	__vo uint8_t button_push  = RESET;
//	__vo uint8_t ButtonBounce = TRUE;


	//external LED
	/*pin14 of port A is also used as SWCLK after reset, therefore when we use SWD to program
	 * the MCU, it causes error as it interferes with SWD function. Check the user manual of MCU
	 */
	led[0].GPIOPinConfig.GPIO_PinNum = PIN8;
	led[0].GPIOPinConfig.GPIOmode	=	OUTPUT_MODE;
	led[0].GPIOPinConfig.GPIOOutputType = OP_TYPE_PP;
	led[0].GPIOPinConfig.GPIOspeed = HIGH_SPEED;
	led[0].GPIOPinConfig.GPIOPullUpPullDown = NO_PU_NO_PD;
	led[0].GPIOPinConfig.GPIOPort = PORTA;
	led[0].pGPIO = GPIOA;

	button.GPIOPinConfig.GPIO_PinNum = PIN12;
	button.GPIOPinConfig.GPIOmode    = GPIO_INT_FT;
	button.GPIOPinConfig.GPIOPullUpPullDown = PULL_UP;
	button.GPIOPinConfig.GPIOspeed   = HIGH_SPEED;
	button.GPIOPinConfig.GPIOOutputType = OP_TYPE_PP; //though not needed but code is written in a way that requires a place holder
	button.GPIOPinConfig.GPIOPort = PORTB;
	button.pGPIO = GPIOB;

	button_int.IRQNum = EXTI15_10_IRQ;
	button_int.IntPri = 0;
	button_int.IsEnable = TRUE;

	//unable the GPIOA clock
	GPIO_PeriClkCtrl(GPIOA,ENABLE);

	//unable the clock to GPIOB
	GPIO_PeriClkCtrl(GPIOB,ENABLE);

	//initialize GPIOB pin 12
	GPIO_Init(&button);

	//initialize the GPIOA pin 8
	GPIO_Init(&led[0]);

	//configure the interrupt on button
	GPIO_IRQConfig(&button_int);



	/* Loop forever */
	while(1)
	{
//		button_push = GPIO_readFrmInputPin(GPIOB, PIN12);
//		ButtonBounce = button_push;
//		//add a timer delay and not a for loop, a for loop is highly unreliable
//		delay(100U);
//		button_push = GPIO_readFrmInputPin(GPIOB, PIN12);
//		if(ButtonBounce==button_push)
//		{
//			//button is pressed
//			if(button_push==FALSE)
//			{
//				GPIO_ToggleOutputPin(GPIOA,PIN8);
//			}
//		}

//		if(GPIO_readFrmInputPin(GPIOA, PIN0) == SET)
//		{
//			delay(1000);
//			GPIO_ToggleOutputPin(GPIOD,PIN12);
//		}
	}
}

void EXTI15_10_IRQHandler(void)
{
	//call the GPIO interrupt handler
	GPIO_IRQHandling(button.GPIOPinConfig.GPIO_PinNum);

	//perform the task
	GPIO_ToggleOutputPin(GPIOA,PIN8);

}
