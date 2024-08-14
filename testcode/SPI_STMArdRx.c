/*
 * STMArdRx.c
 *
 *  Created on: 11-Mar-2024
 *      Author: romsh
 */

#include "stm32f407xx_spi_drv.h"
#include "stm32f407xx_gpio_drv.h"
#include<string.h>

__vo uint8_t SPIflag = RESET;

void delay(uint32_t count){
	__vo int i;
	//delay
	for(i=0;i<count;i++);
}

int main()
{
	//__vo uint8_t vInProcessing = FALSE;
	char *string ="this is very good!";
	size_t  stringSize = strlen((const char*)string);

	GPIO_Handle_t masterSPI2;
	GPIO_Handle_t UserButton;
	SPI_Handle_t SPI2Config;
	//InterruptHandle UserButtonInt;

	//configure the GPIO pins for SPI
	memset(&masterSPI2,0,sizeof(GPIO_Handle_t));

	masterSPI2.GPIOPinConfig.GPIOmode = ALT_FUNC;
	masterSPI2.GPIOPinConfig.GPIOAltFunc = AF5;
	masterSPI2.GPIOPinConfig.GPIOOutputType= OP_TYPE_PP;
	masterSPI2.GPIOPinConfig.GPIOspeed = HIGH_SPEED;
	masterSPI2.GPIOPinConfig.GPIOPullUpPullDown = NO_PU_NO_PD;
	masterSPI2.GPIOPinConfig.GPIOPort = PORTB;
	masterSPI2.pGPIO = GPIOB;

	//for MOSI
	masterSPI2.GPIOPinConfig.GPIO_PinNum = PIN15;
	GPIO_Init(&masterSPI2);
	//for SCLK
	masterSPI2.GPIOPinConfig.GPIO_PinNum = PIN10;
	GPIO_Init(&masterSPI2);
	//for NSS
	masterSPI2.GPIOPinConfig.GPIO_PinNum = PIN12;
	GPIO_Init(&masterSPI2);

	memset(&UserButton,0,sizeof(GPIO_Handle_t));

	//configure the user button
	UserButton.pGPIO = GPIOA;
	UserButton.GPIOPinConfig.GPIOPort    = PORTA;
	UserButton.GPIOPinConfig.GPIO_PinNum = PIN0;
	UserButton.GPIOPinConfig.GPIOspeed   = HIGH_SPEED;
	UserButton.GPIOPinConfig.GPIOmode    = INPUT_MODE;
	UserButton.GPIOPinConfig.GPIOPullUpPullDown = NO_PU_NO_PD;

	//configure the user button interrupt
//	UserButtonInt.IRQNum = EXTI0_IRQ;
//	UserButtonInt.IntPri = 1;
//	UserButtonInt.IsEnable = TRUE;


	//initializing the GPIO PA0 pin
	GPIO_Init(&UserButton);

	//configure the user button interrupt
	//GPIO_IRQConfig(&UserButtonInt);
/*
 * ************************************************************************************************************************************************
 */
	// initializing SPI2 port

	memset(&SPI2Config,0,sizeof(SPI_Handle_t));

	SPI2Config.pSPI 					 = SPI2;
	SPI2Config.sSPIConfig.SPI_deviceMode = SPI_MASTER;
	SPI2Config.sSPIConfig.SPI_busConfig  = FULL_DUPLEX;
	SPI2Config.sSPIConfig.SPI_SS         = SPI_SW_DIS;
	SPI2Config.sSPIConfig.SPI_CPHA		 = SPI_CLK_LEADING_EDGE;
	SPI2Config.sSPIConfig.SPI_DFF        = SPI_EIGHT_BIT;
	SPI2Config.sSPIConfig.SPI_speed		 = PRESCALER_32;

	SPI_Init(&SPI2Config);



	//SPI_DeInit(SPI2Config.pSPI);

	while(1)
	{
		//if(SPIflag==SET&&

		while(!GPIO_readFrmInputPin(GPIOA,PIN0));

		delay(100000);
//		if(vInProcessing == FALSE)
//		{
			//vInProcessing = TRUE;

			//enable SPI2
			SPI_Enable(SPI2);

			//send the no. of bytes first
			SPI_SendData(SPI2Config.pSPI,&stringSize,1);

			delay(50000);

			//send the string
			SPI_SendData(SPI2Config.pSPI,string,strlen((const char*)string));

			//disable SPI
			SPI_Disable(SPI2);

			//vInProcessing = FALSE;
			//SPIflag = RESET;

//		}
	}

	return 1;

}


void EXTI0_IRQHandler(void)
{

	GPIO_IRQHandling(PIN0);

	//set the flag to signal SPI2 to send the string to ardiuno slave
	//ignore the button press if the previous string is not sent
//	SPIflag = SET;


	return;

}
