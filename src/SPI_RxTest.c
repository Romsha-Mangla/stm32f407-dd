/*
 * RxTest.c
 *
 *  Created on: 22-Mar-2024
 *      Author: romsh
 */



#include "stm32f407xx_spi_drv.h"
#include "stm32f407xx_gpio_drv.h"
#include<string.h>




__vo uint8_t SPIflag = RESET;

int main()
{
	//__vo uint8_t vInProcessing = FALSE;
	char *string ="Ola";
	char Rxbuffer[10];

	GPIO_Handle_t masterSPI2;
	GPIO_Handle_t slaveSPI3;
	GPIO_Handle_t UserButton;
	SPI_Handle_t SPI2Config;
	SPI_Handle_t SPI3Config;
	//InterruptHandle UserButtonInt;

	//configure the GPIO pins for SPI master
	memset(&masterSPI2,0,sizeof(GPIO_Handle_t));

	masterSPI2.GPIOPinConfig.GPIOmode = ALT_FUNC;
	masterSPI2.GPIOPinConfig.GPIOAltFunc = AF5;
	masterSPI2.GPIOPinConfig.GPIOOutputType= OP_TYPE_PP;
	masterSPI2.GPIOPinConfig.GPIOspeed = LOW_SPEED;
	masterSPI2.GPIOPinConfig.GPIOPullUpPullDown = NO_PU_NO_PD;
	masterSPI2.GPIOPinConfig.GPIOPort = PORTB;
	masterSPI2.pGPIO = GPIOB;

	//enabling and initializing GPIO port for master SPI
	//GPIO_PeriClkCtrl(GPIOB,TRUE);

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

	GPIO_Init(&UserButton);

	//configuring pins for SPI slave -> SPI3
	memset(&slaveSPI3,0,sizeof(GPIO_Handle_t));


	slaveSPI3.GPIOPinConfig.GPIOmode 			= ALT_FUNC;
	slaveSPI3.GPIOPinConfig.GPIOPort 			= PORTB;
	slaveSPI3.GPIOPinConfig.GPIOPullUpPullDown 	= NO_PU_NO_PD;
	slaveSPI3.GPIOPinConfig.GPIOAltFunc 		= AF6;

	//MOSI pin
	slaveSPI3.pGPIO      				= GPIOB;
	slaveSPI3.GPIOPinConfig.GPIO_PinNum = PIN5;
	GPIO_Init(&slaveSPI3);

	//SCK pin
	slaveSPI3.GPIOPinConfig.GPIO_PinNum = PIN3;
	GPIO_Init(&slaveSPI3);

	//NSS pin
	slaveSPI3.GPIOPinConfig.GPIOmode 	= INPUT_MODE;
	slaveSPI3.GPIOPinConfig.GPIOPort 	= PORTA;
	slaveSPI3.pGPIO					 	= GPIOA;
	slaveSPI3.GPIOPinConfig.GPIO_PinNum = PIN15;
	GPIO_Init(&slaveSPI3);



	//configure the user button interrupt
	//GPIO_IRQConfig(&UserButtonInt);


/*
 * ************************************************************************************************************************************************
 */
	// initializing master ->SPI2 port

	memset(&SPI2Config,0,sizeof(SPI_Handle_t));

	SPI2Config.pSPI 					 = SPI2;
	SPI2Config.sSPIConfig.SPI_deviceMode = SPI_MASTER;
	SPI2Config.sSPIConfig.SPI_busConfig  = FULL_DUPLEX;
	SPI2Config.sSPIConfig.SPI_SS         = SPI_SW_DIS;
	SPI2Config.sSPIConfig.SPI_DFF        = SPI_EIGHT_BIT;
	SPI2Config.sSPIConfig.SPI_speed		 = PRESCALER_2;

	SPI_Init(&SPI2Config);

	//initializing slave ->SPI3 port
	memset(&SPI3Config,0,sizeof(SPI_Handle_t));

	SPI3Config.pSPI						= SPI3;
	SPI3Config.sSPIConfig.SPI_busConfig = FULL_DUPLEX;
	SPI3Config.sSPIConfig.SPI_speed     = PRESCALER_2;

	SPI_Init(&SPI3Config);



	//SPI_DeInit(SPI2Config.pSPI);

	while(1)
	{
		//if(SPIflag==SET&&

		while(!GPIO_readFrmInputPin(GPIOA,PIN0))
		{

		}


//		if(vInProcessing == FALSE)
//		{
			//vInProcessing = TRUE;

			//enable SPI2
			SPI_Enable(SPI2);
			SPI_Enable(SPI3);

			//send the no. of bytes first
			//SPI_SendData(SPI2Config.pSPI,&stringSize,1);



			//send the string
			SPI_SendData(SPI2Config.pSPI,string,strlen((const char*)string));

			//disable SPI
			SPI_Disable(SPI2);


			//receive the bytes
			SPI_ReceiveData(SPI3Config.pSPI, Rxbuffer, strlen((const char*)string));

			//disable channel 3 after receiving
			SPI_Disable(SPI3);

			//vInProcessing = FALSE;
			//SPIflag = RESET;

//		}
	}

}


void EXTI0_IRQHandler(void)
{

	GPIO_IRQHandling(PIN0);

	//set the flag to signal SPI2 to send the string to ardiuno slave
	//ignore the button press if the previous string is not sent
//	SPIflag = SET;


	return;

}
