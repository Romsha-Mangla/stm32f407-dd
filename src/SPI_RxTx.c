/*
 * SPIRxTx.c
 *
 *  Created on: 13-Mar-2024
 *      Author: romsh
 */

#include"stm32f407xx_spi_drv.h"
#include"stm32f407xx_gpio_drv.h"
#include<string.h>

void MasterSPIConfig()
{
	GPIO_Handle_t masterSPI2;
	SPI_Handle_t SPI2Config;
	InterruptHandle SPI2InterruptHandle;

	//configure the GPIO pins for SPI master
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

	// initializing master ->SPI2 port
	memset(&SPI2Config,0,sizeof(SPI_Handle_t));

	SPI2Config.pSPI 					 = SPI2;
	SPI2Config.sSPIConfig.SPI_deviceMode = SPI_MASTER;
	SPI2Config.sSPIConfig.SPI_busConfig  = FULL_DUPLEX;
	SPI2Config.sSPIConfig.SPI_SS         = SPI_SW_DIS;
	SPI2Config.sSPIConfig.SPI_DFF        = SPI_EIGHT_BIT;
	SPI2Config.sSPIConfig.SPI_speed		 = PRESCALER_2;

	SPI_Init(&SPI2Config);

	// initializing SPI2 interrupt
	memset(&SPI2InterruptHandle,0,sizeof(InterruptHandle));

	SPI2InterruptHandle.IRQNum = SPI2_IRQ;
	SPI2InterruptHandle.IntPri = 2;
	SPI2InterruptHandle.IsEnable = TRUE;

	SPI_IRQConfig(&SPI2InterruptHandle);

}

void SPISlaveConfig()
{
	GPIO_Handle_t slaveSPI3;
	SPI_Handle_t SPI3Config;
	InterruptHandle SPI3InterruptHandle;

	//configuring pins for SPI slave -> SPI3
	memset(&slaveSPI3,0,sizeof(GPIO_Handle_t));

	slaveSPI3.GPIOPinConfig.GPIOmode 			= ALT_FUNC;
	slaveSPI3.GPIOPinConfig.GPIOspeed			= HIGH_SPEED;
	slaveSPI3.GPIOPinConfig.GPIOPort 			= PORTB;
	slaveSPI3.GPIOPinConfig.GPIOPullUpPullDown 	= NO_PU_NO_PD;
	slaveSPI3.GPIOPinConfig.GPIOAltFunc 		= AF6;

	//MOSI pin
	slaveSPI3.pGPIO      				= GPIOB;
	slaveSPI3.GPIOPinConfig.GPIO_PinNum = PIN5;
	GPIO_Init(&slaveSPI3);

	//SCK pin
	slaveSPI3.pGPIO      				= GPIOB;
	slaveSPI3.GPIOPinConfig.GPIO_PinNum = PIN3;
	GPIO_Init(&slaveSPI3);

	//NSS pin
	slaveSPI3.pGPIO					 	= GPIOA;
	slaveSPI3.GPIOPinConfig.GPIOmode 	= INPUT_MODE;
	slaveSPI3.GPIOPinConfig.GPIOPort 	= PORTA;
	slaveSPI3.GPIOPinConfig.GPIO_PinNum = PIN15;
	GPIO_Init(&slaveSPI3);

	//initializing slave ->SPI3 port
	memset(&SPI3Config,0,sizeof(SPI_Handle_t));

	SPI3Config.pSPI						= SPI3;
	SPI3Config.sSPIConfig.SPI_busConfig = SIMPLEX_RX_ONLY;
	SPI3Config.sSPIConfig.SPI_speed     = PRESCALER_2;

	SPI_Init(&SPI3Config);

	// initializing SPI2 interrupt
	memset(&SPI3InterruptHandle,0,sizeof(InterruptHandle));

	SPI3InterruptHandle.IRQNum = SPI3_IRQ;
	SPI3InterruptHandle.IntPri = 1;
	SPI3InterruptHandle.IsEnable = TRUE;

	SPI_IRQConfig(&SPI3InterruptHandle);

}


int main()
{
	uint8_t msgtxBuffer[] = {'H','e','l','l','o'};
	uint8_t msgrxBuffer[20];

	SPI_Message_t SPI2Message;
	SPI2Message.Buffer     = msgtxBuffer;
	SPI2Message.Len        = sizeof(msgtxBuffer)+1;//one more for the guard byte
	SPI2Message.MsgStatus  = TX_NOT_STARTED;
	SPI2Message.SpiChannel = SPI_2;

	SPI_Message_t SPI3Message;
	SPI3Message.Buffer     = msgrxBuffer;
	SPI3Message.Len        = sizeof(msgtxBuffer);
	SPI3Message.MsgStatus  = READ_NOT_STARTED;
	SPI3Message.SpiChannel = SPI_3;

	MasterSPIConfig();
	SPISlaveConfig();

	//enable the SPI channels
	SPI_Enable(SPI2);
	SPI_Enable(SPI3);

	//make the channel ready for receive
	SPI_ReceiveDataInt(&SPI3Message);

	//transmit some bytes
	SPI_SendDataInt(&SPI2Message);

	SPI_Disable(SPI2);
	SPI_Disable(SPI3);

	while(1);


	return 0;
}


