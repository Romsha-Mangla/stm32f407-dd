/*
 * SPITest.c
 *
 *  Created on: 20-Feb-2024
 *      Author: romsh
 */


#include "stm32f407xx_spi_drv.h"
#include "stm32f407xx_gpio_drv.h"
#include<string.h>





int main()
{
	/*
	 * before we use SPI, we must configure GPIO as SPI pins ( MOSI, MISO, SCLK and NSS)
	 * We should look into alternate function mapping of GPIO in data sheet to find the pins.
	 * In this case, as per data sheet AF5 is SPI2;
	 * SPI2 NSS -PB9
	 * SPI2 SCK -PB10
	 * SPI2 MISO-PB14
	 * SPI2 MOSI-PB15
	 *
	 * AF6 is SPI3
	 * SPI3 NSS -PA15
	 * SPI SCK  -PC10
	 * SPI3 MISO-PC11
	 * SPI3 MOSI-PC12
	 *
	 */
	char *string ="hello world!";
//	uint8_t array[]={1,2,3,4};
	GPIO_Handle_t masterSPI2;
	SPI_Handle_t SPI2Config;

	memset(&masterSPI2,0,sizeof(masterSPI2));

	masterSPI2.GPIOPinConfig.GPIOmode = ALT_FUNC;
	masterSPI2.GPIOPinConfig.GPIOAltFunc = AF5;
	masterSPI2.GPIOPinConfig.GPIOOutputType= OP_TYPE_PP;
	masterSPI2.GPIOPinConfig.GPIOspeed = HIGH_SPEED;
	masterSPI2.GPIOPinConfig.GPIOPullUpPullDown = NO_PU_NO_PD;
	masterSPI2.GPIOPinConfig.GPIOPort = PORTB;
	masterSPI2.pGPIO = GPIOB;

	//enabling and initializing GPIO port for master SPI
	GPIO_PeriClkCtrl(GPIOB,TRUE);

	//for MOSI
	masterSPI2.GPIOPinConfig.GPIO_PinNum = PIN15;
	GPIO_Init(&masterSPI2);

	//for MISO
//	masterSPI2.GPIOPinConfig.GPIO_PinNum = PIN14;
//	GPIO_Init(&masterSPI2);

	//for SCLK
	masterSPI2.GPIOPinConfig.GPIO_PinNum = PIN10;
	GPIO_Init(&masterSPI2);

	//for NSS
//	masterSPI2.GPIOPinConfig.GPIO_PinNum = PIN9;
//	GPIO_Init(&masterSPI2);



	// initializing SPI2 port

	memset(&SPI2Config,0,sizeof(SPI2Config));

	SPI2Config.pSPI 					 = SPI2;
	SPI2Config.sSPIConfig.SPI_deviceMode = SPI_MASTER;
	SPI2Config.sSPIConfig.SPI_SS         = SPI_SW_EN;
	SPI2Config.sSPIConfig.SPI_CPHA		 = SPI_CLK_TRAILING_EDGE;
	SPI2Config.sSPIConfig.SPI_DFF        = SPI_SIXTEEN_BIT;


	SPI_PeriClkCtrl(SPI2,TRUE);
	SPI_Init(&SPI2Config);





	SPI_SendData(SPI2Config.pSPI,string,strlen(string));

	SPI_DeInit(SPI2Config.pSPI);

//	SPI2Config.sSPIConfig.SPI_DFF = SPI_SIXTEEN_BIT;
//	SPI_PeriClkCtrl(SPI2,TRUE);
//	SPI_Init(&SPI2Config);
//	SPI_SendData(SPI2Config.pSPI,array,sizeof(array));


	while(1)
	{
	//	SPI_SendData(SPI2Config.pSPI,string,sizeof(string));
	}

	return 0;

}
