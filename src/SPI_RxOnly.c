/*
 * SPIRxOnly.c
 *
 *  Created on: 01-May-2024
 *      Author: romsh
 */

#include"stm32f407xx_spi_drv.h"
#include"stm32f407xx_gpio_drv.h"
#include<string.h>
#include<stdio.h>


volatile uint8_t IsDataReady = FALSE;

void delay(uint32_t count){
	__vo int i;
	//delay
	for(i=0;i<count;i++);
}
uint8_t pBuffer[500];

int main(void){
	GPIO_Handle_t masterSPI2,SlaveInformMaster;
	SPI_Handle_t SPI2Config;
	InterruptHandle SPI2InterruptHandle, SlaveInformMasterIntHndl;
	SPI_Message_t SPIMsgHandleRx,SPIMsgHandleTx;
	uint8_t ch=1;
	uint8_t pBuffer[500];
	int i = 0;

	memset(pBuffer,'-',500);


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
	//for MISO
	masterSPI2.GPIOPinConfig.GPIO_PinNum = PIN14;
	GPIO_Init(&masterSPI2);

/********************initializing SPI2 port**************************/

	memset(&SPI2Config,0,sizeof(SPI_Handle_t));

	SPI2Config.pSPI 					 = SPI2;
	SPI2Config.sSPIConfig.SPI_deviceMode = SPI_MASTER;
	SPI2Config.sSPIConfig.SPI_busConfig  = FULL_DUPLEX;
	SPI2Config.sSPIConfig.SPI_SS         = SPI_SW_DIS;
	SPI2Config.sSPIConfig.SPI_CPHA		 = SPI_CLK_LEADING_EDGE;
	SPI2Config.sSPIConfig.SPI_DFF        = SPI_EIGHT_BIT;
	SPI2Config.sSPIConfig.SPI_speed		 = PRESCALER_256;

	SPI_Init(&SPI2Config);

/******************SPI2 interrupt configuration***********************************/
	SPI2InterruptHandle.IRQNum = SPI2_IRQ;
	SPI2InterruptHandle.IntPri = 0;
	SPI2InterruptHandle.IsEnable = TRUE;

	SPI_IRQConfig(&SPI2InterruptHandle);

/*******************SPI2 message initialization**********************************/
	memset(&SPIMsgHandleRx,0,sizeof(SPIMsgHandleRx));

	SPIMsgHandleRx.SpiChannel = SPI_2;
	SPIMsgHandleRx.Len = 1;

	memset(&SPIMsgHandleTx,0,sizeof(SPIMsgHandleTx));
	SPIMsgHandleTx.Buffer = &ch;
	SPIMsgHandleTx.SpiChannel = SPI_2;
	SPIMsgHandleTx.Len= 1;

/**********************configure GPIO pin PD6 to receive interrupt from slave*******/
	memset(&SlaveInformMaster,0,sizeof(SlaveInformMaster));

	SlaveInformMaster.pGPIO = GPIOC;
	SlaveInformMaster.GPIOPinConfig.GPIOmode = GPIO_INT_FT;
	SlaveInformMaster.GPIOPinConfig.GPIOPullUpPullDown = NO_PU_NO_PD;
	SlaveInformMaster.GPIOPinConfig.GPIOspeed = LOW_SPEED;
	SlaveInformMaster.GPIOPinConfig.GPIO_PinNum = PIN4;
	SlaveInformMaster.GPIOPinConfig.GPIOPort = PORTC;

	GPIO_Init(&SlaveInformMaster);

/**********************GPIO interrupt configuration*************************************/

	memset(&SlaveInformMasterIntHndl,0,sizeof(SlaveInformMasterIntHndl));
	SlaveInformMasterIntHndl.IsEnable = TRUE;
	SlaveInformMasterIntHndl.IRQNum = EXTI4_IRQ;
	SlaveInformMasterIntHndl.IntPri = 2;

	GPIO_IRQConfig(&SlaveInformMasterIntHndl);

/***************************************************************************************/


	while(1){

		//keep looping until data ready signal is received from slave
		while(!IsDataReady);

		//disable the interrupt
		GPIO_DisableInterrupt(EXTI4_IRQ);

		//set the flag to false
		IsDataReady = FALSE;

		//enable the SPI
		SPI_Enable(SPI2Config.pSPI);

		//do a dummy write to initiate transfer from slave
		//since it is a FULL DUPLEX configuration

		while(i<500){
		//SPI_SendData(SPI2, &ch, 1);
		//SPI_ReceiveData(SPI2, &pBuffer[i], 1);
		SPIMsgHandleRx.Buffer = &pBuffer[i];
		//set up the board to receive message from uno
		SPI_ReceiveDataInt(&SPIMsgHandleRx);
		//send the dummy message to generate clock for the slave
		SPI_SendDataInt(&SPIMsgHandleTx);
		//added a delay so that uno can update its data register
		//stm32 service the interrupt and read the data before index i
		//is updated
		delay(10000);
		if(pBuffer[i]=='\0')
			break;
		i++;
		//delay(10000);
		}
		i=0;
		//Disable the SPI
		SPI_Disable(SPI2Config.pSPI);

		printf("%s\n\r",pBuffer);

		//enable the slave interrupt
		GPIO_EnableInterrupt(EXTI4_IRQ);


	 }

	return 0;
}


void EXTI4_IRQHandler(void)
{
	//call the GPIO interrupt handler
	GPIO_IRQHandling(PIN4);

	IsDataReady = TRUE;
}
