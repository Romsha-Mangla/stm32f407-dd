/*
 * ArdSlaveCommand.c
 *
 *  Created on: 29-Mar-2024
 *      Author: romsh
 */


#include "stm32f407xx_spi_drv.h"
#include "stm32f407xx_gpio_drv.h"
#include<string.h>
#include<stdio.h>

#define SLAVE_ACK				0xF5
#define SLAVE_NACK				0xA5

//command codes
#define COMMAND_LED_CTRL          0x50
#define COMMAND_SENSOR_READ       0x51
#define COMMAND_LED_READ          0x52
#define COMMAND_PRINT             0x53
#define COMMAND_ID_READ           0x54
#define TOTAL_COMMANDS				5

#define ARD_LED						9
#define LED_ON     					1
#define LED_OFF   				    0

//arduino analog pins
#define ANALOG_PIN0   0
#define ANALOG_PIN1   1
#define ANALOG_PIN2   2
#define ANALOG_PIN3   3
#define ANALOG_PIN4   4


void delay(uint32_t count){
	__vo int i;
	//delay
	for(i=0;i<count;i++);
}

int main()
{
	uint8_t command[]={COMMAND_LED_CTRL,COMMAND_SENSOR_READ,COMMAND_LED_READ,COMMAND_PRINT,COMMAND_ID_READ};
	uint8_t commandArguments[] = {ARD_LED,LED_ON,ANALOG_PIN0,};
	char *string = "hi";
	size_t size = strlen(string);
	uint8_t response[]={0,0,0,0,0};
	uint8_t SensorReading =0;
	uint8_t LedValue=0;
	char boardId[20];
	uint8_t dummyWrite = 'M';
	uint8_t i = 0;
	GPIO_Handle_t masterSPI2;
	GPIO_Handle_t UserButton;
	SPI_Handle_t SPI2Config;

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

	memset(&UserButton,0,sizeof(GPIO_Handle_t));

	//configure the user button
	UserButton.pGPIO = GPIOA;
	UserButton.GPIOPinConfig.GPIOPort    = PORTA;
	UserButton.GPIOPinConfig.GPIO_PinNum = PIN0;
	UserButton.GPIOPinConfig.GPIOspeed   = HIGH_SPEED;
	UserButton.GPIOPinConfig.GPIOmode    = INPUT_MODE;
	UserButton.GPIOPinConfig.GPIOPullUpPullDown = NO_PU_NO_PD;

	//initializing the GPIO PA0 pin
	GPIO_Init(&UserButton);


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
	SPI2Config.sSPIConfig.SPI_speed		 = PRESCALER_64;

	SPI_Init(&SPI2Config);

	printf("hello world\n\r");

	while(1)
	{

		while(!GPIO_readFrmInputPin(GPIOA,PIN0));

		//to handle button de-bouncing
		delay(100000);

		//enable SPI2
		SPI_Enable(SPI2);

		//send the command
		SPI_SendData(SPI2Config.pSPI,&command[i],1);

		//when data are received and the previous data have not yet been read from
		//SPI_DR. As a result, the incoming data are lost. In this case, the receive
		//buffer contents are not updated with the newly received data from
		//the transmitter device. A read operation to the SPI_DR register returns the previous
		//correctly received data. All other subsequently transmitted half-words are lost
		//Clearing the OVR bit is done by a read operation on the SPI_DR register followed by a read
		//access to the SPI_SR register.
		//therefore dummy read
		SPI_ReceiveData(SPI2Config.pSPI,&dummyWrite, 1);

		dummyWrite = 'A';

		//send the dummy byte to receive ACK, this is because slave will not send anything unless
		//master initiates a transfer
		SPI_SendData(SPI2Config.pSPI,&dummyWrite,1);


		//receive the ack or nack
		SPI_ReceiveData(SPI2Config.pSPI,&response[i], 1);

		//send command arguments based on slave response
		if(response[i]==SLAVE_ACK)
		{
			switch(command[i])
			{
			case COMMAND_LED_CTRL:
									SPI_SendData(SPI2Config.pSPI,&commandArguments[0],1);
									//dummy read
									SPI_ReceiveData(SPI2Config.pSPI,&dummyWrite, 1);
									//dummy read
									SPI_SendData(SPI2Config.pSPI,&commandArguments[1],1);
									SPI_ReceiveData(SPI2Config.pSPI,&dummyWrite, 1);
									break;

			case COMMAND_SENSOR_READ:

									SPI_SendData(SPI2Config.pSPI,&commandArguments[2],1);
									//dummy read to clear RxNe flag
									SPI_ReceiveData(SPI2Config.pSPI,&dummyWrite, 1);
									//dummy write to fetch sensor read from slave
									SPI_SendData(SPI2Config.pSPI,&dummyWrite,1);
									delay(25000);
									//read the actual sensor reading
									SPI_ReceiveData(SPI2Config.pSPI,&SensorReading, 1);
									break;

			case COMMAND_LED_READ:
									SPI_SendData(SPI2Config.pSPI,&commandArguments[0],1);
									//dummy read to clear RxNe flag
									SPI_ReceiveData(SPI2Config.pSPI,&dummyWrite, 1);
									//dummy write to fetch led read from slave
									SPI_SendData(SPI2Config.pSPI,&dummyWrite,1);
									delay(10000);
									//read the actual led
									SPI_ReceiveData(SPI2Config.pSPI,&LedValue, 1);
									break;

			case COMMAND_PRINT:
									SPI_SendData(SPI2Config.pSPI,&size,1);
									//dummy read
									//SPI_ReceiveData(SPI2Config.pSPI,&dummyWrite, 1);
									SPI_SendData(SPI2Config.pSPI,string,size);
									//dummy read
									//SPI_ReceiveData(SPI2Config.pSPI,&dummyWrite, 1);
									break;

			case COMMAND_ID_READ:
									//dummy read to clear RxNe flag
									//SPI_ReceiveData(SPI2Config.pSPI,&dummyWrite, 1);
									//dummy write to fetch led read from slave
									SPI_SendData(SPI2Config.pSPI,&dummyWrite,1);
									//delay(10000);
									//read actual id
									SPI_ReceiveData(SPI2Config.pSPI,boardId,10);
									break;
			default:
									break;

			}
			//increment the index into command array
			i++;
			//wrap around if it reaches the limit
			if(i%TOTAL_COMMANDS==0)
				i=0;
		}

		//disable SPI
		SPI_Disable(SPI2);

	}

	return 0;

}
