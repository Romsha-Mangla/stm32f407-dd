/*
 * USART_Stm32SendToArd.c
 *
 *  Created on: Jun 2, 2024
 *      Author: romsha
 */


#include "stm32f407xx_usart_drv.h"
#include "stm32f407xx_gpio_drv.h"
#include<stdio.h>

USART_Message_t USART_Tx_Handle;

void delay(uint32_t count){
	volatile int i;
	//delay
	for(i=0;i<count;i++);
}

int main(){

	USART_Handle_t USART_Handle;
	GPIO_Handle_t USART1_PB6_PB7,Button_PA6;
	char * string[] = {"This is STM32F407VGT6\n\r","How are you?\r\n","Are you okay\r\n?","Do you need help?\n\r"};
	uint32_t i=0;
	uint8_t buttonRead = FALSE;

	memset(&USART_Handle,0,sizeof(USART_Handle_t));
	memset(&USART1_PB6_PB7,0,sizeof(GPIO_Handle_t));

	//configure GPIO
	//button
	Button_PA6.pGPIO = GPIOA;
	Button_PA6.GPIOPinConfig.GPIOmode = INPUT_MODE;
	Button_PA6.GPIOPinConfig.GPIOPullUpPullDown = PULL_UP	;
	Button_PA6.GPIOPinConfig.GPIO_PinNum = PIN6;
	Button_PA6.GPIOPinConfig.GPIOPort = PORTA;
	Button_PA6.GPIOPinConfig.GPIOspeed = HIGH_SPEED;
	GPIO_Init(&Button_PA6);

	//baudrate: 115200
	//frame format: 1 stop bit, 8 bits, no parity
	//PB6- USART1_TX
	//PB7- USART1_RX

	//USART1 pins
	USART1_PB6_PB7.pGPIO = GPIOB;
	USART1_PB6_PB7.GPIOPinConfig.GPIOmode    = ALT_FUNC;
	USART1_PB6_PB7.GPIOPinConfig.GPIOAltFunc = AF7;
	USART1_PB6_PB7.GPIOPinConfig.GPIOspeed   = HIGH_SPEED;
	USART1_PB6_PB7.GPIOPinConfig.GPIOPort    = PORTB;
	USART1_PB6_PB7.GPIOPinConfig.GPIOPullUpPullDown = PULL_UP;


	//USART_TX
	USART1_PB6_PB7.GPIOPinConfig.GPIO_PinNum = PIN6;
	GPIO_Init(&USART1_PB6_PB7);

	//USART_RX
	USART1_PB6_PB7.GPIOPinConfig.GPIO_PinNum = PIN7;
	GPIO_Init(&USART1_PB6_PB7);

	//configuring USART1
	USART_Handle.pUSARTx =  USART1;
	USART_Handle.USARTConfig.USART_BaudRate = USART_BAUD_115200;
	USART_Handle.USARTConfig.USART_Mode = USART_TX_RX;
	USART_Handle.USARTConfig.USART_Parity = USART_NO_PARITY	;
	USART_Handle.USARTConfig.USART_StopBit = USART_ONE_BIT_STOP;

	//initialize USART1
	USART_Init(&USART_Handle);

	//configure USART1 TX message handle
	USART_Tx_Handle.USARTChannel = USART_1;
	USART_Tx_Handle.errorType    = 0;

	//configure USART1 RX message handle
	USART_Tx_Handle.USARTChannel = USART_1;
	USART_Tx_Handle.errorType    = 0;


	while(1){

		//while button is not pressed
		//button is active low
		while((buttonRead = GPIO_readFrmInputPin(GPIOA,PIN6))==TRUE);

		delay(100000);

		if((buttonRead = GPIO_readFrmInputPin(GPIOA,PIN6))==FALSE){
			USART_Tx_Handle.msgBuffer    = (uint8_t*)string[i%4];
			USART_Tx_Handle.msgSize      = strlen(string[i%4]);
			USART_Tx_Handle.msgStatus    = TX_NOT_STARTED;
			i++;
			if(USART_ITTxData(&USART_Handle, &USART_Tx_Handle)){

				while(USART_Tx_Handle.msgStatus==TX_COMPLETE);

			}
			else{
				printf("Tx failed\n\r");
			}
		}
	}
	return 0;
}
