/*
 * USART_Stm32RxTxFromToArd.c
 *
 *  Created on: 03-Jun-2024
 *      Author: romsh
 */


#include "stm32f407xx_usart_drv.h"
#include "stm32f407xx_gpio_drv.h"


USART_Message_t USART_Tx_Handle,USART_Rx_Handle;
uint8_t RxBuf[10];

void delay(uint32_t count){
	volatile int i;
	//delay
	for(i=0;i<count;i++);
}

int main(){

	USART_Handle_t USART_Handle;
	GPIO_Handle_t USART1_PB6_PB7,Button_PA6;
	InterruptHandle USART1_Int;
	char *string[] = {"abcd","Hi I am Romsha","c123WX","BlahHI"};
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
	USART_Handle.USARTConfig.USART_Mode = USART_TX_RX	;
	USART_Handle.USARTConfig.USART_Parity = USART_NO_PARITY	;
	USART_Handle.USARTConfig.USART_StopBit = USART_ONE_BIT_STOP;

	//configure the interrupt
	USART1_Int.IRQNum = USART1_IRQ;
	USART1_Int.IntPri = 17;

	USART_IRQConfig(&USART1_Int);


	//initialize USART1
	USART_Init(&USART_Handle);

	//configure USART1 TX message handle
	USART_Tx_Handle.USARTChannel = USART_1;
	USART_Tx_Handle.errorType    = 0;
	USART_Tx_Handle.msgBuffer    = (uint8_t*)string[0];
	USART_Tx_Handle.msgSize      = strlen(string[0]);
	USART_Tx_Handle.msgStatus    = TX_NOT_STARTED;

	//configure USART1 TX message handle
	USART_Rx_Handle.USARTChannel = USART_1;
	USART_Rx_Handle.errorType    = 0;
	USART_Rx_Handle.msgBuffer    = RxBuf;
	USART_Rx_Handle.msgSize      = strlen(string[0]);
	USART_Rx_Handle.msgStatus    = READ_NOT_STARTED;




	while(1){
		//while button is not pressed
				//button is active low
				while((buttonRead = GPIO_readFrmInputPin(GPIOA,PIN6))==TRUE);

				delay(100000);
				if((buttonRead = GPIO_readFrmInputPin(GPIOA,PIN6))==FALSE){
				//prepare the USART channel for receiving RX interrupt
				USART_ITRxData(&USART_Handle,&USART_Rx_Handle);
				USART_TxData(&USART_Handle,&USART_Tx_Handle);

				while(USART_Rx_Handle.msgStatus!=READ_COMPLETE);
				//add the null character at the last
				USART_Rx_Handle.msgBuffer[USART_Rx_Handle.msgLen] = '\0';
				printf("msglen = %lu and msgSize=%lu\n",USART_Rx_Handle.msgLen,USART_Rx_Handle.msgSize);
				//print the received string onto data trace
				printf("%s\n\r",USART_Rx_Handle.msgBuffer);
				i++;
				//prepare the TX message packet to send next string
				USART_Tx_Handle.msgBuffer    = (uint8_t*)string[i%4];
				USART_Tx_Handle.msgSize      = strlen(string[i%4]);
				USART_Tx_Handle.msgLen       = 0;

				//prepare the RX message buffer for next string
				USART_Rx_Handle.msgBuffer = RxBuf;
				USART_Rx_Handle.msgSize  = strlen(string[i%4]);
				}
	}
	return 0;
}
