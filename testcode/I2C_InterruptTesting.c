/*
 * I2CMasterRxString.c
 *
 *  Created on: May 25, 2024
 *      Author: romsha
 */


#include<stdio.h>
#include<string.h>
#include"stm32f407xx_gpio_drv.h"
#include"stm32f407xx_i2c_drv.h"

#define MY_ADDR					0x61
#define ARD_ADDR				0x68
#define READ_DATA_LEN			0x51
#define READ_DATA				0x52
#define BUFFER_LEN_MAX			32U

void delay(uint32_t count){
	volatile int i;
	//delay
	for(i=0;i<count;i++);
}

int main(){

	GPIO_Handle_t Button_PA6 ;
	GPIO_Handle_t I2C1_PB6_PB7; //PB9 is used for Single Wire interface module (swim)
	I2C_Handle_t  I2C1Master ;
	I2C_Message_t I2CTxMsg,I2CRxMsg ;
	InterruptHandle I2C_1_EV,I2C_1_ER;
	uint8_t bufferTx;
	uint8_t bufferRx[BUFFER_LEN_MAX] = {0};
	uint8_t buttonRead = FALSE;

	//configure GPIO
	//button
	Button_PA6.pGPIO = GPIOA;
	Button_PA6.GPIOPinConfig.GPIOmode = INPUT_MODE;
	Button_PA6.GPIOPinConfig.GPIOPullUpPullDown = PULL_UP	;
	Button_PA6.GPIOPinConfig.GPIO_PinNum = PIN6;
	Button_PA6.GPIOPinConfig.GPIOPort = PORTA;
	Button_PA6.GPIOPinConfig.GPIOspeed = MED_SPEED;
	GPIO_Init(&Button_PA6);

	memset(&I2CTxMsg,0,sizeof(I2CTxMsg));
	//configure the I2C Tx message handle
	I2CTxMsg.I2CChannel = I2C_1;
	I2CTxMsg.msgBuffer  = &bufferTx;
	I2CTxMsg.msgSize     = 1;
	I2CTxMsg.msgStatus  = TX_NOT_STARTED;
	I2CTxMsg.slaveAddr  = ARD_ADDR;
	I2CTxMsg.I2C_RepeatedStart = TRUE;

	memset(&I2CRxMsg,0,sizeof(I2CRxMsg));
	//configure the I2C Rx message handle
	I2CRxMsg.I2CChannel = I2C_1;
	I2CRxMsg.msgBuffer  = &bufferRx[0];
	I2CRxMsg.msgSize    = 1;
	I2CRxMsg.msgStatus  = READ_NOT_STARTED;
	I2CRxMsg.slaveAddr  = ARD_ADDR;
	I2CRxMsg.I2C_RepeatedStart = TRUE;


	//I2C pins
	I2C1_PB6_PB7.pGPIO = GPIOB;
	I2C1_PB6_PB7.GPIOPinConfig.GPIOmode    = ALT_FUNC;
	I2C1_PB6_PB7.GPIOPinConfig.GPIOAltFunc = AF4;
	I2C1_PB6_PB7.GPIOPinConfig.GPIOspeed   = LOW_SPEED;
	I2C1_PB6_PB7.GPIOPinConfig.GPIOPort    = PORTB;
	I2C1_PB6_PB7.GPIOPinConfig.GPIOOutputType = OP_TYPE_OD;

	//I2C1_SCL
	I2C1_PB6_PB7.GPIOPinConfig.GPIO_PinNum = PIN6;
	GPIO_Init(&I2C1_PB6_PB7);

	I2C1_PB6_PB7.GPIOPinConfig.GPIO_PinNum = PIN7;
	GPIO_Init(&I2C1_PB6_PB7);

	//configure I2C1 peripheral
	I2C1Master.pI2Cx = I2C1;
	I2C1Master.I2CConfig.I2C_ACKControl = I2C_ACK_ENABLE;
	I2C1Master.I2CConfig.I2C_FMDutyCycle = FM_DUTY_CYCLE_2;
	I2C1Master.I2CConfig.I2C_SCLSpeed    = I2C_STANDARD_MODE_100K;
	I2C1Master.I2CConfig.I2C_DeviceAddress = MY_ADDR; //doesn't matter because it is master

	//configure interrupt
	I2C_1_EV.IRQNum = I2C1_EV;
	I2C_1_EV.IntPri = 1;
	I2C_1_EV.IsEnable = FALSE;

	I2C_1_ER.IRQNum = I2C1_ER;
	I2C_1_ER.IntPri = 1;
	I2C_1_ER.IsEnable = FALSE;

	I2C_IRQConfig(&I2C_1_EV);
	I2C_IRQConfig(&I2C_1_ER);

	I2C_Init(&I2C1Master);

	while(1){

		//while button is not pressed
		//button is active low
		while((buttonRead = GPIO_readFrmInputPin(GPIOA,PIN6))==TRUE);

		delay(100000);
		if((buttonRead = GPIO_readFrmInputPin(GPIOA,PIN6))==FALSE){
			bufferTx = READ_DATA_LEN;
			//it means button is pressed
			//send the cmd to get data length
			I2C_ITMasterTx(&I2C1Master,&I2CTxMsg);
			while(I2CTxMsg.msgStatus !=TX_COMPLETE);

			//cmd is successfully received by slave
			//get the length
			I2C_ITMasterRx(&I2C1Master,&I2CRxMsg);
			while(I2CRxMsg.msgStatus !=READ_COMPLETE);

			printf("data length is %d\n\r",I2CRxMsg.msgBuffer[0]);

			//store the no. of bytes to be read from
			//slave in the Rx mesg length field
			I2CRxMsg.msgSize =  2;//I2CRxMsg.msgBuffer[0];
			I2CRxMsg.msgBuffer = &I2CRxMsg.msgBuffer[0];

			//send the cmd to get data
			bufferTx = READ_DATA;
			I2C_ITMasterTx(&I2C1Master,&I2CTxMsg);
			while(I2CTxMsg.msgStatus !=TX_COMPLETE);

			I2CRxMsg.I2C_RepeatedStart = DISABLE;
			I2C_ITMasterRx(&I2C1Master,&I2CRxMsg);
			while(I2CRxMsg.msgStatus !=READ_COMPLETE);

			printf("%s\n\r",&I2CRxMsg.msgBuffer[1]);

			buttonRead = TRUE;
		}
		else{
			delay(1000000);
		}
	}
	return 0;
}



