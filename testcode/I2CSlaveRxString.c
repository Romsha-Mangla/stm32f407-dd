/*
 * I2CSlaveRxString.c
 *
 *  Created on: May 27, 2024
 *      Author: romsha
 */


#include<stdio.h>
#include"stm32f407xx_gpio_drv.h"
#include"stm32f407xx_i2c_drv.h"


#define MY_ADDR					0x69
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

	GPIO_Handle_t I2C1_PB6_PB7; //PB9 is used for Single Wire interface module (swim)
	I2C_Handle_t  I2C1Slave ;
	I2C_Message_t I2CTxMsg_1,I2CTxMsg_2,I2CRxMsg_1 ;
	InterruptHandle I2C_1_EV,I2C_1_ER;
	uint8_t bufferRx = 0;
	uint32_t stringSize = 400;
	uint8_t string[stringSize];

	memset(string,'B',400);
	memset(&I2CTxMsg_1,0,sizeof(I2CTxMsg_1));
	memset(&I2CTxMsg_2,0,sizeof(I2CTxMsg_2));
	memset(&I2CRxMsg_1,0,sizeof(I2CRxMsg_1));

	//configure the I2C Tx message handle
	I2CTxMsg_1.I2CChannel = I2C_1;
	I2CTxMsg_1.msgBuffer  = 0;
	I2CTxMsg_1.msgSize    = 0;
	I2CTxMsg_1.msgStatus  = TX_NOT_STARTED;
	I2CTxMsg_1.slaveAddr  = ARD_ADDR;

	//configure the I2C Tx message handle
	I2CTxMsg_2.I2CChannel = I2C_1;
	I2CTxMsg_2.msgBuffer  = string;
	I2CTxMsg_2.msgSize    = 0;
	I2CTxMsg_2.msgStatus  = TX_NOT_STARTED;
	I2CTxMsg_2.slaveAddr  = ARD_ADDR;

	//configure the I2C Rx message handle
	I2CRxMsg_1.I2CChannel = I2C_1;
	I2CRxMsg_1.msgBuffer  = &bufferRx;
	I2CRxMsg_1.msgLen     = 0;
	I2CRxMsg_1.msgStatus  = READ_NOT_STARTED;
	I2CRxMsg_1.slaveAddr  = ARD_ADDR;

	//I2C pins
	I2C1_PB6_PB7.pGPIO = GPIOB;
	I2C1_PB6_PB7.GPIOPinConfig.GPIOmode    		= ALT_FUNC;
	I2C1_PB6_PB7.GPIOPinConfig.GPIOAltFunc 		= AF4;
	I2C1_PB6_PB7.GPIOPinConfig.GPIOspeed   		= LOW_SPEED;
	I2C1_PB6_PB7.GPIOPinConfig.GPIOPort    		= PORTB;
	I2C1_PB6_PB7.GPIOPinConfig.GPIOOutputType 	= OP_TYPE_OD;

	//I2C1_SCL
	I2C1_PB6_PB7.GPIOPinConfig.GPIO_PinNum = PIN6;
	GPIO_Init(&I2C1_PB6_PB7);

	I2C1_PB6_PB7.GPIOPinConfig.GPIO_PinNum = PIN7;
	GPIO_Init(&I2C1_PB6_PB7);

	//configure I2C1 peripheral
	I2C1Slave.pI2Cx = I2C1;
	I2C1Slave.I2CConfig.I2C_ACKControl 		= I2C_ACK_ENABLE;
	I2C1Slave.I2CConfig.I2C_FMDutyCycle 	= FM_DUTY_CYCLE_2;
	I2C1Slave.I2CConfig.I2C_SCLSpeed    	= I2C_STANDARD_MODE_100K;
	I2C1Slave.I2CConfig.I2C_DeviceAddress 	= ARD_ADDR;

	//configure I2C1 event interrupt
	I2C_1_EV.IRQNum = I2C1_EV;
	I2C_1_EV.IntPri = 2;
	I2C_1_EV.IsEnable = FALSE;

	//configure I2C1 error interrupt
	I2C_1_ER.IRQNum = I2C1_ER;
	I2C_1_ER.IntPri = 1;
	I2C_1_ER.IsEnable = FALSE;

	I2C_IRQConfig(&I2C_1_EV);
	I2C_IRQConfig(&I2C_1_ER);


	I2C_Init(&I2C1Slave);

	while(1){

			if(I2C_ITSlaveRx(&I2C1Slave,&I2CRxMsg_1))
				printf("slave reception started...\n\r");
			while(I2CRxMsg_1.msgStatus!=READ_COMPLETE);
			printf("command received is %x\n\r",*(I2CRxMsg_1.msgBuffer));
			printf("received data length is %ld bytes\n\r",I2CRxMsg_1.msgLen);

			if((*(I2CRxMsg_1.msgBuffer)) == READ_DATA_LEN){
				//update the tx message packet
				I2CTxMsg_1.msgBuffer  = (uint8_t*)(&stringSize);
				I2CTxMsg_1.msgLen  = 0;
				//slave send message to master about length of incoming data
				if(I2C_ITSlaveTx(&I2C1Slave, &I2CTxMsg_1))
					printf("slave sent data length\n\r");
				while(I2CTxMsg_1.msgStatus!=TX_COMPLETE);
				printf("sent data length is %ld bytes\n\r",I2CTxMsg_1.msgLen);
			}
			else if((*(I2CRxMsg_1.msgBuffer)) == READ_DATA){
				//send string to master
				I2CTxMsg_2.msgBuffer = string;
				if(I2C_ITSlaveTx(&I2C1Slave, &I2CTxMsg_2))
					printf("slave started sending string\n\r");
				while(I2CTxMsg_1.msgStatus!=TX_COMPLETE);
				I2CTxMsg_2.msgBuffer += I2CTxMsg_2.msgLen+1;
				printf("sent string length is %ld bytes\n\r",I2CTxMsg_2.msgLen);

		}
	}

	return 0;
}



