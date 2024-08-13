/*
 * stm32f407xx_i2c_drv.h
 *
 *  Created on: 26-Mar-2024
 *      Author: romsh
 */

#ifndef INC_STM32F407XX_I2C_DRV_H_
#define INC_STM32F407XX_I2C_DRV_H_

#include "stm32f407Header.h"
#include "interrupt.h"
#include <assert.h>

/*
 * application msg status
 */
#define TX_NOT_STARTED			0xA1
#define TX_IN_PROGRESS			0xA2
#define	TX_COMPLETE				0xA3
#define TX_ERROR				0xA4
#define READ_NOT_STARTED		0xB1
#define READ_IN_PROGRESS		0xB2
#define READ_COMPLETE			0xB3
#define READ_ERROR				0xB4
#define I2C_AF_ERROR			0xC1
#define I2C_BERR_ERROR			0xC2
#define I2C_ARLO_ERROR			0xC3
#define I2C_OVR_ERROR			0xC4


#define WRITE_ENABLE_BITMASK 	0xFE
#define READ_ENABLE_BITMASK		0x01

/*
 * Rise time
 */
#define T_RISE_SM_NS			1000 //maximum rise time in standard mode as per I2C specification
#define T_RISE_FM_NS			300	 //maximum rise time in fast mode as per I2C specification

/*
 * I2C CR1 Register
 */
#define SWRST				15						//software reset
#define ALERT				13						//SMBus alert
#define PEC					12						//Packet alert checking
#define POS					11						//acknowledge/PEC Position
#define ACK					10						//acknowledge
#define STOP				9						//stop generation
#define START				8						//start
#define NOSTRETCH			7						//clock stretching disable
#define ENGC				6						//general call enable
#define ENPEC				5						//PEC enable
#define ENARP				4						//ARP enable
#define SMBTYPE				3						//SMBus type
#define SMBUS				1						//SMBus mode
#define PE					0						//Peripheral enable

/*
 * I2C CR2 Register
 */
#define LAST				12						//DMA Last transfer
#define DMAEN				11						//DMA enable
#define ITBUFEN				10						//Buffer interrupt enable
#define ITEVTEN				9						//Event interrupt enable
#define ITERREN				8						//Error interrupt enable
#define FREQ				0						//frequency 5:0 ( 5 bit freq no.)

/*
 * I2C Own Register 1
 */
#define ADDMODE				15						//Addressing mode
#define SHOULD_BE_ONE		14						//should be always be kept 1 by software
#define INFACE_ADD_9_8		8						//Interface address 9:8
#define INFACE_ADD_7_1		1						//Interface address 7:1
#define INFAC_ADD_0			0						//bit 0 of 10 bit addressing

/*
 * I2C Own Register 2
 */
#define INFACE_DUADD_7_1	1					//bit 7:1 of dual address
#define ENDUAL				0					//enable/disable dual address mode

/*
 * DR of I2C
 */
#define DR_FIRST_BIT		1
/*
 * Status Register 1 I2C_SR1
 */
#define SMBALERT			15					//SMBus alert
#define TIMEOUT				14					//timeout or flow error
#define PECERR				12					//PEC Error in reception
#define OVR					11					//Overrun/Underrun
#define AF					10					//Acknowledge failure
#define ARLO				9					//Arbitration lost (master mode)
#define BERR				8					//Bus Error
#define TxE					7					//Data register emp(Tx)
#define RxNE				6					//Data register not emp(Rx)
#define STOPF				4					//Stop detection(slave mode)
#define ADD10				3					//10-bit header sent (Master mode)
#define BTF					2					//Byte transfer finished
#define ADDR				1					//Address sent (master mode)/matched (slave mode)
#define SB					0					//Start bit (Master mode)

/*
 * Status Register 2 I2C_SR2
 */
#define PEC_7_0				8					//Packet error checking register
#define DUALF				7					//Dual flag (Slave mode)
#define SMBHOST				6					//SMBus host header (Slave mode)
#define SMBDEFAULT			5					//SMBus device default address (Slave mode)
#define GENCALL				4					//General call address (Slave mode)
#define TRA					2					//Transmitter/receiver
#define BUSY				1					//Bus busy
#define MSL					0					//master slave

/*
 * I2C CCR- clock control register
 */
#define F_S					15					//I2C master mode selection
#define DUTY				14					//Fm mode duty cycle
#define CCR_11_0			0					//Clock control register in Fm/Sm mode (Master mode)

/*
 * I2C TRISE register
 */
#define TRISE_5_0			0					//Maximum rise time in Fm/Sm mode (Master mode)

/*
 * I2C SCL speed
 */
#define I2C_STANDARD_MODE_50K		50000
#define I2C_STANDARD_MODE_100K		100000
#define I2C_FAST_MODE_200K			200000
#define I2C_FAST_MODE_400K			400000

/*
 * ACK control
 */
#define I2C_ACK_DISABLE			0
#define I2C_ACK_ENABLE			1

/*
 * FM duty cycle
 */
#define FM_DUTY_CYCLE_2			0
#define FM_DUTY_CYCLE_9_16		1

/*
 * I2C channel
 */
#define I2C_1					1
#define I2C_2					2
#define I2C_3					3

typedef struct{
	uint32_t I2C_SCLSpeed;
	uint8_t  I2C_DeviceAddress;
	uint8_t  I2C_ACKControl;
	uint8_t  I2C_FMDutyCycle;
}I2C_config_t;

typedef struct{
	I2C_reg_t *pI2Cx;
	I2C_config_t I2CConfig;
}I2C_Handle_t;

typedef struct{
	uint32_t msgLen;  	//holds the no. of bytes received or transmitted as of now
	uint32_t msgSize; 	//total no. of bytes to be received or sent by master
	uint8_t *msgBuffer;
	uint8_t errorType; 	//holds the type of error
	uint8_t msgStatus;
	uint8_t I2CChannel;
	uint8_t slaveAddr;
	uint8_t I2C_RepeatedStart;
	uint8_t IsACKEnabled;
	uint8_t IsMaster;
}I2C_Message_t;

/*
 * I2C initialization
 */
void I2C_Init(I2C_Handle_t *I2CHandle);

/*
 * Enable I2C
 */
void I2C_Enable(I2C_reg_t *pI2Cx);

/*
 * I2C_Disable
 */
void I2C_Disable(I2C_reg_t *pI2Cx);

/*
 * I2C clock stretch disable
 */
void I2C_ClockStretchDisable(I2C_reg_t *pI2Cx);

/*
 * I2C clock stretch Enable
 */
void I2C_ClockStretchEnable(I2C_reg_t *pI2Cx);

/*
 * I2C_StopTransaction
 */
void I2C_StopTransaction(I2C_reg_t *pI2Cx);

/*
 * I2C reset
 */
void I2C_reset(I2C_reg_t *pI2Cx);

/*
 * Master Tx
 */
void I2C_MasterTx(I2C_Handle_t *I2CHandle,I2C_Message_t *Msg);

/*
 * master Rx
 */
void I2C_MasterRx(I2C_Handle_t *I2CHandle,I2C_Message_t *Msg);

/*
 * slave Tx
 */
void I2C_SlaveTx(I2C_Handle_t *I2CHandle,I2C_Message_t *Msg);

/*
 * slave rx
 */
void I2C_SlaveRx(I2C_Handle_t *I2CHandle,I2C_Message_t *Msg);

/*
 * Master Interrupt Tx
 */
uint8_t I2C_ITMasterTx(I2C_Handle_t *I2CHandle, I2C_Message_t *Msg);

/*
 * Master Interrupt Rx
 */
uint8_t I2C_ITMasterRx(I2C_Handle_t *I2CHandle, I2C_Message_t *Msg);

/*
 * Slave Interrupt Tx
 */
uint8_t I2C_ITSlaveTx(I2C_Handle_t *I2CHandle, I2C_Message_t *Msg);

/*
 * Slave Interrupt Rx
 */
uint8_t I2C_ITSlaveRx(I2C_Handle_t *I2CHandle, I2C_Message_t *Msg);

/*
 *  I2C interrupt config
 */
void I2C_IRQConfig(InterruptHandle * pIntHandle);

#endif /* INC_STM32F407XX_I2C_DRV_H_ */
