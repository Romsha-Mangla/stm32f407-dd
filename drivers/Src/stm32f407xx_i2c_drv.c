/*
 * stm32f407xx_i2c_drv.c
 *
 *  Created on: 26-Mar-2024
 *      Author: romsha
 */

#include "stm32f407xx_i2c_drv.h"
#include <stdio.h>

//global I2C message structures
static I2C_Message_t *I2C_Msg[I2C_3] = { 0 };

static void I2C_ITMasterStopTx(I2C_reg_t *pI2Cx,uint8_t ChannelNo);
static void I2C_ITMasterStopRx(I2C_reg_t *pI2Cx, uint8_t ChannelNo);
static void I2C_ITSlaveStopTx(I2C_reg_t *pI2Cx ,uint8_t ChannelNo);
static void I2C_ITSlaveStopRx(I2C_reg_t *pI2Cx ,uint8_t ChannelNo);
static void I2C_IRQ_EV_Handler(I2C_reg_t *pI2Cx, uint8_t ChannelNo);
static void I2C_IRQ_ER_Handler(I2C_reg_t *pI2Cx, uint8_t ChannelNo);
static void I2C_PeriClkCtrl(I2C_reg_t *pI2Cx, uint8_t IsEnabled);
static uint8_t I2C_FindTheAPB1Freq();

/****************************************************************************************************
 * @fn      		  - I2C_Init
 *
 * @brief             - This function configures the I2C channel for use as per user preference
 *
 * @param[in]         - Handle to I2C config
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */

void I2C_Init(I2C_Handle_t *I2CHandle) {
	float CCRValue = 0;
	uint8_t APB1ClkFreq = 0;

	//enable the peripheral clock
	I2C_PeriClkCtrl(I2CHandle->pI2Cx, TRUE);

	//get the APB1 clock frequency
	APB1ClkFreq = I2C_FindTheAPB1Freq();

	//configure the clock(reset value of the register is 0x0000)
	I2CHandle->pI2Cx->I2C_CR2 |= (APB1ClkFreq << FREQ);

	//configure the mode-fm/sm
	if (I2CHandle->I2CConfig.I2C_SCLSpeed > I2C_STANDARD_MODE_100K) {
		//set FM
		I2CHandle->pI2Cx->I2C_CCR = I2CHandle->pI2Cx->I2C_CCR | (SET << F_S);

		//set the duty cycle, reset value of the register is 0x0000
		I2CHandle->pI2Cx->I2C_CCR = I2CHandle->pI2Cx->I2C_CCR
				| (I2CHandle->I2CConfig.I2C_FMDutyCycle << DUTY);

		if (!I2CHandle->I2CConfig.I2C_FMDutyCycle) {
			//if duty is 0, it means T(low)=2*T(Pclk), T(high)= T(pclk) , T(high)+T(low)=3*T(Pclk)*CCR
			CCRValue = (1.0
					/ ((float) (I2CHandle->I2CConfig.I2C_SCLSpeed) * 3.0))
					* ((float) (1000000.0 * APB1ClkFreq));
		} else {
			//if duty is 1, it means T(low)=16*T(Pclk), T(high)= 9*T(pclk) , T(high)+T(low)=25*T(Pclk)*CCR
			CCRValue = (1.0
					/ ((float) (I2CHandle->I2CConfig.I2C_SCLSpeed) * 25.0))
					* ((float) (1000000.0 * APB1ClkFreq));
		}

		//set the clock control value to generate right SCL
		I2CHandle->pI2Cx->I2C_CCR |= (((uint16_t) CCRValue) << CCR_11_0);

		//set the Trise
		I2CHandle->pI2Cx->I2C_TRISE |= ((uint8_t) ((T_RISE_FM_NS
				* (APB1ClkFreq / 1000.0)) + 1)) << TRISE_5_0;

	} else if (I2CHandle->I2CConfig.I2C_SCLSpeed <= I2C_STANDARD_MODE_100K) {
		//set SM
		I2CHandle->pI2Cx->I2C_CCR = I2CHandle->pI2Cx->I2C_CCR & (~(SET << F_S));

		//calculate CCR value
		CCRValue = (1.0 / ((float) (I2CHandle->I2CConfig.I2C_SCLSpeed) * 2.0))
				* ((float) (1000000.0 * APB1ClkFreq));

		//set the clock control value to generate right SCL
		I2CHandle->pI2Cx->I2C_CCR |= (((uint16_t) CCRValue) << CCR_11_0);

		//set the Trise
		I2CHandle->pI2Cx->I2C_TRISE |= ((uint8_t) ((T_RISE_SM_NS
				* (APB1ClkFreq / 1000.0)) + 1)) << TRISE_5_0;
	}

	//set the device address mode
	I2CHandle->pI2Cx->I2C_OAR1 &= (~(SET << ADDMODE));
	//bit should be high
	I2CHandle->pI2Cx->I2C_OAR1 |= (SET << SHOULD_BE_ONE);

	//set the device address
	I2CHandle->pI2Cx->I2C_OAR1 |= ((I2CHandle->I2CConfig.I2C_DeviceAddress)
			<< INFACE_ADD_7_1);

	//enable the I2C peripheral
	I2C_Enable(I2CHandle->pI2Cx);

	//set the ACK control
	if (I2CHandle->I2CConfig.I2C_ACKControl) {
		I2CHandle->pI2Cx->I2C_CR1 |= (SET << ACK);
	} else {
		I2CHandle->pI2Cx->I2C_CR1 &= (~(SET << ACK));
	}

	return;
}

/****************************************************************************************************
 * @fn      		  - I2C_reset
 *
 * @brief             -  This function resets I2C registers
 *
 * @param[in]         -  base address of the I2C peripheral
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void I2C_reset(I2C_reg_t *pI2Cx) {
	I2C_Disable(pI2Cx);

	if (pI2Cx == I2C1) {
		RESET_I2C1();
		SET_I2C1();
	} else if (pI2Cx == I2C2) {
		RESET_I2C2();
		SET_I2C2();
	} else {
		RESET_I2C3();
		SET_I2C3();
	}
}

/****************************************************************************************************
 * @fn      		  - I2C_Enable
 *
 * @brief             - This function enables the I2C peripheral
 *
 * @param[in]         - pointer to I2C peripheral
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  frequency cannot exceed 50Mhz

 */
void I2C_Enable(I2C_reg_t *pI2Cx) {
	pI2Cx->I2C_CR1 |= (SET << PE);
}

/****************************************************************************************************
 * @fn      		  - I2C_Disable
 *
 * @brief             - This function disables the I2C peripheral
 *
 * @param[in]         - pointer to I2C peripheral
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  frequency cannot exceed 50Mhz

 */
void I2C_Disable(I2C_reg_t *pI2Cx) {
	pI2Cx->I2C_CR1 &= (~(SET << PE));
}

/****************************************************************************************************
 * @fn      		  - I2C_ClockStretchDisable
 *
 * @brief             - This function disables the I2C peripheral clock stretch (valid for slave)
 *
 * @param[in]         - pointer to I2C peripheral
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  frequency cannot exceed 50Mhz

 */
void I2C_ClockStretchDisable(I2C_reg_t *pI2Cx) {
	pI2Cx->I2C_CR1 |= (SET << NOSTRETCH);
}

/****************************************************************************************************
 * @fn      		  - I2C_ClockStretchEnable
 *
 * @brief             - This function enables the I2C peripheral clock stretch (valid for slave)
 *
 * @param[in]         - pointer to I2C peripheral
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  frequency cannot exceed 50Mhz

 */
void I2C_ClockStretchEnable(I2C_reg_t *pI2Cx) {
	pI2Cx->I2C_CR1 &= (~(SET << NOSTRETCH));
}

/****************************************************************************************************
 * @fn      		  - I2C_StopTransaction
 *
 * @brief             - This function stops the I2C transaction
 *
 * @param[in]         - pointer to I2C peripheral
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  frequency cannot exceed 50Mhz

 */

void I2C_StopTransaction(I2C_reg_t *pI2Cx) {

	SET_BIT(pI2Cx->I2C_CR1,STOP);
}

/****************************************************************************************************
 * @fn      		  - I2C_MasterTx
 *
 * @brief             - This function sends data to slave
 *
 * @param[in]         - Handle to I2C config
 * @param[in]         - pointer to message packet
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  frequency cannot exceed 50Mhz

 */

void I2C_MasterTx(I2C_Handle_t *I2CHandle, I2C_Message_t *Msg) {
	__vo uint32_t Var = 0;

	//1. The EV5, EV6, EV9, EV8_1 and EV8_2 events stretch SCL low until the end of the
	//corresponding software sequence.
	//2. The EV8 event stretches SCL low if the software sequence is not complete before
	//the end of the next byte transmission.
	//by default, I2C clock stretch is enabled

	//set the msg status
	Msg->msgStatus = TX_IN_PROGRESS;
	Msg->IsMaster = TRUE;

	//generate the start condition to initiate the communication
	I2CHandle->pI2Cx->I2C_CR1 |= (SET << START);

	//EV5 sequence
	//wait till start bit is set
	//1. SB bit is set.Clear SB bit by reading the SR1 register
	//2. followed by putting slave address in data register
	while (!(Var = READ_BIT_VALUE(I2CHandle->pI2Cx->I2C_SR1, SB)))
		;
	//LSB of address byte is 'write enable'
	I2CHandle->pI2Cx->I2C_DR = ((Msg->slaveAddr << DR_FIRST_BIT)
			& WRITE_ENABLE_BITMASK);

	//EV6 sequence
	//1. ADDR bit is set. Clear the ADDR bit by reading the SR1 register
	//2. followed by reading SR2 register
	//wait till ADDR =1, it means slave corresponding to address is present on the bus
	while (!READ_BIT_VALUE(I2CHandle->pI2Cx->I2C_SR1, ADDR))
		;
	Var = I2CHandle->pI2Cx->I2C_SR2;
	//ensure Var is 0 before next line is executed
	if (Var != 0) {
		Var = 0;
	}

	//EV8_1 sequence
	//1.TxE is set, shift register is empty
	//2.write the first data byte onto DR
	while (Var < Msg->msgLen) {
		//EV8 sequence
		//wait till TxE becomes 1. TxE is set when an ACK pulse is received
		while (!READ_BIT_VALUE(I2CHandle->pI2Cx->I2C_SR1, TxE))
			;
		I2CHandle->pI2Cx->I2C_DR = Msg->msgBuffer[Var++];
	}

	//EV8_2 sequence
	//Stop condition should be programmed during EV8_2 event
	//this happens when last byte is transmitted, It means shift register is empty and DR is empty

	while (!READ_BIT_VALUE(I2CHandle->pI2Cx->I2C_SR1, TxE))
		;
	while (!READ_BIT_VALUE(I2CHandle->pI2Cx->I2C_SR1, BTF))
		;

	if (Msg->I2C_RepeatedStart == DISABLE) {
		//stop the transmission now
		I2CHandle->pI2Cx->I2C_CR1 |= (SET << STOP);
	}

	Msg->msgStatus = TX_COMPLETE;
	return;
}

/****************************************************************************************************
 * @fn      		  - I2C_MasterRx
 *
 * @brief             - This function receives data from slave
 *
 * @param[in]         - Handle to I2C config
 * @param[in]         - pointer to message packet
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */

void I2C_MasterRx(I2C_Handle_t *I2CHandle, I2C_Message_t *Msg) {
	__vo uint32_t Var;

	//ACK indicates to slave that one more byte is requested
	//NACK on the other hand indicate that no more data is requested
	//And Stop bit releases the control of the bus

	//1. The EV5, EV6, EV9, EV8_1 and EV8_2 events stretch SCL low until the end of the
	//corresponding software sequence.
	//2. The EV8 event stretches SCL low if the software sequence is not complete before
	//the end of the next byte transmission.
	//by default, I2C clock stretch is enabled

	//set the msg status
	Msg->msgStatus = READ_IN_PROGRESS;
	Msg->IsMaster = TRUE;

	//generate the start condition to initiate the communication
	I2CHandle->pI2Cx->I2C_CR1 |= (SET << START);

	//EV5 sequence
	//wait till start bit is set
	//1. SB bit is set.Clear SB bit by reading the SR1 register
	//2. followed by putting slave address in data register
	while (!READ_BIT_VALUE(I2CHandle->pI2Cx->I2C_SR1, SB))
		;

	//LSB of address byte is 'read enable'
	I2CHandle->pI2Cx->I2C_DR = ((Msg->slaveAddr << DR_FIRST_BIT)
			| READ_ENABLE_BITMASK);

	//EV6 sequence
	//wait till ADDR =1, it means slave corresponding to address is present on the bus
	//1. ADDR bit is set. Clear the ADDR bit by reading the SR1 register
	//2. followed by reading SR2 register

	while (!READ_BIT_VALUE(I2CHandle->pI2Cx->I2C_SR1, ADDR))
		;

	//if there is only one byte to be received
	if (Msg->msgLen == 1) {
		//ACK must be disabled before clearing ADDR bit
		//setting of ADDR bit stretches the clock low
		//both sender and receiver are in wait state
		I2CHandle->pI2Cx->I2C_CR1 &= (~(SET << ACK));

		//dummy read of SR2 to clear ADDR flag
		Var = I2CHandle->pI2Cx->I2C_SR1;
		Var = I2CHandle->pI2Cx->I2C_SR2;

		//read the data byte
		while (!READ_BIT_VALUE(I2CHandle->pI2Cx->I2C_SR1, RxNE))
			;

		//program the STOP bit if repeated start is disabled by user
		if (Msg->I2C_RepeatedStart == DISABLE) {
			I2CHandle->pI2Cx->I2C_CR1 |= (SET << STOP);
		}

		Msg->msgBuffer[0] = I2CHandle->pI2Cx->I2C_DR;
	}
	//if there are only two bytes to be received
	else if (Msg->msgLen == 2) {
		//set ACK low
		I2CHandle->pI2Cx->I2C_CR1 &= (~(SET << ACK));

		//set POC high, it means ACK bit controls the (N)ACK
		//of the next byte which will be received in the shift register.
		I2CHandle->pI2Cx->I2C_CR1 |= (SET << POS);

		//dummy read of SR2 to clear ADDR flag
		Var = I2CHandle->pI2Cx->I2C_SR1;
		Var = I2CHandle->pI2Cx->I2C_SR2;

		//wait till BTF is high
		while (!READ_BIT_VALUE(I2CHandle->pI2Cx->I2C_SR1, BTF))
			;

		//program the STOP bit if repeated start is disabled by user
		if (Msg->I2C_RepeatedStart == DISABLE) {
			I2CHandle->pI2Cx->I2C_CR1 |= (SET << STOP);
		}

		//read data byte 1 in DR and data byte 2 (in shift register)
		Msg->msgBuffer[0] = I2CHandle->pI2Cx->I2C_DR;

		while (!READ_BIT_VALUE(I2CHandle->pI2Cx->I2C_SR1, RxNE))
			;
		Msg->msgBuffer[1] = I2CHandle->pI2Cx->I2C_DR;
	} else {

		//dummy read of SR2 to clear ADDR flag
		Var = I2CHandle->pI2Cx->I2C_SR2;

		//ensure Var is 0 before next line is executed
		if (Var != 0) {
			Var = 0;
		}
		while (Var < (Msg->msgLen - 2)) {
			while (!READ_BIT_VALUE(I2CHandle->pI2Cx->I2C_SR1, RxNE))
				;
			Msg->msgBuffer[Var++] = I2CHandle->pI2Cx->I2C_DR;
		}

		//if BTF is set it means, N-2 is in DR and N-1 is in shift register
		while (!READ_BIT_VALUE(I2CHandle->pI2Cx->I2C_SR1, BTF))
			;

		//disable ACK
		I2CHandle->pI2Cx->I2C_CR1 &= (~(SET << ACK));

		//read data N-2
		Msg->msgBuffer[Var++] = I2CHandle->pI2Cx->I2C_DR;

		//N-1 is in DR and N is in shift register
		while (!READ_BIT_VALUE(I2CHandle->pI2Cx->I2C_SR1, BTF))
			;

		//program the STOP bit if repeated start is disabled by user
		if (Msg->I2C_RepeatedStart == DISABLE) {
			I2CHandle->pI2Cx->I2C_CR1 |= (SET << STOP);
		}

		//read N-1 byte
		Msg->msgBuffer[Var] = I2CHandle->pI2Cx->I2C_DR;

	}

	//update the app message status
	Msg->msgStatus = READ_COMPLETE;

	//restore the original I2C configuration
	if (I2CHandle->I2CConfig.I2C_ACKControl == ENABLE) {
		I2CHandle->pI2Cx->I2C_CR1 |= (SET << ACK);
	}

	return;
}

/****************************************************************************************************
 * @fn      		  - I2C_SlaveTx
 *
 * @brief             - This function sends data to master
 *
 * @param[in]         - Handle to I2C config
 * @param[in]         - Handle to Msg
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  blocking function

 */

void I2C_SlaveTx(I2C_Handle_t *I2CHandle, I2C_Message_t *Msg) {
	//By default the I2C interface operates in Slave mode.
	//As soon as a start condition is detected, the address is received from the SDA line and sent
	//to the shift register. Then it is compared with the address of the interface (OAR1) and with
	//OAR2 (if ENDUAL=1) or the General Call address (if ENGC = 1).
	//if the address is not matched, slaves ignores it(detected as NACK by master) wait for another
	//start condition. The interface generates an acknowledge pulse if the
	//ACK bit is set and The ADDR bit is set by hardware

	//EV1
	//ADDR=1,cleared by reading SR1 followed by reading SR2
	//The slave stretches SCL low until ADDR is cleared and DR filled with the data to be sent
	while (!READ_BIT_VALUE(I2CHandle->pI2Cx->I2C_SR1, ADDR))
		;

	//TRA is set if data bytes are to be transmitted by device
	if (READ_BIT_VALUE(I2CHandle->pI2Cx->I2C_SR2, TRA)) {

		//set the msg status
		Msg->msgStatus = TX_IN_PROGRESS;
		Msg->msgLen = 0;

		while (!READ_BIT_VALUE(I2CHandle->pI2Cx->I2C_SR1, AF)) {

			//EV3-1 sequence
			//wait till TxE becomes 1.
			if (READ_BIT_VALUE(I2CHandle->pI2Cx->I2C_SR1, TxE))
				I2CHandle->pI2Cx->I2C_DR = Msg->msgBuffer[Msg->msgLen++];
		}

		//clear NACK if any
		CLEAR_BIT(I2CHandle->pI2Cx->I2C_SR1, AF);
		Msg->msgStatus = TX_COMPLETE;
		//update the no. of bytes transmitted
		Msg->msgLen -= 1;
	}

	return;
}

/****************************************************************************************************
 * @fn      		  - I2C_SlaveRx
 *
 * @brief             - This function sends data from master
 *
 * @param[in]         - Handle to I2C config
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  make sure the msg buffer is sufficiently large to hold the data received

 */

void I2C_SlaveRx(I2C_Handle_t *I2CHandle, I2C_Message_t *Msg) {
	__vo uint32_t Var = 0;

	//EV1
	//ADDR=1,cleared by reading SR1 followed by reading SR2
	//The slave stretches SCL low until ADDR is cleared and DR is read
	while (!READ_BIT_VALUE(I2CHandle->pI2Cx->I2C_SR1, ADDR))
		;

	//TRA is cleared if data bytes are to be received by device
	if (!READ_BIT_VALUE(I2CHandle->pI2Cx->I2C_SR2, TRA)) {

		//set the msg status
		Msg->msgStatus = READ_IN_PROGRESS;

		//keep receiving data until stop is sensed
		while (!READ_BIT_VALUE(I2CHandle->pI2Cx->I2C_SR1, STOPF)) {

			if (READ_BIT_VALUE(I2CHandle->pI2Cx->I2C_SR1, RxNE))
				Msg->msgBuffer[Var++] = I2CHandle->pI2Cx->I2C_DR;
		}

		//if stop is detected, clear the STOPF bit by
		//reading SR1 register followed by write to CR1 register
		Var = I2CHandle->pI2Cx->I2C_SR1;
		I2CHandle->pI2Cx->I2C_CR1 |= 0x0;

		//length of received message
		Msg->msgLen = Var - 1;
		Msg->msgStatus = READ_COMPLETE;

	}
	return;
}

/****************************************************************************************************
 * @fn      		  - I2C_ITMasterTx
 *
 * @brief             - This function sends data to slave
 *
 * @param[in]         - Handle to I2C config
 * @param[in]         - pointer to message packet
 * @param[in]         -
 *
 * @return            -  return TRUE on success
 *
 * @Note              -  frequency cannot exceed 50Mhz

 */

uint8_t I2C_ITMasterTx(I2C_Handle_t *I2CHandle, I2C_Message_t *Msg) {

	uint8_t Channel = 0;
	uint8_t Success = FALSE;

	if(Msg==0||I2CHandle==0){
#ifdef APP_DEBUG
		printf("invalid arguments passed to I2C_ITMasterTx\n\r");
#endif
		return Success;
	}
	else if (I2C_Msg[Channel] == 0) {

		Channel = Msg->I2CChannel -1;
		//message length should be same as message size
		Msg->msgLen = Msg->msgSize;
		//message status should be set to NOT STARTED
		Msg->msgStatus = TX_NOT_STARTED;
		//set it to master
		Msg->IsMaster = TRUE;

		//enable the ACK bit again
		if(I2CHandle->I2CConfig.I2C_ACKControl){
			SET_BIT(I2CHandle->pI2Cx->I2C_CR1, ACK);
			Msg->IsACKEnabled = TRUE;
		}
		else{
			CLEAR_BIT(I2CHandle->pI2Cx->I2C_CR1, ACK);
			Msg->IsACKEnabled = FALSE;
		}

		I2C_Msg[Channel] = Msg;

		//generate the start condition to initiate the communication
		I2CHandle->pI2Cx->I2C_CR1 |= (SET << START);

		//enable event interrupt
		SET_BIT(I2CHandle->pI2Cx->I2C_CR2, ITBUFEN);
		SET_BIT(I2CHandle->pI2Cx->I2C_CR2, ITEVTEN);
		SET_BIT(I2CHandle->pI2Cx->I2C_CR2, ITERREN);

		Success = TRUE;
   }
	return Success;
}

/****************************************************************************************************
 * @fn      		  - I2C_ITMasterRx
 *
 * @brief             - This function receives data from slave
 *
 * @param[in]         - Handle to I2C config
 * @param[in]         - pointer to message packet
 * @param[in]         -
 *
 * @return            -  return TRUE on successful initiation of communication
 *
 * @Note              -  frequency cannot exceed 50Mhz

 */

uint8_t I2C_ITMasterRx(I2C_Handle_t *I2CHandle, I2C_Message_t *Msg) {

	uint8_t Channel = 0;
	uint8_t Success = FALSE;

	if(Msg==0||I2CHandle==0){
#ifdef APP_DEBUG
		printf("invalid arguments passed to I2C_ITMasterRx\n\r");
#endif
		return Success;
	}
	//if the channel message structure is NULL, it means it is free and can be used
	else if (I2C_Msg[Channel] == 0) {

		Channel = Msg->I2CChannel-1;
		//msglen should be set equal to msgSize
		Msg->msgLen = Msg->msgSize;

		//set it to master
		Msg->IsMaster = TRUE;

		//set the message status
		Msg->msgStatus = READ_NOT_STARTED;

		//enable the ACK bit again
		if(I2CHandle->I2CConfig.I2C_ACKControl){
			SET_BIT(I2CHandle->pI2Cx->I2C_CR1, ACK);
			Msg->IsACKEnabled = TRUE;
		}
		else{
			CLEAR_BIT(I2CHandle->pI2Cx->I2C_CR1, ACK);
			Msg->IsACKEnabled = FALSE;
		}

		I2C_Msg[Channel] = Msg;

		//generate the start condition to initiate the communication
		I2CHandle->pI2Cx->I2C_CR1 |= (SET << START);

		//enable event and error interrupt flags
		SET_BIT(I2CHandle->pI2Cx->I2C_CR2, ITEVTEN);
		SET_BIT(I2CHandle->pI2Cx->I2C_CR2, ITBUFEN);
		SET_BIT(I2CHandle->pI2Cx->I2C_CR2, ITERREN);

		Success = TRUE;

	}
	return Success;
}

/****************************************************************************************************
 * @fn      		  - I2C_ITSlaveTx
 *
 * @brief             - This function is called when device is asked to transmit by master
 *
 * @param[in]         - Handle to I2C config
 * @param[in]         - pointer to message packet
 * @param[in]         -
 *
 * @return            -  return TRUE on Success
 *
 * @Note              -  frequency cannot exceed 50Mhz

 */

uint8_t I2C_ITSlaveTx(I2C_Handle_t *I2CHandle, I2C_Message_t *Msg) {

	uint8_t Success = FALSE;
	uint8_t Channel = 0;

	if(Msg==0||I2CHandle==0){
#ifdef APP_DEBUG
		printf("invalid arguments passed to I2C_ITSlaveTx\n\r");
#endif
	}
	else if (I2C_Msg[Channel] == 0) {

		Channel = Msg->I2CChannel-1;
		//set the ACK status
		if(I2CHandle->I2CConfig.I2C_ACKControl){
			SET_BIT(I2CHandle->pI2Cx->I2C_CR1, ACK);
			Msg->IsACKEnabled = TRUE;
		}
		else{
			CLEAR_BIT(I2CHandle->pI2Cx->I2C_CR1, ACK);
			Msg->IsACKEnabled = FALSE;
		}

		Msg->msgStatus = TX_NOT_STARTED;
		Msg->msgLen    = 0;
		Msg->msgSize   = 0;
		Msg->IsMaster  = FALSE;

		I2C_Msg[Channel] = Msg;

		//enable event interrupt
		SET_BIT(I2CHandle->pI2Cx->I2C_CR2, ITBUFEN);
		SET_BIT(I2CHandle->pI2Cx->I2C_CR2, ITEVTEN);
		SET_BIT(I2CHandle->pI2Cx->I2C_CR2, ITERREN);

		Success = TRUE;
	}
	return Success;
}

/****************************************************************************************************
 * @fn      		  - I2C_ITSlaveRx
 *
 * @brief             - This function is called when device is asked to receive by master
 *
 * @param[in]         - Handle to I2C config
 * @param[in]         - pointer to message packet
 * @param[in]         -
 *
 * @return            -  return TRUE on success
 *
 * @Note              -  frequency cannot exceed 50Mhz

 */

uint8_t I2C_ITSlaveRx(I2C_Handle_t *I2CHandle, I2C_Message_t *Msg) {

	uint8_t Success = FALSE;
	uint8_t Channel = 0;

	if(Msg==0||I2CHandle==0){
#ifdef APP_DEBUG
		printf("invalid arguments passed to I2C_ITSlaveRx\n\r");
#endif
	}
	else if (I2C_Msg[Channel] == 0) {

		Channel = Msg->I2CChannel-1;
		// message size should be 0 since slave doesn't know in advance
		// how many bytes it is to receive from master
		Msg->msgSize   = 0;
		Msg->msgLen    = 0;
		Msg->msgStatus = READ_NOT_STARTED;
		Msg->IsMaster  = FALSE;

		//enable the ACK bit again
		if(I2CHandle->I2CConfig.I2C_ACKControl){
			SET_BIT(I2CHandle->pI2Cx->I2C_CR1, ACK);
			Msg->IsACKEnabled = TRUE;
		}
		else{
			CLEAR_BIT(I2CHandle->pI2Cx->I2C_CR1, ACK);
			Msg->IsACKEnabled = FALSE;
		}

		//store the message pointer in the global pointer
		I2C_Msg[Channel] = Msg;

		//enable event interrupt
		SET_BIT(I2CHandle->pI2Cx->I2C_CR2, ITBUFEN);
		SET_BIT(I2CHandle->pI2Cx->I2C_CR2, ITEVTEN);
		SET_BIT(I2CHandle->pI2Cx->I2C_CR2, ITERREN);

		Success = TRUE;
	}
	return Success;
}


/****************************************************************************************************
 * @fn      		  - I2C_IRQConfig
 *
 * @brief             - This function configures I2C interrupt
 *
 * @param[in]         - handle to interrupt config
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -

 */
void I2C_IRQConfig(InterruptHandle *pIntHandle) {
	if (pIntHandle->IRQNum == I2C1_EV || pIntHandle->IRQNum == I2C1_ER
			|| pIntHandle->IRQNum == I2C2_EV || pIntHandle->IRQNum == I2C2_ER
			|| pIntHandle->IRQNum == I2C3_EV || pIntHandle->IRQNum == I2C3_ER) {

		//enable the interrupt line in NVIC
		CortexM4EnableIRQ(pIntHandle->IRQNum);

		//configure the priority of interrupt
		CortexM4SetIntPriority(pIntHandle->IRQNum, pIntHandle->IntPri);

		pIntHandle->IsEnable = TRUE;
	} else {
		pIntHandle->IsEnable = I2C_INTERRUPT_CONFIG_ERROR;
	}

	return;
}

/****************************************************************************************************
 * @fn      		  - I2C1_EV_IRQHandler
 *
 * @brief             - ISR of I2C1 event interrupt
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -

 */

void I2C1_EV_IRQHandler() {
	I2C_IRQ_EV_Handler(I2C1,I2C_1);
}

/****************************************************************************************************
 * @fn      		  - I2C1_EV_IRQHandler
 *
 * @brief             - ISR of I2C1 error interrupt
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -

 */

void I2C1_ER_IRQHandler() {
	I2C_IRQ_ER_Handler(I2C1, I2C_1);
}

/****************************************************************************************************
 * @fn      		  - I2C2_EV_IRQHandler
 *
 * @brief             - ISR of I2C2 event interrupt
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -

 */

void I2C2_EV_IRQHandler() {
	I2C_IRQ_EV_Handler(I2C2,I2C_2);
}

/****************************************************************************************************
 * @fn      		  - I2C2_EV_IRQHandler
 *
 * @brief             - ISR of I2C2 error interrupt
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -

 */

void I2C2_ER_IRQHandler() {
	I2C_IRQ_ER_Handler(I2C2,I2C_2);
}

/****************************************************************************************************
 * @fn      		  - I2C3_EV_IRQHandler
 *
 * @brief             - ISR of I2C3 event interrupt
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -

 */

void I2C3_EV_IRQHandler() {
	I2C_IRQ_EV_Handler(I2C3,I2C_3);
}

/****************************************************************************************************
 * @fn      		  - I2C3_ER_IRQHandler
 *
 * @brief             - ISR of I2C3 error interrupt
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -

 */

void I2C3_ER_IRQHandler() {
	I2C_IRQ_ER_Handler(I2C3,I2C_3);
}

//static function

/****************************************************************************************************
 * @fn      		  - I2C_ITMasterStopTx
 *
 * @brief             - This function is called when master wants to stop the data transmission
 *
 * @param[in]         - Handle to I2C config
 * @param[in]         - pointer to message packet
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  frequency cannot exceed 50Mhz

 */

void I2C_ITMasterStopTx(I2C_reg_t *pI2Cx, uint8_t ChannelNo) {

	if ((I2C_Msg[ChannelNo]->msgStatus == TX_ERROR||I2C_Msg[ChannelNo]->msgStatus == TX_COMPLETE)) {
		I2C_Msg[ChannelNo] = 0;

		//disable event interrupt
		CLEAR_BIT(pI2Cx->I2C_CR2, ITBUFEN);
		CLEAR_BIT(pI2Cx->I2C_CR2, ITEVTEN);
		CLEAR_BIT(pI2Cx->I2C_CR2, ITERREN);
	}

}

/****************************************************************************************************
 * @fn      		  - I2C_ITMasterStopRx
 *
 * @brief             - This function is called when master wants to stop the data reception
 *
 * @param[in]         - Handle to I2C config
 * @param[in]         - pointer to message packet
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  frequency cannot exceed 50Mhz

 */

void I2C_ITMasterStopRx(I2C_reg_t *pI2Cx ,uint8_t ChannelNo) {

	if ((I2C_Msg[ChannelNo]->msgStatus==READ_ERROR||I2C_Msg[ChannelNo]->msgStatus == READ_COMPLETE)) {

		//disable event interrupt
		CLEAR_BIT(pI2Cx->I2C_CR2, ITBUFEN);
		CLEAR_BIT(pI2Cx->I2C_CR2, ITEVTEN);
		CLEAR_BIT(pI2Cx->I2C_CR2, ITERREN);

		//enable the ACK bit if it was enabled
		if(I2C_Msg[ChannelNo]->IsACKEnabled)
		SET_BIT(pI2Cx->I2C_CR1, ACK);

		I2C_Msg[ChannelNo] = 0;
	}

}

/****************************************************************************************************
 * @fn      		  - I2C_ITSlaveStopTx
 *
 * @brief             - This function is called when slave detects stop transmission condition
 *
 * @param[in]         - Handle to I2C config
 * @param[in]         - pointer to message packet
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  frequency cannot exceed 50Mhz

 */

void I2C_ITSlaveStopTx(I2C_reg_t *pI2Cx ,uint8_t ChannelNo) {


	if (I2C_Msg[ChannelNo]->msgStatus == TX_COMPLETE) {

		I2C_Msg[ChannelNo] = 0;

		//disable event interrupt
		CLEAR_BIT(pI2Cx->I2C_CR2, ITBUFEN);
		CLEAR_BIT(pI2Cx->I2C_CR2, ITEVTEN);
		CLEAR_BIT(pI2Cx->I2C_CR2, ITERREN);
	}

}

/****************************************************************************************************
 * @fn      		  - I2C_ITSlaveStopRx
 *
 * @brief             - This function is called when slave detects stop reception condition
 *
 * @param[in]         - Handle to I2C config
 * @param[in]         - pointer to message packet
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  frequency cannot exceed 50Mhz

 */

void I2C_ITSlaveStopRx(I2C_reg_t *pI2Cx ,uint8_t ChannelNo) {

	if (I2C_Msg[ChannelNo]->msgStatus == READ_COMPLETE) {

		I2C_Msg[ChannelNo] = 0;

		//disable event interrupt
		CLEAR_BIT(pI2Cx->I2C_CR2, ITBUFEN);
		CLEAR_BIT(pI2Cx->I2C_CR2, ITEVTEN);
		CLEAR_BIT(pI2Cx->I2C_CR2, ITERREN);
	}

}

/****************************************************************************************************
 * @fn      		  - I2C_IRQHandling
 *
 * @brief             - This function does some interrupt handling.
 *
 * @param[in]         - base address of the I2C peripheral
 * @param[in]         - I2C channel number
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -

 */

void I2C_IRQ_EV_Handler(I2C_reg_t *pI2Cx, uint8_t ChannelNo) {

	uint8_t Channel = ChannelNo-1;

		if(I2C_Msg[Channel]==0){
#ifdef APP_DEBUG
			sprintf(dbgBuf,I2C_INTERRUPT_HANDLE_ERROR,strlen(I2C_INTERRUPT_HANDLE_ERROR));
			dbgBuf += strlen(I2C_INTERRUPT_HANDLE_ERROR)+1;
#endif
			return;
		}
	//if ITEVTEN is set it means the interrupt could be due to SB, ADDR, ADD10, STOPF or BTF
	if (READ_BIT_VALUE(pI2Cx->I2C_CR2, ITEVTEN)) {

		//if SB is set
		if (READ_BIT_VALUE(pI2Cx->I2C_SR1, SB)) {
			//clear the bit by reading SR1 and putting the address of the slave in DR
			if (I2C_Msg[Channel]->msgStatus == TX_NOT_STARTED)
				//if master wants to write
				pI2Cx->I2C_DR = ((I2C_Msg[Channel]->slaveAddr
						<< DR_FIRST_BIT) & WRITE_ENABLE_BITMASK);
			else if (I2C_Msg[Channel]->msgStatus == READ_NOT_STARTED)
				//if master wants to read
				pI2Cx->I2C_DR = ((I2C_Msg[Channel]->slaveAddr
						<< DR_FIRST_BIT) | READ_ENABLE_BITMASK);
		}
		//if ADDR is set
		//for master it means Slave with corresponding address is present
		//for slave it means address of slave is present on the bus
		//clear ADDR by reading SR1 followed by reading SR2
		if (READ_BIT_VALUE(pI2Cx->I2C_SR1, ADDR)) {
			//for two byte reception in master mode
			if (I2C_Msg[Channel]->msgLen == 2&&I2C_Msg[Channel]->msgStatus == READ_NOT_STARTED
					&&I2C_Msg[Channel]->IsMaster==TRUE){
				//disable the ACK before ADDR is cleared
				CLEAR_BIT(pI2Cx->I2C_CR1, ACK);
				SET_BIT(pI2Cx->I2C_CR1, POS);
			}
			//by reading SR2 addr will be cleared
			//set the message status
				if (READ_BIT_VALUE(pI2Cx->I2C_SR2, TRA)) {
					//if TRA is set,device is transmitting
					I2C_Msg[Channel]->msgStatus = TX_IN_PROGRESS;
				} else {
					//if TRA is clear, device is receiving
					I2C_Msg[Channel]->msgStatus = READ_IN_PROGRESS;
				}
			}
		}
			//BTF is set if clock stretching is enabled
		if (READ_BIT_VALUE(pI2Cx->I2C_SR1, BTF)) {
			//if master is transmitting
			if(I2C_Msg[Channel]->IsMaster == TRUE){
				if(I2C_Msg[Channel]->msgStatus==TX_IN_PROGRESS){
					if(I2C_Msg[Channel]->msgLen == 0&&READ_BIT_VALUE(pI2Cx->I2C_SR1, TxE)){
					if(I2C_Msg[Channel]->I2C_RepeatedStart== DISABLE){
					//msgLen = 0 in master mode indicate that transmission is finished
					//generate a stop condition when the last byte is transmitted by master
					SET_BIT(pI2Cx->I2C_CR1, STOP);
				}
					I2C_Msg[Channel]->msgStatus = TX_COMPLETE;
					I2C_ITMasterStopTx(pI2Cx,Channel);
				}
			}
				else if (I2C_Msg[Channel]->msgStatus == READ_IN_PROGRESS){
					//read both the bytes
					//set the message status to complete
					//release the global message structure
					if(I2C_Msg[Channel]->msgLen==2){
						//disable ACK
						CLEAR_BIT(pI2Cx->I2C_CR1, ACK);
						//generate STOP if repeated start is disabled
						if (I2C_Msg[Channel]->I2C_RepeatedStart
								== DISABLE)
							SET_BIT(pI2Cx->I2C_CR1, STOP);
						//read 1st byte
						I2C_Msg[Channel]->msgBuffer[I2C_Msg[Channel]->msgSize
							- I2C_Msg[Channel]->msgLen--] = pI2Cx->I2C_DR;
						//read 2nd byte
						I2C_Msg[Channel]->msgBuffer[I2C_Msg[Channel]->msgSize
							- I2C_Msg[Channel]->msgLen--] = pI2Cx->I2C_DR;
						//set the message status to READ COMPLETE
						I2C_Msg[Channel]->msgStatus = READ_COMPLETE;
						//and release the channel
						I2C_ITMasterStopRx(pI2Cx,Channel);
				}
			}
		}
	}
		if (READ_BIT_VALUE(pI2Cx->I2C_SR1, STOPF)) {
			//applicable for slave
			if (!READ_BIT_VALUE(pI2Cx->I2C_SR2, MSL)) {
				//clear the STOPF bit
				//by reading SR1 followed by writing to CR1
				pI2Cx->I2C_CR1 |= 0x0;
				//stop reading
				I2C_Msg[Channel]->msgStatus = READ_COMPLETE;
				I2C_ITSlaveStopRx(pI2Cx,Channel);
			}
		}
		//if ITBUFEN is set it means the interrupt could occur due to RxNE or TxE events
		if (READ_BIT_VALUE(pI2Cx->I2C_CR2, ITBUFEN)) {
			if (READ_BIT_VALUE(pI2Cx->I2C_SR1, TxE)) {
				//slave is transmitting
				if (!READ_BIT_VALUE(pI2Cx->I2C_SR2, MSL)) {
					if (I2C_Msg[Channel]->msgStatus
							== TX_IN_PROGRESS){
						pI2Cx->I2C_DR =
								I2C_Msg[Channel]->msgBuffer[I2C_Msg[Channel]->msgLen++];
								//printf("I2C_Msg[Channel]->msgLen %ld\n\r",I2C_Msg[Channel]->msgLen);
					}
				}
				else {
					//master is transmitting
					if (I2C_Msg[Channel]->msgLen > 0)
						pI2Cx->I2C_DR =
								I2C_Msg[Channel]->msgBuffer[I2C_Msg[Channel]->msgSize
										- I2C_Msg[Channel]->msgLen--];
				}
			}
			else if (READ_BIT_VALUE(pI2Cx->I2C_SR1, RxNE)) {
					//slave is receiving
					if (!READ_BIT_VALUE(pI2Cx->I2C_SR2, MSL)) {
						if (I2C_Msg[Channel]->msgStatus
								== READ_IN_PROGRESS) {
							I2C_Msg[Channel]->msgBuffer[I2C_Msg[Channel]->msgLen++] =
									pI2Cx->I2C_DR;
						}
					} else {
						//master will receive
						if(I2C_Msg[Channel]->msgStatus==READ_IN_PROGRESS){
						if (I2C_Msg[Channel]->msgLen == 1) {
							//disable ACK
							CLEAR_BIT(pI2Cx->I2C_CR1, ACK);
							//if repeated start is disabled, generate the stop condition
							if (I2C_Msg[Channel]->I2C_RepeatedStart
									== DISABLE)
								SET_BIT(pI2Cx->I2C_CR1, STOP);
							I2C_Msg[Channel]->msgBuffer[I2C_Msg[Channel]->msgSize
									- I2C_Msg[Channel]->msgLen--] =
									pI2Cx->I2C_DR;
							I2C_Msg[Channel]->msgStatus =
							READ_COMPLETE;
							I2C_ITMasterStopRx(pI2Cx,Channel);
						} else if (I2C_Msg[Channel]->msgLen > 1) {
							I2C_Msg[Channel]->msgBuffer[I2C_Msg[Channel]->msgSize
									- I2C_Msg[Channel]->msgLen--] =
									pI2Cx->I2C_DR;
						}
					}
				}
			}
		}
		return;
}

/****************************************************************************************************
 * @fn      		  - I2C_IRQ_ER_Handling
 *
 * @brief             - This function I2C handles error interrupts
 *
 * @param[in]         - base address of the I2C peripheral
 * @param[in]         - I2C channel number
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -

 */

void I2C_IRQ_ER_Handler(I2C_reg_t *pI2Cx, uint8_t ChannelNo){

	uint8_t Channel = ChannelNo-1;

		//handling ACK failure
	if (READ_BIT_VALUE(pI2Cx->I2C_CR2, ITERREN)) {
		if (READ_BIT_VALUE(pI2Cx->I2C_SR1, AF)) {
			//applicable for slave
			if (!READ_BIT_VALUE(pI2Cx->I2C_SR2, MSL)) {
				I2C_Msg[Channel]->msgStatus = TX_COMPLETE;
				I2C_ITSlaveStopTx(pI2Cx,Channel);
			}
			else{
				//set the error type in the message handle
				I2C_Msg[Channel]->errorType = I2C_AF_ERROR;
				//generate the stop
				SET_BIT(pI2Cx->I2C_CR1, STOP);
				//cease the communication and change the application status
				if(I2C_Msg[Channel]->msgStatus==READ_IN_PROGRESS
						||I2C_Msg[Channel]->msgStatus==READ_NOT_STARTED){
					I2C_Msg[Channel]->msgStatus = READ_ERROR;
					I2C_Msg[Channel]->I2C_RepeatedStart = DISABLE;
					I2C_ITMasterStopRx(pI2Cx, Channel);
				}
				else if(I2C_Msg[Channel]->msgStatus==TX_IN_PROGRESS
						||I2C_Msg[Channel]->msgStatus==TX_NOT_STARTED){
					I2C_Msg[Channel]->msgStatus = TX_ERROR;
					I2C_Msg[Channel]->I2C_RepeatedStart = DISABLE;
					I2C_ITMasterStopTx(pI2Cx, Channel);
				}
			}
			//clear AF bit
			CLEAR_BIT(pI2Cx->I2C_SR1, AF);
		}
		//handling bus error
		if(READ_BIT_VALUE(pI2Cx->I2C_SR1, BERR)){
			//set the error status
			I2C_Msg[Channel]->errorType = I2C_BERR_ERROR;
			//master should start the communication again
			//as slave has released the line and it is waiting
			//for either a start or a stop condition
			//generate the stop
			SET_BIT(pI2Cx->I2C_CR1, STOP);
			if(I2C_Msg[Channel]->IsMaster==TRUE){
				if(I2C_Msg[Channel]->msgStatus==READ_IN_PROGRESS
						||I2C_Msg[Channel]->msgStatus==READ_NOT_STARTED){
					I2C_Msg[Channel]->msgStatus = READ_ERROR;
					I2C_Msg[Channel]->I2C_RepeatedStart = DISABLE;
					I2C_ITMasterStopRx(pI2Cx, Channel);
				}
				else if(I2C_Msg[Channel]->msgStatus==TX_IN_PROGRESS
						||I2C_Msg[Channel]->msgStatus==TX_NOT_STARTED){
					I2C_Msg[Channel]->msgStatus = TX_ERROR;
					I2C_Msg[Channel]->I2C_RepeatedStart = DISABLE;
					I2C_ITMasterStopTx(pI2Cx, Channel);
				}
			}
			else{//for slave
				if(I2C_Msg[Channel]->msgStatus==READ_IN_PROGRESS
						||I2C_Msg[Channel]->msgStatus==READ_NOT_STARTED){
					I2C_Msg[Channel]->msgStatus = READ_ERROR;
					I2C_ITSlaveStopRx(pI2Cx,Channel);
				}
				else if(I2C_Msg[Channel]->msgStatus==TX_IN_PROGRESS
						||I2C_Msg[Channel]->msgStatus==TX_NOT_STARTED){
					I2C_Msg[Channel]->msgStatus = TX_ERROR;
					I2C_ITSlaveStopTx(pI2Cx,Channel);
				}
			}
			//clear BERR bit
			CLEAR_BIT(pI2Cx->I2C_SR1, BERR);
		}
		if(READ_BIT_VALUE(pI2Cx->I2C_SR1, ARLO)){
			//set the error status
			I2C_Msg[Channel]->errorType = I2C_ARLO_ERROR;
			//clear ARLO bit
			CLEAR_BIT(pI2Cx->I2C_SR1, ARLO);
		}
		if(READ_BIT_VALUE(pI2Cx->I2C_SR1, OVR)){
			//set the error status
			I2C_Msg[Channel]->errorType = I2C_OVR_ERROR;
			if(READ_BIT_VALUE(pI2Cx->I2C_SR1, RxNE)){
			//clear RxNE flag
			CLEAR_BIT(pI2Cx->I2C_SR1, RxNE);
			}
			if(READ_BIT_VALUE(pI2Cx->I2C_SR1, TxE)){
			//clear TxE flag
			CLEAR_BIT(pI2Cx->I2C_SR1, TxE);
			}
		}
	}

	return ;
}

/****************************************************************************************************
 * @fn      		  - I2C_PeriClkCtrl
 *
 * @brief             - This function enables or disables peripheral clock for the given I2C bus
 *
 * @param[in]         - base address of the I2C peripheral
 * @param[in]         - ENABLE or DISABLE
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void I2C_PeriClkCtrl(I2C_reg_t *pI2Cx, uint8_t IsEnabled) {
	assert(pI2Cx != 0);

	if (IsEnabled) {
		if (pI2Cx == I2C1) {
			EN_PCLK_I2C1();
		} else if (pI2Cx == I2C2) {
			EN_PCLK_I2C2();
		} else {
			EN_PCLK_I2C3();
		}
	} else {
		if (pI2Cx == I2C1) {
			DIS_PCLK_I2C1();
		} else if (pI2Cx == I2C2) {
			DIS_PCLK_I2C2();
		} else {
			DIS_PCLK_I2C3();
		}
	}
	return;
}

/****************************************************************************************************
 * @fn      		  - I2C_FindTheAPB1Freq
 *
 * @brief             - This function find the APB1 clock frequency
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  6 bit clock frequency
 *
 * @Note              -  none

 */
uint8_t I2C_FindTheAPB1Freq() {
	uint8_t freq = 0;
	uint16_t div = 0;

	//find the clock source
	switch ((((RCC )->RCC_CFGR) & CLOCK_SOURCE_BITMASK) >> SWS) {
	case CS_HSI:
		freq = 16;
		break;
	case CS_HSE:
		freq = 8;
		break;
	case CS_PLL:
		freq = 50;
		break;
	default:
		freq = 4;
		break;
	}

	//find the AHB divisor
	switch ((((RCC )->RCC_CFGR) & CLOCK_AHB_BITMASK) >> HPRE) {
	case 8:
		div = 2;
		break;
	case 9:
		div = 4;
		break;
	case 10:
		div = 8;
		break;
	case 11:
		div = 16;
		break;
	case 12:
		div = 64;
		break;
	case 13:
		div = 128;
		break;
	case 14:
		div = 256;
		break;
	case 15:
		div = 512;
		break;
	default:
		div = 1;
		break;

	}

	//divide the original frequency with AHB pre-scaler
	freq /= div;

	//find APB low pre-scaler
	switch (((RCC )->RCC_CFGR & CLOCK_APB1_BITMASK) >> PPRE1) {
	case 4:
		div = 2;
		break;
	case 5:
		div = 4;
		break;
	case 6:
		div = 8;
		break;
	case 7:
		div = 16;
		break;
	default:
		div = 1;
	}

	//divide the frequency with APB low pre-scaler
	freq /= div;

	return freq;
}
