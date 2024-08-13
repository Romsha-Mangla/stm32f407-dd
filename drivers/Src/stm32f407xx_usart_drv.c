/*
 * stm32f407xx_usart_drv.c
 *
 *  Created on: 31-May-2024
 *      Author: romsha
 */

#include <stm32f407xx_usart_drv.h>

static USART_Message_t *USART_TxMsg[USART_6];
static USART_Message_t *USART_RxMsg[USART_6];

static void USART_PeriClkCtrl(USART_reg_t *pUSARTx, uint8_t IsEnabled);
static void USART_IRQHandler(USART_reg_t *pUSARTx, uint8_t USARTChannel);
static uint8_t USART_FindTheAPB2Freq();

/****************************************************************************************************
 * @fn      		  - USART_Init
 *
 * @brief             - This function configures the USART channel for use as per user preference
 *
 * @param[in]         - Handle to USART config
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  the function will either enable transmission or reception or both

 */

void USART_Init(USART_Handle_t *pUSARTHandle) {

	assert(pUSARTHandle != 0);
	float USART_div = 0;
	uint32_t mantissa = 0, fraction = 0;

	//enable the peripheral clock
	USART_PeriClkCtrl(pUSARTHandle->pUSARTx, TRUE);

	//enable UE
	SET_BIT(pUSARTHandle->pUSARTx->USART_CR1, UE);

	//set the word length
	//if 1 it is 9 bits otherwise 8 bits
	if (pUSARTHandle->USARTConfig.USART_WordLength)
		SET_BIT(pUSARTHandle->pUSARTx->USART_CR1, M);

	//set the parity bit
	//if zero, set even parity
	if (pUSARTHandle->USARTConfig.USART_Parity == USART_EVEN_PARITY) {
		SET_BIT(pUSARTHandle->pUSARTx->USART_CR1, PCE);
		CLEAR_BIT(pUSARTHandle->pUSARTx->USART_CR1, PS);
	}
	//odd parity
	else if (pUSARTHandle->USARTConfig.USART_Parity == USART_ODD_PARITY) {
		SET_BIT(pUSARTHandle->pUSARTx->USART_CR1, PCE);
		SET_BIT(pUSARTHandle->pUSARTx->USART_CR1, PS);
	}

	//over sampling
	//OVER8=1, 8 otherwise 16
	if (pUSARTHandle->USARTConfig.USART_OverSampling)
		SET_BIT(pUSARTHandle->pUSARTx->USART_CR1, OVER8);

	//set STOP bit
	pUSARTHandle->pUSARTx->USART_CR2 = pUSARTHandle->USARTConfig.USART_StopBit
			<< STOP_1_0;

	//set HW flow control if enabled
	//not available for uart 4 and 5
	if (pUSARTHandle->pUSARTx != UART4 || pUSARTHandle->pUSARTx != UART5) {
		if (pUSARTHandle->USARTConfig.USART_IsHWFctrlEnbl
				== USART_HW_FLOW_CTRL_CTS) {
			SET_BIT(pUSARTHandle->pUSARTx->USART_CR3, CTSE);
		} else if (pUSARTHandle->USARTConfig.USART_IsHWFctrlEnbl
				== USART_HW_FLOW_CTRL_RTS) {
			SET_BIT(pUSARTHandle->pUSARTx->USART_CR3, RTSE);
		} else if (pUSARTHandle->USARTConfig.USART_IsHWFctrlEnbl
				== USART_HW_FLOW_CTRL_CTS_RTS) {
			SET_BIT(pUSARTHandle->pUSARTx->USART_CR3, CTSE);
			SET_BIT(pUSARTHandle->pUSARTx->USART_CR3, RTSE);
		}
	}

//	Tx/Rx baud =
//	fCK/ (8 × (2 – OVER8) × USARTDIV)
	USART_div = (float) (USART_FindTheAPB2Freq() * 1000000)
			/ (float) (8 * (2 - pUSARTHandle->USARTConfig.USART_OverSampling)
					* pUSARTHandle->USARTConfig.USART_BaudRate);
	mantissa = (uint32_t) USART_div;
	//50 is added to round off the number
	fraction = (uint32_t) (((USART_div - mantissa * 1.0)
			* (8 * (2 - pUSARTHandle->USARTConfig.USART_OverSampling))) + 0.5);
	//set baud rate in BRR
	pUSARTHandle->pUSARTx->USART_BRR = (mantissa << DIV_MAN_4_11)
			| (fraction << DIV_FRA_3_0);

	//enable TE or RE or both
	if (pUSARTHandle->USARTConfig.USART_Mode == USART_ONLY_TX) {
		SET_BIT(pUSARTHandle->pUSARTx->USART_CR1, TE);
	} else if (pUSARTHandle->USARTConfig.USART_Mode == USART_ONLY_RX) {
		SET_BIT(pUSARTHandle->pUSARTx->USART_CR1, RE);
	} else if (pUSARTHandle->USARTConfig.USART_Mode == USART_TX_RX) {
		SET_BIT(pUSARTHandle->pUSARTx->USART_CR1, TE);
		SET_BIT(pUSARTHandle->pUSARTx->USART_CR1, RE);
	}

	return;
}

/****************************************************************************************************
 * @fn      		  - USART_Enable
 *
 * @brief             - This function enables USART
 *
 * @param[in]         - base address of the USART peripheral
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void USART_Enable(USART_reg_t *pUSARTx) {
	assert(pUSARTx != 0);

	SET_BIT(pUSARTx->USART_CR1, UE);
}

/****************************************************************************************************
 * @fn      		  - USART_Disable
 *
 * @brief             - This function disables USART
 *
 * @param[in]         - base address of the USART peripheral
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void USART_Disable(USART_reg_t *pUSARTx) {
	assert(pUSARTx != 0);

	//wait till transaction is complete
	while(!READ_BIT_VALUE(pUSARTx->USART_SR,TC));
	//clear TC bit
	CLEAR_BIT(pUSARTx->USART_SR, TC);
	//disable USART
	CLEAR_BIT(pUSARTx->USART_CR1, UE);
}

/****************************************************************************************************
 * @fn      		  - USART_EnableTX
 *
 * @brief             - This function enables given USART transmission block
 *
 * @param[in]         - base address of the USART peripheral
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void USART_EnableTX(USART_reg_t *pUSARTx) {
	assert(pUSARTx != 0);

	SET_BIT(pUSARTx->USART_CR1, TE);
}

/****************************************************************************************************
 * @fn      		  - USART_DisableTX
 *
 * @brief             - This function disables USART transmission block
 *
 * @param[in]         - base address of the USART peripheral
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void USART_DisableTX(USART_reg_t *pUSARTx) {
	assert(pUSARTx != 0);

	CLEAR_BIT(pUSARTx->USART_CR1, TE);
}

/****************************************************************************************************
 * @fn      		  - USART_EnableRX
 *
 * @brief             - This function enables given USART reception block
 *
 * @param[in]         - base address of the USART peripheral
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void USART_EnableRX(USART_reg_t *pUSARTx) {
	assert(pUSARTx != 0);

	SET_BIT(pUSARTx->USART_CR1, RE);
}

/****************************************************************************************************
 * @fn      		  - USART_DisableRX
 *
 * @brief             - This function disables USART reception block
 *
 * @param[in]         - base address of the USART peripheral
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void USART_DisableRX(USART_reg_t *pUSARTx) {
	assert(pUSARTx != 0);

	CLEAR_BIT(pUSARTx->USART_CR1, RE);
}

/****************************************************************************************************
 * @fn      		  - USART_reset
 *
 * @brief             - This function resets the given USART bus
 *
 * @param[in]         - base address of the USART peripheral
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void USART_reset(USART_reg_t *pUSARTx) {
	assert(pUSARTx != 0);

	if (pUSARTx == USART1) {
		RESET_USART1();
		SET_USART1();
	} else if (pUSARTx == USART2) {
		RESET_USART2();
		SET_USART2();
	} else if (pUSARTx == USART3) {
		RESET_USART3();
		SET_USART3();
	} else if (pUSARTx == UART4) {
		RESET_UART4();
		SET_UART4();
	} else if (pUSARTx == UART5) {
		RESET_UART5();
		SET_UART5();
	} else {
		RESET_USART6();
		SET_USART6();
	}

	return;
}

/****************************************************************************************************
 * @fn      		  - USART_TxData
 *
 * @brief             - This function sends data to another USART device
 *
 * @param[in]         - pointer to USART handle
 * @param[in]         - pointer to message message
 * @param[in]         -
 *
 * @return            -  returns TRUE on success else FALSE
 *
 * @Note              -  none

 */
uint8_t USART_TxData(USART_Handle_t *pUSARTHandle, USART_Message_t *Msg) {

	assert(pUSARTHandle != 0);
	assert(Msg);
	assert(Msg->msgBuffer!=0);
	uint8_t success = FALSE;


	uint16_t *pdata;

   //Loop over until "Len" number of bytes are transferred
	for(Msg->msgLen = 0 ; Msg->msgLen < Msg->msgSize;)
	{
		Msg->msgStatus = TX_IN_PROGRESS;

		//Implement the code to wait until TXE flag is set in the SR
		while(!READ_BIT_VALUE(pUSARTHandle->pUSARTx->USART_SR,TXE));

		//Check the USART_WordLength item for 9BIT or 8BIT in a frame
		if(pUSARTHandle->USARTConfig.USART_WordLength == USART_9_BIT)
		{
			//if 9BIT load the DR with 2bytes masking  the bits other than first 9 bits
			pdata = (uint16_t*) (&Msg->msgBuffer[Msg->msgLen]);
			pUSARTHandle->pUSARTx->USART_DR = (*pdata & (uint16_t)0x01FF);

			//check for USART_ParityControl
			if(pUSARTHandle->USARTConfig.USART_Parity == USART_NO_PARITY)
			{
				//No parity is used in this transfer , so 9bits of user data will be sent
				//Implement the code to increment pTxBuffer twice
				Msg->msgLen += 2;
			}
			else
			{
				//Parity bit is used in this transfer . so 8bits of user data will be sent
				//The 9th bit will be replaced by parity bit by the hardware
				Msg->msgLen++;
			}

		}
		else
		{
			//This is 8bit data transfer
			printf("%c\n",Msg->msgBuffer[Msg->msgLen]);
			pUSARTHandle->pUSARTx->USART_DR = Msg->msgBuffer[Msg->msgLen++];

		}

		success = TRUE;
	}

	//Implement the code to wait till TC flag is set in the SR
	while(!READ_BIT_VALUE(pUSARTHandle->pUSARTx->USART_SR,TC));
	Msg->msgStatus = TX_COMPLETE;
	return success;
}

/****************************************************************************************************
 * @fn      		  - USART_RxData
 *
 * @brief             - This function receives data from another USART device
 *
 * @param[in]         - pointer to USART handle
 * @param[in]         - pointer to message message
 * @param[in]         -
 *
 * @return            -  returns TRUE on success else FALSE
 *
 * @Note              -  none

 */
uint8_t USART_RxData(USART_Handle_t *pUSARTHandle, USART_Message_t *Msg) {

	assert(pUSARTHandle != 0);
	assert(Msg);
	uint8_t success = FALSE;
	if (Msg->msgSize > 0 && Msg->msgBuffer != 0) {
		Msg->msgStatus = READ_IN_PROGRESS;
		//set message length to zero
		//as message length indicate number of bytes received so far
		Msg->msgLen = 0;

		if (pUSARTHandle->USARTConfig.USART_WordLength == USART_8_BIT) {
			while (Msg->msgLen < Msg->msgSize) {
				while (!READ_BIT_VALUE(pUSARTHandle->pUSARTx->USART_SR, RXNE))
					;
				if (pUSARTHandle->USARTConfig.USART_Parity > USART_NO_PARITY)
					//the 8th bit is parity
					//receive all 8 bits
					Msg->msgBuffer[Msg->msgLen++] = pUSARTHandle->pUSARTx->USART_DR & 0x7f;
				else
					//it means no parity
					//read all 8 bits
					pUSARTHandle->pUSARTx->USART_DR =
							Msg->msgBuffer[Msg->msgLen++];
			}

			Msg->msgStatus = READ_COMPLETE;
			success = TRUE;

		} else {
			while (Msg->msgLen < Msg->msgSize) {
				while (!READ_BIT_VALUE(pUSARTHandle->pUSARTx->USART_SR, RXNE))
					;
				if (pUSARTHandle->USARTConfig.USART_Parity > USART_NO_PARITY) {
					//9th bit will be parity bit
					pUSARTHandle->pUSARTx->USART_DR =
							Msg->msgBuffer[Msg->msgLen++];

				} else{
					//no parity
					//all 9 bits from DR into user buffer
					pUSARTHandle->pUSARTx->USART_DR =
							*((uint16_t*) &Msg->msgBuffer[Msg->msgLen]) & 0x1ff;
					Msg->msgLen += 2;
				}
			}

			Msg->msgStatus = READ_COMPLETE;
			success = TRUE;
		}
	}

	return success;

}

/****************************************************************************************************
 * @fn      		  - USART_ITTxData
 *
 * @brief             - This function sends data to another USART device using interrupt
 *
 * @param[in]         - pointer to USART handle
 * @param[in]         - pointer to message message
 * @param[in]         -
 *
 * @return            - returns TRUE on success else FALSE
 *
 * @Note              - none

 */
uint8_t USART_ITTxData(USART_Handle_t *pUSARTHandle, USART_Message_t *Msg) {

	assert(pUSARTHandle != 0);
	assert(Msg!=0);
	assert(Msg->msgBuffer!=0);

	uint8_t success = FALSE;
	uint8_t ChannelNo = Msg->USARTChannel-1;

	if(USART_TxMsg[ChannelNo]==0){

		USART_TxMsg[ChannelNo] = Msg;
		Msg->msgLen = 0;
		Msg->msgStatus = TX_IN_PROGRESS;

		success = TRUE;

		//enable the TX and TC interrupt bits
		SET_BIT(pUSARTHandle->pUSARTx->USART_CR1,TXEIE);
		SET_BIT(pUSARTHandle->pUSARTx->USART_CR1,TCIE);
	}

	return success;

}

/****************************************************************************************************
 * @fn      		  - USART_ITRxData
 *
 * @brief             - This function receives data from another USART device using interrupt
 *
 * @param[in]         - pointer to USART handle
 * @param[in]         - pointer to message message
 * @param[in]         -
 *
 * @return            -  returns TRUE on success else FALSE
 *
 * @Note              -  none

 */
uint8_t USART_ITRxData(USART_Handle_t *pUSARTHandle, USART_Message_t *Msg) {

	assert(pUSARTHandle != 0);
	assert(Msg);
	assert(Msg->msgBuffer!=0);
	uint8_t success = FALSE;
	uint8_t ChannelNo = Msg->USARTChannel-1;

	if(USART_RxMsg[ChannelNo]==0){

		USART_RxMsg[ChannelNo] = Msg;
		Msg->msgLen = 0;
		Msg->msgStatus = READ_IN_PROGRESS;

		//enable the RX and idle interrupt bits
		SET_BIT(pUSARTHandle->pUSARTx->USART_CR1,RXNEIE);

		success = TRUE;
	}

	return success;
}

/****************************************************************************************************
 * @fn      		  - USART1_IRQHandler
 *
 * @brief             - ISR of USART 1
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -

 */
void USART1_IRQHandler() {

	USART_IRQHandler(USART1, USART_1-1);
}

/****************************************************************************************************
 * @fn      		  - USART2_IRQHandler
 *
 * @brief             - ISR of USART 2
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -

 */
void USART2_IRQHandler() {

	USART_IRQHandler(USART2, USART_2-1);
}

/****************************************************************************************************
 * @fn      		  - USART3_IRQHandler
 *
 * @brief             - ISR of USART 3
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -

 */
void USART3_IRQHandler() {

	USART_IRQHandler(USART3, USART_3-1);
}

/****************************************************************************************************
 * @fn      		  - USART4_IRQHandler
 *
 * @brief             - ISR of UART 4
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -

 */
void UART4_IRQHandler() {

	USART_IRQHandler(UART4, UART_4-1);
}

/****************************************************************************************************
 * @fn      		  - USART5_IRQHandler
 *
 * @brief             - ISR of USART 5
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -

 */
void UART5_IRQHandler() {

	USART_IRQHandler(UART5, UART_5-1);
}

/****************************************************************************************************
 * @fn      		  - USART6_IRQHandler
 *
 * @brief             - ISR of USART 6
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -

 */
void USART6_IRQHandler() {

	USART_IRQHandler(USART6, USART_6-1);
}

/****************************************************************************************************
 * @fn      		  - USART_IRQConfig
 *
 * @brief             - This configures interrupt on behalf of user
 *
 * @param[in]         - pointer to Interrupt handle
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void USART_IRQConfig(InterruptHandle *pIntHandle) {

	if (pIntHandle->IRQNum == USART1_IRQ || pIntHandle->IRQNum == USART2_IRQ
			|| pIntHandle->IRQNum == USART3_IRQ
			|| pIntHandle->IRQNum == UART4_IRQ
			|| pIntHandle->IRQNum == UART5_IRQ
			|| pIntHandle->IRQNum == USART6_IRQ) {

		//enable the interrupt line in NVIC
		CortexM4EnableIRQ(pIntHandle->IRQNum);

		//configure the priority of interrupt
		CortexM4SetIntPriority(pIntHandle->IRQNum, pIntHandle->IntPri);

		pIntHandle->IsEnable = TRUE;
	} else {
		pIntHandle->IsEnable = USART_INTERRUPT_CONFIG_ERROR;
	}

	return;
}
/****************************************************************************************************
 * @fn      		  - USART_PeriClkCtrl
 *
 * @brief             - This function enables or disables peripheral clock for the given USART bus
 *
 * @param[in]         - base address of the USART peripheral
 * @param[in]         - ENABLE or DISABLE
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void USART_PeriClkCtrl(USART_reg_t *pUSARTx, uint8_t IsEnabled) {
	assert(pUSARTx != 0);

	if (IsEnabled) {
		if (pUSARTx == USART1) {
			EN_PCLK_USART1();
		} else if (pUSARTx == USART2) {
			EN_PCLK_USART2();
		} else if (pUSARTx == USART3) {
			EN_PCLK_USART3();
		} else if (pUSARTx == UART4) {
			EN_PCLK_UART4();
		} else if (pUSARTx == UART5) {
			EN_PCLK_UART5();
		} else {
			EN_PCLK_USART6();
		}
	} else {
		if (pUSARTx == USART1) {
			DIS_PCLK_USART1();
		} else if (pUSARTx == USART2) {
			DIS_PCLK_USART2();
		} else if (pUSARTx == USART3) {
			DIS_PCLK_USART3();
		} else if (pUSARTx == UART4) {
			DIS_PCLK_UART4();
		} else if (pUSARTx == UART5) {
			DIS_PCLK_UART5();
		} else {
			DIS_PCLK_USART6();
		}
	}

	return;
}

/****************************************************************************************************
 * @fn      		  - USART_IRQHandler
 *
 * @brief             - This function is called from ISR to do interrupt handling
 * @param[in]         - base address of the USART peripheral
 * @param[in]         - USART channel no
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void USART_IRQHandler(USART_reg_t *pUSARTx, uint8_t USARTChannel) {


	if(READ_BIT_VALUE(pUSARTx->USART_CR1,TXEIE)&&READ_BIT_VALUE(pUSARTx->USART_CR1,TXE)){
			//if parity is enabled and data length is 9 bits
			//if parity is not set and data length is 8 bits
		if(!(READ_BIT_VALUE(pUSARTx->USART_CR1,PCE)^READ_BIT_VALUE(pUSARTx->USART_CR1,M))){
			//printf("a\n");
			pUSARTx->USART_DR = USART_TxMsg[USARTChannel]->msgBuffer[USART_TxMsg[USARTChannel]->msgLen++];
		}
	}

	if(READ_BIT_VALUE(pUSARTx->USART_CR1,TCIE)&&READ_BIT_VALUE(pUSARTx->USART_CR1,TC)){
		//we are done transmitting, close the channel
		if(USART_TxMsg[USARTChannel]->msgLen>=USART_TxMsg[USARTChannel]->msgSize){
			USART_TxMsg[USARTChannel]->msgStatus = TX_COMPLETE;
			CLEAR_BIT(pUSARTx->USART_CR1,TE);
			USART_StopTxData(pUSARTx, USARTChannel);
		}
	}
	if(READ_BIT_VALUE(pUSARTx->USART_CR1,RXNEIE)&&READ_BIT_VALUE(pUSARTx->USART_CR1,RXNE)){
			//if parity is enabled and data length is 9 bits
			//so 9th bit is parity
			//no parity and data length is 8 bits
		if(!(READ_BIT_VALUE(pUSARTx->USART_CR1,PCE)^READ_BIT_VALUE(pUSARTx->USART_CR1,M))){
			USART_RxMsg[USARTChannel]->msgBuffer[USART_RxMsg[USARTChannel]->msgLen++] = pUSARTx->USART_DR;
		}

		if(USART_RxMsg[USARTChannel]->msgLen>=USART_RxMsg[USARTChannel]->msgSize){
				USART_RxMsg[USARTChannel]->msgStatus = READ_COMPLETE;
				USART_StopRxData(pUSARTx,USARTChannel);
			}
		}

 	return;
}

/****************************************************************************************************
 * @fn      		  - USART_StopTxData
 *
 * @brief             - This function stops TX interrupt
 *
 * @param[in]         - pointer to USART handle
 * @param[in]         - pointer to message message
 * @param[in]         -
 *
 * @return            - returns TRUE on success else FALSE
 *
 * @Note              - none

 */

void USART_StopTxData(USART_reg_t *pUSARTx, uint8_t USARTChannel){

	if(USART_TxMsg[USARTChannel]->msgStatus==TX_COMPLETE){
		USART_TxMsg[USARTChannel] = 0;

		//disable the interrupts
		CLEAR_BIT(pUSARTx->USART_CR1,TXEIE);
		CLEAR_BIT(pUSARTx->USART_CR1,TCIE);
	}
}

/****************************************************************************************************
 * @fn      		  - USART_StopRxData
 *
 * @brief             - This function stops RX interrupt
 *
 * @param[in]         - pointer to USART handle
 * @param[in]         - pointer to message message
 * @param[in]         -
 *
 * @return            - returns TRUE on success else FALSE
 *
 * @Note              - none

 */

void USART_StopRxData(USART_reg_t *pUSARTx, uint8_t USARTChannel){

	if(USART_RxMsg[USARTChannel]->msgStatus==READ_COMPLETE){
		USART_RxMsg[USARTChannel] = 0;

		//disable the interrupts
		CLEAR_BIT(pUSARTx->USART_CR1,RXNEIE);
	}
}

/****************************************************************************************************
 * @fn      		  - USART_FindTheAPB1Freq
 *
 * @brief             - This function find the APB2 clock frequency
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  6 bit clock frequency
 *
 * @Note              -  none

 */
uint8_t USART_FindTheAPB2Freq() {
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

	//find APB high pre-scaler
	switch (((RCC )->RCC_CFGR & CLOCK_APB2_BITMASK) >> PPRE2) {
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

	//divide the frequency with APB high pre-scaler
	freq /= div;

	return freq;
}
