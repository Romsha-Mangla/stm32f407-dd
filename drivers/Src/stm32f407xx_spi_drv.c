/*
 * stm32f407xx_spi_drv.c
 *
 *  Created on: 09-Feb-2024
 *      Author: romsha
 */

#include "stm32f407xx_spi_drv.h"
#include <string.h>

#define SPI_WRITE_IS_COMPLETE				0x55


//flags
static uint8_t SPI_IntializationDone   			= FALSE;
static uint8_t SPI_InterruptConfigDone 			= FALSE;
__vo static uint8_t SPI1_TransmissionInProgress	= FALSE;
__vo static uint8_t SPI2_TransmissionInProgress	= FALSE;
__vo static uint8_t SPI3_TransmissionInProgress	= FALSE;
__vo static uint8_t SPI1_ReadInProgress			= FALSE;
__vo static uint8_t SPI2_ReadInProgress			= FALSE;
__vo static uint8_t SPI3_ReadInProgress			= FALSE;



//read and write buffer for each channel
static SPI_Message_t SPIWrite[SPI_3];
static SPI_Message_t SPIRead[SPI_3];

static void SPI_PeriClkCtrl(SPI_reg_t *pSPIx, uint8_t IsEnabled );
static void SPI_RXIConfig(SPI_reg_t * pSPI,uint8_t IsEnable);
static void SPI_TXIConfig(SPI_reg_t * pSPI,uint8_t IsEnable);

//write functions for each SPI channel
static void SPI1Write();
static void SPI2Write();
static void SPI3Write();

//read functions for each SPI channel
static void SPI1Read();
static void SPI2Read();
static void SPI3Read();

#ifdef SPI_RX_DEBUG
static void TurnOnDebugLed();
#endif

static void (*SPIWritefunc[3])() = {SPI1Write,SPI2Write,SPI3Write};
static void (*SPIReadfunc[3])()  = {SPI1Read,SPI2Read,SPI3Read};


/****************************************************************************************************
 * @fn      		  - SPI_Init
 *
 * @brief             - This function configures the SPI channel for use as per user preference
 *
 * @param[in]         - Handle to SPI config
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	assert(pSPIHandle!=0);

	//enable the clock first
	SPI_PeriClkCtrl(pSPIHandle->pSPI,TRUE);

	//if the SPI is configured as master
	if(pSPIHandle->sSPIConfig.SPI_deviceMode == SPI_MASTER)
	{
		pSPIHandle->pSPI->SPI_CR1 |= (SET<<MSTR); //this bit should not be changed if the comm. is going on
	}
	else
	{
		pSPIHandle->pSPI->SPI_CR1 &= (~(SET<<MSTR));
	}

	//set the data frame
	if(pSPIHandle->sSPIConfig.SPI_DFF == SPI_SIXTEEN_BIT)
	{
		pSPIHandle->pSPI->SPI_CR1 = pSPIHandle->pSPI->SPI_CR1 |(SET<<DFF);
	}
	else
	{
		pSPIHandle->pSPI->SPI_CR1 = pSPIHandle->pSPI->SPI_CR1 &(~(SET<<DFF));
	}

	//set clock phase

	if(pSPIHandle->sSPIConfig.SPI_CPHA == SPI_CLK_TRAILING_EDGE)
	{
		pSPIHandle->pSPI->SPI_CR1 |= (SET<CPHA);
	}
	else
	{
		pSPIHandle->pSPI->SPI_CR1 &= (~(SET<CPHA));
	}

	//set clock polarity

	if(pSPIHandle->sSPIConfig.SPI_CPOL == SPI_CLK_POLARITY_HIGH)
	{
		pSPIHandle->pSPI->SPI_CR1 = pSPIHandle->pSPI->SPI_CR1 | (SET<<CPOL);
	}
	else
	{
		pSPIHandle->pSPI->SPI_CR1 = pSPIHandle->pSPI->SPI_CR1 &(~ (SET<<CPOL));
	}

	//set slave select management

	if(pSPIHandle->sSPIConfig.SPI_SS == SPI_SW_EN)
	{
		pSPIHandle->pSPI->SPI_CR1 = pSPIHandle->pSPI->SPI_CR1| (SET<<SSM);

		if(pSPIHandle->sSPIConfig.IsMultiMaster == FALSE)
		{
			//added for single master network, it will keep the NSS high
			pSPIHandle->pSPI->SPI_CR1 |= (SET<<SSI);
		}
	}
	else if(pSPIHandle->sSPIConfig.SPI_SS == SPI_SW_DIS)
	{
		//make SSM bit 0
		pSPIHandle->pSPI->SPI_CR1 = pSPIHandle->pSPI->SPI_CR1&(~(SET<<SSM));

		if(pSPIHandle->sSPIConfig.IsMultiMaster == FALSE)
		{
			//added for single master network, it will keep the NSS low
			//when the software management of NSS pin is disabled.
			pSPIHandle->pSPI->SPI_CR2 |= (SET<<SSOE);
		}
	}

	//set frame format
	if(pSPIHandle->sSPIConfig.SPI_FF == SPI_MSB_FIRST)
	{
		pSPIHandle->pSPI->SPI_CR1  = pSPIHandle->pSPI->SPI_CR1 |(SET<<LSBFIRST);
	}
	else
	{
		pSPIHandle->pSPI->SPI_CR1  = pSPIHandle->pSPI->SPI_CR1 &(~(SET<<LSBFIRST));
	}

	//set baud rate
	pSPIHandle->pSPI->SPI_CR1 = pSPIHandle->pSPI->SPI_CR1| (pSPIHandle->sSPIConfig.SPI_speed<<BR0);

	//set simplex/half-duplex/duplex comm. config
	//by default, 2 line uni-direction mode i.e. FULL DUPLEX is selected

	if(pSPIHandle->sSPIConfig.SPI_busConfig==FULL_DUPLEX)
	{
		pSPIHandle->pSPI->SPI_CR1  &= (~(SET<<BIDIMODE));
	}
	else if(pSPIHandle->sSPIConfig.SPI_busConfig==HALF_DUPLEX)
	{
		//the master and slave will select complementary functionality for this mode
		pSPIHandle->pSPI->SPI_CR1 = pSPIHandle->pSPI->SPI_CR1 | (SET<<BIDIMODE);
	}
	else if(pSPIHandle->sSPIConfig.SPI_busConfig==SIMPLEX_RX_ONLY)
	{
		//master will not generate clock if there is no data to transmit, to put master in
		//receiver only mode, we must write to RXONLY bit in CR1 register to generate
		//clock on the line

		//also to put slave in receive only mode
		pSPIHandle->pSPI->SPI_CR1 = pSPIHandle->pSPI->SPI_CR1 | (SET<<RXONLY);
	}



#ifdef SPI_RX_DEBUG
		TurnOnDebugLed();
#endif

	SPI_IntializationDone = TRUE;

}



/****************************************************************************************************
 * @fn      		  - SPI_ReceiveData
 *
 * @brief             - This function receives data from SPI buffer
 *
 * @param[in]         - base address of the SPI peripheral
 * @param[in]         - pointer to rx buffer
 * @param[in]         - length of data to receive
 *
 * @return            -  none
 *
 * @Note              -  blocking call

 */

void SPI_ReceiveData(__vo SPI_reg_t* pSPI, void *pBuffer, uint32_t vDatalen)
{
	assert(pSPI);
	assert(pBuffer);
	assert(vDatalen!=0);

	//for full-duplex connection
	// if the data is 16 bit long
	if(READ_BIT_VALUE(pSPI->SPI_CR1,DFF)==SET)
	{
		// data length is even
		assert(vDatalen%2==0);

		while(vDatalen!=0)
		{
			//wait till bytes have been received
			//NOTE: can hang permanently, need watch dog to reset the MCU
			while(!(READ_BIT_VALUE(pSPI->SPI_SR,RXNE)));

			//copy 16 bits or two bytes at a time
			*((uint16_t*)pBuffer) = pSPI->SPI_DR;
			vDatalen = vDatalen-sizeof(uint16_t);
			(uint16_t*)pBuffer++;

#ifdef SPI_RX_DEBUG
			//for debugging
			GPIO_ToggleOutputPin(GPIOC,PIN8);
#endif
		}
	}
	else
	{
		while(vDatalen!=0)
		{
			//wait till bytes have been received
			while(!(READ_BIT_VALUE(pSPI->SPI_SR,RXNE)));

			//copy 8 bits or one byte at a time
			*((uint8_t*)pBuffer)= pSPI->SPI_DR ;
			vDatalen--;
			(uint8_t*)pBuffer++;

#ifdef SPI_RX_DEBUG
			//for debugging
			GPIO_ToggleOutputPin(GPIOC,PIN8);
#endif

		}
	}

		return;
}

/****************************************************************************************************
 * @fn      		  - SPI_SendData
 *
 * @brief             - This function sends data on given SPI channel;
 *
 * @param[in]         - base address of the SPI peripheral
 * @param[in]         - pointer to tx buffer
 * @param[in]         - length of the data in bytes
 *
 * @return            -  none
 *
 * @Note              -  this is a blocking call

 */

void SPI_SendData(__vo SPI_reg_t* pSPI, void *pBuffer, uint32_t vDataLen )
{
	assert(pSPI);
	assert(pBuffer);
	assert(vDataLen!=0);

	//for full-duplex connection
	// if the data is 16 bit long
	if(READ_BIT_VALUE(pSPI->SPI_CR1,DFF)==SET)
	{
		// data length is even
		assert(vDataLen%2==0);

		while(vDataLen!=0)
		{
			//wait till bytes have been sent
			//NOTE: can hang permanently, need watch dog to reset the MCU
			while(!(READ_BIT_VALUE(pSPI->SPI_SR,TXE)));

			//copy 16 bits or two bytes at a time
			pSPI->SPI_DR = *((uint16_t*)pBuffer);
			vDataLen = vDataLen-sizeof(uint16_t);
			(uint16_t*)pBuffer++;
#ifdef SPI_TX_DEBUG
			//for debugging
			GPIO_ToggleOutputPin(GPIOC,PIN5);
#endif
		}
	}
	else
	{
		while(vDataLen!=0)
		{
			//wait till bytes have been sent
			while(!(READ_BIT_VALUE(pSPI->SPI_SR,TXE)));

			//copy 8 bits or one byte at a time
			pSPI->SPI_DR = *((uint8_t*)pBuffer);
			vDataLen--;
			(uint8_t*)pBuffer++;



#ifdef SPI_TX_DEBUG
			//for debugging
			GPIO_ToggleOutputPin(GPIOC,PIN5);
#endif

		}
	}


	return;
}

/****************************************************************************************************
 * @fn      		  - SPI_IRQConfig
 *
 * @brief             - This function configures SPI interrupt
 *
 * @param[in]         - handle to interrupt config
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -

 */
void SPI_IRQConfig(InterruptHandle* pIntHandle)
{
	assert(pIntHandle!=NULL);
	if(pIntHandle->IRQNum==SPI1_IRQ||pIntHandle->IRQNum==SPI2_IRQ||pIntHandle->IRQNum==SPI3_IRQ){
	SPI_InterruptConfigDone = FALSE;

	CortexM4SetIntPriority(pIntHandle->IRQNum, pIntHandle->IntPri);

	if(pIntHandle->IsEnable==TRUE)
	{
		CortexM4EnableIRQ(pIntHandle->IRQNum);
	}

	SPI_InterruptConfigDone = TRUE;
	}
	else{
		pIntHandle->IsEnable = SPI_INTERRUPT_CONFIG_ERROR;
	}
	return;

}

/****************************************************************************************************
 * @fn      		  - SPI_EnableInterrupt
 *
 * @brief             - This function enables SPI interrupt
 *
 * @param[in]         - handle to interrupt config
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  this function will enable the interrupts if the irq config is already done.

 */

void SPI_EnableInterrupt(InterruptHandle* pIntHandle)
{
	assert(pIntHandle!=NULL);
	assert((pIntHandle->IRQNum==SPI1_IRQ||pIntHandle->IRQNum==SPI2_IRQ||pIntHandle->IRQNum==SPI3_IRQ));


	assert(SPI_InterruptConfigDone==TRUE);

	CortexM4EnableIRQ(pIntHandle->IRQNum);
	pIntHandle->IsEnable = TRUE;

}

/****************************************************************************************************
 * @fn      		  - SPI_DisableInterrupt
 *
 * @brief             - This function disables SPI interrupt
 *
 * @param[in]         - handle to interrupt config
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  this function will disable the interrupts if the irq config is already done.

 */
void SPI_DisableInterrupt(InterruptHandle* pIntHandle)
{
	assert(pIntHandle!=NULL);
	assert((pIntHandle->IRQNum==SPI1_IRQ||pIntHandle->IRQNum==SPI2_IRQ||pIntHandle->IRQNum==SPI3_IRQ));

	CortexM4DisableIRQ(pIntHandle->IRQNum);
	pIntHandle->IsEnable = FALSE;
}

/****************************************************************************************************
 * @fn      		  - SPI_IRQHandling
 *
 * @brief             - This function does some interrupt handling.
 *
 * @param[in]         - base address of the SPI peripheral
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -

 */

void SPI_IRQHandling(__vo SPI_reg_t* pSPI,uint8_t SpiNo)
{

	if((((pSPI->SPI_SR>>RXNE)&SET)==SET)&&((pSPI->SPI_CR2>>RXNEIE&SET)==SET))
	{
		SPIReadfunc[SpiNo-1]();
	}

	if((((pSPI->SPI_SR>>TXE)&SET)==SET)&&((pSPI->SPI_CR2>>TXEIE&SET)==SET))
	{
		SPIWritefunc[SpiNo-1]();
	}

	return;
}


/****************************************************************************************************
 * @fn      		  - SPI2_IRQHandler
 *
 * @brief             - ISR of SPI1
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -

 */
void SPI1_IRQHandler()
{
	SPI_IRQHandling(SPI1,SPI_1);
}

/****************************************************************************************************
 * @fn      		  - SPI2_IRQHandler
 *
 * @brief             - ISR of SPI2
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -

 */
void SPI2_IRQHandler()
{
	SPI_IRQHandling(SPI2,SPI_2);
}

/****************************************************************************************************
 * @fn      		  - SPI3_IRQHandler
 *
 * @brief             - ISR of SPI3
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -

 */
void SPI3_IRQHandler()
{
	SPI_IRQHandling(SPI3,SPI_3);
}

/****************************************************************************************************
 * @fn      		  - SPI_SendDataInt
 *
 * @brief             - This function is called by application to send data through SPI channel
 * @param[in]         - handle to SPI message packet
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -
 *
 */

void SPI_SendDataInt(SPI_Message_t *SPIMsgHandle)
{

 switch(SPIMsgHandle->SpiChannel)
  {
	case SPI_1:
				if(!SPI1_TransmissionInProgress)
				{
					//disable the TX interrupt
					SPI_TXIConfig(SPI1,FALSE);

					//update the global message handler with application pointer
					SPIWrite[0].Buffer        = SPIMsgHandle->Buffer;
					SPIWrite[0].Len    	   	  = SPIMsgHandle->Len;
					SPIWrite[0].SpiChannel    = SPIMsgHandle->SpiChannel;
					//enable the TX interrupt
					SPI_TXIConfig(SPI1,TRUE);
				}
				break;

	case SPI_2:
				if(!SPI2_TransmissionInProgress)
				{
					//disable the TX interrupt
					SPI_TXIConfig(SPI2,FALSE);

					//update the global message handler with application pointer
					SPIWrite[1].Buffer        = SPIMsgHandle->Buffer;
					SPIWrite[1].Len    	      = SPIMsgHandle->Len;
					SPIWrite[1].SpiChannel    = SPIMsgHandle->SpiChannel;

					//enable the TX interrupt
					SPI_TXIConfig(SPI2,TRUE);
				}

				break;

	case SPI_3:
				if(!SPI3_TransmissionInProgress)
				{
					//disable the TX interrupt
					SPI_TXIConfig(SPI3,FALSE);

					//update the global message handler with application pointer
					SPIWrite[2].Buffer        = SPIMsgHandle->Buffer;
					SPIWrite[2].Len    	      = SPIMsgHandle->Len;
					SPIWrite[2].SpiChannel    = SPIMsgHandle->SpiChannel;

					//enable the TX interrupt
					SPI_TXIConfig(SPI3,TRUE);
				 }

				break;

	default:

			break;
	}

	return;
}

/****************************************************************************************************
 * @fn      		  - SPI_ReceiveDataInt
 *
 * @brief             - This function receive data when an interrupt occurs
 * @param[in]         - handle to SPI peripheral
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -

 */
void SPI_ReceiveDataInt(SPI_Message_t *SPIMsgHandle)
{
	switch(SPIMsgHandle->SpiChannel)
	{
		case SPI_1:
					if(!SPI1_ReadInProgress)
					{
						SPI_RXIConfig(SPI1,FALSE);
						SPIRead[0].Buffer 		= SPIMsgHandle->Buffer;
						SPIRead[0].Len    		= SPIMsgHandle->Len;
						SPIRead[0].SpiChannel   = SPIMsgHandle->SpiChannel;
						SPI_RXIConfig(SPI1,TRUE);
					}
					break;
		case SPI_2:
					if(!SPI2_ReadInProgress)
					{
						SPI_RXIConfig(SPI2,FALSE);
						SPIRead[1].Buffer 		= SPIMsgHandle->Buffer;
						SPIRead[1].Len    		= SPIMsgHandle->Len;
						SPIRead[1].SpiChannel   = SPIMsgHandle->SpiChannel;
						SPI_RXIConfig(SPI2,TRUE);
					}
					break;
		case SPI_3:
					if(!SPI3_ReadInProgress)
					{
						SPI_RXIConfig(SPI3,FALSE);
						SPIRead[2].Buffer 		= SPIMsgHandle->Buffer;
						SPIRead[2].Len    		= SPIMsgHandle->Len;
						SPIRead[2].SpiChannel  = SPIMsgHandle->SpiChannel;
						SPI_RXIConfig(SPI3,TRUE);
					}
					break;

		default:
					break;
	}

	return;
}
/****************************************************************************************************
 * @fn      		  - SPI_SSIConfig
 *
 * @brief             - This function sets/resets the bit SSI which in turn connects/disconnects NSS pin internally to Vdd.
 *
 * @param[in]         - base address of the SPI peripheral
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  must be called after initialization if there is only single master in TX only mode

 */
void SPI_SSIconfig(__vo SPI_reg_t* pSPI, uint8_t IsHigh)
{
	 if(IsHigh==SET)
	 {
		 //set the NSS pin high
		 pSPI->SPI_CR1 |= (SET<<SSI);
	 }
	 else
	 {
		 //set the NSS pin low
		 pSPI->SPI_CR1 &= (~(SET<<SSI));
	 }
}

/****************************************************************************************************
 * @fn      		  - SPI_TXICOnfig
 *
 * @brief             - This function enables/disables the TX interrupt
 *
 * @param[in]         - base address of the SPI peripheral
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  must be called after initialization if there is only single master in TX only mode

 */
void SPI_TXIConfig(SPI_reg_t * pSPI,uint8_t IsEnable)
{
	if(IsEnable==FALSE)
	{
		pSPI->SPI_CR2&=~(SET<<TXEIE);

	}
	else
	{
		pSPI->SPI_CR2|=(SET<<TXEIE);
	}
}

/****************************************************************************************************
 * @fn      		  - SPI_RXICOnfig
 *
 * @brief             - This function enables/disables the RX interrupt
 *
 * @param[in]         - base address of the SPI peripheral
 * @param[in]         - TRUE or FALSE
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  must be called after initialization if there is only single master in TX only mode

 */
void SPI_RXIConfig(SPI_reg_t * pSPI,uint8_t IsEnable)
{
	if(IsEnable==FALSE)
	{
		//reset RXNEIE flag to disable the read interrupt
		pSPI->SPI_CR2&=~(SET<<RXNEIE);

	}
	else
	{
		pSPI->SPI_CR2|=(SET<<RXNEIE);
	}

}

/****************************************************************************************************
 * @fn      		  - SPI_Enable
 *
 * @brief             - This function enables the SPI channel.setting SPE to 1 will make the SPI busy
 *
 * @param[in]         - base address of the SPI peripheral
 * @param[in]         -
 * @param[in]         -
 *
 * @return            - none
 *
 * @Note              - Must be called after initialization is done.

 */
void SPI_Enable(__vo SPI_reg_t* pSPI)
{
	assert(pSPI);

	assert(SPI_IntializationDone);

	//enable the SPI by writing one to bit 6(SPE) in CR1 register
	//enable the peripheral after config because as soon as the bit is set SPI will
	//become busy in communication, it will not accept any config change

	//as soon as SPE is set, and if SSM = 0 i.e NSS pin is hardware managed, the NSS pin is pulled
	//low automatically ( NSS output =0 ), this is the feature of STM32 micro controller.
	pSPI->SPI_CR1 |= (SET<<SPE);

}


/****************************************************************************************************
 * @fn      		  - SPI_DeInit
 *
 * @brief             - This function de-initializes the SPI channel
 *
 * @param[in]         - base address of the SPI peripheral
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  setting SPE to 0 will make NSS high if SSM=0(software management is disabled)

 */
void SPI_Disable(__vo SPI_reg_t* pSPIx)
{
	assert(pSPIx);

	//it means SPI is in full-duplex mode
	if((READ_BIT_VALUE(pSPIx->SPI_CR1,BIDIMODE)==RESET)&&(READ_BIT_VALUE(pSPIx->SPI_CR1,RXONLY)==RESET))
	{
//		//there is data in RX buffer, when RXNE is reset, it means rx buffer is read
//		while((pSPIx->SPI_SR>>RXNE&1)==SET);
//
//		//when TXE is set it means TX buffer is empty, so wait till the data is sent
//		while((pSPIx->SPI_SR>>TXE&1)==RESET);

		//check BSY flag, if it is set, it means SPI is busy communicating
		while((pSPIx->SPI_SR>>BSY&1)==SET);

		//disable the SPI
		pSPIx->SPI_CR1 &= ~(SET<<SPE);
	}
	//transmit only
	else if(READ_BIT_VALUE(pSPIx->SPI_CR1,BIDIOE)==SET)
	{
		//when TXE is set it means TX buffer is empty, so wait till the data is sent
		while((pSPIx->SPI_SR>>TXE&1)==RESET);

		//check BSY flag, if it is set, it means SPI is busy communicating
		while((pSPIx->SPI_SR>>BSY&1)==SET);

		//disable the SPI
		pSPIx->SPI_CR1 &= ~(SET<<SPE);
	}
	//slave in receive only mode
	//this is done because BSY flag has the tendency to get stuck when slave is in RX only mode
	else if(((READ_BIT_VALUE(pSPIx->SPI_CR1,RXONLY)==SET)||
			((READ_BIT_VALUE(pSPIx->SPI_CR1,BIDIOE)==RESET)&&(READ_BIT_VALUE(pSPIx->SPI_CR1,BIDIMODE)==SET))))
	{
		//there is data in RX buffer, when RXNE is reset, it means rx buffer is read
		while((pSPIx->SPI_SR>>RXNE&1)==SET);

		//disable the SPI
		pSPIx->SPI_CR1 &= ~(SET<<SPE);
	}

}

/****************************************************************************************************
 * @fn      		  - SPI_IsRXFlagSet
 *
 * @brief             -  This function return TRUE if RXNE flag is set
 *
 * @param[in]         -  base address of the SPI peripheral
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */

uint8_t SPI_IsRXFlagSet(__vo SPI_reg_t* pSPIx)
{
	//there is data in RX buffer, RXNE remains set
	//when RXNE is reset, it means rx buffer is read
	if((pSPIx->SPI_SR>>RXNE&1)==SET)
		return TRUE;
	else
		return FALSE;
}

/****************************************************************************************************
 * @fn      		  - SPI_IsTXFlagSet
 *
 * @brief             -  This function return TRUE if TXNE flag is set
 *
 * @param[in]         -  base address of the SPI peripheral
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */

uint8_t SPI_IsTXFlagSet(__vo SPI_reg_t* pSPIx){

	//when TXE is set it means TX buffer is empty,
	// and the application can send the data
	if((pSPIx->SPI_SR>>TXE&1)==SET)
		return TRUE;
	else
		return FALSE;
}

/****************************************************************************************************
 * @fn      		  - SPI_reset
 *
 * @brief             -  This function resets SPI registers
 *
 * @param[in]         -  base address of the SPI peripheral
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */

void SPI_reset(SPI_reg_t *pSPIx)
{

	SPI_Disable(pSPIx);

	if(pSPIx==SPI1)
	{
		RESET_SPI1();
		SET_SPI1();
	}
	else if(pSPIx==SPI2)
	{
		RESET_SPI2();
		SET_SPI2();
	}
	else
	{
		RESET_SPI3();
		SET_SPI3();
	}
}

/****************************************************************************************************
 * @fn      		  - SPI_PeriClkCtrl
 *
 * @brief             - This function enables or disables peripheral clock for the given SPI bus
 *
 * @param[in]         - base address of the SPI peripheral
 * @param[in]         - ENABLE or DISABLE
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void SPI_PeriClkCtrl( SPI_reg_t *pSPIx, uint8_t IsEnabled )
{
	assert(pSPIx!=0);

	if(IsEnabled)
	{
		if(pSPIx==SPI1)
		{
			EN_PCLK_SPI1();
		}
		else if(pSPIx==SPI2)
		{
			EN_PCLK_SPI2();
		}
		else
		{
			EN_PCLK_SPI3();
		}
	}
	else
	{
		if(pSPIx==SPI1)
		{
			DIS_PCLK_SPI1();
		}
		else if(pSPIx==SPI2)
		{
			DIS_PCLK_SPI2();
		}
		else
		{
			DIS_PCLK_SPI3();
		}
	}
	return;
}

/****************************************************************************************************
 * @fn      		  - SPI1Write
 *
 * @brief             - This function transmits the data on SPIx( where x is 1,2 0r 3) when TX interrupt
 * 					    is received
 *
 * @param[in]         - None
 * @param[in]         - None
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */

void SPI1Write()
{
	if(((SPI1->SPI_CR1>>DFF)&1)==RESET)
	{
		if(SPIWrite[0].Len)
		{
			SPI1->SPI_DR = *(SPIWrite[0].Buffer);
			(SPIWrite[0].Buffer)++;
			SPIWrite[0].Len--;
			SPI1_TransmissionInProgress = SET;
		}
		else
		{
			SPI1_TransmissionInProgress = RESET;
			SPI_TXIConfig(SPI1,FALSE);
		}
	}
	else
	{
		if(SPIWrite[0].Len)
		{
			SPI1->SPI_DR = *((uint16_t*)(SPIWrite[0].Buffer));
			(uint16_t*)(SPIWrite[0].Buffer)++;
			SPIWrite[0].Len-=2;
			SPI1_TransmissionInProgress = SET;
		}
		else
		{
			SPI1_TransmissionInProgress = RESET;
			SPI_TXIConfig(SPI1,FALSE);
		}
	}

	return;
}

/****************************************************************************************************
 * @fn      		  - SPI2Write
 *
 * @brief             - This function transmits the data on SPIx( where x is 1,2 0r 3) when TX interrupt
 * 						 is received
 *
 * @param[in]         - None
 * @param[in]         - None
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */

void SPI2Write()
{
	if(((SPI2->SPI_CR1>>DFF)&1)==RESET)
	{
		if(SPIWrite[1].Len)
		{
			SPI2->SPI_DR = *(SPIWrite[1].Buffer);
			(SPIWrite[1].Buffer)++;
			SPIWrite[1].Len--;
			SPI2_TransmissionInProgress = SET;
		}
		else
		{
			SPI2_TransmissionInProgress = RESET;
			SPI_TXIConfig(SPI2,FALSE);
		}
	}
	else
	{
		if(SPIWrite[1].Len)
		{
			SPI2->SPI_DR = *((uint16_t*)(SPIWrite[1].Buffer));
			(uint16_t*)(SPIWrite[1].Buffer)++;
			SPIWrite[1].Len-=2;
			SPI2_TransmissionInProgress = SET;
		}
		else
		{
			SPI2_TransmissionInProgress = RESET;
			SPI_TXIConfig(SPI2,FALSE);
		}
	}
}

/****************************************************************************************************
 * @fn      		  - SPI3Write
 *
 * @brief             - This function transmits the data on SPIx( where x is 1,2 0r 3) when TX interrupt
 * 						 is received
 *
 * @param[in]         - None
 * @param[in]         - None
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */

void SPI3Write()
{
	if(((SPI3->SPI_CR1>>DFF)&1)==RESET)
	{
		if(SPIWrite[2].Len)
		{
			SPI3->SPI_DR = *(SPIWrite[2].Buffer);
			(SPIWrite[2].Buffer)++;
			SPIWrite[2].Len--;
			SPI3_TransmissionInProgress = SET;
		}
		else
		{
			SPI3_TransmissionInProgress = RESET;
			SPI_TXIConfig(SPI3,FALSE);
		}
	}
	else
	{
		if(SPIWrite[2].Len)
		{
			SPI3->SPI_DR = *((uint16_t*)(SPIWrite[2].Buffer));
			(uint16_t*)(SPIWrite[2].Buffer)++;
			SPIWrite[2].Len-=2;
			SPI3_TransmissionInProgress = SET;
		}
		else
		{
			SPI3_TransmissionInProgress = RESET;
			SPI_TXIConfig(SPI2,FALSE);
		}
	}
}

/****************************************************************************************************
 * @fn      		  - SPI1Read
 *
 * @brief             - This function receives the data on SPIx( where x is 1,2 0r 3) when RX interrupt
 *                      is received
 *
 * @param[in]         - None
 * @param[in]         - None
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */

void SPI1Read()
{
	if(((SPI1->SPI_CR1>>DFF)&1)==RESET)
	{
		if(SPIRead[0].Len)
		{
			*(SPIRead[0].Buffer) = SPI1->SPI_DR;
			 (SPIRead[0].Buffer)++;
			  SPIRead[0].Len--;
			  SPI1_ReadInProgress = SET;
		}
		else
		{
			SPI1_ReadInProgress = RESET;
			SPI_RXIConfig(SPI1,FALSE);
		}
	}
	else
	{
		if(SPIRead[0].Len)
		{
			*((uint16_t*)(SPIRead[0].Buffer)) = SPI1->SPI_DR;
			 (uint16_t*)(SPIRead[0].Buffer)++;
			  SPIRead[0].Len-=2;;
			  SPI1_ReadInProgress = SET;
		}
		else
		{
			SPI1_ReadInProgress = RESET;
			SPI_RXIConfig(SPI1,FALSE);
		}
	}
	return;
}

/****************************************************************************************************
 * @fn      		  - SPI2Read
 *
 * @brief             - This function transmits the data on SPIx( where x is 1,2 0r 3) when TX interrupt
 * 						is received
 *
 * @param[in]         - None
 * @param[in]         - None
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */

void SPI2Read()
{
	if(((SPI2->SPI_CR1>>DFF)&1)==RESET)
	{
		if(SPIRead[1].Len)
		{
			*(SPIRead[1].Buffer) = SPI2->SPI_DR;
			 (SPIRead[1].Buffer)++;
			  SPIRead[1].Len--;
			  SPI2_ReadInProgress = SET;
		}
		else
		{
			SPI2_ReadInProgress = RESET;
			SPI_RXIConfig(SPI2,FALSE);
		}
	}
	else
	{
		if(SPIRead[1].Len)
		{
			*((uint16_t*)(SPIRead[1].Buffer)) = SPI2->SPI_DR;
			 (uint16_t*)(SPIRead[1].Buffer)++;
			  SPIRead[1].Len-=2;;
			  SPI2_ReadInProgress = SET;
		}
		else
		{
			SPI2_ReadInProgress = RESET;
			SPI_RXIConfig(SPI1,FALSE);
		}
	}
	return;
}

/****************************************************************************************************
 * @fn      		  - SPI3Read
 *
 * @brief             - This function transmits the data on SPIx( where x is 1,2 0r 3) when TX interrupt
 * 						is received
 *
 * @param[in]         - None
 * @param[in]         - None
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */

void SPI3Read()
{
	if(((SPI3->SPI_CR1>>DFF)&1)==RESET)
	{
		if(SPIRead[2].Len)
		{
			*(SPIRead[2].Buffer) = SPI3->SPI_DR;
			 (SPIRead[2].Buffer)++;
			  SPIRead[2].Len--;
			  SPI3_ReadInProgress = SET;
		}
		else
		{
			SPI3_ReadInProgress = RESET;
			SPI_RXIConfig(SPI3,FALSE);
		}
	}
	else
	{
		if(SPIRead[2].Len)
		{
			*((uint16_t*)(SPIRead[2].Buffer)) = SPI3->SPI_DR;
			 (uint16_t*)(SPIRead[2].Buffer)++;
			  SPIRead[2].Len-=2;;
			  SPI3_ReadInProgress = SET;
		}
		else
		{
			SPI3_ReadInProgress = RESET;
			SPI_RXIConfig(SPI3,FALSE);
		}
	}
	return;
}

#ifdef SPI_RX_DEBUG
void TurnOnDebugLed()
{
	GPIO_Handle_t led;
	led.pGPIO 								= GPIOC;
	led.GPIOPinConfig.GPIOmode    			= OUTPUT_MODE;
	led.GPIOPinConfig.GPIOOutputType		= OP_TYPE_PP;
	led.GPIOPinConfig.GPIOspeed 			= VERY_HIGH_SPEED;
	led.GPIOPinConfig.GPIOPullUpPullDown  	= NO_PU_NO_PD;

	led.GPIOPinConfig.GPIO_PinNum = PIN8;
	GPIO_Init(&led);

#ifdef SPI_TX_DEBUG
	led.GPIOPinConfig.GPIO_PinNum = PIN5;
	GPIO_Init(&led);
#endif

}
#endif



