/*
 * stm32f407xx_spio_drv.h
 *
 *  Created on: 09-Feb-2024
 *      Author: romsh
 */

#ifndef INC_STM32F407XX_SPI_DRV_H_
#define INC_STM32F407XX_SPI_DRV_H_

//#define SPI_RX_DEBUG
//#define SPI_TX_DEBUG

#include "stm32f407Header.h"
#include "stm32f407xx_gpio_drv.h"
#include "Interrupt.h"
#include <assert.h>

/*
 * application msg status
 */
#define TX_NOT_STARTED			0xA1
#define TX_IN_PROGRESS			0xA2
#define	TX_COMPLETE				0xA3
#define READ_NOT_STARTED		0xB1
#define READ_IN_PROGRESS		0xB2
#define READ_COMPLETE			0xB3

#define SPI_1					1
#define	SPI_2					2
#define SPI_3					3

#define SPI_SLAVE				0
#define SPI_MASTER				1

#define FULL_DUPLEX				0
#define HALF_DUPLEX				1
#define SIMPLEX_RX_ONLY			2
#define SIMPLEX_TX_ONLY			3

//data frame format
#define SPI_EIGHT_BIT			0
#define SPI_SIXTEEN_BIT			1

//clock phase
#define SPI_CLK_LEADING_EDGE	0
#define SPI_CLK_TRAILING_EDGE	1

//clock polarity
#define SPI_CLK_POLARITY_LOW	0
#define SPI_CLK_POLARITY_HIGH	1

//slave select management-HW or SW?
#define SPI_SW_DIS				0
#define SPI_SW_EN				1

//frame format
#define SPI_LSB_FIRST			0
#define SPI_MSB_FIRST			1

//baud rate
#define PRESCALER_2				0
#define PRESCALER_4				1
#define PRESCALER_8				2
#define PRESCALER_16			3    //communication with ard uno r3 doesnt take place till this frequency
#define PRESCALER_32			4
#define PRESCALER_64			5
#define PRESCALER_128			6
#define PRESCALER_256			7


//bit field of SPI CR1
#define MSTR					2
#define DFF						11
#define CPHA					0
#define CPOL					1
#define LSBFIRST				7
#define SSM						9
#define SSI						8
#define BR0						3
#define BR1						4
#define BR3						5
#define BIDIMODE				15
#define BIDIOE					14
#define RXONLY					10
#define CRCEN					13
#define CRCNEXT					12
#define SPE 					6

//bit field of SPI CR2
#define TXEIE					7
#define RXNEIE					6
#define ERRIE					5
#define FRF						4
#define SSOE					2
#define TXDMAEN					1
#define RXDMAEN					0

//bit field of SPI SR
#define FRE						8
#define BSY						7
#define OVR						6
#define MODF					5
#define CRCERR					4
#define UDR						3
#define CHSIDE					2
#define TXE						1
#define RXNE					0


typedef struct{
	uint8_t SPI_deviceMode;		//transmission mode - slave or master
	uint8_t SPI_busConfig;		//simplex or half duplex or full duplex
	uint8_t SPI_DFF;			//8 bit or 16 bit data
	uint8_t SPI_CPHA;			//clock phase (optional by default it is zero)
	uint8_t SPI_CPOL;			//clock polarity
	uint8_t SPI_SS;				//slave select management-software or hardware
	uint8_t SPI_FF;				//LSB first or MSB first frame format
	uint8_t SPI_speed;			//serial clock speed
	uint8_t IsMultiMaster;		//if the SPI network has multiple masters
	uint8_t SPI_RXNEIE;			//if yes, enable the RX interrupt
	uint8_t SPI_TXEIE;			//if yes, enable the TX interrupt
	uint8_t	SPI_ERRIE;			//interrupt on error
}SPI_config_t;

typedef struct{

	SPI_config_t sSPIConfig;	//structure containing config info
	SPI_reg_t *  pSPI;			//pointer to SPIx base address
}SPI_Handle_t;

typedef struct{
	__vo uint8_t *   Buffer;			//application tx buffer
	__vo uint8_t 	 Len;			//datalength of rx buffer
	uint8_t		     SpiChannel;		//SPI_channel
}SPI_Message_t;
/*
 * SPI Initialization
 */
void SPI_Init(SPI_Handle_t *pSPIHandle);

/*
 * used when software management of NSS pin is enabled
 */
void SPI_SSIconfig(__vo SPI_reg_t* pSPI, uint8_t IsHigh);

/*
 * SPI Enable
 */
void SPI_Enable(__vo SPI_reg_t* pSPI);

/*
 * SPI Disable
 */
void SPI_Disable(__vo SPI_reg_t* pSPIx);

/*
 * receive data from SPI bus
 * three types of receive methods
 * 1. polling ( blocking )
 * 2. interrupt ( non blocking )
 * 3. DMA based
 */
void SPI_ReceiveData(__vo SPI_reg_t* pSPI, void *pBuffer, uint32_t vBuflen);

/*
 * send data via SPI bus
 */
void SPI_SendData(__vo SPI_reg_t* pSPI, void *pBuffer, uint32_t vDataLen );

/*
 * IRQ configuration
 */
void SPI_IRQConfig(InterruptHandle* pIntHandle);

/*
 * enable SPI interrupt
 */
void SPI_EnableInterrupt(InterruptHandle* pIntHandle);

/*
 * Disable SPI interrupt
 */
void SPI_DisableInterrupt(InterruptHandle* pIntHandle);

/*
 * IRQ Handling
 */
void SPI_IRQHandling(__vo SPI_reg_t* pSPI,uint8_t SPINo);

/*
 * interrupt tx
 */

void SPI_SendDataInt(SPI_Message_t *SPIMsgHandle);

/*
 * interrupt rx
 */
void SPI_ReceiveDataInt(SPI_Message_t *SPIMsgHandle);

/*
 * SPI reset
 */
void SPI_reset(SPI_reg_t *pSPIx);

/*
 * SPI RXNE flag status
 */
uint8_t SPI_IsRXFlagSet(__vo SPI_reg_t* pSPIx);

/*
 * SPI TXNE flag status
 */
uint8_t SPI_IsTXFlagSet(__vo SPI_reg_t* pSPIx);

#endif /* INC_STM32F407XX_SPI_DRV_H_ */
