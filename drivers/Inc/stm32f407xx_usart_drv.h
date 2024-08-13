/*
 * stm32f407xx_usart_drv.h
 *
 *  Created on: 31-May-2024
 *      Author: romsha
 */

#ifndef INC_STM32F407XX_USART_DRV_H_
#define INC_STM32F407XX_USART_DRV_H_

#include "stm32f407Header.h"
#include "interrupt.h"

/*
 * USART_SR -reset value 0x000000C0
 */
#define CTS					9		//CTS flag
#define LBD					8		//LIN break detection flag
#define TXE					7		//Transmit data register empty
#define TC					6		//Transmission complete
#define RXNE				5		//Read data register not empty
#define IDLE				4		//IDLE line detected
#define ORE					3		//Overrun error
#define NF					2		//Noise detected flag
#define FE					1		//Framing error
#define PE					0		//Parity error

/*
 * USART_BRR
 */
#define DIV_FRA_3_0 		0		//These 4 bits define the fraction of the USART Divider
#define DIV_MAN_4_11		4		//These 12 bits define the mantissa of the USART Divider (USARTDIV)

/*
 * USART_CR1
 */
#define OVER8				15		//over sampling
#define UE					13		//USART enable
#define M					12		//Word Length
#define WAKE				11		//Wakeup method
#define PCE					10		//parity control
#define PS					9		//Parity selection
#define PEIE				8		//PE interrupt enable
#define TXEIE				7		//TXE interrupt enable
#define TCIE 				6		//Transmission complete interrupt enable
#define RXNEIE 				5		//RXNE interrupt enable
#define IDLEIE				4		//Idle interrupt enable
#define TE					3		//Transmitter enable
#define RE					2		//Receiver enable
#define RWU					1		//Receiver wakeup
#define SBK					0		//Send break

/*
 * USART_CR2
 */
#define LINEN				14		//LIN mode enable
#define STOP_1_0			12		//STOP bits
#define CLKEN				11		//Clock enable
#define CPOL				10		//clock polarity
#define CPHA				9		//clock phase
#define LBCL				8		//last bit clock pulse
#define LBDIE				6		//LIN break detection interrupt enable
#define LBDL				5		//lin break detection length
#define ADD_3_0				0		//Address of the USART node

/*
 * USART_CR3
 */
#define ONEBIT				11	    //One sample bit method enable
#define CTSIE				10		//CTS interrupt enable
#define CTSE 				9		//CTS enable (clear to send)
#define RTSE 				8		//RTS enable (request to send)
#define DMAT 				7		//DMA enable transmitter
#define DMAR 				6		//DMA enable receiver
#define SCEN 				5		//Smart card mode enable
#define NACK 				4		//Smart card NACK enable
#define HDSEL 				3		//Half Duplex selection
#define IRLP				2		//IrDA low power
#define IREN				1		//IrDA mode enable
#define EIE					0		//Error interrupt enable

/*
 * USART_GTPR
 */
#define PSC_7_0				0		//IrDA Low-Power Baud Rate
#define GT_7_0				8		//Guard time value

/*
 * USART channel no
 */
#define USART_1				1
#define USART_2				2
#define USART_3				3
#define UART_4				4
#define UART_5				5
#define USART_6				6

/*
 * type of error
 */
#define TX_NOT_STARTED		0xA1
#define TX_IN_PROGRESS		0xA2
#define	TX_COMPLETE			0xA3
#define TX_ERROR			0xA4
#define READ_NOT_STARTED	0xB1
#define READ_IN_PROGRESS	0xB2
#define READ_COMPLETE		0xB3
#define READ_ERROR			0xB4

#define USART_OVERRUN		0xD1
#define USART_NOISE_DET		0xD2
#define USART_FRM_ERROR		0xD3
#define USART_PAR_ERROR		0xD4

/*
 * WORD LENGTH
 */
#define USART_8_BIT			0
#define USART_9_BIT			1

/*
 * STOP BIT
 */
#define USART_HALF_BIT_STOP				0
#define USART_ONE_BIT_STOP				1
#define USART_ONEANDHALF_BIT_STOP		2
#define USART_TWO_BIT_STOP				3

/*
 * USART mode
 */

#define USART_ONLY_TX		0
#define USART_ONLY_RX		1
#define USART_TX_RX			2

/*
 * USART flow control
 */
#define USART_HW_FLOW_CTRL_NONE		0
#define USART_HW_FLOW_CTRL_CTS 		1
#define USART_HW_FLOW_CTRL_RTS 		2
#define USART_HW_FLOW_CTRL_CTS_RTS	3

/*
 * OVERSAMPLING rate
 */
#define USART_OVER_SMPLING_16				0
#define USART_OVER_SMPLING_8				1

/*
 * BAUD RATE
 */
#define USART_BAUD_1200					1200
#define USART_BAUD_2400					2400
#define USART_BAUD_9600					9600
#define USART_BAUD_19200				19200
#define USART_BAUD_38400				38400
#define USART_BAUD_57600				57600
#define USART_BAUD_115200			    115200
#define USART_BAUD_230400				230400
#define USART_BAUD_460800				460800
#define USART_BAUD_921600				921600
#define USART_BAUD_2000000				2000000
#define USART_BAUD_3000000				3000000

/*
 * PARITY
 */
#define USART_NO_PARITY				0
#define USART_EVEN_PARITY			1
#define USART_ODD_PARITY			2

typedef struct{
uint8_t USART_Parity;
uint8_t USART_Mode;
uint8_t USART_WordLength;
uint8_t USART_StopBit;
uint8_t USART_IsHWFctrlEnbl;		//true for enable. false for disable
uint8_t USART_OverSampling;
uint32_t USART_BaudRate;
}USART_config_t;

typedef struct{
	USART_reg_t *pUSARTx;
	USART_config_t USARTConfig;
}USART_Handle_t;

typedef struct{
	uint32_t msgLen;  	//holds the no. of bytes received or transmitted as of now
	uint32_t msgSize; 	//total no. of bytes to be received or sent by master
	uint8_t *msgBuffer;
	uint8_t errorType; 	//holds the type of error
	uint8_t msgStatus;
	uint8_t USARTChannel;
}USART_Message_t;


/*
 * USART initialization
 */
void USART_Init(USART_Handle_t *USARTHandle);

/*
 * USART reset function
 */
void USART_reset(USART_reg_t *pUSARTx);

/*
 * Enable USART
 */
void USART_Enable(USART_reg_t *pUSARTx);

/*
 * USART_Disable
 */
void USART_Disable(USART_reg_t *pUSARTx);

/*
 * USART send function
 */
uint8_t USART_TxData(USART_Handle_t *pUSARTHandle, USART_Message_t *Msg);

/*
 * Enable Transmission
 */
void USART_EnableTX(USART_reg_t *pUSARTx);

/*
 * Disable USART transmission
 */
void USART_DisableTX(USART_reg_t *pUSARTx);

/*
 * USART receive function
 */
uint8_t USART_RxData(USART_Handle_t *pUSARTHandle, USART_Message_t *Msg);

/*
 * Enable reception
 */
void USART_EnableRX(USART_reg_t *pUSARTx);

/*
 * Disable reception
 */
void USART_DisableRX(USART_reg_t *pUSARTx);

/*
 * USART interrupt TX function
 */
uint8_t USART_ITTxData(USART_Handle_t *pUSARTHandle, USART_Message_t *Msg);

/*
 * USART interrupt RX function
 */
uint8_t USART_ITRxData(USART_Handle_t *pUSARTHandle, USART_Message_t *Msg);

/*
 * stop tranmission
 */
void USART_StopTxData(USART_reg_t *pUSARTx, uint8_t USARTChannel);

/*
 * Stop reception
 */
void USART_StopRxData(USART_reg_t *pUSARTx, uint8_t USARTChannel);

/*
 *  USART interrupt config
 */
void USART_IRQConfig(InterruptHandle * pIntHandle);

#endif /* INC_STM32F407XX_USART_DRV_H_ */
