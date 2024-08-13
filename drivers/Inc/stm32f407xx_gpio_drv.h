/*
 * stm32f407xx_gpio_drv.h
 *
 *  Created on: 13-Jan-2024
 *      Author: romsha
 */

#ifndef INC_STM32F407XX_GPIO_DRV_H_
#define INC_STM32F407XX_GPIO_DRV_H_

#include "stm32f407Header.h"
#include "Interrupt.h"
//GPIO handle which contains the info from the user to configure the GPIO pin

/*
 * GPIO pin number
 */
#define PIN0	0
#define PIN1	1
#define PIN2	2
#define PIN3	3
#define PIN4	4
#define PIN5	5
#define PIN6	6
#define PIN7	7
#define PIN8	8
#define PIN9	9
#define PIN10	10
#define PIN11	11
#define PIN12	12
#define PIN13	13
#define PIN14	14
#define PIN15	15

/*
 * GPIO mode
 */
#define INPUT_MODE	0
#define OUTPUT_MODE	1
#define ALT_FUNC	2
#define ANALOG_MODE	3
#define GPIO_INT_FT	4 //falling edge
#define GPIO_INT_RT	5 //rising edge
#define GPIO_INT_FR	6

/*
 * OUTPUT TYPE
 */
#define OP_TYPE_PP	0 // output type push pull
#define OP_TYPE_OD	1 //output type open drain

/*
 * OUTPUT SPEED
 */
#define LOW_SPEED		0
#define MED_SPEED		1
#define HIGH_SPEED		2
#define VERY_HIGH_SPEED	3

/*
 * PUSH-PULL register values
 */
#define NO_PU_NO_PD		0
#define PULL_UP			1
#define PUSH_DOWN		2


/*
 * Lock port register values
 */
#define LOCK_NOT_ACTIVE	0
#define LOCK_ACTIVE		1 //until MCU or peripheral reset

/*
 * alternate functions AFRLy and AFRHy
 */
#define AF0		0 //sys
#define AF1		1 //TIM1/2
#define AF2		2 //TIM3/4/5
#define AF3		3 //TIM8/9/10/11
#define AF4		4 //I2C 1/2/3
#define AF5		5 //SPI1/SPI2/I2S2/I2S2ext
#define AF6		6 //SPI3/I2Sext/I2S3
#define AF7		7 //USART1/2/3/I2S3ext
#define AF8		8 //UART4/5 USART 6
#define AF9		9 //CAN1/2 TIM12/13/14
#define AF10	10 //OTG_FS/HS
#define AF11	11 //ETH
#define AF12	12 //FSMC/SDIO/OTG/_FS
#define AF13	13 //DCMI
#define AF14	14 //
#define AF15	15 //Eventout

/*
 * for ports
 */
#define PORTA	0
#define PORTB	1U
#define PORTC	2U
#define PORTD	3U
#define PORTE	4U
#define PORTF	5U
#define PORTG	6U
#define PORTH	7U
#define PORTI	8U


typedef struct{
	uint8_t GPIO_PinNum;
	uint8_t GPIOmode;
	uint8_t GPIOspeed;
	uint8_t GPIOOutputType;
	uint8_t GPIOPullUpPullDown;
	uint8_t GPIOAltFunc;
	uint8_t GPIOPort;
}GPIO_Pinconfig_t;

typedef struct{
	GPIO_reg_t* pGPIO; //the base address of GPIO peripheral
	GPIO_Pinconfig_t GPIOPinConfig; // the pin no and configuration info
}GPIO_Handle_t;

/*
 * peripheral clock
 */
void GPIO_PeriClkCtrl(GPIO_reg_t *pGPIOx, uint8_t IsEnabled );

/*
 * GPIO port Initialization
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);

/*
 * GPIO port disable
 */
void GPIO_DeInit(GPIO_reg_t* pGPIO); // this parameter to identify which port registers to reset

/*
 * read and write to pin
 */
uint8_t GPIO_readFrmInputPin(GPIO_reg_t *pGPIO,uint8_t pinNum);
void GPIO_writeToOutputPin(GPIO_reg_t *GPIO, uint8_t pinNumber, uint8_t ValueToWrite);

/*
 * read and write Port
 */
uint16_t GPIO_readFrmInputPort(GPIO_reg_t *pGPIO);
void GPIO_writeToOutputPort(GPIO_reg_t* pGPIO, uint16_t ValueToWrite);

/*
 * Toggle pin
 */
void GPIO_ToggleOutputPin(GPIO_reg_t *pGPIO, uint8_t PinNumber);

/*
 * GPIO interrupt configuration and handling
 *
pos|pri|	|type of pri|  |acronym|		|description|       |Address|
6 	13 		settable 		EXTI0 		 	Line0 interrupt 	0x00000058
7 	14 		settable 		EXTI1 		 	Line1 interrupt 	0x0000005C
8 	15 		settable 		EXTI2 		 	Line2 interrupt 	0x00000060
9 	16 		settable 		EXTI3 		 	Line3 interrupt 	0x00000064
10 	17 		settable 		EXTI4 		 	Line4 interrupt 	0x00000068

 */
void GPIO_IRQConfig(InterruptHandle * pIntHandle);
void GPIO_EnableInterrupt(__vo uint8_t IRQNum);
void GPIO_DisableInterrupt(__vo uint8_t IRQNum);
void GPIO_IRQHandling(uint8_t PinNumber);


#endif /* INC_STM32F407XX_GPIO_DRV_H_ */
