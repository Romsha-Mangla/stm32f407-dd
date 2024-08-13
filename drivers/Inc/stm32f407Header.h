/*
 * stm32f407Header.h
 *
 *  Created on: Jan 10, 2024
 *      Author: Romsha
 */

#ifndef DRIVERS_INC_STM32F407HEADER_H_
#define DRIVERS_INC_STM32F407HEADER_H_

#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <assert.h>
#include "CortexM4.h"

#define __vo volatile //short form of volatile keyword

//describe the base address of the memories - flash and SRAM
//the memory address must be unsigned type so put 'U' at the end of the address below
#define FLASH_BASEADDR						0x08000000UL //( 1Mb divided into sectors)
#define SRAM1_BASEADDR						0x20000000UL //( 112Kb )
#define SRAM2_BASEADDR						0x2001C000 	// ( 16kb )
#define SRAM								SRAM1_BASEADDR
#define ROM_BASEADDR						0x1FFF0000UL //( 29Kb )also known as system mem

//Bus domain
#define APB1PERIPH_BASEADDR					0x40000000UL
#define APB2PERIPH_BASEADDR					0x40010000UL
#define AHB1PERIPH_BASEADDR					0x40020000UL
#define AHB2PERIPH_BASEADDR					0x50000000UL
#define AHB3PERIPH_BASEADDR					0xA0000000UL

//base address of peripheral registers

//GPIO
#define GPIOA_BASEADDR						(AHB1PERIPH_BASEADDR+ 0x0000U)
#define GPIOB_BASEADDR						(AHB1PERIPH_BASEADDR+ 0x0400U)
#define GPIOC_BASEADDR						(AHB1PERIPH_BASEADDR+ 0x0800U)
#define GPIOD_BASEADDR						(AHB1PERIPH_BASEADDR+ 0x0C00U)
#define GPIOE_BASEADDR						(AHB1PERIPH_BASEADDR+ 0x1000U)
#define GPIOF_BASEADDR						(AHB1PERIPH_BASEADDR+ 0x1400U)
#define GPIOG_BASEADDR						(AHB1PERIPH_BASEADDR+ 0x1800U)
#define GPIOH_BASEADDR						(AHB1PERIPH_BASEADDR+ 0x1C00U)
#define GPIOI_BASEADDR						(AHB1PERIPH_BASEADDR+ 0x2000U)
//RCC
#define RCC_BASEADDR						(AHB1PERIPH_BASEADDR+ 0x3800U)
//USART
#define USART1_BASEADDR						(APB2PERIPH_BASEADDR+ 0x1000U)
#define USART2_BASEADDR						(APB1PERIPH_BASEADDR+ 0x4400U)
#define USART3_BASEADDR						(APB1PERIPH_BASEADDR+ 0x4800U)
#define UART4_BASEADDR						(APB1PERIPH_BASEADDR+ 0x4C00U)
#define UART5_BASEADDR						(APB1PERIPH_BASEADDR+ 0x5000U)
#define USART6_BASEADDR						(APB2PERIPH_BASEADDR+ 0x1400U)
#define UART7_BASEADDR						(APB1PERIPH_BASEADDR+ 0x7800U)
#define UART8_BASEADDR						(APB1PERIPH_BASEADDR+ 0x7C00U)
//SPI
#define SPI1_BASEADDR						(APB2PERIPH_BASEADDR+ 0x3000U)
#define SPI2_BASEADDR						(APB1PERIPH_BASEADDR+ 0x3800U) //also I2S2
#define SPI3_BASEADDR						(APB1PERIPH_BASEADDR+ 0x3C00U) //also I2S3
//#define SPI4_BASEADDR						(APB2PERIPH_BASEADDR+ 0x3400U) //not part of uC, address range is reserved
//#define SPI5_BASEADDR						(APB2PERIPH_BASEADDR+ 0x5000U)	//not part of uC, address range is reserved
//#define SPI6_BASEADDR						(APB2PERIPH_BASEADDR+ 0x5400U)	//not part of uC, address range is reserved

//I2C
#define I2C1_BASEADDR						(APB1PERIPH_BASEADDR+ 0x5400U)
#define I2C2_BASEADDR						(APB1PERIPH_BASEADDR+ 0x5800U)
#define I2C3_BASEADDR						(APB1PERIPH_BASEADDR+ 0x5C00U)

#define SYSCFG_BASEADDR						(APB2PERIPH_BASEADDR+ 0x3800U)
#define EXTI_BASEADDR						(APB2PERIPH_BASEADDR+ 0x3C00U)

typedef struct {
	__vo uint32_t EXTI_IMR;	//interrupt mask 0 means interrupt is masked and 1 means not masked 00
	__vo uint32_t EXTI_EMR;	//event mask register												04
	__vo uint32_t EXTI_RTSR;//rising edge trigger												08
	__vo uint32_t EXTI_FTSR;//falling trigger selection 0 means disabled and 1 means FT enabled	0C
	__vo uint32_t EXTI_SWIER;//software interrupt on line x										10
	__vo uint32_t EXTI_PR;//Pending register 1 means selected trigger request occured			14
} EXTI_reg_t;

#define EXTI								(EXTI_reg_t*)(EXTI_BASEADDR)

typedef struct {
	__vo uint32_t SYSCFG_MEMRMP;			//configuration of memory remap	00
	__vo uint32_t SYSCFG_PMC;				//peripheral mode cfg register  04
	__vo uint32_t SYSCFG_EXTICR[4];			//ext int cfg register	pin0 s	08
	__vo uint32_t SYSCFG_CMPCR;				//								20
} SYSCFG_reg_t;

#define SYSCFG								(SYSCFG_reg_t*)(SYSCFG_BASEADDR)

typedef struct {
	__vo uint32_t MODER; 					//mode register for GPIO 	00
	__vo uint32_t OTYPER; 					//output type				04
	__vo uint32_t OSPEEDR; 					//output speed register		08
	__vo uint32_t PUPDR;					//push pull					0C
	__vo uint32_t IDR;						//input data register		10
	__vo uint32_t ODR;						//output data register		14
	__vo uint32_t BSRR;						//bit set/reset register	18
	__vo uint32_t LCKR;						//config lock register		1C
	__vo uint32_t AFRL;						//alternate function select	20
	__vo uint32_t AFRH;						//alternate function select	24
} GPIO_reg_t;

#define GPIOA								(GPIO_reg_t*)GPIOA_BASEADDR
#define GPIOB								(GPIO_reg_t*)GPIOB_BASEADDR
#define GPIOC								(GPIO_reg_t*)GPIOC_BASEADDR
#define GPIOD								(GPIO_reg_t*)GPIOD_BASEADDR
#define GPIOE								(GPIO_reg_t*)GPIOE_BASEADDR
#define GPIOF								(GPIO_reg_t*)GPIOF_BASEADDR
#define GPIOG								(GPIO_reg_t*)GPIOG_BASEADDR
#define GPIOH								(GPIO_reg_t*)GPIOH_BASEADDR
#define GPIOI								(GPIO_reg_t*)GPIOI_BASEADDR

typedef struct {
	__vo uint32_t RCC_CR;					//clock register			00
	__vo uint32_t RCC_PLLCFGR;				//PLL config register		04
	__vo uint32_t RCC_CFGR;					//rcc configuration reg		08
	__vo uint32_t RCC_CIR;					//							0C
	__vo uint32_t RCC_AHB1RSTR;				// abh1 prphl reset			10
	__vo uint32_t RCC_AHB2RSTR;				//							14
	__vo uint32_t RCC_AHB3RSTR;				//							18
	__vo uint32_t reserved1;				//reserved					1C
	__vo uint32_t RCC_APB1RSTR;				//rcc APB1 reset			20
	__vo uint32_t RCC_APB2RSTR;				//							24
	__vo uint32_t reserved2;				//reserved					28
	__vo uint32_t reserved3;				//reserved					2C
	__vo uint32_t RCC_AHB1ENR;				//en clk for peripheral		30
	__vo uint32_t RCC_AHB2ENR;				//							34
	__vo uint32_t RCC_AHB3ENR;				//							38
	__vo uint32_t reserved4;				//reserved					3C
	__vo uint32_t RCC_APB1ENR;				//							40
	__vo uint32_t RCC_APB2ENR;				//							44
	__vo uint32_t reserved5;				//reserved					48
	__vo uint32_t reserved6;				//reserved					4C
	__vo uint32_t RCC_AHB1LPENR;			//en lp clk					50
	__vo uint32_t RCC_AHB2LPENR;			//							54
	__vo uint32_t RCC_AHB3LPENR;			//							58
	__vo uint32_t reserved7;				//reserved					5C
	__vo uint32_t RCC_APB1LPENR;			//							60
	__vo uint32_t RCC_APB2LPENR;			//							64
	__vo uint32_t reserved8;				//reserved					68
	__vo uint32_t reserved9;				//reserved					6C
	__vo uint32_t RCC_BDCR;					//							70
	__vo uint32_t RCC_CSR;					//							74
	__vo uint32_t reserved10;				//reserved					78
	__vo uint32_t reserved11;				//reserved					7C
	__vo uint32_t RCC_SSCGR;				//							80
	__vo uint32_t RCC_PLLI2SCFGR;			//							84
} RCC_reg_t;

#define RCC										(RCC_reg_t*)RCC_BASEADDR

/* macro to enable clock for GPIO port A*/
#define EN_PCLK_GPIOA()							((RCC)->RCC_AHB1ENR|=(1<<0))

/* macro to enable clock for GPIO port B*/
#define EN_PCLK_GPIOB()							((RCC)->RCC_AHB1ENR|=(1<<1))

/* macro to enable clock for GPIO port C*/
#define EN_PCLK_GPIOC()							((RCC)->RCC_AHB1ENR|=(1<<2))

/* macro to enable clock for GPIO port D*/
#define EN_PCLK_GPIOD()							((RCC)->RCC_AHB1ENR|=(1<<3))

/* macro to enable clock for GPIO port E*/
#define EN_PCLK_GPIOE()							((RCC)->RCC_AHB1ENR|=(1<<4))

/* macro to enable clock for GPIO port F*/
#define EN_PCLK_GPIOF()							((RCC)->RCC_AHB1ENR|=(1<<5))

/* macro to enable clock for GPIO port G*/
#define EN_PCLK_GPIOG()							((RCC)->RCC_AHB1ENR|=(1<<6))

/* macro to enable clock for GPIO port H*/
#define EN_PCLK_GPIOH()							((RCC)->RCC_AHB1ENR|=(1<<7))

/* macro to enable clock for GPIO port I*/
#define EN_PCLK_GPIOI()							((RCC)->RCC_AHB1ENR|=(1<<8))

//--------------------------------------------------------------------------------------------
/* macro to disable clock for GPIO port A*/
#define DIS_PCLK_GPIOA()						((RCC)->RCC_AHB1ENR&=~(1<<0))

/* macro to disable clock for GPIO port B*/
#define DIS_PCLK_GPIOB()						((RCC)->RCC_AHB1ENR&=~(1<<1))

/* macro to disable clock for GPIO port C*/
#define DIS_PCLK_GPIOC()						((RCC)->RCC_AHB1ENR&=~(1<<2))

/* macro to disable clock for GPIO port D*/
#define DIS_PCLK_GPIOD()						((RCC)->RCC_AHB1ENR&=~(1<<3))

/* macro to disable clock for GPIO port E*/
#define DIS_PCLK_GPIOE()						((RCC)->RCC_AHB1ENR&=~(1<<4))

/* macro to disable clock for GPIO port F*/
#define DIS_PCLK_GPIOF()						((RCC)->RCC_AHB1ENR&=~(1<<5))

/* macro to disable clock for GPIO port G*/
#define DIS_PCLK_GPIOG()						((RCC)->RCC_AHB1ENR&=~(1<<6))

/* macro to disable clock for GPIO port H*/
#define DIS_PCLK_GPIOH()						((RCC)->RCC_AHB1ENR&=~(1<<7))

/* macro to disable clock for GPIO port I*/
#define DIS_PCLK_GPIOI()						((RCC)->RCC_AHB1ENR&=~(1<<8))

/* macros to reset GPIOx registers */

#define RESET_GPIOA()							((RCC)->RCC_AHB1RSTR = (RCC)->RCC_AHB1RSTR |(1<<0))
#define RESET_GPIOB()							((RCC)->RCC_AHB1RSTR=  (RCC)->RCC_AHB1RSTR |(1<<1))
#define RESET_GPIOC()							((RCC)->RCC_AHB1RSTR=  (RCC)->RCC_AHB1RSTR |(1<<2))
#define RESET_GPIOD()							((RCC)->RCC_AHB1RSTR=  (RCC)->RCC_AHB1RSTR |(1<<3))
#define RESET_GPIOE()							((RCC)->RCC_AHB1RSTR=  (RCC)->RCC_AHB1RSTR |(1<<4))
#define RESET_GPIOF()							((RCC)->RCC_AHB1RSTR=  (RCC)->RCC_AHB1RSTR |(1<<5))
#define RESET_GPIOG()							((RCC)->RCC_AHB1RSTR=  (RCC)->RCC_AHB1RSTR |(1<<6))
#define RESET_GPIOH()							((RCC)->RCC_AHB1RSTR=  (RCC)->RCC_AHB1RSTR |(1<<7))
#define RESET_GPIOI()							((RCC)->RCC_AHB1RSTR=  (RCC)->RCC_AHB1RSTR |(1<<8))

/* macros to set the GPIOx registers */

#define SET_GPIOA()								((RCC)->RCC_AHB1RSTR = (RCC)->RCC_AHB1RSTR &(~(1<<0)))
#define SET_GPIOB()								((RCC)->RCC_AHB1RSTR=  (RCC)->RCC_AHB1RSTR &(~(1<<1)))
#define SET_GPIOC()								((RCC)->RCC_AHB1RSTR=  (RCC)->RCC_AHB1RSTR &(~(1<<2)))
#define SET_GPIOD()								((RCC)->RCC_AHB1RSTR=  (RCC)->RCC_AHB1RSTR &(~(1<<3)))
#define SET_GPIOE()								((RCC)->RCC_AHB1RSTR=  (RCC)->RCC_AHB1RSTR &(~(1<<4)))
#define SET_GPIOF()								((RCC)->RCC_AHB1RSTR=  (RCC)->RCC_AHB1RSTR &(~(1<<5)))
#define SET_GPIOG()								((RCC)->RCC_AHB1RSTR=  (RCC)->RCC_AHB1RSTR &(~(1<<6)))
#define SET_GPIOH()								((RCC)->RCC_AHB1RSTR=  (RCC)->RCC_AHB1RSTR &(~(1<<7)))
#define SET_GPIOI()								((RCC)->RCC_AHB1RSTR=  (RCC)->RCC_AHB1RSTR &(~(1<<8)))

//-------------------------enable clock for USART/UART---------------------------------------------------------------------

#define EN_PCLK_USART1()							((RCC)->RCC_APB2ENR|=(1<<4))
#define EN_PCLK_USART2()							((RCC)->RCC_APB1ENR|=(1<<17))
#define EN_PCLK_USART3()							((RCC)->RCC_APB1ENR|=(1<<18))
#define EN_PCLK_UART4()								((RCC)->RCC_APB1ENR|=(1<<19))
#define EN_PCLK_UART5()								((RCC)->RCC_APB1ENR|=(1<<20))
#define EN_PCLK_USART6()							((RCC)->RCC_APB2ENR|=(1<<5))

//-------------------------disable clock for USART/UART--------------------------------------------------------------------
#define DIS_PCLK_USART1()							((RCC)->RCC_APB2ENR&=~(1<<4))
#define DIS_PCLK_USART2()							((RCC)->RCC_APB1ENR&=~(1<<17))
#define DIS_PCLK_USART3()							((RCC)->RCC_APB1ENR&=~(1<<18))
#define DIS_PCLK_UART4()							((RCC)->RCC_APB1ENR&=~(1<<19))
#define DIS_PCLK_UART5()							((RCC)->RCC_APB1ENR&=~(1<<20))
#define DIS_PCLK_USART6()							((RCC)->RCC_APB2ENR&=~(1<<5))

typedef struct{
__vo uint32_t USART_SR;								//00	reset value 0x000000C0
__vo uint32_t USART_DR;								//04	reset value 0xXXXXXXXX 8bit
__vo uint32_t USART_BRR;							//08	reset value 0x00000000
__vo uint32_t USART_CR1;							//0C	reset value 0x00000000
__vo uint32_t USART_CR2;							//10	reset value 0x00000000
__vo uint32_t USART_CR3;							//14	reset value 0x00000000
__vo uint32_t USART_GTPR;							//18    reset value 0x00000000
}USART_reg_t;

#define USART1										((USART_reg_t*)USART1_BASEADDR)
#define USART2										((USART_reg_t*)USART2_BASEADDR)
#define USART3										((USART_reg_t*)USART3_BASEADDR)
#define UART4										((USART_reg_t*)UART4_BASEADDR)
#define UART5										((USART_reg_t*)UART5_BASEADDR)
#define USART6										((USART_reg_t*)USART6_BASEADDR)

/* macros to reset USARTx registers */

#define RESET_USART1()							((RCC)->RCC_APB2RSTR = (RCC)->RCC_APB2RSTR |(1<<4))
#define RESET_USART2()							((RCC)->RCC_APB1RSTR = (RCC)->RCC_APB1RSTR |(1<<17))
#define RESET_USART3()							((RCC)->RCC_APB1RSTR = (RCC)->RCC_APB1RSTR |(1<<18))
#define RESET_UART4()							((RCC)->RCC_APB1RSTR = (RCC)->RCC_APB1RSTR |(1<<19))
#define RESET_UART5()							((RCC)->RCC_APB1RSTR = (RCC)->RCC_APB1RSTR |(1<<20))
#define RESET_USART6()							((RCC)->RCC_APB2RSTR = (RCC)->RCC_APB2RSTR |(1<<5))

/* macros to set the USARTx registers */

#define SET_USART1()								((RCC)->RCC_APB2RSTR = (RCC)->RCC_APB2RSTR &(~(1<<4)))
#define SET_USART2()								((RCC)->RCC_APB1RSTR=  (RCC)->RCC_APB1RSTR &(~(1<<17)))
#define SET_USART3()								((RCC)->RCC_APB1RSTR=  (RCC)->RCC_APB1RSTR &(~(1<<18)))
#define SET_UART4()									((RCC)->RCC_APB1RSTR=  (RCC)->RCC_APB1RSTR &(~(1<<19)))
#define SET_UART5()									((RCC)->RCC_APB1RSTR=  (RCC)->RCC_APB1RSTR &(~(1<<20)))
#define SET_USART6()								((RCC)->RCC_APB2RSTR = (RCC)->RCC_APB2RSTR &(~(1<<5)))

typedef struct {
	__vo uint32_t SPI_CR1; //SPI control register	 	00 reset value 0x00000000
	__vo uint32_t SPI_CR2; 					//SPI control register		04
	__vo uint32_t SPI_SR; 					//SPI						08
	__vo uint8_t SPI_DR;					//SPI data register			0C
	__vo uint32_t SPI_CRCPR;				//holds CRC polynomial		10
	__vo uint32_t SPI_RXCRCR;				//RX CRC					14
	__vo uint32_t SPI_TXCRCR;				//TXCRC						18
	__vo uint32_t SPI_I2SCFGR;				//SPI I2S config			1C
	__vo uint32_t SPI_I2SPR;				//SPI I2S preset value		20
} SPI_reg_t;

#define SPI1										((SPI_reg_t*)SPI1_BASEADDR)
#define SPI2										((SPI_reg_t*)SPI2_BASEADDR)
#define SPI3										((SPI_reg_t*)SPI3_BASEADDR)

//-------------------------enable clock for SPI----------------------------------------------------------------------------
#define EN_PCLK_SPI1()								((RCC)->RCC_APB2ENR|=(1<<12))
#define EN_PCLK_SPI2()								((RCC)->RCC_APB1ENR|=(1<<14))
#define EN_PCLK_SPI3()								((RCC)->RCC_APB1ENR|=(1<<15))

//-------------------------disable clock for SPI---------------------------------------------------------------------------
#define DIS_PCLK_SPI1()								((RCC)->RCC_APB2ENR&=~(1<<12))
#define DIS_PCLK_SPI2()								((RCC)->RCC_APB1ENR&=~(1<<14))
#define DIS_PCLK_SPI3()								((RCC)->RCC_APB1ENR&=~(1<<15))

typedef struct {
	__vo uint32_t I2C_CR1;								//control register 1
	__vo uint32_t I2C_CR2;								//control register 2
	__vo uint32_t I2C_OAR1;                             //Own Address register 1
	__vo uint32_t I2C_OAR2;								//Own Address register 2
	__vo uint32_t I2C_DR;								//Data Register
	__vo uint32_t I2C_SR1;								//Status register 1
	__vo uint32_t I2C_SR2;								//Status register 2
	__vo uint32_t I2C_CCR;								//clock control register
	__vo uint32_t I2C_TRISE;						//Maximum rise time in SM/FM
} I2C_reg_t;

#define I2C1										((I2C_reg_t*)I2C1_BASEADDR)
#define I2C2										((I2C_reg_t*)I2C2_BASEADDR)
#define I2C3										((I2C_reg_t*)I2C3_BASEADDR)

//-------------------------enable clock for I2C----------------------------------------------------------------------------
#define EN_PCLK_I2C1()								((RCC)->RCC_APB1ENR|=(1<<21))
#define EN_PCLK_I2C2()								((RCC)->RCC_APB1ENR|=(1<<22))
#define EN_PCLK_I2C3()								((RCC)->RCC_APB1ENR|=(1<<23))

//-------------------------disable clock for I2C----------------------------------------------------------------------------
#define DIS_PCLK_I2C1()								((RCC)->RCC_APB1ENR&=~(1<<21))
#define DIS_PCLK_I2C2()								((RCC)->RCC_APB1ENR&=~(1<<22))
#define DIS_PCLK_I2C3()								((RCC)->RCC_APB1ENR&=~(1<<23))

/* macros to reset I2Cx registers */

#define RESET_I2C1()							((RCC)->RCC_APB1RSTR = (RCC)->RCC_APB1RSTR |(1<<21))
#define RESET_I2C2()							((RCC)->RCC_APB1RSTR = (RCC)->RCC_APB1RSTR |(1<<22))
#define RESET_I2C3()							((RCC)->RCC_APB1RSTR = (RCC)->RCC_APB1RSTR |(1<<23))

/* macros to set the I2Cx registers */

#define SET_I2C1()								((RCC)->RCC_APB1RSTR = (RCC)->RCC_APB1RSTR &(~(1<<21)))
#define SET_I2C2()								((RCC)->RCC_APB1RSTR=  (RCC)->RCC_APB1RSTR &(~(1<<22)))
#define SET_I2C3()								((RCC)->RCC_APB1RSTR=  (RCC)->RCC_APB1RSTR &(~(1<<23)))

/* macros to reset SPIx registers */

#define RESET_SPI1()							((RCC)->RCC_APB2RSTR = (RCC)->RCC_APB2RSTR |(1<<12))
#define RESET_SPI2()							((RCC)->RCC_APB1RSTR = (RCC)->RCC_APB1RSTR |(1<<14))
#define RESET_SPI3()							((RCC)->RCC_APB1RSTR = (RCC)->RCC_APB1RSTR |(1<<15))

/* macros to set the SPIx registers */

#define SET_SPI1()								((RCC)->RCC_APB2RSTR = (RCC)->RCC_APB2RSTR &(~(1<<12)))
#define SET_SPI2()								((RCC)->RCC_APB1RSTR=  (RCC)->RCC_APB1RSTR &(~(1<<14)))
#define SET_SPI3()								((RCC)->RCC_APB1RSTR=  (RCC)->RCC_APB1RSTR &(~(1<<15)))

//-------------------------enable clock for syscfg-----------------------------------------------------------------------------
//#define _PCLK_SYSCGF()							(RCC->RCC_APB2ENR|=(1<<14)) // was missing brackets around RCC that is why it was giving error of invalid type for '->'

//------------------------disable clock for syscfg-------------------------------------------------------------------------
#define EN_PCLK_SYSCFG()							((RCC)->RCC_APB2ENR|=(1<<14))

/*
 * IRQ number connected to EXTI line
 */
#define PVD_IRQ					1			//PVD through EXTI line detection interrupt
#define TIMESTAMP_IRQ			2			//Tamper and TimeStamp interrupts through the EXTI line
#define RTC_WAKEUP_IRQ			3			//RTC Wakeup interrupt through the EXTI line
#define RTC_ALARM_IRQ			41			//RTC alarm
#define USB_OTG_FS_WAKEUP_IRQ	42			//USB On-The-Go FS Wakeup through EXTI line interrupt
#define ETH_WAKEUP_IRQ			62			//Ethernet Wakeup through EXTI line interrupt
#define USB_OTG_HS_WAKEUP_IRQ	76			//USB On The Go HS Wakeup through EXTI line interrupt

/*
 * IRQ line connected to EXTI(GPIO)
 * each EXTI line belong to the corresponding GPIO pin of every port A-I
 * so for example PIN1 of port will have its interrupt line mapped to EXTI1
 */
#define EXTI0_IRQ		6			//EXTI line 0 interrupt
#define EXTI1_IRQ		7			//EXTI line 1 interrupt
#define EXTI2_IRQ		8			//EXTI line 2 interrupt
#define EXTI3_IRQ		9			//EXTI line 3 interrupt
#define EXTI4_IRQ		10			//EXTI line 4 interrupt
#define EXTI9_5_IRQ		23			//EXTI line 9:5 interrupt
#define EXTI15_10_IRQ	40			//EXTI line 15:10 interrupt

/*
 * IRQ line connected to USART
 */
#define USART1_IRQ		37
#define USART2_IRQ		38
#define USART3_IRQ		39
#define UART4_IRQ		52
#define UART5_IRQ		53
#define USART6_IRQ		71

/*
 * IRQ line connected to SPI
 */
#define SPI1_IRQ		35
#define SPI2_IRQ		36
#define SPI3_IRQ		51

/*
 * IRQ line connected to I2C
 */

#define I2C1_EV			31
#define I2C1_ER			32
#define I2C2_EV			33
#define I2C2_ER			34
#define I2C3_EV			72
#define I2C3_ER			73

//generic macros
#define ENABLE 		1U
#define DISABLE 	0U
#define SET 		ENABLE
#define RESET		DISABLE
#define GPIO_SET 	ENABLE
#define GPIO_RESET	DISABLE
#define TRUE		ENABLE
#define FALSE		DISABLE

/*
 * macro functions
 */
#define READ_BIT_VALUE(reg,bit)						((reg>>bit)&1 )
#define SET_BIT(reg,bit)							(reg|=(1<<bit))
#define CLEAR_BIT(reg,bit)							(reg&=(~(1<<bit)))

/*
 * system clock source
 */
#define  CS_HSI 				0
#define  CS_HSE					1
#define  CS_PLL					2
#define  CLOCK_SOURCE_BITMASK	((1<<3)|(1<<2))
#define  CLOCK_APB1_BITMASK		((1<<12)|(1<<11)|(1<<10))
#define  CLOCK_APB2_BITMASK     ((1<<15)|(1<<14)|(1<<13))
#define  CLOCK_AHB_BITMASK		((1<<7)|(1<<6)|(1<<5)|(1<<4))

/*
 * RCC CFGR bit fields
 */
#define SWS				2				//bit 3:2 system clock switch status
#define PPRE1			10				//bit 12:10 APB low speed prescaler
#define PPRE2			13				//bit 13:15 APB high speed prescaler
#define ISAPB1DIV		12				//if bit 12 is 0, it means APB clock is not divided
#define HPRE			4				//bit 7:4 AHB prescaler
#define ISAHBDIV		7				//if bit 7 is 0, it means AHB clock is not divided

//error macros
#define I2C_INTERRUPT_CONFIG_ERROR			0xA5
#define SPI_INTERRUPT_CONFIG_ERROR			0x5A
#define USART_INTERRUPT_CONFIG_ERROR		0x69

#ifdef APP_DEBUG
//debug log buffer of 1K size
char dbgBuf[1024];
//error strings
#define I2C_INTERRUPT_HANDLE_ERROR			"I2C global message structure cannot be NULL\n\r"
#define USART_INTERRUPT_HANDLE_ERROR		"USART global message structure cannot be NULL\n\r"
#endif

#endif /* DRIVERS_INC_STM32F407HEADER_H_ */

