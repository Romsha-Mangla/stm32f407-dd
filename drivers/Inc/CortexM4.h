/*
 * CortexM4.h
 *
 *  Created on: 03-Feb-2024
 *      Author: romsha
 */

#ifndef INC_CORTEXM4_H_
#define INC_CORTEXM4_H_

#include<stdint.h>

//NVIC registers
#define __vo				volatile

#define TOTAL_INTERRUPTS		240
#define SMALLEST_PRIORITY		16

//generic macros
#define EN 		1
#define DIS 	0

typedef struct{
	__vo uint32_t STCSR; //RW 0xE000E010 0x00000000 SysTick Control and Status Register
	__vo uint32_t STRVR; //RW 0xE000E014 Unknown SysTick Reload Value Register
	__vo uint32_t STCVR; //RW 0xE000E018 clear Unknown SysTick Current Value Register
	__vo uint32_t STCR;	 //RO 0xE000E01C STCALIB SysTick Calibration Value Register
}SysTick_reg_t;

#define SYSTICK					((SysTick_reg_t*)0xE000E010)

//interrupt set registers
typedef struct{
	__vo uint32_t ISER[8];	//IRQ 0-239
}IntSet_reg_t;

#define NVIC_ISER				((IntSet_reg_t*)0xE000E100)

//interrupt clear registers
typedef struct{
	__vo uint32_t ICER[8];
}IntClear_reg_t;

#define NVIC_ICER				((IntClear_reg_t*)0xE000E180)

//interrupt set pending register
typedef struct{
	__vo uint32_t ISPR[8];

}IntPendSet_reg_t;

#define NVIC_ISPR				((IntPendSet_reg_t*)0XE000E200)

//interrupt clear pending register
typedef struct{
	__vo uint32_t ICPR[8];
}IntPendClear_reg_t;

#define NVIC_ICPR				((IntPendClear_reg_t*)0XE000E280)

//Interrupt priority register
typedef struct{
	__vo uint32_t IPR[60];
}IntSetPri_reg_t;

#define NVIC_IPR				((IntSetPri_reg_t*)0xE000E400)



/*
 * NVIC API
 */
void CortexM4EnableIRQ(uint32_t IRQNum);

void CortexM4DisableIRQ(uint32_t IRQNum);

uint8_t CortexM4GetIntEnableStatus(uint32_t IRQNum);

void CortexM4SetIntPriority(uint32_t IRQNum, uint8_t Pri);

uint8_t CortexM4GetIntPriority(uint32_t IRQNum);

void CortexM4SetIntPend(uint32_t IRQNum);

void CortexM4ClearIntPend(uint32_t IRQNum);

uint8_t CortexM4GetIntPendStatus(uint32_t IRQNum);



#endif /* INC_CORTEXM4_H_ */
