/*
 * Interrupt.h
 *
 *  Created on: 03-Feb-2024
 *      Author: romsh
 */

#ifndef INC_INTERRUPT_H_
#define INC_INTERRUPT_H_

#include "stm32f407Header.h"

typedef struct{
	__vo uint8_t IsEnable;				//interrupt enable-1, interrupt disable-0
	__vo uint8_t IRQNum;				//position of interrupt in NVIC table
	__vo uint8_t IntPri;				// interrupt priority
}InterruptHandle;


#endif /* INC_INTERRUPT_H_ */
