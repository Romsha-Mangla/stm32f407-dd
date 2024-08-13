/*
 * CortexM4.c
 *
 *  Created on: 03-Feb-2024
 *      Author: romsh
 */


#include "cortexM4.h"

void CortexM4EnableIRQ(uint32_t IRQNum)
{
	uint8_t regNum = IRQNum/32;
	uint8_t bitNum = IRQNum%32;
	if(IRQNum<TOTAL_INTERRUPTS)
	{
		NVIC_ISER->ISER[regNum]|=(1<<bitNum);
	}
}

void CortexM4DisableIRQ(uint32_t IRQNum)
{
	uint8_t regNum = IRQNum/32;
	uint8_t bitNum = IRQNum%32;
	if(IRQNum<TOTAL_INTERRUPTS)
	{
		NVIC_ICER->ICER[regNum]|=(1<<bitNum);
	}
}

uint8_t CortexM4GetIntEnableStatus(uint32_t IRQNum)
{
	uint8_t regNum = IRQNum/32;
	uint8_t bitNum = IRQNum%32;
	uint8_t intStatus = DIS;
	if(IRQNum<TOTAL_INTERRUPTS)
	{
		if(((NVIC_ISER->ISER[regNum]>>bitNum)&EN)==EN)
			intStatus = EN;
	}
	return intStatus;
}

void CortexM4SetIntPriority(uint32_t IRQNum, uint8_t Pri)
{
	uint8_t regNum = IRQNum/4;
	uint8_t bitPos = (IRQNum%4)*8;

	if((IRQNum<TOTAL_INTERRUPTS)&&(Pri<SMALLEST_PRIORITY))
	{
		//shift again by 4 so that priority is present in bit 4:7 as per PM of cortex M4
		NVIC_IPR->IPR[regNum]|=((Pri<<bitPos)<<4);
	}
}

uint8_t CortexM4GetIntPriority(uint32_t IRQNum)
{
	uint8_t regNum = IRQNum/4;
	uint8_t bitPos = (IRQNum%4)*8;
	uint16_t priority = SMALLEST_PRIORITY;

	if(IRQNum<TOTAL_INTERRUPTS)
	{

		priority = (NVIC_IPR->IPR[regNum]>>bitPos)>>4;
	}

	return priority;
}

void CortexM4SetIntPend(uint32_t IRQNum)
{
	uint8_t regNum = IRQNum/32;
	uint8_t bitNum = IRQNum%32;
	if(IRQNum<TOTAL_INTERRUPTS)
	{
		NVIC_ISPR->ISPR[regNum]|=(1<<bitNum);
	}
}

void CortexM4ClearIntPend(uint32_t IRQNum)
{
	uint8_t regNum = IRQNum/32;
	uint8_t bitNum = IRQNum%32;
	if(IRQNum<TOTAL_INTERRUPTS)
	{
		NVIC_ICPR->ICPR[regNum]|=(1<<bitNum);
	}
}

uint8_t CortexM4GetIntPendStatus(uint32_t IRQNum)
{
	uint8_t regNum = IRQNum/32;
	uint8_t bitNum = IRQNum%32;
	uint8_t status = DIS;
	if(IRQNum<TOTAL_INTERRUPTS)
	{
		if(((NVIC_ISPR->ISPR[regNum]>>bitNum)&EN)==EN)
			status = EN;
	}
	return status;
}

