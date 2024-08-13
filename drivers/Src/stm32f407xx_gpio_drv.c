/*
 * stm32f407xx_gpio_drv.c
 *
 *  Created on: 13-Jan-2024
 *      Author: romsh
 */



#include "stm32f407xx_gpio_drv.h"

static void GPIO_syscfgExti(uint8_t port, uint8_t PinNum);

/****************************************************************************************************
 * @fn      		  - GPIO_PeriClockControl
 *
 * @brief             - This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void GPIO_PeriClkCtrl(GPIO_reg_t *pGPIOx, uint8_t IsEnabled )
{

	if(IsEnabled == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			EN_PCLK_GPIOA();
		}

		if(pGPIOx == GPIOB)
		{
			EN_PCLK_GPIOB();
		}

		if(pGPIOx == GPIOC)
		{
			EN_PCLK_GPIOC();
		}

		if(pGPIOx == GPIOD)
		{
			EN_PCLK_GPIOD();
		}

		if(pGPIOx == GPIOE)
		{
			EN_PCLK_GPIOE();
		}

		if(pGPIOx == GPIOF)
		{
			EN_PCLK_GPIOF();
		}

		if(pGPIOx == GPIOG)
		{
			EN_PCLK_GPIOG();
		}

		if(pGPIOx == GPIOH)
		{
			EN_PCLK_GPIOH();
		}

		if(pGPIOx == GPIOI)
		{
			EN_PCLK_GPIOI();
		}

	}else
	{
		if(pGPIOx == GPIOA)
		{
			DIS_PCLK_GPIOA();
		}

		if(pGPIOx == GPIOB)
		{
			DIS_PCLK_GPIOB();
		}

		if(pGPIOx == GPIOC)
		{
			DIS_PCLK_GPIOC();
		}

		if(pGPIOx == GPIOD)
		{
			DIS_PCLK_GPIOD();
		}

		if(pGPIOx == GPIOE)
		{
			DIS_PCLK_GPIOE();
		}

		if(pGPIOx == GPIOF)
		{
			DIS_PCLK_GPIOF();
		}

		if(pGPIOx == GPIOG)
		{
			DIS_PCLK_GPIOG();
		}

		if(pGPIOx == GPIOH)
		{
			DIS_PCLK_GPIOH();
		}

		if(pGPIOx == GPIOI)
		{
			DIS_PCLK_GPIOI();
		}
	}
}



/*********************************************************************
 * @fn      		  - GPIO_Init
 *
 * @brief             - This function initializes GPIOx registers
 *
 * @param[in]         - Handle to GPIO port
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp = 0;
	temp= 2*pGPIOHandle->GPIOPinConfig.GPIO_PinNum;

	//enable the clock to GPIO
	GPIO_PeriClkCtrl(pGPIOHandle->pGPIO,ENABLE);

	//configure mode of the GPIO
	if(pGPIOHandle->GPIOPinConfig.GPIOmode<=ANALOG_MODE)
	{
		//configuring the pin mode
		if(pGPIOHandle->GPIOPinConfig.GPIOmode==INPUT_MODE)
		{
			//set the input mode
			pGPIOHandle->pGPIO->MODER &= ~(3U<<temp);
		}
		else if(pGPIOHandle->GPIOPinConfig.GPIOmode == OUTPUT_MODE)
		{
			//set the output mode
			pGPIOHandle->pGPIO->MODER |= (OUTPUT_MODE<<temp);
		}
		else if(pGPIOHandle->GPIOPinConfig.GPIOmode == ALT_FUNC)
		{
			//set the alternate function mode
			pGPIOHandle->pGPIO->MODER |= (ALT_FUNC<<temp);

			//to find the AFRL or AFRH register for given pin number
			uint32_t temp1 = pGPIOHandle->GPIOPinConfig.GPIO_PinNum/8;

			//to find the correct bit positions for  given pin number
			uint32_t temp2 = pGPIOHandle->GPIOPinConfig.GPIO_PinNum%8;


			//for pin 0..7
			if(temp1==0)
			{
				if(pGPIOHandle->GPIOPinConfig.GPIOAltFunc==AF0)
				{
					pGPIOHandle->pGPIO->AFRL = pGPIOHandle->pGPIO->AFRL&(~(0xF<<(4*temp2)));
				}
				else
				{
					pGPIOHandle->pGPIO->AFRL = pGPIOHandle->pGPIO->AFRL|(pGPIOHandle->GPIOPinConfig.GPIOAltFunc << 4*temp2);
				}
			}
			else if(temp1==1)
			{	//for pin 8..15
				if(pGPIOHandle->GPIOPinConfig.GPIOAltFunc==AF0)
				{
					pGPIOHandle->pGPIO->AFRH = pGPIOHandle->pGPIO->AFRH&(~(0xF<<(4*temp2)));
				}
				else
				{
					pGPIOHandle->pGPIO->AFRH = pGPIOHandle->pGPIO->AFRH|(pGPIOHandle->GPIOPinConfig.GPIOAltFunc << 4*temp2);
				}
			}
		}
		else
		{
			pGPIOHandle->pGPIO->MODER |= (ANALOG_MODE<<temp);
		}

		//configure the pull up or pull down
		if(pGPIOHandle->GPIOPinConfig.GPIOPullUpPullDown == NO_PU_NO_PD)//if the pin mode is input
			{
				pGPIOHandle->pGPIO->PUPDR &= ~(3U<<temp);
			}
		else{
				pGPIOHandle->pGPIO->PUPDR |=(pGPIOHandle->GPIOPinConfig.GPIOPullUpPullDown<<temp);
			}
		//configuring output type
		if(pGPIOHandle->GPIOPinConfig.GPIOOutputType== OP_TYPE_PP)
			{
				pGPIOHandle->pGPIO->OTYPER&=~(1<<pGPIOHandle->GPIOPinConfig.GPIO_PinNum);
			}
		else
			{
				pGPIOHandle->pGPIO->OTYPER|=(OP_TYPE_OD<<pGPIOHandle->GPIOPinConfig.GPIO_PinNum);
			}

		//configuring speed of the GPIO
		if(pGPIOHandle->GPIOPinConfig.GPIOspeed == LOW_SPEED)
			{
				pGPIOHandle->pGPIO->OSPEEDR  = pGPIOHandle->pGPIO->OSPEEDR&(~(3<<temp));
			}
			else
			{
				pGPIOHandle->pGPIO->OSPEEDR |= (pGPIOHandle->GPIOPinConfig.GPIOspeed<<temp);
			}
	}
	//for interrupt mode
	else
	{
		pGPIOHandle->pGPIO->MODER &= ~(3U<<temp);

		if(pGPIOHandle->GPIOPinConfig.GPIOPullUpPullDown == NO_PU_NO_PD)
			{
				pGPIOHandle->pGPIO->PUPDR &= ~(3U<<temp);
			}
		else{
				pGPIOHandle->pGPIO->PUPDR |=(pGPIOHandle->GPIOPinConfig.GPIOPullUpPullDown<<temp);
			}

		//enable the clock to EXTI
		EN_PCLK_SYSCFG();

		//un-mask the interrupt line
		(EXTI)->EXTI_IMR = (EXTI)->EXTI_IMR |(1<<pGPIOHandle->GPIOPinConfig.GPIO_PinNum);

		if(pGPIOHandle->GPIOPinConfig.GPIOmode == GPIO_INT_FT)
		{
			(EXTI)->EXTI_FTSR = (EXTI)->EXTI_FTSR | (1<<pGPIOHandle->GPIOPinConfig.GPIO_PinNum);
		}
		else if(pGPIOHandle->GPIOPinConfig.GPIOmode == GPIO_INT_RT)
		{
			(EXTI)->EXTI_RTSR = (EXTI)->EXTI_RTSR | (1<<pGPIOHandle->GPIOPinConfig.GPIO_PinNum);
		}
		else
		{
			//triggers interrupt on both rising and falling edge
			(EXTI)->EXTI_FTSR = (EXTI)->EXTI_FTSR | (1<<pGPIOHandle->GPIOPinConfig.GPIO_PinNum);
			(EXTI)->EXTI_RTSR = (EXTI)->EXTI_RTSR | (1<<pGPIOHandle->GPIOPinConfig.GPIO_PinNum);
		}

		//syscfg settings to activate the given  GPIO pin so that it can trigger the interrupt
		GPIO_syscfgExti(pGPIOHandle->GPIOPinConfig.GPIOPort, pGPIOHandle->GPIOPinConfig.GPIO_PinNum);
	}

}

/*********************************************************************
 * @fn      		  - GPIO_DeInit
 *
 * @brief             - This function resets GPIOx registers
 *
 * @param[in]         - Handle to GPIO port
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void GPIO_DeInit(GPIO_reg_t* pGPIO)
{
	if(pGPIO==GPIOA)
	{
		//make it 0 to reset the registers
		RESET_GPIOA();
		//make it 1 to release the registers from reset state
		SET_GPIOA();
	}
	else if(pGPIO==GPIOB)
	{
		RESET_GPIOB();
		SET_GPIOB();
	}
	else if(pGPIO==GPIOC)
	{
		RESET_GPIOC();
		SET_GPIOC();
	}
	else if(pGPIO==GPIOD)
	{
		RESET_GPIOD();
		SET_GPIOD();
	}
	else if(pGPIO==GPIOE)
	{
		RESET_GPIOE();
		SET_GPIOE();
	}
	else if(pGPIO==GPIOF)
	{
		RESET_GPIOF();
		SET_GPIOF();
	}
	else if(pGPIO==GPIOG)
	{
		RESET_GPIOG();
		SET_GPIOG();
	}
	else if(pGPIO==GPIOH)
	{
		RESET_GPIOH();
		SET_GPIOH();
	}
	else if(pGPIO==GPIOI)
	{
		RESET_GPIOI();
		SET_GPIOI();
	}
}



/*********************************************************************
 * @fn      		  - GPIO_readFrmInputPin
 *
 * @brief             - This function reads from the given input pin of given GPIO port
 *
 * @param[in]         - base address of GPIO port
 * @param[in]         - pin number of the GPIO port
 * @param[in]         -
 *
 * @return            -  0 or 1
 *
 * @Note              -  none

 */
uint8_t GPIO_readFrmInputPin(GPIO_reg_t *pGPIO, uint8_t pinNum)
{
	uint8_t value = ((pGPIO->IDR>>pinNum)&(1<<0));
	return value;
}

/*********************************************************************
 * @fn      		  - GPIO_writeToOutputPin
 *
 * @brief             - This function writes to GPIO pin of given GPIO port
 *
 * @param[in]         - base address of port
 * @param[in]         - pin number of the port
 * @param[in]         - 0 or 1
 *
 * @return            -  none
 *
 * @Note              -  none

 */

void GPIO_writeToOutputPin(GPIO_reg_t *pGPIO, uint8_t pinNumber, uint8_t ValueToWrite)
{
	if(ValueToWrite==0)
	{
		pGPIO->ODR &= ~(pinNumber<<ENABLE);
	}
	else
	{
		pGPIO->ODR |= (pinNumber<<ENABLE);
	}
}

/*********************************************************************
 * @fn      		  - GPIO_readFrmInputPort
 *
 * @brief             - This function reads from the given GPIO port
 *
 * @param[in]         - base address of GPIO port
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  data value read
 *
 * @Note              -  none

 */

uint16_t GPIO_readFrmInputPort(GPIO_reg_t *pGPIO)
{
	uint16_t value =(pGPIO->IDR);
	return value;
}

/*********************************************************************
 * @fn      		  - GPIO_writeToOutputPin
 *
 * @brief             - This function writes to GPIO port
 *
 * @param[in]         - base address of port
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */

void GPIO_writeToOutputPort(GPIO_reg_t* pGPIO, uint16_t ValueToWrite)
{
	pGPIO->ODR = ValueToWrite;
}

/*********************************************************************
 * @fn      		  - GPIO_ToggleOutputPin
 *
 * @brief             - This function toggles the pin of the given port
 *
 * @param[in]         - base address of port
 * @param[in]         - pin number of the port
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void GPIO_ToggleOutputPin(GPIO_reg_t *pGPIO, uint8_t PinNumber)
{
	pGPIO->ODR = pGPIO->ODR ^ (1<<PinNumber);
}

/*********************************************************************
 * @fn      		  - GPIO_IRQConfig
 *
 * @brief             - This function configures the interrupt in NVIC
 *
 * @param[in]         - interrupt number
 * @param[in]         - interrupt priority
 * @param[in]         - DISABLE or ENABLE
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void GPIO_IRQConfig(InterruptHandle * pIntHandle)
{

	if((pIntHandle->IRQNum>=EXTI0_IRQ&&pIntHandle->IRQNum<=EXTI4_IRQ)||pIntHandle->IRQNum==EXTI9_5_IRQ||pIntHandle->IRQNum==EXTI15_10_IRQ)
	{
			if(pIntHandle->IsEnable==TRUE)
			{
				//enable the NVIC interrupt line
				CortexM4EnableIRQ(pIntHandle->IRQNum);

				//clear the NVIC pending register
				//CortexM4ClearIntPend(pIntHandle->IRQNum);

				//set the Priority of the interrupt
				CortexM4SetIntPriority(pIntHandle->IRQNum,pIntHandle->IntPri);
			}
			else
			{
				//disable the interrupt in NVIC
				CortexM4DisableIRQ(pIntHandle->IRQNum);

				//clear the NVIC pending status
				//CortexM4ClearIntPend(pIntHandle->IRQNum);
			}
	}

	return;
}

/*********************************************************************
 * @fn      		  - GPIO_EnableInterrupt
 *
 * @brief             - This function enables given interrupt no.
 *
 * @param[in]         - pin number of the port
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void GPIO_EnableInterrupt(__vo uint8_t IRQNum)
{
	if((IRQNum>=EXTI0_IRQ&&IRQNum<=EXTI4_IRQ)||IRQNum==EXTI9_5_IRQ||IRQNum==EXTI15_10_IRQ){

		CortexM4EnableIRQ(IRQNum);
	}
}

/*********************************************************************
 * @fn      		  - GPIO_DisableInterrupt
 *
 * @brief             - This function disables given interrupt no.
 *
 * @param[in]         - pin number of the port
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void GPIO_DisableInterrupt(__vo uint8_t IRQNum)
{
	if((IRQNum>=EXTI0_IRQ&&IRQNum<=EXTI4_IRQ)||IRQNum==EXTI9_5_IRQ||IRQNum==EXTI15_10_IRQ){

		CortexM4DisableIRQ(IRQNum);
	}
}


/*********************************************************************
 * @fn      		  - GPIO_IRQHandling
 *
 * @brief             - This function handles the GPIO interrupt
 *
 * @param[in]         - pin number of the port
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void GPIO_IRQHandling(uint8_t PinNumber)
{
	//check if the interrupt is pending on given EXTI line
	if((((EXTI)->EXTI_PR>>PinNumber)&SET)==TRUE)
	{
		//clear the pending interrupt indicating that interrupt is received and processed
		(EXTI)->EXTI_PR|=(1<<PinNumber);
	}
	return;
}

/*********************************************************************
 * @fn      		  - GPIO_syscfgExti
 *
 * @brief             - This function sets the SYSCFG_EXTICRx reg bits for given GPIO port and pin
 *
 * @param[in]         - GPIO port
 * @param[in]         -	GPIO pin
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  static function

 */
static void GPIO_syscfgExti(uint8_t port, uint8_t PinNum)
{
	uint8_t Bitpos = (PinNum%4)*4;
	uint8_t regNo  = PinNum/4;

	//clear the bit first
	(SYSCFG)->SYSCFG_EXTICR[regNo] &=~(0xff<<Bitpos);
	//set
	(SYSCFG)->SYSCFG_EXTICR[regNo] |= (port<<Bitpos);


//	//for pin0-3
//	if(PinNum>=PIN0&&PinNum<=PIN3)
//	{
//		//clear
//		SYSCFG->SYSCFG_EXTICR1 &=~(0xff<<Bitpos);
//		//set
//		SYSCFG->SYSCFG_EXTICR1 |= (port<<Bitpos);
//	}
//	if(PinNum>=PIN4&&PinNum<=PIN7)
//	{
//		//clear
//		SYSCFG->SYSCFG_EXTICR2 &=~(0xff<<Bitpos);
//		//set
//		SYSCFG->SYSCFG_EXTICR2 |= (port<<Bitpos);
//	}
//	if(PinNum>=PIN8&&PinNum<=PIN11)
//	{
//		//clear
//		SYSCFG->SYSCFG_EXTICR3 &=~(0xff<<Bitpos);
//		//set
//		SYSCFG->SYSCFG_EXTICR3 |= (port<<Bitpos);
//	}
//	if(PinNum>=PIN12&&PinNum<=PIN15)
//	{
//		//clear
//		SYSCFG->SYSCFG_EXTICR4 &=~(0xff<<Bitpos);
//		//set
//		SYSCFG->SYSCFG_EXTICR4 |= (port<<Bitpos);
//	}
}
