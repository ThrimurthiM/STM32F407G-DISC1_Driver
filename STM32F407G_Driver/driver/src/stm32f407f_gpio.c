/*
 * stm32f407f_gpio.c
 *
 *  Created on: Jan 2, 2021
 *      Author: SL-BLR-10169
 */

#include "stm32f407g_gpio.h"

void GPIO_PeripheralClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi	)
{
	if(EnorDi == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}
		else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}
		else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}
		// TODO: All other GPIO Port
	}
	else
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_DI();
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DI();
		}
		else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_DI();
		}
		else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_DI();
		}
		// TODO: All other GPIO Port
	}
}

void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp = 0;
	// 1. Configure the Mode of GPIO pin
	if(pGPIOHandle->GPIO_PinConfig.GPIO_INTR == GPIO_NO_INTR)
	{
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~( temp | 0x02); //Clearing
		pGPIOHandle->pGPIOx->MODER |= temp; //Setting
		temp = 0;
	}
	else
	{
		//1. Configure the Interrupt type
		if(pGPIOHandle->GPIO_PinConfig.GPIO_INTR == GPIO_INTER_FALLING_EDGE)
		{
			EXTI->EXTI_FTSR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->EXTI_RTSR &= ~(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_INTR == GPIO_INTER_RAISING_EDGE)
		{
			EXTI->EXTI_RTSR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->EXTI_FTSR &= ~(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_INTR == GPIO_INTER_BOTH_EDGE)
		{
			EXTI->EXTI_FTSR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->EXTI_RTSR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_INTR == GPIO_INTER_LOW_LEVEL)
		{

		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_INTR == GPIO_INTER_HIGH_LEVEL)
		{

		}
		//2. Configure the GPIO Port register in SYSCFG_EXTICR
		uint8_t tem1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber/4;
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber%4;
		uint8_t portcode = GPIO_BASE_ADD_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[tem1]= portcode << (temp2 * 4);
		//3. Enable EXTI interrupt delivery using IMR (Interrupt mask Register)
		EXTI->EXTI_IMR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}

	// 2. Configure the Speed
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~( temp | 0x02); //Clearing
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;
	temp = 0;

	// 3. Configure the Pullup/Pulldown
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~( temp | 0x02); //Clearing
	pGPIOHandle->pGPIOx->PUPDR |= temp;
	temp = 0;

	// 4. Configure the output type
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << (1 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OTYPER &= ~( temp | 0x01); //Clearing
	pGPIOHandle->pGPIOx->OTYPER |= temp;
	temp = 0;

	// 5. Configure the alt Functionality
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_ALTERNATE_FUNCTION_MODE)
	{
		temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / GPIO_PIN_NUM_8;
		pGPIOHandle->pGPIOx->AFR[temp] &= ~( 0x0F | (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4* (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % GPIO_PIN_NUM_8))));
		pGPIOHandle->pGPIOx->AFR[temp] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4* (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % GPIO_PIN_NUM_8)));
	}
	// Config Alternate Function
}

void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if(pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	}
	else if(pGPIOx == GPIOB)
	{
		GPIOA_REG_RESET();
	}
	// TODO: All other GPIO Port
}


uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber)
{
	return (uint8_t)((pGPIOx->IDR >> pinNumber) & 0x01);
}

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	return (uint16_t)pGPIOx->IDR;
}

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, GPIO_PinNum_t pinNumber, uint8_t val)
{
	if(val == GPIO_PIN_SET)
	{
		pGPIOx->ODR |= (1 << pinNumber); //Set
	}
	else
	{
		pGPIOx->ODR &= ~(1 << pinNumber); //Reset
	}
}
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t val)
{
	pGPIOx->ODR = val;
}

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, GPIO_PinNum_t pinNumber)
{
	pGPIOx->ODR ^= (1<< pinNumber);
}

void GPIO_IRQIntrruptConfig(IrqNum_t IRQNum, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(IRQNum <= 31)
		{
			// Program ISER0 REG
			*NVIC_ISER0 |= (1 <<IRQNum);
		}
		else if(IRQNum >= 32 && IRQNum <= 63)
		{
			// Program ISER1 REG
			*NVIC_ISER1 |= (1 <<IRQNum % 32);
		}
		else if(IRQNum >= 64 && IRQNum <= 96)
		{
			// Program ISER2 REG
			*NVIC_ISER2 |= (1 <<IRQNum % 64);
		}
	}
	else
	{
		if(IRQNum <= 31)
		{
			// Program ICER0 REG
			*NVIC_ICER0 |= (1 <<IRQNum);
		}
		else if(IRQNum >= 32 && IRQNum <= 63)
		{
			// Program ICER1 REG
			*NVIC_ICER1 |= (1 <<IRQNum);
		}
		else if(IRQNum >= 64 && IRQNum <= 96)
		{
			// Program ICER2 REG
			*NVIC_ICER2 |= (1 <<IRQNum);
		}
	}
}

void GPIO_IRQPriorityConfig(IrqNum_t IRQNum, IrqPriority_t IRQPriority)
{
	// 1. First lets find out the ipr register
	uint8_t iprx = IRQNum / 4;
	uint8_t iprx_section = IRQNum % 4;
	uint8_t shitf_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + (iprx *4)) |= (IRQPriority << shitf_amount);
}

void GPIO_IRQHandling(GPIO_PinNum_t pinNumbr)
{
	//  Clear EXTI PR Register
	if(EXTI->EXTI_PR & (1 << pinNumbr))
	{
		EXTI->EXTI_PR |= (1 << pinNumbr);
	}
}

