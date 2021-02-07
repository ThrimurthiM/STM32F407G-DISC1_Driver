/*
 * gpio_Toggle.c
 *
 *  Created on: Jan 5, 2021
 *      Author: SL-BLR-10169
 */

#include "stm32f407g.h"
#include "stm32f407g_gpio.h"

static void delay(uint32_t delay)
{

	for(int i = 0; i <= delay * 1000; i++);
}

void EXTI0_IRQHandler(void)
{
	delay(20); //wait some time to overcome re-bouncing of interrupt event
	GPIO_IRQHandling(GPIO_PIN_NUM_0);
	GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NUM_12);
}

int main()
{

	//Push Button Interrupt
	GPIO_Handle_t intrGpioConfig;
	intrGpioConfig.pGPIOx = GPIOA;
	intrGpioConfig.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM_0;
	intrGpioConfig.GPIO_PinConfig.GPIO_PinMode = GPIO_INPUT_MODE;
	intrGpioConfig.GPIO_PinConfig.GPIO_PinOPType = GPIO_OPEN_DAIN_OUTPUT;
	intrGpioConfig.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PULL_UD;
	intrGpioConfig.GPIO_PinConfig.GPIO_PinSpeed = GPIO_LOW_SPEED;
	intrGpioConfig.GPIO_PinConfig.GPIO_INTR = GPIO_INTER_FALLING_EDGE;

	// 1. Enable Clock Source
	CLOCK_SOURCE_EN(); // PLL Clock

	// 2. Enable Peripheral clock
	GPIO_PeripheralClockControl(GPIOA,ENABLE);

	// 3. Init GPIO 0 port A as Push Button
	GPIO_Init(&intrGpioConfig);

	// 4. Interrupt Configuration
	GPIO_IRQIntrruptConfig(IRQ_NUMBER_EXTI0,ENABLE);
	GPIO_IRQPriorityConfig(IRQ_NUMBER_EXTI0,NVIC_IRQ_PRI15);


	//LED GPIO Configuration
	GPIO_Handle_t gpioConfig;
	gpioConfig.pGPIOx = GPIOD;
	gpioConfig.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM_12;
	gpioConfig.GPIO_PinConfig.GPIO_PinMode = GPIO_OUTPUT_MODE;
	gpioConfig.GPIO_PinConfig.GPIO_PinOPType = GPIO_PUSH_PULL_OUTPUT;
	gpioConfig.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PULL_UD;
	gpioConfig.GPIO_PinConfig.GPIO_PinSpeed = GPIO_LOW_SPEED;

	// 1. Enable Peripheral clock
	GPIO_PeripheralClockControl(GPIOD,ENABLE);

	// 2. Init GPIO 12 port D as LED
	GPIO_Init(&gpioConfig);

	while(1)
	{
		delay(1000);
	}
}
