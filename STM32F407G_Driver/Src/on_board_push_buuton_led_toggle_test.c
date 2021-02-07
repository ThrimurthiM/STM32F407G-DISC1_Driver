/*
 * push_buuton.c
 *
 *  Created on: Jan 6, 2021
 *      Author: SL-BLR-10169
 */


#include "stm32f407g.h"
#include "stm32f407g_gpio.h"

static void delay(uint32_t delay)
{
	for(int i = 0; i <= delay * 1000; i++);
}

int main_test1()
{

	//LED
	GPIO_Handle_t gpioConfig;
	gpioConfig.pGPIOx = GPIOD;
	gpioConfig.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM_12;
	gpioConfig.GPIO_PinConfig.GPIO_PinMode = GPIO_OUTPUT_MODE;
	gpioConfig.GPIO_PinConfig.GPIO_PinOPType = GPIO_PUSH_PULL_OUTPUT;
	gpioConfig.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PULL_UD;
	gpioConfig.GPIO_PinConfig.GPIO_PinSpeed = GPIO_LOW_SPEED;

	// 1. Enable Clock Source
	CLOCK_SOURCE_EN(); // PLL Clock

	// 2. Enable Peripheral clock
	GPIO_PeripheralClockControl(GPIOD,ENABLE);

	// 3. Init GPIO 12 port D as LED
	GPIO_Init(&gpioConfig);

	//Push Button
	GPIO_Handle_t gpioConfig1;
	gpioConfig1.pGPIOx = GPIOA;
	gpioConfig1.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM_0;
	gpioConfig1.GPIO_PinConfig.GPIO_PinMode = GPIO_INPUT_MODE;
	gpioConfig1.GPIO_PinConfig.GPIO_PinOPType = GPIO_PUSH_PULL_OUTPUT;
	gpioConfig1.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PULL_UD;
	gpioConfig1.GPIO_PinConfig.GPIO_PinSpeed = GPIO_LOW_SPEED;

	// 2. Enable Peripheral clock
	GPIO_PeripheralClockControl(GPIOA,ENABLE);

	// 3. Init GPIO 0 port A as Push Button
	GPIO_Init(&gpioConfig1);

	while(1)
	{
		if(GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_NUM_0))
		{
			delay(100);
			GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NUM_12);
		}

	}
}
