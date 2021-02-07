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

int main_test()
{

	GPIO_Handle_t gpioConfig;
	gpioConfig.pGPIOx = GPIOD;
	gpioConfig.GPIO_PinConfig.GPIO_PinMode = GPIO_OUTPUT_MODE;
	// gpioConfig.GPIO_PinConfig.GPIO_PinOPType = GPIO_PUSH_PULL_OUTPUT; // Push Pull
	gpioConfig.GPIO_PinConfig.GPIO_PinOPType = GPIO_OPEN_DAIN_OUTPUT; // Open Dain
	gpioConfig.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PULL_UP_ON;
	gpioConfig.GPIO_PinConfig.GPIO_PinSpeed = GPIO_LOW_SPEED;

	// 1. Enable Clock Source
	CLOCK_SOURCE_EN(); // PLL Clock

	// 2. Enable Peripheral clock
	GPIO_PeripheralClockControl(GPIOD,ENABLE);

	// 3. Init GPIO 12
	gpioConfig.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM_12;
	GPIO_Init(&gpioConfig);

	// 3. Init GPIO 13
	gpioConfig.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM_13;
	GPIO_Init(&gpioConfig);

	// 4. Togle GPIO PD12 and PD13
	GPIO_WriteToOutputPin(GPIOD, GPIO_PIN_NUM_12, GPIO_PIN_SET);
	GPIO_WriteToOutputPin(GPIOD, GPIO_PIN_NUM_13, GPIO_PIN_RESET);
	while(1)
	{
		GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NUM_12);
		GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NUM_13);
		delay(1000);
	}
}
