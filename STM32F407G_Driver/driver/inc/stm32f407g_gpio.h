/*
 * stm32f407g_gpio.h
 *
 *  Created on: Jan 2, 2021
 *      Author: SL-BLR-10169
 */

#ifndef INC_STM32F407G_GPIO_H_
#define INC_STM32F407G_GPIO_H_

#include "stm32f407g.h"

/*
 * Typedef and enum
 */

typedef enum
{
	GPIO_PIN_NUM_0 = 0,
	GPIO_PIN_NUM_1,
	GPIO_PIN_NUM_2,
	GPIO_PIN_NUM_3,
	GPIO_PIN_NUM_4,
	GPIO_PIN_NUM_5,
	GPIO_PIN_NUM_6,
	GPIO_PIN_NUM_7,
	GPIO_PIN_NUM_8,
	GPIO_PIN_NUM_9,
	GPIO_PIN_NUM_10,
	GPIO_PIN_NUM_11,
	GPIO_PIN_NUM_12,
	GPIO_PIN_NUM_13,
	GPIO_PIN_NUM_14,
	GPIO_PIN_NUM_15,
}GPIO_PinNum_t;

typedef enum
{
	GPIO_INPUT_MODE               = 0,
	GPIO_OUTPUT_MODE,
	GPIO_ALTERNATE_FUNCTION_MODE,
	GPIO_ANALOG_MODE
}GPIO_Mode_t;

typedef enum
{
	GPIO_PUSH_PULL_OUTPUT = 0,
	GPIO_OPEN_DAIN_OUTPUT
}GPIO_OutputType_t;

typedef enum
{
	GPIO_LOW_SPEED       = 0,
	GPIO_MEDIUM_SPEED,
	GPIO_HIGH_SPEED,
	GPIO_VERY_HIGH_SPEED,
}GPIO_Speed_t;


typedef enum
{
	GPIO_NO_PULL_UD      = 0,
	GPIO_PULL_UP_ON,
	GPIO_PULL_DOWN_ON,
}GPIO_PuPd_t;

typedef enum
{
	GPIO_NO_INTR      = 0,
	GPIO_INTER_FALLING_EDGE,
	GPIO_INTER_RAISING_EDGE,
	GPIO_INTER_BOTH_EDGE,
	GPIO_INTER_LOW_LEVEL,   // Not Used
	GPIO_INTER_HIGH_LEVEL   // Not Used
}GPIO_IntruptMode_t;

typedef enum
{
	GPIO_AF0  = 0,
	GPIO_AF1,
	GPIO_AF2,
	GPIO_AF3,
	GPIO_AF4,
	GPIO_AF5,
	GPIO_AF6,
	GPIO_AF7,
	GPIO_AF8,
	GPIO_AF9,
	GPIO_AF10,
	GPIO_AF11,
	GPIO_AF12,
	GPIO_AF13,
	GPIO_AF14,
	GPIO_AF15
}GPIO_AltFunMode_t;

typedef enum
{
IRQ_NUMBER_EXTI0 = 6,
IRQ_NUMBER_EXTI1 = 7,
IRQ_NUMBER_EXTI2 = 8,
IRQ_NUMBER_EXTI3 = 9,
IRQ_NUMBER_EXTI4 = 10,
IRQ_NUMBER_EXTI9_5 = 23,
IRQ_NUMBER_EXTI15_10 = 40
}IrqNum_t;

typedef enum
{
NVIC_IRQ_PRI0 = 0,
NVIC_IRQ_PRI1,
NVIC_IRQ_PRI2,
NVIC_IRQ_PRI3,
NVIC_IRQ_PRI4,
NVIC_IRQ_PRI5,
NVIC_IRQ_PRI6,
NVIC_IRQ_PRI7,
NVIC_IRQ_PRI8,
NVIC_IRQ_PRI9,
NVIC_IRQ_PRI10,
NVIC_IRQ_PRI11,
NVIC_IRQ_PR12,
NVIC_IRQ_PRI13,
NVIC_IRQ_PRI14,
NVIC_IRQ_PRI15
}IrqPriority_t;

/*
 * This is the configuration strcture of GPIO Pin
 */

typedef struct
{

	GPIO_PinNum_t      GPIO_PinNumber;
	GPIO_Mode_t        GPIO_PinMode;
	GPIO_Speed_t       GPIO_PinSpeed;
	GPIO_PuPd_t        GPIO_PinPuPdControl;
	GPIO_OutputType_t  GPIO_PinOPType;
	GPIO_AltFunMode_t  GPIO_PinAltFunMode;
	GPIO_IntruptMode_t GPIO_INTR;
}GPIO_PinConfig_t;

/* Handle structure for GPIO Pin */
typedef struct
{
	GPIO_RegDef_t *pGPIOx;         /* This holds the base address of Gio port to which pin belongs */
	GPIO_PinConfig_t   GPIO_PinConfig; /* This holds GPIO pin COnfiguration settings */
}GPIO_Handle_t;

/* External API */
void GPIO_PeripheralClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi	);

void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber, uint8_t val);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t val);

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber);

void GPIO_IRQIntrruptConfig(IrqNum_t IRQNum, uint8_t EnorDi);
void GPIO_IRQPriorityConfig(IrqNum_t IRQNum, IrqPriority_t IRQPriority);
void GPIO_IRQHandling(GPIO_PinNum_t pinNumbr);

#endif /* INC_STM32F407G_GPIO_H_ */
