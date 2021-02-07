/*
 * stm32f407g.h
 *
 *  Created on: Dec 30, 2020
 *      Author: SL-BLR-10169
 */

#ifndef INC_STM32F407G_H_
#define INC_STM32F407G_H_

/* Header */
#include <stdint.h>


/* Define */

/***************Processor Specific Details **************************/
/*
 * ARM Cortex Mx(M4) Processor NVIC ISERx Register Address
 */

#define NVIC_ISER0      (volatile uint32_t*)(0xE000E100)
#define NVIC_ISER1      (volatile uint32_t*)(0xE000E104)
#define NVIC_ISER2      (volatile uint32_t*)(0xE000E108)

/*
 * ARM Cortex Mx(M4) Processor NVIC ICERx Register Address
 */

#define NVIC_ICER0      (volatile uint32_t*)(0xE000E180)
#define NVIC_ICER1      (volatile uint32_t*)(0xE000E184)
#define NVIC_ICER2      (volatile uint32_t*)(0xE000E188)

/*
 * ARM Cortex Mx(M4) Processor NVIC IPR (Priority) Register Address
 */
#define NVIC_PR_BASE_ADDR    (volatile uint32_t*)(0xE000E400)

#define NO_PR_BITS_IMPLEMENTED     (4)
/*
 *  Base Address of Flash and SRAM Memory
 */

#define FLASH_BASE_ADDR             0x08000000U /* Main Memory */
#define SRAM1_BASE_ADDR 	        0x20000000U /* SRAM1 Base Address */
#define SRAM2_BASE_ADDR 	        0x2001C000U /* SRAM2 Base Address */
#define SRAM3_BASE_ADDR 	        0x20020000U /* SRAM3 Base Address */
#define ROM_BASE_ADDR               0x1FFF0000U /*System Memory */


/*
 * AHBx and APBx bus Peripheral Base Address
 */

#define PERIPHERAL_BASE_ADDR        0x40000000U          /* Peripheral Base Address */
#define APB1_BASE_ADDR              PERIPHERAL_BASE_ADDR /* APB1 Base Address */
#define APB2_BASE_ADDR              0x40010000U          /* APB2 Base Address */
#define AHB1_BASE_ADDR              0x40020000U          /* AHB1 Base Address */
#define AHB2_BASE_ADDR              0x50000000U 	     /* AHB2 Base Address */
#define AHB3_BASE_ADDR              0x60000000U 	     /* AHB3 Base Address */

/*
 * Base Address of Peripherals which are hanging on AHB1 Bus
 */

#define GPIOA_BASE_ADDR          (AHB1_BASE_ADDR + 0x0000)
#define GPIOB_BASE_ADDR          (AHB1_BASE_ADDR + 0x0400)
#define GPIOC_BASE_ADDR          (AHB1_BASE_ADDR + 0x0800)
#define GPIOD_BASE_ADDR          (AHB1_BASE_ADDR + 0x0C00)
#define GPIOE_BASE_ADDR          (AHB1_BASE_ADDR + 0x1000)
#define GPIOF_BASE_ADDR          (AHB1_BASE_ADDR + 0x1400)
#define GPIOG_BASE_ADDR          (AHB1_BASE_ADDR + 0x1800)
#define GPIOH_BASE_ADDR          (AHB1_BASE_ADDR + 0x1C00)
#define GPIOI_BASE_ADDR          (AHB1_BASE_ADDR + 0x2000)
#define RCC_BASE_ADDR            (AHB1_BASE_ADDR + 0x3800)

/*
 * Base Address of Peripherals which are hanging on APB1 Bus
 */

#define TIM2_BASE_ADDR                   (APB1_BASE_ADDR + 0x0000)
#define TIM3_BASE_ADDR                   (APB1_BASE_ADDR + 0x0400)
#define TIM4_BASE_ADDR                   (APB1_BASE_ADDR + 0x0800)
#define TIM5_BASE_ADDR                   (APB1_BASE_ADDR + 0x0C00)
#define TIM6_BASE_ADDR                   (APB1_BASE_ADDR + 0x1000)
#define TIM7_BASE_ADDR                   (APB1_BASE_ADDR + 0x1400)
#define TIM12_BASE_ADDR                  (APB1_BASE_ADDR + 0x1800)
#define TIM13_BASE_ADDR                  (APB1_BASE_ADDR + 0x1C00)
#define TIM14_BASE_ADDR                  (APB1_BASE_ADDR + 0x2000)
#define RTC_BKP_BASE_ADDR                (APB1_BASE_ADDR + 0x2800)
#define WWDG_BASE_ADDR                   (APB1_BASE_ADDR + 0x2C00)
#define IWDG_BASE_ADDR                   (APB1_BASE_ADDR + 0x3000)
#define I2S2ext_BASE_ADDR                (APB1_BASE_ADDR + 0x3400)
#define SPI2ORI2S2_BASE_ADDR             (APB1_BASE_ADDR + 0x3800)
#define SPI3ORI2S3_BASE_ADDR             (APB1_BASE_ADDR + 0x3C00)
#define I2S3ext_BASE_ADDR                (APB1_BASE_ADDR + 0x4000)
#define USART2_BASE_ADDR                 (APB1_BASE_ADDR + 0x4400)
#define USART3_BASE_ADDR                 (APB1_BASE_ADDR + 0x4800)
#define UART4_BASE_ADDR                  (APB1_BASE_ADDR + 0x4C00)
#define UART5_BASE_ADDR                  (APB1_BASE_ADDR + 0x5000)
#define I2C1_BASE_ADDR                   (APB1_BASE_ADDR + 0x5400)
#define I2C2_BASE_ADDR                   (APB1_BASE_ADDR + 0x5800)
#define I2C3_BASE_ADDR                   (APB1_BASE_ADDR + 0x5C00)
#define CAN1_BASE_ADDR                   (APB1_BASE_ADDR + 0x6400)
#define CAN2_BASE_ADDR                   (APB1_BASE_ADDR + 0x6800)
#define PWR_BASE_ADDR                    (APB1_BASE_ADDR + 0x7000)
#define DAC_BASE_ADDR                    (APB1_BASE_ADDR + 0x7400)

/*
 * Base Address of Peripherals which are hanging on APB2 Bus
 */

#define  TIM11_BASE_ADDR                       (APB2_BASE_ADDR + 0x4800)
#define  TIM10_BASE_ADDR                       (APB2_BASE_ADDR + 0x4400)
#define  TIM9_BASE_ADDR                        (APB2_BASE_ADDR + 0x4000)
#define  EXTI_BASE_ADDR                        (APB2_BASE_ADDR + 0x3C00)
#define  SYSCFG_BASE_ADDR                      (APB2_BASE_ADDR + 0x3800)
#define  Reserved                              (APB2_BASE_ADDR + 0x3400)
#define  SPI1_BASE_ADDR                        (APB2_BASE_ADDR + 0x3000)
#define  SDIO_BASE_ADDR                        (APB2_BASE_ADDR + 0x2C00)
#define  ADC1_ADC2_ADC3_BASE_ADDR              (APB2_BASE_ADDR + 0x2000)
#define  USART6_BASE_ADDR                      (APB2_BASE_ADDR + 0x1400)
#define  USART1_BASE_ADDR                      (APB2_BASE_ADDR + 0x1000)
#define  TIM8_BASE_ADDR                        (APB2_BASE_ADDR + 0x0400)
#define  TIM1_BASE_ADDR                        (APB2_BASE_ADDR + 0x0000)

/* Structure and Typedef */

/********************** Peripheral Register Defintion Structures ***********************************/

/* GPIO */
typedef struct
{
	volatile uint32_t MODER;  /* GPIO port mode register Address offset: 0x00 */
	volatile uint32_t OTYPER; /* GPIO port output type register Address offset: 0x04 */
	volatile uint32_t OSPEEDR;/* GPIO port output speed register offset: 0x08 */
	volatile uint32_t PUPDR;  /* GPIO port pull-up/pull-down registers offset: 0x0C */
	volatile uint32_t IDR;    /* GPIO port input data register offset: 0x10 */
	volatile uint32_t ODR;    /* GPIO port output data register offset: 0x14 */
	volatile uint32_t BSRR;   /* GPIO port bit set/reset register offset: 0x18 */
	volatile uint32_t LCKR;   /* GPIO port configuration lock register offset: 0x1C */
	volatile uint32_t AFR[2]; /* GPIO alternate function low register AFRL& High Register AFRH offset: 0x20	 */
}GPIO_RegDef_t;

/* RCC */
typedef struct
{
	volatile uint32_t CR;
	volatile uint32_t PLLCFGR;
	volatile uint32_t CFGR;
	volatile uint32_t CIR;
	volatile uint32_t AHB1RSTR;
	volatile uint32_t AHB2RSTR;
	volatile uint32_t AHB3RSTR;
	volatile uint32_t RESERVED0;
	volatile uint32_t APB1RSTR;
	volatile uint32_t APB2RSTR;
	volatile uint32_t RESERVED1[2];
	volatile uint32_t AHB1ENR;
	volatile uint32_t AHB2ENR;
	volatile uint32_t AHB3ENR;
	volatile uint32_t RESERVED3;
	volatile uint32_t APB1ENR;
	volatile uint32_t APB2ENR;
	volatile uint32_t RESERVED4[2];
	volatile uint32_t AHB1LPENR;
	volatile uint32_t AHB2LPENR;
	volatile uint32_t AHB3LPENR;
	volatile uint32_t RESERVED5;
	volatile uint32_t APB1LPENR;
	volatile uint32_t APB2LPENR;
	volatile uint32_t RESERVED6[2];
	volatile uint32_t BDCR;
	volatile uint32_t CSR;
	volatile uint32_t RESERVED7[2];
	volatile uint32_t SSCGR;
	volatile uint32_t PLLI2SCFGR;
	volatile uint32_t PLLSAICFGR;
	volatile uint32_t DCKCFGR;
}Rcc_RegDef_t;

typedef struct
{
	volatile uint32_t EXTI_IMR;
	volatile uint32_t EXTI_EMR;
	volatile uint32_t EXTI_RTSR;
	volatile uint32_t EXTI_FTSR;
	volatile uint32_t EXTI_SWIER;
	volatile uint32_t EXTI_PR;
}Exti_RegDef_t;

typedef struct
{
	volatile uint32_t MEMRMP;
	volatile uint32_t PMC;
	volatile uint32_t EXTICR[4];
	volatile uint32_t RESERVED[2];
	volatile uint32_t CMPCR;
}SysCfg_RegDef_t;

/* Peripheral Register Defintion (Peripheral Register type caste to Gpio_RegDef_t) */
#define GPIOA		((GPIO_RegDef_t*)GPIOA_BASE_ADDR)
#define GPIOB		((GPIO_RegDef_t*)GPIOB_BASE_ADDR)
#define GPIOC		((GPIO_RegDef_t*)GPIOC_BASE_ADDR)
#define GPIOD		((GPIO_RegDef_t*)GPIOD_BASE_ADDR)
#define GPIOE		((GPIO_RegDef_t*)GPIOE_BASE_ADDR)
#define GPIOF		((GPIO_RegDef_t*)GPIOF_BASE_ADDR)
#define GPIOG		((GPIO_RegDef_t*)GPIOG_BASE_ADDR)
#define GPIOH		((GPIO_RegDef_t*)GPIOH_BASE_ADDR)
#define GPIOI		((GPIO_RegDef_t*)GPIOI_BASE_ADDR)

#define RCC		    ((Rcc_RegDef_t*)RCC_BASE_ADDR)
#define EXTI        ((Exti_RegDef_t*)EXTI_BASE_ADDR)
#define SYSCFG      ((SysCfg_RegDef_t*)SYSCFG_BASE_ADDR)



/*
 * Enable Clock Source
 */
//#define EXTERNAL_CLOCK_SOURCE
//#define INTERNAL_CLOCK_SOURCE
#define PLL_SOURCE

#if defined  EXTERNAL_CLOCK_SOURCE
#define CLOCK_SOURCE_EN() \
		do\
		{\
			/* Enable the HSE clock using HSEON bit (RCC_CR) */\
		(RCC->CR |= ( 1 << 16) );\
		/*Wait until HSE clock from the external crystal stabilizes (only if crystal is connected ) */\
		while( ! (RCC->CR & ( 1 << 17) ) );\
		/* Switch the system clock to HSE (RCC_CFGR) */\
		RCC->CFGR |= ( 1 << 0);\
		/* Configure the RCC_CFGR MCO1 bit fields to select HSE as clock source */\
		RCC->CFGR |= ( 1 << 22);/* clear 21 and SET 22 */\
		/* Configure MCO1 prescaler divisor as 4 */\
		RCC->CFGR |= ( 1 << 25);\
		RCC->CFGR |= ( 1 << 26);\
		} while(0)
#elif defined INTERNAL_CLOCK_SOURCE
#define CLOCK_SOURCE_EN()\
		do\
		{\
			/* Enable the HSI clock using HSEON bit (RCC_CR) */\
		(RCC->CR |= ( 1 << 0) );\
		/*Wait until HSI clock from the Internal RC Circuite stabilizes */\
		while( ! (RCC->CR & ( 1 << 1) ) );\
		/* Switch the system clock to HSI (RCC_CFGR) */\
		RCC->CFGR |= ( 0 << 0);\
		/* Configure the RCC_CFGR MCO1 bit fields to select HSI as clock source */\
		RCC->CFGR |= ( 0 << 21); /* clear 21 */\
		RCC->CFGR |= ( 0 << 22); /* clear 22 */\
		/* Configure MCO1 prescaler divisor as 4 */\
		RCC->CFGR |= ( 1 << 25);\
		RCC->CFGR |= ( 1 << 26);\
		} while(0)
#elif defined PLL_SOURCE
#define CLOCK_SOURCE_EN()\
		do\
		{\
			/* Enable the HSI clock using HSI ON bit (RCC_CR) */\
		(RCC->CR |= ( 1 << 0) );\
		/* Enable the PLL clock using PLL ON bit (RCC_CR) */\
		(RCC->CR |= ( 1 << 24) );\
		/* Select PLL Clock Source as HSI */ \
		RCC->PLLCFGR |= ( 00 << 0); /* 0: HSI, 1: HSE */\
		/*Wait until PLL stabilizes */\
		while( ! (RCC->CR & ( 1 << 25) ) );\
		/* Switch the system clock to PLL (RCC_CFGR) */\
		RCC->CFGR |= ( 0x03 << 0);\
		/* Configure the RCC_CFGR MCO1 bit fields to select PLL as clock source */\
		RCC->CFGR |= ( 1 << 21); /* Set 21 */\
		RCC->CFGR |= ( 1 << 22); /* Set 22 */\
		/* Configure MCO1 prescaler divisor as 4 */\
		RCC->CFGR |= ( 1 << 25);\
		RCC->CFGR |= ( 1 << 26);\
		} while(0)
#else
#define CLOCK_SOURCE_EN() do { (RCC->CFGR &= ~( 0x03 << 21) ); 	RCC->CFGR |= ( 1 << 25); RCC->CFGR |= ( 1 << 26); } while(0)
#endif

			/*
			 * Clock Enable Macro for GPIOx Peripheral
			 */
#define GPIOA_PCLK_EN()		(RCC->AHB1ENR |= ( 1 << 0) )
#define GPIOB_PCLK_EN()		(RCC->AHB1ENR |= ( 1 << 1) )
#define GPIOC_PCLK_EN()		(RCC->AHB1ENR |= ( 1 << 2) )
#define GPIOD_PCLK_EN()		(RCC->AHB1ENR |= ( 1 << 3) )
#define GPIOE_PCLK_EN()		(RCC->AHB1ENR |= ( 1 << 4) )
#define GPIOF_PCLK_EN()		(RCC->AHB1ENR |= ( 1 << 5) )
#define GPIOG_PCLK_EN()		(RCC->AHB1ENR |= ( 1 << 6) )
#define GPIOH_PCLK_EN()		(RCC->AHB1ENR |= ( 1 << 7) )
#define GPIOI_PCLK_EN()		(RCC->AHB1ENR |= ( 1 << 8) )

			/*
			 * Reset Macro for GPIOx Peripheral
			 */
#define GPIOA_REG_RESET()   do{ (RCC->AHB1RSTR |= ( 1 << 0) ); (RCC->AHB1RSTR &= ~( 1 << 0)); }while(0)
#define GPIOB_REG_RESET()   do{ (RCC->AHB1RSTR |= ( 1 << 0) ); (RCC->AHB1RSTR &= ~( 1 << 1)); }while(0)
#define GPIOC_REG_RESET()   do{ (RCC->AHB1RSTR |= ( 1 << 0) ); (RCC->AHB1RSTR &= ~( 1 << 2)); }while(0)
#define GPIOD_REG_RESET()   do{ (RCC->AHB1RSTR |= ( 1 << 0) ); (RCC->AHB1RSTR &= ~( 1 << 3)); }while(0)
#define GPIOE_REG_RESET()   do{ (RCC->AHB1RSTR |= ( 1 << 0) ); (RCC->AHB1RSTR &= ~( 1 << 4)); }while(0)
#define GPIOF_REG_RESET()   do{ (RCC->AHB1RSTR |= ( 1 << 0) ); (RCC->AHB1RSTR &= ~( 1 << 5)); }while(0)
#define GPIOG_REG_RESET()   do{ (RCC->AHB1RSTR |= ( 1 << 0) ); (RCC->AHB1RSTR &= ~( 1 << 6)); }while(0)
#define GPIOH_REG_RESET()   do{ (RCC->AHB1RSTR |= ( 1 << 0) ); (RCC->AHB1RSTR &= ~( 1 << 7)); }while(0)
#define GPIOI_REG_RESET()   do{ (RCC->AHB1RSTR |= ( 1 << 0) ); (RCC->AHB1RSTR &= ~( 1 << 8)); }while(0)

			/*
			 * Clock Enable Macro for I2Cx Peripheral
			 */
#define I2C1_PCLK_EN()		(RCC->AHB1ENR |= ( 1 << 21) )
#define I2C2_PCLK_EN()		(RCC->AHB1ENR |= ( 1 << 22) )
#define I2C3_PCLK_EN()		(RCC->AHB1ENR |= ( 1 << 23) )

			/*
			 * Clock Enable Macro for SPIx Peripheral
			 */
#define SPI1_PCLK_EN()		(RCC->AHB2ENR |= ( 1 << 12) )
#define SPI2_PCLK_EN()		(RCC->AHB1ENR |= ( 1 << 14) )
#define SPI3_PCLK_EN()		(RCC->AHB1ENR |= ( 1 << 15) )
#define SPI4_PCLK_EN()		(RCC->AHB2ENR |= ( 1 << 13) )

			/*
			 * Clock Enable Macro for UARTx Peripheral
			 */
#define UART1_PCLK_EN()		(RCC->AHB2ENR |= ( 1 << 4 ) )
#define UART2_PCLK_EN()		(RCC->AHB1ENR |= ( 1 << 17) )
#define UART3_PCLK_EN()		(RCC->AHB1ENR |= ( 1 << 18) )
#define UART4_PCLK_EN()		(RCC->AHB1ENR |= ( 1 << 19) )
#define UART5_PCLK_EN()		(RCC->AHB1ENR |= ( 1 << 20) )
#define USART6_PCLK_EN()	(RCC->AHB2ENR |= ( 1 << 5) )

			/*
			 * Clock Enable Macro for SYSCFG Peripheral
			 */
#define SYSCFG_PCLK_EN()	(RCC->AHB2ENR |= ( 1 << 14) )

			/*
			 * Clock Disable Macro for GPIOx Peripheral
			 */
#define GPIOA_PCLK_DI()		(RCC->AHB1ENR &= ~( 1 << 0) )
#define GPIOB_PCLK_DI()		(RCC->AHB1ENR &= ~( 1 << 1) )
#define GPIOC_PCLK_DI()		(RCC->AHB1ENR &= ~( 1 << 2) )
#define GPIOD_PCLK_DI()		(RCC->AHB1ENR &= ~( 1 << 3) )
#define GPIOE_PCLK_DI()		(RCC->AHB1ENR &= ~( 1 << 4) )
#define GPIOF_PCLK_DI()		(RCC->AHB1ENR &= ~( 1 << 5) )
#define GPIOG_PCLK_DI()		(RCC->AHB1ENR &= ~( 1 << 6) )
#define GPIOH_PCLK_DI()		(RCC->AHB1ENR &= ~( 1 << 7) )
#define GPIOI_PCLK_DI()		(RCC->AHB1ENR &= ~( 1 << 8) )

			/*
			 * Clock Disable Macro for I2Cx Peripheral
			 */
#define I2C1_PCLK_DI()		(RCC->AHB1ENR &= ~( 1 << 21) )
#define I2C2_PCLK_DI()		(RCC->AHB1ENR &= ~( 1 << 22) )
#define I2C3_PCLK_DI()		(RCC->AHB1ENR &= ~( 1 << 23) )

			/*
			 * Clock Disable Macro for SPIx Peripheral
			 */
#define SPI1_PCLK_DI()		(RCC->AHB2ENR &= ~( 1 << 12) )
#define SPI2_PCLK_DI()		(RCC->AHB1ENR &= ~( 1 << 14) )
#define SPI3_PCLK_DI()		(RCC->AHB1ENR &= ~( 1 << 15) )
#define SPI4_PCLK_DI()		(RCC->AHB2ENR &= ~( 1 << 13) )

			/*
			 * Clock Disable Macro for UARTx Peripheral
			 */
#define UART1_PCLK_DI()		(RCC->AHB2ENR &= ~( 1 << 4 ) )
#define UART2_PCLK_DI()		(RCC->AHB1ENR &= ~( 1 << 17) )
#define UART3_PCLK_DI()		(RCC->AHB1ENR &= ~( 1 << 18) )
#define UART4_PCLK_DI()		(RCC->AHB1ENR &= ~( 1 << 19) )
#define UART5_PCLK_DI()		(RCC->AHB1ENR &= ~( 1 << 20) )
#define USART6_PCLK_DI()	(RCC->AHB2ENR &= ~( 1 << 5) )

			/*
			 * Generic Macros
			 */
#define ENABLE            1
#define DISABLE           0
#define SET               ENABLE
#define RESET             DISABLE
#define GPIO_PIN_SET      SET
#define GPIO_PIN_RESET    RESET
#define GPIO_BASE_ADD_TO_CODE(x)    (x == GPIOA)? 0:\
		                            (x == GPIOB)? 1:\
				                    (x == GPIOC)? 2:\
				                    (x == GPIOD)? 3:\
						            (x == GPIOE)? 4:\
						            (x == GPIOF)? 5:\
						            (x == GPIOG)? 6:\
						            (x == GPIOH)? 7:\
						            (x == GPIOI)? 8:0

#endif /* INC_STM32F407G_H_ */
