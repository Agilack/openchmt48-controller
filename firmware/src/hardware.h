/**
 * @file  hardware.h
 * @brief Headers and definitions for low-level hardware handling
 *
 * @author Saint-Genest Gwenael <gwen@agilack.fr>
 * @copyright Agilack (c) 2018-2020
 *
 * @page License
 * This firmware is free software: you can redistribute it and/or modify it
 * under the terms of the GNU Lesser General Public License version 3 as
 * published by the Free Software Foundation. You should have received a
 * copy of the GNU Lesser General Public License along with this program,
 * see LICENSE.md file for more details.
 * This program is distributed WITHOUT ANY WARRANTY.
 */
#ifndef HARDWARE_H
#define HARDWARE_H

#include "types.h"

/* APB 1 */
#define TIM2   0x40000000
#define TIM3   0x40000400
#define TIM4   0x40000800
#define TIM5   0x40000C00
#define TIM6   0x40001000
#define TIM7   0x40001400
#define TIM12  0x40001800
#define TIM13  0x40001C00
#define TIM14  0x40002000
#define LPTIM1 0x40002400
#define RTC    0x40002800
#define WWDG   0x40002C00
#define IWDG   0x40003000
#define UART3  0x40004800
#define UART7  0x40007800
/* APB 2 */
#define TIM1   0x40010000
#define UART1  0x40011000
#define ADC1   0x40012000
#define ADC2   0x40012100
#define ADC3   0x40012200
#define ADC123 0x40012300
#define SYSCFG 0x40013800
#define EXTI   0x40013C00
#define LTDC   0x40016800
/* AHB 1 */
#define GPIOA 0x40020000
#define GPIOB 0x40020400
#define GPIOC 0x40020800
#define GPIOD 0x40020C00
#define GPIOE 0x40021000
#define GPIOF 0x40021400
#define GPIOG 0x40021800
#define GPIOI 0x40022000
#define GPIOJ 0x40022400
#define GPIOK 0x40022800
#define RCC   0x40023800
#define FLASH 0x40023C00
#define ETH   0x40028000
/* Cortex-M7 Internal Peripherals */
#define CM7_SCB_ACR   0xE000E008
#define CM7_SYSTICK   0xE000E010
#define CM7_NVIC      0xE000E100
#define CM7_SCB       0xE000ED00
#define CM7_FEATURE   0xE000ED78
#define CM7_MPU       0xE000ED90
#define CM7_NVIC_STIR 0xE000EF00
#define CM7_FPU       0xE000EF30
#define CM7_CACHE     0xE000EF50
#define CM7_ACR       0xE000EF90

#define TIM1_CR1    (TIM1 + 0x00)
#define TIM1_SMCR   (TIM1 + 0x08)
#define TIM1_DIER   (TIM1 + 0x0C)
#define TIM1_EGR    (TIM1 + 0x14)
#define TIM1_CCMR1  (TIM1 + 0x18)
#define TIM1_CCMR2  (TIM1 + 0x1C)
#define TIM1_CCER   (TIM1 + 0x20)
#define TIM1_PSC    (TIM1 + 0x28)
#define TIM1_ARR    (TIM1 + 0x2C)
#define TIM1_CCR1   (TIM1 + 0x34)
#define TIM1_CCR2   (TIM1 + 0x38)
#define TIM1_CCR3   (TIM1 + 0x3C)
#define TIM1_CCR4   (TIM1 + 0x40)
#define TIM1_BDTR   (TIM1 + 0x44)

#define TIM2_CR1    (TIM2 + 0x00)
#define TIM2_DIER   (TIM2 + 0x0C)
#define TIM2_EGR    (TIM2 + 0x14)
#define TIM2_CCMR1  (TIM2 + 0x18)
#define TIM2_CCMR2  (TIM2 + 0x1C)
#define TIM2_CCER   (TIM2 + 0x20)
#define TIM2_PSC    (TIM2 + 0x28)
#define TIM2_ARR    (TIM2 + 0x2C)
#define TIM2_CCR1   (TIM2 + 0x34)
#define TIM2_CCR2   (TIM2 + 0x38)
#define TIM2_CCR3   (TIM2 + 0x3C)
#define TIM2_CCR4   (TIM2 + 0x40)
#define TIM2_BDTR   (TIM2 + 0x44)

#define TIM5_CR1    (TIM5 + 0x00)
#define TIM5_DIER   (TIM5 + 0x0C)
#define TIM5_EGR    (TIM5 + 0x14)
#define TIM5_CCMR1  (TIM5 + 0x18)
#define TIM5_CCMR2  (TIM5 + 0x1C)
#define TIM5_CCER   (TIM5 + 0x20)
#define TIM5_PSC    (TIM5 + 0x28)
#define TIM5_ARR    (TIM5 + 0x2C)
#define TIM5_CCR1   (TIM5 + 0x34)
#define TIM5_CCR2   (TIM5 + 0x38)
#define TIM5_CCR3   (TIM5 + 0x3C)
#define TIM5_CCR4   (TIM5 + 0x40)
#define TIM5_BDTR   (TIM5 + 0x44)

#define TIM_CR1(x)   (x + 0x00)
#define TIM_DIER(x)  (x + 0x0C)
#define TIM_SR(x)    (x + 0x10)
#define TIM_EGR(x)   (x + 0x14)
#define TIM_CCMR1(x) (x + 0x18)
#define TIM_CCMR2(x) (x + 0x1C)
#define TIM_CCER(x)  (x + 0x20)
#define TIM_CNT(x)   (x + 0x24)
#define TIM_PSC(x)   (x + 0x28)
#define TIM_ARR(x)   (x + 0x2C)
#define TIM_CCR1(x)  (x + 0x34)
#define TIM_CCR2(x)  (x + 0x38)
#define TIM_CCR3(x)  (x + 0x3C)
#define TIM_CCR4(x)  (x + 0x40)

#define RTC_BKP0R   (RTC  + 0x50)
#define RTC_BKP1R   (RTC  + 0x54)

#define IWDG_KR     (IWDG + 0x000)

#define RCC_CR      (RCC + 0x000)
#define RCC_PLLCFGR (RCC + 0x004)
#define RCC_CFGR    (RCC + 0x008)
#define RCC_AHB1ENR (RCC + 0x030)
#define RCC_APB1ENR (RCC + 0x040)
#define RCC_APB2ENR (RCC + 0x044)
#define RCC_PLLSAI  (RCC + 0x088)
#define RCC_DKCFGR1 (RCC + 0x08C)

#define GPIO_MODER(x)  (x + 0x000)
#define GPIO_OTYPE(x)  (x + 0x004)
#define GPIO_OSPEED(x) (x + 0x008)
#define GPIO_PUPD(x)   (x + 0x00C)
#define GPIO_IDR(x)    (x + 0x010)
#define GPIO_ODR(x)    (x + 0x014)
#define GPIO_BSRR(x)   (x + 0x018)
#define GPIO_LCK(x)    (x + 0x01C)
#define GPIO_AFRL(x)   (x + 0x020)
#define GPIO_AFRH(x)   (x + 0x024)

#define GPIOA_MODER (GPIOA + 0x000)
#define GPIOA_AFRL  (GPIOA + 0x020)
#define GPIOA_AFRH  (GPIOA + 0x024)
#define GPIOA_BSRR  (GPIOA + 0x018)

#define GPIOB_MODER (GPIOB + 0x000)
#define GPIOB_ODR   (GPIOB + 0x014)
#define GPIOB_BSRR  (GPIOB + 0x018)
#define GPIOB_AFRL  (GPIOB + 0x020)
#define GPIOB_AFRH  (GPIOB + 0x024)

#define GPIOC_MODER (GPIOC + 0x000)
#define GPIOC_BSRR  (GPIOC + 0x018)
#define GPIOC_AFRL  (GPIOC + 0x020)

#define GPIOD_MODER (GPIOD + 0x000)
#define GPIOD_ODR   (GPIOD + 0x014)
#define GPIOD_BSRR  (GPIOD + 0x018)

#define GPIOE_MODER (GPIOE + 0x000)
#define GPIOE_ODR   (GPIOE + 0x014)
#define GPIOE_BSRR  (GPIOE + 0x018)
#define GPIOE_AFRL  (GPIOE + 0x020)
#define GPIOE_AFRH  (GPIOE + 0x024)

#define GPIOF_MODER (GPIOF + 0x000)
#define GPIOF_ODR   (GPIOF + 0x014)
#define GPIOF_BSRR  (GPIOF + 0x018)
#define GPIOF_AFRL  (GPIOF + 0x020)

#define GPIOG_MODER (GPIOG + 0x000)
#define GPIOG_ODR   (GPIOG + 0x014)
#define GPIOG_BSRR  (GPIOG + 0x018)
#define GPIOG_AFRL  (GPIOG + 0x020)
#define GPIOG_AFRH  (GPIOG + 0x024)

#define GPIOI_MODER (GPIOI + 0x000)
#define GPIOI_ODR   (GPIOI + 0x014)
#define GPIOI_BSRR  (GPIOI + 0x018)
#define GPIOI_AFRH  (GPIOI + 0x024)

#define GPIOJ_MODER (GPIOJ + 0x000)
#define GPIOJ_AFRL  (GPIOJ + 0x020)
#define GPIOJ_AFRH  (GPIOJ + 0x024)

#define GPIOK_MODER (GPIOK + 0x000)
#define GPIOK_BSRR  (GPIOK + 0x018)
#define GPIOK_AFRL  (GPIOK + 0x020)

#define FLASH_ACR     (FLASH + 0x00)
#define FLASH_KEYR    (FLASH + 0x04)
#define FLASH_OPTKEYR (FLASH + 0x08)
#define FLASH_SR      (FLASH + 0x0C)
#define FLASH_CR      (FLASH + 0x10)
#define FLASH_OPTCR   (FLASH + 0x14)
#define FLASH_OPTCR1  (FLASH + 0x18)

#define UART7  0x40007800
#define UART_CR1 (UART7 + 0x00)
#define UART_BRR (UART7 + 0x0C)
#define UART_ISR (UART7 + 0x1C)
#define UART_ICR (UART7 + 0x20)
#define UART_RDR (UART7 + 0x24)
#define UART_TDR (UART7 + 0x28)

#define DMA __attribute__((section(".dma_buffer")))

void hw_init(void);

/* Low level register functions */
void reg_wr (u32 reg, u32 value);
u32  reg_rd (u32 reg);
void reg_set(u32 reg, u32 value);   
void reg_clr(u32 reg, u32 value);   
u16  reg16_rd (u32 reg);
void reg16_clr(u32 reg, u16 value);
void reg16_set(u32 reg, u16 value);
void reg16_wr (u32 reg, u16 value);

#endif
