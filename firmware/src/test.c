/**
 * @file  test.c
 * @brief contains test functions to test the hardware of pnp
 *
 * @author Axel Paolillo <axel.paolillo@agilack.fr>
 * @copyright Agilack (c) 2021
 *
 * @page License
 * This firmware is free software: you can redistribute it and/or modify it
 * under the terms of the GNU Lesser General Public License version 3 as
 * published by the Free Software Foundation. You should have received a
 * copy of the GNU Lesser General Public License along with this program,
 * see LICENSE.md file for more details.
 * This program is distributed WITHOUT ANY WARRANTY.
 */

#include "types.h"
#include "hardware.h"
#include "pnp.h"
#include "test.h"
//#include "rtos/include/FreeRTOS.h"
#include "uart.h"
//#include "main.h"
#include "valve.h"
#include "light.h"

#define OFF 0
#define ON  1


void delay_rtos(int ms);

void adc_test(void)
{
    uint adc_val=0;

    while(1)
    {
        delay_rtos(500);
        reg_set((u32)ADC1_CR2, (1 << 30));      //START CONVERSION
        while (!(reg_rd(ADC1_SR) & (1 << 1)));  //Waiting for end of conversion
        adc_val = reg_rd(ADC1_DR);
        uart_puts("\r\n ADC VAL = ");
        uart_putdec(adc_val);
    }
}

void pump_vacuum_blow_test(void)
{
    reg16_set((u32)TIM4_CCER, (1 << 0));        //Enable BLOW PWM

    //reg16_clr((u32)TIM4_CCER, (1 << 0));        //Disable BLOW PWM

    reg16_set((u32)TIM4_CCER, (1 << 4));        //Enable Vacuum PWM

    //reg16_clr((u32)TIM4_CCER, (1 << 4));        //Disable BLOW PWM
}

void nozzle_test(void)
{
    reg16_set((u32)TIM3_CCER, (1 << 0));        //Enable Nozzle CLK PWM

    //reg16_clr((u32)TIM3_CCER, (1 << 0));        //Disable Nozzle CLK PWM

    reg16_set((u32)TIM3_CCER, (1 << 4));        //Enable Rot-Noz1-CLK PWM

    //reg16_clr((u32)TIM3_CCER, (1 << 4));        //Disable Rot-Noz1-CLK PWM

    reg16_set((u32)TIM3_CCER, (1 << 8));        //Enable Rot-Noz2-CLK PWM

    //reg16_clr((u32)TIM3_CCER, (1 << 8));        //Disable Rot-Noz2-CLK PWM
}

void valve_light_test(void)
{
    while(1)
    {
        valve_right(ON);
        valve_left(ON);
        down_light(ON);
        up_light(ON);
        delay_rtos(3000);
        valve_right(OFF);
        valve_left(OFF);
        down_light(OFF);
        up_light(OFF);
        delay_rtos(3000);
    }
}

void test_hw_encoder(void)
{
    delay_rtos(200);
    if ((reg_rd(GPIO_IDR(GPIOA)) & (1 << 11)) == 0)
    {
        reg_clr((u32)TIM_CNT(TIM1), 0xFFFF);
        uart_puts("Encoder value reset !\r\n");
    }
    uart_puts("Encoder value : ");
    uart_putdec(reg_rd((u32)TIM_CNT(TIM1)));
    uart_puts("\r\n");
}

void encoder_init(void)     //WIP
{
    u32 val;

    reg_set((u32)RCC_APB2ENR, (1 << 14));    //Enbale SYSCFG
    reg_set((u32)RCC_APB2ENR, (1 << 0));     //TIM1 enable

//Add_IO8 PA8
    reg_clr((u32)GPIOA_MODER, 0x30000);       //Clear bits 16 and 17
    reg_set((u32)GPIOA_MODER, (2 << 16));    //AF mode

    val = reg_rd((u32)GPIOA_AFRH);
    val |= (1 << 0);                    //AF1 (TIM1_CH1) on PA8
    reg_wr((u32)GPIOA_AFRH, val);

//Add_IO9 PA9
    reg_clr((u32)GPIOA_MODER, 0xC0000);       //Clear bits 18 and 19
    reg_set((u32)GPIOA_MODER, (2 << 18));    //AF mode

    val = reg_rd((u32)GPIOA_AFRH);
    val |= (1 << 4);                    //AF1 (TIM1_CH2) on PA9
    reg_wr((u32)GPIOA_AFRH, val);

    val = reg_rd((u32)GPIO_PUPD(GPIOA));
    val &= ~(0xF0000);                        //Clear bits 16 to 19
    val |= 0x50000;                            //Pull down on PA8 and PA9
    reg_wr((u32)GPIO_PUPD(GPIOA), val);

//Add_IO11 PA11    Encoder pusher button
    reg_clr((u32)GPIOA_MODER, 0xC00000);       //Clear bits 22 and 23
    reg_set((u32)GPIOA_MODER, (0 << 22));    //Input mode


    reg_set((u32)TIM1_CCMR1, (1 << 0));      //Channel 1 as TI1
    reg_set((u32)TIM1_CCMR1, (1 << 8));      //Channel 2 as TI2

    reg_set((u32)TIM1_SMCR, (3 << 0));        //Encoder mode 3
    reg_set((u32)TIM1_CR1, (1 << 0));        //Enable
}