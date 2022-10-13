/**
 * @file  hardware.c
 * @brief Registers read and write functions
 *
 * @authors Axel Paolillo Axel <axel.paolillo@agilack.fr>
 *          Saint-Genest Gwenael <gwen@agilack.fr>
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

#include "hardware.h"
static void pwm_initilize(void);
static void init_io(void);
static void adc_init(void);
static void Setup_X_A_interupt(void);
static void Setup_X_B_interupt(void);
static void Setup_Y_A_interupt(void);
static void Setup_Y_B_interupt(void);

void hw_init(void)
{
    init_io();
    pwm_initilize();
    adc_init();
    //encoder_init();
    Setup_X_A_interupt();
    Setup_X_B_interupt();
    Setup_Y_A_interupt();
    Setup_Y_B_interupt();
}

static void pwm_initilize(void)
{
    u32 val;
    u32 a_mode;

     /* Activate GPIO controller(s) */
    val = reg_rd((u32)RCC_AHB1ENR);
    val |= (1 << 0); /* GPIO-A */
    reg_wr((u32)RCC_AHB1ENR, val);

    //X_CLK PA0
    a_mode = reg_rd((u32)GPIOA_MODER);
    a_mode |= (2 << 0);                 //PA0 alternate function mode
    reg_wr((u32)GPIOA_MODER, a_mode); 

    val = reg_rd((u32)GPIOA_AFRL);
    val |= (1 << 0);                    //AF1 (TIM2_CH1) on PA0
    reg_wr((u32)GPIOA_AFRL, val);

    val = reg_rd((u32)RCC_APB1ENR);
    val |= (1 << 0);                    //TIM2 enabled
    reg_wr((u32)RCC_APB1ENR, val);

    reg16_set((u32)TIM2_CR1, (1 << 7));     //Auto-reload preload enable
    //reg16_set((u32)TIM2_PSC, 0);          //Prescaler = 0
    reg_wr((u32)TIM2_ARR, 3200);            //Preload = 3200 (Period = 200us)
    reg_wr((u32)TIM2_CCR1, 1600);           //Duty cycle 50 percent
    //reg_set((u32)TIM2_CCMR1, (0 << 0));   //CC1 channel configured as output
    //reg_set((u32)TIM2_CCER, (0 << 1));    //Set polarity to active high
    reg_set((u32)TIM2_CCMR1, (6 << 4));     //Output Compare 1 mode set to PWM mode 1
    //reg_set((u32)TIM2_CCMR1, (0 << 16)); 
    reg_set((u32)TIM2_CCMR1, (1 << 3));     //Preload enable
    //reg16_set((u32)TIM2_CCER, (1 << 0));  //Enable CC1
    reg16_set((u32)TIM2_EGR, (1 << 0));     //Update generation
    ////reg_set((u32)TIM2_BDTR, (1 << 15)); //Main Output Enable

    //Y_CLK PA1
    a_mode = reg_rd((u32)GPIOA_MODER);
    a_mode |= (2 << 2);                 //PA1 alternate function mode
    reg_wr((u32)GPIOA_MODER, a_mode); 

    val = reg_rd((u32)GPIOA_AFRL);
    val |= (2 << 4);                    //AF2 (TIM5_CH2) on PA1
    reg_wr((u32)GPIOA_AFRL, val);

    val = reg_rd((u32)RCC_APB1ENR);
    val |= (1 << 3);                    //TIM5 enabled
    reg_wr((u32)RCC_APB1ENR, val);

    reg16_set((u32)TIM5_CR1, (1 << 7));     //Auto-reload preload enable
    reg_wr((u32)TIM5_ARR, 3200);            //Preload = 3200 (Period = 200us)
    reg_wr((u32)TIM5_CCR2, 1600);           //Duty cycle 50 percent
    reg_set((u32)TIM5_CCMR1, (6 << 12));    //Output Compare 2 mode set to PWM mode 1
    reg_set((u32)TIM5_CCMR1, (1 << 11));    //Preload enable
    //reg16_set((u32)TIM5_CCER, (1 << 4));  //Enable CC2

    reg16_set((u32)TIM5_EGR, (1 << 0));     //Update generation

//PUMPS INIT

     /* Activate GPIO controller(s) */
    val = reg_rd((u32)RCC_AHB1ENR);
    val |= (1 << 3); /* GPIO-D */
    reg_wr((u32)RCC_AHB1ENR, val);
    
    //BLOW_PWM PD12
    reg_clr((u32)GPIOD_MODER, 0x3000000);       //Clear bits 24 and 25
    reg_set((u32)GPIOD_MODER, (2 << 24));    //AF mode

    val = reg_rd((u32)GPIOD_AFRH);
    val |= (2 << 16);                    //AF2 (TIM4_CH1) on PD12
    reg_wr((u32)GPIOD_AFRH, val);

    val = reg_rd((u32)RCC_APB1ENR);
    val |= (1 << 2);                    //TIM4 enabled
    reg_wr((u32)RCC_APB1ENR, val);

    reg16_set((u32)TIM4_CR1, (1 << 7));     //Auto-reload preload enable    (ARPE)
    //reg16_set((u32)TIM4_PSC, 1);          //Prescaler = 1
    reg_wr((u32)TIM4_ARR, 1000);            //Preload = 1000 (Period = 62,5us / Fq = 16KHz)
    reg_wr((u32)TIM4_CCR1, 103);           //Duty cycle 10,3%
    reg_set((u32)TIM4_CCMR1, (6 << 4));    //Output Compare 1 mode set to PWM mode 1
    reg_set((u32)TIM4_CCMR1, (1 << 3));    //Preload enable
    //reg16_set((u32)TIM4_CCER, (1 << 0));  //Enable CC2

    //VACUUM_PWM PD13
    reg_clr((u32)GPIOD_MODER, 0xC000000);       //Clear bits 26 and 27
    reg_set((u32)GPIOD_MODER, (2 << 26));    //AF mode

    val = reg_rd((u32)GPIOD_AFRH);
    val |= (2 << 20);                    //AF2 (TIM4_CH2) on PD13
    reg_wr((u32)GPIOD_AFRH, val);

    reg_wr((u32)TIM4_CCR2, 573);           //Duty cycle 57,3%
    reg_set((u32)TIM4_CCMR1, (6 << 12));    //Output Compare 2 mode set to PWM mode 1
    reg_set((u32)TIM4_CCMR1, (1 << 11));    //Preload enable

    reg16_set((u32)TIM4_EGR, (1 << 0));     //Update generation

//NOZZLE INIT

     /* Activate GPIO controller(s) */
    val = reg_rd((u32)RCC_AHB1ENR);
    val |= (1 << 1); /* GPIO-B */
    reg_wr((u32)RCC_AHB1ENR, val);
    
    //NOZZLE_CLK PA6
    reg_clr((u32)GPIOA_MODER, 0x3000);       //Clear bits 12 and 13
    reg_set((u32)GPIOA_MODER, (2 << 12));    //AF mode

    val = reg_rd((u32)GPIOA_AFRL);
    val |= (2 << 24);                    //AF2 (TIM3_CH1) on PA6
    reg_wr((u32)GPIOA_AFRL, val);

    val = reg_rd((u32)RCC_APB1ENR);
    val |= (1 << 1);                    //TIM3 enabled
    reg_wr((u32)RCC_APB1ENR, val);

    reg16_set((u32)TIM3_CR1, (1 << 7));     //Auto-reload preload enable    (ARPE)
    //reg16_set((u32)TIM3_PSC, 1);          //Prescaler = 1
    reg_wr((u32)TIM3_ARR, 32000);            //Preload = 32000 (Period = 2000us / Fq = 500Hz)
    reg_wr((u32)TIM3_CCR1, 16000);           //Duty cycle 50%
    reg_set((u32)TIM3_CCMR1, (6 << 4));    //Output Compare 1 mode set to PWM mode 1
    reg_set((u32)TIM3_CCMR1, (1 << 3));    //Preload enable

    //ROT_NOZ1_CLK PA7
    reg_clr((u32)GPIOA_MODER, 0xC000);       //Clear bits 14 and 15
    reg_set((u32)GPIOA_MODER, (2 << 14));    //AF mode

    val = reg_rd((u32)GPIOA_AFRL);
    val |= (2 << 28);                    //AF2 (TIM3_CH2) on PA7
    reg_wr((u32)GPIOA_AFRL, val);

    reg_wr((u32)TIM3_CCR2, 16000);           //Duty cycle 50%
    reg_set((u32)TIM3_CCMR1, (6 << 12));    //Output Compare 2 mode set to PWM mode 1
    reg_set((u32)TIM3_CCMR1, (1 << 11));    //Preload enable

    //ROT_NOZ2_CLK PB0
    reg_clr((u32)GPIOB_MODER, 0x3);       //Clear bits 0 and 1
    reg_set((u32)GPIOB_MODER, (2 << 0));    //AF mode

    val = reg_rd((u32)GPIOB_AFRL);
    val |= (2 << 0);                    //AF2 (TIM3_CH3) on PB0
    reg_wr((u32)GPIOB_AFRL, val);

    reg_wr((u32)TIM3_CCR3, 16000);           //Duty cycle 50%
    reg_set((u32)TIM3_CCMR2, (6 << 4));    //Output Compare 3 mode set to PWM mode 1
    reg_set((u32)TIM3_CCMR2, (1 << 3));    //Preload enable

    reg16_set((u32)TIM3_EGR, (1 << 0));     //Update generation

//NEEDLE INIT on PB14 = TIM12_CH1

    /* Activate GPIO controller(s) */
    val = reg_rd((u32)RCC_AHB1ENR);
    val |= (1 << 1); /* GPIO-B */
    reg_wr((u32)RCC_AHB1ENR, val);
    
    //NEEDLE PB14
    reg_clr((u32)GPIOB_MODER, 0x30000000);       //Clear bits 28 and 29
    reg_set((u32)GPIOB_MODER, (2 << 28));    //AF mode

    val = reg_rd((u32)GPIOB_AFRH);
    val |= (9 << 24);                    //AF9 (TIM12_CH1) on PB14
    reg_wr((u32)GPIOB_AFRH, val);

    val = reg_rd((u32)RCC_APB1ENR);
    val |= (1 << 6);                    //TIM12 enabled
    reg_wr((u32)RCC_APB1ENR, val);

    reg16_set((u32)TIM12_CR1, (1 << 7));     //Auto-reload preload enable    (ARPE)
    //reg16_set((u32)TIM12_PSC, 1);          //Prescaler = 1
    reg16_wr((u32)TIM12_ARR, 1000);            //Preload = 1000 (Period = 62,5us / Fq = 16KHz)
    reg16_wr((u32)TIM12_CCR1, 700);           //Duty cycle 70%
    reg_set((u32)TIM12_CCMR1, (6 << 4));    //Output Compare 1 mode set to PWM mode 1
    reg_set((u32)TIM12_CCMR1, (1 << 3));    //Preload enable
    //reg16_set((u32)TIM4_CCER, (1 << 0));  //Enable CC2

    reg16_set((u32)TIM12_EGR, (1 << 0));     //Update generation


    //Start timers
    reg16_set((u32)TIM2_CR1, (1 << 0));     //Start timer 2
    reg16_set((u32)TIM5_CR1, (1 << 0));     //Start timer 5
    reg16_set((u32)TIM4_CR1, (1 << 0));     //Start timer 4
    reg16_set((u32)TIM3_CR1, (1 << 0));     //Start timer 3
    reg16_set((u32)TIM12_CR1, (1 << 0));    //Start timer 12
}

static void init_io(void)
{
    u32 val;

    /* Activate GPIO controller(s) */
    val = reg_rd((u32)RCC_AHB1ENR);
    val |= (1 << 0); /* GPIO-A */
    val |= (1 << 1); /* GPIO-B */
    val |= (1 << 2); /* GPIO-C */
    val |= (1 << 3); /* GPIO-D */
    val |= (1 << 4); /* GPIO-E */
    val |= (1 << 5); /* GPIO-F */
    reg_wr((u32)RCC_AHB1ENR, val);

//XYZ_EN PA4
    reg_clr((u32)GPIOA_MODER, 0x300);       //Clear bits 8 and 9
    reg_set((u32)GPIOA_MODER, (1 << 8));    //Output mode

    reg_set((u32)GPIOA_BSRR, (1 << 4));     //PA4 HIGH = ENABLE

//BCD_EN PA5
    reg_clr((u32)GPIOA_MODER, 0xC00);       //Clear bits 10 and 11
    reg_set((u32)GPIOA_MODER, (1 << 10));    //Output mode

    reg_set((u32)GPIOA_BSRR, (1 << 5));     //PA5 HIGH = ENABLE

//X_DIR PC0
    reg_clr((u32)GPIOC_MODER, 0x3);         //Clear bits 0 and 1
    reg_set((u32)GPIOC_MODER, (1 << 0));    //Output mode

    reg_set((u32)GPIOC_BSRR, (1 << 0));     //PC0 HIGH

//Y_DIR PC1
    reg_clr((u32)GPIOC_MODER, 0xC);         //Clear bits 2 and 3
    reg_set((u32)GPIOC_MODER, (1 << 2));    //Output mode

    reg_set((u32)GPIOC_BSRR, (1 << 1));     //PC1 HIGH

//Nozzle_DIR PF13
    reg_clr((u32)GPIOF_MODER, 0xC000000);     //Clear bits 26 and 27
    reg_set((u32)GPIOF_MODER, (1 << 26));     //Output mode

    reg_set((u32)GPIOF_BSRR, (1 << 13));     //PF13 HIGH = ENABLE

//Rot-Noz2_DIR PF14
    reg_clr((u32)GPIOF_MODER, 0x30000000);     //Clear bits 28 and 29
    reg_set((u32)GPIOF_MODER, (1 << 28));     //Output mode

    reg_set((u32)GPIOF_BSRR, (1 << 14));     //PF14 HIGH = ENABLE

//Rot-Noz1_DIR PF15
    reg_clr((u32)GPIOF_MODER, 0xC0000000);     //Clear bits 30 and 31
    reg_set((u32)GPIOF_MODER, (1 << 30));     //Output mode

    reg_set((u32)GPIOF_BSRR, (1 << 15));     //PF15 HIGH = ENABLE

//X_A PE14
    reg_clr((u32)GPIOE_MODER, 0x30000000);     //Clear bits 28 and 29
    reg_set((u32)GPIOE_MODER, (0 << 28));      //Input mode

//X_B PE15
    reg_clr((u32)GPIOE_MODER, 0xC0000000);     //Clear bits 30 and 31
    reg_set((u32)GPIOE_MODER, (0 << 30));      //Input mode

//Y_A PB12
    reg_clr((u32)GPIOB_MODER, 0x3000000);      //Clear bits 24 and 25
    reg_set((u32)GPIOB_MODER, (0 << 24));      //Input mode

//Y_B PB13
    reg_clr((u32)GPIOB_MODER, 0xC000000);     //Clear bits 26 and 27
    reg_set((u32)GPIOB_MODER, (0 << 26));      //Input mode

//X_lim PC10
    reg_clr((u32)GPIOC_MODER, 0x300000);     //Clear bits 20 and 21
    reg_set((u32)GPIOC_MODER, (0 << 20));      //Input mode

//Y_lim PC11
    reg_clr((u32)GPIOC_MODER, 0xC00000);     //Clear bits 22 and 23
    reg_set((u32)GPIOC_MODER, (0 << 22));      //Input mode

    val = reg_rd((u32)GPIO_PUPD(GPIOC));
    val &= ~(0xF00000);                        //Clear bits 20 to 23
    val |= 0x500000;                            //Pull up on PC10 and PC11
    reg_wr((u32)GPIO_PUPD(GPIOC), val);

//Valve_Right PB15
    reg_clr((u32)GPIOB_MODER, 0xC0000000);     //Clear bits 30 and 31
    reg_set((u32)GPIOB_MODER, (1 << 30));      //Output mode

//Valve_Left PD8
    reg_clr((u32)GPIOD_MODER, 0x30000);     //Clear bits 16 and 17
    reg_set((u32)GPIOD_MODER, (1 << 16));      //Output mode

//Downward_Light PD9
    reg_clr((u32)GPIOD_MODER, 0xC0000);     //Clear bits 18 and 19
    reg_set((u32)GPIOD_MODER, (1 << 18));      //Output mode

//Upward_Light PD10
    reg_clr((u32)GPIOD_MODER, 0x300000);     //Clear bits 20 and 21
    reg_set((u32)GPIOD_MODER, (1 << 20));      //Output mode

//Right_Nozzle_Pressure PC4
    reg_clr((u32)GPIOC_MODER, 0x300);     //Clear bits 8 and 9
    reg_set((u32)GPIOC_MODER, (3 << 8));      //Analog mode

//Left_Nozzle_Pressure PC5
    reg_clr((u32)GPIOC_MODER, 0xC00);     //Clear bits 10 and 11
    reg_set((u32)GPIOC_MODER, (3 << 10));      //Analog mode
}

static void adc_init(void)
{
    reg_set((u32)RCC_APB2ENR, (1 << 8));    //Enbale ADC1 CLK
    reg_clr((u32)ADC1_CR1, (1 << 8));       //SCAN mode disabled
    reg_clr((u32)ADC1_CR1, (3 << 24));      //Resolution set to 12bit
    reg_clr((u32)ADC1_SQR1, (0xF << 20));   //Number of conversions in sequece = 1
    reg_set((u32)ADC1_SQR3, (14 << 0));     //Set 1st conversion on channel 14
    reg_clr((u32)ADC1_CR2, (1 << 1));       //Disable continuous mode (single conversion)
    reg_clr((u32)ADC1_CR2, (1 << 11));      //Date aligned right
    reg_set((u32)ADC_SMPR1, (0b111 << 12)); //Sample time selection to 480 cycle (max), is this on the right channel ?

    reg_set((u32)ADC1_CR2, (1 << 0));       //Enable ADC1
}

//Setup interruption from the X_A signal (PE14)
static void Setup_X_A_interupt(void)
{
    unsigned long int val;

    reg_set((u32)RCC_APB2ENR, (1 << 14));    //Enbale SYSCFG

    *(volatile unsigned long int*) (0xE000E100 + 0x4) = (1 << 8); //Activate EXTI15_10 into NVIC
    /* Configure External interrupt 14 to use PE14 */
    val = *(volatile unsigned long int*) (SYSCFG + 0x14);
    val &= 0xF0FF; /* Clear EXTI14 */
    val |= (4 << 8); /* EXTI14 is now PE14 */
    *(volatile unsigned long int*) ((unsigned long) SYSCFG + 0x14) = val;
    /* Trigger configuration */
    reg_set(EXTI + 0x08, (1 << 14)); /* Rising  trigger */

    reg_set(EXTI + 0x0C, (1 << 14)); /* Falling trigger */
    /* Activate EXTI8 (IMR) */
    reg_set(EXTI + 0x00, 1 << 14);
}

//Setup interruption for the X_B signal (PE15)
static void Setup_X_B_interupt(void)
{
    unsigned long int val;

    reg_set((u32)RCC_APB2ENR, (1 << 14));    //Enbale SYSCFG

    *(volatile unsigned long int*) (0xE000E100 + 0x4) = (1 << 8); //Activate EXTI15_10 into NVIC
    /* Configure External interrupt 15 to use PE15 */
    val = *(volatile unsigned long int*) (SYSCFG + 0x14);
    val &= 0x0FFF; /* Clear EXTI15 */
    val |= (4 << 12); /* EXTI15 is now PE15 */
    *(volatile unsigned long int*) ((unsigned long) SYSCFG + 0x14) = val;
    /* Trigger configuration */
    reg_set(EXTI + 0x08, (1 << 15)); /* Rising  trigger */

    reg_set(EXTI + 0x0C, (1 << 15)); /* Falling trigger */

    /* Activate EXTI9 (IMR) */
    reg_set(EXTI + 0x00, 1 << 15);
}

//Setup interruption from the Y_A signal (PB12)
static void Setup_Y_A_interupt(void)
{
    unsigned long int val;

    reg_set((u32)RCC_APB2ENR, (1 << 14));    //Enbale SYSCFG

    *(volatile unsigned long int*) (0xE000E100 + 0x4) = (1 << 8); //Activate EXTI15_10 into NVIC
    /* Configure External interrupt 12 to use PB12 */
    val = *(volatile unsigned long int*) (SYSCFG + 0x14);
    val &= 0xFFF0; /* Clear EXTI12 */
    val |= (1 << 0); /* EXTI12 is now PB12 */
    *(volatile unsigned long int*) ((unsigned long) SYSCFG + 0x14) = val;
    /* Trigger configuration */
    reg_set(EXTI + 0x08, (1 << 12)); /* Rising  trigger */

    reg_set(EXTI + 0x0C, (1 << 12)); /* Falling trigger */
    /* Activate EXTI12 (IMR) */
    reg_set(EXTI + 0x00, 1 << 12);
}

//Setup interruption for the Y_B signal (PB13)
static void Setup_Y_B_interupt(void)
{
    unsigned long int val;

    reg_set((u32)RCC_APB2ENR, (1 << 14));    //Enbale SYSCFG

    *(volatile unsigned long int*) (0xE000E100 + 0x4) = (1 << 8); //Activate EXTI15_10 into NVIC
    /* Configure External interrupt 13 to use PB13 */
    val = *(volatile unsigned long int*) (SYSCFG + 0x14);
    val &= 0xFF0F; /* Clear EXTI13 */
    val |= (1 << 4); /* EXTI13 is now PB13 */
    *(volatile unsigned long int*) ((unsigned long) SYSCFG + 0x14) = val;
    /* Trigger configuration */
    reg_set(EXTI + 0x08, (1 << 13)); /* Rising  trigger */

    reg_set(EXTI + 0x0C, (1 << 13)); /* Falling trigger */

    /* Activate EXTI13 (IMR) */
    reg_set(EXTI + 0x00, 1 << 13);
}

/**
 * @brief Read the value of a 32bits memory mapped register
 *
 * @param reg Address of the register to read
 * @return u32 Value of the register
 */
u32 reg_rd(u32 reg)
{
    return ( *(volatile u32 *)reg );
}

/**
 * @brief Write a value to a 32bits memory mapped register
 *
 * @param reg Address of the register to write
 * @param value Value to write into the register
 */
void reg_wr(u32 reg, u32 value)
{
    *(volatile u32 *)reg = value;
}

/**
 * @brief Modify a 32bits memory mapped register by setting some bits
 *
 * @param reg Address of the register to write
 * @param value Mask of the bits to set
 */
void reg_set(u32 reg, u32 value)
{
    *(volatile u32 *)reg = ( *(volatile u32 *)reg | value );
}

/**
 * @brief Modify a 32bits memory mapped register by clearing some bits
 *
 * @param reg Address of the register to write
 * @param value Mask of the bits to clear
 */
void reg_clr(u32 reg, u32 value)
{
    *(volatile u32 *)reg = ( *(volatile u32 *)reg & ~value );
}

/**
 * @brief Read the value of a 16bits memory mapped register
 *
 * @param  reg Address of the register to read
 * @return u16 Value of the register
 */
u16 reg16_rd(u32 reg)
{
        return ( *(volatile u16 *)reg );
}


/**
 * @brief Write a value to a 16bits memory mapped register
 *
 * @param reg Address of the register to write
 * @param value Value to write into the register
 */
void reg16_wr(u32 reg, u16 value)
{
        *(volatile u16 *)reg = value;
}

/**
 * @brief Modify a 16bits memory mapped register by clearing some bits
 *
 * @param reg Address of the register to write
 * @param value Mask of the bits to clear
 */
void reg16_clr(u32 reg, u16 value)
{
        *(volatile u16 *)reg = ( *(volatile u16 *)reg & ~value );
}

/**
 * @brief Modify a 16bits memory mapped register by setting some bits
 *
 * @param reg Address of the register to write
 * @param value Value to write into the register
 */
void reg16_set(u32 reg, u16 value)
{
        *(volatile u16 *)reg = ( *(volatile u16 *)reg | value );
}
