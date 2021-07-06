/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef unsigned long u32;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LINEMAX 200

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

UART_HandleTypeDef huart7;

/* USER CODE BEGIN PV */

static unsigned int encoder_pos_x=0;	//X position with encoder
static unsigned int encoder_pos_y=0;	//Y position with encoder
static char buff[1] = "a";
static char flag_crlf = 0;
static int rx_index = 0;
static char rx_buffer[LINEMAX];   // Local holding buffer to build line
static unsigned int target_pos_x=0;
static unsigned int target_pos_y=0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_UART7_Init(void);
/* USER CODE BEGIN PFP */

void delay_us(int us);
void Setup_X_A_interupt(void);
void Setup_X_B_interupt(void);
void Setup_Y_A_interupt(void);
void Setup_Y_B_interupt(void);
void reg_set(unsigned long int reg, unsigned long int value);
void reg_wr(u32 reg, u32 value);
u32 reg_rd(u32 reg);
void flag_crlf_handler(void);
void UART_Write(char *str);
void x_test(void);
void PnP_init(void);
void x_goto_2(int pos_encoder_target);
void y_goto_2(int pos_encoder_target);
void nozzle_down_1(void);
void nozzle_down_2(void);
void nozzle_up_1(void);
void nozzle_up_2(void);
void nozzle_reset(void);
void amazing_square(void);




/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_UART7_Init();
  /* USER CODE BEGIN 2 */

  /* Activate SYSCFG */
  	RCC->APB2ENR |= (1 << 14);


  	Setup_X_A_interupt();
  	Setup_X_B_interupt();
  	Setup_Y_A_interupt();
  	Setup_Y_B_interupt();


	HAL_UART_Receive_IT(&huart7, (uint8_t*) buff, 1);

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET); //Enable XYZ
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET); //Enable BCD


	PnP_init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{

		flag_crlf_handler();




    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_UART7;
  PeriphClkInitStruct.Uart7ClockSelection = RCC_UART7CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief UART7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART7_Init(void)
{

  /* USER CODE BEGIN UART7_Init 0 */

  /* USER CODE END UART7_Init 0 */

  /* USER CODE BEGIN UART7_Init 1 */

  /* USER CODE END UART7_Init 1 */
  huart7.Instance = UART7;
  huart7.Init.BaudRate = 115200;
  huart7.Init.WordLength = UART_WORDLENGTH_8B;
  huart7.Init.StopBits = UART_STOPBITS_1;
  huart7.Init.Parity = UART_PARITY_NONE;
  huart7.Init.Mode = UART_MODE_TX_RX;
  huart7.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart7.Init.OverSampling = UART_OVERSAMPLING_16;
  huart7.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart7.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart7) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART7_Init 2 */

  /* USER CODE END UART7_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, X_DIR_Pin|Y_DIR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, X_CLK_Pin|Y_CLK_Pin|XYZ_ENABLE_Pin|BCD_ENABLE_Pin
                          |NOZZLE_CLK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(NOZZLE_DIR_GPIO_Port, NOZZLE_DIR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : X_DIR_Pin Y_DIR_Pin */
  GPIO_InitStruct.Pin = X_DIR_Pin|Y_DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : X_CLK_Pin Y_CLK_Pin XYZ_ENABLE_Pin BCD_ENABLE_Pin
                           NOZZLE_CLK_Pin */
  GPIO_InitStruct.Pin = X_CLK_Pin|Y_CLK_Pin|XYZ_ENABLE_Pin|BCD_ENABLE_Pin
                          |NOZZLE_CLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : NOZZLE_DIR_Pin */
  GPIO_InitStruct.Pin = NOZZLE_DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(NOZZLE_DIR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : X_A_Pin X_B_Pin */
  GPIO_InitStruct.Pin = X_A_Pin|X_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : Y_A_Pin Y_B_Pin */
  GPIO_InitStruct.Pin = Y_A_Pin|Y_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : X_Lim_Pin Y_Lim_Pin */
  GPIO_InitStruct.Pin = X_Lim_Pin|Y_Lim_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
//Creates a delay. Legnth is chosen by user through "us"
void delay_us(int us)
{
		for(int i=0;i<us*5;i++)
			  {
				  asm volatile("nop");

			  }


}

//Setup interruption from the X_A signal (PE14)
void Setup_X_A_interupt(void) {
	unsigned long int val;
	*(volatile unsigned long int*) (0xE000E100 + 0x4) = (1 << 8); //Activate EXTI15_10 into NVIC
	/* Configure External interrupt 14 to use PE14 */
	val = *(volatile unsigned long int*) (SYSCFG_BASE + 0x14);
	val &= 0xF0FF; /* Clear EXTI14 */
	val |= (4 << 8); /* EXTI14 is now PE14 */
	*(volatile unsigned long int*) ((unsigned long) SYSCFG_BASE + 0x14) = val;
	/* Trigger configuration */
	reg_set(EXTI_BASE + 0x08, (1 << 14)); /* Rising  trigger */

	reg_set(EXTI_BASE + 0x0C, (1 << 14)); /* Falling trigger */
	/* Activate EXTI8 (IMR) */
	reg_set(EXTI_BASE + 0x00, 1 << 14);
}

//Setup interruption for the X_B signal (PE15)
void Setup_X_B_interupt(void) {
	unsigned long int val;
	*(volatile unsigned long int*) (0xE000E100 + 0x4) = (1 << 8); //Activate EXTI15_10 into NVIC
	/* Configure External interrupt 15 to use PE15 */
	val = *(volatile unsigned long int*) (SYSCFG_BASE + 0x14);
	val &= 0x0FFF; /* Clear EXTI15 */
	val |= (4 << 12); /* EXTI15 is now PE15 */
	*(volatile unsigned long int*) ((unsigned long) SYSCFG_BASE + 0x14) = val;
	/* Trigger configuration */
	reg_set(EXTI_BASE + 0x08, (1 << 15)); /* Rising  trigger */

	reg_set(EXTI_BASE + 0x0C, (1 << 15)); /* Falling trigger */

	/* Activate EXTI9 (IMR) */
	reg_set(EXTI_BASE + 0x00, 1 << 15);
}

//Setup interruption from the Y_A signal (PB12)
void Setup_Y_A_interupt(void) {
	unsigned long int val;
	*(volatile unsigned long int*) (0xE000E100 + 0x4) = (1 << 8); //Activate EXTI15_10 into NVIC
	/* Configure External interrupt 12 to use PB12 */
	val = *(volatile unsigned long int*) (SYSCFG_BASE + 0x14);
	val &= 0xFFF0; /* Clear EXTI12 */
	val |= (1 << 0); /* EXTI12 is now PB12 */
	*(volatile unsigned long int*) ((unsigned long) SYSCFG_BASE + 0x14) = val;
	/* Trigger configuration */
	reg_set(EXTI_BASE + 0x08, (1 << 12)); /* Rising  trigger */

	reg_set(EXTI_BASE + 0x0C, (1 << 12)); /* Falling trigger */
	/* Activate EXTI12 (IMR) */
	reg_set(EXTI_BASE + 0x00, 1 << 12);
}

//Setup interruption for the Y_B signal (PB13)
void Setup_Y_B_interupt(void) {
	unsigned long int val;
	*(volatile unsigned long int*) (0xE000E100 + 0x4) = (1 << 8); //Activate EXTI15_10 into NVIC
	/* Configure External interrupt 13 to use PB13 */
	val = *(volatile unsigned long int*) (SYSCFG_BASE + 0x14);
	val &= 0xFF0F; /* Clear EXTI13 */
	val |= (1 << 4); /* EXTI13 is now PB13 */
	*(volatile unsigned long int*) ((unsigned long) SYSCFG_BASE + 0x14) = val;
	/* Trigger configuration */
	reg_set(EXTI_BASE + 0x08, (1 << 13)); /* Rising  trigger */

	reg_set(EXTI_BASE + 0x0C, (1 << 13)); /* Falling trigger */

	/* Activate EXTI13 (IMR) */
	reg_set(EXTI_BASE + 0x00, 1 << 13);
}

//Interruptions from pins 15 to 10
void EXTI15_10_IRQHandler(void) {
	u32 val;
	val = reg_rd(EXTI_BASE + 0x14);		//Checks which interruption is called

	reg_wr(EXTI_BASE + 0x14, val & 0xFC00); // Clear pending register from 15 to 10

	if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_10) == 0)	// X limit switch closed
	//if ( (GPIOC->IDR & (1 << 10)) == 0)
		encoder_pos_x = 0;							//Reset X coordinate

	if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_11) == 0)	// Y limit switch closed
	//if ( (GPIOC->IDR & (1 << 11)) == 0)
		encoder_pos_y = 0;							//Reset Y coordinate

	if (val & (1 << 14))		//EXTI14
			{
		if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_14))			//Rising edge on X_A
				{
			if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_15))				//X_B is HIGH
					{
				encoder_pos_x++;
			} else if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_15) == 0)	//X_B is LOW

					{
				encoder_pos_x--;
			}
		} else if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_14) == 0)//Falling edge on X_A
				{
			if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_15))				//X_B is HIGH
					{
				encoder_pos_x--;
			} else if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_15) == 0)	//X_B is LOW

					{
				encoder_pos_x++;
			}
		}
	}


	if (val & (1 << 15))		//EXTI15
			{
		if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_15))			//Rising edge on X_B
				{
			if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_14))				//X_A is HIGH
					{
				encoder_pos_x--;
			} else if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_14) == 0)	//X_A is LOW
					{
				encoder_pos_x++;
			}
		} else if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_15) == 0)//Falling edge on X_B
				{
			if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_14))				//X_A is HIGH
					{
				encoder_pos_x++;
			} else if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_14) == 0)	//X_A is LOW
					{
				encoder_pos_x--;
			}
		}
	}


	if(val & (1 << 12))		//R or F on Y_A
		{
			if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12))		//Rising edge on Y_A
			{
				if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13))	//Y_B is HIGH
					encoder_pos_y--;
				else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13)==0)	//Y_B is LOW
					encoder_pos_y++;
			}
			else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12)==0)
			{
				if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13))
								encoder_pos_y++;
				else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13)==0)
								encoder_pos_y--;
			}
		}

	if(val & (1 << 13))		//R or F on Y_B
	{
		if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13))		//Rising edge on Y_B
		{
			if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12))	//Y_A is HIGH
				encoder_pos_y++;
			else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12)==0)	//Y_A is LOW
				encoder_pos_y--;
		}
		else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13)==0)
		{
			if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12))
							encoder_pos_y--;
			else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12)==0)
							encoder_pos_y++;
		}
	}
}

//Sets chosen bits of a 32bit register using value
void reg_set(unsigned long int reg, unsigned long int value) {
	*(volatile unsigned long int*) reg = (*(volatile unsigned long int*) reg
			| value);
}

//Overwrite a 32bit register with given value
void reg_wr(u32 reg, u32 value) {
	*(volatile u32*) reg = value;
}

//Returns a 32bit register from provided address
u32 reg_rd(u32 reg) {
	return (*(volatile u32*) reg);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {

	if (buff[0] >= 0 && buff[0] <= 127) {
		switch (buff[0]) {

		case 8:
			HAL_UART_Transmit_IT(&huart7, (uint8_t*) "\b \b", 3);
			//HAL_UART_Transmit_IT(&huart7, (uint8_t *) space, 1);
			//HAL_UART_Transmit_IT(&huart7, (uint8_t *) buff, 1);
			if (rx_index > 0)
				rx_index--;
			break;

		case 13:
			//HAL_UART_Transmit_IT(&huart7, (uint8_t *) buff, 1);
			HAL_UART_Transmit_IT(&huart7, (uint8_t*) "\r\n", 2);
			flag_crlf = 1;
			break;

		default:
			HAL_UART_Transmit_IT(&huart7, (uint8_t*) buff, 1);

			break;

		}

	}

	if (rx_index < LINEMAX && buff[0] != 8) {

		rx_buffer[rx_index] = buff[0];
		rx_index++;
	}

	HAL_UART_Receive_IT(&huart7, (uint8_t*) buff, 1);
}

void flag_crlf_handler(void) {
	int strlength = 0;
	if (flag_crlf) {

		flag_crlf = 0;
		strlength = rx_index;
		rx_index = 0;
		rx_buffer[strlength - 1] = '\0';


		/*Commands with arguments*/
		if (strchr(rx_buffer, ' ')) {

			//UART_Write("SPAAAAAAAACE\r\n");
			if (strncmp(rx_buffer, "move ", 5) == 0) {
				char *args = &(rx_buffer[5]);
				if(strncmp(args, "x:", 2) == 0)
				{
					//UART_Write("MOVE X OR X and Y DETECTED\r\n");
					if (strchr(args, ' '))
					{
						char *args2 = strchr(args,' ');
						int x_digits = strlen(args)-strlen(args2)-2;
						int y_digits = strlen(args2)-3;

						int x_decimal=0;
						for(int i=0;i<x_digits;i++)
						{
							x_decimal += (args[2+i]-'0')*pow(10,x_digits-1-i);
						}
						target_pos_x=x_decimal;

						int y_decimal=0;
						for(int i=0;i<y_digits;i++)
						{
							y_decimal += (args2[3+i]-'0')*pow(10,y_digits-1-i);
						}
						target_pos_y=y_decimal;

						UART_Write("MOVE X AND Y DETECTED\r\n");
						x_goto_2(target_pos_x);
						y_goto_2(target_pos_y);

					}
					else
					{
						int x_digits = strlen(rx_buffer)-7;
						int x_decimal=0;
						for(int i=0;i<x_digits;i++)
						{
							x_decimal += (args[2+i]-'0')*pow(10,x_digits-1-i);
						}
						target_pos_x=x_decimal;
						UART_Write("MOVE ONLY X DETECTED\r\n");
						x_goto_2(target_pos_x);
					}
				}
				else if(strncmp(args, "y:", 2) == 0)
				{
					int y_digits = strlen(rx_buffer)-7;
					int y_decimal=0;
					for(int i=0;i<y_digits;i++)
					{
						y_decimal += (args[2+i]-'0')*pow(10,y_digits-1-i);
					}
					target_pos_y=y_decimal;
					UART_Write("MOVE ONLY Y DETECTED\r\n");
					y_goto_2(target_pos_y);
				}
				else
				{
					UART_Write("MOVE ERROR\r\n");
				}
			}
			if (strncmp(rx_buffer, "nozzle ", 7) == 0)
			{
				char *args = &(rx_buffer[7]);
				if(strncmp(args, "move ", 5) == 0)
				{

					UART_Write("NOZZLE MOVE DETECTED\r\n");
					char *args2 = &(args[5]);
					if(strncmp(args2, "1", 1) == 0)
					{
						if (strchr(args2, ' '))
						{
							char *args3 = &(args2[2]);
							int z_digits = strlen(rx_buffer)-14;
							int z_decimal=0;
							for(int i=0;i<z_digits;i++)
							{
								z_decimal += (args3[i]-'0')*pow(10,z_digits-1-i);
							}
							UART_Write("MOVE NOZZLE 1 DETECTED\r\n");
						}
					}
					else if(strncmp(args2, "2", 1) == 0)
					{
						if (strchr(args2, ' '))
						{
							char *args3 = &(args2[2]);
							int z_digits = strlen(rx_buffer)-14;
							int z_decimal=0;
							for(int i=0;i<z_digits;i++)
							{
								z_decimal += (args3[i]-'0')*pow(10,z_digits-1-i);
							}

							UART_Write("MOVE NOZZLE 2 DETECTED\r\n");
						}
					}
				}
				else if(strncmp(args, "down ", 5) == 0)
				{
					char *args2 = &(args[5]);
					if(strncmp(args2, "1", 1) == 0)
					{
						nozzle_down_1();
					}
					else if(strncmp(args2, "2", 1)==0)
					{
						nozzle_down_2();
					}
				}
				else if(strncmp(args, "up ", 3) == 0)
				{
					char *args2 = &(args[3]);
					if(strncmp(args2, "1", 1) == 0)
					{
						nozzle_up_1();
					}
					else if(strncmp(args2, "2", 1)==0)
					{
						nozzle_up_2();
					}
				}
				else if(strncmp(args, "reset", 5) == 0)
				{
					nozzle_reset();
				}
				else if(strncmp(args, "rotate ", 7) == 0)
				{
					UART_Write("NOZZLE ROTATE DETECTED\r\n");

					char *args2 = &(args[7]);
					if(strncmp(args2, "1", 1) == 0)
					{
						if (strchr(args2, ' '))
						{
							char *args3 = &(args2[2]);
							int z_digits = strlen(rx_buffer)-16;
							int z_decimal=0;
							for(int i=0;i<z_digits;i++)
							{
								z_decimal += (args3[i]-'0')*pow(10,z_digits-1-i);
							}
							UART_Write("ROTATE NOZZLE 1 DETECTED\r\n");
						}
					}
					else if(strncmp(args2, "2", 1) == 0)
					{
						if (strchr(args2, ' '))
						{
							char *args3 = &(args2[2]);
							int z_digits = strlen(rx_buffer)-16;
							int z_decimal=0;
							for(int i=0;i<z_digits;i++)
							{
								z_decimal += (args3[i]-'0')*pow(10,z_digits-1-i);
							}
							UART_Write("ROTATE NOZZLE 2 DETECTED\r\n");
						}
					}
				}
			}
			UART_Write("DONE\r\n");
		}

		/*Commands without arguments*/
		else {
			if (strcmp(rx_buffer, "enable-xyz") == 0) {

				UART_Write("WOW ENABLE\r\n");
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);	//XYZ ENABLE
			}
			else if (strcmp(rx_buffer, "disable-xyz") == 0) {

				UART_Write("WOW DISABLE\r\n");
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);	//XYZ DISABLE
			}
			else if (strcmp(rx_buffer, "enable-bcd") == 0) {

				UART_Write("WOW ENABLE\r\n");
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);	//BCD ENABLE
			}
			else if (strcmp(rx_buffer, "disable-bcd") == 0) {

				UART_Write("WOW DISABLE\r\n");
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);	//BCD DISABLE
			}
			else if (strcmp(rx_buffer, "square") == 0) {
				while(1)
				amazing_square();
			}

		}
		if(strcmp(rx_buffer, "\0") != 0)
		{
			UART_Write("\n");
		}
	}
}

void UART_Write(char *str)
{
	while ((HAL_UART_Transmit(&huart7, (uint8_t*) str, strlen(str), 0xFFFF)) != HAL_OK);
}


//Test that makes the head go back and forth, user can choose the length via step_target, no verification via encoder
void x_test(void) {
	int i = 0;			//Counts number of loop iterations
	while (1) {
		int step = 0;								//Counts iterations
		int step_target = 5000;	//Number of steps done by the program (9524 = 30cm)

		while (step <= (step_target * 2)) {
			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_0);	//Set PA0 to 1 or 0

			step++;

			delay_us(1000); //Choose the half-period in us of the signal. Lower = Faster motor

		}

		delay_us(1000000); //500ms delay

		i++;
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_0); //Change DIRECTION

		//encoder_pos_x = 0; //JUST FOR TESTING
	}
}

//Head goes to zero
void PnP_init(void) {
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET); //Set X direction to left
	//int pinstatec10 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_10);
	while (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_10)) //While limit switch open
	{
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_0);	//X CLK

		delay_us(100); //Choose the half-period in us of the signal. Lower = Faster motor
	}
	encoder_pos_x = 0;

	delay_us(100000);

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET); //Set Y direction to down
		while (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_11)) //While limit switch open
		{
			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1);	//Y CLK

			delay_us(100); //Choose the half-period in us of the signal. Lower = Faster motor
		}
		encoder_pos_y = 0;
}

//Go to X position pos_encoder_target based on encoder, verification via encoder
void x_goto_2(int pos_encoder_target) {
	int delta = pos_encoder_target - encoder_pos_x;

	if (delta > 0) {
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET); //Set X direction to right

		if ((delta + encoder_pos_x) > 31745) {
			delta = 0;
		}

		while (encoder_pos_x < pos_encoder_target) {
			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_0);	//X CLK

			delay_us(100); //Choose the half-period in us of the signal. Lower = Faster motor

		}
	} else {
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET); //Set X direction to left
		delta = -delta;

		if (delta > encoder_pos_x) {
			delta = 0;
		}

		while (encoder_pos_x > pos_encoder_target) {
			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_0);	//X CLK

			delay_us(100); //Choose the half-period in us of the signal. Lower = Faster motor

		}
	}
}

//Go to Y position pos_encoder_target based on encoder, verification via encoder
void y_goto_2(int pos_encoder_target) {
	int delta = pos_encoder_target - encoder_pos_y;

	if (delta > 0) {
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET); //Set Y direction to up

		if ((delta + encoder_pos_y) > 31745) {
			delta = 0;
		}

		while (encoder_pos_y < pos_encoder_target) {
			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1);	//Y CLK

			delay_us(100); //Choose the half-period in us of the signal. Lower = Faster motor

		}
	} else {
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET); //Set Y direction to down
		delta = -delta;

		if (delta > encoder_pos_y) {
			delta = 0;
		}

		while (encoder_pos_y > pos_encoder_target) {
			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1);	//Y CLK

			delay_us(100); //Choose the half-period in us of the signal. Lower = Faster motor

		}
	}
}

void nozzle_down_1(void)
{
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, GPIO_PIN_SET); //Left nozzle down
	for (int i = 0; i < 250; i++) {
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_6);	//NOZZLE CLK

		delay_us(1000); //Choose the half-period in us of the signal. Lower = Faster motor

	}
}

void nozzle_down_2(void)
{
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, GPIO_PIN_RESET); //Left nozzle down
	for (int i = 0; i < 250; i++) {
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_6);	//NOZZLE CLK

		delay_us(1000); //Choose the half-period in us of the signal. Lower = Faster motor

	}
}

void nozzle_up_1(void)
{
	nozzle_down_2();
}

void nozzle_up_2(void)
{
	nozzle_down_1();
}

void nozzle_reset(void)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET); //Disable BCD

	delay_us(100000);

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET); //Enable BCD
}

void amazing_square(void)
{
	x_goto_2(26000);
	delay_us(100000);
	y_goto_2(26000);
	delay_us(100000);
	x_goto_2(5000);
	delay_us(100000);
	y_goto_2(5000);
	delay_us(100000);
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
