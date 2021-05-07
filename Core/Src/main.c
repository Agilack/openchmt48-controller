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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef unsigned long u32;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */

static unsigned int pos_x;			//old X position without encoder
static unsigned int encoder_pos_x;	//X position with encoder
static unsigned int global_go=0;	//Used for test


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
/* USER CODE BEGIN PFP */

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
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  /* USER CODE BEGIN 2 */

  //HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_13);	//Set X direction to left

  startup_delay();	//6 seconds delay with flashing LED








  /* Activate SYSCFG */
    RCC->APB2ENR|= (1 << 14);

    Setup_user_button_led();
    Setup_A_interupt();
    Setup_B_interupt();


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */



    PnP_init();			//Go to zero
    delay_us(1000000); //1s delay
    x_goto_2(9000);
  while (1)
  {



	  /*
	  *(volatile unsigned long *)0x40021418 = (1<<12);	//set PF12=1
	  HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_12);
	  delay_us(1000);
	  *(volatile unsigned long *)0x40021418 = (1<<28);	//set PF12=0
	  HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_12);
	  */


	  //x_test();

	  //switch_test();

	  //x_test_pos();


	  if(global_go)			//Has User Button been pressed ?
	  {
		  x_goto_2(9000);
		  global_go=0;
	  }



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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_12|DIR_X_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CLK_X_GPIO_Port, CLK_X_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LIM_Y_Pin */
  GPIO_InitStruct.Pin = LIM_Y_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(LIM_Y_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_MDC_Pin RMII_RXD0_Pin RMII_RXD1_Pin */
  GPIO_InitStruct.Pin = RMII_MDC_Pin|RMII_RXD0_Pin|RMII_RXD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_REF_CLK_Pin RMII_MDIO_Pin RMII_CRS_DV_Pin */
  GPIO_InitStruct.Pin = RMII_REF_CLK_Pin|RMII_MDIO_Pin|RMII_CRS_DV_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LIM_X_Pin */
  GPIO_InitStruct.Pin = LIM_X_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(LIM_X_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PF12 DIR_X_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_12|DIR_X_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : CLK_X_Pin */
  GPIO_InitStruct.Pin = CLK_X_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CLK_X_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RMII_TXD1_Pin */
  GPIO_InitStruct.Pin = RMII_TXD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(RMII_TXD1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_TX_EN_Pin RMII_TXD0_Pin */
  GPIO_InitStruct.Pin = RMII_TX_EN_Pin|RMII_TXD0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

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

//Head goes to zero
void PnP_init(void)
{
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, GPIO_PIN_SET); //Set X direction to left
	while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3)) //While limit switch open
	{
		  HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_9);	//Set PE9 to 1 or 0

		  delay_us(1000); //Choose the half-period in us of the signal. Lower = Faster motor
	}
	pos_x = 0;
	encoder_pos_x=0;
}

//Go to position pos based on motor steps, no verification via encoder
void x_goto(int pos)
{
	int delta = pos - pos_x;



	if(delta>0)
	{
		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, GPIO_PIN_RESET); //Set X direction to right

		if((delta+pos_x)>12698)		//Position is too far to the right
		{
			delta=0;
		}
	}
	else
	{
		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, GPIO_PIN_SET); //Set X direction to left
		delta = -delta;

		if(delta>pos_x)				//Position is too far to the left
		{
			delta=0;
		}
	}

	int i=0;

	while(i<=(delta*2))				//Creates steps for the motor
		  {


		  HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_9);	//Set PE9 to 1 or 0

		  i++;

		  delay_us(500); //Choose the half-period in us of the signal. Lower = Faster motor

		  }
	if(delta!=0)
	{
	pos_x = pos;		//update x position
	}
}


//Go to position pos_encoder_target based on encoder, verification via encoder
void x_goto_2(int pos_encoder_target)
{
	int delta = pos_encoder_target - encoder_pos_x;



		if(delta>0)
		{
			HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, GPIO_PIN_RESET); //Set X direction to right

			if((delta+encoder_pos_x)>31745)
			{
				delta=0;
			}

			while(encoder_pos_x<pos_encoder_target)
			{
				HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_9);	//Set PE9 to 1 or 0

				delay_us(1000); //Choose the half-period in us of the signal. Lower = Faster motor

			}
		}
		else
		{
			HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, GPIO_PIN_SET); //Set X direction to left
			delta = -delta;

			if(delta>encoder_pos_x)
			{
				delta=0;
			}

			while(encoder_pos_x>pos_encoder_target)
			{
				HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_9);	//Set PE9 to 1 or 0

				delay_us(1000); //Choose the half-period in us of the signal. Lower = Faster motor

			}
		}
}


//Test that makes the head go back and forth, user can choose the length via step_target, no verification via encoder
void x_test(void)
{
	int i=0;			//Counts number of loop iterations
	while(1)
	{
	int step=0;								//Counts iterations
	int step_target=100;						//Number of steps done by the program (9524 = 30cm)

	while(step<=(step_target*2))
	{
	 HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_9);	//Set PE9 to 1 or 0

	 step++;

	 delay_us(500); //Choose the half-period in us of the signal. Lower = Faster motor

	 }

	 delay_us(500000); //500ms delay

	 i++;
	 HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_13); //Change DIRECTION

	 //encoder_pos_x = 0; //JUST FOR TESTING
	}
}


//Test to ensure the limit switch works, LED flashes faster when switch is closed
void switch_test(void)
{
	while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3)) //While limit switch open
		{
			  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7);	//BLUE LED ON/OFF

			  delay_us(1000000); //1s delay
		}

	while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3)==0) //While limit switch closed
			{
				  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7);	//BLUE LED ON/OFF

				  delay_us(500000); //500ms delay
			}
}


//Test to make the head go to various positions, no verification via encoder
void x_test_pos(void)
{
	delay_us(2000000); //2s
	x_goto(6000);	//go to position 6000
	delay_us(2000000); //2s
	x_goto(9000);
	delay_us(2000000); //2s
	x_goto(3000);
}



//Sets chosen bits of a 32bit register using value
void reg_set(unsigned long int reg, unsigned long int value)
{
        *(volatile unsigned long int *)reg = ( *(volatile unsigned long int *)reg | value );
}

//Overwrite a 32bit register with given value
void reg_wr(u32 reg, u32 value)
{
        *(volatile u32 *)reg = value;
}

//Returns a 32bit register from provided address
u32 reg_rd(u32 reg)
{
        return ( *(volatile u32 *)reg );
}

//Creates a 6 seconds delay and flashes the LED
void startup_delay(void)
{
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7);	//TURN OFF BLUE LED

	HAL_Delay(3000);							//3s delay
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7);	//TURN ON BLUE LED

	HAL_Delay(3000);
}

//User button interruption
void EXTI15_10_IRQHandler(void)
{
	reg_wr(EXTI_BASE + 0x14, (1 << 13));			//Clear pending register

	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7);	//TURN ON BLUE LED

	global_go=1;

}

//Interruptions from pins 9 to 5
void EXTI9_5_IRQHandler(void)
{
	u32 val;
	val = reg_rd(EXTI_BASE + 0x14);		//Checks which interruption is called

	reg_wr(EXTI_BASE + 0x14, val & 0x03E0); 	// CLear pending register from 9 to 5

	if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3)==0)	// X limit switch closed
	encoder_pos_x=0;

	if (val & (1 << 8))		//EXTI8
	{
		if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8))					//Rising edge on A
		{
			if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_9))				//B is HIGH
			{
				encoder_pos_x++;
			}
			else if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_9)==0)		//B is LOW

			{
				encoder_pos_x--;
			}
		}
		else if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8)==0)			//Falling edge on A
		{
			if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_9))				//B is HIGH
			{
				encoder_pos_x--;
			}
			else if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_9)==0)		//B is LOW

			{
				encoder_pos_x++;
			}
		}
	}

	if (val & (1 << 9))		//EXTI9
	{
		if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_9))					//Rising edge on B
		{
			if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8))				//A is HIGH
			{
				encoder_pos_x--;
			}
			else if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8)==0)		//A is LOW
				{
				encoder_pos_x++;
			}
		}
		else if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_9)==0)			//Falling edge on B
		{
			if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8))				//A is HIGH
			{
				encoder_pos_x++;
			}
			else if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8)==0)		//A is LOW
			{
				encoder_pos_x--;
			}
		}
	}

}

//Setup interruption for the user button
void Setup_user_button_led(void)
{
	unsigned long int val;
	*(volatile unsigned long int *)(0xE000E100 + 0x4) = (1 <<  8); //Activate EXTI15_10 into NVIC
	/* Configure External interrupt 13 to use PC13 */
	val = *(volatile unsigned long int *)(SYSCFG_BASE + 0x14);
	val &= 0xFF0F;    /* Clear EXTI13 */
	val |= (2 << 4); /* EXTI13 is now PC13 */
	*(volatile unsigned long int *)((unsigned long)SYSCFG_BASE + 0x14) = val;
	/* Trigger configuration */
	reg_set(EXTI_BASE + 0x08, (1 << 13)); /* Rising  trigger */

	reg_set(EXTI_BASE + 0x0C, (1 << 13)); /* Falling trigger */

	/* Activate EXTI13 (IMR) */
	reg_set(EXTI_BASE + 0x00, 1 << 13);
}

//Setup interruption from the A signal
void Setup_A_interupt(void)
{
	unsigned long int val;
	*(volatile unsigned long int *)(0xE000E100) = (1 <<  23); //Activate EXTI9_5 into NVIC
	/* Configure External interrupt 8 to use PC8 */
	val = *(volatile unsigned long int *)(SYSCFG_BASE + 0x10);
	val &= 0xFFF0;    /* Clear EXTI8 */
	val |= (2 << 0); /* EXTI13 is now PC8 */
	*(volatile unsigned long int *)((unsigned long)SYSCFG_BASE + 0x10) = val;
	/* Trigger configuration */
	reg_set(EXTI_BASE + 0x08, (1 << 8)); /* Rising  trigger */

	reg_set(EXTI_BASE + 0x0C, (1 << 8)); /* Falling trigger */
	/* Activate EXTI8 (IMR) */
	reg_set(EXTI_BASE + 0x00, 1 << 8);
}

//Setup interruption for the B signal
void Setup_B_interupt(void)
{
	unsigned long int val;
	*(volatile unsigned long int *)(0xE000E100) = (1 <<  23); //Activate EXTI9_5 into NVIC
	/* Configure External interrupt 9 to use PC9 */
	val = *(volatile unsigned long int *)(SYSCFG_BASE + 0x10);
	val &= 0xFF0F;    /* Clear EXTI9 */
	val |= (2 << 4); /* EXTI9 is now PC9 */
	*(volatile unsigned long int *)((unsigned long)SYSCFG_BASE + 0x10) = val;
	/* Trigger configuration */
	reg_set(EXTI_BASE + 0x08, (1 << 9)); /* Rising  trigger */

	reg_set(EXTI_BASE + 0x0C, (1 << 9)); /* Falling trigger */

	/* Activate EXTI9 (IMR) */
	reg_set(EXTI_BASE + 0x00, 1 << 9);
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
