/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */
#include "echo.h"
#include "hmc5983.h"
#include "flash.h"
#include "moto.h"
#include "math.h"


/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

osThreadId Echo_1Handle;
osThreadId Echo_2Handle;
osThreadId Echo_3Handle;
osThreadId Echo_4Handle;
osThreadId myTask05Handle;
osThreadId myTask06Handle;
osThreadId myTask07Handle;
osThreadId hmc5983Handle;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
osThreadId startinitHandle;
osThreadId main_1Handle;
osThreadId Echo_5Handle;
osThreadId Echo_6Handle;
osThreadId moto_jzHandle;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM4_Init(void);
void Echo_1Task(void const * argument);
void Echo_2task(void const * argument);
void Echo_3task(void const * argument);
void Echo_4task(void const * argument);
void StartTask05(void const * argument);
void StartTask06(void const * argument);
void StartTask07(void const * argument);
extern void hmc5983task(void const * argument);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
 unsigned char SW1,SW2,SW3,SW4,SW5,SW6,SFmoto;
 uint32_t ED1,ED2,ED3,ED4,ED5,ED6;
 uint32_t EDjl1 = 0,EDjl2 = 0,EDjl3 = 0,EDjl4 = 0,EDjl5 = 0,EDjl6 = 0;
 unsigned char temp1,temp2,temp3,temp4,temp5,temp6;	
 uint32_t echo1[2], echo2[2], echo3[2], echo4[2],echo5[2],echo6[2];
 double angle;
 uint16_t angle1;
 int16_t x,y,z,offsetX=0,offsetY=0,offsetZ=0,hmcwritedata[4],hmcreaddata[4];
 
 void startinittask(void const * argument);
 void main_1task(void const * argument);
 void Echo_5task(void const * argument);
 void Echo_6task(void const * argument);
 extern void moto_jztask(void const * argument);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
  UDelayUS(200000);
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  Echo_Init();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_TIM4_Init();

  /* USER CODE BEGIN 2 */

   HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_4);
   TIM3->CCR3 = 600;
   TIM3->CCR4 = 600;
//	HAL_NVIC_SetPriority(EXTI9_5_IRQn, 4, 0);
//	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 4, 0);
//	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
//	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
	
//	HAL_TIM_IC_Start_DMA(&htim1,TIM_CHANNEL_1,echo1,1);
//	HAL_TIM_IC_Start_DMA(&htim1,TIM_CHANNEL_2,echo2,1);
//	HAL_TIM_IC_Start_DMA(&htim1,TIM_CHANNEL_3,echo3,1);
//	HAL_TIM_IC_Start_DMA(&htim1,TIM_CHANNEL_1,echo4,1);
    HAL_TIM_Base_Start_IT(&htim1);
	 HAL_TIM_Base_Start_IT(&htim4);
	HAL_TIM_IC_Start_IT(&htim1,TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim1,TIM_CHANNEL_2);
	HAL_TIM_IC_Start_IT(&htim1,TIM_CHANNEL_3);
	HAL_TIM_IC_Start_IT(&htim1,TIM_CHANNEL_4);
	HAL_TIM_IC_Start_IT(&htim4,TIM_CHANNEL_3);
	HAL_TIM_IC_Start_IT(&htim4,TIM_CHANNEL_4);
	//hmc5983_init();
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of Echo_1 */
//  osThreadDef(Echo_1, Echo_1Task, osPriorityRealtime, 0, 128);
//  Echo_1Handle = osThreadCreate(osThread(Echo_1), NULL);

//  /* definition and creation of Echo_2 */
//  osThreadDef(Echo_2, Echo_2task, osPriorityRealtime, 0, 128);
//  Echo_2Handle = osThreadCreate(osThread(Echo_2), NULL);

//  /* definition and creation of Echo_3 */
//  osThreadDef(Echo_3, Echo_3task, osPriorityRealtime, 0, 128);
//  Echo_3Handle = osThreadCreate(osThread(Echo_3), NULL);

//  /* definition and creation of Echo_4 */
//  osThreadDef(Echo_4, Echo_4task, osPriorityRealtime, 0, 128);
//  Echo_4Handle = osThreadCreate(osThread(Echo_4), NULL);

//  /* definition and creation of myTask05 */
//  osThreadDef(myTask05, StartTask05, osPriorityNormal, 0, 256);
//  myTask05Handle = osThreadCreate(osThread(myTask05), NULL);

//  /* definition and creation of myTask06 */
//  osThreadDef(myTask06, StartTask06, osPriorityHigh, 0, 64);
//  myTask06Handle = osThreadCreate(osThread(myTask06), NULL);

//  /* definition and creation of myTask07 */
//  osThreadDef(myTask07, StartTask07, osPriorityHigh, 0, 64);
//  myTask07Handle = osThreadCreate(osThread(myTask07), NULL);

//  /* definition and creation of hmc5983 */
//  osThreadDef(hmc5983, hmc5983task, osPriorityRealtime, 0, 128);
//  hmc5983Handle = osThreadCreate(osThread(hmc5983), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  osThreadDef(startinit, startinittask, osPriorityRealtime, 0, 256);
  startinitHandle = osThreadCreate(osThread(startinit), NULL);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
 

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_IC_InitTypeDef sConfigIC;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 72-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0xFFFF;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 72-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim3);

}

/* TIM4 init function */
static void MX_TIM4_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_IC_InitTypeDef sConfigIC;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 72-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 0xFFFF;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_IC_Init(&htim4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 460800;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, Echo_1state_Pin|Echo_2state_Pin|Echo_3state_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Trig_1_Pin|Trig_2_Pin|Trig_3_Pin|Trig_4_Pin 
                          |Trig_5_Pin|Trig_6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, N1_Pin|N2_Pin|N3_Pin|N4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Echo_4state_GPIO_Port, Echo_4state_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(I2CERR_GPIO_Port, I2CERR_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : Echo_1state_Pin Echo_2state_Pin Echo_3state_Pin */
  GPIO_InitStruct.Pin = Echo_1state_Pin|Echo_2state_Pin|Echo_3state_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : Trig_1_Pin Trig_2_Pin Trig_3_Pin Trig_4_Pin 
                           Trig_5_Pin Trig_6_Pin */
  GPIO_InitStruct.Pin = Trig_1_Pin|Trig_2_Pin|Trig_3_Pin|Trig_4_Pin 
                          |Trig_5_Pin|Trig_6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : N1_Pin N2_Pin N3_Pin N4_Pin */
  GPIO_InitStruct.Pin = N1_Pin|N2_Pin|N3_Pin|N4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : Echo_4state_Pin */
  GPIO_InitStruct.Pin = Echo_4state_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(Echo_4state_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : hmc_init_Pin mgstart_Pin */
  GPIO_InitStruct.Pin = hmc_init_Pin|mgstart_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : I2CERR_Pin */
  GPIO_InitStruct.Pin = I2CERR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(I2CERR_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void startinittask(void const * argument)
{  
	uint8_t aa = 1;
	for(;;)
	{
	
			if(HAL_GPIO_ReadPin(hmc_init_GPIO_Port,hmc_init_Pin) == 1)
	      {
				osDelay(500);
					if(HAL_GPIO_ReadPin(hmc_init_GPIO_Port,hmc_init_Pin) == 0)
					{
						//osDelay(500);				
						moto_stop();					
							  osThreadDef(Echo_1, Echo_1Task, osPriorityRealtime, 0, 128);
							  Echo_1Handle = osThreadCreate(osThread(Echo_1), NULL);

							  /* definition and creation of Echo_2 */
							  osThreadDef(Echo_2, Echo_2task, osPriorityRealtime, 0, 128);
							  Echo_2Handle = osThreadCreate(osThread(Echo_2), NULL);

							  /* definition and creation of Echo_3 */
							  osThreadDef(Echo_3, Echo_3task, osPriorityRealtime, 0, 128);
							  Echo_3Handle = osThreadCreate(osThread(Echo_3), NULL);

							  /* definition and creation of Echo_4 */
							  osThreadDef(Echo_4, Echo_4task, osPriorityRealtime, 0, 128);
							  Echo_4Handle = osThreadCreate(osThread(Echo_4), NULL);
							
							  /* definition and creation of Echo_5 */
							  osThreadDef(Echo_5,Echo_5task,osPriorityRealtime, 0, 128);
							  Echo_5Handle = osThreadCreate(osThread(Echo_5), NULL);

							  /* definition and creation of Echo_6 */
							  osThreadDef(Echo_6, Echo_6task, osPriorityRealtime, 0, 128);
							  Echo_6Handle = osThreadCreate(osThread(Echo_6), NULL);
										  
							  /* definition and creation of hmc5983 */
							  osThreadDef(hmc5983, hmc5983task, osPriorityRealtime, 0, 128);
							  hmc5983Handle = osThreadCreate(osThread(hmc5983), NULL);

                     osThreadDef(myTask05, StartTask05, osPriorityNormal, 0, 256);
						  myTask05Handle = osThreadCreate(osThread(myTask05), NULL);
						
					  aa = 1;
					}
					else 
					{ 
						while(HAL_GPIO_ReadPin(hmc_init_GPIO_Port,hmc_init_Pin) == 1){}
						if(aa ==1)
						{
						vTaskDelete(hmc5983Handle);
						vTaskDelete(Echo_1Handle);
						vTaskDelete(Echo_2Handle);
						vTaskDelete(Echo_3Handle);
						vTaskDelete(Echo_4Handle);
						vTaskDelete(Echo_5Handle);
						vTaskDelete(Echo_6Handle);
						vTaskDelete(myTask05Handle);
							aa = 0;
						}
						hmc5983jzinit();
					}
	       
		   }
		if(HAL_GPIO_ReadPin(mgstart_GPIO_Port,mgstart_Pin) == 1)
		{    osDelay(300);
			if(HAL_GPIO_ReadPin(mgstart_GPIO_Port,mgstart_Pin) == 1)
		   { 
				while(HAL_GPIO_ReadPin(mgstart_GPIO_Port,mgstart_Pin) == 1){}
			   flashread1(hmcreaddata,4);
			  offsetX = hmcreaddata[1];
			  offsetY = hmcreaddata[2];
			  offsetZ = hmcreaddata[3];
			  //osDelay(500);
					  osThreadDef(Echo_1, Echo_1Task, osPriorityRealtime, 0, 128);
					  Echo_1Handle = osThreadCreate(osThread(Echo_1), NULL);

					  /* definition and creation of Echo_2 */
					  osThreadDef(Echo_2, Echo_2task, osPriorityRealtime, 0, 128);
					  Echo_2Handle = osThreadCreate(osThread(Echo_2), NULL);

					  /* definition and creation of Echo_3 */
					  osThreadDef(Echo_3, Echo_3task, osPriorityRealtime, 0, 128);
					  Echo_3Handle = osThreadCreate(osThread(Echo_3), NULL);

					  /* definition and creation of Echo_4 */
					  osThreadDef(Echo_4, Echo_4task, osPriorityRealtime, 0, 128);
					  Echo_4Handle = osThreadCreate(osThread(Echo_4), NULL);
					
					  /* definition and creation of Echo_5 */
					  osThreadDef(Echo_5,Echo_5task,osPriorityRealtime, 0, 128);
					  Echo_5Handle = osThreadCreate(osThread(Echo_5), NULL);

					  /* definition and creation of Echo_6 */
					  osThreadDef(Echo_6, Echo_6task, osPriorityRealtime, 0, 128);
					  Echo_6Handle = osThreadCreate(osThread(Echo_6), NULL);
					  
		//			  /* definition and creation of myTask05 */
		//			  osThreadDef(myTask05, StartTask05, osPriorityNormal, 0, 256);
		//			  myTask05Handle = osThreadCreate(osThread(myTask05), NULL);

					  /* definition and creation of hmc5983 */
					  osThreadDef(hmc5983, hmc5983task, osPriorityRealtime, 0, 128);
					  hmc5983Handle = osThreadCreate(osThread(hmc5983), NULL);

					  /* definition and creation of myTask07 */
					  osThreadDef(myTask07, StartTask07,osPriorityRealtime , 0, 64);
					  myTask07Handle = osThreadCreate(osThread(myTask07), NULL);


					  
					  osThreadDef(main_1, main_1task, osPriorityHigh, 0, 256);
					  main_1Handle = osThreadCreate(osThread(main_1), NULL);
					  
					  osThreadDef(moto_jz, moto_jztask, osPriorityHigh, 0, 256);
					  moto_jzHandle = osThreadCreate(osThread(moto_jz), NULL);
						
						/* definition and creation of myTask06 */
					  osThreadDef(myTask06, StartTask06, osPriorityHigh, 0, 64);
					  myTask06Handle = osThreadCreate(osThread(myTask06), NULL);
					  
					  vTaskSuspend(myTask07Handle);
					  vTaskSuspend(moto_jzHandle);

			  vTaskDelete(startinitHandle);
			  if(aa == 1)
			  vTaskDelete(myTask05Handle);

			  aa = 0;
			}
		  }
		osDelay(1);
		
  }
}

void main_1task(void const * argument)
{
	//osDelay(500);
	uint8_t setmoto = 1;
	xQueueHandle MsgQueue;
	MsgQueue = xQueueCreate(10, sizeof(uint8_t));
	TIM3->CCR3 = SPEEDA;
   TIM3->CCR4 = SPEEDB;
	for(;;)
	{
		if(EDjl1>400)
		{
			if(EDjl2>300)
			{
				if(EDjl4>300)
				{
					if(EDjl3>300)
					{
						if(EDjl5>300) moto_front();
					}
				}
			}
			else
			{
				if(EDjl3<300)
				{
						vTaskResume(moto_jzHandle);
                  vTaskSuspend(main_1Handle);
				}
			}
		}
			
		
//		if(EDjl1>400)
//		{
//			if(setmoto == 1)
//			{				
//			   moto_front();
//				setmoto = 0;
//			}
//			if(EDjl3<300|EDjl5<300)
//		   {
//				if(EDjl2>200)
//				{
//				  if(EDjl2<300)
//				  {
//						if(EDjl3>200)
//						{
//							if((EDjl2-EDjl3)>=10)
//							{
//								TIM3->CCR3 = 500;
//								TIM3->CCR4 = 500;
//								moto_left();
//							}
//							else if((EDjl3-EDjl2)>=10)
//							{
//								TIM3->CCR3 = 500;
//								TIM3->CCR4 = 500;
//								moto_right();
//							}
//							else 
//							{
//								 setmoto = 1;
//								 TIM3->CCR3 = 600;
//								 TIM3->CCR4 = 600;
//							}
//						}
//						else
//						{
//							setmoto = 1;
//							TIM3->CCR3 = 600;
//							TIM3->CCR4 = 600;
//						}
//					}
//				  else
//				  {
//					  moto_left();
//				  }
//				}
//				else
//				{
//					setmoto = 1;
//					TIM3->CCR3 = 600+150;
//					TIM3->CCR4 = 600-100;
//				}
//			}
//		}
//      else
//		{
//			setmoto = 1;

//			if(EDjl1<=150)
//			{
//				if(setmoto == 1)
//				{
//					//moto_back();
//					//osDelay(5);
//					moto_stop();
//					setmoto = 0;
//				}if(EDjl4>400&EDjl3>400)
//					{
//						SFmoto = 1;
//						vTaskResume(myTask07Handle);
//                  vTaskSuspend(main_1Handle);
//					}
////				if((EDjl2-EDjl3)>=10)
////				{
////					TIM3->CCR3 = 500;
////					TIM3->CCR4 = 500;
////					moto_left();
////				}
////				else if((EDjl3-EDjl2)>=10)
////				{
////					TIM3->CCR3 = 500;
////					TIM3->CCR4 = 500;
////					moto_right();
////				}
//				else 
//				{
//					
//				}
//			 }
//			}
        
		osDelay (1);		
	}
}

/* Echo_5Task function */
void Echo_5task(void const * argument)
{

  /* USER CODE BEGIN 5 */
  /* Infinite loop */

  for(;;)
  {
	   echo_5();
		osDelay(1);
  }
  /* USER CODE END 5 */ 
}

/* Echo_6task function */
void Echo_6task(void const * argument)
{
  /* USER CODE BEGIN Echo_2task */
	
  /* Infinite loop */

  for(;;)
  {
       echo_6();
		osDelay(1);
  }
  /* USER CODE END Echo_2task */
}

/* USER CODE END 4 */

/* Echo_1Task function */
void Echo_1Task(void const * argument)
{

  /* USER CODE BEGIN 5 */
  /* Infinite loop */

  for(;;)
  {
	   echo_1();
		osDelay(1);
  }
  /* USER CODE END 5 */ 
}

/* Echo_2task function */
void Echo_2task(void const * argument)
{
  /* USER CODE BEGIN Echo_2task */
	
  /* Infinite loop */

  for(;;)
  {
       echo_2();
		osDelay(1);
  }
  /* USER CODE END Echo_2task */
}

/* Echo_3task function */
void Echo_3task(void const * argument)
{
  /* USER CODE BEGIN Echo_3task */
  /* Infinite loop */
	
  for(;;)
  {	   
      echo_3();
		osDelay(1);		  
  }
  /* USER CODE END Echo_3task */
}

/* Echo_4task function */
void Echo_4task(void const * argument)
{
  /* USER CODE BEGIN Echo_4task */
  /* Infinite loop */
  for(;;)
  {
	  echo_4();	
    osDelay(1);
   
  }
  /* USER CODE END Echo_4task */
}

/* StartTask05 function */
void StartTask05(void const * argument)
{
  /* USER CODE BEGIN StartTask05 */
  /* Infinite loop */
	//uint8_t JL[] = {"Ç°    cm×ó   cmÓÒ   cmºó   cm"};
	uint16_t c;
uint8_t JL[] = {"F---.- L---.- E---.- R---.- G---.- B---.- ---¡ã  ----, ----, ----\n\r"};
//uint8_t JL[] = {"--------------------------\n\r"};
  for(;;)
  { 
	    angle1 = angle;    
	   //JL[1] = echo1[0]/10000+48;
		JL[1] = EDjl1%10000/1000+48;
		JL[2] = EDjl1%10000%1000/100+48;
		JL[3] = EDjl1%10000%1000%100/10+48;
	   JL[5] = EDjl1%10000%1000%100%10+48;
	  
	  //	JL[11] = echo1[1]/10000+48;
		JL[8] = EDjl2%10000/1000+48;
		JL[9] = EDjl2%10000%1000/100+48;
		JL[10] = EDjl2%10000%1000%100/10+48;
	   JL[12] = EDjl2%10000%1000%100%10+48;
	  
	  // JL[21] = EDjl1/10000+48;
		JL[15] = EDjl3%10000/1000+48;
		JL[16] = EDjl3%10000%1000/100+48;
		JL[17] = EDjl3%10000%1000%100/10+48;
	   JL[19] = EDjl3%10000%1000%100%10+48;
	  
   	//JL[14] = EDjl4/10000+48;
		JL[22] = EDjl4%10000/1000+48;
		JL[23] = EDjl4%10000%1000/100+48;
		JL[24] = EDjl4%10000%1000%100/10+48;
	   JL[26] = EDjl4%10000%1000%100%10+48;
		
		JL[29] = EDjl5%10000/1000+48;
		JL[30] = EDjl5%10000%1000/100+48;
		JL[31] = EDjl5%10000%1000%100/10+48;
	   JL[33] = EDjl5%10000%1000%100%10+48;
	  
   	//JL[14] = EDjl4/10000+48;
		JL[36] = EDjl6%10000/1000+48;
		JL[37] = EDjl6%10000%1000/100+48;
		JL[38] = EDjl6%10000%1000%100/10+48;
	   JL[40] = EDjl6%10000%1000%100%10+48;
		
		JL[42] = angle1/100+48;
		JL[43] = angle1%100/10+48;
	   JL[44] = angle1%100%10+48;
		
		if(x<0) JL[48] = '-',c = -x;
		else JL[48] = ' ',c = x;
		
		JL[49] = c/1000+48;
		JL[50] = c%1000/100+48;
	   JL[51] = c%1000%100/10+48;
		JL[52] = c%1000%100%10+48;
		
		if(y<0) JL[54] = '-',c = -y;
		else JL[54] = ' ',c = y;
		
		JL[55] = c/1000+48;
		JL[56] = c%1000/100+48;
	   JL[57] = c%1000%100/10+48;
		JL[58] = c%1000%100%10+48;
		
		if(z<0) JL[60] = '-',c = -z;
		else JL[60] = ' ',c = z;
		
		JL[61] = c/1000+48;
		JL[62] = c%1000/100+48;
	   JL[63] = c%1000%100/10+48;
		JL[64] = c%1000%100%10+48;
//		JL[9] = EDjl2/100+48;
//		JL[10] = EDjl2%100/10+48;
//		JL[11] = EDjl2%100%10+48;
//		
//		JL[16] = EDjl3/100+48;
//		JL[17] = EDjl3%100/10+48;
//		JL[18] = EDjl3%100%10+48;
//		
//		JL[23] = EDjl4/100+48;
//		JL[24] = EDjl4%100/10+48;
//		JL[25] = EDjl4%100%10+48;
	 HAL_UART_Transmit(&huart2,JL,66,0xffff);
	 osDelay(120);
  }
  /* USER CODE END StartTask05 */
}

/* StartTask06 function */
void StartTask06(void const * argument)
{
  /* USER CODE BEGIN StartTask06 */
  /* Infinite loop */
//		uint16_t led0pwmval=800;
//	 uint8_t dir=1;
  for(;;)
  {
//      		osDelay(5);	 
//		if(dir)led0pwmval++;
//		else led0pwmval--;
//    
// 		if(led0pwmval>996)dir=0;
//		if(led0pwmval==800)dir=1;
//	  TIM3->CCR2 = led0pwmval;
	  osDelay(1);
  
  }
  /* USER CODE END StartTask06 */
}

/* StartTask07 function */
void StartTask07(void const * argument)
{
  /* USER CODE BEGIN StartTask07 */
  /* Infinite loop */
  for(;;)
  { 
	  moto_90(SFmoto);
	  if(SFmoto == 2)
	  {
		  vTaskResume(main_1Handle);
		  vTaskSuspend(myTask07Handle);
	  }
	 osDelay(1);
  }
  /* USER CODE END StartTask07 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
/* USER CODE BEGIN Callback 0 */

/* USER CODE END Callback 0 */
  if (htim->Instance == TIM2) {
    HAL_IncTick();
  }
/* USER CODE BEGIN Callback 1 */
//  if (htim->Instance == htim3.Instance)
//    {
//        /* Toggle LED */
//     echo_1();
//    }
/* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
