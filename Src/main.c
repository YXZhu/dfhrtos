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
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_rx;
DMA_HandleTypeDef hdma_i2c1_tx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

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
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
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
 unsigned char SW1,SW2,SW3,SW4;
 uint32_t ED1,ED2,ED3,ED4;
 uint32_t EDjl1 = 0,EDjl2 = 0,EDjl3 = 0,EDjl4 = 0;
 unsigned char temp1,temp2,temp3,temp4;	
 uint32_t echo1[2], echo2[2], echo3[2], echo4[2];
 double angle;
 uint16_t angle1;
 int16_t x,y,z,offsetX=0,offsetY=0,offsetZ=0;
 
 void startinittask(void const * argument);
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
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();

  /* USER CODE BEGIN 2 */

   HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
   TIM3->CCR1 = 600;
   TIM3->CCR2 = 600;
//	HAL_NVIC_SetPriority(EXTI9_5_IRQn, 4, 0);
//	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 4, 0);
//	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
//	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
	
//	HAL_TIM_IC_Start_DMA(&htim1,TIM_CHANNEL_1,echo1,1);
//	HAL_TIM_IC_Start_DMA(&htim1,TIM_CHANNEL_2,echo2,1);
//	HAL_TIM_IC_Start_DMA(&htim1,TIM_CHANNEL_3,echo3,1);
//	HAL_TIM_IC_Start_DMA(&htim1,TIM_CHANNEL_1,echo4,1);
    HAL_TIM_Base_Start_IT(&htim1);
	HAL_TIM_IC_Start_IT(&htim1,TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim1,TIM_CHANNEL_2);
	HAL_TIM_IC_Start_IT(&htim1,TIM_CHANNEL_3);
	HAL_TIM_IC_Start_IT(&htim1,TIM_CHANNEL_4);
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
  sConfigOC.Pulse = 800;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim3);

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

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Trig_1_Pin|Trig_2_Pin|Trig_3_Pin|Trig_4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, N1_Pin|N2_Pin|N3_Pin|N4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Trig_1_Pin Trig_2_Pin Trig_3_Pin Trig_4_Pin */
  GPIO_InitStruct.Pin = Trig_1_Pin|Trig_2_Pin|Trig_3_Pin|Trig_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : N1_Pin N2_Pin N3_Pin N4_Pin */
  GPIO_InitStruct.Pin = N1_Pin|N2_Pin|N3_Pin|N4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void startinittask(void const * argument)
{
	hmc5983jzinit();
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

  /* definition and creation of myTask05 */
  osThreadDef(myTask05, StartTask05, osPriorityNormal, 0, 256);
  myTask05Handle = osThreadCreate(osThread(myTask05), NULL);

  /* definition and creation of myTask06 */
  osThreadDef(myTask06, StartTask06, osPriorityHigh, 0, 64);
  myTask06Handle = osThreadCreate(osThread(myTask06), NULL);

  /* definition and creation of myTask07 */
  osThreadDef(myTask07, StartTask07,osPriorityRealtime , 0, 64);
  myTask07Handle = osThreadCreate(osThread(myTask07), NULL);

  /* definition and creation of hmc5983 */
  osThreadDef(hmc5983, hmc5983task, osPriorityRealtime, 0, 128);
  hmc5983Handle = osThreadCreate(osThread(hmc5983), NULL);
  
  vTaskDelete(startinitHandle);
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
	//uint8_t JL[] = {"前    cm左   cm右   cm后   cm"};
	uint16_t c;
uint8_t JL[] = {"前---.-CM 左---.-CM 右---.-CM 后---.-CM ---°  ----, ----, ----\n\r"};
//uint8_t JL[] = {"--------------------------\n\r"};
  for(;;)
  { 
	    angle1 = angle;    
	   //JL[1] = echo1[0]/10000+48;
		JL[2] = EDjl1%10000/1000+48;
		JL[3] = EDjl1%10000%1000/100+48;
		JL[4] = EDjl1%10000%1000%100/10+48;
	   JL[6] = EDjl1%10000%1000%100%10+48;
	  
	  //	JL[11] = echo1[1]/10000+48;
		JL[12] = EDjl2%10000/1000+48;
		JL[13] = EDjl2%10000%1000/100+48;
		JL[14] = EDjl2%10000%1000%100/10+48;
	   JL[16] = EDjl2%10000%1000%100%10+48;
	  
	  // JL[21] = EDjl1/10000+48;
		JL[22] = EDjl3%10000/1000+48;
		JL[23] = EDjl3%10000%1000/100+48;
		JL[24] = EDjl3%10000%1000%100/10+48;
	   JL[26] = EDjl3%10000%1000%100%10+48;
	  
   	//JL[14] = EDjl4/10000+48;
		JL[32] =EDjl4%10000/1000+48;
		JL[33] = EDjl4%10000%1000/100+48;
		JL[34] = EDjl4%10000%1000%100/10+48;
	   JL[36] = EDjl4%10000%1000%100%10+48;
		
		JL[40] = angle1/100+48;
		JL[41] = angle1%100/10+48;
	   JL[42] = angle1%100%10+48;
		
		if(x<0) JL[46] = '-',c = -x;
		else JL[46] = ' ',c = x;
		
		JL[47] = c/1000+48;
		JL[48] = c%1000/100+48;
	   JL[49] = c%1000%100/10+48;
		JL[50] = c%1000%100%10+48;
		
		if(y<0) JL[52] = '-',c = -y;
		else JL[52] = ' ',c = y;
		
		JL[53] = c/1000+48;
		JL[54] = c%1000/100+48;
	   JL[55] = c%1000%100/10+48;
		JL[56] = c%1000%100%10+48;
		
		if(z<0) JL[58] = '-',c = -z;
		else JL[58] = ' ',c = z;
		
		JL[59] = c/1000+48;
		JL[60] = c%1000/100+48;
	   JL[61] = c%1000%100/10+48;
		JL[62] = c%1000%100%10+48;
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
	 HAL_UART_Transmit(&huart2,JL,64,0xffff); 
	 osDelay(60); 
  }
  /* USER CODE END StartTask05 */
}

/* StartTask06 function */
void StartTask06(void const * argument)
{
  /* USER CODE BEGIN StartTask06 */
  /* Infinite loop */
		uint16_t led0pwmval=800;
	 uint8_t dir=1;
  for(;;)
  {
//      		osDelay(5);	 
//		if(dir)led0pwmval++;
//		else led0pwmval--;
//    
// 		if(led0pwmval>996)dir=0;
//		if(led0pwmval==800)dir=1;
//	  TIM3->CCR2 = led0pwmval;
	 if(EDjl1<200) HAL_GPIO_WritePin(GPIOC,GPIO_PIN_14,GPIO_PIN_RESET) ;
	  else HAL_GPIO_WritePin(GPIOC,GPIO_PIN_14,GPIO_PIN_SET);
	  osDelay(1);
  
  }
  /* USER CODE END StartTask06 */
}

/* StartTask07 function */
void StartTask07(void const * argument)
{
  /* USER CODE BEGIN StartTask07 */
  /* Infinite loop */
	uint16_t du,du1,du3,tem=1;
	
	HAL_GPIO_WritePin(N1_GPIO_Port,N1_Pin,GPIO_PIN_SET);
   HAL_GPIO_WritePin(N2_GPIO_Port,N2_Pin,GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(N3_GPIO_Port,N3_Pin,GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(N4_GPIO_Port,N4_Pin,GPIO_PIN_SET);
  for(;;)
  { 
	  du1 = angle;
	  if(tem ==1) 
	  {
		  du = angle;
	     tem = 0;
		  TIM3->CCR1 = 600;
	     TIM3->CCR2 = 600;
	  }
	  osDelay(1);
	  if(du>du1)
	  {
		  du3 = (360-du)+du1;
		  
	  }
	  else
	 {
		 du3 = du1-du;
	 }
	 
	  if(du3>50&du3<90&tem ==0)
	 {
		  TIM3->CCR1 = 400;
	     TIM3->CCR2 = 400;
		 tem = 2;
	 }
	 
	 if(du3>87&du3<91)
	 {
		  HAL_GPIO_WritePin(N1_GPIO_Port,N1_Pin,GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(N2_GPIO_Port,N2_Pin,GPIO_PIN_SET);
		  HAL_GPIO_WritePin(N3_GPIO_Port,N3_Pin,GPIO_PIN_SET);
		  HAL_GPIO_WritePin(N4_GPIO_Port,N4_Pin,GPIO_PIN_RESET);
		 osDelay(2);
		 HAL_GPIO_WritePin(N1_GPIO_Port,N1_Pin,GPIO_PIN_SET);
		     HAL_GPIO_WritePin(N2_GPIO_Port,N2_Pin,GPIO_PIN_SET);
		     HAL_GPIO_WritePin(N3_GPIO_Port,N3_Pin,GPIO_PIN_SET);
           HAL_GPIO_WritePin(N4_GPIO_Port,N4_Pin,GPIO_PIN_SET);
		 osDelay(1000);
		 HAL_GPIO_WritePin(N1_GPIO_Port,N1_Pin,GPIO_PIN_SET);
		 HAL_GPIO_WritePin(N2_GPIO_Port,N2_Pin,GPIO_PIN_RESET);
		 HAL_GPIO_WritePin(N3_GPIO_Port,N3_Pin,GPIO_PIN_RESET);
       HAL_GPIO_WritePin(N4_GPIO_Port,N4_Pin,GPIO_PIN_SET);
		 
			  tem = 1;
		}
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
