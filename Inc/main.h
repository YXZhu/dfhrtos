/**
  ******************************************************************************
  * File Name          : main.h
  * Description        : This file contains the common defines of the application
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */
#define SPEEDA 600
#define SPEEDB 600
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define Echo_1state_Pin GPIO_PIN_13
#define Echo_1state_GPIO_Port GPIOC
#define Echo_2state_Pin GPIO_PIN_14
#define Echo_2state_GPIO_Port GPIOC
#define Echo_3state_Pin GPIO_PIN_15
#define Echo_3state_GPIO_Port GPIOC
#define Trig_1_Pin GPIO_PIN_0
#define Trig_1_GPIO_Port GPIOA
#define Trig_2_Pin GPIO_PIN_1
#define Trig_2_GPIO_Port GPIOA
#define Trig_3_Pin GPIO_PIN_4
#define Trig_3_GPIO_Port GPIOA
#define Trig_4_Pin GPIO_PIN_5
#define Trig_4_GPIO_Port GPIOA
#define Trig_5_Pin GPIO_PIN_6
#define Trig_5_GPIO_Port GPIOA
#define Trig_6_Pin GPIO_PIN_7
#define Trig_6_GPIO_Port GPIOA
#define ENA_Pin GPIO_PIN_0
#define ENA_GPIO_Port GPIOB
#define ENB_Pin GPIO_PIN_1
#define ENB_GPIO_Port GPIOB
#define N1_Pin GPIO_PIN_12
#define N1_GPIO_Port GPIOB
#define N2_Pin GPIO_PIN_13
#define N2_GPIO_Port GPIOB
#define N3_Pin GPIO_PIN_14
#define N3_GPIO_Port GPIOB
#define N4_Pin GPIO_PIN_15
#define N4_GPIO_Port GPIOB
#define Echo_1_Pin GPIO_PIN_8
#define Echo_1_GPIO_Port GPIOA
#define Echo_2_Pin GPIO_PIN_9
#define Echo_2_GPIO_Port GPIOA
#define Echo_3_Pin GPIO_PIN_10
#define Echo_3_GPIO_Port GPIOA
#define Echo_4_Pin GPIO_PIN_11
#define Echo_4_GPIO_Port GPIOA
#define Echo_4state_Pin GPIO_PIN_15
#define Echo_4state_GPIO_Port GPIOA
#define hmc_init_Pin GPIO_PIN_3
#define hmc_init_GPIO_Port GPIOB
#define mgstart_Pin GPIO_PIN_4
#define mgstart_GPIO_Port GPIOB
#define I2CERR_Pin GPIO_PIN_5
#define I2CERR_GPIO_Port GPIOB
#define Echo_5_Pin GPIO_PIN_8
#define Echo_5_GPIO_Port GPIOB
#define Echo_6_Pin GPIO_PIN_9
#define Echo_6_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
