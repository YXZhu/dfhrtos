#include "main.h"
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"
#include "echo.h"


extern unsigned char SW1,SW2,SW3,SW4,SW5,SW6;
extern uint32_t ED1,ED2,ED3,ED4,ED5,ED6;
extern uint32_t EDjl1,EDjl2,EDjl3,EDjl4,EDjl5,EDjl6;
extern unsigned char temp1,temp2,temp3,temp4,temp5,temp6;	
extern uint32_t echo1[2], echo2[2], echo3[2], echo4[2],echo5[2],echo6[2];

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim4;

void UDelayUS (unsigned int ulCount)
{
	unsigned int i;
	for ( i = 0; i < ulCount; i ++ )
	{
		uint8_t uc = 12;     //设置值为12，大约延1微秒  	      
		while ( uc -- );     //延1微秒	
	}	
}
void Echo_Init(void)
{
	//P0 = 0x00;
	//P2 = 0x00;
	//P1 = 0x00;
//	GPIO_InitTypeDef GPIO_InitStruct;
//	__HAL_RCC_GPIOC_CLK_ENABLE();
//	__HAL_RCC_GPIOB_CLK_ENABLE();
	SW1 = 1;
	SW2 = 1;
	SW3 = 1;
	SW4 = 1;
	SW5 = 1;
	SW6 = 1;
	ED1 = 0;
	ED2 = 0;
	ED3 = 0;
	ED4 = 0;
	ED5 = 0;
	ED6 = 0;
	temp1 = 1;
	temp2 = 1;
	temp3 = 1;
	temp4 = 1;
	temp5 = 1;
	temp6 = 1;
//	 GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
//    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
//    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
//	 HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_SET);
//	 GPIO_InitStruct.Pin = GPIO_PIN_0;
//    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
//    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
//	 HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_SET);
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	TIM_IC_InitTypeDef sConfigIC;
	if(__HAL_TIM_GET_FLAG(&htim1, TIM_FLAG_CC1) == 1)
	{
		
		if(temp1==1)
		{
			echo1[0] =  HAL_TIM_ReadCapturedValue(&htim1,TIM_CHANNEL_1);
			sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
		   sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
		   sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
		   sConfigIC.ICFilter = 0;
		   HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1);
			__HAL_TIM_CLEAR_IT(&htim1, TIM_IT_CC1);
		 // 启动输入捕获并开启中断
		  HAL_TIM_IC_Start_IT(&htim1,TIM_CHANNEL_1);
			temp1 = 0;
			HAL_GPIO_TogglePin(Echo_1state_GPIO_Port,Echo_1state_Pin);
		}
		else
		{
			echo1[1] =  HAL_TIM_ReadCapturedValue(&htim1,TIM_CHANNEL_1);
			sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
		   sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
		   sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
		   sConfigIC.ICFilter = 0;
		   HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1);
			__HAL_TIM_CLEAR_IT(&htim1, TIM_IT_CC1);
		   // 启动输入捕获并开启中断
		   HAL_TIM_IC_Start_IT(&htim1,TIM_CHANNEL_1);
			temp1 = 1;
			//HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
			//HAL_GPIO_TogglePin(Echo_1state_GPIO_Port,Echo_1state_Pin);
		}
	}
	if(__HAL_TIM_GET_FLAG(&htim1, TIM_FLAG_CC2) == 1)
	{
		if(temp2==1)
		{
			echo2[0] =  HAL_TIM_ReadCapturedValue(&htim1,TIM_CHANNEL_2);
			sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
		   sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
		   sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
		   sConfigIC.ICFilter = 0;
		   HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_2);
			__HAL_TIM_CLEAR_IT(&htim1, TIM_IT_CC2);
		 // 启动输入捕获并开启中断
		  HAL_TIM_IC_Start_IT(&htim1,TIM_CHANNEL_2);
			temp2 = 0;
		//HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_14);
			HAL_GPIO_TogglePin(Echo_2state_GPIO_Port,Echo_2state_Pin);
		}
		else
		{
			echo2[1] = HAL_TIM_ReadCapturedValue(&htim1,TIM_CHANNEL_2);
			sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
		   sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
		   sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
		   sConfigIC.ICFilter = 0;
		   HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_2);
			__HAL_TIM_CLEAR_IT(&htim1, TIM_IT_CC2);
		   // 启动输入捕获并开启中断
		   HAL_TIM_IC_Start_IT(&htim1,TIM_CHANNEL_2);
			temp2 = 1;
			//HAL_GPIO_TogglePin(Echo_2state_GPIO_Port,Echo_2state_Pin);
			
		}
	}
	if(__HAL_TIM_GET_FLAG(&htim1, TIM_FLAG_CC3) == 1)
	{
		if(temp3==1)
		{
			echo3[0] =  HAL_TIM_ReadCapturedValue(&htim1,TIM_CHANNEL_3);
			sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
		   sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
		   sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
		   sConfigIC.ICFilter = 0;
		   HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_3);
			__HAL_TIM_CLEAR_IT(&htim1, TIM_IT_CC3);
		   // 启动输入捕获并开启中断
		   HAL_TIM_IC_Start_IT(&htim1,TIM_CHANNEL_3);
			temp3 = 0;
			//HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_15);
			HAL_GPIO_TogglePin(Echo_3state_GPIO_Port,Echo_3state_Pin);
		}
		else
		{
			echo3[1] =  HAL_TIM_ReadCapturedValue(&htim1,TIM_CHANNEL_3);
			sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
		   sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
		   sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
		   sConfigIC.ICFilter = 0;
		   HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_3);
			__HAL_TIM_CLEAR_IT(&htim1, TIM_IT_CC3);
		   // 启动输入捕获并开启中断
		   HAL_TIM_IC_Start_IT(&htim1,TIM_CHANNEL_3);
			temp3 = 1;
			//HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_15);
			//HAL_GPIO_TogglePin(Echo_3state_GPIO_Port,Echo_3state_Pin);
		}
	}
	if(__HAL_TIM_GET_FLAG(&htim1, TIM_FLAG_CC4) == 1)
	{
		if(temp4==1)
		{
			echo4[0] =  HAL_TIM_ReadCapturedValue(&htim1,TIM_CHANNEL_4);
			sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
		   sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
		   sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
		   sConfigIC.ICFilter = 0;
		   HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_4);
			__HAL_TIM_CLEAR_IT(&htim1, TIM_IT_CC4);
		 // 启动输入捕获并开启中断
		  HAL_TIM_IC_Start_IT(&htim1,TIM_CHANNEL_4);
			temp4 = 0;
			//HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_0);
			HAL_GPIO_TogglePin(Echo_4state_GPIO_Port,Echo_4state_Pin);
		}
		else
		{
			echo4[1] =  HAL_TIM_ReadCapturedValue(&htim1,TIM_CHANNEL_4);
			sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
		   sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
		   sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
		   sConfigIC.ICFilter = 0;
		   HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_4);
			__HAL_TIM_CLEAR_IT(&htim1, TIM_IT_CC4);
		   // 启动输入捕获并开启中断
		   HAL_TIM_IC_Start_IT(&htim1,TIM_CHANNEL_4);
			temp4 = 1;
			//HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_0);
			//HAL_GPIO_TogglePin(Echo_4state_GPIO_Port,Echo_4state_Pin);
		}
	}
	if(__HAL_TIM_GET_FLAG(&htim4, TIM_FLAG_CC3) == 1)
	{
		if(temp5==1)
		{
			echo5[0] =  HAL_TIM_ReadCapturedValue(&htim4,TIM_CHANNEL_3);
			sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
		   sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
		   sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
		   sConfigIC.ICFilter = 0;
		   HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_3);
			__HAL_TIM_CLEAR_IT(&htim4, TIM_IT_CC3);
		   // 启动输入捕获并开启中断
		   HAL_TIM_IC_Start_IT(&htim4,TIM_CHANNEL_3);
			temp5 = 0;
			//HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_15);
			//HAL_GPIO_TogglePin(Echo_3state_GPIO_Port,Echo_3state_Pin);
		}
		else
		{
			echo5[1] =  HAL_TIM_ReadCapturedValue(&htim4,TIM_CHANNEL_3);
			sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
		   sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
		   sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
		   sConfigIC.ICFilter = 0;
		   HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_3);
			__HAL_TIM_CLEAR_IT(&htim4, TIM_IT_CC3);
		   // 启动输入捕获并开启中断
		   HAL_TIM_IC_Start_IT(&htim4,TIM_CHANNEL_3);
			temp5 = 1;
			//HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_15);
			//HAL_GPIO_TogglePin(Echo_3state_GPIO_Port,Echo_3state_Pin);
		}
	}
	if(__HAL_TIM_GET_FLAG(&htim4, TIM_FLAG_CC4) == 1)
	{
		if(temp6==1)
		{
			echo6[0] =  HAL_TIM_ReadCapturedValue(&htim4,TIM_CHANNEL_4);
			sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
		   sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
		   sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
		   sConfigIC.ICFilter = 0;
		   HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_4);
			__HAL_TIM_CLEAR_IT(&htim4, TIM_IT_CC4);
		 // 启动输入捕获并开启中断
		  HAL_TIM_IC_Start_IT(&htim4,TIM_CHANNEL_4);
			temp6 = 0;
			//HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_0);
			//HAL_GPIO_TogglePin(Echo_4state_GPIO_Port,Echo_4state_Pin);
		}
		else
		{
			echo6[1] =  HAL_TIM_ReadCapturedValue(&htim4,TIM_CHANNEL_4);
			sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
		   sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
		   sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
		   sConfigIC.ICFilter = 0;
		   HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_4);
			__HAL_TIM_CLEAR_IT(&htim4, TIM_IT_CC4);
		   // 启动输入捕获并开启中断
		   HAL_TIM_IC_Start_IT(&htim4,TIM_CHANNEL_4);
			temp6 = 1;
			//HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_0);
			//HAL_GPIO_TogglePin(Echo_4state_GPIO_Port,Echo_4state_Pin);
		}
	}
} 
void echo_1(void)
{
	if(SW1==1) 
		{
		  HAL_GPIO_WritePin(Trig_1_GPIO_Port,Trig_1_Pin,GPIO_PIN_SET);//Trig_1 = 1;			
			//UDelayUS(11);
			osDelay(1);
			HAL_GPIO_WritePin(Trig_1_GPIO_Port,Trig_1_Pin,GPIO_PIN_RESET);//Trig_1 = 0;
			SW1 = 0;
			//temp1 = 1;
//			echo1[0]= 0 ;//echo1[1]= 0 ;
//			echo1[1]= 0 ;
		}
		else
		{	
		     osDelay(59);
			  if(echo1[0]<echo1[1]) ED1 = echo1[1] - echo1[0];
			  else ED1 = (0xffff-echo1[0]) + echo1[1];			   
				ED1*=0.17;
			   if(ED1<6000)	EDjl1 = ED1;		  				
				SW1 = 1;		   		  	
		}
}
void echo_2(void)
{
	if(SW2==1) 
	{
	  HAL_GPIO_WritePin(Trig_2_GPIO_Port,Trig_2_Pin,GPIO_PIN_SET);//Trig_1 = 1;			
		//UDelayUS(11);
		osDelay(1);
		HAL_GPIO_WritePin(Trig_2_GPIO_Port,Trig_2_Pin,GPIO_PIN_RESET);//Trig_1 = 0;
		SW2 = 0;
		
	}
	else
	{	
			
        osDelay(59);
		  if(echo2[0]<echo2[1]) ED2 =  echo2[1] - echo2[0];
		  else ED2 = (0xffff-echo2[0]) + echo2[1];
		   ED2*=0.17;
		  if(ED2<6000) EDjl2 = ED2;
		  		  
		  SW2 = 1;

		  
	}
}
void echo_3(void)
{
	if(SW3==1) 
	{
	  HAL_GPIO_WritePin(Trig_3_GPIO_Port,Trig_3_Pin,GPIO_PIN_SET);//Trig_1 = 1;			
		//UDelayUS(11);
		osDelay(1);
		HAL_GPIO_WritePin(Trig_3_GPIO_Port,Trig_3_Pin,GPIO_PIN_RESET);//Trig_1 = 0;
		SW3 = 0;
		
		//echo3[0]= 0 ;echo3[1]= 0 ;
	}
	else
	{	
		  osDelay(59);				
		  if(echo3[0]<echo3[1]) ED3 = echo3[1] - echo3[0];
		  else ED3 = (0xffff-echo3[0]) + echo3[1];				 
		  ED3*=0.17;
		  if(ED3<6000) EDjl3 = ED3;
		  SW3 = 1; 
	}
}
void echo_4(void)
{
	if(SW4==1) 
	{
	   HAL_GPIO_WritePin(Trig_4_GPIO_Port,Trig_4_Pin,GPIO_PIN_SET);//Trig_1 = 1;			
		//UDelayUS(11);
		osDelay(1);
		HAL_GPIO_WritePin(Trig_4_GPIO_Port,Trig_4_Pin,GPIO_PIN_RESET);//Trig_1 = 0;
		SW4 = 0;
		
		//echo4[0]= 0 ;echo4[1]= 0 ;
	}
	else
	{	
			osDelay(59);			
			if(echo4[0]<echo4[1]) ED4 = echo4[1] - echo4[0];
			else ED4 = (0xffff-echo4[0]) + echo4[1];			
			ED4*=0.17;
		   if(ED4<6000) EDjl4 = ED4;
			SW4 = 1;
	}
}

void echo_5(void)
{
	if(SW5==1) 
	{
	  HAL_GPIO_WritePin(Trig_5_GPIO_Port,Trig_5_Pin,GPIO_PIN_SET);//Trig_1 = 1;			
		//UDelayUS(11);
		osDelay(1);
		HAL_GPIO_WritePin(Trig_5_GPIO_Port,Trig_5_Pin,GPIO_PIN_RESET);//Trig_1 = 0;
		SW5 = 0;
		
		//echo3[0]= 0 ;echo3[1]= 0 ;
	}
	else
	{	
		  osDelay(59);				
		  if(echo5[0]<echo5[1]) ED5 = echo5[1] - echo5[0];
		  else ED5 = (0xffff-echo5[0]) + echo5[1];				 
		  ED5*=0.17;
		  if(ED5<6000) EDjl5 = ED5;
		  SW5 = 1; 
	}
}
void echo_6(void)
{
	if(SW6==1) 
	{
	   HAL_GPIO_WritePin(Trig_6_GPIO_Port,Trig_6_Pin,GPIO_PIN_SET);//Trig_1 = 1;			
		//UDelayUS(11);
		osDelay(1);
		HAL_GPIO_WritePin(Trig_6_GPIO_Port,Trig_6_Pin,GPIO_PIN_RESET);//Trig_1 = 0;
		SW6 = 0;
		
		//echo4[0]= 0 ;echo4[1]= 0 ;
	}
	else
	{	
			osDelay(59);			
			if(echo6[0]<echo6[1]) ED6 = echo6[1] - echo6[0];
			else ED6 = (0xffff-echo6[0]) + echo6[1];			
			ED6*=0.17;
		   if(ED6<6000) EDjl6 = ED6;
			SW6 = 1;
	}
}
