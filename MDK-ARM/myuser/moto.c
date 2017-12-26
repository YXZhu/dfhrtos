#include "moto.h"
#include "main.h"
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"
#include "echo.h"

extern double angle;
extern uint32_t EDjl1,EDjl2,EDjl3,EDjl4,EDjl5,EDjl6;
uint16_t angle_temp1 = 0,angle_temp2 = 0,angle_temp3 = 0;
uint8_t angle_temp4 = 0;


void moto_90(unsigned char SF)
{

	  if(angle_temp4 == 0)
	  {
		  angle_temp4 = 1;
	     TIM3->CCR1 = 700;
	     TIM3->CCR2 = 700;
		 if(SF == 1) //·´
		 {			 
		  HAL_GPIO_WritePin(N1_GPIO_Port,N1_Pin,GPIO_PIN_SET);
		  HAL_GPIO_WritePin(N2_GPIO_Port,N2_Pin,GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(N3_GPIO_Port,N3_Pin,GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(N4_GPIO_Port,N4_Pin,GPIO_PIN_SET);
		 }
		 if(SF == 0) //Ë³
		 {
			  HAL_GPIO_WritePin(N1_GPIO_Port,N1_Pin,GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(N2_GPIO_Port,N2_Pin,GPIO_PIN_SET);
			  HAL_GPIO_WritePin(N3_GPIO_Port,N3_Pin,GPIO_PIN_SET);
			  HAL_GPIO_WritePin(N4_GPIO_Port,N4_Pin,GPIO_PIN_RESET);
		 }
        angle_temp1 = angle;
	  }
	  angle_temp2 = angle;	
	  
	  if(angle_temp2<angle_temp1) angle_temp3 = (360-angle_temp1)+angle_temp2;
	  else angle_temp3 = angle_temp2 - angle_temp1;
	  
	  if(angle_temp3>50&angle_temp3<91)
	 {
		  TIM3->CCR1 = 700-angle_temp3*4;
	     TIM3->CCR2 = 700-angle_temp3*4;
	 }
	 
	 if(angle_temp3>88&angle_temp3<91)
	 {
		 if(SF == 1)
		 {
		  HAL_GPIO_WritePin(N1_GPIO_Port,N1_Pin,GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(N2_GPIO_Port,N2_Pin,GPIO_PIN_SET);
		  HAL_GPIO_WritePin(N3_GPIO_Port,N3_Pin,GPIO_PIN_SET);
		  HAL_GPIO_WritePin(N4_GPIO_Port,N4_Pin,GPIO_PIN_RESET);
		 }
		 if(SF == 0)
		 {
		  HAL_GPIO_WritePin(N1_GPIO_Port,N1_Pin,GPIO_PIN_SET);
		  HAL_GPIO_WritePin(N2_GPIO_Port,N2_Pin,GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(N3_GPIO_Port,N3_Pin,GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(N4_GPIO_Port,N4_Pin,GPIO_PIN_SET);	
		 }			 
		 osDelay(2);
		 HAL_GPIO_WritePin(N1_GPIO_Port,N1_Pin,GPIO_PIN_SET);
	    HAL_GPIO_WritePin(N2_GPIO_Port,N2_Pin,GPIO_PIN_SET);
	    HAL_GPIO_WritePin(N3_GPIO_Port,N3_Pin,GPIO_PIN_SET);
	    HAL_GPIO_WritePin(N4_GPIO_Port,N4_Pin,GPIO_PIN_SET);
		 SF = 2;
		 angle_temp4 = 0;
		}
}

void moto_front(void)
{
	TIM3->CCR3 = SPEEDA;
   TIM3->CCR4 = SPEEDB;
	HAL_GPIO_WritePin(N1_GPIO_Port,N1_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(N2_GPIO_Port,N2_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(N3_GPIO_Port,N3_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(N4_GPIO_Port,N4_Pin,GPIO_PIN_RESET);
}
void moto_back(void)
{
	TIM3->CCR3 = 400;
   TIM3->CCR4 = 400;
	HAL_GPIO_WritePin(N1_GPIO_Port,N1_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(N2_GPIO_Port,N2_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(N3_GPIO_Port,N3_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(N4_GPIO_Port,N4_Pin,GPIO_PIN_SET);
}

void moto_right(unsigned char a)
{	
	if(a != 3)
	{
	  TIM3->CCR3 = 600;
     TIM3->CCR4 = 600;
	  HAL_GPIO_WritePin(N1_GPIO_Port,N1_Pin,GPIO_PIN_SET);
	  HAL_GPIO_WritePin(N4_GPIO_Port,N4_Pin,GPIO_PIN_SET);
	}
	if(a == 1)
	{
		//HAL_GPIO_WritePin(N1_GPIO_Port,N1_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(N2_GPIO_Port,N2_Pin,GPIO_PIN_RESET);		
		HAL_GPIO_WritePin(N3_GPIO_Port,N3_Pin,GPIO_PIN_RESET);
		//HAL_GPIO_WritePin(N4_GPIO_Port,N4_Pin,GPIO_PIN_SET);
	}
	if(a == 0)
	{
		//HAL_GPIO_WritePin(N1_GPIO_Port,N1_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(N2_GPIO_Port,N2_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(N3_GPIO_Port,N3_Pin,GPIO_PIN_SET);
		//HAL_GPIO_WritePin(N4_GPIO_Port,N4_Pin,GPIO_PIN_SET);
	}
	if(a == 2)
	{
		//HAL_GPIO_WritePin(N1_GPIO_Port,N1_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(N2_GPIO_Port,N2_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(N3_GPIO_Port,N3_Pin,GPIO_PIN_RESET);
		//HAL_GPIO_WritePin(N4_GPIO_Port,N4_Pin,GPIO_PIN_SET);
	}
	if(a == 3)
	{
		TIM3->CCR3 = SPEEDA+200;
      TIM3->CCR4 = 400;		
		HAL_GPIO_WritePin(N1_GPIO_Port,N1_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(N2_GPIO_Port,N2_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(N3_GPIO_Port,N3_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(N4_GPIO_Port,N4_Pin,GPIO_PIN_RESET);
	}
}

void moto_left(unsigned char a)
{	
	if(a != 3)
	{
		TIM3->CCR3 = 600;
      TIM3->CCR4 = 600;
	
	HAL_GPIO_WritePin(N2_GPIO_Port,N2_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(N3_GPIO_Port,N3_Pin,GPIO_PIN_SET);
	}
	if(a == 1)
	{	
		HAL_GPIO_WritePin(N1_GPIO_Port,N1_Pin,GPIO_PIN_RESET);
//		HAL_GPIO_WritePin(N2_GPIO_Port,N2_Pin,GPIO_PIN_SET);
//		HAL_GPIO_WritePin(N3_GPIO_Port,N3_Pin,GPIO_PIN_SET);
	   HAL_GPIO_WritePin(N4_GPIO_Port,N4_Pin,GPIO_PIN_RESET);
	}
	if(a == 0)
	{
		HAL_GPIO_WritePin(N1_GPIO_Port,N1_Pin,GPIO_PIN_SET);
//		HAL_GPIO_WritePin(N2_GPIO_Port,N2_Pin,GPIO_PIN_SET);
//		HAL_GPIO_WritePin(N3_GPIO_Port,N3_Pin,GPIO_PIN_SET);
	   HAL_GPIO_WritePin(N4_GPIO_Port,N4_Pin,GPIO_PIN_RESET);
	}
	if(a == 2)
	{
		HAL_GPIO_WritePin(N1_GPIO_Port,N1_Pin,GPIO_PIN_RESET);
	   HAL_GPIO_WritePin(N4_GPIO_Port,N4_Pin,GPIO_PIN_SET);
	}
	if(a == 3)
	{
		TIM3->CCR3 = 400;
      TIM3->CCR4 = SPEEDB+200;		
		HAL_GPIO_WritePin(N1_GPIO_Port,N1_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(N2_GPIO_Port,N2_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(N3_GPIO_Port,N3_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(N4_GPIO_Port,N4_Pin,GPIO_PIN_RESET);
	}
}

void moto_stop(void)
{
	
	HAL_GPIO_WritePin(N1_GPIO_Port,N1_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(N2_GPIO_Port,N2_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(N3_GPIO_Port,N3_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(N4_GPIO_Port,N4_Pin,GPIO_PIN_SET);
}

void moto_jztask(void const * argument)
{
	for(;;)
	{
		if(EDjl2<200)
	   {
			if(EDjl3<200)
			{
				moto_right(0);
			}
			else
			{
				moto_right(0);
			}
		}
		else
		{
			if(EDjl3<200)
			{
				moto_left(0);
			}
			else
			{
				if(EDjl2<250)
				{
					if(EDjl3<250)
					{
						moto_front();
					}
					else
					{
						moto_right(0);
					}
				}
				else
				{
					if(EDjl3<250)
					{
						moto_left(0);
					}
				}
			}
		}
		if(EDjl1<200)
		{
			moto_stop();
		}
		osDelay(1);
	}
		
}
