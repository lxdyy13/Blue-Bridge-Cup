#include "interrupt.h"
#include "usart.h"
extern uint8_t pdata;
extern char rx_data[];

int pointer=0;
struct KEY keys[4]={0,0,0};

int flag=0;

long int R39_value;
long int R40_value;
long int R40_frq;
long int R39_frq;


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart->Instance==USART1)
	{
		rx_data[pointer++]=pdata;
		HAL_UART_Receive_IT(&huart1, &pdata, 1);
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance==TIM15)
	{
			keys[0].key_sta=HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_0);
			keys[1].key_sta=HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_1);
			keys[2].key_sta=HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_2);
			keys[3].key_sta=HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0);
		if(flag>=5)
		{
			for(int i=0;i<4;i++)
			{
				switch (keys[i].key_judge)
				{
					case 0:
					{
						if(keys[i].key_sta==0)
						{
							keys[i].key_judge=1;
						}
					}break;
					case 1:
					{
						if(keys[i].key_sta==0)
						{
							keys[i].short_flag=1;
							keys[i].key_judge=0;
						}
						else
						{
							keys[i].key_judge=0;
						}
					}break;
				}
			}
		flag=0;
		}
	else flag++;
	}
	
}



void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance==TIM3)
	{
		R39_value=HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_1);
		__HAL_TIM_SET_COUNTER(htim,0);
		R39_frq=(80000000/80)/R39_value;
		
	}
	if(htim->Instance==TIM2)
	{
		R40_value=HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_1);
		__HAL_TIM_SET_COUNTER(htim,0);
		R40_frq=(80000000/80)/R40_value;
	}
}


