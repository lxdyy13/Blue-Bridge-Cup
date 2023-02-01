#include "interrput.h"
#include "usart.h"

struct KEYS keys[4]={0,0,0};
int time_flag=0;
int time1=0;
extern uint8_t P_uart_data;

char receive_data[30];
int data_pointer=0;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	
	if(huart->Instance==USART1)
	{
		receive_data[data_pointer++]=P_uart_data;
		HAL_UART_Receive_IT(&huart1, &P_uart_data,1);
	}
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance==TIM15)
	{
		time_flag++;
		keys[0].key_sta=HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_0);
		keys[1].key_sta=HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_1);
		keys[2].key_sta=HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_2);
		keys[3].key_sta=HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0);
		
		if(time1>=50)
		{
			for(int i=0;i<4;i++)
			{
				switch (keys[i].key_judge)
				{
					case 0:
					{
						if(keys[i].key_sta==0)
							keys[i].key_judge=1;
					}break;
					case 1:
					{
						if(keys[i].key_sta==0)
						{
							keys[i].key_short_flag=1;
							keys[i].key_judge=0;
						}
						else
							keys[i].key_judge=0;
					}break;
				}
			}
			time1=0;
		}
		else
		{
			time1++;
		}
		
	}

}

