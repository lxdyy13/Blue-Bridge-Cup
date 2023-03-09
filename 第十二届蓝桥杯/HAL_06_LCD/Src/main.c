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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"

int fputc(int ch,FILE *f){
	HAL_UART_Transmit(&huart1,(uint8_t*)&ch,1,0xffff);
	return ch;
}
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct KEYs{
	_Bool singe_flag;
	_Bool sta;
	int judge;
	int counter;
}KEY;
KEY key[4]={0,0,0,0};

typedef struct DATA{
	char car_pate[4];
	char date[6];
	char time[6];
}DATA_car;
DATA_car car_message[8];
DATA_car car_temp;
char rx_data[30];
char tx_text[30];
int pointer=0;
uint8_t rx_pData;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DATA 1
#define PARA 0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
_Bool lcd_change=DATA;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
char lcd_text[20];

int remain_car=8;
int cnbr_car=0;
int vnbr_car=0;

double cnbr_car_price=3.5f;
double vnbr_car_price=2.0f;

unsigned int B4_number=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int double_flag=0;
int wirte_counter=1;
int last_counter=0;
int i=0;
void count_money(){
	int hour1,min1;
	float money;
	hour1=(rx_data[16]-car_message[i].time[0])*10+(rx_data[17]-car_message[i].time[1]);
	min1=(rx_data[18]-car_message[i].time[2])*10+(rx_data[19]-car_message[i].time[3]);
	if(min1>0)
	{
		hour1+=1;
		min1=0;
	}
	//发送数据给上位机
	if(rx_data[0]=='V')
	{
		money=vnbr_car_price*hour1;
		sprintf(tx_text,"VNBR:%s:%d:%.2f\r\n",car_message[i].car_pate,hour1,money);
		HAL_UART_Transmit(&huart1, (uint8_t *)tx_text, strlen(tx_text),100);	
		vnbr_car--;
	}
	else if(rx_data[0]=='C')
	{
		money=cnbr_car_price*hour1;
		sprintf(tx_text,"CNBR:%s:%d:%.2f\r\n",car_message[i].car_pate,hour1,money);
		HAL_UART_Transmit(&huart1, (uint8_t *)tx_text, strlen(tx_text),100);	
		cnbr_car--;
	}
}
void deal_data(){
	for(i=0;i<wirte_counter;i++)
	{
		int x=strcmp(car_message[i].car_pate,car_temp.car_pate);
		if(x==0)
		{
			count_money();//统计钱
			//memset(car_message[i].car_pate,0,4);
			car_message[i].car_pate[0]=0;
			car_message[i].car_pate[1]=0;
			car_message[i].car_pate[2]=0;
			car_message[i].car_pate[3]=0;
			double_flag++;
		}
	}
	if(double_flag!=0)//有这个车牌
	{
		double_flag=0;
	}
	else//没有这个车牌
	{
		if(remain_car!=0)//有车位
		{
			if(rx_data[0]=='V')
			{
				vnbr_car++;
			}
			else if(rx_data[0]=='C')
			{
				cnbr_car++;
			}
			
			car_message[wirte_counter].car_pate[0]=rx_data[5];
			car_message[wirte_counter].car_pate[1]=rx_data[6];
			car_message[wirte_counter].car_pate[2]=rx_data[7];
			car_message[wirte_counter].car_pate[3]=rx_data[8];
			printf("car_pate:%s\r\n",car_message[wirte_counter].car_pate);
			
			car_message[wirte_counter].time[0]=rx_data[16];
			car_message[wirte_counter].time[1]=rx_data[17];
			car_message[wirte_counter].time[2]=rx_data[18];
			car_message[wirte_counter].time[3]=rx_data[19];
			car_message[wirte_counter].time[4]=rx_data[20];
			car_message[wirte_counter].time[5]=rx_data[21];
			printf("car_time:%s\r\n",car_message[wirte_counter].time);	
			
			wirte_counter++;
		}
		else
			printf("no park\r\n");

	}

}

void usart_proc(){
	if(pointer!=0)
	{
		int temp=pointer;
		HAL_Delay(1);
		if(temp==pointer)//接收完成
		{
			if(pointer>0)
			{
				if(pointer==22)
				{
					car_temp.car_pate[0]=rx_data[5];
					car_temp.car_pate[1]=rx_data[6];
					car_temp.car_pate[2]=rx_data[7];
					car_temp.car_pate[3]=rx_data[8];
					//printf("car_temp:%s\r\n",car_temp.car_pate);
					
					deal_data();
				}
				else
				{
					sprintf(tx_text,"Error\r\n");
					HAL_UART_Transmit(&huart1, (uint8_t *)tx_text, strlen(tx_text),100);
				}
				pointer=0;
			}

		}
	}
}
void key_proc(){
	//B0
	if(key[0].singe_flag==1)
	{
		if(lcd_change==DATA)
			lcd_change=PARA;
		else
			lcd_change=DATA;
		key[0].singe_flag=0;
	}
	//B1
	if(key[1].singe_flag==1&&lcd_change==PARA)
	{
		cnbr_car_price+=0.5;
		vnbr_car_price+=0.5;
		key[1].singe_flag=0;
	}
	else if(key[1].singe_flag==1)
	{
		key[1].singe_flag=0;
	}
	//B2
	if(key[2].singe_flag==1&&lcd_change==PARA)
	{
		cnbr_car_price-=0.5;
		vnbr_car_price-=0.5;
		cnbr_car_price=cnbr_car_price>0?cnbr_car_price:0;
		vnbr_car_price=vnbr_car_price>0?vnbr_car_price:0;
//		if(cnbr_car_price<=0)
//			cnbr_car_price=0;
//		if(vnbr_car_price<=0)
//			vnbr_car_price=0;
		
		key[2].singe_flag=0;
	}
	else if(key[2].singe_flag==1)
	{
		key[2].singe_flag=0;
	}
	//B3
	if(key[3].singe_flag==1)
	{
		B4_number++;
		key[3].singe_flag=0;
	}
	if(B4_number%2==0)
		__HAL_TIM_SET_COMPARE(&htim17, TIM_CHANNEL_1,0);
	else
		__HAL_TIM_SET_COMPARE(&htim17, TIM_CHANNEL_1,20);
}
void lcd_proc(){
	if(lcd_change==DATA)
	{
	sprintf(lcd_text,"       Data        ");
	LCD_DisplayStringLine(Line1,(unsigned char *)lcd_text);	

	sprintf(lcd_text,"   CNBR:%d        ",cnbr_car);
	LCD_DisplayStringLine(Line3,(unsigned char *)lcd_text);	

	sprintf(lcd_text,"   VNBR:%d        ",vnbr_car);
	LCD_DisplayStringLine(Line5,(unsigned char *)lcd_text);		

	sprintf(lcd_text,"   IDLE:%d        ",remain_car-cnbr_car-vnbr_car);
	LCD_DisplayStringLine(Line7,(unsigned char *)lcd_text);			
	}
	else
	{
	sprintf(lcd_text,"       Para        ");
	LCD_DisplayStringLine(Line1,(unsigned char *)lcd_text);
		
	sprintf(lcd_text,"   CNBR:%.2f        ",cnbr_car_price);
	LCD_DisplayStringLine(Line3,(unsigned char *)lcd_text);	

	sprintf(lcd_text,"   VNBR:%.2f        ",vnbr_car_price);
	LCD_DisplayStringLine(Line5,(unsigned char *)lcd_text);		

	sprintf(lcd_text,"                    ");
	LCD_DisplayStringLine(Line7,(unsigned char *)lcd_text);		
	}
	

}
void led_proc(){
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
	
	if(remain_car>0)
	{
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
	}
	else
	{
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);	
	}
	
	if(B4_number%2==0)
	{
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
	}
	else
	{
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);	
	}
}
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
  MX_TIM15_Init();
  MX_TIM17_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_PWM_Start(&htim17, TIM_CHANNEL_1);
	__HAL_TIM_SET_COMPARE(&htim17, TIM_CHANNEL_1,0);
	
	HAL_TIM_Base_Start_IT(&htim15);
	
	HAL_UART_Receive_IT(&huart1,&rx_pData,1);
	LCD_Init();
	
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	
	LCD_Clear(Black);
	LCD_SetBackColor(Black);
	LCD_SetTextColor(White);
	
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		usart_proc();
		key_proc();
		lcd_proc();
		led_proc();
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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 20;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
int key_time=0;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance==TIM15)
	{
		key[0].sta=HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_0);
		key[1].sta=HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_1);
		key[2].sta=HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_2);
		key[3].sta=HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0);
		
		if(key_time>=70)
			
		{
			for(int i=0;i<4;i++)
			{
				switch(key[i].judge)
				{
					case 0:
					{
						if(key[i].sta==0)
							key[i].judge=1;
					}break;
					case 1:
					{
						if(key[i].sta==0)
							key[i].singe_flag=1;
						else		
							key[i].judge=0;							
					}break;
				}
			}
			key_time=0;
		}
		else
			key_time++;
	}

}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart->Instance==USART1)
	{
		rx_data[pointer++]=rx_pData;
		HAL_UART_Receive_IT(&huart1,&rx_pData,1);		
	}

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
