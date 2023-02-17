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
#include "interrupt.h"
#include "bsp.h"
#include "stdio.h"
#include "i2c_hal.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
extern struct KEY keys[];

extern long int R40_frq;
extern long int R39_frq;

extern int pointer;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
struct Channel_para channel_para[2]={0,0};
struct Usart_get_data usart_get_data[2]={0,0};
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
char text[30];
uint8_t pdata;
char rx_data[15];
char tx_data[15];

int key0_flag=0;
int key1_flag=0;

int show_channel_flag=0;

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
int pa6_valuedata=0;
int pa7_valuedata=0;

long int limit_pa6_data=0;
long int limit_pa7_data=0;

_Bool pa6_out_flag=0;
_Bool pa7_out_flag=0;

_Bool led_off=1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
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
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM15_Init();
  MX_TIM16_Init();
  MX_TIM17_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	LCD_Init();
	HAL_UART_Receive_IT(&huart1, &pdata, 1);
	HAL_TIM_Base_Start_IT(&htim15); //开启按键扫描定时器
	
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
	
	HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim17, TIM_CHANNEL_1);
	
	if(eeprom_read(0xff)!=20)
	{
		HAL_Delay(5);
		channel_para[0].N=2;
		channel_para[1].N=2;
		eeprom_wirte(0xff,20);
		HAL_Delay(5);
	}
	else
	{
		channel_para[0].N=eeprom_read(0);
		HAL_Delay(5);
		channel_para[1].N=eeprom_read(1);
		HAL_Delay(5);
	}
	frq_init();
		
	LCD_Clear(Black);
	LCD_SetBackColor(Black);
	LCD_SetTextColor(White);
	
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_8
                          |GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 **/

	  key_proc();
		if(key0_flag%2!=0)
		{
			receive_data();
		}
		
		Frq_setting();
		led_display();
		lcd_proc();
	//__HAL_TIM_SetCompare(&htim16,TIM_CHANNEL_1,1000);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV3;
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
void deal_rx_data(){
	if(pointer==8)
	{
		if(rx_data[4]=='1')//通道0
		{
			usart_get_data[0].double_num1=(rx_data[6]-48)*10+(rx_data[7]-48);
			if(usart_get_data[0].double_num1<=10&&usart_get_data[0].double_num1>=2)
			{
				channel_para[0].N=usart_get_data[0].double_num1;
				eeprom_wirte(0,channel_para[0].N);
				pa6_out_flag=0;
			}
			else
				pa6_out_flag=1;
		}
		else if(rx_data[4]=='2')//通道1
		{
			usart_get_data[1].double_num1=(rx_data[6]-48)*10+(rx_data[7]-48);		
			if(usart_get_data[1].double_num1<=10&&usart_get_data[1].double_num1>=2)
			{
				channel_para[1].N=usart_get_data[1].double_num1;
				eeprom_wirte(1,channel_para[1].N);
				pa7_out_flag=0;
			}
			else
				pa7_out_flag=1;
		}
		else
		{
			sprintf(tx_data,"send error2\r\n");
			HAL_UART_Transmit(&huart1, (uint8_t*)tx_data,strlen(tx_data),100);
		}
		pointer=0;
	}
	else if(pointer==7)
	{
		if(rx_data[4]=='1')//通道0
		{
			usart_get_data[0].double_num1=(rx_data[6]-48);
			if(usart_get_data[0].double_num1<=10&&usart_get_data[0].double_num1>=2)
			{
				channel_para[0].N=usart_get_data[0].double_num1;
				eeprom_wirte(0,channel_para[0].N);
				pa6_out_flag=0;
			}
			else
				pa6_out_flag=1;
		}
		else if(rx_data[4]=='2')//通道1
		{
			usart_get_data[1].double_num1=(rx_data[6]-48);
			if(usart_get_data[1].double_num1<=10&&usart_get_data[1].double_num1>=2)
			{
				channel_para[1].N=usart_get_data[1].double_num1;
				eeprom_wirte(1,channel_para[1].N);
				pa7_out_flag=0;
			}
			else
				pa7_out_flag=1;
		}
		else
		{
			sprintf(tx_data,"send error3\r\n");
			HAL_UART_Transmit(&huart1, (uint8_t*)tx_data,strlen(tx_data),100);
		}
		pointer=0;
	}
	else
	{
		sprintf(tx_data,"send error1\r\n");
		HAL_UART_Transmit(&huart1, (uint8_t*)tx_data,strlen(tx_data),100);
		pointer=0;
	}
}

void receive_data(){
	if(pointer>0)
	{
		int temp=pointer;
		HAL_Delay(1);
		if(temp==pointer)
			deal_rx_data();
	}
}
void key_proc(){
//////////////////////////////////////////////////////////////////key0
	if(keys[0].short_flag==1)
	{
		key0_flag+=1;
		if(key0_flag%2!=0)show_channel_flag=55;
		keys[0].short_flag=0;
	}
	
/////////////////////////////////////////////////////////////////key1
	if(keys[1].short_flag==1&&key0_flag%2==0)
	{
		key1_flag+=1;
		if(key1_flag%2==0)
			show_channel_flag=0;	
		else
			show_channel_flag=1;	
		keys[1].short_flag=0;
	}
	else if(keys[1].short_flag==1)
	{
		keys[1].short_flag=0;
	}
	
/////////////////////////////////////////////////////////////////key2
	if(keys[2].short_flag==1&&key0_flag%2==0)
	{
		if(key1_flag%2==0)
		{
			if(channel_para[0].N>0)
			{
				channel_para[0].N-=1;////////////加入eeprom
			}
			else
			{
				channel_para[0].N=channel_para[0].N;
			}
			eeprom_wirte(0,channel_para[0].N);
			HAL_Delay(5);
		}
		else
		{
			if(channel_para[1].N>0)
			{
				channel_para[1].N-=1;////////////加入eeprom
			}
			else
			{
				channel_para[1].N=channel_para[1].N;
			}
			eeprom_wirte(1,channel_para[1].N);
			HAL_Delay(5);
		}
		keys[2].short_flag=0;
	}
	else if(keys[2].short_flag==1)
	{
		keys[2].short_flag=0;
	}
	
/////////////////////////////////////////////////////////////////key3
	if(keys[3].short_flag==1&&key0_flag%2==0)
	{
		if(key1_flag%2==0)
		{
			if(channel_para[0].N<10)
			{
				channel_para[0].N+=1;////////////加入eeprom
			}
			else
			{
				channel_para[0].N=channel_para[0].N;
			}
			eeprom_wirte(0,channel_para[0].N);
			HAL_Delay(5);
		}
		else
		{
			if(channel_para[1].N<10)
			{
				channel_para[1].N+=1;////////////加入eeprom
			}
			else
			{
				channel_para[1].N=channel_para[1].N;
			}
			eeprom_wirte(1,channel_para[1].N);
			HAL_Delay(5);
		}
		keys[3].short_flag=0;
	}
	else if(keys[3].short_flag==1)
	{
		keys[3].short_flag=0;
	}
}


void Frq_setting(){
	if(key1_flag%2==0)//pa6
	{
		limit_pa6_data=R39_frq*channel_para[0].N;
	}
	else
	{
		limit_pa7_data=R40_frq*channel_para[1].N;
		
	}
	//////pa6
	if(limit_pa6_data>=50&&limit_pa6_data<=50000)
	{
		pa6_valuedata=(80000000/(160*R39_frq*channel_para[0].N));
	__HAL_TIM_SetAutoreload(&htim16,pa6_valuedata-1);
	__HAL_TIM_SetCompare(&htim16,TIM_CHANNEL_1,(pa6_valuedata-1)/2);
		pa6_out_flag=0;
	}
	else
	{
		__HAL_TIM_SetAutoreload(&htim16,10000-1);
		__HAL_TIM_SetCompare(&htim16,TIM_CHANNEL_1,5000);
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_RESET);
		pa6_out_flag=1;
	}
	//////pa7
	if(limit_pa7_data>=50&&limit_pa7_data<=50000)
	{
		pa7_valuedata=(80000000/(160*R40_frq*channel_para[1].N));
	__HAL_TIM_SetAutoreload(&htim17,pa7_valuedata-1);
	__HAL_TIM_SetCompare(&htim17,TIM_CHANNEL_1,(pa7_valuedata-1)/2);
		pa7_out_flag=0;
	}
	else
	{
		__HAL_TIM_SetAutoreload(&htim17,10000-1);
		__HAL_TIM_SetCompare(&htim17,TIM_CHANNEL_1,5000);
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_RESET);
		pa6_out_flag=1;
	}
}


void frq_init(){
	limit_pa6_data=R39_frq*channel_para[0].N;
	limit_pa7_data=R40_frq*channel_para[1].N;
	if(limit_pa6_data>=50&&limit_pa6_data<=50000)
	{
		pa6_valuedata=(80000000/(160*R39_frq*channel_para[0].N));
	__HAL_TIM_SetAutoreload(&htim16,pa6_valuedata-1);
	__HAL_TIM_SetCompare(&htim16,TIM_CHANNEL_1,(pa6_valuedata-1)/2);
		pa6_out_flag=0;
	}
	else
	{
		__HAL_TIM_SetAutoreload(&htim16,10000-1);
		__HAL_TIM_SetCompare(&htim16,TIM_CHANNEL_1,5000);
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_RESET);
		pa6_out_flag=1;
	}
	//////
	if(limit_pa7_data>=50&&limit_pa7_data<=50000)
	{
	pa7_valuedata=(80000000/(160*R40_frq*channel_para[1].N));
	__HAL_TIM_SetAutoreload(&htim17,pa7_valuedata-1);
	__HAL_TIM_SetCompare(&htim17,TIM_CHANNEL_1,(pa7_valuedata-1)/2);
		pa7_out_flag=0;
	}
	else
	{
		__HAL_TIM_SetAutoreload(&htim17,10000-1);
		__HAL_TIM_SetCompare(&htim17,TIM_CHANNEL_1,5000);
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_RESET);
		pa6_out_flag=1;
	}

}


void lcd_proc(){
		////////////////////////////////第1 2行
	LCD_SetTextColor(White);
	sprintf(text,"R39_frq:%ld       ",R39_frq);
	LCD_DisplayStringLine(Line0,(unsigned char *)text);	
	LCD_SetTextColor(White);
	sprintf(text,"R40_frq:%ld       ",R40_frq);
	LCD_DisplayStringLine(Line1,(unsigned char *)text);		
	
		////////////////////////////////第6 7 行
	LCD_SetTextColor(White);
	sprintf(text,"N(1):%d       ",channel_para[0].N);
	LCD_DisplayStringLine(Line6,(unsigned char *)text);	
	LCD_SetTextColor(White);
	sprintf(text,"N(2):%d       ",channel_para[1].N);
	LCD_DisplayStringLine(Line7,(unsigned char *)text);	
	
	////////////////////////////////第八行
	if(show_channel_flag==0)
	{
		LCD_SetTextColor(White);
		sprintf(text,"Channel:0      ");
		LCD_DisplayStringLine(Line8,(unsigned char *)text);	
	}
	else if(show_channel_flag==1)
	{
		LCD_SetTextColor(White);
		sprintf(text,"Channel:1      ");
		LCD_DisplayStringLine(Line8,(unsigned char *)text);	
	}
	else
	{
		LCD_SetTextColor(White);
		sprintf(text,"           ");               
		LCD_DisplayStringLine(Line8,(unsigned char *)text);	
	}
	////////////////////////////////第9行		
	LCD_SetTextColor(White);
	sprintf(text,"usart_double_num:%d",usart_get_data[1].double_num1);
	LCD_DisplayStringLine(Line9,(unsigned char *)text);	
}

void led_display(){
	
	if(led_off==1)
	{
		led_dispaly(GPIOC,GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_11|GPIO_PIN_12,GPIO_PIN_SET);
	}
	
	
	if(key0_flag%2!=0)
	{
		led_dispaly(GPIOC,GPIO_PIN_10,GPIO_PIN_RESET);
	}
	else
	{
		led_dispaly(GPIOC,GPIO_PIN_10,GPIO_PIN_SET);
	}
	
	if(key1_flag%2==0)
	{
		if(pa6_out_flag==0)
		{
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_RESET);
		}
		else
		{
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_SET);
		}
		led_dispaly(GPIOC,GPIO_PIN_9,GPIO_PIN_SET);
	}
	else
	{
		if(pa7_out_flag==0)
		{
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_RESET);
		}
		else
		{
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_SET);
		}
		led_dispaly(GPIOC,GPIO_PIN_8,GPIO_PIN_SET);
	}
	
	
//	if(key0_flag%2!=0&&key1_flag%2==0) //led3 led1亮
//	{
//		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);
//		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_All,GPIO_PIN_SET);
//		if(pa6_out_flag==0)
//		{
//			HAL_GPIO_WritePin(GPIOC,0x5<<8,GPIO_PIN_RESET);
//		}
//		else
//		{
//			HAL_GPIO_WritePin(GPIOC,0x4<<8,GPIO_PIN_RESET);
//		}
//		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
//	}
//	else if(key0_flag%2==0&&key1_flag%2==0)//led1亮
//	{
//		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);
//		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_All,GPIO_PIN_SET);
//		if(pa6_out_flag==0)
//		{
//			HAL_GPIO_WritePin(GPIOC,0x1<<8,GPIO_PIN_RESET);
//		}
//		else
//		{
//			HAL_GPIO_WritePin(GPIOC,0x0<<8,GPIO_PIN_RESET);
//		}
//		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
//	}
//	else if(key0_flag%2!=0&&key1_flag%2!=0)//led2 led3
//	{
//		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);
//		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_All,GPIO_PIN_SET);
//		if(pa7_out_flag==0)
//		{
//			HAL_GPIO_WritePin(GPIOC,0x6<<8,GPIO_PIN_RESET);
//		}
//		else
//		{
//			HAL_GPIO_WritePin(GPIOC,0x4<<8,GPIO_PIN_RESET);
//		}
//		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
//	}
//	else if(key0_flag%2==0&&key1_flag%2!=0)//led2亮
//	{
//		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);
//		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_All,GPIO_PIN_SET);
//		if(pa7_out_flag==0)
//		{
//			HAL_GPIO_WritePin(GPIOC,0x2<<8,GPIO_PIN_RESET);
//		}
//		else
//		{
//			HAL_GPIO_WritePin(GPIOC,0x0<<8,GPIO_PIN_RESET);
//		}
//		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
//	}
}


void led_dispaly(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState){
	
	HAL_GPIO_WritePin(GPIOx,GPIO_Pin,PinState);
	
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
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
