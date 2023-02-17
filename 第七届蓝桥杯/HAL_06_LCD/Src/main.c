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
#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "stdio.h"
#include "string.h"
#include "i2c_hal.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
struct KEY{
	_Bool single_flag;
	_Bool key_sta;
	int key_judge;
	int key_number;
}key[4]={0,0,0,0};
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define filter_N 0.8 
#define K 30.35
double adc_vaule=0.0f;
double last_adc_vaule=0.0f;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
char lcd_text[20];
char tx_data[20];
char rx_data[20];
uint8_t PData=0;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
int Height=0;

int H_limit=70;
int M_limit=50;
int L_limit=30;

int H_limit_temp=0;
int M_limit_temp=0;
int L_limit_temp=0;

_Bool tranform_limit_flag=0;

int liquid_level=0;
int last_liquid_level=0;

int led0_time=0;
int led1_time=0;
int led2_time=0;

_Bool led1_flag=0;
_Bool led2_flag=0;

int modify_limit=0;

int delay_key_num=0;
int text_flag=0;
int switch_lcd_flag=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void adc_get(){
	uint32_t data=0;
	HAL_ADC_Start(&hadc2);
	HAL_ADC_PollForConversion(&hadc2,100);
	if(HAL_IS_BIT_SET(HAL_ADC_GetState(&hadc2),HAL_ADC_STATE_REG_EOC))
	{
	 data=HAL_ADC_GetValue(&hadc2);
	}
	adc_vaule=(float)(data*3.3/4096);
	adc_vaule=adc_vaule*filter_N+last_adc_vaule*(1-filter_N);
	last_adc_vaule=adc_vaule;
}
void led_off(){
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_11|GPIO_PIN_12, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
}
void led_display(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState){
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, PinState);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
}
void led_proc(){
	if(led0_time<=1000)
		led_display(GPIOC,GPIO_PIN_8,GPIO_PIN_RESET);
	else if(led0_time<=2000&&led0_time>1000)
		led_display(GPIOC,GPIO_PIN_8,GPIO_PIN_SET);
	else if(led0_time>2000)
		led0_time=0;
	
	//////led1
	if(led1_flag==1)
	{
		if(led1_time<=200)
			led_display(GPIOC,GPIO_PIN_9,GPIO_PIN_RESET);
		else if(led1_time<=400&&led1_time>200)
			led_display(GPIOC,GPIO_PIN_9,GPIO_PIN_SET);
		
		else if(led1_time<=600&&led1_time>400)
		led_display(GPIOC,GPIO_PIN_9,GPIO_PIN_RESET);
		else if(led1_time<=800&&led1_time>600)
			led_display(GPIOC,GPIO_PIN_9,GPIO_PIN_SET);
		
		else if(led1_time<=1000&&led1_time>800)
			led_display(GPIOC,GPIO_PIN_9,GPIO_PIN_RESET);
		else if(led1_time<=1200&&led1_time>1000)
			led_display(GPIOC,GPIO_PIN_9,GPIO_PIN_SET);
		
		else if(led1_time<=1400&&led1_time>1200)
			led_display(GPIOC,GPIO_PIN_9,GPIO_PIN_RESET);
		else if(led1_time<=1600&&led1_time>1400)
			led_display(GPIOC,GPIO_PIN_9,GPIO_PIN_SET);
		
		else if(led1_time<=1800&&led1_time>1600)
		led_display(GPIOC,GPIO_PIN_9,GPIO_PIN_RESET);
		else if(led1_time<=2000&&led1_time>1800)
			led_display(GPIOC,GPIO_PIN_9,GPIO_PIN_SET);
		else if(led1_time>2000)
		{
			led_display(GPIOC,GPIO_PIN_9,GPIO_PIN_SET);
			led1_flag=0;
		}
	}
	else
	{
		led_display(GPIOC,GPIO_PIN_9,GPIO_PIN_SET);
	}

	/////////led2
	if(led2_flag==1)
	{
		if(led2_time<=200)
			led_display(GPIOC,GPIO_PIN_10,GPIO_PIN_RESET);
		else if(led2_time<=400&&led2_time>200)
			led_display(GPIOC,GPIO_PIN_10,GPIO_PIN_SET);
		
		else if(led2_time<=600&&led2_time>400)
		led_display(GPIOC,GPIO_PIN_10,GPIO_PIN_RESET);
		else if(led2_time<=800&&led2_time>600)
			led_display(GPIOC,GPIO_PIN_10,GPIO_PIN_SET);
		
		else if(led2_time<=1000&&led2_time>800)
			led_display(GPIOC,GPIO_PIN_10,GPIO_PIN_RESET);
		else if(led2_time<=1200&&led2_time>1000)
			led_display(GPIOC,GPIO_PIN_10,GPIO_PIN_SET);
		
		else if(led2_time<=1400&&led2_time>1200)
			led_display(GPIOC,GPIO_PIN_10,GPIO_PIN_RESET);
		else if(led2_time<=1600&&led2_time>1400)
			led_display(GPIOC,GPIO_PIN_10,GPIO_PIN_SET);
		
		else if(led2_time<=1800&&led2_time>1600)
		led_display(GPIOC,GPIO_PIN_10,GPIO_PIN_RESET);
		else if(led2_time<=2000&&led2_time>1800)
			led_display(GPIOC,GPIO_PIN_10,GPIO_PIN_SET);
		else if(led2_time>2000)
		{
			led_display(GPIOC,GPIO_PIN_10,GPIO_PIN_SET);
			led2_flag=0;
		}
	}
	else
	{
		led_display(GPIOC,GPIO_PIN_10,GPIO_PIN_SET);
	}
}

void detact_liquid(){
	Height=(int)(adc_vaule*K);
	if(Height<=L_limit)
	{
		liquid_level=0;
	}
	else if(Height>L_limit&&Height<=M_limit)
	{
		liquid_level=1;
	}
	else if(Height>M_limit&&Height<=H_limit)
	{
		liquid_level=2;
	}
	else if(Height>H_limit)
	{
		liquid_level=3;
	}
	
	if(liquid_level-last_liquid_level>0)///up
	{
		delay_key_num++;
		sprintf(tx_data,"A:H%d+L%d+U\r\n",Height,liquid_level);
		HAL_UART_Transmit(&huart1, (uint8_t *)tx_data, strlen(tx_data),100);
		if(delay_key_num>1)
		led1_flag=1;
	}
	else if(liquid_level-last_liquid_level<0)//down
	{
		delay_key_num++;
		sprintf(tx_data,"A:H%d+L%d+D\r\n",Height,liquid_level);
		HAL_UART_Transmit(&huart1, (uint8_t *)tx_data, strlen(tx_data),100);
		if(delay_key_num>1)
			led1_flag=1;
	}
	last_liquid_level=liquid_level;

}


void key_proc(){
	if(key[0].single_flag==1)
	{
		key[0].key_number++;
		key[0].single_flag=0;
		
		if(key[0].key_number%2!=0)
		{
			 H_limit_temp=H_limit;
			 M_limit_temp=M_limit;
			 L_limit_temp=L_limit;
		}
		if((key[0].key_number%2==0)&&tranform_limit_flag==1)//////添加eeprom
		{
			H_limit=H_limit_temp;
			M_limit=M_limit_temp;
			L_limit=L_limit_temp;
			
			write_eeprom(0x01,H_limit);
			HAL_Delay(10);
			write_eeprom(0x02,M_limit);
			HAL_Delay(10);
			write_eeprom(0x03,L_limit);
			HAL_Delay(10);
			
			tranform_limit_flag=0;
		}
	}
	
	////B1
	else if(key[1].single_flag==1&&key[0].key_number%2!=0)
	{
		key[1].key_number++;
		switch(key[1].key_number%3)
		{
			case 0:
			{
				modify_limit=0;
			}break;
			case 1:
			{
				modify_limit=1;
			}break;
			case 2:
			{
				modify_limit=2;
			}break;
		}
		tranform_limit_flag=1;
		key[1].single_flag=0;
	}
	else if(key[1].single_flag==1)
	{
		key[1].single_flag=0;
	}
	
	///B2
	else if(key[2].single_flag==1&&key[0].key_number%2!=0)
	{
		switch(modify_limit)
		{
			case 0:
			{
				if(L_limit_temp<95)
					L_limit_temp+=5;
			}break;
			case 1:
			{
				if(M_limit_temp<95)
					M_limit_temp+=5;
			}break;
			case 2:
			{
				if(H_limit_temp<95)
					H_limit_temp+=5;
			}break;
		};
		tranform_limit_flag=1;
		key[2].single_flag=0;
	}
	else if(key[2].single_flag==1)
	{
		key[2].single_flag=0;
	}
	
	
	///B3
	else if(key[3].single_flag==1&&key[0].key_number%2!=0)
	{
		switch(modify_limit)
		{
			case 0:
			{
				if(L_limit_temp>5)
					L_limit_temp-=5;
			}break;
			case 1:
			{
				if(M_limit_temp>5)
					M_limit_temp-=5;
			}break;
			case 2:
			{
				if(H_limit_temp>5)
					H_limit_temp-=5;
			}break;
		};
		tranform_limit_flag=1;
		key[3].single_flag=0;
	}
	else if(key[3].single_flag==1)
	{
		key[3].single_flag=0;
	}
}


void lcd_dispaly(){
	if(key[0].key_number%2==0)
	{
		if(switch_lcd_flag==0)
		{
			LCD_Clear(White);
			switch_lcd_flag=1;
		}
		/////第2行
		LCD_SetTextColor(Black);
		sprintf(lcd_text,"   Liquid Level       ");
		LCD_DisplayStringLine(Line2 ,(unsigned char *)lcd_text);
		/////第3行
		LCD_SetTextColor(Black);
		sprintf(lcd_text,"  Height:%dcm       ",Height);
		LCD_DisplayStringLine(Line3 ,(unsigned char *)lcd_text);
		/////第4行
		LCD_SetTextColor(Black);
		sprintf(lcd_text,"  ADC:%.2fV       ",adc_vaule);
		LCD_DisplayStringLine(Line4 ,(unsigned char *)lcd_text);
		/////第5行
		LCD_SetTextColor(Black);
		sprintf(lcd_text,"  Level:%d       ",liquid_level);
		LCD_DisplayStringLine(Line5 ,(unsigned char *)lcd_text);
		
		//////////////////////测试
		/////第3行
		LCD_SetTextColor(Black);
		sprintf(lcd_text,"  Threshold1:%dcm       ",L_limit);
		LCD_DisplayStringLine(Line7 ,(unsigned char *)lcd_text);
		/////第4行
		LCD_SetTextColor(Black);
		sprintf(lcd_text,"  Threshold2:%dcm       ",M_limit);
		LCD_DisplayStringLine(Line8 ,(unsigned char *)lcd_text);
		/////第5行
		LCD_SetTextColor(Black);
		sprintf(lcd_text,"  Threshold3:%dcm       ",H_limit);
		LCD_DisplayStringLine(Line9 ,(unsigned char *)lcd_text);
	}
	else
	{
		if(switch_lcd_flag==1)
		{
			LCD_Clear(White);
			switch_lcd_flag=0;
		}
		LCD_SetTextColor(Black);
		sprintf(lcd_text,"   Parameter Setup       ");
		LCD_DisplayStringLine(Line2 ,(unsigned char *)lcd_text);
		
		switch(modify_limit)
		{
			case 0:
			{
				/////第3行
				LCD_SetTextColor(Blue);
				sprintf(lcd_text,"  Threshold1:%dcm       ",L_limit_temp);
				LCD_DisplayStringLine(Line3 ,(unsigned char *)lcd_text);
				/////第4行
				LCD_SetTextColor(Black);
				sprintf(lcd_text,"  Threshold2:%dcm       ",M_limit_temp);
				LCD_DisplayStringLine(Line4 ,(unsigned char *)lcd_text);
				/////第5行
				LCD_SetTextColor(Black);
				sprintf(lcd_text,"  Threshold3:%dcm       ",H_limit_temp);
				LCD_DisplayStringLine(Line5 ,(unsigned char *)lcd_text);
			}break;
			case 1:
			{
				/////第3行
				LCD_SetTextColor(Black);
				sprintf(lcd_text,"  Threshold1:%dcm       ",L_limit_temp);
				LCD_DisplayStringLine(Line3 ,(unsigned char *)lcd_text);
				/////第4行
				LCD_SetTextColor(Blue);
				sprintf(lcd_text,"  Threshold2:%dcm       ",M_limit_temp);
				LCD_DisplayStringLine(Line4 ,(unsigned char *)lcd_text);
				/////第5行
				LCD_SetTextColor(Black);
				sprintf(lcd_text,"  Threshold3:%dcm       ",H_limit_temp);
				LCD_DisplayStringLine(Line5 ,(unsigned char *)lcd_text);
			}break;
			case 2:
			{
				/////第3行
				LCD_SetTextColor(Black);
				sprintf(lcd_text,"  Threshold1:%dcm       ",L_limit_temp);
				LCD_DisplayStringLine(Line3 ,(unsigned char *)lcd_text);
				/////第4行
				LCD_SetTextColor(Black);
				sprintf(lcd_text,"  Threshold2:%dcm       ",M_limit_temp);
				LCD_DisplayStringLine(Line4 ,(unsigned char *)lcd_text);
				/////第5行
				LCD_SetTextColor(Blue);
				sprintf(lcd_text,"  Threshold3:%dcm       ",H_limit_temp);
				LCD_DisplayStringLine(Line5 ,(unsigned char *)lcd_text);
			}break;
		}
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
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_ADC2_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
	
	LCD_Init();
	HAL_TIM_Base_Start_IT(&htim3);
	HAL_TIM_Base_Start_IT(&htim15);
	
	HAL_UART_Receive_IT(&huart1, &PData, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	
	if(read_eeprom(0xff)!=10)
	{
		 H_limit=70;
		 M_limit=50;
		 L_limit=30;
		HAL_Delay(10);
		write_eeprom(0xff,10);
		HAL_Delay(10);
	}
	else
	{
		H_limit=read_eeprom(0x01);
		HAL_Delay(10);
		M_limit=read_eeprom(0x02);
		HAL_Delay(10);
		L_limit=read_eeprom(0x03);
		HAL_Delay(10);
		last_adc_vaule=read_eeprom(0x06);
		HAL_Delay(10);
	}
	LCD_Clear(White);
	LCD_SetBackColor(White);
	LCD_SetTextColor(Black);
	
  while (1)
  {

		led_off();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		key_proc();
		detact_liquid();
		lcd_dispaly();
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

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart->Instance==USART1)
	{
		if(PData=='C')
		{
			sprintf(tx_data,"C:H%d+L%d\r\n",Height,liquid_level);
			HAL_UART_Transmit(&huart1, (uint8_t *)tx_data, strlen(tx_data),100);
			led2_flag=1;
		}
		else if(PData=='S')
		{
			sprintf(tx_data,"S:TL%d+TM%d+TH%d\r\n",L_limit,M_limit,H_limit);
			HAL_UART_Transmit(&huart1, (uint8_t *)tx_data, strlen(tx_data),100);
			led2_flag=1;
		}
		HAL_UART_Receive_IT(&huart1, &PData, 1);
		
	}


}
int key_time=0;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	//按键扫描中断
	if(htim->Instance==TIM15)
	{
		led0_time++;
		
		if(led1_flag==1)
			led1_time++;
		else
			led1_time=0;
		
		if(led2_flag==1)
			led2_time++;
		else
			led2_time=0;

		if(key_time>=70)
		{
			key[0].key_sta=HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_0);
			key[1].key_sta=HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_1);
			key[2].key_sta=HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_2);
			key[3].key_sta=HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0);
			for(int i=0;i<4;i++)
			{
				switch(key[i].key_judge)
				{
					case 0:
					{
						if(key[i].key_sta==0)
							key[i].key_judge=1;
					}break;
					case 1:
					{
						if(key[i].key_sta==0)
						{
							key[i].single_flag=1;
						}
						else
							key[i].key_judge=0;
					}break;
				}
			}
			key_time=0;
		}
		else
			key_time++;
	}
	//adc定时中断
	if(htim->Instance==TIM3)
	{
		//text_flag++;
		adc_get();
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
