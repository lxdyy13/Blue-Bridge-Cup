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
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
struct KEY{
	_Bool key_sta;
	_Bool single_flag;
	
	int key_judge;
	int key_count;
}key[4]={0,0,0,0};

struct STATUS{
	double max_limit;
	double min_limit;
}volt={2.4,1.2};
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
int setting_tranform=0;

int high_led_number=1;
int low_led_number=2;
unsigned char  show_led_addr=0x00; 
int led_time=0;
_Bool time_flag=0;

_Bool swtich_led_flag=1;
_Bool swtich_lcd_flag=0;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
char lcd_text[30];
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
double adc_value=0.0f;
double last_adc_value=0.0f;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void adc_get(){
	unsigned int value=0;
	HAL_ADC_Start(&hadc2);
	HAL_ADC_PollForConversion(&hadc2,100);
	if(HAL_IS_BIT_SET(HAL_ADC_GetState(&hadc2),HAL_ADC_STATE_REG_EOC))
		value=HAL_ADC_GetValue(&hadc2);
	adc_value=value*3.3/4096;
	adc_value=0.7*adc_value + 0.3*last_adc_value;
	last_adc_value=adc_value;
}


void led_display(unsigned char pled,GPIO_PinState PinState){
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);
	
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_All, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, pled<<8, PinState);
	
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
}

void led_proc(){
		if(adc_value<volt.min_limit)
		{
			time_flag=1;
			if(swtich_led_flag==1)
			{
			show_led_addr=0x01<<(low_led_number-1);
				swtich_led_flag=0;
			}
			
			if(led_time<=200)
				led_display(show_led_addr,GPIO_PIN_RESET);
			else if(led_time<=400&&led_time>200)
				led_display(show_led_addr,GPIO_PIN_SET);
			else if(led_time>400)
				led_time=0;
			
		}
		else if(adc_value>volt.max_limit)
		{
			time_flag=1;
			if(swtich_led_flag==1)
			{
				show_led_addr=0x01<<(high_led_number-1);
				swtich_led_flag=0;
			}
			
			if(led_time<=200)
				led_display(show_led_addr,GPIO_PIN_RESET);
			else if(led_time<=400&&led_time>200)
				led_display(show_led_addr,GPIO_PIN_SET);
			else if(led_time>400)
				led_time=0;
		}
		else
		{
			time_flag=0;
			swtich_led_flag=1;
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_All, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
		}

}


void key_proc(){
		///////////B1
		if(key[0].single_flag==1)
		{
			key[0].key_count++;
			key[0].single_flag=0;
		}
		
		
		/////////B2
		else if(key[1].single_flag==1&&key[0].key_count%2!=0)
		{
			if(setting_tranform>=3)
				setting_tranform=0;
			else
				setting_tranform++;
			key[1].single_flag=0;
		}
		else if(key[1].single_flag==1)
		{
			key[1].single_flag=0;
		}
		
		///////B3 ¼Ó
		else if(key[2].single_flag==1&&key[0].key_count%2!=0)
		{
			switch(setting_tranform)
			{
				case 0:
				{
					volt.max_limit+=0.3;
					if(volt.max_limit>=3.3)
						volt.max_limit=3.3;
				}break;
				case 1:
				{
					volt.min_limit+=0.3;
					if(volt.min_limit>=volt.max_limit)		
						volt.min_limit=volt.max_limit-0.3;
				}break;
				
				case 2:
				{
					high_led_number++;
					if(high_led_number==low_led_number&&low_led_number!=8)
						high_led_number++;
					else if(high_led_number==low_led_number&&low_led_number==8)
						high_led_number=7;
					
					if(high_led_number>=8)
						high_led_number=8;
					
					swtich_led_flag=1;
				}break;
				case 3:
				{
					low_led_number++;
					if(high_led_number==low_led_number&&high_led_number!=8)
						low_led_number++;
					else if(high_led_number==low_led_number&&high_led_number==8)
						low_led_number=7;
					
				 if(low_led_number>=8)
						low_led_number=8;
				 
				 swtich_led_flag=1;
				}break;
			}
			key[2].single_flag=0;
		}
		else if(key[2].single_flag==1)
		{
			key[2].single_flag=0;
		}
		
		//////B4 ¼õ
		else if(key[3].single_flag==1&&key[0].key_count%2!=0)
		{
			switch(setting_tranform)
			{
				case 0:
				{
						volt.max_limit-=0.3;
						if(volt.max_limit<=volt.min_limit)
							volt.max_limit=volt.min_limit+0.3;
				}break;
				case 1:
				{
						volt.min_limit-=0.3;
						if(volt.min_limit<=0)
							volt.min_limit=0;
				}break;
				
				case 2:
				{
					high_led_number--;
					if(high_led_number==low_led_number&&low_led_number!=1)
						high_led_number--;
					else if(high_led_number==low_led_number&&low_led_number==1)
						high_led_number=2;
					
					if(high_led_number<=1)
						high_led_number=1;
					
					swtich_led_flag=1;
				}break;
				case 3:
				{
					low_led_number--;
					if(high_led_number==low_led_number&&high_led_number!=1)
						low_led_number--;
					else if(high_led_number==low_led_number&&high_led_number==1)
						low_led_number=2;
					
					if(low_led_number<=1)
							low_led_number=1;
					
					swtich_led_flag=1;   
				}break;
			}
			key[3].single_flag=0;
		}
		else if(key[3].single_flag==1)
		{
			key[3].single_flag=0;
		}

}
void lcd_dispaly(){
	if(key[0].key_count%2==0)
	{
		if(swtich_lcd_flag==0)
		{
			LCD_Clear(White);
			swtich_lcd_flag=1;
		}
		LCD_SetBackColor(White);
		sprintf(lcd_text,"       Main             ");
		LCD_DisplayStringLine(Line1,(unsigned char *)lcd_text);	
		
		LCD_SetBackColor(White);
		sprintf(lcd_text,"  Volt:%.2fV             ",adc_value);
		LCD_DisplayStringLine(Line3,(unsigned char *)lcd_text);	
		
		if(adc_value<volt.min_limit)
		{
			LCD_SetBackColor(White);
			sprintf(lcd_text,"  Status:Lower             ");
			LCD_DisplayStringLine(Line4,(unsigned char *)lcd_text);	
		}
		else if(adc_value>volt.max_limit)
		{
			LCD_SetBackColor(White);
			sprintf(lcd_text,"  Status:Upper             ");
			LCD_DisplayStringLine(Line4,(unsigned char *)lcd_text);	
		}
		else
		{
			LCD_SetBackColor(White);
			sprintf(lcd_text,"  Status:Normal             ");
			LCD_DisplayStringLine(Line4,(unsigned char *)lcd_text);	
		}
		
		LCD_SetBackColor(White);
		sprintf(lcd_text,"min:%.2f,max:%.2f         ",volt.min_limit,volt.max_limit);
		LCD_DisplayStringLine(Line6,(unsigned char *)lcd_text);	
		
		
	}
	else
	{	
		if(swtich_lcd_flag==1)
		{
			LCD_Clear(White);
			swtich_lcd_flag=0;
		}
		
		LCD_SetBackColor(White);
		sprintf(lcd_text,"       Setting             ");
		LCD_DisplayStringLine(Line1,(unsigned char *)lcd_text);	
		
		switch(setting_tranform)
		{
			case 0:
			{
				LCD_SetBackColor(Green);
				sprintf(lcd_text,"  Max Volt:%.2fV             ",volt.max_limit);
				LCD_DisplayStringLine(Line2,(unsigned char *)lcd_text);	
				LCD_SetBackColor(White);
				sprintf(lcd_text,"  Min Volt:%.2fV             ",volt.min_limit);
				LCD_DisplayStringLine(Line3,(unsigned char *)lcd_text);	
				LCD_SetBackColor(White);
				sprintf(lcd_text,"  Upper:LD%d             ",high_led_number);
				LCD_DisplayStringLine(Line4,(unsigned char *)lcd_text);	
				LCD_SetBackColor(White);
				sprintf(lcd_text,"  Lower:LD%d             ",low_led_number);
				LCD_DisplayStringLine(Line5,(unsigned char *)lcd_text);	
			}break;
			case 1:
			{
				LCD_SetBackColor(White);
				sprintf(lcd_text,"  Max Volt:%.2fV             ",volt.max_limit);
				LCD_DisplayStringLine(Line2,(unsigned char *)lcd_text);	
				LCD_SetBackColor(Green);
				sprintf(lcd_text,"  Min Volt:%.2fV             ",volt.min_limit);
				LCD_DisplayStringLine(Line3,(unsigned char *)lcd_text);	
				LCD_SetBackColor(White);
				sprintf(lcd_text,"  Upper:LD%d             ",high_led_number);
				LCD_DisplayStringLine(Line4,(unsigned char *)lcd_text);	
				LCD_SetBackColor(White);
				sprintf(lcd_text,"  Lower:LD%d             ",low_led_number);
				LCD_DisplayStringLine(Line5,(unsigned char *)lcd_text);	
			}break;
			case 2:
			{
				LCD_SetBackColor(White);
				sprintf(lcd_text,"  Max Volt:%.2fV             ",volt.max_limit);
				LCD_DisplayStringLine(Line2,(unsigned char *)lcd_text);	
				LCD_SetBackColor(White);
				sprintf(lcd_text,"  Min Volt:%.2fV             ",volt.min_limit);
				LCD_DisplayStringLine(Line3,(unsigned char *)lcd_text);	
				LCD_SetBackColor(Green);
				sprintf(lcd_text,"  Upper:LD%d             ",high_led_number);
				LCD_DisplayStringLine(Line4,(unsigned char *)lcd_text);	
				LCD_SetBackColor(White);
				sprintf(lcd_text,"  Lower:LD%d             ",low_led_number);
				LCD_DisplayStringLine(Line5,(unsigned char *)lcd_text);	
			}break;
			case 3:
			{
				LCD_SetBackColor(White);
				sprintf(lcd_text,"  Max Volt:%.2fV             ",volt.max_limit);
				LCD_DisplayStringLine(Line2,(unsigned char *)lcd_text);	
				LCD_SetBackColor(White);
				sprintf(lcd_text,"  Min Volt:%.2fV             ",volt.min_limit);
				LCD_DisplayStringLine(Line3,(unsigned char *)lcd_text);	
				LCD_SetBackColor(White);
				sprintf(lcd_text,"  Upper:LD%d             ",high_led_number);
				LCD_DisplayStringLine(Line4,(unsigned char *)lcd_text);	
				LCD_SetBackColor(Green);
				sprintf(lcd_text,"  Lower:LD%d             ",low_led_number);
				LCD_DisplayStringLine(Line5,(unsigned char *)lcd_text);	
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
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_TIM15_Init();
  /* USER CODE BEGIN 2 */
	
	HAL_TIM_Base_Start_IT(&htim15);
	LCD_Init();
	
	
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_All, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	
	LCD_Clear(White);
	LCD_SetBackColor(White);
	LCD_SetTextColor(Blue);
	
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		adc_get();
		key_proc();
		led_proc();
		lcd_dispaly();
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
int key_time=0;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance==TIM15)
	{
		if(time_flag==1)
			led_time++;
		else
			led_time=0;
		
		key[0].key_sta=HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_0);
		key[1].key_sta=HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_1);
		key[2].key_sta=HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_2);
		key[3].key_sta=HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0);
		
		if(key_time>=65)
		{
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
							key[i].single_flag=1;
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
