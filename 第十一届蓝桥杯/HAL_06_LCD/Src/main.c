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
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
struct KEYS{
	_Bool single_flag;
	_Bool sta;
	
	unsigned char judge_sta;
	unsigned char counter;

}key[4]={0,0,0,0};
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FIFTER_N 0.7

#define PARA 1
#define DATA 0

#define AUTO 1
#define MANU 0

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
double adc_value=0.0f;
double adc_value_last=0.0f;

int refer_volt=0;
int pa6_duty=10;
int pa7_duty=10;

_Bool lcd_mode=DATA;
_Bool use_mode=AUTO;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
char lcd_text[20];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void get_adc(){
	unsigned int data=0;
	HAL_ADC_Start(&hadc2);
	HAL_ADC_PollForConversion(&hadc2, 100);
	if(HAL_IS_BIT_SET(HAL_ADC_GetState(&hadc2),HAL_ADC_STATE_REG_EOC))
		data=HAL_ADC_GetValue(&hadc2);
	adc_value=data*3.3/4096;
	adc_value=FIFTER_N*adc_value+(1-FIFTER_N)*adc_value_last;
	adc_value_last=adc_value;
	
	refer_volt=(int)(adc_value_last*30.8);
}

void key_proc(){
	///B1
	if(key[0].single_flag==1)
	{
		if(lcd_mode==DATA)
			lcd_mode=PARA;
		else
			lcd_mode=DATA;
		
		key[0].single_flag=0;
	}
	
	/////B4
	if(key[3].single_flag==1)
	{
		if(use_mode==AUTO)
			use_mode=MANU;
		else
			use_mode=AUTO;
		
		key[3].single_flag=0;
	}
	
	////B2
	if(key[1].single_flag==1&&use_mode==MANU&&lcd_mode==PARA)
	{
		if(pa6_duty>=90)
			pa6_duty=10;
		else 
			pa6_duty+=10;
		key[1].single_flag=0;
	}
	
	if(key[1].single_flag==1)
	{
		key[1].single_flag=0;
	}
		////B3
	if(key[2].single_flag==1&&use_mode==MANU&&lcd_mode==PARA)
	{
		if(pa7_duty>=90)
			pa7_duty=10;
		else 
			pa7_duty+=10;
		key[2].single_flag=0;
	}
	
	if(key[2].single_flag==1)
	{
		key[2].single_flag=0;
	}
}
void output_proc(){
	
	if(use_mode==AUTO)
	{
		if(adc_value==3.25)
		{
			__HAL_TIM_SET_COMPARE(&htim16,TIM_CHANNEL_1,100);
			__HAL_TIM_SET_COMPARE(&htim17,TIM_CHANNEL_1,100);
		}
		else if(adc_value==0)
		{
			__HAL_TIM_SET_COMPARE(&htim16,TIM_CHANNEL_1,0);
			__HAL_TIM_SET_COMPARE(&htim17,TIM_CHANNEL_1,0);	
		}
		else
		{
			__HAL_TIM_SET_COMPARE(&htim16,TIM_CHANNEL_1,refer_volt);
			__HAL_TIM_SET_COMPARE(&htim17,TIM_CHANNEL_1,refer_volt);	
		}
	}
	else
	{
		__HAL_TIM_SET_COMPARE(&htim16,TIM_CHANNEL_1,pa6_duty);
		__HAL_TIM_SET_COMPARE(&htim17,TIM_CHANNEL_1,pa7_duty);
	}

}

void lcd_proc(){
	if(lcd_mode==DATA)
	{
		sprintf(lcd_text,"      Data               ");
		LCD_DisplayStringLine(Line0 ,(unsigned char *)lcd_text);
		
		sprintf(lcd_text,"    V:%.2f               ",adc_value);
		LCD_DisplayStringLine(Line2 ,(unsigned char *)lcd_text);
		
		if(use_mode==AUTO)
			sprintf(lcd_text,"    Mode:AUTO               ");
		else
			sprintf(lcd_text,"    Mode:MANU               ");
		LCD_DisplayStringLine(Line4 ,(unsigned char *)lcd_text);
	}
	else
	{
		sprintf(lcd_text,"      Para               ");
		LCD_DisplayStringLine(Line0 ,(unsigned char *)lcd_text);
		
		sprintf(lcd_text,"    PA6:%d%%               ",pa6_duty);
		LCD_DisplayStringLine(Line2 ,(unsigned char *)lcd_text);
		
		sprintf(lcd_text,"    PA7:%d%%               ",pa7_duty);
		LCD_DisplayStringLine(Line4 ,(unsigned char *)lcd_text);	
	}

	
	
	
	///////////test
//	sprintf(lcd_text,"    refer_volt:%d   ",refer_volt);
//	LCD_DisplayStringLine(Line9 ,(unsigned char *)lcd_text);

}

void led_proc(){
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|
                          GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
	
	if(use_mode==AUTO)
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
	
	if(lcd_mode==DATA)
	{
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
		
	}
	else
	{
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
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
  MX_TIM16_Init();
  MX_TIM17_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim15);
	
	HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim17, TIM_CHANNEL_1);
	
	__HAL_TIM_SET_COMPARE(&htim16,TIM_CHANNEL_1,10);
	__HAL_TIM_SET_COMPARE(&htim17,TIM_CHANNEL_1,10);
	
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
		get_adc();
		key_proc();
		output_proc();
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
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance==TIM15)
	{
		key[0].sta=HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_0);
		key[1].sta=HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_1);
		key[2].sta=HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_2);
		key[3].sta=HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0);
		
		if(key_time>=65)
		{
			for(int i=0;i<4;i++)
			{
				switch(key[i].judge_sta)
				{
					case 0:
					{
						if(key[i].sta==0)
							key[i].judge_sta=1;
					}break;
					case 1:
					{
						if(key[i].sta==0)
							key[i].single_flag=1;
						else
							key[i].judge_sta=0;
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
