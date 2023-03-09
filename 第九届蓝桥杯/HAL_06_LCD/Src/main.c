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
#include "rtc.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
#include "i2c_hal.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
struct KEYS{
	_Bool single_flag;
	_Bool long_flag;
	_Bool key_sta;
	
	int key_judge;
	int long_time;
	int key_number;
}key[4]={0,0,0,0,0,0};

struct TIME{
	int hour;
	int min;
	int sec;
	int flag;
	int number;
	
	int change_highlight;
	int status;
	int time_flag;
}counter[5]={0,0,0,0,0,0,0,0},counter_temp[5]={0,0,0,0,0,0,0,0},counter_temp2[5]={0,0,0,0,0,0,0,0};

/* USER CODE END PTD */
int all_text=0;
/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
char text[20];
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
_Bool led_flag=0;
int led_count=0;

_Bool pwm_flag=0; 
int lcd_cut=0;

_Bool lcd_setting=0;

_Bool key1_to_key2_flag=1;
_Bool short_to_long_key2_flag=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void led_disp(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState){
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOx,GPIO_Pin,PinState);
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_RESET);
}
void led_off(){
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_RESET);
}

void led_proc(){
	if(led_flag)
	{
		if(led_count<=500)
		led_disp(GPIOC,GPIO_PIN_8,GPIO_PIN_SET);
		else if(led_count>500&&led_count<=1000)
		led_disp(GPIOC,GPIO_PIN_8,GPIO_PIN_RESET);
		else
			led_count=0;
	}
	else
	led_disp(GPIOC,GPIO_PIN_8,GPIO_PIN_SET);
}

void pwm_proc(){
	if(pwm_flag)
		__HAL_TIM_SET_COMPARE(&htim16,TIM_CHANNEL_1,80);
	else
		__HAL_TIM_SET_COMPARE(&htim16,TIM_CHANNEL_1,0);

}
void key_proc(){
	if(key[0].single_flag==1&&lcd_setting==0)
	{
		lcd_cut=(lcd_cut+1)%5;
		counter[lcd_cut].change_highlight=0;
		key1_to_key2_flag=1;
		key[0].single_flag=0;
	}
	else if(key[0].single_flag==1)
	{
		key[0].single_flag=0;
	}
	//////////////////////////////B2    seeting Êý×Ö
	if(key[1].single_flag==1)
	{
		if(key1_to_key2_flag==1)
		{
			counter_temp[lcd_cut].hour=counter[lcd_cut].hour;
			counter_temp[lcd_cut].min=counter[lcd_cut].min;
			counter_temp[lcd_cut].sec=counter[lcd_cut].sec;
			key1_to_key2_flag=0;
			short_to_long_key2_flag=1;
		}
		lcd_setting=1;
		counter[lcd_cut].flag=1;
		counter[lcd_cut].change_highlight++;
		if(counter[lcd_cut].change_highlight>3)
			counter[lcd_cut].change_highlight=1;
		key[1].single_flag=0;
	}
	if(key[1].long_flag==1)
	{
			counter[lcd_cut].hour=counter_temp[lcd_cut].hour;
			counter[lcd_cut].min=counter_temp[lcd_cut].min;
			counter[lcd_cut].sec=counter_temp[lcd_cut].sec;
		if(short_to_long_key2_flag)
		{
		//Ìí¼Óeeprom
			switch(lcd_cut)
			{
				case 0:
				{
					write_eeprom(0x01,counter_temp[lcd_cut].hour);
					HAL_Delay(5);
					write_eeprom(0x02,counter_temp[lcd_cut].min);
					HAL_Delay(5);
					write_eeprom(0x03,counter_temp[lcd_cut].sec);
					HAL_Delay(5);
				}break;
				case 1:
				{
					write_eeprom(0x04,counter_temp[lcd_cut].hour);
					HAL_Delay(5);
					write_eeprom(0x05,counter_temp[lcd_cut].min);
					HAL_Delay(5);
					write_eeprom(0x06,counter_temp[lcd_cut].sec);
					HAL_Delay(5);
				}break;
				case 2:
				{
					write_eeprom(0x07,counter_temp[lcd_cut].hour);
					HAL_Delay(5);
					write_eeprom(0x08,counter_temp[lcd_cut].min);
					HAL_Delay(5);
					write_eeprom(0x09,counter_temp[lcd_cut].sec);
					HAL_Delay(5);
				}break;
				case 3:
				{
					write_eeprom(0x11,counter_temp[lcd_cut].hour);
					HAL_Delay(5);
					write_eeprom(0x12,counter_temp[lcd_cut].min);
					HAL_Delay(5);
					write_eeprom(0x13,counter_temp[lcd_cut].sec);
					HAL_Delay(5);
				}break;									
				case 4:
				{
					write_eeprom(0x14,counter_temp[lcd_cut].hour);
					HAL_Delay(5);
					write_eeprom(0x15,counter_temp[lcd_cut].min);
					HAL_Delay(5);
					write_eeprom(0x16,counter_temp[lcd_cut].sec);
					HAL_Delay(5);
				}break;	
			}
			write_eeprom(lcd_cut*5,counter_temp[lcd_cut].hour);
			HAL_Delay(5);
			write_eeprom(lcd_cut*5<<1,counter_temp[lcd_cut].min);
			HAL_Delay(5);
			write_eeprom(lcd_cut*5<<2,counter_temp[lcd_cut].sec);
			HAL_Delay(5);
			short_to_long_key2_flag=0;
		}

		lcd_setting=0;
		counter[lcd_cut].flag=0;
		key[1].long_flag=0;
	}
	
	//////////////////////////////B3 
	if(key[2].single_flag==1&&lcd_setting==1)
	{
		all_text++;
		switch(counter[lcd_cut].change_highlight)
		{
			case 1:
			{
				counter_temp[lcd_cut].sec++;
				if(counter_temp[lcd_cut].sec>60)
					counter_temp[lcd_cut].sec=0;
			}break;
			case 2:
			{
				counter_temp[lcd_cut].min++;
				if(counter_temp[lcd_cut].min>60)
					counter_temp[lcd_cut].min=0;
			}break;
			case 3:
			{
				counter_temp[lcd_cut].hour++;
				if(counter_temp[lcd_cut].hour>=24)
					counter_temp[lcd_cut].hour=0;
			}break;		
		};
		key[2].single_flag=0;
	}
	else if(key[2].single_flag==1)
		key[2].single_flag=0;
	
	
	if(key[2].long_flag==1&&lcd_setting==1)
	{
		switch(counter[lcd_cut].change_highlight)
		{
			case 1:(counter_temp[lcd_cut].sec>59)?(counter_temp[lcd_cut].sec=0):(counter_temp[lcd_cut].sec=counter_temp[lcd_cut].sec+1);break;
			case 2:(counter_temp[lcd_cut].min>59)?(counter_temp[lcd_cut].min=0):(counter_temp[lcd_cut].min=counter_temp[lcd_cut].min+1);break;
			case 3:(counter_temp[lcd_cut].hour>23)?(counter_temp[lcd_cut].hour=0):(counter_temp[lcd_cut].hour=counter_temp[lcd_cut].hour+1);break;
//			case 1:
//			{
//				counter_temp[lcd_cut].sec++;
//				if(counter_temp[lcd_cut].sec>60)
//					counter_temp[lcd_cut].sec=0;
//			}000000000000000000000000break;
//			case 2:
//			{
//				counter_temp[lcd_cut].min++;
//				if(counter_temp[lcd_cut].min>60)
//					counter_temp[lcd_cut].min=0;
//			}break;
//			case 3:
//			{
//				counter_temp[lcd_cut].hour++;
//				if(counter_temp[lcd_cut].hour>=24)
//					counter_temp[lcd_cut].hour=0;
//			}break;		
		};
		key[2].long_flag=0;
	}
	else if(key[2].long_flag==1)
		key[2].long_flag=0;
	
	////////////////////////////B4
	if(key[3].single_flag==1&&lcd_setting==0)
	{
		key[3].key_number++;
		key[3].single_flag=0;
	}
	else if(key[3].single_flag==1)
	{
		key[3].single_flag=0;
	}

	if(key[3].key_number%2!=0)
	{
		led_flag=1;
		pwm_flag=1;
		counter[lcd_cut].time_flag=1;
		counter[lcd_cut].status=1;
	}
	else if(key[3].key_number%2==0&&key[3].key_number!=0)
	{
		led_flag=0;
		pwm_flag=0;	
		counter[lcd_cut].time_flag=0;
		counter[lcd_cut].status=2;
	}
	else if(key[3].key_number==0)
	{
		led_flag=0;
		pwm_flag=0;
		counter_temp2[lcd_cut].hour=counter[lcd_cut].hour;
		counter_temp2[lcd_cut].min=counter[lcd_cut].min;
		counter_temp2[lcd_cut].sec=counter[lcd_cut].sec;
		
		counter[lcd_cut].time_flag=0;
		counter[lcd_cut].status=0;
	}
		
	if(key[3].long_flag==1)
	{
		counter[lcd_cut].hour=counter_temp2[lcd_cut].hour;
		counter[lcd_cut].min=counter_temp2[lcd_cut].min;
		counter[lcd_cut].sec=counter_temp2[lcd_cut].sec;
		
		key[3].key_number=0;
		key[3].long_flag=0;
	}
}

void highlight(char *str,u8 line, u8 column){
	int i=0;
	for(i=0;i<20;i++)
	{
		if(i!=column)
			LCD_DisplayChar(line,320-(16*i),str[i]);
	}
	LCD_SetBackColor(Yellow);
	LCD_DisplayChar(line,320-(16*column),str[column]);
	LCD_SetBackColor(White);
}

void highlight_double(char *str,u8 line, u8 column){
	int i=0;
	for(i=0;i<20;i++)
	{
		if(i!=column||i!=(column+1))
			LCD_DisplayChar(line,320-(16*i),str[i]);
	}
	LCD_SetBackColor(Yellow);
	LCD_DisplayChar(line,320-(16*column),str[column]);
	LCD_DisplayChar(line,320-(16*(column+1)),str[column+1]);
	LCD_SetBackColor(White);
}
void lcd_proc(){
	//switch(lcd_cut)
			LCD_SetTextColor(Black);
			sprintf(text,"  NO %d" ,lcd_cut+1);
			LCD_DisplayStringLine(Line0,(unsigned char *)text);	
			
		///////////line2
			LCD_SetTextColor(Black);
			if(lcd_setting)
				sprintf(text,"   %02d:%02d:%02d                            ",counter_temp[lcd_cut].hour,counter_temp[lcd_cut].min,counter_temp[lcd_cut].sec);
			else
				sprintf(text,"   %02d:%02d:%02d                            ",counter[lcd_cut].hour,counter[lcd_cut].min,counter[lcd_cut].sec);
			
			if(counter[lcd_cut].flag==1)
			{
				switch(counter[lcd_cut].change_highlight)
				{
					case 1://sec
					{	
						highlight_double(text,Line2, 9);//9 10
					}break;
					case 2://min
					{
						highlight_double(text,Line2, 6);
					}break;
					case 3://hour
					{
						highlight_double(text,Line2, 3);
					}break;
				}
			}
			else
				LCD_DisplayStringLine(Line2,(unsigned char *)text);
			///////////////////line4
			LCD_SetTextColor(Black);
			if(lcd_setting)
				sprintf(text,"  Setting   ");
			else
			{
				switch(counter[lcd_cut].status)
				{
					case 0:
						sprintf(text,"  Standby   ");break;
					case 1:
						sprintf(text,"  Running   ");break;
					case 2:
						sprintf(text,"  Stop   ");break;
				}
			}
			LCD_DisplayStringLine(Line4,(unsigned char *)text);
			

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
  MX_RTC_Init();
  MX_TIM3_Init();
  MX_TIM15_Init();
  MX_TIM16_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim3);
	HAL_TIM_Base_Start_IT(&htim15);
	HAL_TIM_PWM_Start(&htim16,TIM_CHANNEL_1);
	__HAL_TIM_SET_COMPARE(&htim16,TIM_CHANNEL_1,80);
	LCD_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
		
	LCD_Clear(White);
	LCD_SetBackColor(White);
	LCD_SetTextColor(Black);
	if(read_eeprom(0xff)!=30)
	{
		for(int i=1;i<=5;i++)
		{
			counter[i].hour=0;
			counter[i].min=0;
			counter[i].sec=0;
		}
		write_eeprom(0xff,30);
	}
	else
	{
		for(int i=0;i<=5;i++)
		{
			switch(i)
			{
				case 0:
				{
					counter[i].hour=read_eeprom(0x01);
					HAL_Delay(5);
					counter[i].min=read_eeprom(0x02);
					HAL_Delay(5);
					counter[i].sec=read_eeprom(0x03);
					HAL_Delay(5);
				}break;
				case 1:
				{
					counter[i].hour=read_eeprom(0x04);
					HAL_Delay(5);
					counter[i].min=read_eeprom(0x05);
					HAL_Delay(5);
					counter[i].sec=read_eeprom(0x06);
					HAL_Delay(5);
				}break;
				case 2:
				{
					counter[i].hour=read_eeprom(0x07);
					HAL_Delay(5);
					counter[i].min=read_eeprom(0x08);
					HAL_Delay(5);
					counter[i].sec=read_eeprom(0x09);
					HAL_Delay(5);
				}break;
				case 3:
				{
					counter[i].hour=read_eeprom(0x11);
					HAL_Delay(5);
					counter[i].min=read_eeprom(0x12);
					HAL_Delay(5);
					counter[i].sec=read_eeprom(0x13);
					HAL_Delay(5);
				}break;									
				case 4:
				{
					counter[i].hour=read_eeprom(0x14);
					HAL_Delay(5);
					counter[i].min=read_eeprom(0x15);
					HAL_Delay(5);
					counter[i].sec=read_eeprom(0x16);
					HAL_Delay(5);
				}break;	
			}			
		}
	}

  while (1)
  {
		led_off();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		key_proc();
		led_proc();
		lcd_proc();
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
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
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the peripherals clocks
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
int key_time=0;
int data=0;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance==TIM3)
	{
		if(counter[lcd_cut].time_flag==1)
		{
			counter[lcd_cut].sec+=1;
			if(counter[lcd_cut].sec>60)
			{
				counter[lcd_cut].min+=1;
				counter[lcd_cut].sec=0;
			}
			if(counter[lcd_cut].min>60)
			{
				counter[lcd_cut].hour+=1;
				counter[lcd_cut].min=0;
			}
			if(counter[lcd_cut].hour>=24)
				counter[lcd_cut].hour=0;
		}
	
	}
	
	if(htim->Instance==TIM15)
	{
		if(led_flag)
			led_count++;
		else
			led_count=0;
	
		key[0].key_sta=HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_0);
		key[1].key_sta=HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_1);
		key[2].key_sta=HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_2);
		key[3].key_sta=HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0);
		
		if(key_time>40)
		{
			for(int i=0;i<4;i++)
			{
				switch(key[i].key_judge)
				{
					case 0:
					{
						key[i].long_time=0;
						
						if(key[i].key_sta==0)
							key[i].key_judge=1;
					}break;
					case 1:
					{
						if(key[i].key_sta==0)
							key[i].key_judge=2;
						else
							key[i].key_judge=0;													
					}break;
					case 2:
					{
						if(key[i].key_sta==1&&key[i].long_time<20)
						{
							key[i].single_flag=1;
							key[i].key_judge=0;
						}
						else if(key[i].key_sta==1&&key[i].long_time>=20)
							key[i].key_judge=0;
						else
						{
							key[i].long_time++;
							if(key[i].long_time>=20)
								key[i].long_flag=1;
						}
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
