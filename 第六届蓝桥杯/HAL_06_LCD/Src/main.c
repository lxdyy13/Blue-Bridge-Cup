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
#include "rtc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
#include "i2c_hal.h"
int fputc(int ch,FILE *f)
{
	HAL_UART_Transmit (&huart1 ,(uint8_t *)&ch,1,HAL_MAX_DELAY );
	return ch;
}
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
struct KEY{
	_Bool key_sta;
	_Bool key_singe_flag;
	unsigned int key_judge;
};
struct KEY key[4]={0,0,0};

int test_flag=0;

struct TIME{
	uint8_t hour;
	uint8_t min;
	uint8_t sec;
	uint8_t flag;
}set_time={0,0,0,0},set_time_temp={0,0,0,0};


int key0_count=0;
int key1_count=0;
int key2_count=0;
int key3_count=0;

_Bool key0_flag=0;
_Bool key1_flag=0;
_Bool key2_flag=0;
_Bool key3_flag=0;

_Bool led_flag=0;

#define N 0.7

double Vdd=3.3f;
double K=0.1f;

extern RTC_AlarmTypeDef sAlarm;
RTC_TimeTypeDef rtc_time;
RTC_DateTypeDef rtc_date;

unsigned int key_time=0;
unsigned int led_delay_time=0;

unsigned int setting_choose=1;
unsigned int setting_time_choose=1;
unsigned int lcd_choose=1;

double adc_value=0.0f;
double last_adc_value=0.0f;

char lcd_text[20];

uint8_t Pdata;
int data_pointer=0;
char rx_data[30];
char tx_data[30];
_Bool tx_flag=0;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//获取ADC电压函数
void get_dac(){
	double value=0.0;
	HAL_ADC_Start(&hadc2);
		value=HAL_ADC_GetValue(&hadc2);
		adc_value=(float)((value/4096.0)*3.3);
		adc_value=0.7*adc_value+(1-0.7)*last_adc_value;
		last_adc_value=adc_value;
}
//关闭其余灯
void led_off(){
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_9|GPIO_PIN_10
                          |GPIO_PIN_11|GPIO_PIN_12, GPIO_PIN_SET);
	
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2, GPIO_PIN_RESET);
}
//led灯开关函数
void led_display(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState){
	HAL_GPIO_WritePin(GPIOx,GPIO_Pin, PinState);
	
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2, GPIO_PIN_RESET);
}
//显示屏处理函数
void lcd_proc(){
	if(key1_count%2==0)
	{
		if(lcd_choose==1)
		{
			LCD_Clear(White);
			lcd_choose=2;
		}
		//////////////////////////////////////////////////////1
		LCD_SetTextColor(Black);
		sprintf(lcd_text,"   V1_real:%.2fV        ",adc_value);
		LCD_DisplayStringLine(Line1,(unsigned char *)lcd_text);	
		//////////////////////////////////////////////////////2
		LCD_SetTextColor(Black);
		sprintf(lcd_text,"   V1_set:%.2fV        ",Vdd*K);
		LCD_DisplayStringLine(Line2,(unsigned char *)lcd_text);	
		//////////////////////////////////////////////////////3
		LCD_SetTextColor(Black);
		sprintf(lcd_text,"   K:%.2f        ",K);
		LCD_DisplayStringLine(Line3,(unsigned char *)lcd_text);	
		//////////////////////////////////////////////////////4
		if(adc_value>(Vdd*K)&&key0_count%2==0)
		{
			LCD_SetTextColor(Black);
			sprintf(lcd_text,"   LED:ON      ");
			LCD_DisplayStringLine(Line4,(unsigned char *)lcd_text);	
		}
		else
		{
			LCD_SetTextColor(Black);
			sprintf(lcd_text,"   LED:OFF        ");
			LCD_DisplayStringLine(Line4,(unsigned char *)lcd_text);
		}
		//////////////////////////////////////////////////////4
		LCD_SetTextColor(Black);
		sprintf(lcd_text,"   T:%d-%d-%d        ",rtc_time.Hours,rtc_time.Minutes,rtc_time.Seconds);
		LCD_DisplayStringLine(Line5,(unsigned char *)lcd_text);	
		
		LCD_SetTextColor(Black);
		sprintf(lcd_text,"   A:%d-%d-%d        ",set_time.hour,set_time.min,set_time.sec);
		LCD_DisplayStringLine(Line6,(unsigned char *)lcd_text);	
	}
	else////////设置界面
	{
		if(lcd_choose==2)
		{
			LCD_Clear(White);
			lcd_choose=1;
		}
		//////////////////////////////////////////////////////1
		LCD_SetTextColor(Black);
		sprintf(lcd_text,"   Setting");
		LCD_DisplayStringLine(Line1,(unsigned char *)lcd_text);	
		
		LCD_SetTextColor(Black);
		sprintf(lcd_text,"   T:%d-%d-%d        ",set_time_temp.hour,set_time_temp.min,set_time_temp.sec);
		LCD_DisplayStringLine(Line3,(unsigned char *)lcd_text);
	}

}
//按键处理函数
void key_proc(){
	if(key[0].key_singe_flag==1)
	{
		key0_count++;
		key[0].key_singe_flag=0;
	}
	else if(key[1].key_singe_flag==1)/////////////B2
	{
		key1_count++;
		if(key1_count%2==0)
		{
			setting_choose=1;////时间选择
			if(setting_time_choose==2)
			{
				set_time.hour=set_time_temp.hour;
				set_time.min=set_time_temp.min;
				set_time.sec=set_time_temp.sec;
				setting_time_choose=1;
			}
		}
		else
		{
				set_time_temp.hour=set_time.hour;
				set_time_temp.min=set_time.min;
				set_time_temp.sec=set_time.sec;
		}
		key[1].key_singe_flag=0;
	}
	else if(key[2].key_singe_flag==1)///////////B3
	{
		if(key1_count%2!=0)
		{
			if(setting_choose==1)
			{
				setting_choose=2;
			}
			else if(setting_choose==2)
			{
				setting_choose=3;
			}
			else if(setting_choose==3)
			{
				setting_choose=1;
			}
		}
		if(setting_time_choose==1)
		setting_time_choose=2;
		key[2].key_singe_flag=0;
	}
	else if(key[3].key_singe_flag==1)////////B4
	{
		if(key1_count%2!=0)
		{
			if(setting_choose==1)
			{
				set_time_temp.sec+=1;
				if(set_time_temp.sec==60) set_time_temp.sec=0;
			}
			else if(setting_choose==2)
			{
				set_time_temp.min+=1;
				if(set_time_temp.min==60) set_time_temp.min=0;
			}
			else if(setting_choose==3)
			{
				set_time_temp.hour+=1;
				if(set_time_temp.hour==24) set_time_temp.hour=0;
			}
		}
		if(setting_time_choose==1)
		setting_time_choose=2;
		key[3].key_singe_flag=0;
	}

}
//led灯任务函数
void led_proc(){
	if(key0_count%2==0)
	{
		if(adc_value>(Vdd*K))
		{
			led_flag=1;
			if(led_delay_time<200)
			{
				led_display(GPIOC,GPIO_PIN_8,GPIO_PIN_RESET);//开灯
			}
			else if(led_delay_time>=200&&led_delay_time<=400)
			{
				led_display(GPIOC,GPIO_PIN_8,GPIO_PIN_SET);//关灯
			}
			else
			{
				led_delay_time=0;
			}
			
		}
		else
		{
			led_flag=0;
			led_delay_time=0;
			led_display(GPIOC,GPIO_PIN_8,GPIO_PIN_SET);//关灯
		}
	}
	else
	{
		led_display(GPIOC,GPIO_PIN_8,GPIO_PIN_SET);//关灯
	}

}
//处理串口数据函数
void deal_data(){
	if(data_pointer>0)
	{
		if(data_pointer==6)
		{
			if(rx_data[1]=='0')
			{
				K=(rx_data[3]-48);
				write_eeprom(1,K);
				K=K*0.1;
				HAL_Delay(10);
				HAL_Delay(10);
				printf("ok\n");
			}
			data_pointer=0;
		}
		else
		{
			sprintf(tx_data,"error\r\n");
		HAL_UART_Transmit(&huart1, (uint8_t*)tx_data, strlen(tx_data),100);
			data_pointer=0;
		}
	}
}
////串口接收处理函数
void usart_proc(){
	int temp=data_pointer;
	HAL_Delay(2);
	if(temp==data_pointer)
		deal_data();
	
	if(rtc_time.Hours==set_time.hour&&rtc_time.Minutes==set_time.min&&rtc_time.Seconds==set_time.sec&&tx_flag==0)
	{
		sprintf(tx_data,"%.2f+%.2f+%d%d%d\n",adc_value,K,rtc_time.Hours,rtc_time.Minutes,rtc_time.Seconds);
		HAL_UART_Transmit(&huart1, (uint8_t*)tx_data, strlen(tx_data),100);
		tx_flag=1;
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
  MX_ADC2_Init();
  MX_ADC1_Init();
  MX_RTC_Init();
  MX_TIM15_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	
	HAL_UART_Receive_IT(&huart1, &Pdata, 1);
	
	HAL_TIM_Base_Start_IT(&htim15);
	
	HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BIN);
	HAL_RTC_GetTime(&hrtc, &rtc_time, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &rtc_date, RTC_FORMAT_BIN);
	LCD_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	LCD_Clear(White);
	LCD_SetBackColor(White);
	LCD_SetTextColor(Black);
///////////////eeprom初始化	
	if(read_eeprom(0xff)!=20)
	{
		HAL_Delay(5);
		K=0.1;
		write_eeprom(0xff,20);
		HAL_Delay(10);
	}
	else
	{
		HAL_Delay(5);
		K=read_eeprom(1);
		HAL_Delay(5);
		K=K*0.1;
	}
  while (1)
  {
		led_off();
		get_dac();
		key_proc();
		led_proc();
		usart_proc();
		lcd_proc();	
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
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
//中断回调函数
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart->Instance==USART1)
	{
		rx_data[data_pointer++]=Pdata;
		HAL_UART_Receive_IT(&huart1, &Pdata, 1);
	}
}


void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc){
	tx_flag=0;
	HAL_RTC_GetTime(hrtc, &rtc_time, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(hrtc, &rtc_date, RTC_FORMAT_BIN);
	sAlarm.AlarmTime.Seconds=rtc_time.Seconds+1;
	if(sAlarm.AlarmTime.Seconds==60)sAlarm.AlarmTime.Seconds=0;
	HAL_RTC_SetAlarm_IT(hrtc, &sAlarm, RTC_FORMAT_BIN);
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance==TIM15)
	{
		if(led_flag)
		{
			led_delay_time++;
		}
		
		if(key_time>=80)
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
							key[i].key_singe_flag=1;
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
