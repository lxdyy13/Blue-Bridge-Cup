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
#include "led.h"
#include "interrput.h"
#include "i2c_hal.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define eeprom_first_flag 50 //第一次上电保持不变需要修改

void lcd_dispaly(void);
void time_computer(void);
void key_proc(void);
void led_show(void);
void time_setting(void);
void eeprom_bsp(void);

#define eeprom_delay 5

extern struct KEYS keys[];
extern int time_flag;
extern char receive_data[];
extern int data_pointer;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
union EEPROM_DATA {
	uint32_t ID;
	uint8_t str[8];
} My_ID;

struct TIME{
	uint hour;
	uint min;
	uint sec;
	
	uint hour_set;
	uint min_set;
	uint sec_set;
	
	uint delay_time;
}time={23,59,50,99,99,99,0};

	
//	time.hour=23;
//	time.min=59;
//	time.sec=50;
//	
//	time.hour_set=99;
//	time.min_set=99;
//	time.sec_set=99;
//	
//	time.delay_time=0;

int flagg=1;//时间自锁标志位

int pa6_flag=1; //开机处于不调制状态
int pa7_flag=1;

int time_set_flag=0;
int time_set_real=0;

int pa6_status_flag=0;
int pa7_status_flag=0;

int rx_show_flag=0;

int pa6_duty=80;
int pa7_duty=10; 

uint8_t P_uart_data;

uint pwm_select=0;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
char lcd_buff[25];
char rx_buff[25];
char temp_rx_data[30];
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
  MX_TIM3_Init();
  MX_TIM15_Init();
  MX_TIM16_Init();
  MX_TIM17_Init();
  MX_USART1_UART_Init();
	
	
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim15); //按键定时器
	LCD_Init();
	I2CInit();
	HAL_UART_Receive_IT(&huart1, &P_uart_data,1);
	led_disp(0x00);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	LCD_Clear(Black);
	LCD_SetBackColor(Black);
	LCD_SetTextColor(White);
	
	eeprom_bsp();
	pa6_duty=eeprom_read(0x00);
	HAL_Delay(eeprom_delay);
	pa7_duty=eeprom_read(0x01);
	HAL_Delay(eeprom_delay);
	
  while (1)
  {
    /* USER CODE END WHILE */
		
    /* USER CODE BEGIN 3 */
		time_computer();
		
		key_proc();
		
		time_setting();
		
		lcd_dispaly();
		led_show();
		if(data_pointer!=0)
		{
			int temp=data_pointer;
			HAL_Delay(1);
			if(temp==data_pointer)
				data_deal();
		}
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
void eeprom_bsp(){
		if(eeprom_read(0xff)!=eeprom_first_flag){
		   	HAL_Delay(eeprom_delay);
				eeprom_write(0x00,80);
				HAL_Delay(eeprom_delay);
			  eeprom_write(0x01,10);
				HAL_Delay(eeprom_delay);
				eeprom_write(0xff,eeprom_first_flag);
				HAL_Delay(eeprom_delay);
		}
}
void key_proc(){
	if(keys[0].key_short_flag==1)
	{
		pa6_flag++;
		keys[0].key_short_flag=0;
	}
	
	if((pa6_flag%2==0)&&(keys[1].key_short_flag==1))
	{
		if(pa6_duty==90)
			pa6_duty=10;
		else
			pa6_duty+=10;
		
		//pwm_pa6_value=pa6_duty;
		//deal_eeprom();
		//wirte_eeprom(0,pa6_duty);
		eeprom_write(0,pa6_duty);
		HAL_Delay(eeprom_delay);
		
		keys[1].key_short_flag=0;
	}
	else if(keys[1].key_short_flag==1)
	{
		keys[1].key_short_flag=0;
	}
	
	
	//pa7 b1
	if(keys[2].key_short_flag==1)
	{
		pa7_flag++;
		keys[2].key_short_flag=0;
	}
	//pa7 b2
	if((pa7_flag%2==0)&&(keys[3].key_short_flag==1))
	{
		if(pa7_duty==90)
			pa7_duty=10;
		else
			pa7_duty+=10;
		
		//pwm_pa7_value=pa7_duty;
		//deal_eeprom();
		//wirte_eeprom(1,pa7_duty);
		eeprom_write(1,pa7_duty);
		HAL_Delay(eeprom_delay);
		
		keys[3].key_short_flag=0;
	}
	else if(keys[3].key_short_flag==1)
	{
		keys[3].key_short_flag=0;
	}
}
void time_computer(){
	if(time_flag>=1000)
	{
		time_set_real+=1;//设定时间计时
		time.sec+=1;
		time_flag=1;
	}
	if(time.sec==60)
	{
		time.min+=1;
		time.sec=0;
	}
	if(time.min==60)
	{
		time.hour+=1;
		time.min=0;
	}
	if(time.hour==24)
	{
		time.sec=0;
		time.min=0;
		time.hour=0;
	}
}
void data_deal(){
	if(data_pointer>0)
	{
		if(data_pointer==15)
		{
			time.hour_set=((receive_data[0])-48)*10+(receive_data[1]-48);
			time.min_set=((receive_data[3])-48)*10+(receive_data[4]-48);
			time.sec_set=((receive_data[6])-48)*10+(receive_data[7]-48);
			
			pwm_select=((receive_data[11])-48);
			
			time.delay_time=((receive_data[13])-48);
			//sscanf(receive_data,"%2s:%2s:%2s-%3s-%2s",rx_data1.data12,rx_data1.data34,rx_data1.data56,rx_data1.data789,rx_data1.data10);
			for(int i=0;i<30;i++)
				temp_rx_data[i]=receive_data[i];
			rx_show_flag=1;
		}
		else
		{
			sprintf(rx_buff,"Error\r\n");
			HAL_UART_Transmit(&huart1,(uint8_t *)rx_buff,strlen(rx_buff),100);
			rx_show_flag=0;
		}
		data_pointer=0;
		memset(receive_data,0,30);
	}
}
void time_setting(){
	if(time.hour==time.hour_set&&time.min==time.min_set&&time.sec==time.sec_set)
	{
		if(flagg)
		{	if(pa6_flag%2!=0)
			{
				if(pwm_select==6)
				{
					pa6_flag+=1;
				}
			}
			if(pa7_flag%2!=0)
			{
				if(pwm_select==7)
				{
					pa7_flag+=1;
				}
			}
			time_set_real=0;//计时开始
			time_set_flag=1;
			flagg=0;
		}
	}
	if(time_set_flag==1)
	{
		if(time_set_real<=time.delay_time)
		{
		}
		else
		{	
			if(pwm_select==6)
			{
				//if(pa6_flag%2!=0)
					pa6_flag+=1;
			}
			if(pwm_select==7)
			{
				//if(pa6_flag%2!=0)
					pa7_flag+=1;
			}
			flagg=1;
			rx_show_flag=0;//关闭显示接收
			pwm_select=0;
			time_set_flag=0;
		}
	}

}


void lcd_dispaly(){
	
	
	if(pa6_flag%2==0)//调制状态
	{
		//pwm_pa6_value=(read_eeprom(1)<<8)+read_eeprom(2);
		sprintf(lcd_buff,"  PWM-PA6:%d%%          ",pa6_duty);
		LCD_DisplayStringLine(Line0,(unsigned char *)lcd_buff);
		//led_one_show(GPIOC,GPIO_PIN_8,GPIO_PIN_RESET);
	}
	else 
	{
		sprintf(lcd_buff,"  PWM-PA6:0          ");
		LCD_DisplayStringLine(Line0,(unsigned char *)lcd_buff);
		//led_one_show(GPIOC,GPIO_PIN_8,GPIO_PIN_SET);
	}
	
	if(pa7_flag%2==0)//调制状态
	{
		//pwm_pa7_value=(read_eeprom(5)<<8)+read_eeprom(6);
		sprintf(lcd_buff,"  PWM-PA7:%d%%          ",pa7_duty);
		LCD_DisplayStringLine(Line2,(unsigned char *)lcd_buff);
		//led_one_show(GPIOC,GPIO_PIN_9,GPIO_PIN_RESET);
	}
	else
	{
		sprintf(lcd_buff,"  PWM-PA7:0          ");
		LCD_DisplayStringLine(Line2,(unsigned char *)lcd_buff);
		//led_one_show(GPIOC,GPIO_PIN_9,GPIO_PIN_SET);
	}
	
	sprintf(lcd_buff,"  Time:%d:%d:%d          ",time.hour,time.min,time.sec);
	LCD_DisplayStringLine(Line4,(unsigned char *)lcd_buff);
	
	if(pwm_select==6||pwm_select==7)
	{
		sprintf(lcd_buff,"  Channel:PA%d          ",pwm_select);
		LCD_DisplayStringLine(Line6,(unsigned char *)lcd_buff);
	}
	else
	{
		sprintf(lcd_buff,"  Channel:           ");
		LCD_DisplayStringLine(Line6,(unsigned char *)lcd_buff);
	}
	
	sprintf(lcd_buff,"  Command:           ");
	LCD_DisplayStringLine(Line8,(unsigned char *)lcd_buff);
	
	if(rx_show_flag)
	{
	sprintf(lcd_buff,"  %15s",temp_rx_data);
	LCD_DisplayStringLine(Line9,(unsigned char *)lcd_buff);
	}
	else
	{
		sprintf(lcd_buff,"          None           ");
		LCD_DisplayStringLine(Line9,(unsigned char *)lcd_buff);
	}
}

void led_show(){
	if(pa6_flag%2==0&&pa7_flag%2!=0)//仅PA6调制状态
	{
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_All,GPIO_PIN_SET);
		
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_RESET);
		
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_RESET);
	}
	else if(pa6_flag%2!=0&&pa7_flag%2==0)//仅PA7调制状态
	{
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_All,GPIO_PIN_SET);
		
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_RESET);
		
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_RESET);

	}
	else if(pa6_flag%2==0&&pa7_flag%2==0)//pa6 pa7调制状态
	{
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_All,GPIO_PIN_SET);
		
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_RESET);
		
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_RESET);

	}
	else if(pa6_flag%2!=0&&pa7_flag%2!=0)//pa6 pa7都不调制状态
	{
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_SET);	
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_All,GPIO_PIN_SET);
		
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_RESET);

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
