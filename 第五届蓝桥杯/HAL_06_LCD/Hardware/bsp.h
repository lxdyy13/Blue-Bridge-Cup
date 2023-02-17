#ifndef _BSP_H_
#define _BSP_H_

struct Channel_para{
	int channel_req;
	int N;
};
#include "main.h"

struct Usart_get_data{
	int channel;
	int double_num1;
};
void test(void);
void deal_rx_data(void);
void receive_data(void);
void lcd_proc(void);
void Frq_setting(void);
void key_proc(void);
void led_display(void);
void frq_init(void);
void led_dispaly(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState);
#endif

