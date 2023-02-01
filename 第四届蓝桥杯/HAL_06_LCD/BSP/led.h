#ifndef _LED_H_
#define _LED_H_

#include "main.h"

void led_disp(uchar led);
void led_one_show(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState);
#endif

