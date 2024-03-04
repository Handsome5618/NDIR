#ifndef _LED_H_
#define _LED_H_

#include "main.h"
#include "gpio.h"
#include "tim.h"

#define LED_GPIO GPIOA
#define LED_PIN  GPIO_PIN_15

void Led_LedRunTask(void);
void Led_ledOff(void);
void Led_ledOn(void);

#endif
