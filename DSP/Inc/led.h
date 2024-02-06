#ifndef	_LED_H_
#define _LED_H_

#include "main.h"
#include "gpio.h"
#include "tim.h"

#define LED_GPIO		GPIOB
#define	LED_PIN			GPIO_PIN_12

void Led_LedRunTask(void);
void Led_ledOff(void);
void Led_ledOn(void);

#endif
