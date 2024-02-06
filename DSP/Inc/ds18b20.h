#ifndef __DS18B20_H_
#define __DS18B20_H_

#include "main.h"
#include "gpio.h"

#define DS18B20_Dout_GPIO_CLK_ENABLE()              __HAL_RCC_GPIOD_CLK_ENABLE()
#define DS18B20_Dout_PORT                           GPIOA
#define DS18B20_Dout_PIN                            GPIO_PIN_3

/***********************   DS18B20 函数宏定义  ****************************/
#define DS18B20_Dout_LOW()                          HAL_GPIO_WritePin(DS18B20_Dout_PORT,DS18B20_Dout_PIN,GPIO_PIN_RESET)
#define DS18B20_Dout_HIGH()                         HAL_GPIO_WritePin(DS18B20_Dout_PORT,DS18B20_Dout_PIN,GPIO_PIN_SET)
#define DS18B20_Data_IN()                           HAL_GPIO_ReadPin(DS18B20_Dout_PORT,DS18B20_Dout_PIN)

/* 扩展变量 ------------------------------------------------------------------*/
/* 函数声明 ------------------------------------------------------------------*/
uint8_t DS18B20_Init(void);
void DS18B20_ReadId( uint8_t * ds18b20_id);
float DS18B20_GetTemp(void);

#endif
