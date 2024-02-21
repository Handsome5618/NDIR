#ifndef __ADS1118_H_
#define __ADS1118_H_

#include "gpio.h"
#include "main.h"
#include "usart.h"
#include "stdio.h"

// #include "spi.h"

#define SET_SS          0x8000 // 单次采集模式开始采集

#define CH0             0x4000 // 单端模式采集通道
#define CH1             0x5000
#define CH2             0x6000
#define CH3             0x7000

#define V6_144          0x0000 // 正负量程
#define V4_096          0x0200
#define V2_048          0x0400
#define V1_024          0x0600
#define V0_512          0x0800
#define V0_256          0x0A00

#define MODE_SINGLE     0x0100 // 单次采集模式
#define MODE_CONTINUE   0x0000 // 连续采集模式

#define RATE_8SPS       0x0000 // 采集速率
#define RATE_16SPS      0x0020
#define RATE_32SPS      0x0040
#define RATE_64SPS      0x0060
#define RATE_128SPS     0x0080
#define RATE_250SPS     0x00A0
#define RATE_475SPS     0x00C0
#define RATE_860SPS     0x00E0

#define TEMPERATURE     0x0010 // 采集温度
#define GET_ADC         0x0000 // 采集外部电压

#define PULL_UP_EN      0x0008 // 内部上拉DOUT

#define VALID_SET       0x0002 // 此次设置生效
#define INVALID_SET     0x0006 // 此次设置无效

#define POWERON_DEFAULT 0x058A // 芯片开机后的默认初始值

#define ADS1118_CS_GPIO GPIOA
#define ADS1118_CS_PIN  GPIO_PIN_4

typedef enum {
    CHANNEL_IN1 = 0x4000,
    CHANNEL_IN2 = 0x5000,
    CHANNEL_IN3 = 0x6000,
    CHANNEL_IN4 = 0x7000
} ADS1118_CHANNEL;

void ADS1118_Init(void);
void ADS1118_Read_ADC(uint16_t Channel, float *pData, uint16_t Size);

void MySPI_Init(void);
void MySPI_Start(void);
void MySPI_Stop(void);
uint8_t MySPI_SwapByte(uint8_t ByteSend);

#define ADC_CS_PIN         GPIO_PIN_4
#define ADC_CS_GPIO_PORT   GPIOA
#define ADC_CLK_Pin        GPIO_PIN_5
#define ADC_CLK_GPIO_Port  GPIOA
#define ADC_MOSI_Pin       GPIO_PIN_7
#define ADC_MOSI_GPIO_Port GPIOA
#define ADC_MISO_Pin       GPIO_PIN_6
#define ADC_MISO_GPIO_Port GPIOA

// char ADS1118_Init(void);
// char ADS1118_Get_CH0(float *val);
// char ADS1118_Get_CH1(float *val);
// char ADS1118_Get_CH2(float *val);
// char ADS1118_Get_CH3(float *val);
// char ADS1118_Get_Temperature(float *Temperature);

#endif
