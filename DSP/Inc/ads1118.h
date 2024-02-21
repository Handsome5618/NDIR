#ifndef __ADS1118_H_
#define __ADS1118_H_

#include "gpio.h"
#include "main.h"
#include "usart.h"
#include "stdio.h"

// #include "spi.h"

#define SET_SS          0x8000 // ���βɼ�ģʽ��ʼ�ɼ�

#define CH0             0x4000 // ����ģʽ�ɼ�ͨ��
#define CH1             0x5000
#define CH2             0x6000
#define CH3             0x7000

#define V6_144          0x0000 // ��������
#define V4_096          0x0200
#define V2_048          0x0400
#define V1_024          0x0600
#define V0_512          0x0800
#define V0_256          0x0A00

#define MODE_SINGLE     0x0100 // ���βɼ�ģʽ
#define MODE_CONTINUE   0x0000 // �����ɼ�ģʽ

#define RATE_8SPS       0x0000 // �ɼ�����
#define RATE_16SPS      0x0020
#define RATE_32SPS      0x0040
#define RATE_64SPS      0x0060
#define RATE_128SPS     0x0080
#define RATE_250SPS     0x00A0
#define RATE_475SPS     0x00C0
#define RATE_860SPS     0x00E0

#define TEMPERATURE     0x0010 // �ɼ��¶�
#define GET_ADC         0x0000 // �ɼ��ⲿ��ѹ

#define PULL_UP_EN      0x0008 // �ڲ�����DOUT

#define VALID_SET       0x0002 // �˴�������Ч
#define INVALID_SET     0x0006 // �˴�������Ч

#define POWERON_DEFAULT 0x058A // оƬ�������Ĭ�ϳ�ʼֵ

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
