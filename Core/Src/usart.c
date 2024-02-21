/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    usart.c
 * @brief   This file provides code for the configuration
 *          of the USART instances.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "usart.h"

/* USER CODE BEGIN 0 */
#include <stdio.h>

uint16_t Serial1_RxState;              /*串口接收状态标志*/
uint8_t Serial1_RxData[USART_REC_LEN]; /*串口接收数组*/
uint8_t Serial1_RxBuff[RXBUFFERSIZE];  /*串口缓存接收数组*/
/* USER CODE END 0 */

UART_HandleTypeDef huart1;

/* USART1 init function */

void MX_USART1_UART_Init(void)
{

    /* USER CODE BEGIN USART1_Init 0 */

    /* USER CODE END USART1_Init 0 */

    /* USER CODE BEGIN USART1_Init 1 */

    /* USER CODE END USART1_Init 1 */
    huart1.Instance          = USART1;
    huart1.Init.BaudRate     = 115200;
    huart1.Init.WordLength   = UART_WORDLENGTH_8B;
    huart1.Init.StopBits     = UART_STOPBITS_1;
    huart1.Init.Parity       = UART_PARITY_NONE;
    huart1.Init.Mode         = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart1) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN USART1_Init 2 */

    /* USER CODE END USART1_Init 2 */
}

void HAL_UART_MspInit(UART_HandleTypeDef *uartHandle)
{

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if (uartHandle->Instance == USART1) {
        /* USER CODE BEGIN USART1_MspInit 0 */

        /* USER CODE END USART1_MspInit 0 */
        /* USART1 clock enable */
        __HAL_RCC_USART1_CLK_ENABLE();

        __HAL_RCC_GPIOA_CLK_ENABLE();
        /**USART1 GPIO Configuration
        PA9     ------> USART1_TX
        PA10     ------> USART1_RX
        */
        GPIO_InitStruct.Pin   = GPIO_PIN_9;
        GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        GPIO_InitStruct.Pin  = GPIO_PIN_10;
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        /* USART1 interrupt Init */
        HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(USART1_IRQn);
        /* USER CODE BEGIN USART1_MspInit 1 */

        /* USER CODE END USART1_MspInit 1 */
    }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef *uartHandle)
{

    if (uartHandle->Instance == USART1) {
        /* USER CODE BEGIN USART1_MspDeInit 0 */

        /* USER CODE END USART1_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_USART1_CLK_DISABLE();

        /**USART1 GPIO Configuration
        PA9     ------> USART1_TX
        PA10     ------> USART1_RX
        */
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9 | GPIO_PIN_10);

        /* USART1 interrupt Deinit */
        HAL_NVIC_DisableIRQ(USART1_IRQn);
        /* USER CODE BEGIN USART1_MspDeInit 1 */

        /* USER CODE END USART1_MspDeInit 1 */
    }
}

/* USER CODE BEGIN 1 */
/* 输出重定向 printf */
extern UART_HandleTypeDef huart1;

#ifdef __GNUC__
/* With GCC, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

PUTCHAR_PROTOTYPE
{
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);

    return ch;
}

int _write(int file, char *ptr, int len)
{
    int DataIdx;

    for (DataIdx = 0; DataIdx < len; DataIdx++) {
        __io_putchar(*ptr++);
    }
    return len;
}

/**
 * @brief    Rx传输回调函数
 * @param  	 huart: UART句柄类型指针
 * @retval   无
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) /* 如果是串口1 */
    {
        if ((Serial1_RxState & 0x8000) == 0) /* 接收未完成 */
        {
            if (Serial1_RxState & 0x4000) {
                if (Serial1_RxBuff[0] != 0x0a) {
                    Serial1_RxState = 0;
                } else {
                    Serial1_RxState |= 0x8000;
                }
            } else {
                if (Serial1_RxBuff[0] == 0x0d) {
                    Serial1_RxState |= 0x4000;
                } else {
                    Serial1_RxData[Serial1_RxState & 0X3FFF] = Serial1_RxBuff[0];
                    Serial1_RxState++;
                    if (Serial1_RxState > (USART_REC_LEN - 1)) {
                        Serial1_RxState = 0; /* 接收数据错误,重新开始接收 */
                    }
                }
            }
        }
        HAL_UART_Receive_IT(&huart1, (uint8_t *)Serial1_RxBuff, RXBUFFERSIZE);
    }
}

/**
 * @brief   判断串口1接收状态接口
 * @param	无
 * @retval 	接收完成返回1 接收失败返回0
 */
uint8_t Serial_GetSerial1_RxFlag(void)
{
    if (Serial1_RxState & 0x8000) {
        return 1;
    }
    return 0;
}

/**
 * @brief   清除串口1接收标志位
 * @param	无
 * @retval 	无
 */
void Serial_ClearSerial1_RxFlag(void)
{
    Serial1_RxState = 0;
    for (uint16_t i = 0; i < USART_REC_LEN; i++) {
        Serial1_RxData[i] = 0x00;
    }
}

/**
 * @brief   获取串口1接收数组大小
 * @param	无
 * @retval 	接收数组地址
 */
uint16_t Serial_GetSerial1_RxDataNum(void)
{
    return (Serial1_RxState & 0x3FFF);
}

/**
 * @brief   获取串口1接收数组地址
 * @param	无
 * @retval 	接收数组地址
 */
uint8_t *Serial_GetSerial1_RxData(void)
{
    return Serial1_RxData;
}
/* USER CODE END 1 */
