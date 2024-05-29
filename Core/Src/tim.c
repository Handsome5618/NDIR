/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    tim.c
 * @brief   This file provides code for the configuration
 *          of the TIM instances.
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
#include "tim.h"

/* USER CODE BEGIN 0 */
uint8_t RI_Status   = 0;
uint32_t ADC_Status = 0;
float Goal_Pow      = 450.0;
float Goal_Voltage  = 5700.0;
extern Mypid_t IR_PID;
uint8_t Rx_Time;
/* USER CODE END 0 */

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim15;

/* TIM2 init function */
void MX_TIM2_Init(void)
{

    /* USER CODE BEGIN TIM2_Init 0 */

    /* USER CODE END TIM2_Init 0 */

    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig     = {0};
    TIM_OC_InitTypeDef sConfigOC              = {0};

    /* USER CODE BEGIN TIM2_Init 1 */

    /* USER CODE END TIM2_Init 1 */
    htim2.Instance               = TIM2;
    htim2.Init.Prescaler         = 1 - 1;
    htim2.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim2.Init.Period            = 9999;
    htim2.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }
    sConfigOC.OCMode     = TIM_OCMODE_PWM2;
    sConfigOC.Pulse      = 8000;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM2_Init 2 */
    HAL_TIM_Base_Start_IT(&htim2);
    /* USER CODE END TIM2_Init 2 */
    HAL_TIM_MspPostInit(&htim2);
}
/* TIM3 init function */
void MX_TIM3_Init(void)
{

    /* USER CODE BEGIN TIM3_Init 0 */

    /* USER CODE END TIM3_Init 0 */

    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig     = {0};

    /* USER CODE BEGIN TIM3_Init 1 */

    /* USER CODE END TIM3_Init 1 */
    htim3.Instance               = TIM3;
    htim3.Init.Prescaler         = 15000 - 1;
    htim3.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim3.Init.Period            = 9999;
    htim3.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM3_Init 2 */
    HAL_TIM_Base_Start_IT(&htim3);
    /* USER CODE END TIM3_Init 2 */
}
/* TIM4 init function */
void MX_TIM4_Init(void)
{

    /* USER CODE BEGIN TIM4_Init 0 */

    /* USER CODE END TIM4_Init 0 */

    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig     = {0};

    /* USER CODE BEGIN TIM4_Init 1 */

    /* USER CODE END TIM4_Init 1 */
    htim4.Instance               = TIM4;
    htim4.Init.Prescaler         = 15000 - 1;
    htim4.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim4.Init.Period            = 4999;
    htim4.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim4) != HAL_OK) {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK) {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM4_Init 2 */
    // HAL_TIM_Base_Start_IT(&htim4);
    /* USER CODE END TIM4_Init 2 */
}
/* TIM5 init function */
void MX_TIM5_Init(void)
{

    /* USER CODE BEGIN TIM5_Init 0 */

    /* USER CODE END TIM5_Init 0 */

    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig     = {0};

    /* USER CODE BEGIN TIM5_Init 1 */

    /* USER CODE END TIM5_Init 1 */
    htim5.Instance               = TIM5;
    htim5.Init.Prescaler         = 1500 - 1;
    htim5.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim5.Init.Period            = 99;
    htim5.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim5) != HAL_OK) {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK) {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM5_Init 2 */
    // HAL_TIM_Base_Start_IT(&htim5);
    /* USER CODE END TIM5_Init 2 */
}
/* TIM6 init function */
void MX_TIM6_Init(void)
{

    /* USER CODE BEGIN TIM6_Init 0 */

    /* USER CODE END TIM6_Init 0 */

    TIM_MasterConfigTypeDef sMasterConfig = {0};

    /* USER CODE BEGIN TIM6_Init 1 */

    /* USER CODE END TIM6_Init 1 */
    htim6.Instance               = TIM6;
    htim6.Init.Prescaler         = 1500 - 1;
    htim6.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim6.Init.Period            = 49999;
    htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim6) != HAL_OK) {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM6_Init 2 */
    HAL_TIM_Base_Start_IT(&htim6);
    /* USER CODE END TIM6_Init 2 */
}
/* TIM7 init function */
void MX_TIM7_Init(void)
{

    /* USER CODE BEGIN TIM7_Init 0 */

    /* USER CODE END TIM7_Init 0 */

    TIM_MasterConfigTypeDef sMasterConfig = {0};

    /* USER CODE BEGIN TIM7_Init 1 */

    /* USER CODE END TIM7_Init 1 */
    htim7.Instance               = TIM7;
    htim7.Init.Prescaler         = 150 - 1;
    htim7.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim7.Init.Period            = 999;
    htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim7) != HAL_OK) {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM7_Init 2 */

    /* USER CODE END TIM7_Init 2 */
}
/* TIM15 init function */
void MX_TIM15_Init(void)
{

    /* USER CODE BEGIN TIM15_Init 0 */

    /* USER CODE END TIM15_Init 0 */

    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig     = {0};

    /* USER CODE BEGIN TIM15_Init 1 */

    /* USER CODE END TIM15_Init 1 */
    htim15.Instance               = TIM15;
    htim15.Init.Prescaler         = 150 - 1;
    htim15.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim15.Init.Period            = 799;
    htim15.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim15.Init.RepetitionCounter = 0;
    htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim15) != HAL_OK) {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim15, &sClockSourceConfig) != HAL_OK) {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM15_Init 2 */
    HAL_TIM_Base_Start_IT(&htim15);
    /* USER CODE END TIM15_Init 2 */
}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *tim_baseHandle)
{

    if (tim_baseHandle->Instance == TIM2) {
        /* USER CODE BEGIN TIM2_MspInit 0 */

        /* USER CODE END TIM2_MspInit 0 */
        /* TIM2 clock enable */
        __HAL_RCC_TIM2_CLK_ENABLE();
        /* USER CODE BEGIN TIM2_MspInit 1 */

        /* USER CODE END TIM2_MspInit 1 */
    } else if (tim_baseHandle->Instance == TIM3) {
        /* USER CODE BEGIN TIM3_MspInit 0 */

        /* USER CODE END TIM3_MspInit 0 */
        /* TIM3 clock enable */
        __HAL_RCC_TIM3_CLK_ENABLE();

        /* TIM3 interrupt Init */
        HAL_NVIC_SetPriority(TIM3_IRQn, 3, 0);
        HAL_NVIC_EnableIRQ(TIM3_IRQn);
        /* USER CODE BEGIN TIM3_MspInit 1 */

        /* USER CODE END TIM3_MspInit 1 */
    } else if (tim_baseHandle->Instance == TIM4) {
        /* USER CODE BEGIN TIM4_MspInit 0 */

        /* USER CODE END TIM4_MspInit 0 */
        /* TIM4 clock enable */
        __HAL_RCC_TIM4_CLK_ENABLE();

        /* TIM4 interrupt Init */
        HAL_NVIC_SetPriority(TIM4_IRQn, 2, 0);
        HAL_NVIC_EnableIRQ(TIM4_IRQn);
        /* USER CODE BEGIN TIM4_MspInit 1 */

        /* USER CODE END TIM4_MspInit 1 */
    } else if (tim_baseHandle->Instance == TIM5) {
        /* USER CODE BEGIN TIM5_MspInit 0 */

        /* USER CODE END TIM5_MspInit 0 */
        /* TIM5 clock enable */
        __HAL_RCC_TIM5_CLK_ENABLE();

        /* TIM5 interrupt Init */
        HAL_NVIC_SetPriority(TIM5_IRQn, 1, 0);
        HAL_NVIC_EnableIRQ(TIM5_IRQn);
        /* USER CODE BEGIN TIM5_MspInit 1 */

        /* USER CODE END TIM5_MspInit 1 */
    } else if (tim_baseHandle->Instance == TIM6) {
        /* USER CODE BEGIN TIM6_MspInit 0 */

        /* USER CODE END TIM6_MspInit 0 */
        /* TIM6 clock enable */
        __HAL_RCC_TIM6_CLK_ENABLE();
        /* USER CODE BEGIN TIM6_MspInit 1 */

        /* USER CODE END TIM6_MspInit 1 */
    } else if (tim_baseHandle->Instance == TIM7) {
        /* USER CODE BEGIN TIM7_MspInit 0 */

        /* USER CODE END TIM7_MspInit 0 */
        /* TIM7 clock enable */
        __HAL_RCC_TIM7_CLK_ENABLE();

        /* TIM7 interrupt Init */
        HAL_NVIC_SetPriority(TIM7_DAC_IRQn, 4, 0);
        HAL_NVIC_EnableIRQ(TIM7_DAC_IRQn);
        /* USER CODE BEGIN TIM7_MspInit 1 */

        /* USER CODE END TIM7_MspInit 1 */
    } else if (tim_baseHandle->Instance == TIM15) {
        /* USER CODE BEGIN TIM15_MspInit 0 */

        /* USER CODE END TIM15_MspInit 0 */
        /* TIM15 clock enable */
        __HAL_RCC_TIM15_CLK_ENABLE();
        /* USER CODE BEGIN TIM15_MspInit 1 */

        /* USER CODE END TIM15_MspInit 1 */
    }
}
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *timHandle)
{

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if (timHandle->Instance == TIM2) {
        /* USER CODE BEGIN TIM2_MspPostInit 0 */

        /* USER CODE END TIM2_MspPostInit 0 */

        __HAL_RCC_GPIOB_CLK_ENABLE();
        /**TIM2 GPIO Configuration
        PB11     ------> TIM2_CH4
        */
        GPIO_InitStruct.Pin       = GPIO_PIN_11;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        /* USER CODE BEGIN TIM2_MspPostInit 1 */

        /* USER CODE END TIM2_MspPostInit 1 */
    }
}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef *tim_baseHandle)
{

    if (tim_baseHandle->Instance == TIM2) {
        /* USER CODE BEGIN TIM2_MspDeInit 0 */

        /* USER CODE END TIM2_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_TIM2_CLK_DISABLE();
        /* USER CODE BEGIN TIM2_MspDeInit 1 */

        /* USER CODE END TIM2_MspDeInit 1 */
    } else if (tim_baseHandle->Instance == TIM3) {
        /* USER CODE BEGIN TIM3_MspDeInit 0 */

        /* USER CODE END TIM3_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_TIM3_CLK_DISABLE();

        /* TIM3 interrupt Deinit */
        HAL_NVIC_DisableIRQ(TIM3_IRQn);
        /* USER CODE BEGIN TIM3_MspDeInit 1 */

        /* USER CODE END TIM3_MspDeInit 1 */
    } else if (tim_baseHandle->Instance == TIM4) {
        /* USER CODE BEGIN TIM4_MspDeInit 0 */

        /* USER CODE END TIM4_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_TIM4_CLK_DISABLE();

        /* TIM4 interrupt Deinit */
        HAL_NVIC_DisableIRQ(TIM4_IRQn);
        /* USER CODE BEGIN TIM4_MspDeInit 1 */

        /* USER CODE END TIM4_MspDeInit 1 */
    } else if (tim_baseHandle->Instance == TIM5) {
        /* USER CODE BEGIN TIM5_MspDeInit 0 */

        /* USER CODE END TIM5_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_TIM5_CLK_DISABLE();

        /* TIM5 interrupt Deinit */
        HAL_NVIC_DisableIRQ(TIM5_IRQn);
        /* USER CODE BEGIN TIM5_MspDeInit 1 */

        /* USER CODE END TIM5_MspDeInit 1 */
    } else if (tim_baseHandle->Instance == TIM6) {
        /* USER CODE BEGIN TIM6_MspDeInit 0 */

        /* USER CODE END TIM6_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_TIM6_CLK_DISABLE();
        /* USER CODE BEGIN TIM6_MspDeInit 1 */

        /* USER CODE END TIM6_MspDeInit 1 */
    } else if (tim_baseHandle->Instance == TIM7) {
        /* USER CODE BEGIN TIM7_MspDeInit 0 */

        /* USER CODE END TIM7_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_TIM7_CLK_DISABLE();

        /* TIM7 interrupt Deinit */
        HAL_NVIC_DisableIRQ(TIM7_DAC_IRQn);
        /* USER CODE BEGIN TIM7_MspDeInit 1 */

        /* USER CODE END TIM7_MspDeInit 1 */
    } else if (tim_baseHandle->Instance == TIM15) {
        /* USER CODE BEGIN TIM15_MspDeInit 0 */

        /* USER CODE END TIM15_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_TIM15_CLK_DISABLE();
        /* USER CODE BEGIN TIM15_MspDeInit 1 */

        /* USER CODE END TIM15_MspDeInit 1 */
    }
}

/* USER CODE BEGIN 1 */
/**
 * @brief    定时器回调函数
 * @param  	 huart: UART句柄类型指针
 * @retval   无
 */
extern uint8_t SF6_Data[];
uint16_t Pow_Original = 0;
// uint16_t Voltage_Original = 0;
float Pow = 0;
// float Voltage = 0;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    // 1Hz频率驱动光源
    if (htim->Instance == TIM4) {
        RI_Status = ~RI_Status;
    }
    // 定时计算PID输出 频率1000HZ
    if (htim->Instance == TIM5) {
        Pow_Original = INA226_Read_Pow();
        // Voltage_Original = INA226_Read_Bus_Voltage();

        Pow = Power_Register_LSB * Pow_Original;
        // Voltage = Bus_Voltage_Register_LSB * Voltage_Original;

        if (RI_Status) {
            PID_Calc(&IR_PID, Goal_Pow, Pow);
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, IR_PID.output);
        } else {
            PID_Calc(&IR_PID, 0.0, Pow);
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, IR_PID.output);
        }
    }
    // 定时上报数据
    if (htim->Instance == TIM3) {
        HAL_UART_Transmit(&huart2, SF6_Data, 9, 0xff);
    }
    // 数据接收辅助
    if (htim->Instance == TIM7) {
        Rx_Time_Add();
        if (Rx_Time_Get() > 4 && !Serial_Serial2_RxFlag_Get()) {
            if (Serial_Serial2_RxDataNum_Get() == 9) {
                uint8_t *Sreial_Data_Addr = Serial_Serial2_RxData_Get();
                if (Sreial_Data_Addr[0] == 0xFF && Sreial_Data_Addr[1] == 0x10 && Sreial_Data_Addr[2] == 0x04 && Sreial_Data_Addr[3] == 0x00 && Sreial_Data_Addr[4] == 0x00 && Sreial_Data_Addr[5] == 0x00 && Sreial_Data_Addr[6] == 0x13 && Sreial_Data_Addr[7] == 0x88 && Sreial_Data_Addr[8] == 0x51) {
                    float SF6     = SF6_Data_Get();
                    POINT_2 pt[3] = {{SF6 - 2, 0},
                                     {SF6 - 5.0, 100.0},
                                     {SF6 - 177.0, 5000.0}};
                    double k = 0, b = 0;
                    LineInfo(pt, 3, &k, &b);
                    SF6_Coefficient_Set(k);
                    SF6_Intercept_Set(b);
                    SF6_Zero_Calibration(k, b);
                    Serial_Serial2_RxFlag_Clear();
                } else {
                    Serial_Serial2_RxFlag_Clear();
                }
            } else {
                Serial_Serial2_RxFlag_Clear();
            }
            HAL_TIM_Base_Stop_IT(&htim7);
        }
    }
}

/**
 * @brief    驱动更新状态获取
 * @param  	 无
 * @retval   状态位
 */
uint8_t RI_Status_Get(void)
{
    return RI_Status;
}
/**
 * @brief    辅助定时器时间增加 一次为1ms 4ms超时
 * @param  	 无
 * @retval   无
 */
void Rx_Time_Add(void)
{
    Rx_Time++;
}
/**
 * @brief    辅助定时器时间获取
 * @param  	 无
 * @retval   计时值
 */
uint8_t Rx_Time_Get(void)
{
    return Rx_Time;
}
/**
 * @brief    辅助定时器时间清除
 * @param  	 无
 * @retval   无
 */
void Rx_Time_Clear(void)
{
    Rx_Time = 0;
}
/* USER CODE END 1 */
