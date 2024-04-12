/* USER CODE BEGIN Header */
/*
                       _oo0oo_
                      o8888888o
                      88" . "88
                      (| -_- |)
                      0\  =  /0
                    ___/`---'\___
                  .' \\|     |// '.
                 / \\|||  :  |||// \
                / _||||| -:- |||||- \
               |   | \\\  -  /// |   |
               | \_|  ''\---/''  |_/ |
               \  .-\__  '-'  ___/-. /
             ___'. .'  /--.--\  `. .'___
          ."" '<  `.___\_<|>_/___.' >' "".
         | | :  `- \`.;`\ _ /`;.`/ - ` : | |
         \  \ `_.   \_ __\ /__ _/   .-` /  /
     =====`-.____`.___ \_____/___.-`___.-'=====
                       `=---='
     ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "pid.h"
#include "led.h"
#include "ds18b20.h"
#include "stdio.h"
#include <inttypes.h>
#include <string.h>
#include "ina226.h"
#include "filter.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
// #define IR_PID_P 10.0f // 光源
// #define IR_PID_I 1.60f
// #define IR_PID_D 1.20f

// #define IR_PID_P           6.0f // 光源
// #define IR_PID_I           0.55f
// #define IR_PID_D           0.2f

#define IR_PID_P           6.0f // 光源
#define IR_PID_I           0.7f
#define IR_PID_D           0.05f

#define IR_PID_MAXOUTPUT   7000
#define IR_PID_MAXINTEGRAL 7000

#define GOAL_POW           460.0f

#define DATA_SIZE          5500
#define PEAK_SIZE          4
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
float SF6_PPM;
uint32_t ADC2_Value = 0;
float Temperature   = 0;

extern float Goal_Pow;

Mypid_t IR_PID;

uint16_t ADC1_Value[DATA_SIZE][2]; // 双通道ADC数据

float ADC_IN1_Data[DATA_SIZE]; // ADC1通道1数据
float ADC_IN2_Data[DATA_SIZE]; // ADC1通道2数据

float ADC_IN1_FirData[DATA_SIZE]; // ADC1通道1滤波数据
float ADC_IN2_FirData[DATA_SIZE]; // ADC1通道2滤波数据

float PeaktoPeak1[PEAK_SIZE]; // ADC1通道1峰峰值
float MinitoMini1[PEAK_SIZE]; // ADC1通道1谷谷值

float PeaktoPeak2[PEAK_SIZE]; // ADC1通道2峰峰值
float MinitoMini2[PEAK_SIZE]; // ADC1通道2谷谷值

uint32_t num;

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
    MX_DMA_Init();
    MX_ADC1_Init();
    MX_ADC2_Init();
    MX_I2C2_Init();
    MX_USART2_UART_Init();
    MX_TIM2_Init();
    MX_TIM4_Init();
    MX_TIM5_Init();
    MX_TIM6_Init();
    MX_TIM15_Init();
    /* USER CODE BEGIN 2 */
    INA226_Init();
    Led_ledOff();
    HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_4); // 开启PWM通道1
    PID_Init(&IR_PID, IR_PID_P, IR_PID_I, IR_PID_D, IR_PID_MAXOUTPUT, IR_PID_MAXINTEGRAL);

    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED); // 初始校准
    HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED); // 初始校准

    HAL_ADC_Start_DMA(&hadc2, (uint32_t *)&ADC2_Value, 1);

    HAL_Delay(10000);

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1) {

        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
        while (!RI_Status_Get()) {
            Led_ledOff();
        }

        HAL_ADC_Start_DMA(&hadc1, (uint32_t *)ADC1_Value, 2 * DATA_SIZE);
        while (ADC1_Value[DATA_SIZE - 1][1] == 0) {
            Led_ledOff();
        }
        HAL_ADC_Stop_DMA(&hadc1);
        for (uint32_t i = 0; i < DATA_SIZE; i++) {
            ADC_IN1_Data[i]  = ADC1_Value[i][0];
            ADC_IN2_Data[i]  = ADC1_Value[i][1];
            ADC1_Value[i][0] = 0;
            ADC1_Value[i][1] = 0;
        }
        arm_fir_f32_lp(ADC_IN1_Data, ADC_IN1_FirData, DATA_SIZE);
        arm_fir_f32_lp(ADC_IN2_Data, ADC_IN2_FirData, DATA_SIZE);
        NDIR_Data_Processor(ADC_IN1_FirData, PeaktoPeak1, MinitoMini1);
        NDIR_Data_Processor(ADC_IN2_FirData, PeaktoPeak2, MinitoMini2);
        SF6_PPM     = ((PeaktoPeak2[0] - PeaktoPeak1[0]) + (MinitoMini1[0] - MinitoMini2[0]) + (PeaktoPeak2[1] - PeaktoPeak1[1]) + (MinitoMini1[1] - MinitoMini2[1]) + (PeaktoPeak2[2] - PeaktoPeak1[2]) + (MinitoMini1[2] - MinitoMini2[2]) + (PeaktoPeak2[3] - PeaktoPeak1[3]) + (MinitoMini1[3] - MinitoMini2[3])) / 4;
        Temperature = (float)(ADC2_Value * 330) / 4095;

        // printf("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\r\n", MinitoMini2[0], MinitoMini2[1], MinitoMini2[2], MinitoMini2[3], PeaktoPeak2[0], PeaktoPeak2[1], PeaktoPeak2[2], PeaktoPeak2[3]);

        // 自适应功率调节
        if (((MinitoMini2[0] + MinitoMini2[1] + MinitoMini2[2] + MinitoMini2[3]) / 4) < 589) {
            Goal_Pow = Goal_Pow - 0.2;
            if (Goal_Pow >= 540) { // 功率限幅
                Goal_Pow = 540;
            }
            HAL_Delay(1000);
        } else if ((((MinitoMini2[0] + MinitoMini2[1] + MinitoMini2[2] + MinitoMini2[3]) / 4) > 591)) {
            Goal_Pow = Goal_Pow + 0.2;
            if (Goal_Pow <= 450) { // 功率限幅
                Goal_Pow = 450;
            }

            HAL_Delay(1000);
        }

        printf("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\r\n", SF6_PPM, Temperature, Goal_Pow, ((MinitoMini2[0] + MinitoMini2[1] + MinitoMini2[2] + MinitoMini2[3]) / 4), ((PeaktoPeak2[0] + PeaktoPeak2[1] + PeaktoPeak2[2] + PeaktoPeak2[3]) / 4), ((-2.7902 * Temperature) + (0.63794 * SF6_PPM) - 561.99532));

        while (RI_Status_Get()) {
            Led_ledOff();
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
    RCC_OscInitStruct.HSEState       = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM       = RCC_PLLM_DIV2;
    RCC_OscInitStruct.PLL.PLLN       = 75;
    RCC_OscInitStruct.PLL.PLLP       = RCC_PLLP_DIV4;
    RCC_OscInitStruct.PLL.PLLQ       = RCC_PLLQ_DIV2;
    RCC_OscInitStruct.PLL.PLLR       = RCC_PLLR_DIV2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
        Error_Handler();
    }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1) {
    }
    /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
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
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
