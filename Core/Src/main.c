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
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "pid.h"
#include "ds18b20.h"
#include "led.h"
#include "flash.h"
#include "stdio.h"
#include <inttypes.h>
#include <string.h>
#include "ina226.h"
#include "filter.h"
// #include "arm_math.h"
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
// #define IR_PID_P           35.0f //电阻
// #define IR_PID_I           1.25f
// #define IR_PID_D           0.015f

#define IR_PID_P           10.0f // 光源
#define IR_PID_I           1.60f
#define IR_PID_D           1.20f

#define IR_PID_MAXOUTPUT   5000
#define IR_PID_MAXINTEGRAL 5000

#define GOAL_POW           460.0f

#define DATA_SIZE          2048
#define PEAK_SIZE          4
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

Mypid_t IR_PID;
uint8_t ADC_Start = 0;

uint16_t ADC_Value[2]; // 双通道ADC数据

float ADC_IN1_Data[DATA_SIZE];
float ADC_IN2_Data[DATA_SIZE];

float ADC_IN1_FirData[DATA_SIZE];
float ADC_IN2_FirData[DATA_SIZE];

float PeaktoPeak1[PEAK_SIZE];
float MinitoMini1[PEAK_SIZE];

float PeaktoPeak2[PEAK_SIZE];
float MinitoMini2[PEAK_SIZE];

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
    MX_ADC1_Init();
    MX_TIM3_Init();
    MX_USART1_UART_Init();
    MX_TIM4_Init();
    MX_I2C1_Init();
    MX_TIM5_Init();
    MX_TIM2_Init();
    /* USER CODE BEGIN 2 */
    INA226_Init();

    HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_1); // 开启PWM通道1
    PID_Init(&IR_PID, IR_PID_P, IR_PID_I, IR_PID_D, IR_PID_MAXOUTPUT, IR_PID_MAXINTEGRAL);

    HAL_ADCEx_Calibration_Start(&hadc1); // 初始校准
                                         /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1) {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
        while (!RI_Status_Get() && ADC_Start == 0) {
            Led_ledOff();
            ;
        }
        ADC_Start = 1;
        while (num < DATA_SIZE && ADC_Start == 1) {
            if (ADC_Status_Get()) {
                for (uint8_t i = 0; i < 2; i++) {
                    ADC_Value[i] = ADC_Value_Get();
                    if (i == 0) {
                        ADC_IN1_Data[num] = ADC_Value[i];
                    }
                    if (i == 1) {
                        ADC_IN2_Data[num] = ADC_Value[i];
                    }
                }
                num++;
            }
            ADC_Status_Clear();
            if (num >= DATA_SIZE) {

                arm_fir_f32_lp(ADC_IN1_Data, ADC_IN1_FirData, DATA_SIZE);
                arm_fir_f32_lp(ADC_IN2_Data, ADC_IN2_FirData, DATA_SIZE);

                NDIR_Data_Processor(ADC_IN1_FirData, PeaktoPeak1, MinitoMini1);
                NDIR_Data_Processor(ADC_IN2_FirData, PeaktoPeak2, MinitoMini2);
                // for (uint8_t i = 0; i < 3; i++) {
                //     printf("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\r\n", PeaktoPeak1[i], MinitoMini1[i], PeaktoPeak2[i], MinitoMini2[i], PeaktoPeak2[i] - PeaktoPeak1[i], MinitoMini1[i] - MinitoMini2[i], (PeaktoPeak2[i] - PeaktoPeak1[i]) + (MinitoMini1[i] - MinitoMini2[i]));
                // }
                printf("%.2f\r\n", ((PeaktoPeak2[0] - PeaktoPeak1[0]) + (MinitoMini1[0] - MinitoMini2[0]) + (PeaktoPeak2[1] - PeaktoPeak1[1]) + (MinitoMini1[1] - MinitoMini2[1]) + (PeaktoPeak2[2] - PeaktoPeak1[2]) + (MinitoMini1[2] - MinitoMini2[2]) + (PeaktoPeak2[3] - PeaktoPeak1[3]) + (MinitoMini1[3] - MinitoMini2[3])) / 4);
                // for (uint16_t i = 0; i < DATA_SIZE; i++) {
                //     printf("%.2f,%.2f,%.2f,%.2f,%d\r\n", ADC_IN1_Data[i], ADC_IN2_Data[i], ADC_IN1_FirData[i], ADC_IN2_FirData[i], i);
                // }
            }
        }
        num       = 0;
        ADC_Start = 0;
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
    RCC_OscInitTypeDef RCC_OscInitStruct   = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct   = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState       = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState       = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL     = RCC_PLL_MUL9;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
        Error_Handler();
    }
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
    PeriphClkInit.AdcClockSelection    = RCC_ADCPCLK2_DIV6;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
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
