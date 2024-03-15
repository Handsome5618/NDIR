#include "filter.h"

// 滤波器系数 28阶FIR滤波器
float firCoeffs32LP[NUM_TAPS] = {
    0.004708919674, 0.005489378236, 0.00766007416, 0.01117761619, 0.01591668092,
    0.02167543769, 0.02818588167, 0.03512848914, 0.0421503298, 0.04888555408,
    0.05497694388, 0.06009719148, 0.06396857649, 0.06637971848, 0.06719842553,
    0.06637971848, 0.06396857649, 0.06009719148, 0.05497694388, 0.04888555408,
    0.0421503298, 0.03512848914, 0.02818588167, 0.02167543769, 0.01591668092,
    0.01117761619, 0.00766007416, 0.005489378236, 0.004708919674};

uint32_t blockSize = BLOCK_SIZE;

float firStateF32[BLOCK_SIZE + NUM_TAPS - 1]; /* 状态缓存，大小numTaps + blockSize - 1*/

/**
 * @brief   FIR滤波器
 * @param   采样数组
 * @param   输出数组
 * @param   采样点数
 * @retval  无
 */
void arm_fir_f32_lp(float32_t *InputData, float32_t *OutputData, uint32_t DataNum)
{
    uint32_t i;
    arm_fir_instance_f32 S;

    /* 初始化结构体S */
    arm_fir_init_f32(&S,
                     NUM_TAPS,
                     (float32_t *)&firCoeffs32LP[0],
                     &firStateF32[0],
                     blockSize);

    /* 实现FIR滤波，这里每次处理1个点 */
    for (i = 0; i < BLOCK_SIZE * DataNum; i++) {
        arm_fir_f32(&S, InputData + (i * blockSize), OutputData + (i * blockSize), blockSize);
    }
}

/**
 * @brief   计算指定周期峰峰值
 * @param   采样数组
 * @param   采样点数
 * @param   采样周期
 * @param   峰峰值存放数组
 * @retval  无
 */
#include <math.h>

/**
 * @brief   中值滤波器
 * @param   采样数组
 * @param   采样点数
 * @retval  中值
 */
int Median_Filter(int *ADC_Data, uint16_t Size)
{

    // 对采样数据进行排序
    for (int i = 0; i < Size - 1; i++) {
        for (int j = 0; j < Size - 1 - i; j++) {
            if (ADC_Data[j] > ADC_Data[j + 1]) {
                int temp        = ADC_Data[j];
                ADC_Data[j]     = ADC_Data[j + 1];
                ADC_Data[j + 1] = temp;
            }
        }
    }

    // 获取排序后的中间值
    int mid_value = ADC_Data[(Size / 2)];

    return (int)mid_value;
}

/**
 * @brief   均值滤波器
 * @param   采样数组
 * @param   采样点数
 * @retval  均值
 */
float Average_Filter(float *Data, uint16_t Size)
{
    float sum = 0;
    float average;

    for (int i = 1; i < Size - 1; i++) {
        sum += Data[i];
    }

    average = sum / Size - 2;

    return average;
}

/**
 * @brief   峰值输出
 * @param   采样数组
 * @param   采样点数
 * @retval  峰值
 */
float Find_Max(float *arr, uint32_t size)
{
    float max = arr[30];

    for (int i = 1; i < size; i++) {
        if (arr[i] > max) {
            max = arr[i];
        }
    }
    return max;
}

/**
 * @brief   谷值输出
 * @param   采样数组
 * @param   采样点数
 * @retval  谷值
 */
float Find_Mni(float *arr, uint32_t size)
{
    float min = arr[30];

    for (int i = 1; i < size; i++) {
        if (arr[i] < min) {
            min = arr[i];
        }
    }
    return min;
}

#define FFT_SIZE    1024
#define SAMPLE_RATE 500

uint32_t fftSize      = FFT_SIZE;
uint32_t ifftFlag     = 0;
uint32_t doBitReverse = 1;

void fft(float *Input_Data, float *Output_Data)
{
    arm_cfft_radix4_instance_f32 fftInstance;
    // // 初始化FFT
    arm_cfft_radix4_init_f32(&fftInstance, FFT_SIZE, ifftFlag, doBitReverse);
    // // 执行FFT
    arm_cfft_radix4_f32(&fftInstance, Input_Data);
    // 计算幅值
    arm_cmplx_mag_f32(Input_Data, Output_Data, FFT_SIZE);
}

/**
 * @brief   NDIR定制信号处理
 * @param   采样数组
 * @param   峰峰值存放数组
 * @param   谷谷值存放数组
 * @retval  无
 */
void NDIR_Data_Processor(float *InputData, float *PeaktoPeak, float *MinitoMini)
{
    float temp;
    temp = InputData[100];
    for (int i = 100; i < 340; i++) {
        if (InputData[i] > temp) {
            temp = InputData[i];
        }
    }
    PeaktoPeak[0] = temp;

    temp = InputData[341];
    for (int i = 341; i < 560; i++) {
        if (InputData[i] < temp) {
            temp = InputData[i];
        }
    }
    MinitoMini[0] = temp;

    temp = InputData[561];
    for (int i = 561; i < 770; i++) {
        if (InputData[i] > temp) {
            temp = InputData[i];
        }
    }
    PeaktoPeak[1] = temp;

    temp = InputData[771];
    for (int i = 771; i < 1000; i++) {
        if (InputData[i] < temp) {
            temp = InputData[i];
        }
    }
    MinitoMini[1] = temp;

    temp = InputData[1001];
    for (int i = 1001; i < 1210; i++) {
        if (InputData[i] > temp) {
            temp = InputData[i];
        }
    }
    PeaktoPeak[2] = temp;

    temp = InputData[1211];
    for (int i = 1211; i < 1440; i++) {
        if (InputData[i] < temp) {
            temp = InputData[i];
        }
    }
    MinitoMini[2] = temp;

    temp = InputData[1441];
    for (int i = 1441; i < 1680; i++) {
        if (InputData[i] > temp) {
            temp = InputData[i];
        }
    }
    PeaktoPeak[3] = temp;

    temp = InputData[1681];
    for (int i = 1681; i < 1900; i++) {
        if (InputData[i] < temp) {
            temp = InputData[i];
        }
    }
    MinitoMini[3] = temp;

    // temp = InputData[100];
    // for (int i = 100; i < 210; i++) {
    //     if (InputData[i] > temp) {
    //         temp = InputData[i];
    //     }
    // }
    // PeaktoPeak[0] = temp;

    // temp = InputData[211];
    // for (int i = 211; i < 320; i++) {
    //     if (InputData[i] < temp) {
    //         temp = InputData[i];
    //     }
    // }
    // MinitoMini[0] = temp;

    // temp = InputData[321];
    // for (int i = 321; i < 440; i++) {
    //     if (InputData[i] > temp) {
    //         temp = InputData[i];
    //     }
    // }
    // PeaktoPeak[1] = temp;

    // temp = InputData[441];
    // for (int i = 441; i < 560; i++) {
    //     if (InputData[i] < temp) {
    //         temp = InputData[i];
    //     }
    // }
    // MinitoMini[1] = temp;

    // temp = InputData[561];
    // for (int i = 561; i < 680; i++) {
    //     if (InputData[i] > temp) {
    //         temp = InputData[i];
    //     }
    // }
    // PeaktoPeak[2] = temp;

    // temp = InputData[681];
    // for (int i = 681; i < 800; i++) {
    //     if (InputData[i] < temp) {
    //         temp = InputData[i];
    //     }
    // }
    // MinitoMini[2] = temp;

    // temp = InputData[561];
    // for (int i = 561; i < 680; i++) {
    //     if (InputData[i] > temp) {
    //         temp = InputData[i];
    //     }
    // }
}
