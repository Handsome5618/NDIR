#include "filter.h"

// 滤波器系数 99阶FIR滤波器 截止频率5Hz
float firCoeffs32LP[NUM_TAPS] = {
    0.001185529982, 0.001213171519, 0.001268957742, 0.001353632077, 0.001467787311,
    0.00161186012, 0.001786126639, 0.001990698045, 0.002225517761, 0.00249036029,
    0.002784828888, 0.003108356846, 0.003460207488, 0.003839477198, 0.004245098215,
    0.004675841425, 0.005130324047, 0.005607013125, 0.006104233209, 0.00662017474,
    0.007152902894, 0.007700365037, 0.008260402828, 0.008830764331, 0.009409110993,
    0.009993034415, 0.01058006752, 0.01116769388, 0.01175336912, 0.01233452652,
    0.01290859375, 0.01347300876, 0.01402523275, 0.01456275955, 0.01508313697,
    0.01558397431, 0.01606296003, 0.01651786827, 0.0169465784, 0.01734708436,
    0.01771750301, 0.01805608533, 0.01836123131, 0.01863149554, 0.01886559092,
    0.01906240359, 0.01922099479, 0.01934060641, 0.01942066662, 0.01946079358,
    0.01946079358, 0.01942066662, 0.01934060641, 0.01922099479, 0.01906240359,
    0.01886559092, 0.01863149554, 0.01836123131, 0.01805608533, 0.01771750301,
    0.01734708436, 0.0169465784, 0.01651786827, 0.01606296003, 0.01558397431,
    0.01508313697, 0.01456275955, 0.01402523275, 0.01347300876, 0.01290859375,
    0.01233452652, 0.01175336912, 0.01116769388, 0.01058006752, 0.009993034415,
    0.009409110993, 0.008830764331, 0.008260402828, 0.007700365037, 0.007152902894,
    0.00662017474, 0.006104233209, 0.005607013125, 0.005130324047, 0.004675841425,
    0.004245098215, 0.003839477198, 0.003460207488, 0.003108356846, 0.002784828888,
    0.00249036029, 0.002225517761, 0.001990698045, 0.001786126639, 0.00161186012,
    0.001467787311, 0.001353632077, 0.001268957742, 0.001213171519, 0.001185529982};

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
float Average_Filter(uint16_t *Data, uint16_t Size)
{
    float sum = 0;
    float average;

    for (int i = 0; i < Size; i++) {
        sum += Data[i];
    }

    average = sum / Size;

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
    temp = InputData[320];
    for (int i = 320; i < 880; i++) {
        if (InputData[i] > temp) {
            temp = InputData[i];
        }
    }
    PeaktoPeak[0] = temp;

    temp = InputData[881];
    for (int i = 881; i < 1600; i++) {
        if (InputData[i] < temp) {
            temp = InputData[i];
        }
    }
    MinitoMini[0] = temp;

    // temp = InputData[1601];
    // for (int i = 1601; i < 2300; i++) {
    //     if (InputData[i] > temp) {
    //         temp = InputData[i];
    //     }
    // }
    // PeaktoPeak[1] = temp;

    // temp = InputData[2301];
    // for (int i = 2301; i < 2750; i++) {
    //     if (InputData[i] < temp) {
    //         temp = InputData[i];
    //     }
    // }
    // MinitoMini[1] = temp;

    // temp = InputData[2751];
    // for (int i = 2751; i < 3500; i++) {
    //     if (InputData[i] > temp) {
    //         temp = InputData[i];
    //     }
    // }
    // PeaktoPeak[2] = temp;

    // temp = InputData[3501];
    // for (int i = 3501; i < 4100; i++) {
    //     if (InputData[i] < temp) {
    //         temp = InputData[i];
    //     }
    // }
    // MinitoMini[2] = temp;

    // temp = InputData[4101];
    // for (int i = 4101; i < 4800; i++) {
    //     if (InputData[i] > temp) {
    //         temp = InputData[i];
    //     }
    // }
    // PeaktoPeak[3] = temp;

    // temp = InputData[4801];
    // for (int i = 4801; i < 5100; i++) {
    //     if (InputData[i] < temp) {
    //         temp = InputData[i];
    //     }
    // }
    // MinitoMini[3] = temp;
}
