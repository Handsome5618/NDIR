#ifndef __FILTER_H_
#define __FILTER_H_

#include "main.h"
#include "arm_math.h"
#include "arm_common_tables.h"
#include "arm_const_structs.h"

#define BLOCK_SIZE 1  /* ����һ��arm_fir_f32����Ĳ�������� */
#define NUM_TAPS   29 /* �˲���ϵ������ */

int Median_Filter(int *ADC_Data, uint16_t Size);
float Find_Max(float *arr, uint32_t size);
float Find_Mni(float *arr, uint32_t size);
float Average_Filter(float *Data, uint16_t Size);

void arm_fir_f32_lp(float32_t *InputData, float32_t *OutputData, uint32_t DataNum);
void fft(float *Input_Data, float *Output_Data);
void NDIR_Data_Processor(float *InputData, float *PeaktoPeak, float *MinitoMini);

#endif
