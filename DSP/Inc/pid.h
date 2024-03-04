#ifndef __PID_H_
#define __PID_H_
// ���ȶ���PID�ṹ�����ڴ��һ��PID������
typedef struct
{
    float kp, ki, kd;            // ����ϵ��
    float error, lastError;      // ���ϴ����
    float integral, maxIntegral; // ���֡������޷�
    float output, maxOutput;     // ���������޷�
} Mypid_t;
void PID_Init(Mypid_t *pid, float p, float i, float d, float maxI, float maxOut);
void PID_Calc(Mypid_t *pid, float reference, float feedback);

#endif