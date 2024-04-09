#include "pid.h"

/**
 * @brief    ��ʼ��pid�����ĺ���
 * @param  	 ����P I D �޷�ֵ
 * @retval   ��
 */
void PID_Init(Mypid_t *pid, float p, float i, float d, float maxI, float maxOut)
{
    pid->kp          = p;
    pid->ki          = i;
    pid->kd          = d;
    pid->maxIntegral = maxI;
    pid->maxOutput   = maxOut;
}

/**
**
* @brief    ����һ��pid����
* @param  	Ŀ��ֵ ����ֵ ����������pid�ṹ���output��Ա��
* @retval   ��
*/
void PID_Calc(Mypid_t *pid, float reference, float feedback)
{
    // ��������
    pid->lastError = pid->error;           // ����error������
    pid->error     = reference - feedback; // ������error
    // ����΢��
    float dout = (pid->error - pid->lastError) * pid->kd;
    // �������
    float pout = pid->error * pid->kp;
    // �������
    pid->integral += pid->error * pid->ki;
    // �����޷�
    if (pid->integral > pid->maxIntegral)
        pid->integral = pid->maxIntegral;
    else if (pid->integral <= 0)
        pid->integral = 0;
    // �������
    pid->output = pout + dout + pid->integral;
    // ����޷�
    if (pid->output > pid->maxOutput)
        pid->output = pid->maxOutput;
    else if (pid->output <= 0)
        pid->output = 0;
}