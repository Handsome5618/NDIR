#include "pid.h"

/**
 * @brief    初始化pid参数的函数
 * @param  	 设置P I D 限幅值
 * @retval   无
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
* @brief    进行一次pid计算
* @param  	目标值 反馈值 计算结果放在pid结构体的output成员中
* @retval   无
*/
void PID_Calc(Mypid_t *pid, float reference, float feedback)
{
    // 更新数据
    pid->lastError = pid->error;           // 将旧error存起来
    pid->error     = reference - feedback; // 计算新error
    // 计算微分
    float dout = (pid->error - pid->lastError) * pid->kd;
    // 计算比例
    float pout = pid->error * pid->kp;
    // 计算积分
    pid->integral += pid->error * pid->ki;
    // 积分限幅
    if (pid->integral > pid->maxIntegral)
        pid->integral = pid->maxIntegral;
    else if (pid->integral <= 0)
        pid->integral = 0;
    // 计算输出
    pid->output = pout + dout + pid->integral;
    // 输出限幅
    if (pid->output > pid->maxOutput)
        pid->output = pid->maxOutput;
    else if (pid->output <= 0)
        pid->output = 0;
}
