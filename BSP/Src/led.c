#include "led.h"

/**
 * @brief    ����ָʾ��
 * @param  	 ��
 * @retval   ��
 */
void Led_ledOn(void)
{
    HAL_GPIO_WritePin(LED_GPIO, LED_PIN, GPIO_PIN_RESET);
}

/**
 * @brief    Ϩ��ָʾ��
 * @param  	 ��
 * @retval   ��
 */
void Led_ledOff(void)
{
    HAL_GPIO_WritePin(LED_GPIO, LED_PIN, GPIO_PIN_SET);
}

/**
 * @brief    ��������ָʾ
 * @param  	 ��
 * @retval   ��
 */
void Led_LedRunTask(void)
{
    HAL_GPIO_TogglePin(LED_GPIO, LED_PIN);
}
