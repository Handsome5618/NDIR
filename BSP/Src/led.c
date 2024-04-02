#include "led.h"

/**
 * @brief    点亮指示灯
 * @param  	 无
 * @retval   无
 */
void Led_ledOn(void)
{
    HAL_GPIO_WritePin(LED_GPIO, LED_PIN, GPIO_PIN_RESET);
}

/**
 * @brief    熄灭指示灯
 * @param  	 无
 * @retval   无
 */
void Led_ledOff(void)
{
    HAL_GPIO_WritePin(LED_GPIO, LED_PIN, GPIO_PIN_SET);
}

/**
 * @brief    程序运行指示
 * @param  	 无
 * @retval   无
 */
void Led_LedRunTask(void)
{
    HAL_GPIO_TogglePin(LED_GPIO, LED_PIN);
}
