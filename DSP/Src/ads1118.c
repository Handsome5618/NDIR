#include "ads1118.h"

/**	
 * @brief    ADS1118初始化
 * @param  	 无
 * @retval   无
 */
void ADS11181_Init(void)
{
	uint16_t REG_Init = CH0|V6_144|MODE_SINGLE|RATE_860SPS|GET_ADC|PULL_UP_EN|VALID_SET;
	uint8_t pData[4];
	pData[0] = (uint8_t)(REG_Init >> 8);
	pData[1] = (uint8_t)REG_Init;
	pData[2] = (uint8_t)(REG_Init >> 8);
	pData[3] = (uint8_t)REG_Init;
	
	HAL_GPIO_WritePin(ADS1118_CS_GPIO, ADS1118_CS_PIN, GPIO_PIN_RESET);
	
	HAL_SPI_Transmit(&hspi1, pData, 4, 0xffffffff);
	
	HAL_GPIO_WritePin(ADS1118_CS_GPIO, ADS1118_CS_PIN, GPIO_PIN_SET);
}	

/**	
 * @brief    ADS1118读取指定次数ADC值
 * @param  	 读取的通道
 * @param  	 存放数据的数组
 * @param  	 要读取的个数
 * @retval   无
 */
void ADS1118_Read_ADC(uint8_t Channel, float *pData, uint16_t Size)
{
	uint8_t pTxData[4];
	uint8_t pRxData[4];
	uint16_t REG_Init = Channel|V6_144|MODE_SINGLE|RATE_860SPS|GET_ADC|PULL_UP_EN|VALID_SET;
	pTxData[0] = (uint8_t)(REG_Init >> 8);
	pTxData[1] = (uint8_t)REG_Init;
	pTxData[2] = (uint8_t)(REG_Init >> 8);
	pTxData[3] = (uint8_t)REG_Init;
	
	for(uint16_t i = 0; i < Size; i++)
	{
		HAL_GPIO_WritePin(ADS1118_CS_GPIO, ADS1118_CS_PIN, GPIO_PIN_RESET);

		while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) != GPIO_PIN_RESET)
		{
			;
		}

		HAL_SPI_TransmitReceive(&hspi1, pTxData, pRxData,4, 0x00ffffff);

		HAL_GPIO_WritePin(ADS1118_CS_GPIO, ADS1118_CS_PIN, GPIO_PIN_SET);

		pData[i] = ((pRxData[0] << 8) | (pRxData[1])) *187.5;
	}
}

