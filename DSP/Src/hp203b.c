#include "hp203b.h"

// HP203B_Data_t HP203B_Data;

// /**
//  * @brief    更新温度值和压力值
//  * @param  	 无
//  * @retval   无
//  */
// void HP203B_Read_Data(void)
// {
// 	uint8_t pData[6];
// 	uint32_t Temperature;
// 	uint32_t Pressure;

// 	HAL_I2C_Mem_Read(&hi2c1, HP203B_ADDR, HP203B_ADC_CVT, 8, pData, 6, 0xfff);
// 	Temperature = ((pData[0] << 16) | (pData[1] << 8) | (pData[2])) & 0xfffff;
// 	Pressure = ((pData[3] << 16) | (pData[4] << 8) | (pData[5])) & 0xfffff;

// 	Temperature  = Temperature | 0xfff000000;
// 	Pressure = Pressure | 0xfff000000;

// 	HP203B_Data.Temperature = (float)Temperature / 100.0;
// 	HP203B_Data.Pressure = (float)Pressure / 100.0;
// }

// /**
//  * @brief    HP203B初始化
//  * @param  	 无
//  * @retval   无
//  */
// void HP203B_Init(void)
// {
// 	uint8_t pData = HP203B_READ_PT;
// 	HAL_I2C_Master_Transmit(&hi2c1, HP203B_ADDR, &pData, 1, 0xfff);
// }

// /**
//  * @brief    获取温度值
//  * @param  	 无
//  * @retval   无
//  */
// float HP203B_Read_Temperature(void)
// {
// 	return HP203B_Data.Temperature;
// }

// /**
//  * @brief    获取压力值
//  * @param  	 无
//  * @retval   无
//  */
// float HP203B_Read_Pressure(void)
// {
// 	return HP203B_Data.Pressure;
// }
