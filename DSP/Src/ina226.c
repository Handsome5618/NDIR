#include "ina226.h"

/**
 * @brief    INA226初始化
 * @param  	 无
 * @retval   无
 */
void INA226_Init(void)
{
    uint8_t tData[3];
    tData[0] = Configuration_Register;
    tData[1] = Configuration_Register_Init >> 8;
    tData[2] = (uint8_t)Configuration_Register_Init;
    tData[2] = HAL_I2C_Master_Transmit(&hi2c1, INA226_ADDR, tData, 3, 0xff);
    HAL_Delay(5);
    tData[0] = Calibration_Register;
    tData[1] = Calibration_Register_Init >> 8;
    tData[2] = (uint8_t)Calibration_Register_Init;
    tData[2] = HAL_I2C_Master_Transmit(&hi2c1, INA226_ADDR, tData, 3, 0xff);
    HAL_Delay(5);
}

/**
 * @brief    INA226读取总线电压值
 * @param  	 无
 * @retval   总线电压值
 */
uint16_t INA226_Read_Bus_Voltage(void)
{
    uint16_t Bus_Voltage;
    uint8_t rData[2];
    uint8_t tData[1] = {Bus_Voltage_Register};
    HAL_I2C_Master_Transmit(&hi2c1, INA226_ADDR, tData, 1, 0xff);
    HAL_I2C_Master_Receive(&hi2c1, INA226_ADDR, rData, 2, 0xff);
    Bus_Voltage = rData[0] << 8 | rData[1];
    return Bus_Voltage;
}

/**
 * @brief    INA226读取电流
 * @param  	 无
 * @retval   总线电流值
 */
uint16_t INA226_Read_Current(void)
{
    uint16_t Current;
    uint8_t rData[2];
    uint8_t tData[1] = {Current_Register};
    HAL_I2C_Master_Transmit(&hi2c1, INA226_ADDR, tData, 1, 0xff);
    HAL_I2C_Master_Receive(&hi2c1, INA226_ADDR, rData, 2, 0xff);
    Current = rData[0] << 8 | rData[1];
    return Current;
}

/**
 * @brief    INA226读取功率
 * @param  	 无
 * @retval   功率值
 */
uint16_t INA226_Read_Pow(void)
{
    uint16_t Pow;
    uint8_t rData[2];
    uint8_t tData[1] = {Power_Register};
    HAL_I2C_Master_Transmit(&hi2c1, INA226_ADDR, tData, 1, 0xff);
    HAL_I2C_Master_Receive(&hi2c1, INA226_ADDR, rData, 2, 0xff);
    Pow = rData[0] << 8 | rData[1];
    return Pow;
}
