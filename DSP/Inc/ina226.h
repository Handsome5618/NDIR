#ifndef __INA226_H_
#define __INA226_H_

#include "main.h"
#include "i2c.h"

#define INA226_ADDR                 0x80   // 地址
#define Configuration_Register_Init 0x41ff // 1次平均 8.244ms 连续监测
#define Calibration_Register_Init   0x0824 // 0.5Ω 0.005mA

#define Configuration_Register      0x00 // 配置寄存器
#define Shunt_Voltage_Register      0x01 // 电压降寄存器
#define Bus_Voltage_Register        0x02 // 总线电压寄存器
#define Power_Register              0x03 // 功率寄存器
#define Current_Register            0x04 // 电流寄存器
#define Calibration_Register        0x05 // 校准寄存器

#define Shunt_Voltage_Register_LSB  2.5f
#define Bus_Voltage_Register_LSB    1.25f
#define Current_Register_LSB        0.005f
#define Power_Register_LSB          0.125f

void INA226_Init(void);
uint16_t INA226_Read_Bus_Voltage(void);
uint16_t INA226_Read_Current(void);
uint16_t INA226_Read_Pow(void);

#endif //  _INA226_H_