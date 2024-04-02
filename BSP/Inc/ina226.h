#ifndef __INA226_H_
#define __INA226_H_

#include "main.h"
#include "i2c.h"

#define I2C_HANDLE                  hi2c2

#define INA226_ADDR                 0x80
#define Configuration_Register_Init 0x4127
#define Calibration_Register_Init   0x0800

#define Configuration_Register      0x00
#define Shunt_Voltage_Register      0x01
#define Bus_Voltage_Register        0x02
#define Power_Register              0x03
#define Current_Register            0x04
#define Calibration_Register        0x05

#define Shunt_Voltage_Register_LSB  2.5f
#define Bus_Voltage_Register_LSB    1.25f
#define Current_Register_LSB        0.005f
#define Power_Register_LSB          0.125f

void INA226_Init(void);
uint16_t INA226_Read_Bus_Voltage(void);
uint16_t INA226_Read_Current(void);
uint16_t INA226_Read_Pow(void);

#endif //  _INA226_H_