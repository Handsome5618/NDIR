#include "ads1118.h"

// void MySPI_W_SS(uint8_t BitValue)
// {
//     HAL_GPIO_WritePin(ADC_CS_GPIO_PORT, ADC_CS_PIN, BitValue);
// }

// void MySPI_W_MOSI(uint8_t BitValue)
// {
//     HAL_GPIO_WritePin(ADC_MOSI_GPIO_Port, ADC_CLK_Pin, BitValue);
// }

// void MySPI_W_SCK(uint8_t BitValue)
// {
//     HAL_GPIO_WritePin(ADC_CLK_GPIO_Port, ADC_CLK_Pin, BitValue);
// }

// uint8_t MySPI_R_MISO(void)
// {
//     return HAL_GPIO_ReadPin(ADC_MISO_GPIO_Port, ADC_MISO_Pin);
// }

// void MySPI_Start(void)
// {
//     MySPI_W_SS(0);
// }

// void MySPI_Stop(void)
// {
//     MySPI_W_SS(1);
// }

// uint8_t MySPI_SwapByte(uint8_t ByteSend)
// {
//     uint8_t i, ByteReceive = 0x00;

//     for (i = 0; i < 8; i++) {
//         MySPI_W_MOSI(ByteSend & (0x80 >> i));
//         MySPI_W_SCK(1);
//         if (MySPI_R_MISO() == 1) { ByteReceive |= (0x80 >> i); }
//         MySPI_W_SCK(0);
//     }
//     return ByteReceive;
// }

// /**
//  * @brief    ADS1118初始化
//  * @param  	 无
//  * @retval   无
//  */
// void ADS1118_Init(void)
// {
//     uint16_t REG_Init = CH0 | V6_144 | MODE_SINGLE | RATE_860SPS | GET_ADC | PULL_UP_EN | VALID_SET;
//     uint8_t pTxData[4];
//     uint8_t pRxData[4];
//     pTxData[0] = (uint8_t)(REG_Init >> 8);
//     pTxData[1] = (uint8_t)REG_Init;
//     pTxData[2] = (uint8_t)(REG_Init >> 8);
//     pTxData[3] = (uint8_t)REG_Init;

//     MySPI_W_SS(0);
//     while (HAL_GPIO_ReadPin(ADC_MISO_GPIO_Port, ADC_MISO_Pin) != GPIO_PIN_RESET) {
//         printf("Error\r\n");
//     }

//     for (uint8_t i = 0; i < 4; i++) {
//         pRxData[i] = MySPI_SwapByte(pTxData[i]);
//     }
//     MySPI_W_SS(1);
// }

// /**
//  * @brief    ADS1118初始化
//  * @param  	 无
//  * @retval   无
//  */
// void ADS1118_Init(void)
// {
//     uint16_t REG_Init = CH0 | V6_144 | MODE_SINGLE | RATE_860SPS | GET_ADC | PULL_UP_EN | VALID_SET;
//     uint8_t pData[4];
//     pData[0] = (uint8_t)(REG_Init >> 8);
//     pData[1] = (uint8_t)REG_Init;
//     pData[2] = (uint8_t)(REG_Init >> 8);
//     pData[3] = (uint8_t)REG_Init;

//     HAL_GPIO_WritePin(ADS1118_CS_GPIO, ADS1118_CS_PIN, GPIO_PIN_RESET);

//     HAL_Delay(10);

//     HAL_SPI_Transmit(&hspi1, pData, 4, 0xffffffff);

//     HAL_Delay(10);

//     HAL_GPIO_WritePin(ADS1118_CS_GPIO, ADS1118_CS_PIN, GPIO_PIN_SET);
// }

// /**
//  * @brief    ADS1118读取指定次数ADC值
//  * @param  	 读取的通道
//  * @param  	 存放数据的数组
//  * @param  	 要读取的个数
//  * @retval   无
//  */
// void ADS1118_Read_ADC(uint16_t Channel, float *pData, uint16_t Size)
// {
//     uint8_t pTxData[2];
//     uint8_t pRxData[2];
//     uint16_t REG_Init = Channel | V6_144 | MODE_SINGLE | RATE_860SPS | GET_ADC | PULL_UP_EN | INVALID_SET;
//     pTxData[0]        = (uint8_t)(REG_Init >> 8);
//     pTxData[1]        = (uint8_t)REG_Init;

//     for (uint16_t i = 0; i < Size; i++) {
//         HAL_Delay(100);
//         HAL_GPIO_WritePin(ADS1118_CS_GPIO, ADS1118_CS_PIN, GPIO_PIN_RESET);

//         HAL_Delay(10);
//         // while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) != GPIO_PIN_RESET) {
//         //     ;
//         // }

//         HAL_SPI_TransmitReceive(&hspi1, pTxData, pRxData, 2, 0x00ffffff);

//         HAL_Delay(10);

//         HAL_GPIO_WritePin(ADS1118_CS_GPIO, ADS1118_CS_PIN, GPIO_PIN_SET);

//         pData[i] = ((pRxData[0] << 8) | (pRxData[1])) * 187.5;
//     }
// }

// #define SET_SS          0x8000 // 单次采集模式开始采集

// #define CH0             0x4000 // 单端模式采集通道
// #define CH1             0x5000
// #define CH2             0x6000
// #define CH3             0x7000

// #define V6_144          0x0000 // 正负量程
// #define V4_096          0x0200
// #define V2_048          0x0400
// #define V1_024          0x0600
// #define V0_512          0x0800
// #define V0_256          0x0A00

// #define MODE_SINGLE     0x0100 // 单次采集模式
// #define MODE_CONTINUE   0x0000 // 连续采集模式

// #define RATE_8SPS       0x0000 // 采集速率
// #define RATE_16SPS      0x0020
// #define RATE_32SPS      0x0040
// #define RATE_64SPS      0x0060
// #define RATE_128SPS     0x0080
// #define RATE_250SPS     0x00A0
// #define RATE_475SPS     0x00C0
// #define RATE_860SPS     0x00E0

// #define TEMPERATURE     0x0010 // 采集温度
// #define GET_ADC         0x0000 // 采集外部电压

// #define PULL_UP_EN      0x0008 // 内部上拉DOUT

// #define VALID_SET       0x0002 // 此次设置生效
// #define INVALID_SET     0x0006 // 此次设置无效

// #define POWERON_DEFAULT 0x058A // 芯片开机后的默认初始值

// char ADS1118_ReadWrite(unsigned short data, short *val);

// char ADS1118_Init(void)
// {
//     unsigned short REG_Init = CH0 | V6_144 | MODE_SINGLE | RATE_64SPS | TEMPERATURE | PULL_UP_EN | VALID_SET;
//     short val;
//     char ret = ADS1118_ReadWrite(REG_Init, &val);
//     if (ret == 0)
//         return 0;
//     else
//         return -1;
// }

// char ADS1118_Get_CH0(float *Value)
// {
//     short val;
//     unsigned short REG_CH0_RUN = SET_SS | CH0 | V6_144 | MODE_SINGLE | RATE_64SPS | GET_ADC | PULL_UP_EN | VALID_SET; // 设置转换需要的寄存器值
//     char ret                   = ADS1118_ReadWrite(REG_CH0_RUN, &val);                                                // 写入寄存器值随即开始转换
//     HAL_Delay(20);                                                                                                    // 等待转换完成
//     ret += ADS1118_ReadWrite(REG_CH0_RUN, &val);                                                                      // 读取转换的数据
//     *Value = val * 187.5 / 1000000;
//     return ret;
// }

// char ADS1118_Get_CH1(float *Value)
// {
//     short val;
//     unsigned short REG_CH1_RUN = SET_SS | CH1 | V6_144 | MODE_SINGLE | RATE_64SPS | GET_ADC | PULL_UP_EN | VALID_SET;
//     char ret                   = ADS1118_ReadWrite(REG_CH1_RUN, &val);
//     HAL_Delay(20);
//     ret += ADS1118_ReadWrite(REG_CH1_RUN, &val);
//     *Value = val * 187.5 / 1000000;
//     return ret;
// }

// char ADS1118_Get_CH2(float *Value)
// {
//     short val;
//     unsigned short REG_CH2_RUN = SET_SS | CH2 | V6_144 | MODE_SINGLE | RATE_64SPS | GET_ADC | PULL_UP_EN | VALID_SET;
//     char ret                   = ADS1118_ReadWrite(REG_CH2_RUN, &val);
//     HAL_Delay(20);
//     ret += ADS1118_ReadWrite(REG_CH2_RUN, &val);
//     *Value = val * 187.5 / 1000000;
//     return ret;
// }

// char ADS1118_Get_CH3(float *Value)
// {
//     short val;
//     unsigned short REG_CH3_RUN = SET_SS | CH3 | V6_144 | MODE_SINGLE | RATE_64SPS | GET_ADC | PULL_UP_EN | VALID_SET;
//     char ret                   = ADS1118_ReadWrite(REG_CH3_RUN, &val);
//     HAL_Delay(20);
//     ret += ADS1118_ReadWrite(REG_CH3_RUN, &val);
//     *Value = val * 187.5 / 1000000;

//     return ret;
// }

// char ADS1118_Get_Temperature(float *Temperature)
// {
//     short val;
//     unsigned short REG_TEMP_RUN = SET_SS | V6_144 | MODE_SINGLE | RATE_64SPS | TEMPERATURE | PULL_UP_EN | VALID_SET;
//     char ret                    = ADS1118_ReadWrite(REG_TEMP_RUN, &val);
//     HAL_Delay(100);
//     ret += ADS1118_ReadWrite(REG_TEMP_RUN, &val);
//     val = ((unsigned short)val) >> 2;                         // 温度数据高十四位有效
//     if (val & 0x2000) {                                       // 温度数据第十四位表明是否为负数
//         unsigned short temp = ((unsigned short)val) ^ 0x2000; // 第十四位清零
//         temp                = temp - 1;                       // 转换为反码
//         temp                = (~temp) & 0x1FFF;               // 低十三位按位取反得原码
//         val                 = -temp;                          // 重新取负转换为十六位的补码格式
//     }
//     *Temperature = val * 0.03125f; // 乘以温度比例系数
//     return ret;
// }

// char ADS1118_ReadWrite(unsigned short data, short *val)
// {
//     unsigned int Din;
//     unsigned short temp = data;
//     HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
//     for (unsigned char i = 0; i < 32; i++) {
//         Din = Din << 1;
//         if (0x8000 & temp) { // 前十六位写入寄存器值 后十六位写零
//             HAL_GPIO_WritePin(ADC_IN_GPIO_Port, ADC_IN_Pin, 1);
//             temp = (temp << 1);
//         } else {
//             HAL_GPIO_WritePin(ADC_IN_GPIO_Port, ADC_IN_Pin, 0);
//             temp = (temp << 1);
//         }
//         // HAL_Delay(1);
//         HAL_GPIO_WritePin(ADC_CLK_GPIO_Port, ADC_CLK_Pin, 1); // 拉高CLK
//         // HAL_Delay(1);
//         if (HAL_GPIO_ReadPin(ADC_OUT_GPIO_Port, ADC_OUT_Pin)) { // 读取数据
//             Din |= 0x01;
//         }
//         // HAL_Delay(1);
//         HAL_GPIO_WritePin(ADC_CLK_GPIO_Port, ADC_CLK_Pin, 0); // 拉低CLK
//         // HAL_Delay(1);
//     }
//     HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
//     *val                  = (short)((unsigned int)(Din & 0xffff0000) >> 16); // 读取到的高十六位为ADC
//     unsigned short Config = Din & 0xffff;                                    // 后16位为刚才写入的寄存器值
//     if ((data | 0x01) == Config) {                                           // 设置寄存器最低位读取出来始终为1
//         return 0;
//     } else {
//         return -1;
//     }
// }
