#ifndef __FALSH_H_
#define __FALSH_H_

#include "main.h"
#include "stm32f1xx_hal_flash.h"

#define TYPEPROGRAM_CHAR                 0x00U  /*!<Program a char (8-bit) at a specified address.*/

void Flash_Write(uint32_t TypeProgram, uint32_t Address, uint64_t (*Data), uint32_t len);
void Flash_Read(uint32_t TypeProgram, uint32_t Address, uint64_t (*Data), uint32_t len);

#endif
