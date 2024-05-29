#include "flash.h"

/**
 * @brief    Flash擦除
 * @param  	 擦除的页码 每次擦除1页
 * @retval   无
 */
void Flash_Data_Rrase(uint32_t Addr)
{
    FLASH_EraseInitTypeDef EraseInitStruct;

    uint32_t PageError        = 0;
    EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
    EraseInitStruct.Page      = GetPage(Addr);
    EraseInitStruct.NbPages   = 1;
    EraseInitStruct.Banks     = FLASH_BANK_BOTH;

    HAL_FLASH_Unlock();
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR);

    HAL_FLASHEx_Erase(&EraseInitStruct, &PageError);

    HAL_FLASH_Lock();
}

/**
 * @brief    Flash读取
 * @param  	 读取的地址
 * @param    存放数组
 * @retval   无
 */
void Flash_Data_Read(uint32_t Addr, uint64_t *pData)
{
    for (uint32_t i = 0; i < 32; i++) {
        pData[i] = *(__IO uint64_t *)(Addr + (i * 8));
    }
}

/**
 * @brief    Flash写入
 * @param  	 读取的地址
 * @param    存放数组
 * @retval   无
 */
void Flash_Data_Write(uint32_t Addr, uint64_t *pData)
{
    HAL_FLASH_Unlock();
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR);
    for (uint16_t i = 0; i < 32; i++) {
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, Addr + (i * 8), pData[i]);
    }
    HAL_FLASH_Lock();
}

/**
 * @brief    获取Flash的页数
 * @param  	 传入的地址
 * @retval   Flash页数
 */
uint32_t GetPage(uint32_t Addr)
{
    return (Addr - FLASH_BASE) / FLASH_PAGE_SIZE;
}
