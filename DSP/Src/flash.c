#include "flash.h"

extern void    FLASH_PageErase(uint32_t PageAddress);

/**
 * @brief       ��ָ����ַдһ���� (32λ����)
 * @param       д������� 8λ�������⴦��
 * @param       д��ĵ�ַ
 * @param      	����ָ��
 * @param       ���ݳ���
 * @retval      ��
 */
void Flash_Write(uint32_t TypeProgram, uint32_t Address, uint64_t (*Data), uint32_t len)
{

    HAL_FLASH_Unlock();

    FLASH_PageErase(Address);  									//�����������
    CLEAR_BIT(FLASH->CR, FLASH_CR_PER);                         //���CR�Ĵ�����PERλ���˲���Ӧ����FLASH_PageErase()����ɣ�
                                                                //����HAL�����沢û������Ӧ����HAL��bug��
    if (TypeProgram==TYPEPROGRAM_CHAR) {
        for (int var = 0; var < len/2; var++) {
            uint8_t data1=(uint8_t)*Data;
            uint8_t data2=(uint8_t)*(Data+1);
            uint64_t datar = data2<<8|data1;
            HAL_FLASH_Program(TYPEPROGRAM_HALFWORD, Address,datar);
            Address+=2;
            Data+=2;
        }
    } else {
        for (int var = 0; var < len; var++) {
            HAL_FLASH_Program(TypeProgram, Address, *Data);
            Address+=(1<<TypeProgram);
            Data+=1;
        }
    }
    HAL_FLASH_Lock();
}

/**
 * @brief       ��ָ����ַ��ȡһ���� (32λ����)
 * @param       д������� 8λ�������⴦��
 * @param       ��ȡ�ĵ�ַ
 * @param      	����ָ��
 * @param       ���ݳ���
 * @retval      ��
 */
void Flash_Read(uint32_t TypeProgram, uint32_t Address, uint64_t (*Data), uint32_t len)
{
    int32_t i=0;
    if (TypeProgram==TYPEPROGRAM_CHAR) {
       for (i = 0; i < len; i++, Address+=1, Data++)
       {
           *Data = *(uint8_t *) Address;
       }
    }else if (TypeProgram==TYPEPROGRAM_HALFWORD) {
       for (i = 0; i < len; i++, Address+=2, Data++)
       {
           *Data = *(uint16_t *) Address;
       }
    }else if (TypeProgram==TYPEPROGRAM_WORD) {
      for (i = 0; i < len; i++, Address+=4, Data++)
      {
          *Data = *(uint32_t *) Address;
      }
    }else if (TypeProgram==TYPEPROGRAM_DOUBLEWORD) {
      for (i = 0; i < len; i++, Address+=8, Data++)
      {
          *Data = *(uint64_t *) Address;
      }
    }
}
