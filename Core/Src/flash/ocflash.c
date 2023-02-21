/*
 * ocflash.c
 *
 *  Created on: 2021年3月6日
 *      Author: Administrator
 */

#include <stdbool.h>

#include "main.h"
#include "stm32f4xx_hal_flash.h"

// Setup the last sector as eeprom base
#define JM_EEPROM_BASE_ADDR 0x080E0000
#define JM_EEPROM_MAX_SIZE  131072  // 128K

//FLASH write data test
uint8_t Flash_WriteData(uint32_t addr,uint16_t *data,uint16_t Size)
{
  if (addr >= JM_EEPROM_MAX_SIZE ||
	  (addr+Size) > JM_EEPROM_MAX_SIZE)
	  return HAL_ERROR;

  //unlock FLASH
  HAL_FLASH_Unlock();

  // erase FLASH
  FLASH_EraseInitTypeDef f;
  f.TypeErase = FLASH_TYPEERASE_SECTORS;
  f.VoltageRange = FLASH_VOLTAGE_RANGE_3;
  f.Sector  = FLASH_SECTOR_11; // the last sector
  f.NbSectors = 1;

  // Set PageError
  uint32_t PageError = 0;
  // call the erase function
  if (HAL_OK != HAL_FLASHEx_Erase(&f, &PageError)){
	  HAL_FLASH_Lock();
	  return HAL_ERROR;
  }

  uint32_t phaddr = JM_EEPROM_BASE_ADDR + addr;

  //burn to FLASH
  uint32_t TempBuf = 0;
  for(uint32_t i = 0;i< Size ;i++) {
    TempBuf = ~(*(data+i));
	TempBuf <<= 16;
	TempBuf += *(data+i); //Reverse check
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD , phaddr + i * 4, TempBuf);
  }

  //lock FLASH
  HAL_FLASH_Lock();
  return HAL_OK;
}

// FLASH read data test, successfully returned 1
uint8_t Flash_ReadData(uint32_t addr, uint16_t *data, uint16_t Size)
{
  if (addr >= JM_EEPROM_MAX_SIZE ||
	 (addr+Size) > JM_EEPROM_MAX_SIZE)
	  return HAL_ERROR;

  uint32_t temp;
  uint8_t result = HAL_OK;

  uint32_t phaddr = JM_EEPROM_BASE_ADDR + addr;

  for(uint32_t i = 0; i< Size; i++) {
    temp = *(__IO uint32_t*)(phaddr + i * 4);
    if((uint16_t)temp == (uint16_t)(~(temp>>16))) {
      *(data+i) = (uint16_t)temp;
    }else {
      result = HAL_ERROR;
    }
  }

  return result;
}
