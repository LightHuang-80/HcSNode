/*
 * ocflash.h
 *
 *  Created on: 2021年3月6日
 *      Author: Administrator
 */

#ifndef SRC_FLASH_OCFLASH_H_
#define SRC_FLASH_OCFLASH_H_


#ifdef __cplusplus
extern "C" {
#endif

uint8_t Flash_WriteData(uint32_t addr,uint16_t *data,uint16_t Size);
uint8_t Flash_ReadData(uint32_t addr,uint16_t *data,uint16_t Size);

#ifdef __cplusplus
}
#endif

#endif /* SRC_FLASH_OCFLASH_H_ */
