/*
 * sw_spi.cpp
 *
 *  Created on: Mar 26, 2021
 *      Author: Administrator
 */

#include "main.h"
#include "fserial.h"

#include "spi.h"

#define SPI_TMC5160_CS_LOW()   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET)
#define SPI_TMC5160_CS_HIGH()  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET)

SW_SPIClass::SW_SPIClass(uint16_t mosi, uint16_t miso, uint16_t sck) :
  mosi_pin(mosi),
  miso_pin(miso),
  sck_pin(sck)
  {}

void SW_SPIClass::init() {
}

void SW_SPIClass::switchCSpin(bool state)
{
	if (state){
		SPI_TMC5160_CS_HIGH();
		HAL_Delay(2);
	}else{
		SPI_TMC5160_CS_LOW();
		HAL_Delay(2);
	}
}

uint8_t SW_SPIClass::transfer(uint8_t ulVal)
{
  uint8_t rx;

  HAL_StatusTypeDef status;
  //SPI_TMC5160_CS_LOW();
  status = HAL_SPI_TransmitReceive(&hspi2, (uint8_t*)&ulVal, (uint8_t*)&rx, 1, HAL_MAX_DELAY);
  //SPI_TMC5160_CS_HIGH();

  if (status != HAL_OK)
	  return 0;

  return rx;
}

uint16_t SW_SPIClass::transfer16(uint16_t data) {
  uint16_t returnVal = 0x0000;
  returnVal |= transfer((data>>8)&0xFF) << 8;
  returnVal |= transfer(data&0xFF) & 0xFF;
  return returnVal;
}


