/*
 * as5048a.c
 *
 *  Created on: 2020年8月19日
 *      Author: Administrator
 */


#include <stdlib.h>
#include <stdio.h>

#include "main.h"
#include "spi.h"

#define CMD_CLEAR      0x4001
#define CMD_NOP        0xc000
#define CMD_ANGLE      0x3FFF
#define CMD_READ_MAG   0x3FFE
#define CMD_READ_DIAD  0x3FFD

#define REG_ERROR      0x1f01
#define READ_ERROR     0x1f02
#define EVEN_ERROR     0x1e01

#define SPI_AS5048A_CS_LOW()   HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_RESET)
#define SPI_AS5048A_CS_HIGH()  HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_SET)

void AS5048A_Init()
{

}

static uint16_t AS5048A_SPI_WriteData(uint16_t data, uint16_t *rx)
{
	HAL_StatusTypeDef status;
	SPI_AS5048A_CS_LOW();
	status = HAL_SPI_TransmitReceive(&hspi3, (uint8_t*)&data, (uint8_t*)rx, 1, HAL_MAX_DELAY);
	SPI_AS5048A_CS_HIGH();

	return status;
}

static uint8_t parity_even(uint16_t v)
{
	if (v == 0) return 0;

    v ^= v >> 8;
    v ^= v >> 4;
    v ^= v >> 2;
    v ^= v >> 1;

    return v & 1;
}

static void AS5048A_ClearAndNop()
{
	uint16_t res = 0;
	uint16_t command = CMD_CLEAR;

	command |= ((uint16_t)parity_even(command) << 15);
	AS5048A_SPI_WriteData(command, &res);

	res = 0;
	AS5048A_SPI_WriteData(command, &res);
}

uint16_t AS5048A_ReadReg(uint16_t cmd, uint16_t *val)
{
	uint16_t status;

	uint16_t rx;
	uint16_t command = 0x4000;

	command = command | cmd;
	command |= ((uint16_t)parity_even(command) << 15);

	status = AS5048A_SPI_WriteData(command, &rx);

	if (status != HAL_OK)
		return REG_ERROR;

	command = CMD_NOP;
	status = AS5048A_SPI_WriteData(command, val);
	if (status != HAL_OK)
		return REG_ERROR;

	if ((*val & 0x4000) != 0){
		AS5048A_ClearAndNop();
		status = READ_ERROR;
	}

	return status;
}


uint16_t AS5048A_WriteReg(uint16_t cmd, uint16_t val)
{
	HAL_StatusTypeDef status;

	uint16_t rx;
	uint16_t command = 0x0000;

	command = command | cmd;
	command |= ((uint16_t)parity_even(command) << 15);

	status = AS5048A_SPI_WriteData(command, &rx);

	if (status != HAL_OK)
		return REG_ERROR;

	command = 0x0000;
	command = command | val;

	command |= ((uint16_t)parity_even(command) << 15);
	status = AS5048A_SPI_WriteData(command, &rx);

	rx = 0;
	command = 0x0000;

	status = AS5048A_SPI_WriteData(command, &rx);

	if ((rx & (1 << 14)) != 0 ){
		AS5048A_ClearAndNop();
		return REG_ERROR;
	}

	uint16_t data = rx & 0x3fff;
	uint16_t error = parity_even(data) ^ (rx >> 15);
	if (error)
		return EVEN_ERROR;

	return HAL_OK;
}

uint16_t AS5048A_ReadValue(uint16_t cmd, uint16_t *value)
{
	HAL_StatusTypeDef status;
	uint16_t rx;

	status = AS5048A_ReadReg(cmd, &rx);

	uint16_t data = rx & 0x3fff;
	uint16_t error = parity_even(data) ^ (rx >> 15);

	if (error){
		*value = 0;
		return EVEN_ERROR;
	}else{
		*value = data;
	}

	return status;
}

uint16_t AS5048A_ReadAngle(uint16_t *angle)
{
  uint16_t status;
  status = AS5048A_ReadValue(CMD_ANGLE, angle);

  return status;
}

uint16_t AS5048A_ZeroPoint()
{
	uint16_t status;
	//1. Read angle information

	uint16_t angle;
	status = AS5048A_ReadAngle(&angle);
	if (status != HAL_OK)
		return status;

	printf("Before burn Zero, the angle: %d\n", angle);

	//2. Set the Programming Enable bit in the OTP control register
	uint16_t prog = 0x0001;
	status = AS5048A_WriteReg(0x0003, prog);
	if (status != HAL_OK)
		return status;

	//3. Write previous read angle position into OTP zero position register
	uint16_t angle_Hi = angle >> 6;
	uint16_t angle_Lo = angle & 0x003F;
	status = AS5048A_WriteReg(0x0016, angle_Hi);
	if (status != HAL_OK)
		return status;

	status = AS5048A_WriteReg(0x0017, angle_Lo);
	if (status != HAL_OK)
		return status;

	//4. Read back for verification the zero position register data
	AS5048A_ReadReg(0x0016, &angle_Hi);
	AS5048A_ReadReg(0x0017, &angle_Lo);

	//5. Set the Burn bit to start the automatic programming procedure
	prog |= 0x0008;
	status = AS5048A_WriteReg(0x0003, prog);
	if (status != HAL_OK)
		return status;

	HAL_Delay(100);

	//6. Read angle information (equals to 0)
	status = AS5048A_ReadAngle(&angle);
	if (status != HAL_OK)
		return status;
	printf("After burn, current position: %d\n", angle);

	//7. Set the Verify bit to load the OTP data again into the internal registers with modified threshold comparator levels
	prog |= 0x0040;
	status = AS5048A_WriteReg(0x0003, prog);
	if (status != HAL_OK)
		return status;

	AS5048A_ReadReg(0x0016, &angle_Hi);
	AS5048A_ReadReg(0x0017, &angle_Lo);

	//8. Read angle information (equals to 0)
	status = AS5048A_ReadAngle(&angle);
	if (status != HAL_OK)
		return status;
	printf("After set Zero, current position: %d\n", angle);

	return status;
}
