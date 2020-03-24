/*
 * mx25r1635r.c
 *
 *  Created on: 2020/03/09
 *      Author: takumi
 */

#include <main.h>
#include <stm32l0xx.h>
#include <stdbool.h>
#include <stdint.h>
#include <mx25r1635f.h>
#include <mx25r_config.h>

#define MX25R_DUMMY_BYTE         0xA5

SPI_HandleTypeDef hspi2;

mx25rxx_t	mx25rxx;

//###################################################################################################################
uint8_t MX25Rxx_Spi(uint8_t Data)
{
	uint8_t ret;
	HAL_SPI_TransmitReceive(&hspi2, &Data, &ret, 1, 100);
	return ret;
}

//###################################################################################################################
uint32_t MX25Rxx_ReadID(void)
{
	uint32_t Temp = 0, Temp0 = 0, Temp1 = 0, Temp2 = 0;
	MX25_CS_LOW();
	MX25Rxx_Spi(0x9F);
	Temp0 = MX25Rxx_Spi(MX25R_DUMMY_BYTE);
	Temp1 = MX25Rxx_Spi(MX25R_DUMMY_BYTE);
	Temp2 = MX25Rxx_Spi(MX25R_DUMMY_BYTE);
	MX25_CS_HIGH();
	Temp = (Temp0 << 16) | (Temp1 << 8) | Temp2;
	return Temp;
}

//###################################################################################################################
void MX25Rxx_WriteEnable(void)
{
	MX25_CS_LOW();
	MX25Rxx_Spi(FLASH_CMD_WREN);
	MX25_CS_HIGH();
	HAL_Delay(1);
}

//###################################################################################################################
uint8_t MX25Rxx_ReadStatusRegister(void)
{
	uint8_t status[2] = { };
	status[1] = MX25R_DUMMY_BYTE;
	MX25_CS_LOW();
	status[0] = 0x05;
	HAL_SPI_Receive(&hspi2, status, sizeof(status), 0xff);
	mx25rxx.StatusRegister1 = status[1];
	MX25_CS_HIGH();
	return status[1];
}

////###################################################################################################################
//void MX25Rxx_WaitForWriteEnd(void)
//{
//	uint8_t status = 0;
//	HAL_Delay(1);
//	MX25_CS_LOW();
//	MX25Rxx_Spi(FLASH_CMD_RDSR);
//	do
//	{
//		status = MX25Rxx_Spi(MX25R_DUMMY_BYTE);
//		mx25rxx.StatusRegister1 = status;
//		HAL_Delay(1);
//	}
//	while ((mx25rxx.StatusRegister1 & 0x01) == 0x01);
//	MX25_CS_HIGH();
//}

//###################################################################################################################
void MX25Rxx_WriteByte(uint8_t* pBuffer, uint32_t Page_Address)
{
	uint8_t status = 0;

//	while (mx25rxx.Lock == 1)
//		HAL_Delay(1);
	mx25rxx.Lock = 1;

	MX25Rxx_WriteEnable();
	while(status != 0x02)
	{
		status = MX25Rxx_ReadStatusRegister();
		HAL_Delay(1);
	}

	MX25_CS_LOW();
	MX25Rxx_Spi(FLASH_CMD_PP);
	MX25Rxx_Spi((Page_Address & 0xFF0000) >> 16);
	MX25Rxx_Spi((Page_Address & 0xFF00) >> 8);
	MX25Rxx_Spi((Page_Address & 0xFF));

	HAL_SPI_Transmit(&hspi2, pBuffer, 0xf9, 0xff);

	while(status != 0x00)
	{
		status = MX25Rxx_ReadStatusRegister();
		HAL_Delay(1);
	}
//	MX25Rxx_WaitForWriteEnd();
//	HAL_Delay(20);
	MX25_CS_HIGH();
	mx25rxx.Lock = 0;
}
//###################################################################################################################
void MX25Rxx_ReadByte(uint8_t* pBuffer, uint32_t Bytes_Address)
{
//	while (mx25rxx.Lock == 1)
//		HAL_Delay(1);
	mx25rxx.Lock = 1;

	MX25_CS_LOW();
	MX25Rxx_Spi(FLASH_CMD_READ);
	MX25Rxx_Spi((Bytes_Address & 0xFF0000) >> 16);
	MX25Rxx_Spi((Bytes_Address & 0xFF00) >> 8);
	MX25Rxx_Spi(Bytes_Address & 0xFF);
	// MX25Rxx_Spi(0);
	HAL_SPI_Receive(&hspi2, pBuffer, 0xf9, 0xff);
	MX25_CS_HIGH();

	mx25rxx.Lock = 0;
}

//###################################################################################################################
void MX25Rxx_EraseSector(uint32_t SectorAddr)
{
	uint8_t status = 0;

//	while (mx25rxx.Lock == 1)
//		HAL_Delay(1);
//	mx25rxx.Lock = 1;

	MX25Rxx_WriteEnable();
	while(status != 0x02)
	{
		status = MX25Rxx_ReadStatusRegister();
		HAL_Delay(1);
	}

	MX25_CS_LOW();
	MX25Rxx_Spi(FLASH_CMD_SE);
	MX25Rxx_Spi((SectorAddr & 0xFF0000) >> 16);
	MX25Rxx_Spi((SectorAddr & 0xFF00) >> 8);
	MX25Rxx_Spi(SectorAddr & 0xFF);
	MX25_CS_HIGH();

	while(status != 0x00)
	{
		status = MX25Rxx_ReadStatusRegister();
		HAL_Delay(1);
	}

	mx25rxx.Lock = 0;
}
//###################################################################################################################
bool MX25Rxx_Init(void)
{
	MX25_WP_HIGH();
	MX25_CS_HIGH();
//	mx25rxx.Lock = 1;
//	while (HAL_GetTick() < 100)
//		HAL_Delay(1);
//	HAL_Delay(100);

	uint32_t id;
	id = MX25Rxx_ReadID();

	mx25rxx.PageSize = 256;
	mx25rxx.SectorSize = 0x1000;
	mx25rxx.SectorCount = mx25rxx.BlockCount * 16;
	mx25rxx.PageCount = (mx25rxx.SectorCount * mx25rxx.SectorSize)
			/ mx25rxx.PageSize;
	mx25rxx.BlockSize = mx25rxx.SectorSize * 16;
	mx25rxx.CapacityInKiloByte = (mx25rxx.SectorCount * mx25rxx.SectorSize)	/ 1024;
	MX25Rxx_ReadStatusRegister();

	mx25rxx.Lock = 0;
	if(id == 0xc22815)
	{
		return true;
	}
	else
	{
		return false;
	}
}
