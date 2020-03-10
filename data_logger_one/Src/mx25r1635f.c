/*
 * mx25r1635r.c
 *
 *  Created on: 2020/03/09
 *      Author: takumi
 */

#include <main.h>
#include <stm32l0xx.h>
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
	MX25Rxx_Spi(0x06);
	MX25_CS_HIGH();
	MX25Rxx_Spi(1);
}

//###################################################################################################################
uint8_t MX25qxx_ReadStatusRegister(uint8_t SelectStatusRegister_1_2_3)
{
	uint8_t status = 0;
	MX25_CS_LOW();
	if (SelectStatusRegister_1_2_3 == 1)
	{
		MX25Rxx_Spi(0x05);
		status = MX25Rxx_Spi(MX25R_DUMMY_BYTE);
		mx25rxx.StatusRegister1 = status;
	}
	else if (SelectStatusRegister_1_2_3 == 2)
	{
		MX25Rxx_Spi(0x35);
		status = MX25Rxx_Spi(MX25R_DUMMY_BYTE);
		mx25rxx.StatusRegister2 = status;
	}
	else
	{
		MX25Rxx_Spi(0x15);
		status = MX25Rxx_Spi(MX25R_DUMMY_BYTE);
		mx25rxx.StatusRegister3 = status;
	}
	MX25_CS_HIGH();
	return status;
}

//###################################################################################################################
void MX25Rxx_WaitForWriteEnd(void)
{
	uint8_t status = 0;
	HAL_Delay(1);
	MX25_CS_LOW();
	MX25Rxx_Spi(FLASH_CMD_WREN);
	do
	{
		status = MX25Rxx_Spi(MX25R_DUMMY_BYTE);
		mx25rxx.StatusRegister1 = status;
		HAL_Delay(1);
	}
	while ((mx25rxx.StatusRegister1 & 0x01) == 0x01);
	MX25_CS_HIGH();
}

//###################################################################################################################
void MX25Rxx_WriteByte(uint8_t pBuffer, uint32_t WriteAddr_inBytes)
{
	MX25Rxx_WaitForWriteEnd();
	MX25Rxx_WriteEnable();
	MX25_CS_LOW();
	MX25Rxx_Spi(FLASH_CMD_PP);
	MX25Rxx_Spi((WriteAddr_inBytes & 0xFF0000) >> 16);
	MX25Rxx_Spi((WriteAddr_inBytes & 0xFF00) >> 8);
	MX25Rxx_Spi(WriteAddr_inBytes & 0xFF);
	MX25Rxx_Spi(pBuffer);
	MX25_CS_HIGH();
	MX25Rxx_WaitForWriteEnd();
}

//###################################################################################################################
void MX25Rxx_ReadByte(uint8_t *pBuffer, uint32_t Bytes_Address)
{
	MX25_CS_LOW();
	MX25Rxx_Spi(FLASH_CMD_READ);
	MX25Rxx_Spi((Bytes_Address & 0xFF0000) >> 16);
	MX25Rxx_Spi((Bytes_Address & 0xFF00) >> 8);
	MX25Rxx_Spi(Bytes_Address & 0xFF);
	MX25Rxx_Spi(0);
	*pBuffer = MX25Rxx_Spi(MX25R_DUMMY_BYTE);
	MX25_CS_HIGH();
}
