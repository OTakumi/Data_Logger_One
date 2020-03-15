/*
 * adxl345.c
 *
 *  Created on: 2020/03/09
 *      Author: takumi
 */

#include <main.h>
#include <stm32l0xx.h>
#include <stdbool.h>
#include <adxl345.h>

SPI_HandleTypeDef hspi1;

//###################################################################################################################
bool ADXL345_init(void)
{
	/*
	 *  ADXL345 setting
	 */
	ADXL345_SPI_Write(XL345_DATA_FORMAT & 0x7f, 0X0b);		//BIT6: SPI4 line mode (default); BIT5: interrupt level 0/1 (high/low active); BIT0-1: range=16g
	ADXL345_SPI_Write(XL345_POWER_CTL & 0x7f, 0x08);		//BIT3=0/1: (measurement mode/standby mode); BIT2=0/1: (work/hibernate);
	ADXL345_SPI_Write(XL345_BW_RATE & 0x7f, 0x0e);			//low 4 bits: output data rate=1600 (at this rate, SPI rate should be set >=2M); BIT4=0/1 (low power/normal)
	ADXL345_SPI_Write(XL345_INT_ENABLE & 0x7f, 0x00);		//Interrupt function setting: not enabled
	ADXL345_SPI_Write(XL345_INT_MAP & 0x7f, 0x00); 			//Set the interrupt mapping to the INT1 pin or the INT2 pin.
	ADXL345_SPI_Write(XL345_FIFO_CTL & 0x7f, 0x80);

	ADXL345_SPI_Write(XL345_OFSX & 0x7f, 0x00); 			//XYZ offset adjustment
	ADXL345_SPI_Write(XL345_OFSY & 0x7f, 0x00);
	ADXL345_SPI_Write(XL345_OFSZ & 0x7f, 0x00);

	// Get device id
	uint8_t xl345_read_data[3] = { XL345_DEVID | 0xc0, 0x00, };
	ADXL345_SPI_Read(xl345_read_data, sizeof(xl345_read_data));

	uint8_t device_id = xl345_read_data[1];
	if (device_id != XL345_I_M_DEVID)
	{
		return false;
	}
	else
	{
		return true;
	}
}

//###################################################################################################################
void XL345_readXYZ(int8_t *xl345_data_buf)
{
	// Setting the data format
	ADXL345_SPI_Write(XL345_DATA_FORMAT & 0x7f, 0x0f);

	// Read multibit
	uint8_t xl345_accel_data[7] = { };
	int16_t xl345_xyz[3] = { };
	xl345_accel_data[0] = XL345_DATAX0 | 0xc0;

	/* data read multi bits XL345_DATAX0 ~ XL345_DATAZ1 */
	ADXL345_SPI_Read(xl345_accel_data, sizeof(xl345_accel_data));

	xl345_xyz[0] = ((int16_t) xl345_accel_data[2] << 8) + xl345_accel_data[1];
	xl345_xyz[1] = ((int16_t) xl345_accel_data[4] << 8) + xl345_accel_data[3];
	xl345_xyz[2] = ((int16_t) xl345_accel_data[6] << 8) + xl345_accel_data[5];

	xl345_data_buf[0] = (float) xl345_xyz[0] * 0.0039;
	xl345_data_buf[1] = (float) xl345_xyz[1] * 0.0039;
	xl345_data_buf[2] = (float) xl345_xyz[2] * 0.0039;
}

//###################################################################################################################
void ADXL345_SPI_Read(uint8_t *read_data_buf, uint8_t buf_size)
{
	XL345_CS_LOW();
	HAL_Delay(5);
	HAL_SPI_Receive(&hspi1, read_data_buf, buf_size, 100);
	HAL_Delay(5);
	XL345_CS_HIGH();
}

//###################################################################################################################
void ADXL345_SPI_Write(uint8_t addr, uint8_t data)
{
	uint8_t xl345_write_data_buf[2] =
	{ };
	xl345_write_data_buf[0] = addr;
	xl345_write_data_buf[1] = data;

	XL345_CS_LOW();
	HAL_Delay(5);
	HAL_SPI_Transmit(&hspi1, xl345_write_data_buf, sizeof(xl345_write_data_buf), 100);
	HAL_Delay(5);
	XL345_CS_HIGH();
}
