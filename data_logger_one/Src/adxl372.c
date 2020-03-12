/*
 * adxl372.c
 *
 *  Created on: 2020/03/09
 *      Author: takumi
 */

#include <main.h>
#include <stm32l0xx.h>
#include <stdbool.h>
#include <adxl372.h>

SPI_HandleTypeDef hspi1;

//###################################################################################################################
bool adxl372_init(void)
{
	uint8_t xl372_rx_data_buf[3] = { };

	// setting the function
	adxl372_Settings();

	// get power mode status
	xl372_rx_data_buf[0] = ADXL372_POWER_CTL << 1 | 0x01;
	adxl372_SPIRead(xl372_rx_data_buf, sizeof(xl372_rx_data_buf));

	// get device id
	xl372_rx_data_buf[0] = ADXL372_PARTID << 1 | 0x01;
	adxl372_SPIRead(xl372_rx_data_buf, sizeof(xl372_rx_data_buf));

	if (xl372_rx_data_buf[1] != ADXL372_PARTID_VAL)
	{
		// Uart_Message("ADXL372 SPI Error\r\n");
		return false;
	}
	else
	{
		// Uart_Message("ADXL372 SPI OK\r\n");
		return true;
	}
}

//###################################################################################################################
void adxl372_Settings(void)
{
	// STANDBY mode
	adxl372_SPIWrite(ADXL372_POWER_CTL << 1 & 0xfe,
			ADXL372_POWER_CTL_MODE(ADXL372_STANDBY));

	// Device Reset
	adxl372_SPIWrite(ADXL372_RESET << 1 & 0xfe, ADXL372_RESET_CODE);
	HAL_Delay(100);

	// FIFO Setting
	// FIFO_CTL_FORMAT_MODE = XYZ_FIFO
	// FIFO_CTL_MODE_MODE = FIFO_STREAMED
	adxl372_SPIWrite(ADXL372_FIFO_CTL << 1 & 0xfe,
			ADXL372_FIFO_CTL_FORMAT_MODE(ADXL372_XYZ_FIFO)
					| ADXL372_FIFO_CTL_MODE_MODE(ADXL372_FIFO_STREAMED) | 0);

	// Measurement CTL Setting
	// AUTOSLEEP_MODE(6bit) = Disable(0)
	// LINKLOOP_MODE(5:4bit) = default mode(0)
	// LOW_NOISE_MODE(3bit) = Enable(1)
	// BANDWIDTH_MODE(2:0bit) = 3200Hz(100)
	adxl372_SPIWrite(ADXL372_MEASURE << 1 & 0xfe,
			ADXL372_MEASURE_AUTOSLEEP_MODE(0)
			| ADXL372_MEASURE_LINKLOOP_MODE(0)
			| ADXL372_MEASURE_LOW_NOISE_MODE(1)
			| ADXL372_MEASURE_BANDWIDTH_MODE(ADXL372_BW_3200HZ));

	// Timing Setting
	// Output data rate(7:5bit) = 800Hz ODR(001)
	// WAKE_UP_RATE_MODE(4:2bit) = 512ms(011)
	// EXT_CLK_MODE(1bit) = Disable(0)
	// EXT_SYNC_MODE(0bit) = Disable(0)
	adxl372_SPIWrite(ADXL372_TIMING << 1 & 0xfe,
			ADXL372_TIMING_ODR_MODE(ADXL372_ODR_800HZ)
					| ADXL372_TIMING_WAKE_UP_RATE_MODE(ADXL372_WUR_512ms)
					| ADXL372_TIMING_EXT_CLK_MODE(0)
					| ADXL372_TIMING_EXT_SYNC_MODE(0));

	// Power CTL Setting
	// 7, 6bit = Reserved
	// INSTANT_ON_TH_MODE(5bit) = low instant on threshold(0)
	// FIL_SETTLE_MODE(4bit) = 370ms(0)
	// POWER_CTL_MODE(1:0bit) = FULL_BW_MEASUREMENT(11)
	adxl372_SPIWrite(ADXL372_POWER_CTL << 1 & 0xfe,
			ADXL372_POWER_CTL_INSTANT_ON_TH_MODE(ADXL372_INSTANT_ON_LOW_TH)
					| ADXL372_POWER_CTL_FIL_SETTLE_MODE(ADXL372_FILTER_SETTLE_370)
					| ADXL372_POWER_CTL_LPF_DIS_MODE(1)
					| ADXL372_POWER_CTL_HPF_DIS_MODE(0)
					| ADXL372_POWER_CTL_MODE(ADXL372_FULL_BW_MEASUREMENT));	// FULL_BW_MEASUREMENT mode

	// Self test
	adxl372_SPIWrite(ADXL372_SELF_TEST << 1 & 0xfe, 0x01);
	// Wait about 300 ms for self-test to complete
	HAL_Delay(320);
	uint8_t self_test_check[4] = { ADXL372_SELF_TEST << 1 | 0x01, };
	adxl372_SPIRead(self_test_check, sizeof(self_test_check));

	if (self_test_check[1] == 0x06)
	{
		Uart_Message("ADXL372 self test finished. \r\n");
	}
	else
	{
		Uart_Message("ADXL372 self test is failed. \r\n");
	}
}

//###################################################################################################################
void adxl372_ReadXYZ(int8_t *xl372_data_buf)
{
	uint8_t check_status[2] = { ADXL372_STATUS_1 << 1 | 0x01, };
	uint8_t xl372_xyz_data[7] =	{ };
	uint8_t xyz_addrs[3] = { };
	int16_t i16_xyz_data[3] = { };

	adxl372_SPIRead(check_status, sizeof(check_status));

	xyz_addrs[0] = ADXL372_X_DATA_H << 1 | 0x01;
	xyz_addrs[1] = ADXL372_Y_DATA_H << 1 | 0x01;
	xyz_addrs[2] = ADXL372_Z_DATA_H << 1 | 0x01;
	for(int8_t i = 0; i < 3; i++)
	{
		xl372_xyz_data[0] = xyz_addrs[i];
		adxl372_SPIRead(xl372_xyz_data, sizeof(xl372_xyz_data));
		i16_xyz_data[i] = ((int16_t) xl372_xyz_data[1] << 8) | (xl372_xyz_data[2]);
	}

	/*
	i16_xyz_data[0] = ((int16_t) xl372_xyz_data[1] << 8)
			| (xl372_xyz_data[2]);
	i16_xyz_data[1] = ((int16_t) xl372_xyz_data[3] << 8)
			| (xl372_xyz_data[4]);
	i16_xyz_data[2] = ((int16_t) xl372_xyz_data[5] << 8)
			| (xl372_xyz_data[6]);
	*/

	xl372_data_buf[0] = i16_xyz_data[0] * 0.1;
	xl372_data_buf[1] = i16_xyz_data[1] * 0.1;
	xl372_data_buf[2] = i16_xyz_data[2] * 0.1;
}

//###################################################################################################################
void adxl372_SPIRead(uint8_t *read_data_buf, uint8_t buf_size)
{
	ADXL372_CS_LOW();
	HAL_Delay(5);
	HAL_SPI_Receive(&hspi1, read_data_buf, buf_size, 100);
	HAL_Delay(5);
	ADXL372_CS_HIGH();
}

//###################################################################################################################
void adxl372_SPIWrite(uint8_t addr, uint8_t data)
{
	uint8_t xl372_write_data_buf[4] = { };
	xl372_write_data_buf[0] = addr;
	xl372_write_data_buf[1] = data;

	ADXL372_CS_LOW();
	HAL_Delay(5);
	HAL_SPI_Transmit(&hspi1, xl372_write_data_buf, sizeof(xl372_write_data_buf), 100);
	HAL_Delay(5);
	ADXL372_CS_HIGH();
}
