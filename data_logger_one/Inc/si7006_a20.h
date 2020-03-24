/*
 * si7006_a20.h
 *
 *  Created on: 2020/01/04
 *      Author: takumi
 */

#pragma once

#define Si7006_ADDERSS		0x40

/* I2C Commands */
#define Humidity_Hold				0xE5
#define Humidity_Not_Hold			0xF5
#define Temperature_Hold			0xE3
#define Temperature_Not_Hold		0xF3
#define Read_Temperature			0xE0

/* SI7006_A20_H_ */
