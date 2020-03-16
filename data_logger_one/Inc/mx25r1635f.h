/*
 * mx25r1635r.h
 *
 *  Created on: 2020/03/09
 *      Author: takumi
 */

#ifndef MX25R1635R_H_
#define MX25R1635R_H_

typedef struct
{
	uint16_t	PageSize;
	uint32_t	PageCount;
	uint32_t	SectorSize;
	uint32_t	SectorCount;
	uint32_t	BlockSize;
	uint32_t	BlockCount;
	uint32_t	CapacityInKiloByte;
	uint8_t		StatusRegister1;
	uint8_t		Lock;

}mx25rxx_t;

extern mx25rxx_t	mx25rxx;

bool MX25Rxx_Init(void);
uint8_t	MX25Rxx_Spi(uint8_t);
uint32_t MX25Rxx_ReadID(void);
//void MX25Rxx_WriteEnable(void);
void MX25Rxx_WaitForWriteEnd(void);
void MX25Rxx_WriteByte(uint8_t*, uint32_t);
void MX25Rxx_ReadByte(uint8_t*, uint32_t);
void MX25Rxx_EraseSector(uint32_t);

#endif /* MX25R1635R_H_ */
