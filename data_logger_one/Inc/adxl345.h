/**************************************************************************/
/*!
    @file     adxl345.h
    @author   Takumi O
    @Created on: 2019/11/03
*/
/**************************************************************************/

#ifndef ADXL345_H_
#define ADXL345_H_



#endif /* ADXL345_H_ */

/*=========================================================================
	ADXL345 REGISTER MAP from ADXL345 Datasheet */

#define XL345_DEVID						0x00		// Device ID
#define XL345_THRESH_TAP				0x1D		// Tap threshold
#define XL345_OFSX						0x1E		// X-axis Offset
#define	XL345_OFSY						0x1F		// Y-axis Offset
#define XL345_OFSZ						0x20		// Z-axis Offset
#define XL345_DUR						0x21		// Tap duration
#define	XL345_Latent					0x22		// Tap latency
#define	XL345_Window					0x23		// Tap window
#define	XL345_THRESH_ACT				0x24		// Activity threshold
#define	XL345_THRESH_INACT				0x25		// Inactivity threshold
#define XL345_TIME_INACT				0x26		// Inactivity time
#define XL345_ACT_INACT_CTL				0x27		// Axis enable control for activity detection
#define	XL345_THRESH_FF					0x28		// Free-fall threshold
#define	XL345_TIME_FF					0x29		// Free-fall time
#define XL345_TAP_AXES					0x2A		// Axis control for single tap/double tap
#define XL345_ACT_TAP_STATUS			0x2B		// Source of single tap/double tap
#define XL345_BW_RATE					0x2C		// Date rate and power mode control
#define	XL345_POWER_CTL					0x2D		// Power-saving feature control
#define XL345_INT_ENABLE				0x2E		// Interrupt enable control
#define XL345_INT_MAP					0x2F		// Interrupt mapping control
#define	XL345_INIT_SOURCE				0x30		// Source of interrupt
#define XL345_DATA_FORMAT				0x31		// Data format control
#define XL345_DATAX0					0x32		// X-Axis Data 0
#define XL345_DATAX1					0x33		// X-Axis Data 1
#define XL345_DATAY0					0x34		// Y-Axis Data 0
#define XL345_DATAY1					0x35		// Y-Axis Data 1
#define XL345_DATAZ0					0x36		// Z-Axis Data 0
#define XL345_DATAZ1					0x37		// Z-Axis Data 1
#define	XL345_FIFO_CTL					0x38		// FIFO control
#define XL345_FIFO_STATUS				0x39		// FIFO status

#define XL345_I_M_DEVID ((uint8_t)0xE5)			// Device ID = 0xE5
/*=========================================================================*/

/*=========================================================================
    REGISTERS
    -----------------------------------------------------------------------*/
    #define ADXL345_MG2G_MULTIPLIER (0.004)  // 4mg per lsb
/*=========================================================================*/
/* Used with register 0x2C (ADXL345_REG_BW_RATE) to set bandwidth */
typedef enum
{
	XL345_DATARATE_3200_HZ = 0b1111, // 1600Hz Bandwidth   140É A IDD
	XL345_DATARATE_1600_HZ = 0b1110, //  800Hz Bandwidth    90É A IDD
	XL345_DATARATE_800_HZ = 0b1101, //  400Hz Bandwidth   140É A IDD
	XL345_DATARATE_400_HZ = 0b1100, //  200Hz Bandwidth   140É A IDD
	XL345_DATARATE_200_HZ = 0b1011, //  100Hz Bandwidth   140É A IDD
	XL345_DATARATE_100_HZ = 0b1010, //   50Hz Bandwidth   140É A IDD
	XL345_DATARATE_50_HZ = 0b1001, //   25Hz Bandwidth    90É A IDD
	XL345_DATARATE_25_HZ = 0b1000, // 12.5Hz Bandwidth    60É A IDD
	XL345_DATARATE_12_5_HZ = 0b0111, // 6.25Hz Bandwidth    50É A IDD
	XL345_DATARATE_6_25HZ = 0b0110, // 3.13Hz Bandwidth    45É A IDD
	XL345_DATARATE_3_13_HZ = 0b0101, // 1.56Hz Bandwidth    40É A IDD
	XL345_DATARATE_1_56_HZ = 0b0100, // 0.78Hz Bandwidth    34É A IDD
	XL345_DATARATE_0_78_HZ = 0b0011, // 0.39Hz Bandwidth    23É A IDD
	XL345_DATARATE_0_39_HZ = 0b0010, // 0.20Hz Bandwidth    23É A IDD
	XL345_DATARATE_0_20_HZ = 0b0001, // 0.10Hz Bandwidth    23É A IDD
	XL345_DATARATE_0_10_HZ = 0b0000 // 0.05Hz Bandwidth    23É A IDD (default value)
} dataRate_t;

/* Used with register 0x31 (ADXL345_REG_DATA_FORMAT) to set g range */
typedef enum
{
	XL345_RANGE_16_G = 0b11,   // +/- 16g
	XL345_RANGE_8_G = 0b10,   // +/- 8g
	XL345_RANGE_4_G = 0b01,   // +/- 4g
	XL345_RANGE_2_G = 0b00    // +/- 2g (default value)
} range_t;

/******************SPI pin configuration macro **********************/
#define XL345_SPIX                    SPI1
#define XL345_SPI_RCC                 RCC_APB1Periph_SPI1

#define XL345_SPI_SCK_PIN             GPIO_Pin_13
#define XL345_SPI_SCK_GPIO_PORT       GPIOB
#define XL345_SPI_SCK_GPIO_RCC        RCC_AHB1Periph_GPIOB
#define XL345_SPI_SCK_SOURCE          GPIO_PinSource13
#define XL345_SPI_SCK_AF              GPIO_AF_SPI2

#define XL345_SPI_MISO_PIN            GPIO_Pin_14
#define XL345_SPI_MISO_GPIO_PORT      GPIOB
#define XL345_SPI_MISO_GPIO_RCC       RCC_AHB1Periph_GPIOB
#define XL345_SPI_MISO_SOURCE         GPIO_PinSource14
#define XL345_SPI_MISO_AF             GPIO_AF_SPI2

#define XL345_SPI_MOSI_PIN            GPIO_Pin_15
#define XL345_SPI_MOSI_GPIO_PORT      GPIOB
#define XL345_SPI_MOSI_GPIO_RCC       RCC_AHB1Periph_GPIOB
#define XL345_SPI_MOSI_SOURCE         GPIO_PinSource15
#define XL345_SPI_MOSI_AF             GPIO_AF_SPI2

#define XL345_SPI_CS_PIN              GPIO_Pin_12
#define XL345_SPI_CS_GPIO_PORT        GPIOB
#define XL345_SPI_CS_GPIO_RCC         RCC_AHB1Periph_GPIOB

#define XL345_CS_LOW()       			GPIO_ResetBits(ADXL345_SPI_CS_GPIO_PORT, ADXL345_SPI_CS_PIN)
#define XL345_CS_HIGH()      			GPIO_SetBits(ADXL345_SPI_CS_GPIO_PORT, ADXL345_SPI_CS_PIN)

/* Private function prototypes -----------------------------------------------*/
