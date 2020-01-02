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

#define DEVID						0x00		// Device ID
#define THRESH_TAP					0x1D		// Tap threshold
#define OFSX						0x1E		// X-axis Offset
#define	OFSY						0x1F		// Y-axis Offset
#define OFSZ						0x20		// Z-axis Offset
#define DUR							0x21		// Tap duration
#define	Latent						0x22		// Tap latency
#define	Window						0x23		// Tap window
#define	THRESH_ACT					0x24		// Activity threshold
#define	THRESH_INACT				0x25		// Inactivity threshold
#define TIME_INACT					0x26		// Inactivity time
#define ACT_INACT_CTL				0x27		// Axis enable control for activity detection
#define	THRESH_FF					0x28		// Free-fall threshold
#define	TIME_FF						0x29		// Free-fall time
#define TAP_AXES					0x2A		// Axis control for single tap/double tap
#define ACT_TAP_STATUS				0x2B		// Source of single tap/double tap
#define BW_RATE						0x2C		// Date rate and power mode control
#define	POWER_CTL					0x2D		// Power-saving feature control
#define INT_ENABLE					0x2E		// Interrupt enable control
#define INT_MAP						0x2F		// Interrupt mapping control
#define	INIT_SOURCE					0x30		// Source of interrupt
#define DATA_FORMAT					0x31		// Data format control
#define DATAX0						0x32		// X-Axis Data 0
#define DATAX1						0x33		// X-Axis Data 1
#define DATAY0						0x34		// Y-Axis Data 0
#define DATAY1						0x35		// Y-Axis Data 1
#define DATAZ0						0x36		// Z-Axis Data 0
#define DATAZ1						0x37		// Z-Axis Data 1
#define	FIFO_CTL					0x38		// FIFO control
#define FIFO_STATUS					0x39		// FIFO status

#define I_M_DEVID ((uint8_t)0xE5)			// Device ID = 0xE5
/*=========================================================================*/

/*=========================================================================
    REGISTERS
    -----------------------------------------------------------------------*/
    #define ADXL345_MG2G_MULTIPLIER (0.004)  // 4mg per lsb
/*=========================================================================*/
/* Used with register 0x2C (ADXL345_REG_BW_RATE) to set bandwidth */
typedef enum
{
  ADXL345_DATARATE_3200_HZ    = 0b1111, // 1600Hz Bandwidth   140É A IDD
  ADXL345_DATARATE_1600_HZ    = 0b1110, //  800Hz Bandwidth    90É A IDD
  ADXL345_DATARATE_800_HZ     = 0b1101, //  400Hz Bandwidth   140É A IDD
  ADXL345_DATARATE_400_HZ     = 0b1100, //  200Hz Bandwidth   140É A IDD
  ADXL345_DATARATE_200_HZ     = 0b1011, //  100Hz Bandwidth   140É A IDD
  ADXL345_DATARATE_100_HZ     = 0b1010, //   50Hz Bandwidth   140É A IDD
  ADXL345_DATARATE_50_HZ      = 0b1001, //   25Hz Bandwidth    90É A IDD
  ADXL345_DATARATE_25_HZ      = 0b1000, // 12.5Hz Bandwidth    60É A IDD
  ADXL345_DATARATE_12_5_HZ    = 0b0111, // 6.25Hz Bandwidth    50É A IDD
  ADXL345_DATARATE_6_25HZ     = 0b0110, // 3.13Hz Bandwidth    45É A IDD
  ADXL345_DATARATE_3_13_HZ    = 0b0101, // 1.56Hz Bandwidth    40É A IDD
  ADXL345_DATARATE_1_56_HZ    = 0b0100, // 0.78Hz Bandwidth    34É A IDD
  ADXL345_DATARATE_0_78_HZ    = 0b0011, // 0.39Hz Bandwidth    23É A IDD
  ADXL345_DATARATE_0_39_HZ    = 0b0010, // 0.20Hz Bandwidth    23É A IDD
  ADXL345_DATARATE_0_20_HZ    = 0b0001, // 0.10Hz Bandwidth    23É A IDD
  ADXL345_DATARATE_0_10_HZ    = 0b0000  // 0.05Hz Bandwidth    23É A IDD (default value)
} dataRate_t;

/* Used with register 0x31 (ADXL345_REG_DATA_FORMAT) to set g range */
typedef enum
{
  ADXL345_RANGE_16_G          = 0b11,   // +/- 16g
  ADXL345_RANGE_8_G           = 0b10,   // +/- 8g
  ADXL345_RANGE_4_G           = 0b01,   // +/- 4g
  ADXL345_RANGE_2_G           = 0b00    // +/- 2g (default value)
} range_t;

/******************SPI pin configuration macro **********************/
#define ADXL345_SPIX                    SPI1
#define ADXL345_SPI_RCC                 RCC_APB1Periph_SPI1

#define ADXL345_SPI_SCK_PIN             GPIO_Pin_13
#define ADXL345_SPI_SCK_GPIO_PORT       GPIOB
#define ADXL345_SPI_SCK_GPIO_RCC        RCC_AHB1Periph_GPIOB
#define ADXL345_SPI_SCK_SOURCE          GPIO_PinSource13
#define ADXL345_SPI_SCK_AF              GPIO_AF_SPI2

#define ADXL345_SPI_MISO_PIN            GPIO_Pin_14
#define ADXL345_SPI_MISO_GPIO_PORT      GPIOB
#define ADXL345_SPI_MISO_GPIO_RCC       RCC_AHB1Periph_GPIOB
#define ADXL345_SPI_MISO_SOURCE         GPIO_PinSource14
#define ADXL345_SPI_MISO_AF             GPIO_AF_SPI2

#define ADXL345_SPI_MOSI_PIN            GPIO_Pin_15
#define ADXL345_SPI_MOSI_GPIO_PORT      GPIOB
#define ADXL345_SPI_MOSI_GPIO_RCC       RCC_AHB1Periph_GPIOB
#define ADXL345_SPI_MOSI_SOURCE         GPIO_PinSource15
#define ADXL345_SPI_MOSI_AF             GPIO_AF_SPI2

#define ADXL345_SPI_CS_PIN              GPIO_Pin_12
#define ADXL345_SPI_CS_GPIO_PORT        GPIOB
#define ADXL345_SPI_CS_GPIO_RCC         RCC_AHB1Periph_GPIOB

#define ADXL345_CS_LOW()       			GPIO_ResetBits(ADXL345_SPI_CS_GPIO_PORT, ADXL345_SPI_CS_PIN)
#define ADXL345_CS_HIGH()      			GPIO_SetBits(ADXL345_SPI_CS_GPIO_PORT, ADXL345_SPI_CS_PIN)

/* Private function prototypes -----------------------------------------------*/
