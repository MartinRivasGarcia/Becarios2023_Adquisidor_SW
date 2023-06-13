/**
 *	Adquisidor.h
 * 	Autor: Nicolas Sidorczuk
 * 
 * 
 */

#include "ADS1294.h"
#include <stdint.h>

//-----  Prototipos  ---------//
int32_t Config_BBB_Pins();
void ADS1294_Read(uint8_t *);
void ADS1294_SingleRead(uint8_t *);
void ADS1294_SendCommand(uint8_t);
void ADS1294_WriteRegister(uint8_t, uint8_t);
uint8_t ADS1294_ReadRegister(uint8_t);
void ADS1294_init(void);
void ADS1294_Set_DataRate(uint8_t);


//-----  General  ---------//
#define HIGH 	1
#define LOW 	0
#define ON 		1
#define OFF 	0

//-----  Beagle Bone Black  ---------//
#define SPI_MODE1set	1	// SPI settings are CPOL = 0 and CPHA = 1
#define DATAREADY 48

//-----  CONFIG1 Register  ---------//
#define CONFIG1_DEFAULT		0xC1	// High Resolution Mode = ON, Multiple ReadBack = ON, CLK_OUT = OFF, Output Data rate = 16kSPS
#define OUTPUT_DR_32K		0		// CONFIG1 Reg Bits2..0 = Output Data rate (en High Resolution Mode habilitado por defecto)
#define OUTPUT_DR_16K		1
#define OUTPUT_DR_8K		2
#define OUTPUT_DR_4K		3
#define OUTPUT_DR_2K		4
#define OUTPUT_DR_1K		5
#define OUTPUT_DR_500		6

//-----  CONFIG2 Register  ---------//
#define CONFIG2_DEFAULT		0x00	// Configures the test signal generation. A priori no lo uso.

//-----  CONFIG3 Register  ---------//
#define CONFIG3_DEFAULT		0x4C	// Reference buffer = OFF, VREFP = 2.4V, RLDIN = open, RLDREF = internal, RLD buffer = ON
#define V_REF_2_4V			0x20	// Bit5 de CONFIG3 en 1 => VREFP en 2.4V
#define V_REF_4V			0x00	// Bit5 de CONFIG3 en 0 => VREFP en 4V

//-----  Comand definitions  ---------//
#define ADS_CMD_WAKEUP 0x02 /*! Wake-up from standby mode Opcode */
#define ADS_CMD_STANDBY 0x04 /*! Enter standby mode Opcode */
#define ADS_CMD_RESET 0x06 /*! Device registers Reset Opcode */
#define ADS_CMD_START 0x08 /*! Start/restart conversions Opcode */
#define ADS_CMD_STOP 0X0A /*! Stop conversions Opcode */
#define ADS_CMD_RDATAC 0X10 /*! Enable Read Data Continuous mode */
#define ADS_CMD_SDATAC 0x11 /*! Stop Read Data Continuously mode */
#define ADS_CMD_RDATA 0x12 /*! Read data by command */
#define ADS_CMD_RREG 0x20 /*! Read Configuration Registers */
#define ADS_CMD_WREG 0x40 /*! Write Configuration Registers */

//-----  CHnSET Register  ---------//
#define CHnSET_DEFAULT		0x10	// Channel n =  ON, PGA_GAIN = 1, ChannelMUX = Normal input
#define PD_CHANNEL			0x80	// Bit7 de CHnSET en 0 => Normal operation, en 1 => Channel power-down
#define PGA_GAIN_6			0
#define PGA_GAIN_1			1
#define PGA_GAIN_2			2
#define PGA_GAIN_3			3
#define PGA_GAIN_4			4
#define PGA_GAIN_8			5
#define PGA_GAIN_12			6
#define CH_MUX_NORMAL		0
#define CH_MUX_SHORT		1
#define CH_MUX_RLD_MES		2
#define CH_MUX_MVDD			3
#define CH_MUX_TEMP_SEN		4
#define CH_MUX_TEST_SIG		5
#define CH_MUX_RLD_DRP		6
#define CH_MUX_RLD_DRN		7

//-----  LOFF, RLD_SENSP, RLD_SENSN, LOFF_SENSP, LOFF_SENSN, LOFF_FLIP, PACE, WCT1, WCT2, Registers  ---------//
#define LOFF_DEFAULT 		0
#define RLD_SENSP_DEFAULT	0
#define RLD_SENSN_DEFAULT	0
#define LOFF_SENSP_DEFAULT	0
#define LOFF_SENSN_DEFAULT	0
#define LOFF_FLIP_DEFAULT 	0
#define PACE_DEFAULT		0
#define WCT1_DEFAULT		0
#define WCT2_DEFAULT		0
#define RESP_DEFAULT		0

//-----  GPIO Register  ---------//
#define GPIO_DEFAULT	0x0F	// Los 4 GPIO como entradas (conectar los pines a masa)
#define GPIO0_CONTROL	0		// Direccion del pin GPIOn 0 => Output, 1 => Input
#define GPIO1_CONTROL	1
#define GPIO2_CONTROL	2
#define GPIO3_CONTROL	3
#define GPIO0_DATA		0		// R/W del pin GPIOn
#define GPIO1_DATA		1
#define GPIO2_DATA		2
#define GPIO3_DATA		3

//-----  CONFIG4 Register  ---------//
#define CONFIG4_DEFAULT		0x00	// Continuous Conversion = ON, WCT to RLD connection = OFF, Lead-off comparators disabled
#define SINGLE_SHOT			0x08	// Bit3 de CONFIG4 en 1 => Single-shot mode, en 0 Continuous conversion mode

/*D Control Register - Address: 0x00 */
#define ADS1294 0b10010000 /*!< Revision ID for ADS1294 */


/* (1) Configuration Register 1 - Address: 0x01 */
#define M_HR 0b10000000 /*!< Mode High-Resolution */
#define M_LP 0b00000000 /*!< Mode Low-Power */
#define MULTI_READ 0b01000000 /*!< Multiple Read Back Mode */
#define CLK_EN 0b00100000 /*!< Oscil. clock output enable */
#define DR_HR_500 0b00000110 /*!< M. HR, Oversamp.: 500 SPS */
#define DR_HR_1K 0b00000101 /*!< M. HR, Oversamp.: 1K SPS */
#define DR_HR_2K 0b00000100 /*!< M. HR, Oversamp.: 2K SPS */
#define DR_HR_4K 0b00000011 /*!< M. HR, Oversamp.: 4K SPS */
#define DR_HR_8K 0b00000010 /*!< M. HR, Oversamp.: 8K SPS */
#define DR_HR_16K 0b00000001 /*!< M. HR, Oversamp.: 16K SPS */
#define DR_HR_32K 0b00000000 /*!< M. HR, Oversamp.: 32K SPS */
#define LP_HR_250 0b00000110 /*!< M. LP, Oversamp.: 250 SPS */
#define LP_HR_500 0b00000101 /*!< M. HR, Oversamp.: 500 SPS */
#define LP_HR_1K 0b00000100 /*!< M. HR, Oversamp.: 1K SPS */
#define LP_HR_2K 0b00000011 /*!< M. HR, Oversamp.: 2K SPS */
#define LP_HR_4K 0b00000010 /*!< M. HR, Oversamp.: 4K SPS */
#define LP_HR_8K 0b00000001 /*!< M. HR, Oversamp.: 8K SPS */
#define LP_HR_16K 0b00000000 /*!< M. HR, Oversamp.: 16K SPS */

/* (2) Configuration Register 2 - Adress: 0x02*/
#define INT_SIGNAL_TEST 0b00010000 /*!< Enable Internal Test Signal */
#define EXT_SIGNAL_TEST 0b00000000 /*!< Enable External Test Signal */
#define TEST_AMP_1 0b00000000 /*!< Test signal amplitud=1mV */
#define TEST_AMP_2 0b00000100 /*!< Test signal amplitud=2mV */
#define TEST_FREQ_1 0b00000000 /*!< Test signal frequency=1Hz */
#define TEST_FREQ_2 0b00000001 /*!< Test signal frecuency=2Hz */
#define TEST_FREQ_DC 0b00000011 /*!< Test signal DC */


/* (5,6,7,8) Channel 1,2,3,4 Settings - Address: 0x05, 0x06, 0x07, 0x08 */
#define POWER_DOWN 0b10000001
#define GAIN_6 0b00000000 /*!< Channel PGA gain 6 */
#define GAIN_1 0b00010000 /*!< Channel PGA gain 1 */
#define GAIN_2 0b00100000 /*!< Channel PGA gain 2 */
#define GAIN_3 0b00110000 /*!< Channel PGA gain 3 */
#define GAIN_4 0b01000000 /*!< Channel PGA gain 4 */
#define GAIN_8 0b01010000 /*!< Channel PGA gain 8 */
#define GAIN_12 0b01100000 /*!< Channel PGA gain 12 */
#define ELECTRODE 0b00000000 /*!< Channel Input: Electrode */
#define INPUT_SHORTED 0b00000001 /*!< Channel Input: Input shorted */
#define TEST_SIGNAL 0b00000101 /*!< Chanel Input: Test Signal */

* (3) Configuration Register 3 - Adress: 0x03 */
#define VREF_2V4 0b11000000 /*!< 2.4V reference */
#define VREF_4V 0b11100000 /*!< 4V reference */
#define RLD_EN 0b01001100 /*!< Enable RLD */
/* (4) Lead-Off Control Register - Adress: 0x04 */
#define LOFF_REG 0b00000000 /*!< Register State after reset */