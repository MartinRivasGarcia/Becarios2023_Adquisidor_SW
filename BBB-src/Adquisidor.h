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




