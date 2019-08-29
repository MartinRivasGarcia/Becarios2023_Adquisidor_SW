/**
 *	ADS1294.h
 * 	Autor: Nicolas Sidorczuk
 * 
 * 	Definiciones de Comandos y Registros para el ADS1294
 */

#ifndef ADS1294_H_
#define ADS1294_H_

//-----  Opcode Commands  ------//
#define ADS1294_WAKEUP 	0x02	// Wakeup from standby mode
#define ADS1294_STANDBY 0x04 	// Enter standby mode
#define ADS1294_RESET 	0x06 	// Reset the device
#define ADS1294_START 	0x08 	// Start/restart (synchronize) conversions
#define ADS1294_STOP 	0x0A 	// Stop conversion 
#define ADS1294_RDATAC 	0x10	// Enable Read Data Continuous mode. Default mode at power up
#define ADS1294_SDATAC 	0x11	// Stop Read Data Continuously mode 
#define ADS1294_RDATA 	0x12	// Read data by command; supports multiple read back
#define ADS1294_RREG_1	0x20 	// First byte: 001r rrrr. 	Read n nnnn registers starting at address r rrrr 
#define ADS1294_RREG_2 	0x00	// Second byte: 000n nnnn. 	n nnnn = number of registers to be read/written – 1
#define ADS1294_WREG_1 	0x40 	// First byte: 010r rrrr. 	Read n nnnn registers starting at address r rrrr 
#define ADS1294_WREG_2 	0x00	// Second byte: 000n nnnn.	n nnnn = number of registers to be read/written – 1


//-----  Register Map  ---------//
#define ADS1294_ID			0x00	// Read-only ID control register, programmed during manufacture to indicate device characteristics.
#define ADS1294_CONFIG1		0x01 	// Configuration Register 1
#define ADS1294_CONFIG2		0x02 	// Configuration register 2 configures the test signal generation
#define ADS1294_CONFIG3		0x03 	// Configuration Register 3 configures multireference and RLD operation.
#define ADS1294_LOFF		0x04 	// The lead-off control register configures the lead-off detection operation.
#define ADS1294_CH1SET		0x05	// The CH[1:8]SET control register configures the power mode, PGA gain, and multiplexer settings channels
#define ADS1294_CH2SET		0x06	// IDEM
#define ADS1294_CH3SET		0x07	// IDEM
#define ADS1294_CH4SET		0x08	// IDEM
// #define ADS1294_CH5SET	0x09 	// Solo tengo 4 canales
// #define ADS1294_CH6SET	0x0A	// IDEM
// #define ADS1294_CH7SET	0x0B	// IDEM
// #define ADS1294_CH8SET	0x0C	// IDEM
#define ADS1294_RLD_SENSP	0x0D	// Selection of the positive signals from each channel for right leg drive (RLD) derivation
#define ADS1294_RLD_SENSN	0x0E	// Selection of the negative signals from each channel for right leg drive (RLD) derivation
#define ADS1294_LOFF_SENSP	0x0F	// Selects the positive side from each channel for lead-off detection
#define ADS1294_LOFF_SENSN	0x10 	// Selects the negative side from each channel for lead-off detection
#define ADS1294_LOFF_FLIP	0x11	// Controls the direction of the current used for lead-off derivation
#define ADS1294_LOFF_STATP	0x12	// Read only. Stores the status of whether the positive electrode on each channel is on or off
#define ADS1294_LOFF_STATN	0x13	// Read only. Stores the status of whether the negative electrode on each channel is on or off
#define ADS1294_GPIO		0x14	// Controls the action of the ¿three? GPIO pins
#define ADS1294_PACE		0x15	// Pace controls that configure the channel signal used to feed the external pace detect circuitry
#define ADS1294_RESP		0x16	// Provides the controls for the respiration circuitry
#define ADS1294_CONFIG4		0x17	// Configuration Register 4
#define ADS1294_WCT1		0x18	// Configures the device WCT circuit channel selection and the augmented leads
#define ADS1294_WCT2		0x19	// Configures the device WCT circuit channel selection.

#endif /* ADS1294_H_ */