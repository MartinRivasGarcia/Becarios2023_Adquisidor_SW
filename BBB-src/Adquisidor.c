/**
 *	Adquisidor.c
 * 	Autor: Nicolas Sidorczuk
 * 
 * % AÑADIR DESCRIPCION %
 */


#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include "Adquisidor.h"
#include "SPI.h"

uint8_t Tx_spi[SPIDEV_BYTES_NUM];
uint8_t Rx_spi[SPIDEV_BYTES_NUM];
uint32_t ADS1294_Status;

int32_t main(int32_t argc, int8_t const *argv[])
{
	
	// Intento tomar el driver de SPI de la beagle
	if( SPI_DEV1_init(SPIDEV_BYTES_NUM, SPIDEV1_BUS_SPEED_HZ, SPI_SS_LOW, SPIDEV_DELAY_US, SPIDEV_DATA_BITS_NUM, SPI_MODE1set) == -1 )
	{
		printf("Inicializacion de spidev1.0 fallida\n");
		return 0;
	}
	printf("Inicializacion de spidev1.0 exitosa\n");

	ADS1294_init();	// Inicializo los registros del ADS1294 con sus valores por defecto

    
	//ADS1294_SendCommand(ADS1294_START);	// Requiere que el pin START este en estado bajo

	//Leer();	// Reubicar

	return 0;
}


void ADS1294_Read(uint8_t * Rx, uint8_t NumberOfBytes)
{
	uint8_t TxDummy[15] = { 0 };	// Leo directamente con la transferencia SPI, y el ADC indica mantener MOSI en bajo

	//# TODO: Leer pin de DRDY #//

	//if(DRDY == LOW)
	//{
		if ( SPIDEV1_transfer(TxDummy, Rx, NumberOfBytes) == 0 )
    		printf("(ADS1294_WriteRegister)spidev1.0: Transaction Complete\r\n");
    	else
    		printf("(ADS1294_WriteRegister)spidev1.0: Transaction Failed\r\n");
	//}
}

void ADS1294_SingleRead(uint8_t * Rx, uint8_t NumberOfBytes)
{
	uint8_t TxDummy[16] = { 0 };	// El ADC indica mantener MOSI en bajo mientras se leen datos
	TxDummy[0] = ADS1294_RDATA;	// El primer byte es el comando de lectura
	//# TODO: Leer pin de DRDY #//

	//if(DRDY == LOW)
	//{
		if ( SPIDEV1_transfer(TxDummy, Rx, NumberOfBytes + 1) == 0 )
    		printf("(ADS1294_WriteRegister)spidev1.0: Transaction Complete\r\n");
    	else
    		printf("(ADS1294_WriteRegister)spidev1.0: Transaction Failed\r\n");
	//}
}

/**
 * @brief      Envia un comando al ADS1294 de un byte
 *
 * @param[in]  Opcode  Comando a enviar
 */
void ADS1294_SendCommand(uint8_t Opcode)
{
	if ( SPIDEV1_single_transfer(Opcode) == 0 )
    	printf("(ADS1294_command)spidev1.0: Transaction Complete\r\n");
    else
    	printf("(ADS1294_command)spidev1.0: Transaction Failed\r\n");
}

/**
 * @brief      Escribe un registro determinado del ADS1294
 *
 * @param[in]  Reg    Direccion del registro a escribir
 * @param[in]  valor  El valor a escribir en el registro indicado
 */
void ADS1294_WriteRegister(uint8_t Reg, uint8_t valor)
{
	uint8_t NumberOfBytes = 3;
	Tx_spi[0] = ADS1294_WREG_1 | Reg;		// Agrego direccion del registro en el primer opcode
	Tx_spi[1] = ADS1294_WREG_2 | (1 - 1);	// Agrego cantidad de registros a escribir en el segundo opcode
	Tx_spi[2] = valor;						// Contenido a escribir al registro
	if ( SPIDEV1_transfer(Tx_spi, NULL, NumberOfBytes) == 0 )
    	printf("(ADS1294_WriteRegister)spidev1.0: Transaction Complete\r\n");
    else
    	printf("(ADS1294_WriteRegister)spidev1.0: Transaction Failed\r\n");
}

/**
 * @brief      Lee el contenido de un registro del ADS1294
 *
 * @param[in]  Reg   Direccion del registro a leer
 *
 * @return     Devuelve el contenido del registro indicado
 */
uint8_t ADS1294_ReadRegister(uint8_t Reg)
{
	uint8_t NumberOfBytes = 3;
	Tx_spi[0] = ADS1294_RREG_1 | Reg;		// Agrego direccion del registro en el primer opcode
	Tx_spi[1] = ADS1294_RREG_2 | (1 - 1);	// Agrego cantidad de registros a escribir en el segundo opcode
	Tx_spi[2] = 0;		// El tercer byte es para leer el valor del registro. Keep DIN low for the entire read operation
	if ( SPIDEV1_transfer(Tx_spi, Rx_spi, NumberOfBytes) == 0 )
    	printf("(ADS1294_ReadRegister)spidev1.0: Transaction Complete\r\n");
    else
    	printf("(ADS1294_ReadRegister)spidev1.0: Transaction Failed\r\n");
    return Rx_spi[2];
}

/**
 * @brief      Inicializa todos los registros (excepto los Read Only) del ADS1294 con la configuracion inicial deseada
 */
void ADS1294_init(void)
{
	uint8_t NumberOfBytes = 0;

	ADS1294_SendCommand(ADS1294_RESET);	// Primero reseteo el ADC
	usleep(100);						// Despues de un comando de reset hay que esperar aprox 9us (18 tCLK)

	// Inicialmente solo escribo los registros y no leo nada. Chequear que anda con NULL (deberia)
	// Necesito 10 bytes
	NumberOfBytes = 10;	// = Cantidad De Registros + 2 (el comando WriteRegister requiere 2 bytes)
	Tx_spi[0] = ADS1294_WREG_1 | ADS1294_CONFIG1;	// Escribo registros a partir del registro CONFIG1
	Tx_spi[1] = ADS1294_WREG_2 | (8 - 1);	// Voy a escribir 8 registros
	Tx_spi[2] = CONFIG1_DEFAULT;
	Tx_spi[3] = CONFIG2_DEFAULT;
	Tx_spi[4] = CONFIG3_DEFAULT;
	Tx_spi[5] = LOFF_DEFAULT;
	Tx_spi[6] = CHnSET_DEFAULT;	// Inicialmente configuro por igual los 4 canales
	Tx_spi[7] = CHnSET_DEFAULT;
	Tx_spi[8] = CHnSET_DEFAULT;
	Tx_spi[9] = CHnSET_DEFAULT;
	if ( SPIDEV1_transfer(Tx_spi, NULL, NumberOfBytes) == 0 )
    	printf("(ADS1294_init)spidev1.0: Transaction 1 Complete\r\n");
    else
    	printf("(ADS1294_init)spidev1.0: Transaction 1 Failed\r\n");

    NumberOfBytes = 7;	// = Cantidad De Registros + 2 (el comando WriteRegister requiere 2 bytes)
	Tx_spi[0] = ADS1294_WREG_1 | ADS1294_RLD_SENSP;	// Escribo registros a partir del registro RLD_SENSP
	Tx_spi[1] = ADS1294_WREG_2 | (5 - 1);	// Voy a escribir 5 registros
	Tx_spi[2] = RLD_SENSP_DEFAULT;
	Tx_spi[3] = RLD_SENSN_DEFAULT;
	Tx_spi[4] = LOFF_SENSP_DEFAULT;
	Tx_spi[5] = LOFF_SENSN_DEFAULT;
	Tx_spi[6] = LOFF_FLIP_DEFAULT;	// Inicialmente configuro los 4 canales por igual
	if ( SPIDEV1_transfer(Tx_spi, NULL, NumberOfBytes) == 0 )
    	printf("(ADS1294_init)spidev1.0: Transaction 2 Complete\r\n");
    else
    	printf("(ADS1294_init)spidev1.0: Transaction 2 Failed\r\n");

    NumberOfBytes = 8;	// = Cantidad De Registros + 2 (el comando WriteRegister requiere 2 bytes)
	Tx_spi[0] = ADS1294_WREG_1 | ADS1294_GPIO;	// Escribo registros a partir del registro GPIO
	Tx_spi[1] = ADS1294_WREG_2 | (6 - 1);	// Voy a escribir 6 registros
	Tx_spi[2] = GPIO_DEFAULT;
	Tx_spi[3] = PACE_DEFAULT;
	Tx_spi[4] = RESP_DEFAULT;
	Tx_spi[5] = CONFIG4_DEFAULT;
	Tx_spi[6] = WCT1_DEFAULT;
	Tx_spi[7] = WCT2_DEFAULT;
	if ( SPIDEV1_transfer(Tx_spi, NULL, NumberOfBytes) == 0 )
    	printf("(ADS1294_init)spidev1.0: Transaction 3 Complete\r\n");
    else
    	printf("(ADS1294_init)spidev1.0: Transaction 3 Failed\r\n");
}

