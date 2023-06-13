//Modificacion de Adquisidor.c basandose en pfc.pdf -> faltaria ver para habilitar las interrupciones del spi
/**
 *	Adquisidor.c
 * 	Autor: Nicolas Sidorczuk
 * 
 * % AÑADIR DESCRIPCION %
 */


#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include "Adquisidor.h"
#include "SPI.h"

int DRDYdir, DRDYvalue, DRDYedge;
uint8_t ADS1294_Status[3];
uint8_t ADS1294_transfer_length = 15; // Por defecto elijo 16KSPS => 15 bytes

#define SPI_TL_DEBUG 64

int32_t main(int32_t argc, int8_t const *argv[])
{
	int i=0;
    unsigned char Rx_aux = 0;
    uint8_t RxBuffer[16];
    uint32_t ChannelData[4];

	if ( Config_BBB_Pins() < 0)	//Configuro los pines de la BBB para SPI y para DRDY
		return 0;

	// Intento tomar el driver de SPI de la beagle
	if( SPI_DEV1_init(SPIDEV_BYTES_NUM, SPIDEV1_BUS_SPEED_HZ, SPI_SS_LOW, SPIDEV_DELAY_US, SPIDEV_DATA_BITS_NUM, SPI_MODE1set) == -1 )
	{
		printf("Inicializacion de spidev1.0 fallida\n");
		return 0;
	}
	printf("Inicializacion de spidev1.0 exitosa\n");
    

    //ADS1294_SendCommand(ADS1294_SDATAC);
    
	ADS1294_init();	// Inicializo los registros del ADS1294 con sus valores por defecto
   
    //ADS1294_SendCommand(ADS1294_SDATAC);

    Rx_aux = ADS1294_ReadRegister(ADS1294_CONFIG3);
    printf("Valor de Registro CONFIG1: %u\n",Rx_aux);
    ADS1294_WriteRegister(ADS1294_CONFIG4,CONFIG4_DEFAULT);
    //Rx_aux = ADS1294_ReadRegister(ADS1294_CONFIG1);
    //printf("Valor de Registro CONFIG1: %u\n",Rx_aux);    /**/

    //ADS1294_Set_DataRate(OUTPUT_DR_8K);

	ADS1294_SendCommand(ADS1294_RDATAC);
	ADS1294_SendCommand(ADS1294_START);	// Requiere que el pin START este en estado bajo
	usleep(100);
	ADS1294_SingleRead(RxBuffer);
	
	printf("Trama recibida: ");
	/*for(i = 1; i < 32; i++)
		printf("%u ", RxBuffer[i]); // %#X
	printf("\n");*/

	for(i = 1; i < 16; i+=3)
		printf("0x%06X ",(int32_t)((RxBuffer[i]<<16) | (RxBuffer[i+1]<<8) | (RxBuffer[i+2])) );
	printf("\n");

	ADS1294_Status[0] = RxBuffer[1];
	ADS1294_Status[1] = RxBuffer[2];
	ADS1294_Status[2] = RxBuffer[3];

	ChannelData[0] = (int32_t)((RxBuffer[4]<<16) | (RxBuffer[5]<<8) | (RxBuffer[6]));
	printf("Canal 1: %d\n", ChannelData[0]);

	ChannelData[1] = (int32_t)((RxBuffer[7]<<16) | (RxBuffer[8]<<8) | (RxBuffer[9]));
	printf("Canal 2: %d\n", ChannelData[1]);

	ChannelData[2] = (int32_t)((RxBuffer[10]<<16) | (RxBuffer[11]<<8) | (RxBuffer[12]));
	printf("Canal 3: %d\n", ChannelData[2]);

	ChannelData[3] = (int32_t)((RxBuffer[13]<<16) | (RxBuffer[14]<<8) | (RxBuffer[15]));
	printf("Canal 4: %d\n", ChannelData[3]);
	//Leer();	// Reubicar

	close(DRDYdir);
	close(DRDYvalue);
	return 0;
}

/**
 * @brief      Utilizo config-pin para elegir la funcionalidad de los pines de la BBB como SPI. Ademas configuro un GPIO para DRDY.
 *
 * @return     Devuelve -1 en caso de error. Devuelve 0 en configuracion exitosa.
 */
int32_t Config_BBB_Pins(void)
{
	/* Pines para SPI */
	if( system("config-pin P9.17 spi_cs") < 0 )
	{
		printf("Error corriendo en consola el comando: config-pin P9.17 spi_cs\n");
		return -1;
	}
	if( system("config-pin P9.18 spi") < 0 )
	{
		printf("Error corriendo en consola el comando: config-pin P9.18 spi\n");
		return -1;
	}
	if( system("config-pin P9.21 spi") < 0 )
	{
		printf("Error corriendo en consola el comando: config-pin P9.21 spi\n");
		return -1;
	}
	if( system("config-pin P9.22 spi_sclk") < 0 )
	{
		printf("Error corriendo en consola el comando: config-pin P9.22 spi_sclk\n");
		return -1;
	}

	/* Pin GPIO para DRDY, DataReady */
	DRDYdir = open("/sys/class/gpio/gpio49/direction",O_RDWR);
	if(DRDYdir < 0)
	{
		printf("Error abirendo gpio49 direction\n");
		return -1;
	}
	if( write(DRDYdir,"in",3) < 0 )
	{
		printf("Error escribiendo gpio49 direction\n");
		close(DRDYdir);
		return -1;
	}
	DRDYvalue = open("/sys/class/gpio/gpio49/value",O_RDWR);
	if(DRDYvalue < 0)
	{
		printf("Error abirendo gpio49 value\n");
		close(DRDYdir);
		return -1;
	}
	return 0;
}

/**
 * @brief      Selecciona la tasa de muestras que quiero del ADC, cambiando el campo DR del registro CONFIG1
 *
 * @param[in]  DR    Indica la nueva tasa de muestras deseada
 */
void ADS1294_Set_DataRate(uint8_t DR)
{
	uint8_t config = CONFIG1_DEFAULT;
	config = (config & ~7) | DR;
	ADS1294_WriteRegister(ADS1294_CONFIG1,config);
	if(DR == OUTPUT_DR_32K)
		ADS1294_transfer_length = 11; // Para 32KSPS el ADC manda 16bits por canal
	else
		ADS1294_transfer_length = 15; // Para tasas menores el ADC manda 24bits por canal
}


/**
 * @brief      Lee la nueva conversion del ADC. Usar en modo Read Data Continuously mode (RDATAC)
 *
 * @param      Rx    Buffer donde se carga la lectura del ADC
 */
void ADS1294_Read(uint8_t * Rx)
{
	/* Siempre el ADC envia primero 24bits de STATUS. Luego las muestras de cada canal de forma consecutiva.
	 * Segun datasheet hoja 53: para 64 y 32 KSPS -> 16 bits/canal, de 8KSPS para abajo -> 24 bits/canal */
	uint8_t TxDummy_LDR[11] = { 0 };	// Leo directamente con la transferencia SPI, y el ADC indica mantener MOSI en bajo
	uint8_t TxDummy_HDR[15] = { 0 };	// Leo directamente con la transferencia SPI, y el ADC indica mantener MOSI en bajo
	uint8_t DRDY;

	if( read(DRDYvalue,&DRDY,1) < 0)
	{
		printf("Error leyendo DRDY\n");
		return;
	}	

	if(DRDY == DATAREADY)
	{
		if(ADS1294_transfer_length == 15)
		{
			if ( SPIDEV1_transfer(TxDummy_HDR, Rx, ADS1294_transfer_length) == 0 )
    			printf("(ADS1294_Read)spidev1.0: Transaction Complete\r\n");
    		else
    			printf("(ADS1294_Read)spidev1.0: Transaction Failed\r\n");
		}
		else if(ADS1294_transfer_length == 11)
			if ( SPIDEV1_transfer(TxDummy_LDR, Rx, ADS1294_transfer_length) == 0 )
    			printf("(ADS1294_Read)spidev1.0: Transaction Complete\r\n");
    		else
    			printf("(ADS1294_Read)spidev1.0: Transaction Failed\r\n");
	}
}

/**
 * @brief      Realiza una lectura de datos al ADC de forma manual
 *
 * @param      Rx    Buffer donde se almacena la lectura del ADC
 */
void ADS1294_SingleRead(uint8_t * Rx)
{
	// El ADC indica mantener MOSI en bajo mientras se leen datos => transmito todos ceros.
	uint8_t TxDummy_LDR[12] = { 0 }; //Buffer para low data rate
	uint8_t TxDummy_HDR[16] = { 0 }; //Buffer para high data rate
	uint8_t DRDY;
	TxDummy_LDR[0] = ADS1294_RDATA;	// El primer byte es el comando de lectura
	TxDummy_HDR[0] = ADS1294_RDATA;	// El primer byte es el comando de lectura
	
	if( read(DRDYvalue,&DRDY,1) < 0)
	{
		printf("Error leyendo DRDY\n");
		return;
	}
	printf("DRDY: %u\n",DRDY);
	if(DRDY == DATAREADY)
	{
		if(ADS1294_transfer_length == 15)
		{
			if ( SPIDEV1_transfer(TxDummy_HDR, Rx, ADS1294_transfer_length + 1) == 0 )
    			printf("(ADS1294_SingleRead)spidev1.0: Transaction Complete\r\n");
    		else
    			printf("(ADS1294_SingleRead)spidev1.0: Transaction Failed\r\n");
		}
		else if(ADS1294_transfer_length == 11)
			if ( SPIDEV1_transfer(TxDummy_LDR, Rx, ADS1294_transfer_length + 1) == 0 )
    			printf("(ADS1294_SingleRead)spidev1.0: Transaction Complete\r\n");
    		else
    			printf("(ADS1294_SingleRead)spidev1.0: Transaction Failed\r\n");
	}
}

/**
 * @brief      Envia un comando al ADS1294 de un byte
 *
 * @param[in]  Opcode  Comando a enviar
 */
void ADS1294_SendCommand(uint8_t Opcode)
{
	if ( SPIDEV1_single_transfer(Opcode) >= 0 )
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
    unsigned char Tx[3];
	uint8_t NumberOfBytes = 3;
	Tx[0] = ADS1294_WREG_1 | Reg;		// Agrego direccion del registro en el primer opcode
	Tx[1] = ADS1294_WREG_2 | (1 - 1);	// Agrego cantidad de registros a escribir en el segundo opcode
	Tx[2] = valor;						// Contenido a escribir al registro
	if ( SPIDEV1_transfer(Tx, NULL, NumberOfBytes) == 0 )
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
    unsigned char Tx[3];
    unsigned char Rx[3];
	uint8_t NumberOfBytes = 3;
	Tx[0] = ADS1294_RREG_1 | Reg;		// Agrego direccion del registro en el primer opcode
	Tx[1] = ADS1294_RREG_2 | (1 - 1);	// Agrego cantidad de registros a escribir en el segundo opcode
	Tx[2] = 0;		// El tercer byte es para leer el valor del registro. Keep DIN low for the entire read operation
	if ( SPIDEV1_transfer(Tx, Rx, NumberOfBytes) == 0 )
    	printf("(ADS1294_ReadRegister)spidev1.0: Transaction Complete\r\n");
    else
    	printf("(ADS1294_ReadRegister)spidev1.0: Transaction Failed\r\n");
    return Rx[2];
}

/**
 * @brief      Inicializa todos los registros (excepto los Read Only) del ADS1294 con la configuracion inicial deseada
 */
void ADS1294_init(void)
{
	uint8_t NumberOfBytes = 0;
    uint8_t Txa[10];
    uint8_t Txb[7];
    uint8_t Txc[8];

	ADS1294_SendCommand(ADS1294_RESET);	// Primero reseteo el ADC
	usleep(100);						// Despues de un comando de reset hay que esperar aprox 9us (18 tCLK)


    ADS1294_SendCommand(ADS1294_SDATAC); //Paro el modo continuo de conversion que es el default del ADC

	ADS1294_SendCommand(ADS_CMD_WREG);
	ADS1294_SendCommand(8);
	ADS1294_SendCommand(ADS1294);		/*Id control*/

	ADS1294_SendCommand(M_HR | DR_HR_1K); /* Configuration 1 */
	ADS1294_SendCommand(EXT_SIGNAL_TEST | TEST_FREQ_2 | TEST_AMP_1 ); /* Configuration 2 */
	ADS1294_SendCommand(VREF_2V4 | RLD_EN); /* Configuration 3 */
	ADS1294_SendCommand(LOFF_REG); /* Lead-off Control */
	ADS1294_SendCommand(ELECTRODE | GAIN_12); /* Channel 1 Settings */
	ADS1294_SendCommand(ELECTRODE | GAIN_6); /* Channel 2 Settings */
	ADS1294_SendCommand(ELECTRODE | GAIN_12); /* Channel 3 Settings */
	ADS1294_SendCommand(POWER_DOWN | INPUT_SHORTED); /* Channel 4 Settings */

}

