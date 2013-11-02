/*
 * communication.c
 *
 * Created: 10/3/2013 7:55:00 PM
 *  Author: Justin
 */ 


#include "avr_compiler.h"
#include "spi_driver.h"
#include "spi_driver.c"





/*  IMU commands */
/*! \brief The number of test data bytes. */
#define NUM_BYTES				2

#define READ_COMMAND			0x00
#define DUMMY_READ				0x00
#define WRITE_COMMAND			0x01
#define READ_UM6_SW_REV			0xAA
#define UM6_GYRO_PROC_XY		0x5C
#define UM6_GYRO_PROC_Z			0x5D
#define UM6_EULER_PHI_THETA		0x62		// returns roll and pitch as 16 bit signed integers
#define UM6_EULER_PSI			0x63
#define ZERO_GYROS				0xAC
#define SET_ACCEL_REF			0xAF
#define SET_MAG_REF				0xB0
#define RESET_TO_FACTORY		0xB1
#define UM6_MAG_RAW_XY			0x5A
#define UM6_MAG_RAW_Z			0x5B



void spi_set_up(void);
void spi_master_write_byte(SPI_Master_t *,uint8_t);
uint8_t spi_master_recive_byte(SPI_Master_t *);



/* Global variables */
 SPI_Master_t spiMasterF;
  SPI_Master_t spiMasterE;

/* Instantiate pointer to ssPort. */
PORT_t *ssPortF = &PORTF;
PORT_t *ssPortE = &PORTE;

/*! \brief Test data to send from master. */
//uint8_t masterSendData[NUM_BYTES] = {READ_COMMAND, UM6_EULER_PSI};

/*! \brief Data received from slave. */
//uint8_t masterReceivedData[NUM_BYTES];

/* global angles */
uint16_t Upper16bitWord, Lower16bitWord, UM6_EulerYaw, UM6_EulerRoll,UM6_EulerPitch;
uint8_t byte1= 0xFF, byte2= 0xFF, byte3= 0xFF, byte4 = 0xFF;

int16_t int16spiData = 0x00;
/*! \brief Result of the test. */
bool success = true;

/* 
	PF4 SS
	PF5 MOSI
	PF6	MISO
	PF7	SCK
*/
	
void spi_set_up()
{
	/* Initialize SPI master on port F for the IMU communication. */
	/* Init SS pin as output with wired AND and pull-up. */
	//*************************************************************
	PORTF.DIRSET = PIN4_bm;
	PORTF.PIN4CTRL = PORT_OPC_WIREDANDPULL_gc;

	/* Set SS output to high. (No slave addressed). */
	PORTF.OUTSET = PIN4_bm;

	/* Initialize SPI master on port F for the IMU communication. */
	SPI_MasterInit(&spiMasterF,
		&SPIF,
		&PORTF,
		false,
		SPI_MODE_0_gc,							//The UM6 SPI clock (SCK) is active high, with data clocked in on the first rising edge1
		SPI_INTLVL_OFF_gc,
		false,									// false to double clock mode
		SPI_PRESCALER_DIV64_gc);				//32mHz /128 = 250kHz  MAX rate is 400kHz but there is not prescaler

		
		
		
		/* Initialize SPI master on port F for the PC communication */
		//*************************************************************
		/* Init SS pin as output with wired AND and pull-up. */
		PORTE.DIRSET = PIN4_bm;
		PORTE.PIN4CTRL = PORT_OPC_WIREDANDPULL_gc;

		/* Set SS output to high. (No slave addressed). */
		PORTE.OUTSET = PIN4_bm;

		SPI_MasterInit(&spiMasterE,
		&SPIE,
		&PORTE,
		false,
		SPI_MODE_0_gc,							//The UM6 SPI clock (SCK) is active high, with data clocked in on the first rising edge1
		SPI_INTLVL_OFF_gc,
		false,									// false to double clock mode
		SPI_PRESCALER_DIV64_gc);				//32mHz /128 = 250kHz  MAX rate is 400kHz but there is not prescaler

}


// SPI write read function
// Load the register, this will start a transfer on MOSI
//  Wait until the intrupt flag is set
//  read the data from the data register, this was on MISO
unsigned char spi_write_read(unsigned char spi_data)
{
	SPIF.DATA = spi_data;
	while(!(SPIF.STATUS & SPI_IF_bm)); // Wait until the data transfer is complete
	return SPIF.DATA;
}


// SPI write read function
// Load the register, this will start a transfer on MOSI
//  Wait until the intrupt flag is set
//  read the data from the data register, this was on MISO
unsigned char spiPC_write_read(unsigned char spi_data)
{
	SPIE.DATA = spi_data;
	while(!(SPIE.STATUS & SPI_IF_bm)); // Wait until the data transfer is complete
	return SPIE.DATA;
}


void ReadUM6DataReg()
{
	
	
	PORTF.OUTCLR = PIN4_bm;	

	uint8_t dummy_read;
	
	dummy_read = spi_write_read(0x00);
	dummy_read = spi_write_read(0xAA);

	 //MSB first
	Upper16bitWord =(Upper16bitWord << 8 ) +  spi_write_read(DUMMY_READ);
	Upper16bitWord =(Upper16bitWord << 8 ) +  spi_write_read(DUMMY_READ);
	Lower16bitWord =(Lower16bitWord << 8 ) +  spi_write_read(DUMMY_READ);
	Lower16bitWord =(Lower16bitWord << 8 ) +  spi_write_read(DUMMY_READ);

	PORTF.OUTSET = PIN4_bm;	
		
}

int32_t Read_UM6_SW_Version()
{

	uint32_t SW_version;
	uint8_t dummy_read;
	PORTF.OUTCLR = PIN4_bm;
	dummy_read = spi_write_read(WRITE_COMMAND);
	dummy_read = spi_write_read(READ_UM6_SW_REV);

	//MSB first
	SW_version =(SW_version << 8 ) +  spi_write_read(DUMMY_READ);
	SW_version =(SW_version << 8 ) +  spi_write_read(DUMMY_READ);
	SW_version =(SW_version << 8 ) +  spi_write_read(DUMMY_READ);
	SW_version =(SW_version << 8 ) +  spi_write_read(DUMMY_READ);

	PORTF.OUTSET = PIN4_bm;
	return SW_version;	
}




uint8_t spi_master_recive_byte(SPI_Master_t *spi)
{
	//SPI_MasterSSLow(ssPort, PIN4_bm);
	/* Read received data. */
	uint8_t result = spi->module->DATA;
	return result;
	//SPI_MasterSSHigh(ssPort, PIN4_bm);
}



void spi_master_get_32bit_reg(SPI_Master_t *spi, uint8_t regToRead)
{
	
		//  set the slave select low for the transmission and receive
		SPI_MasterSSLow(ssPortE, PIN4_bm);

		/* Send pattern.  Placing data in the data register initiates a transfer.
		0x00 is the read command, it should be followed by the register address you want to read */
		spi->module->DATA = 0x00;
		
		/* Wait for transmission complete by checking the interrupt flag */
		while(!(spi->module->STATUS & SPI_IF_bm)) {}
		
		//  Next send the address of the register we want to read
		spi->module->DATA = regToRead;
		
		//  wait for the transfer to complete
		while(!(spi->module->STATUS & SPI_IF_bm))    {}
		
		//  all UM6 registers are 32 bit values, see data sheet for data type and how they should be parsed
		Upper16bitWord =(Upper16bitWord << 8 ) + spi->module->DATA;
		spi->module->DATA = 0x00;		while(!(spi->module->STATUS & SPI_IF_bm)) {}
			
		Upper16bitWord =(Upper16bitWord << 8 ) + spi->module->DATA;
		spi->module->DATA = 0x00;		while(!(spi->module->STATUS & SPI_IF_bm)) {}

		Lower16bitWord =(Lower16bitWord << 8 ) + spi->module->DATA;
		spi->module->DATA = 0x00;		while(!(spi->module->STATUS & SPI_IF_bm)) {}
			
		Lower16bitWord =(Lower16bitWord << 8 ) + spi->module->DATA;
		
		SPI_MasterSSHigh(ssPortE, PIN4_bm);
		
		////  convert form 2's complement
		//Upper16bitWord = (Upper16bitWord ^ 0xFFFF) + 1;
		//Lower16bitWord = (Lower16bitWord ^ 0xFFFF) + 1;
}






void ZeroGyros()
{
	uint8_t dummy_read;
	//psi = yaw  phi = roll    theta = pitch
	dummy_read = spi_write_read(WRITE_COMMAND);
	dummy_read = spi_write_read(ZERO_GYROS);
	
	
}

void ZeroAccelerometers()
{
	uint8_t dummy_read;

	dummy_read = spi_write_read(WRITE_COMMAND);
	dummy_read = spi_write_read(SET_ACCEL_REF);
	
	
}

void ZeroMagnetometer()
{
	
	uint8_t dummy_read;

	dummy_read = spi_write_read(WRITE_COMMAND);
	dummy_read = spi_write_read(SET_MAG_REF);
		
}	


void ResetUM6_ToFactory()
{
	
	uint8_t dummy_read;

	dummy_read = spi_write_read(WRITE_COMMAND);
	dummy_read = spi_write_read(RESET_TO_FACTORY);
	
}