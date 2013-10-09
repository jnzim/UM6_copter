
#include <avr/io.h>
#include <stdlib.h> 
#include "usart_driver.h"
#include "avr_compiler.h"
#include "communication.c"



/*! Define that selects the UASRT used in example. */
#define F_CPU 					32000000UL
#define XBEE_USART				USARTD1



#define MASK_TOP_BYTE			0x00FF


void init32MHzClock(void);
void initUART(void);
void put_USART_PC_char(uint8_t);
void sendData_int16_t(int16_t);

int16_t int16counter;




int main(void)
{

	PORTA.DIRSET = 0xFF;			//  LEDS
	init32MHzClock();
	initUART();
    spi_set_up();
	

		while(1)
		{
		 int16counter++;
		_delay_ms(200);
		//_delay_ms(200);
		 int16counter = 0;
		 //spi_master_get_32bit_reg(&spiMasterF,UM6_EULER_PSI);
		 ReadUM6DataReg();
		//sendData_int16_t(0xffff);
		 sendData_int16_t(Upper16bitWord);
		 //sendData_int16_t(Lower16bitWord);
		
		//put_USART_PC_char(byte1);
		//put_USART_PC_char(byte2);
		//put_USART_PC_char(byte3);
		//put_USART_PC_char(byte4);
		}		
				
		
		
		while(1)
		{}		 
}	
		

		


void sendData_int16_t(int16_t sendthis)
{
	put_USART_PC_char( MASK_TOP_BYTE & (sendthis >> 8));
	put_USART_PC_char (MASK_TOP_BYTE & sendthis);
}







uint8_t get_XBEE_USART_char()	
{
	uint16_t timeout = 1000;
	
	do
	{
		timeout--;
		
	}while(!USART_IsRXComplete(&XBEE_USART)  && timeout != 0);
	
	if (timeout != 0)
	{
		return USART_GetChar(&XBEE_USART);
	}
	else
	{
			return 0;		
	}
}


void put_USART_PC_char(uint8_t sendThis)
{

	do{

	}while(!USART_IsTXDataRegisterEmpty(&XBEE_USART));
	USART_PutChar(&XBEE_USART, sendThis);
}

void init32MHzClock(void)
{

	//	Enable the 32 MHz internal RC oscillator.
	OSC.CTRL|=OSC_RC32MEN_bm;
	//	The R32MRFY flag is set when the 32 MHz internal RC oscillator is stable
	//	and ready to be used as the System Clock source.
	while(!(OSC.STATUS &OSC_RC32MRDY_bm ));
	CCP=CCP_IOREG_gc;
	//	Select the 32MHz RC oscillator for the system clock
	CLK.CTRL = CLK_SCLKSEL_RC32M_gc;

	
	//  Use to output clock to a pin
	//  this must be done on port pin 7, see data sheet.
	//PORTD.DIRSET =  PIN7_bm;                   /* To output clock, port must be set as output */
	//PORTCFG.CLKEVOUT = PORTCFG_CLKOUT_PD7_gc; /* Output the clock frequency on PD7 to measure on Counter/Scope */
	
}

void initUART()
{
	
	//  XBEE on USARTD1
	/* PD6 (RXD1) input*/
	PORTD.DIRCLR = PIN6_bm;
	/* PD7 (TXD1) as output. */
	PORTD.DIRSET = PIN7_bm;
	
	
	//  IMU on USARTC1	
	/* PC6 (RXD1) input*/
	PORTC.DIRCLR = PIN6_bm;
	/* PD7 (TXD1) as output. */
	PORTC.DIRSET = PIN7_bm;
	
	USART_Format_Set(&XBEE_USART, USART_CHSIZE_8BIT_gc, USART_PMODE_DISABLED_gc, true);
	USART_Baudrate_Set(&XBEE_USART, 1047 , -6);			// set for 32MHZ and 115200
	/* Enable both RX and TX. */
	USART_Rx_Enable(&XBEE_USART);
	USART_Tx_Enable(&XBEE_USART);
	

}



/*! \brief This function configures the necessary registers for a block transfer.
 *
 *  \note The transfer must be manually triggered or a trigger source
 *        selected before the transfer starts. It is possible to reload the
 *        source and/or destination address after each data transfer, block
 *        transfer or only when the entire transfer is complete.
 *        Do not change these settings after a transfer has started.
 *
 *  \param  channel        The channel to configure.
 *  \param  srcAddr        Source memory address.
 *  \param  srcReload      Source address reload mode.
 *  \param  srcDirection   Source address direction (fixed, increment, or decrement).
 *  \param  destAddr       Destination memory address.
 *  \param  destReload     Destination address reload mode.
 *  \param  destDirection  Destination address direction (fixed, increment, or decrement).
 *  \param  blockSize      Block size in number of bytes (0 = 64k).
 *  \param  burstMode      Number of bytes per data transfer (1, 2, 4, or 8 bytes).
 *  \param  repeatCount    Number of blocks, 0x00 if you want to repeat at infinitum.
 *  \param  useRepeat      True if repeat should be used, false if not.
 */





