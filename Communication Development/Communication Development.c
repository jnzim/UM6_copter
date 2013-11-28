/*
 * Communication_Development.c
 *  
 * Created: 11/13/2013 7:11:08 PM
 *  Author: Justin
 *
 *
 *  CREATED THIS RPROJECT TO DEBUG/DEVLOPE COMMUNCATION BETWEEN IMU, ATMEGA328P, AND PC
 *
 *
 */ 

#include <avr/io.h>
#include <stdlib.h>
#include <stdbool.h>
#include "communication.h"
#include "C:\Users\Justin\Documents\Atmel Studio\6.1\UM6_copter\UM6_test\usart_driver.h"
#include "C:\Users\Justin\Documents\Atmel Studio\6.1\UM6_copter\UM6_test\PIDcontrol.h"




#define BUFSIZE					8
#define F_CPU 					32000000UL
#define XBEE_USART				USARTD1
#define END_PACKET_CHAR			0xCC
#define START_PACKET_CHAR		0xFF
#define MASK_TOP_BYTE			0x00FF
#define upperByte16(x) (MASK_TOP_BYTE & (x >> 8))		// get the top 8 bits of a 16 bit word
#define lowerByte16(x) (MASK_TOP_BYTE & x)				// get the lower 8 bits of a 16 bit word

volatile uint8_t TxBuffer[BUFSIZE];
int16_t int16counter;
PID_data_t rollAxis,yawAxis,pitchAxis,throttleAxis;


int main(void)
{
	
	
	init32MHzClock();
	initUART();
	spi_set_up();
	intiLoopTimer();
	sendUM6_Data();	
    while(1) {}
}


//  runs on interrupt every 3.5mSec,250Hz
void ControlLoop()
{
	//UpdateEulerAngles();
	
		WriteToPC_SPI();
		//sendUM6_Data();
		int16counter = 0;
	
	
}


// transact PC data
void WriteToPC_SPI()
{
	PORTE.OUTCLR = PIN4_bm;

	uint8_t dummy_read;

	dummy_read = spiPC_write_read(END_PACKET_CHAR);
	dummy_read = spiPC_write_read(END_PACKET_CHAR);
	
	throttleAxis.thrust = spiPC_write_read(upperByte16(throttleAxis.thrust )) << 8;
	throttleAxis.thrust += spiPC_write_read(lowerByte16(throttleAxis.thrust ));
	
	rollAxis.attitude_command = spiPC_write_read(upperByte16(rollAxis.attitude_feedback)) << 8;
	rollAxis.attitude_command  += spiPC_write_read(lowerByte16(rollAxis.attitude_feedback));
	
	pitchAxis.attitude_command = spiPC_write_read(upperByte16(pitchAxis.attitude_feedback)) << 8;
	pitchAxis.attitude_command += spiPC_write_read(lowerByte16(pitchAxis.attitude_feedback));
	
	yawAxis.attitude_command = spiPC_write_read(upperByte16(yawAxis.attitude_feedback)) << 8;
	yawAxis.attitude_command += spiPC_write_read(lowerByte16(yawAxis.attitude_feedback));

	PORTE.OUTSET = PIN4_bm;
	//PORTA.OUTTGL = 0x00000001;
	
}
//***********************************************************************************************************
//  Read in data from the IMU.  Most of the IMU data are 16 bits sotred in 32 bit registers, see
//  data sheet for read commands.
//  
//***********************************************************************************************************
void UpdateEulerAngles()
{

	PORTF.OUTCLR = PIN4_bm;

	uint8_t dummy_read;
	//psi = yaw  phi = roll    theta = pitch
	dummy_read = spiIMU_write_read(READ_COMMAND);
	dummy_read = spiIMU_write_read(UM6_EULER_PHI_THETA);
	
	//MSB first
	rollAxis.attitude_feedback = (spiIMU_write_read(DUMMY_READ)<< 8) | spiIMU_write_read(DUMMY_READ);

	pitchAxis.attitude_feedback = (spiIMU_write_read(DUMMY_READ)<< 8) | spiIMU_write_read(UM6_EULER_PSI);
	
	yawAxis.attitude_feedback = (spiIMU_write_read(DUMMY_READ)<< 8) | spiIMU_write_read(DUMMY_READ);

	dummy_read = spiIMU_write_read(DUMMY_READ);
	dummy_read =  spiIMU_write_read(DUMMY_READ);

	PORTF.OUTSET = PIN4_bm;

}





void sendUM6_Data()
{
	sendData_int16_t(0xFFFF);

	sendData_int16_t(throttleAxis.thrust);
	sendData_int16_t(rollAxis.attitude_command);
	sendData_int16_t(pitchAxis.attitude_command);
	sendData_int16_t(yawAxis.attitude_command);

}






//send 16 bit data on USART, 2 bytes
void sendData_int16_t(int16_t sendthis)
{
	put_USART_PC_char( MASK_TOP_BYTE & (sendthis >> 8));
	put_USART_PC_char (MASK_TOP_BYTE & sendthis);
}





void put_USART_PC_char(uint8_t sendThis)
{

	do{

	}while(!USART_IsTXDataRegisterEmpty(&XBEE_USART));
	USART_PutChar(&XBEE_USART, sendThis);
}





//  set up the serial port for sending data back and forth to the PC
// via the XBEE radio.
//  8 bits, no parity, 2 stop bits
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







void intiLoopTimer()
{
	

	// Set the timer to run at the fastest rate.
	TCD0.CTRLA = TC_CLKSEL_DIV4_gc;

	/* Configure the timer for normal counting. */
	TCD0.CTRLB = TC_WGMODE_NORMAL_gc;

	// At 32 MHz/DIV_4 = 8Mhz,  65353 - (8,000,000 * .0035Sec) = 37535
	TCD0.PER = 37535;

	//Configure timer to generate an interrupt on overflow. */
	TCD0.INTCTRLA = TC_OVFINTLVL_LO_gc;

	/* Enable this interrupt level. */
	PMIC.CTRL |= PMIC_LOLVLEN_bm;

}







// switch  to clock divisor = 4, so  32Mhz / 250Hz = 32000 ticks
/* Function to handle timer overflowing. */
ISR(TCD0_OVF_vect)
{
	ControlLoop();
	TCD0.CNT = 0;
	
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