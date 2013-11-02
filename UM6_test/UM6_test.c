
#include <avr/io.h>
#include <stdlib.h> 
#include <avr/interrupt.h>
#include "usart_driver.h"
#include "avr_compiler.h"
#include "PIDcontrol.h"
#include "communication.c"
#include "tc_driver.h"
#include "tc_driver.c"
#include "myUtilities.h"
#include "PWM.h"
#include "dma_driver.h"
#include "dma_driver.c"





/*! Define that selects the UASRT used in example. */
#define F_CPU 					32000000UL
#define XBEE_USART				USARTD1
#define	IMU_USART				USARTC1
#define DMA_RX_Channel			&DMA.CH1
#define MASK_TOP_BYTE			0x00FF
#define NUM_CMD_BYTES		    32
#define SCALE_THROTTLE			4



void init32MHzClock(void);
void initUART(void);
void put_USART_PC_char(uint8_t);
void sendData_int16_t(int16_t);
void initPWM(void);
void getCommand(void);
void Setup_DMA_ReceiveChannel( void );
void intPID_gains(void);
void Get_DMA_DATA( volatile DMA_CH_t * channel );



int16_t int16counter;
uint8_t newPacketFlag;
PID_data_t rollAxis,yawAxis,pitchAxis,throttleAxis;
uint16_t sendDataLoopCounter = 0;
int commandTicker = 0;
//  buffer for DMA data
static char Rx_Buf[NUM_CMD_BYTES];




int main(void)
{

	PORTA.DIRSET = 0xFF;			//  LEDS
	init32MHzClock();
	initUART();
    spi_set_up();
	intPID_gains();
    initPWM();
    initUART();
	 ZeroGyros();
	 _delay_ms(1000);
	 ZeroAccelerometers();
	 _delay_ms(3000);
    //Initialize DMAC
    //DMA_Enable();
    //Setup_DMA_ReceiveChannel();
    //The receiving DMA channel will wait for characters
    //and write them to Rx_Buf
    //DMA_EnableChannel(DMA_RX_Channel);
	intiLoopTimer();
	 sei(); 

		while(1)
		{
	
			nop();
		}		
				
	 
}	


//  runs on interrupt every 3.5mSec,250Hz
void ControlLoop()
{
	UpdateEulerAngles();
	//pid_attitude(&rollAxis);
	//SetPulseWidths();
	int16counter++;
	if (int16counter >= 7)						// 3.5*2 = 7
	{
		WriteToPC_SPI();
		//sendUM6_Data();
		int16counter = 0;
		
	}
	
}

void SetPulseWidths()
{
	// check the signs
	if(throttleAxis.attitude_command > 2000 && throttleAxis.attitude_command <= 4095)
	{
		doPWM(

		throttleAxis.attitude_command * SCALE_THROTTLE + pitchAxis.attitude_pid_out,
		throttleAxis.attitude_command * SCALE_THROTTLE + rollAxis.attitude_pid_out,
		throttleAxis.attitude_command * SCALE_THROTTLE - pitchAxis.attitude_pid_out,
		throttleAxis.attitude_command * SCALE_THROTTLE - rollAxis.attitude_pid_out	);
		
	}

	else
	{
		doPWM(0,0,0,0);
	}
	
}

uint8_t myDMA_ReturnStatus_blocking( volatile DMA_CH_t * channel )
{
	uint8_t flagMask;
	uint8_t relevantFlags;

	flagMask = DMA_CH_ERRIF_bm | DMA_CH_TRNIF_bm;

	do {
		relevantFlags = channel->CTRLB & flagMask;
	} while (relevantFlags == 0x00);
	//getCommandDoubleBuffer();
	getCommand();
	
	channel->CTRLB = flagMask;
	return relevantFlags;
}






	
	//this data is read in from the IMU on the SPI bus
	
void UpdateEulerAngles()
{

	PORTF.OUTCLR = PIN4_bm;

	uint8_t dummy_read;
	//psi = yaw  phi = roll    theta = pitch
	dummy_read = spi_write_read(READ_COMMAND);
	dummy_read = spi_write_read(UM6_EULER_PHI_THETA);
	
	//MSB first
	rollAxis.attitude_feedback = (spi_write_read(DUMMY_READ)<< 8) | spi_write_read(DUMMY_READ);

	pitchAxis.attitude_feedback = (spi_write_read(DUMMY_READ)<< 8) | spi_write_read(UM6_EULER_PSI);
	
	yawAxis.attitude_feedback = (spi_write_read(DUMMY_READ)<< 8) | spi_write_read(DUMMY_READ);

	
	dummy_read = spi_write_read(DUMMY_READ);
	dummy_read =  spi_write_read(DUMMY_READ);

	PORTF.OUTSET = PIN4_bm;

}


void WriteToPC_SPI()
{
		PORTE.OUTCLR = PIN4_bm;

		uint8_t dummy_read;

		dummy_read = spiPC_write_read(MASK_TOP_BYTE & (rollAxis.attitude_feedback >> 8));
		dummy_read = spiPC_write_read(MASK_TOP_BYTE & rollAxis.attitude_feedback);
	
		dummy_read = spiPC_write_read(MASK_TOP_BYTE & (pitchAxis.attitude_feedback >> 8));
		dummy_read = spiPC_write_read(MASK_TOP_BYTE & pitchAxis.attitude_feedback);
	
		dummy_read = spiPC_write_read(MASK_TOP_BYTE & (yawAxis.attitude_feedback >> 8));
		dummy_read = spiPC_write_read(MASK_TOP_BYTE & yawAxis.attitude_feedback);
			
		dummy_read = spiPC_write_read(0xCC);
		dummy_read = spiPC_write_read(0xCC);
;
	
		PORTE.OUTSET = PIN4_bm;
		//PORTA.OUTTGL = 0x00000001;
		
}


//send 16 bit data on USART, 2 bytes
void sendData_int16_t(int16_t sendthis)
{
	put_USART_PC_char( MASK_TOP_BYTE & (sendthis >> 8));
	put_USART_PC_char (MASK_TOP_BYTE & sendthis);
}


// send 32 bit data on USART, 4 bytes
void sendData_int32_t(int32_t sendthis)
{
	put_USART_PC_char( MASK_TOP_BYTE & (sendthis >> 24));
	put_USART_PC_char( MASK_TOP_BYTE & (sendthis >> 16));
	put_USART_PC_char( MASK_TOP_BYTE & (sendthis >> 8));
	put_USART_PC_char (MASK_TOP_BYTE & sendthis);
}

void sendUM6_Data()
{
	sendData_int16_t(0xFFFF);

	sendData_int16_t(rollAxis.attitude_feedback);
	sendData_int16_t(pitchAxis.attitude_feedback);
	sendData_int16_t(yawAxis.attitude_feedback);
	//sendData_int16_t(rollAxis.attitude_pid_out);
	//sendData_int16_t(rollAxis.attitude_error);
	//sendData_int16_t(rollAxis.attitude_command);

	

}

void SetNot0xFFFF(uint16_t chkValue)
{
	if (chkValue == 0xFFFF)
	{
		return 0xFFFE;
	}
	else return chkValue;
	
}



void Get_DMA_DATA( volatile DMA_CH_t * channel )
{
	uint8_t flagMask;
	uint8_t relevantFlags;

	flagMask = DMA_CH_ERRIF_bm | DMA_CH_TRNIF_bm;
	relevantFlags = channel->CTRLB & flagMask;

	if (relevantFlags != 0x00)
	{
		channel->CTRLB = flagMask;
		getCommandDoubleBuffer();
		//getCommand();
	}
	

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


void intPID_gains()
{

	yawAxis.Kp = 0;
	yawAxis.Ki = 0;
	yawAxis.Kd = 0;
	
	pitchAxis.Kp = 100;
	pitchAxis.Ki = 100;
	pitchAxis.Kd = 100;
	
	rollAxis.Kp = 0;
	rollAxis.Ki =0;
	rollAxis.Kd =0;
	
	rollAxis.Kp_rate = 100;
	rollAxis.Ki_rate =100;
	rollAxis.Kd_rate =100;
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


void Setup_DMA_ReceiveChannel( void )
{
	DMA_SetupBlock(  DMA_RX_Channel,(void *) &USARTD1.DATA, DMA_CH_SRCRELOAD_NONE_gc,  DMA_CH_SRCDIR_FIXED_gc, Rx_Buf,
	DMA_CH_DESTRELOAD_BLOCK_gc, DMA_CH_DESTDIR_INC_gc, NUM_CMD_BYTES, DMA_CH_BURSTLEN_2BYTE_gc,  0x00, true);
	
	DMA_EnableSingleShot(DMA_RX_Channel);
	
	// USART Trigger source, Receive complete
	DMA_SetTriggerSource(DMA_RX_Channel, DMA_CH_TRIGSRC_USARTD1_RXC_gc);
	
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
//  one clock cycle is 1/32Mhz = 30uSec ,  need to overflow ever 4 mSec = 250Hz
// = 128,000 cycles
//2^16 = 65353  65536 -32000 = 33536

// switch  to clock divisor = 4, so  32Mhz / 250Hz = 32000 ticks  
/* Function to handle timer overflowing. */
ISR(TCD0_OVF_vect)
{
	ControlLoop();
	TCD0.CNT = 0; 
	
}


//250 mSec * 1000mSec / 1 Sec * 1/32,000,000
//  this date is read in on the USART, it's sent from the PC
//  joystick commands, gains...
void getCommand()
{
	int i = 0;
	
	if (Rx_Buf[0] == 0xFF && Rx_Buf[1] == 0xFD)
	{
		i =	2;

		throttleAxis.attitude_command =(throttleAxis.attitude_command << 8 ) + Rx_Buf[i++];
		throttleAxis.attitude_command =(throttleAxis.attitude_command << 8 ) + Rx_Buf[i++];
		
		yawAxis.attitude_command = (yawAxis.attitude_command << 8 ) + Rx_Buf[i++];
		yawAxis.attitude_command =(yawAxis.attitude_command << 8 ) + Rx_Buf[i++];
		
		pitchAxis.attitude_command = (pitchAxis.attitude_command << 8 ) + Rx_Buf[i++];
		pitchAxis.attitude_command =(pitchAxis.attitude_command << 8 ) + Rx_Buf[i++];
		
		rollAxis.attitude_command = (rollAxis.attitude_command << 8 ) + Rx_Buf[i++];
		rollAxis.attitude_command =(rollAxis.attitude_command << 8 ) + Rx_Buf[i++];

		rollAxis.Kp = (rollAxis.Kp  << 8 ) + Rx_Buf[i++];
		rollAxis.Kp  =(rollAxis.Kp  << 8 ) + Rx_Buf[i++];
		
		rollAxis.Ki = (rollAxis.Ki << 8 ) + Rx_Buf[i++];
		rollAxis.Ki =(rollAxis.Ki << 8 ) + Rx_Buf[i++];
		
		rollAxis.Kd = (rollAxis.Kd << 8 ) + Rx_Buf[i++];
		rollAxis.Kd =(rollAxis.Kd << 8 ) + Rx_Buf[i++];

		
	}
	else
	{
		//  if we didn't start with the frame header, something isn't right,  rest the DMA channel
		//  would be nice to know why this happens,  maybe we are reading the buffer will the DMA is trying to write to it
		//PORTA.OUTTGL = 0x00000001;
		DMA_Disable();
		DMA_Enable();
		Setup_DMA_ReceiveChannel();
		DMA_EnableChannel(DMA_RX_Channel);
		
		
	}
}