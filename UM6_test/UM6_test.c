
#include <avr/io.h>
#include <stdlib.h> 
#include "usart_driver.h"
#include "avr_compiler.h"
#include "communication.c"
#include "tc_driver.h"
#include "tc_driver.c"
#include "myUtilities.h"
#include "PWM.h"
#include "dma_driver.h"
#include "dma_driver.c"
#include "PIDcontrol.h"




/*! Define that selects the UASRT used in example. */
#define F_CPU 					32000000UL
#define XBEE_USART				USARTD1
#define	IMU_USART				USARTC1
#define DMA_RX_Channel			&DMA.CH1
#define MASK_TOP_BYTE			0x00FF
#define NUM_CMD_BYTES		    10
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
    spi_set_up();intPID_gains();
    initPWM();
    init32MHzClock();
    initUART();
    //Initialize DMAC
    DMA_Enable();
    Setup_DMA_ReceiveChannel();
    //The receiving DMA channel will wait for characters
    //and write them to Rx_Buf
    DMA_EnableChannel(DMA_RX_Channel);
	

		while(1)
		{
		
			_delay_ms(5);

	
			UpdateEulerAngles();

			PORTA.OUTTGL = 0xFF;			//  LEDS
			Get_DMA_DATA(DMA_RX_Channel);


			//CalculateRate(&rollAxis);
			//pid_attitude(&rollAxis);
			//pid_rate(&rollAxis);
			int16counter++;
			if (int16counter >= 15)
			{
					sendUM6_Data();
					int16counter = 0;
			}
			
		}		
				
	 
}	


//250 mSec * 1000mSec / 1 Sec * 1/32,000,000
void getCommand()
{

	int i;
	
	if (Rx_Buf[0] == 0xFF && Rx_Buf[1] == 0xFD)
	{
		i =	2;

		throttleAxis.attitude_command =(throttleAxis.attitude_command << 8 ) + Rx_Buf[i++];
		throttleAxis.attitude_command =(throttleAxis.attitude_command << 8 ) + Rx_Buf[i++];

		//yawAxis.command = (yawAxis.command << 8 ) + Rx_Buf[i++];
		//yawAxis.command =(yawAxis.command << 8 ) + Rx_Buf[i++];

		//pitchAxis.command = (pitchAxis.command  << 8 ) + Rx_Buf[i++];
		//pitchAxis.command  =(pitchAxis.command  << 8 ) + Rx_Buf[i++];
		
		yawAxis.attitude_command = (yawAxis.attitude_command << 8 ) + Rx_Buf[i++];
		yawAxis.attitude_command =(yawAxis.attitude_command << 8 ) + Rx_Buf[i++];
		
		pitchAxis.attitude_command = (pitchAxis.attitude_command << 8 ) + Rx_Buf[i++];
		pitchAxis.attitude_command =(pitchAxis.attitude_command << 8 ) + Rx_Buf[i++];
			
		rollAxis.attitude_command = (rollAxis.attitude_command << 8 ) + Rx_Buf[i++];
		rollAxis.attitude_command =(rollAxis.attitude_command << 8 ) + Rx_Buf[i++];
		
		//squareWave();
		//rollAxis.Kp_rate = (rollAxis.Kp_rate  << 8 ) + Rx_Buf[i++];
		//rollAxis.Kp_rate  =(rollAxis.Kp_rate  << 8 ) + Rx_Buf[i++];
		//
		//rollAxis.Ki_rate = (rollAxis.Ki_rate << 8 ) + Rx_Buf[i++];
		//rollAxis.Ki_rate =(rollAxis.Ki_rate << 8 ) + Rx_Buf[i++];
		//
		//rollAxis.Kd_rate = (rollAxis.Kd_rate << 8 ) + Rx_Buf[i++];
		//rollAxis.Kd_rate =(rollAxis.Kd_rate << 8 ) + Rx_Buf[i++];
//
		//rollAxis.Kp = (rollAxis.Kp  << 8 ) + Rx_Buf[i++];
		//rollAxis.Kp  =(rollAxis.Kp  << 8 ) + Rx_Buf[i++];
		//
		//rollAxis.Ki = (rollAxis.Ki << 8 ) + Rx_Buf[i++];
		//rollAxis.Ki =(rollAxis.Ki << 8 ) + Rx_Buf[i++];
		//
		//rollAxis.Kd = (rollAxis.Kd << 8 ) + Rx_Buf[i++];
		//rollAxis.Kd =(rollAxis.Kd << 8 ) + Rx_Buf[i++];
		
		
	}
	else
	{
		//  if we didn't start with the frame header, something isn't right,  rest the DMA channel
		//  would be nice to know why this happens,  maybe we are reading the buffer will the DMA is trying to write to it
		//PORTA.OUTTGL = 0x00000001;
		//DMA_Disable();
		//DMA_Enable();
		//Setup_DMA_ReceiveChannel();
		//DMA_EnableChannel(DMA_RX_Channel);
	}
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
	sendData_int16_t(0xFFFE);
	//sendData_int16_t(0x0404);
	sendData_int16_t(UM6_EulerYaw);
	sendData_int16_t(UM6_EulerPitch);
	sendData_int16_t(UM6_EulerRoll);
	
}



void Get_DMA_DATA( volatile DMA_CH_t * channel )
{
	uint8_t flagMask;
	uint8_t relevantFlags;

	flagMask = DMA_CH_ERRIF_bm | DMA_CH_TRNIF_bm;
	relevantFlags = channel->CTRLB & flagMask;

	if (relevantFlags != 0x00)
	{
		getCommand();
		channel->CTRLB = flagMask;
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
	DMA_CH_DESTRELOAD_BLOCK_gc, DMA_CH_DESTDIR_INC_gc, NUM_CMD_BYTES, DMA_CH_BURSTLEN_1BYTE_gc,  0x00, true);
	
	DMA_EnableSingleShot(DMA_RX_Channel);
	
	// USART Trigger source, Receive complete
	DMA_SetTriggerSource(DMA_RX_Channel, DMA_CH_TRIGSRC_USARTD1_RXC_gc);
	
}


