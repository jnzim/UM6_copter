
#include <avr/io.h>
#include <stdlib.h> 
#include "usart_driver.h"
#include "avr_compiler.h"
#include "tc_driver.h"
#include "myUtilities.h"
#include "PWM.h"
#include "dma_driver.h"


/*! Define that selects the UASRT used in example. */
#define F_CPU 					32000000UL
#define XBEE_USART				USARTD1
#define	IMU_USART				USARTC1
#define DMA_RX_Channel			&DMA.CH1
#define START_FRAME_TOKEN		0x24
#define END_FRAME_TOKEN			0x0D
#define COMMA					0x2C
#define MASK_TOP_BYTE			0x00FF
#define NUM_BYTES_IN_ANGLE		4
#define NUM_BYTES_IN_FRAME		64
#define PC						0
#define IMU						1
#define NUM_BYTES				10
#define NUM_CMD_BYTES		    10

/*! USART data struct used in example. */
USART_data_t USART_data;
void init32MHzClock(void);
void initUART(void);
uint8_t get_USART_char(void);
uint8_t get_XBEE_USART_char(void);
void put_USART_PC_char(uint8_t);
void initPWM(void);
void sendData_uint16_t(uint16_t);
uint16_t readData_uint16_t(uint8_t );
void getCommand(void);
void getIMU_Data(void);
void sendIMU_Data(void);
void SetupReceiveChannel( void );


uint16_t pitch,roll,yaw, accelX, accelY,accelZ,cmdThrottle,cmdYaw,cmdPitch,cmdRoll;
uint16_t pwm_delta = 250;
uint16_t latch = 0;

static char Rx_Buf[NUM_CMD_BYTES];

int main(void)
{
	PORTA.DIRSET = 0xFF;			//  LEDS
	initPWM();
	init32MHzClock();
	initUART();    
	// Initialize DMAC
	DMA_Enable();
	SetupReceiveChannel();
	// The receiving DMA channel will wait for characters
	// and write them to Rx_Buf
	DMA_EnableChannel(DMA_RX_Channel);
	//PORTB.DIRSET = PIN1_bm;

	while(1) 
	{
		
		PORTA.OUTTGL = 0b00000001;
		latch++;
		getIMU_Data();
		getCommand();
		doPWM(cmdThrottle,cmdYaw,cmdPitch,cmdRoll,cmdYaw);
		if (latch >= 10)
		{	
			PORTA.OUTTGL = 0b00001000;
			sendIMU_Data();
			latch = 0;
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
			//PORTA.OUTTGL = 0xFF;
			cmdThrottle =(cmdThrottle << 8 ) + Rx_Buf[i++];
			cmdThrottle =(cmdThrottle << 8 ) + Rx_Buf[i++];

			cmdYaw = (cmdYaw << 8 ) + Rx_Buf[i++];
			cmdYaw =(cmdYaw << 8 ) + Rx_Buf[i++];
			
			cmdPitch = (cmdPitch << 8 ) + Rx_Buf[i++];
			cmdPitch =(cmdPitch << 8 ) + Rx_Buf[i++];
			
			cmdRoll = (cmdRoll << 8 ) + Rx_Buf[i++];
			cmdRoll =(cmdRoll << 8 ) + Rx_Buf[i++];
			
						
		}
		else
		{

				PORTA.OUTTGL = 0x00000010;
				DMA_Disable();
				DMA_Enable();
				SetupReceiveChannel();
				DMA_EnableChannel(DMA_RX_Channel);
				
		}	
}
   
   
   

 //250 mSec * 1000mSec / 1 Sec * 1/32,000,000
//void getCommand()
//{
//
	//int i = 0;
		//
	//if (Rx_Buf[0] == 0xFF && Rx_Buf[1] == 0xFD) 
	//{
		//i =	2;
			//
		//cmdThrottle =(cmdThrottle << 8 ) + Rx_Buf[i++];
		//cmdThrottle =(cmdThrottle << 8 ) + Rx_Buf[i++];
//
		//cmdYaw = (cmdYaw << 8 ) + Rx_Buf[i++];
		//cmdYaw =(cmdYaw << 8 ) + Rx_Buf[i++];
			//
		//cmdPitch = (cmdPitch << 8 ) + Rx_Buf[i++];
		//cmdPitch =(cmdPitch << 8 ) + Rx_Buf[i++];
			//
		//cmdRoll = (cmdRoll << 8 ) + Rx_Buf[i++];
		//cmdRoll =(cmdRoll << 8 ) + Rx_Buf[i++];
		//
	//}		
	//else
	//{
		 //lose sync, need to get back to the beginning of a frame
			//
	//}			
		//
//}

void getIMU_Data()
{
	// read the data from the IMU
	//  data is sent from the IMU at 50Hz, 20mSec
	yaw = 0; pitch = 0; roll = 0;
	while (get_USART_char() != 0xFF)	{;}
	while( get_USART_char() != 0xFF)   {;}
			
	yaw =(yaw << 8 ) + get_USART_char();
	yaw =(yaw << 8 ) + get_USART_char();
			
	pitch =(pitch << 8 ) + get_USART_char();
	pitch =(pitch << 8 ) + get_USART_char();
			
	roll =(roll << 8 ) + get_USART_char();
	roll =(roll << 8 ) + get_USART_char();
			
	//accelX =(accelX << 8 ) + get_USART_char();
	//accelX =(accelX << 8 ) + get_USART_char();
	//
	//accelY =(accelY << 8 ) + get_USART_char();
	//accelY =(accelY << 8 ) + get_USART_char();
	//
	//accelZ =(accelZ << 8 ) + get_USART_char();
	//accelZ =(accelZ << 8 ) + get_USART_char();
}

void sendIMU_Data()
{
	//  0xFFFE is the signal to the PC that the next data will be IMU data
	// 255 254
	sendData_uint16_t(0xFFFE);
	sendData_uint16_t(cmdThrottle);
	sendData_uint16_t(cmdYaw);
	sendData_uint16_t(cmdPitch);
	sendData_uint16_t(cmdRoll);
	sendData_uint16_t(yaw);
	sendData_uint16_t(pitch);
	sendData_uint16_t(roll);
	//sendData_uint16_t(accelX);
	//sendData_uint16_t(accelY);
	//sendData_uint16_t(accelZ);
}

void sendData_uint16_t(uint16_t sendthis)
{
	
	put_USART_PC_char( MASK_TOP_BYTE & (sendthis >> 8));
	put_USART_PC_char (MASK_TOP_BYTE & sendthis);

	
}

uint16_t readData_uint16_t(uint8_t readPort)
{
	
	uint8_t data = 0;
	if (readPort == 0)
	{
		data =(data << 8 ) + get_USART_char();
		data =(data << 8 ) + get_USART_char();
		
	}
	if (readPort == 1)
	{
		data =(data << 8 ) + get_USART_char();
		data =(data << 8 ) + get_USART_char();
		
	}
	return data;
	
}



uint8_t get_USART_char()
{
	do{
	}while(!USART_IsRXComplete(&IMU_USART));
	return USART_GetChar(&IMU_USART);
}

uint8_t get_XBEE_USART_char()	
{
	uint16_t timeout = 1000;
	
	do{
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
	//uint16_t timeout = 1000;
	/* Send one char. */
	do{
	/* Wait until it is possible to put data into TX data register.
		* NOTE: If TXDataRegister never becomes empty this will be a DEADLOCK. */
	//timeout--;
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
	


	/* USARTC0, 8 Data bits, No Parity, 1 Stop bit. */
	USART_Format_Set(&IMU_USART, USART_CHSIZE_8BIT_gc, USART_PMODE_DISABLED_gc, false);
	USART_Format_Set(&XBEE_USART, USART_CHSIZE_8BIT_gc, USART_PMODE_DISABLED_gc, false);


	USART_Baudrate_Set(&IMU_USART, 1079 , -5);
	USART_Baudrate_Set(&XBEE_USART, 1079 , -5);

	/* Enable both RX and TX. */
	USART_Rx_Enable(&IMU_USART);
	USART_Tx_Enable(&IMU_USART);
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

void SetupReceiveChannel( void )	
{
    DMA_SetupBlock(  DMA_RX_Channel,(void *) &USARTD1.DATA, DMA_CH_SRCRELOAD_NONE_gc,  DMA_CH_SRCDIR_FIXED_gc, Rx_Buf,
    DMA_CH_DESTRELOAD_BLOCK_gc, DMA_CH_DESTDIR_INC_gc, NUM_CMD_BYTES, DMA_CH_BURSTLEN_1BYTE_gc,  0x00, true); 
      
    DMA_EnableSingleShot(DMA_RX_Channel);
	
    // USART Trigger source, Receive complete
    DMA_SetTriggerSource(DMA_RX_Channel, DMA_CH_TRIGSRC_USARTD1_RXC_gc); 
	
}




