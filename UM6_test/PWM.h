/*
 * PWM.h
 *
 * Created: 3/5/2013 8:59:27 AM
 *  Author: jzimmerman
 */ 

void initPWM(void);
void doPWM(uint16_t,uint16_t,uint16_t,uint16_t);


#ifndef  PWM_H_
#define  PWM_H_ 
#define  MOTOR_PMW_TIMER0		TCC0
#define  MOTOR_PORT				PORTC

void initPWM()
{
    /* To change the duty cycle for the PWM for the LED control, use a variable to increase/decrease the
    * CCx registers once for each period */

    //int16_t pwm_delta = 300;

    // Add code to select a Single Slope PWM as Waveform Generation mode. 
    // This is done by setting the WGM bits,in the CTRLB register for the TC.
	
	MOTOR_PMW_TIMER0.CTRLA = TC_WGMODE_SS_gc;
	MOTOR_PMW_TIMER0.CTRLB = TC_WGMODE_SS_gc;
	MOTOR_PMW_TIMER0.CTRLC = TC_WGMODE_SS_gc;
	MOTOR_PMW_TIMER0.CTRLD = TC_WGMODE_SS_gc;

    /* Add code to enable the CC channels we wish to use. Each channel must be separately enabled
    * by setting the corresponding CCxEN bits in the CTRLB register for the TC.
    * Code for enabling CCA is already added, but add code to enable CCB, CCC and CCD */
	MOTOR_PMW_TIMER0.CTRLB |= TC0_CCAEN_bm;
    MOTOR_PMW_TIMER0.CTRLB |= TC0_CCBEN_bm;
	MOTOR_PMW_TIMER0.CTRLB |= TC0_CCCEN_bm;
	MOTOR_PMW_TIMER0.CTRLB |= TC0_CCDEN_bm;


    //Insert code to enable CC for the other channels, B, C and D.


    /* The corresponding port pins MUST be output for the Waveform to be visible
    * on the pin. For TCE0 the corresponding port is PORTE, and pin 0 to 3 for
    * CC channel A to D */

    MOTOR_PORT.DIRSET = PIN0_bm | PIN1_bm | PIN2_bm | PIN3_bm;

    /* Note how the inverted signal is always controlled from the Port pin configuration in XMEGA
    * This can be used with all other peripherals connected to the pin to have inverted output.
    * Below is example code on how you can set inverted output on a pin using the Pin Control Register*/

    //PORTE.PIN0CTRL &= ~PORT_INVEN_bm;

    /* Set a compare value for each compare channel, just as in task2.
    * The compare value decide the PWM duty cycle for the waveform.
    * 
    * Code for CC channel A is added, add code for channel B, C, and D
    * with the compare value 3000 */

    TC_SetCompareA(&MOTOR_PMW_TIMER0, 0);
	TC_SetCompareB(&MOTOR_PMW_TIMER0, 0);
	TC_SetCompareC(&MOTOR_PMW_TIMER0, 0);
	TC_SetCompareD(&MOTOR_PMW_TIMER0, 0);
    //Insert function calls to set the other compare values

    /* Using the TC_driver we set the Period and 
    * start the timer with  pre-scaling */
	// set PWM frequency at 300Hz with clock division = 4.  1/(300Hz / (32MHz /4)) - 1 = 26665      Period = 3.333 mSec
	 // 1mSec = 8000 = Off
	 // 2mSec =  = 15999 = ON
    TC_SetPeriod(&MOTOR_PMW_TIMER0, 26665);
    TC0_ConfigClockSource(&MOTOR_PMW_TIMER0, TC_CLKSEL_DIV4_gc);

	
}

void doPWM(uint16_t speed_1, uint16_t speed_2, uint16_t speed_3, uint16_t speed_4)
{
        /* The code check if the overflow flag is set,
         * if so it clears the flag and sets a new duty cycle for all
         * CC channels */

        /* Check if overflow flag (OVFIF) is set,
         * clear flag and set a new duty cycle
         */
        if(TC_GetOverflowFlag(&MOTOR_PMW_TIMER0) != 0) 
        {
            TC_ClearOverflowFlag(&MOTOR_PMW_TIMER0);      //Clear the IF by writing a logical 1 to the flag

            MOTOR_PMW_TIMER0.CCABUF = speed_1; //Change the compare value to change duty cycle
			MOTOR_PMW_TIMER0.CCBBUF = speed_2;
			MOTOR_PMW_TIMER0.CCCBUF = speed_3;
			MOTOR_PMW_TIMER0.CCDBUF = speed_4;

        }

}

#endif /* PWM_H_ */