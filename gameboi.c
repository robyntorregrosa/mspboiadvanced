#include <msp430.h>				


/**
 * blink.c
 */

int readX(){
    ADC10CTL1 |= INCH_0 | ADC10SSEL_1;           // A0 source, ACLK as source ADC10
    ADC10CTL0 |= ENC + ADC10SC;
    // Enable ADC10, start conversion
    __bis_SR_register(LPM3_bits + GIE);      // Enter LPM1 w/interrupt
    return ADC10MEM;
}

int readY(){
    ADC10CTL1 |= INCH_3 | ADC10SSEL_1;           // A0 source, ACLK as source ADC10
    ADC10CTL0 |= ENC + ADC10SC;
    // Enable ADC10 A3, start conversion
    __bis_SR_register(LPM3_bits + GIE);      // Enter LPM1 w/interrupt
    return ADC10MEM;
}

void main(void)
{
	WDTCTL = WDTPW | WDTHOLD;		// stop watchdog timer
	P1DIR |= 0x01;					// configure P1.0 as output

	// Setup Pins

    P2SEL |= BIT1 | BIT2;                            // P2.1/TA1.1 PWM, P2.2 BTN2
    P2SEL2 &= ~BIT2;                                 //
    P2DIR |= BIT1;                                   // pin 2.1 direction is output
    P2DIR &= ~BIT2;                                   // pin 2.2 direction is input


    // joystick adc input X (Pin 2, A0), Y (pin 5, A3)
    ADC10CTL0 |= ADC10SHT_3 | ADC10ON | ADC10IE;

	volatile unsigned int i;		// volatile to prevent optimization

	int Xdir;
	int Ydir;
	Xdir = 0;
	Ydir = 0;
	while(1)
	{
	    Xdir= readX();
        Ydir= readY();

//		P1OUT ^= 0x01;				// toggle P1.0
//		for(i=10000; i>0; i--);     // delay
	}
}

// ADC10 interrupt service routine
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=ADC10_VECTOR
__interrupt void ADC10_timer(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(ADC10IFG))) watchdog_timer (void)
#else
#error Compiler not supported!
#endif
{
    __bic_SR_register_on_exit(LPM3_bits);
}


