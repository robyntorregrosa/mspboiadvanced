#include <msp430.h>


/**
 * blink.c
 */

int readX(void);
int readY(void);
void setupBTNs(void);
void setupSPIPins(void);
void setupDisplayPins(void);
void comm_out(unsigned int);
void data_out(unsigned int);

void comm_out(unsigned int c){
    P3OUT &= ~BIT0;
    P2OUT &= ~BIT5;
    __delay_cycles(1);
    P2OUT &= ~BIT4;
    UCA0TXBUF = c;
    __delay_cycles(1);
    P2OUT |= BIT4;
    P3OUT |= BIT0;
    __delay_cycles(1);
}


int readX(void){
    ADC10CTL1 |= INCH_0 | ADC10SSEL_1;           // A0 source, ACLK as source ADC10
    ADC10CTL0 |= ENC + ADC10SC;
    // Enable ADC10, start conversion
    __bis_SR_register(LPM3_bits + GIE);      // Enter LPM1 w/interrupt
    return ADC10MEM;
}

int readY(void){
    ADC10CTL1 |= INCH_3 | ADC10SSEL_1;           // A0 source, ACLK as source ADC10
    ADC10CTL0 |= ENC + ADC10SC;
    // Enable ADC10 A3, start conversion
    __bis_SR_register(LPM3_bits + GIE);      // Enter LPM1 w/interrupt
    return ADC10MEM;
}

void setupBTNs(void){
    // setup switch
    // BTN2  P2.2
    P2SEL &= ~(BIT2);        // I/O
    P2SEL2 &= ~BIT2;
    P2DIR &= ~BIT2;                 // pin 2.2 direction is input

    // BUTTON1 P3.6
    P3SEL &= ~BIT6;                 // Setup I/O
    P3SEL2 &= ~BIT6;
    P3DIR &= ~BIT6;                 // As INput


    // setup pin 2.2 for button SWITCH to trigger interrupt, initialize to no pending interrupt
    P2REN |= BIT2;      // pullup/pulldown resistor enabled
    P2OUT |= BIT2;     // pullup
    P2IE  |= BIT2;      // enable pin 2.0 (SWITCH) interrupt flag
    P2IES |= BIT2;      // pin 2.0 interrupt flag set on high-to-low transition
    P2IFG &= ~BIT2;     // initialize pin 2.0 interrupt flag to none pending
}

void setupSPIPins(void){
    //  SPI - O (out to flash)
    //  Pin 3, P1.1/UCA0SOMI
    //  PIN 4, P1.2/UCA0SIMO
    //  PIN 6, P1.4/UCA0CLK

    P1SEL |=  BIT2 | BIT3 | BIT4;                     // select UCA0SOMI and UCA0SIMO
    P1SEL2 |=  BIT2 | BIT3 | BIT4;

    UCA0CTL1 |= UCSWRST + UCSSEL_2;                     // **Initialize USCI state machine**
    UCA0CTL0 |= UCMST+UCSYNC+UCMSB;           // 8-bit SPI mstr, MSb 1st, CPOL=0, CPHS=0, 3 pin SPI
    //UCA0BR0 = 0;                              // Set Frequency
    //UCA0BR1 = 0;
    UCA0CTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**

    // SPI - S (into slave)
    // Pin 22, P1.6/UCB0SOMI
    // Pin 23, P1.7/UCB0SIMO
    // S CLK Pin 7, P1.5/ UCB0CLK

    P1SEL |= BIT5 | BIT6 | BIT7;                     // selectde UCB0SOMI and UCB0SIMO
    P1SEL2 |=  BIT5 | BIT6 | BIT7;

    UCB0CTL1 |= UCSWRST;                     // **Initialize USCI state machine**
    UCB0CTL0 |= UCMST+UCSYNC+UCMSB;           // 8-bit SPI mstr, MSb 1st, CPOL=0, CPHS=0, 3 pin SPI
    UCB0CTL1 |= UCSSEL_2;                     // ACLK
    UCB0BR0 = 0;                           // Set Frequency
    UCB0BR1 = 0;
    UCB0CTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**
}

void setupDisplayPins(void){
    /*
     * CS1 -> 3.0  Active LOW
     * RES -> 3.4  Active LOW
     * A0  -> 2.5  Register Select
     * WR  -> 2.4  Active LOW
     * RD  -> 2.3  Active LOW
     * C86 -> 3.7  Always LOW
     * PS  -> 2.7  Always LOW
     */
    P2DIR |= BIT7 + BIT5 + BIT4 + BIT3;
    P3DIR |= BIT0 + BIT4 + BIT7;

    //Initially LOW pins
    //P2OUT &= ~(BIT7 + BIT5);
    //P3OUT &= ~(BIT7);

    P2OUT &= ~(BIT7 + BIT5);
    P2REN |= BIT7;              // PULL DOWN RESISTOR

    P3REN |= BIT7;              // PULL DOWN RESISTOR
    P3OUT &= ~BIT7;



    //Initially HIGH pins
    P2OUT |= BIT3 + BIT4;
    P3OUT |= BIT0 + BIT4;
}



void main(void)
{
    WDTCTL = WDTPW | WDTHOLD;       // stop watchdog timer

    // Setup Pins
    // P2.1/TA1.1 PWM
    //P2SEL |= BIT1;
    //P2SEL2 &= ~BIT1;
    P2DIR |= BIT1;                  // Timer1_A3.TA1
    P2OUT |= BIT1;
    setupSPIPins();
    setupDisplayPins();

    // joystick adc input X (Pin 2, A0), Y (pin 5, A3)
    ADC10CTL0 |= ADC10SHT_3 | ADC10ON | ADC10IE;

    volatile unsigned int i;        // volatile to prevent optimization

/*
    comm_out(0xA2); //added 1/9 bias
    comm_out(0xA0); //ADC segment driver direction (A0=Normal)
    comm_out(0xC8); //added
    comm_out(0xC0); //COM output scan direction (C0= Normal)
    comm_out(0x40); //Operating Mode
    __delay_cycles(1);
    comm_out(0x25); //resistor ratio
    __delay_cycles(1);
    comm_out(0x81); //electronic volume mode set
    __delay_cycles(1);
    comm_out(0x19); //electronic volume register set
    __delay_cycles(1);
    comm_out(0x2F); //power control set
    __delay_cycles(1);
    comm_out(0xAF); //display ON/OFF - set to ON
*/
/*
    P3OUT &= ~BIT0;                 //Enable Chip Select
    __delay_cycles(10);
    P2OUT &= ~BIT4;                 //Enable Write

    UCA0TXBUF = 0b10101111;              //Send turn on display
    //__delay_cycles(10);
    //UCA0TXBUF = 0b10100101;              //Send all points on
    //__delay_cycles(10);

    P2OUT |= BIT4;                 //Disable Write
    __delay_cycles(10);
    P3OUT |= BIT0;                 //Disable Chip Select
    __delay_cycles(10);


    P3OUT &= ~BIT0;                 //Enable Chip Select
    __delay_cycles(10);
    P2OUT &= ~BIT4;                 //Enable Write

    UCA0TXBUF = 0b10100101;              //Send all points on
    //__delay_cycles(10);
    //UCA0TXBUF = 0b10100101;              //Send all points on
    //__delay_cycles(10);

    P2OUT |= BIT4;                 //Disable Write
    __delay_cycles(10);
    P3OUT |= BIT0;                 //Disable Chip Select
    __delay_cycles(10);
*/


/*
        //TIMER A_1
        TA1CCR0  |= 12000;                         // Setting PWM Freq
        TA1CCR1  |= 6000;                           // Duty Cycle (G)
        TA1CCTL1 |= OUTMOD_7;                      // Reset/Set Mode
        TA1CTL   |= TASSEL_1 + MC_1;               // ACLK and UP TO CCR0
*/



    int Xdir;
    int Ydir;
    Xdir = 0;
    Ydir = 0;
    while(1)
    {
        Xdir= readX();
        Ydir= readY();

//      P1OUT ^= 0x01;              // toggle P1.0
//      for(i=10000; i>0; i--);     // delay
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

// Port 2 interrupt service routine (For BTN1)
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=PORT2_VECTOR
__interrupt void Port_2 (void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(PORT2_VECTOR))) Port_2 (void)
#else
#error Compiler not supported!
#endif
{
    __bic_SR_register_on_exit(LPM1_bits);  // upon exit, clear register of bits that correspond to LPM4 mode to go back to main

}
