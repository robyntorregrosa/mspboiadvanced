/* Based on msp430g2xx3_wdt_02.c from the TI Examples */

#include <msp430.h>

int c = 0;                                  //Char index
int i = 0;                                  //Led index
static int DISPLAY_SIZE = 340;               //number of pixels in display

int readX(void);
int readY(void);
void sendBitmap(unsigned char* bitmapping);
void setPixel(int row, int col, char color);
void clearPixel(int row, int col);
void movePixel(int row, int col, int dest_row, int dest_col);
void shiftPixel(int row, int col, int x_shift, int y_shift);
int myPlace[] = {9,10};

static char BRIGHTNESS = 0xE1;             //Set LED brightness for whole bitmap

unsigned char bitmap[] =
//0   1   2   3   4   5   6   7   8   9  10  11  12  13  14  15  16  17  18  19
{' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',   //0
 ' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',   //1
 ' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',   //2
 ' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',   //3
 ' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',   //4
 ' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',   //5
 ' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',   //6
 ' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',   //7
 ' ',' ',' ',' ',' ',' ',' ',' ',' ','1',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',   //8
 ' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',   //9
 ' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',   //10
 ' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',   //11
 ' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',   //12
 ' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',   //13
 ' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',   //14
 ' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',   //15
 ' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',   //16
};

int main(void)
{

    static const int LEFTBOUND = 830;
    static const int RIGHTBOUND = 230;
    static const int UPBOUND = 550;
    static const int DOWNBOUND = 580;
//    int j = 0;
//    int k = 0;
    /*------------- Setup -------------*/
    BCSCTL3 |= LFXT1S_2;                      // ACLK = VLO
    WDTCTL = WDT_ADLY_16;                    // WDT 16ms, ACLK, interval timer
    //WDTCTL = WDT_ADLY_1_9;
    IE1 |= WDTIE;                            // Enable WDT interrupt
    IE2 |= UCA0TXIE;

    /*---------------SPI--------------*/
    //Multiplex Pin 1.2 to MOSI and Pin 1.4 to CLK
    P1DIR |= BIT2 + BIT4;               // Set P1.2 and P1.4 to output direction
    P1SEL = BIT2 + BIT4;
    P1SEL2 = BIT2 + BIT4;

    UCA0CTL1 = UCSWRST + UCSSEL_2;  //Set clock source to SMCLK and enable reset
    UCA0CTL0 = UCCKPH + UCMSB + UCMST + UCSYNC;          //Enable USCI Module A0
    UCA0CTL1 &= ~UCSWRST;                                      //Disable reset

    /* --------- Setup ADC for XY ------ */
    // joystick input X (Pin 2, A0), Y (pin 5, A3)
    P1SEL |= BIT0 + BIT3;
    ADC10CTL0 &= ~ENC;                         // STOP SAMPLING
    ADC10CTL0 = SREF_0 | ADC10SHT_3 | REFON | ADC10ON | ADC10IE; // Set up ADC
    shiftX = 0;
    shiftY = 0;

    __bis_SR_register(GIE);               // Enable interrupts

    setPixel(myPlace[0], myPlace[1], 'g');
    __delay_cycles(1000000);
    sendBitmap(bitmap);

    int Xread;
    int Yread;
    Xread = 0;
    ADC10CTL0 &= ~ENC;                         // STOP SAMPLING
    Yread = 0;
    int shiftX;
    int shiftY;

    while (1)
    {

        Xread = readX();

        if (Xread > LEFTBOUND)
        {
            shiftX = 1;
            shiftPixel(myPlace[0], myPlace[1], shiftX, 0);
            sendBitmap(bitmap);
        }
        else if (Xread < RIGHTBOUND)
        {
            shiftX = -1;
            shiftPixel(myPlace[0], myPlace[1], shiftX, 0);
            sendBitmap(bitmap);
        }

        Yread = readY();

        if (shiftX == 0)
        {
            if (Yread < UPBOUND)
            {
                shiftY = 1;
                shiftPixel(myPlace[0], myPlace[1], 0, shiftY);
                sendBitmap(bitmap);

            }
            else if (Yread > DOWNBOUND)
            {
                shiftY = -1;
                shiftPixel(myPlace[0], myPlace[1], 0, shiftY);
                sendBitmap(bitmap);

            }
        }
        __bis_SR_register(LPM3_bits);         //Enter LPM1
    }
}

int readY(void)
{
    ADC10CTL1 |= INCH_3 | ADC10SSEL_1;        // A0 source, ACLK as source ADC10
    ADC10CTL0 |= ENC + ADC10SC;
    ADC10AE0 |= BIT3;                          // Using Pin for adc
    // Enable ADC10, start conversion
    __bis_SR_register(LPM3_bits + GIE);      // Enter LPM1 w/interrupt
    return ADC10MEM;
}

int readX(void)
{
    ADC10CTL1 |= INCH_0 | ADC10SSEL_1;        // A0 source, ACLK as source ADC10
    ADC10CTL0 |= ENC + ADC10SC;
    ADC10AE0 |= BIT0;                          // Using Pin for adc
    // Enable ADC10 A3, start conversion
    __bis_SR_register(LPM3_bits + GIE);      // Enter LPM1 w/interrupt
    return ADC10MEM;
}

/*
 * sends a specified bitmap to the display
 */
void sendBitmap(unsigned char* bitmapping)
{
    for (i = 3; i >= 0; i--)
    {
        UCA0TXBUF = 0x00;              //Send start frame
        __delay_cycles(10);
    }
    for (c = 0; c < DISPLAY_SIZE; c++)
    {        //Sends colors for each pixel in bitmap
        UCA0TXBUF = BRIGHTNESS;        //Send brightness byte on transmit buffer
        __delay_cycles(10);
        switch (bitmapping[c])
        {
        case 'r':       //Red
            UCA0TXBUF = 0x00;              //Send blue char on transmit buffer
            __delay_cycles(10);
            UCA0TXBUF = 0x00;              //Send green char on transmit buffer
            __delay_cycles(10);
            UCA0TXBUF = 0xFF;              //Send red char on transmit buffer
            __delay_cycles(10);
            break;
        case 'o':       //Orange
            UCA0TXBUF = 0x00;              //Send blue char on transmit buffer
            __delay_cycles(10);
            UCA0TXBUF = 0x3D;              //Send green char on transmit buffer
            __delay_cycles(10);
            UCA0TXBUF = 0xFF;              //Send red char on transmit buffer
            __delay_cycles(10);
            break;
        case 'y':       //Yellow
            UCA0TXBUF = 0x00;              //Send blue char on transmit buffer
            __delay_cycles(10);
            UCA0TXBUF = 0xEA;              //Send green char on transmit buffer
            __delay_cycles(10);
            UCA0TXBUF = 0xFF;              //Send red char on transmit buffer
            __delay_cycles(10);
            break;
        case 'g':       //Green
            UCA0TXBUF = 0x00;              //Send blue char on transmit buffer
            __delay_cycles(10);
            UCA0TXBUF = 0xFF;              //Send green char on transmit buffer
            __delay_cycles(10);
            UCA0TXBUF = 0x00;              //Send red char on transmit buffer
            __delay_cycles(10);
            break;
        case 'b':       //Blue
            UCA0TXBUF = 0xFF;              //Send blue char on transmit buffer
            __delay_cycles(10);
            UCA0TXBUF = 0x00;              //Send green char on transmit buffer
            __delay_cycles(10);
            UCA0TXBUF = 0x00;              //Send red char on transmit buffer
            __delay_cycles(10);
            break;
        case 'v':       //Violet
            UCA0TXBUF = 0x6B;              //Send blue char on transmit buffer
            __delay_cycles(10);
            UCA0TXBUF = 0x00;              //Send green char on transmit buffer
            __delay_cycles(10);
            UCA0TXBUF = 0x38;              //Send red char on transmit buffer
            __delay_cycles(10);
            break;
        case ' ':       //Off
            UCA0TXBUF = 0x00;              //Send blue char on transmit buffer
            __delay_cycles(10);
            UCA0TXBUF = 0x00;              //Send green char on transmit buffer
            __delay_cycles(10);
            UCA0TXBUF = 0x00;              //Send red char on transmit buffer
            __delay_cycles(10);
            break;
        case '1':       //White
            UCA0TXBUF = 0xFF;              //Send blue char on transmit buffer
            __delay_cycles(10);
            UCA0TXBUF = 0xFF;              //Send green char on transmit buffer
            __delay_cycles(10);
            UCA0TXBUF = 0xFF;              //Send red char on transmit buffer
            __delay_cycles(10);
            break;
        }
    }
}

/*
 * sets the color value of a pixel in a specified row and col
 */
void setPixel(int row, int col, char color)
{
    bitmap[(row) * 20 + col] = color;
}

/*
 * clears a pixel in a specified row and col
 */
void clearPixel(int row, int col)
{
    setPixel(row, col, ' ');
}

/*
 * returns the color value of the pixel at a specified row and col
 */
char readPixel(int row, int col)
{
    return bitmap[(row) * 20 + col];
}

/*
 * moves a pixel from a certain row and column to a destination row and column
 */
void movePixel(int row, int col, int dest_row, int dest_col)
{
    setPixel(dest_row, dest_col, readPixel(row, col));
    clearPixel(row, col);
}

/*
 * shifts a pixel in a specified row and col in a certain x and y direction.
 * x_shift and y_shift can be negative (- y_shift corresponds to going up)
 */
void shiftPixel(int row, int col, int x_shift, int y_shift)
{
    int final_shift_x = 0;
    int final_shift_y = 0;

    //x-direction

    while (x_shift > 20)
    {
        x_shift -= 20;
    }
    while (x_shift < -20)
    {
        x_shift += 20;
    }

    if ((x_shift + col) > 19)
    {
        final_shift_x = x_shift - 20;
    }
    else if ((x_shift + col) < 0)
    {
        final_shift_x = x_shift + (20 - col);
    }
    else
    {
        final_shift_x = x_shift;
    }

    //y-direction
    while (y_shift > 17)
    {
        y_shift -= 17;
    }
    while (y_shift < -17)
    {
        y_shift += 17;
    }
    if ((y_shift + row) > 16)
    {
        final_shift_y = y_shift - 17;
    }
    else if ((y_shift + row) < 0)
    {
        final_shift_y = y_shift + (17 - row);
    }
    else
    {
        final_shift_y = y_shift;
    }

    movePixel(row, col, row + final_shift_y, col + final_shift_x);
    myPlace[0] = row + final_shift_y;
    myPlace[1] = col + final_shift_x;
}

// Watchdog Timer interrupt service routine
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=WDT_VECTOR
__interrupt void watchdog_timer(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(WDT_VECTOR))) watchdog_timer (void)
#else
#error Compiler not supported!
#endif
{

    __bic_SR_register_on_exit(LPM3_bits);        // Exit low power mode 3
}

// USCI Transfer interrupt service routine
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=USCIAB0TX_VECTOR
__interrupt void USCIA0TX_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(USCIAB0TX_VECTOR))) USCIA0TX_ISR (void)
#else
#error Compiler not supported!
#endif
{
    IFG2 &= ~UCA0TXIFG;
    __bic_SR_register_on_exit(LPM1_bits);        // Exit low power mode 1
}

//#pragma vector=USCIAB0TX_VECTOR
//__interrupt void USCIA0_ISR(void)
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
    while ((ADC10CTL1 & ADC10BUSY) == ADC10BUSY)
        ;
    __bic_SR_register_on_exit(LPM3_bits);
}
