/* Based on msp430g2xx3_wdt_02.c from the TI Examples */

#include <msp430.h>
#include <rand.h>

int c = 0;                                      //Char index
int i = 0;                                      //Led index
static int DISPLAY_SIZE = 340;                  //number of pixels in display
#define UPPER 100

void sendBitmap(unsigned char* bitmapping);
void setPixel(int row, int col, char color);
void clearPixel(int row, int col);
char readPixel(int row, int col);
void movePixel(int row, int col, int dest_row, int dest_col);
void shiftPixel(int row, int col, int x_shift, int y_shift);
void setRandCoin(char color);
int readX(void);
int readY(void);
void wait(int t);


int myPlace[] = {9,10};
char nextColor;
int resetflag = 1;

int coincol;
int coinrow;
static char BRIGHTNESS = 0xE1;                 //Set LED brightness for whole bitmap

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
 ' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',   //8
 ' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',   //9
 ' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',   //10
 ' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',   //11
 ' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',   //12
 ' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',   //13
 ' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',   //14
 ' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',   //15
 ' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',   //16
};
int adc[6] = { 0 };  // This will hold the x,y and z axis values
unsigned int X_Axis = 0;
unsigned int Y_Axis = 0;
unsigned int seed = 0;

void readXY(void);

int main(void)
{
    int Xread;
    int Yread;
    Xread = 0;
    Yread = 0;
    int shiftX;
    int shiftY;
    shiftX = 0;
    shiftY = 0;

    static const int LEFTBOUND = 610;
    static const int RIGHTBOUND = 420;
    static const int UPBOUND = 215;
    static const int DOWNBOUND = 815;
    /*------------- Setup -------------*/
    BCSCTL3 |= LFXT1S_2;                                        // ACLK = VLO
    WDTCTL = WDT_ADLY_16;                                       // WDT 16ms, ACLK, interval timer
    //WDTCTL = WDT_ADLY_1_9;
    IE1 |= WDTIE;                                               // Enable WDT interrupt
    IE2 |= UCA0TXIE;

    // setup timer A0 for timing length
    TA0CTL = TASSEL_2 | MC_1 | ID_3;            // SMCLK from VLO 12KHz, up mode, Timer A0
    TA0CCR0 = UPPER>>3;                         // 1/8 MHz clock, divide by 16

    /*---------------SPI--------------*/
                                                                //Multiplex Pin 1.2 to MOSI and Pin 1.4 to CLK
    P1DIR |= BIT2 + BIT4;                                       // Set P1.2 and P1.4 to output direction
    P1SEL = BIT2 + BIT4;
    P1SEL2 = BIT2 + BIT4;

    UCA0CTL1 = UCSWRST + UCSSEL_2;                              //Set clock source to SMCLK and enable reset
    UCA0CTL0 = UCCKPH + UCMSB + UCMST + UCSYNC;                 //Enable USCI Module A0
    UCA0CTL1 &= ~UCSWRST;                                       //Disable reset

    /*------------- Seed LFSR -------------*/
    // seed generator
    ADC10CTL0 &= ~ENC;                                          // STOP SAMPLING
    ADC10CTL1 |= INCH_5 | ADC10SSEL_1 | CONSEQ_1;               // A5 source, ACLK as source ADC10
    ADC10CTL0 |= ADC10SHT_3 | ADC10ON | ADC10IE | MSC;
    ADC10AE0  |= BIT5;
    ADC10CTL0 |= ENC | ADC10SC;                                 // Sampling and conversion start
    ADC10DTC1 = 0x6;                                           // 3 conversions
    ADC10AE0  |= 0b101001;                                      // Disable digital I/O on P1.0 to P1.2
    __bis_SR_register(LPM3_bits + GIE);                         // Enter LPM1 w/interrupt

    readXY();
    srand(seed);

    sendBitmap(bitmap);

    wait(5);
    while (1) {
        if (resetflag) {
         resetflag = 0;
         // setup stuff
         setRandCoin('r');               // evil berry
         unsigned int c;
         for (c = 10; c>0; c--){
             setRandCoin('b');           // yummy berries
         }

         setPixel(myPlace[0], myPlace[1], 'g');      // me!

         sendBitmap(bitmap);
         resetflag = 0;
        }
        shiftX=0;
        readXY();
        Xread = adc[0];
        Yread = adc[2];

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
void readXY(void)
{
    ADC10CTL0 &= ~ENC;
    while (ADC10CTL1 & BUSY);               // Wait if ADC10 core is active
    ADC10SA = (int) adc;           // Copies data in ADC10SA to unsigned int adc array
    ADC10CTL0 |= ENC + ADC10SC;             // Sampling and conversion start

    X_Axis = adc[0];                        // adc array 0 copied to the variable X_Axis
    Y_Axis = adc[2];                        // adc array 1 copied to the variable Y_Axis
    seed = adc[5];                           // adc array 2 copied to the variable Z_Axis
    __bis_SR_register(LPM3_bits + GIE);        // LPM0, ADC10_ISR will force exit
}

/*
 * sends a specified bitmap to the display
 */
void sendBitmap(unsigned char* bitmapping)
{
    for (i = 3; i >= 0; i--)
    {
        UCA0TXBUF = 0x00;              //Send start frame
        wait(1);
    }
    for (c = 0; c < DISPLAY_SIZE; c++)
    {        //Sends colors for each pixel in bitmap
        UCA0TXBUF = BRIGHTNESS;        //Send brightness byte on transmit buffer
        wait(1);
        switch (bitmapping[c])
        {
        case 'r':       //Red
            UCA0TXBUF = 0x00;              //Send blue char on transmit buffer
            wait(2);
            UCA0TXBUF = 0x00;              //Send green char on transmit buffer
            wait(2);
            UCA0TXBUF = 0xFF;              //Send red char on transmit buffer
            wait(2);
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

    // check if the next pixel is coin!
    nextColor = readPixel(row + final_shift_y,col + final_shift_x);
    if (nextColor == 'b') {
       setPixel(myPlace[0], myPlace[1], 'g');                        // yum!
       setRandCoin('b');                                          // set out the next coin
    }
    if (nextColor == 'r') {
        // you lose :(
        setPixel(myPlace[0], myPlace[1], 'r');                        // set self red
        unsigned int r;
        for (r = DISPLAY_SIZE; r > 0; r--){
            setRandCoin('r');                                    // lose sequence
            sendBitmap(bitmap);
            wait(2);
        }
        // button to reset
        unsigned int cc;
        for (r = 17; r > 0; r--){
            for (cc = 20; cc > 0; cc--){
            clearPixel(r-1, cc-1); // lose sequence
            sendBitmap(bitmap);
            wait(2);
            }
            resetflag = 1;
        }


    }

    movePixel(row, col, row + final_shift_y, col + final_shift_x);
    myPlace[0] = row + final_shift_y;
    myPlace[1] = col + final_shift_x;
}

//int checkNextPixel()
void setRandCoin(char color)
{
//    coinrow = 31;
//    coincol = 31;
    coinrow = rand() % 17;    // val between 0- 16
    coincol = rand() % 20;         // val between 0- 20
    setPixel(coinrow, coincol, color);
}

void wait(int t){
    int i;
    for (i = t; i >0; i--){
        TA0CCTL0 = CCIE; // enable note duration interrupt
        __bis_SR_register(LPM0_bits + GIE);     // Enter LPM0 w/interrupt
    }
}

// Timer A0 interrupt service routine
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=TIMER0_A0_VECTOR
__interrupt void Timer_A0 (void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(TIMER0_A1_VECTOR))) Timer_A (void)
#else
#error Compiler not supported!
#endif
{
    TA0CCTL0 &= ~CCIE;                       // disable timer A0 interrupt (note duration)
    __bic_SR_register_on_exit(LPM0_bits);
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
    __bic_SR_register_on_exit(LPM3_bits);
}
