/* Based on msp430g2xx3_wdt_02.c from the TI Examples */

#include <msp430.h>

int c = 0;                                  //Char index
int i = 0;                                  //Led index

void sendBitmap(char* bitmapping);
void setPixel(int row, int col, char color);
void clearPixel(int row, int col);
void movePixel(int row, int col, int dest_row, int dest_col);
//void shiftPixel(int row, int col, int dir, int num_times);    UNCOMMMENT WHEN shiftPixel() is complete


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

/*------------- Setup -------------*/
    BCSCTL3 |= LFXT1S_2;                      // ACLK = VLO
    WDTCTL = WDT_ADLY_16;                    // WDT 16ms, ACLK, interval timer
    //WDTCTL = WDT_ADLY_1_9;
    IE1 |= WDTIE;                            // Enable WDT interrupt
    IE2 |= UCA0TXIE;

/*---------------SPI--------------*/
    //Multiplex Pin 1.2 to MOSI and Pin 1.4 to CLK
    P1DIR |= BIT2 + BIT4;                                       // Set P1.2 and P1.4 to output direction
    P1SEL = BIT2 + BIT4;
    P1SEL2 = BIT2 + BIT4;

    UCA0CTL1 = UCSWRST + UCSSEL_2;                             //Set clock source to SMCLK and enable reset
    UCA0CTL0 = UCCKPH + UCMSB + UCMST + UCSYNC;                //Enable USCI Module A0
    UCA0CTL1 &= ~UCSWRST;                                      //Disable reset

    __bis_SR_register(GIE);                   // Enable interrupts



    setPixel(4,10,'g');
    movePixel(8,9,9,10);

    while(1) {
        sendBitmap(bitmap);

        __bis_SR_register(LPM1_bits);         //Enter LPM1
    }

}


/*
 * sends a specified bitmap to the display
 */
void sendBitmap(char* bitmapping){
    for(i = 3; i >= 0; i--){
        UCA0TXBUF = 0x00;              //Send start frame
        __delay_cycles(10);
    }
    for(c = 0; c < sizeof(bitmapping); c++){        //Sends colors for each pixel in bitmap
        UCA0TXBUF = BRIGHTNESS;              //Send brightness byte on transmit buffer
        __delay_cycles(10);
        switch(bitmapping[c]){
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
void setPixel(int row, int col, char color){
    bitmap[(row)*20 + col] = color;
}

/*
 * clears a pixel in a specified row and col
 */
void clearPixel(int row, int col){
    setPixel(row,col,' ');
}

/*
 * returns the color value of the pixel at a specified row and col
 */
char readPixel(int row, int col){
    return bitmap[(row)*20 + col];
}


/*
 * moves a pixel from a certain row and column to a destination row and column
 */
void movePixel(int row, int col, int dest_row, int dest_col){
    setPixel(dest_row, dest_col, readPixel(row,col));
    clearPixel(row,col);
}

/*
 * TODO: FINISH function and add wraparound cases (if too much shift one way, comes back around the other way)
 * shifts a pixel in a specified row and col in a certain direction and a certain num of times
 * dir:
 * 0 = up
 * 1 = up-right
 * 2 = right
 * 3 = down-right
 * 4 = down
 * 5 = down-left
 * 6 = left
 * 7 = up-left
 *
void shiftPixel(int row, int col, int dir, int num_times){
    if(dir == 0 || dir == 1){

    }

    switch(dir){
    case 0:             //up
        movePixel(row,col,row + num_times,col);
        break;
    case 1:             //up-right
        movePixel(row,col,row,col+num_times);
        break;
    case 2:             //right
        movePixel(row,col,row,col+num_times);
        break;
    case 3:             //down-right
        movePixel(row,col,row,col+num_times);
        break;
    case 4:             //down
        movePixel(row,col,row,col+num_times);
        break;
    case 5:             //down-left
        movePixel(row,col,row,col+num_times);
        break;
    case 6:             //left
        movePixel(row,col,row,col+num_times);
        break;
    case 7:             //up-left
        movePixel(row,col,row,col+num_times);
        break;
    }
}
*/

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


