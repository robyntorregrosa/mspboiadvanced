/* Based on msp430g2xx3_wdt_02.c from the TI Examples */

#include <msp430.h>
#include <rand.h>

int c = 0;                                  //Char index
int i = 0;                                  //Led index
static int DISPLAY_SIZE = 340;               //number of pixels in display

void sendBitmap(unsigned char* bitmapping);
void setPixel(int row, int col, char color);
void clearPixel(int row, int col);
void movePixel(int row, int col, int dest_row, int dest_col);
void shiftPixel(int row, int col, int x_shift, int y_shift);
void setRandCoin();


int coincol;
int coinrow;
int myPlace[] = {0,0};
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
    int j = 0;
    int k = 0;
/*------------- Setup -------------*/
    BCSCTL3 |= LFXT1S_2;                      // ACLK = VLO
    WDTCTL = WDT_ADLY_16;                    // WDT 16ms, ACLK, interval timer
    //WDTCTL = WDT_ADLY_1_9;
    IE1 |= WDTIE;                            // Enable WDT interrupt
    IE2 |= UCA0TXIE;

/*------------- Seed LFSR -------------*/
    // seed generator
    ADC10CTL1 |= INCH_5 | ADC10SSEL_1;           // A5 source, ACLK as source ADC10
    ADC10CTL0 |= ADC10SHT_3 | ADC10ON | ADC10IE;
    ADC10CTL0 |= ENC + ADC10SC;               // Enable ADC10, start conversion
    __bis_SR_register(LPM3_bits + GIE);      // Enter LPM1 w/interrupt
    srand(ADC10MEM);                            // seed the lfsr

/*---------------SPI--------------*/
    //Multiplex Pin 1.2 to MOSI and Pin 1.4 to CLK
    P1DIR |= BIT2 + BIT4;                                       // Set P1.2 and P1.4 to output direction
    P1SEL = BIT2 + BIT4;
    P1SEL2 = BIT2 + BIT4;

    UCA0CTL1 = UCSWRST + UCSSEL_2;                             //Set clock source to SMCLK and enable reset
    UCA0CTL0 = UCCKPH + UCMSB + UCMST + UCSYNC;                //Enable USCI Module A0
    UCA0CTL1 &= ~UCSWRST;                                      //Disable reset

    __bis_SR_register(GIE);                   // Enable interrupts



//    setPixel(1,19,'g');
//
    sendBitmap(bitmap);
//    __delay_cycles(1000000);
//    shiftPixel(1,19,1, 0);
//    sendBitmap(bitmap);
//    __delay_cycles(1000000);
//    shiftPixel(1,0,1, 0);
//    sendBitmap(bitmap);
//    j = 1;
//    setRandCoin();
//    setPixel(coinrow,coincol,'y');

    while(1) {
//        if(j < 0){
//            j=16;
//        }
//        shiftPixel(j,1,0,-1);
//        j --;
        // check if you've gotten the coin
        if (myPlace[0] == coinrow && myPlace[1] == coincol){
            setPixel(coinrow, coincol, 'g');                        // yum!
            setRandCoin();                                          // set out the next coin
        }

//        if (myPlace[0] == 0 && myPlace[1] == 1){
//            setPixel(coinrow, coincol, 'g');
//            setRandCoin();
//        }

        /*
        j++;
        if(j > 19){
            j=0;
        }
        shiftPixel(j,1,0,-1);
        j--;
        */

        sendBitmap(bitmap);
        __bis_SR_register(LPM1_bits);         //Enter LPM1
    }

}


/*
 * sends a specified bitmap to the display
 */
void sendBitmap(unsigned char* bitmapping){
    for(i = 3; i >= 0; i--){
        UCA0TXBUF = 0x00;              //Send start frame
        __delay_cycles(10);
    }
    for(c = 0; c < DISPLAY_SIZE; c++){        //Sends colors for each pixel in bitmap
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
 * shifts a pixel in a specified row and col in a certain x and y direction.
 * x_shift and y_shift can be negative (- y_shift corresponds to going up)
 */
void shiftPixel(int row, int col, int x_shift, int y_shift){
    int final_shift_x = 0;
    int final_shift_y = 0;

    //x-direction

    while(x_shift > 20){
        x_shift -= 20;
    }
    while(x_shift < -20){
        x_shift += 20;
    }

    if((x_shift + col) > 19){
        final_shift_x = x_shift - 20;
    }else if((x_shift + col) < 0){
        final_shift_x = x_shift + (20-col);
    }else{
        final_shift_x = x_shift;
    }

    //y-direction
    while(y_shift > 17){
        y_shift -= 17;
    }
    while(y_shift < -17){
        y_shift += 17;
    }
    if((y_shift + row) > 16){
        final_shift_y = y_shift - 17;
    }else if((y_shift + row) < 0){
        final_shift_y = y_shift + (17-row);
    }else{
        final_shift_y = y_shift;
    }

    movePixel(row,col,row + final_shift_y, col + final_shift_x);
    myPlace[0] = row + final_shift_y;
    myPlace[1] = col + final_shift_x;
}
void setRandCoin(){
//    coinrow = 31;
//    coincol = 31;
    coinrow = rand() % 17;    // val between 0- 16
    coincol = rand() % 20;         // val between 0- 20
    setPixel(coinrow, coincol, 'y');
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

