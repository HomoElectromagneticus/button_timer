/*
 * File:   main.c
 * Author: rschaub
 *
 *                   PIC16F1825
 *                    _______
 *           VDD ---|1      14|--- VSS
 *               ---|2      13|---
 *               ---|3      12|---
 *               ---|4      11|--- button input
 *               ---|5      10|--- shift register data
 *               ---|6       9|--- shift register clock 
 *               ---|7_______8|--- "on" light
 * 
 * This program times the duration a button is pressed, and displays that
 * duration on a pair of simple seven-segment displays. The displays are run
 * using a pair of CD74HC595 shift register (maximum clock rate is ~ 20MHz).
 * Created on April 17, 2019, 1:25 PM
 */

/*
 * 0 = 0b11111100
 * 1 = 0b01100000
 * 2 = 0b11011010
 * 3 = 0b11110010
 * 4 = 0b01100110
 * 5 = 0b10110110
 * 6 = 0b10111110
 * 7 = 0b11100000
 * 8 = 0b11111110
 * 9 = 0b11100110
 */

// CONFIG1
#pragma config FOSC = INTOSC    // Oscillator Selection (INTOSC oscillator: I/O function on CLKIN pin)
#pragma config WDTE = OFF       // Watchdog Timer Enable (WDT enabled)
#pragma config PWRTE = ON       // Power-up Timer Enable (PWRT enabled)
#pragma config MCLRE = ON       // MCLR Pin Function Select (MCLR/VPP pin function is MCLR)
#pragma config CP = OFF         // Flash Program Memory Code Protection (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Memory Code Protection (Data memory code protection is disabled)
#pragma config BOREN = ON       // Brown-out Reset Enable (Brown-out Reset enabled)
#pragma config CLKOUTEN = OFF   // Clock Out Enable (CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin)
#pragma config IESO = OFF       // Internal/External Switchover (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is disabled)

// CONFIG2
#pragma config WRT = OFF        // Flash Memory Self-Write Protection (Write protection off)
#pragma config PLLEN = ON       // PLL Enable (4x PLL enabled)
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable (Stack Overflow or Underflow will cause a Reset)
#pragma config BORV = LO        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), low trip point selected.)
#pragma config LVP = OFF        // Low-Voltage Programming Enable (Low-voltage programming disabled)

// CONFIG3
#define _XTAL_FREQ 8000000

#include "button_timer.h"

const unsigned int display_index[10] = {
    0b11111100,         // 0
    0b01100000,         // 1
    0b11011010,         // 2
    0b11110010,         // 3
    0b01100110,         // 4
    0b10110110,         // 5
    0b10111110,         // 6
    0b11100000,         // 7
    0b11111110,         // 8
    0b11100110,         // 9
};

unsigned int button_state_integral = 0;
unsigned int button_debounce_threshold = 8;
unsigned int button_state_integral_max = 16;
unsigned char button_pushed_flag = 0;
unsigned int shift_register_data[2] = {0b01100110, 0b11011010};
unsigned short long timer_value = 0;
unsigned short long timer1_period = 125000;
unsigned int seconds_clock = 0;
unsigned char PC_shadow = 0;      //"shadow reg for PORTC to avoid R-M-W issues

void timer2_init(void) {
    // timer2 is used to clock the main program loop
    // the clock source is the system clock / 4 (2MHz)
    T2CONbits.TMR2ON = 0;          //turn off timer2 for configuration
    PIR1bits.TMR2IF = 0;           //reset Timer2 overflow interrupt flag
    
    T2CONbits.T2CKPS = 0b10;       //pre-scaler set to 1:16
    T2CONbits.T2OUTPS = 0b1011;    //post-scaler set to 1:12
    TMR2 = 0;                      //clear timer2
    PR2 = 0xFF;                    //set timer2 "match" register to max value
    
    // the above sets the interrupt freq to (((FOSC/4) / 16) / 12) = 10.42KHz
    T2CONbits.TMR2ON = 1;          //turn on Timer2
    
    return;
}

void timer1_init(void) {
    // timer1 is used to measure the duration of the button press
    // the clock source is the system clock / 4 (2MHz)
    T1CONbits.TMR1ON = 0;          //turn off timer2 for configuration
    T1CONbits.TMR1CS = 0b00;       //use the internal system clock (FOSC / 4)
    T1CONbits.T1CKPS = 0b11;       //prescaler set to 1:8
    TMR1 = 0;                      //clear timer1
    
    //disable the gate
    T1GCONbits.TMR1GE = 0;
    
    // the above sets the interrupt freq to ((FOSC/4) / 8) = 125KHz
    
    return;
}

void tmr2_interrupt_handler(void){
    button_debouncer(PORTAbits.RA2);    //update the button state

    //keep counting the seconds for which button pushed?
    if (button_pushed_flag) {
        T1CONbits.TMR1ON = 1;           //start timer1
        PC_shadow |= (1 << 2); 
        PORTC = PC_shadow;              //turn on the light next to the button
    } else {
        T1CONbits.TMR1ON = 0;           //stop timer1
        TMR1 = 0;                       //reset timer1
        timer_value = 0;                //reset the "timer1 post-scaler"
        seconds_clock = 0;
        PC_shadow &= ~(1 << 2);
        PORTC = PC_shadow;              //turn off the light next to the button
    }
    
    //update the shift register data
    update_display_values(seconds_clock);
    //shift out the new data
    shiftout();
    
    PIR1bits.TMR2IF = 0;                 //reset the interrupt flag
    return;
}

void tmr1_interrupt_handler(void){
    //increment the "seconds" clock
    timer_value++;

    if (timer_value >= timer1_period){
        seconds_clock++;
        timer_value = 0;
    }
    
    return;
}

void update_display_values(unsigned int value){
    
    unsigned int tens;
    unsigned int ones;
    
    value = value % 100;                    //remove the hundreds place
    tens = (unsigned int) value / 10;       //intentionally using int division
    ones = (unsigned int) value % 10;          
    
    shift_register_data[1] = display_index[tens];
    shift_register_data[0] = display_index[ones];
    
    return;
};


void shiftout(void){
    // toggle the shift register's clock line 16 times to put in the new
    // display values
    
    unsigned char w;
    
    for (int i = 1; i >= 0; i--){
        for (int j = 7; j >= 0; j--) {
            // extract the relevant bit
            w = (shift_register_data[i] >> j) & 1;
            // put the value on the shift register's data-in line (via shadow reg)
            PC_shadow ^= (-w ^ PC_shadow) & 1;
            // pulse the shift register's clock line. the system clock is 2MHz, 
            // and the old '595 shift registers can run up 20MHz, so this "dumb"
            // approach should be fine here
            PC_shadow |= (1 << 1);
            PORTC = PC_shadow;          //write PORTC all at once to avoid RMW
            __delay_us(1);
            PC_shadow &= ~(1 << 1);
            PORTC = PC_shadow;
            __delay_us(1);
        }
    }

    // pulse the clock line once more to account for the lag inherent in the
    // '595 shift registers when SCLK and RCLK are tied together
    PC_shadow |= (1 << 1);
    PORTC = PC_shadow;
    __delay_us(1);
    PC_shadow &= ~(1 << 1);
    PORTC = PC_shadow;
    
    return;
}

int button_debouncer(unsigned int button_state){
    
    // "integrate" the state of the button over time
    if (button_state == 1 && button_state_integral < button_state_integral_max){
        button_state_integral++;
    } else if (button_state == 0 && button_state_integral > 0) {
        button_state_integral--;
    }
    
    if (button_state_integral >= button_debounce_threshold){
        button_pushed_flag = 1;
    } else {
        button_pushed_flag = 0;
    }
    return 0;
}

void main(void) {
    // configure the inputs and outputs
    TRISAbits.TRISA2 = 1;       //set RA2 (pin 11) as input
    TRISCbits.TRISC0 = 0;       //set RC0 (pin 10) as output
    TRISCbits.TRISC1 = 0;       //set RC1 (pin 9) as output
    TRISCbits.TRISC2 = 0;       //set RC2 (pin 8) as output
    ANSELA = 0b00000000;        //no analog inputs in this program!
    ANSELC = 0b00000000;        //no analog inputs in this program!
    
    // configure the internal program clock to run at 8MHz
    OSCCONbits.IRCF = 0b1110;   // HFINTOSC set to 8MHz
    OSCCONbits.SCS = 0b00;      // clock source is internal
    OSCCONbits.SPLLEN = 0b0;    // 4xPLL disabled
    
    // turn on interrupts
    PIE1bits.TMR2IE = 1;      //enable timer2 to PR2 match interrupt (PR2 is 0xFF by default)
    PIE1bits.TMR1IE = 1;      //enable timer1 overflow interrupt
    INTCONbits.PEIE = 1;      //enable peripheral interrupts
    INTCONbits.GIE = 1;       //general interrupts be on
    
    // configure the timers so that the button duration can be counted with
    // some kind of accuracy
    timer2_init();
    timer1_init();
        
    while(1){
        CLRWDT();               //clear the Watchdog Timer to keep the PIC from
                                //resetting
    }
    return;
}

void interrupt ISR(void){
    // check for timer2 overflow interrupt
    if(PIR1bits.TMR2IF == 1){
        tmr2_interrupt_handler();
    }
    
    // check for timer1 overflow interrupt
    if(PIR1bits.TMR1IF == 1){
        tmr1_interrupt_handler();
    }
    
    return;
}