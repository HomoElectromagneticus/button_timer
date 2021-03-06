/*
 * File:   main.c
 * Author: rschaub
 *
 *                   PIC16F1825
 *                    _______
 *           VDD ---|1      14|--- VSS
 *               ---|2      13|---
 *        CLKOUT ---|3      12|---
 *               ---|4      11|--- button input
 *               ---|5      10|--- shift register data
 *               ---|6       9|--- shift register clock 
 *               ---|7_______8|--- "on" light
 * 
 * This program times the duration a button is pressed, and displays that
 * duration on a pair of simple seven-segment displays. The duration is 
 * displayed as seconds with a tenth of a second after the decimal place. The 
 * displays are run using a pair of CD74HC595 shift registers (maximum clock 
 * rate is ~ 20MHz).
 * Created on April 17, 2019, 1:25 PM
 */

// CONFIG1
#pragma config FOSC = INTOSC    // Oscillator Selection (INTOSC oscillator: I/O function on CLKIN pin)
#pragma config WDTE = ON        // Watchdog Timer Enable (WDT disabled)
#pragma config PWRTE = ON       // Power-up Timer Enable (PWRT enabled)
#pragma config MCLRE = ON       // MCLR Pin Function Select (MCLR/VPP pin function is MCLR)
#pragma config CP = OFF         // Flash Program Memory Code Protection (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Memory Code Protection (Data memory code protection is disabled)
#pragma config BOREN = ON       // Brown-out Reset Enable (Brown-out Reset enabled)
#pragma config CLKOUTEN = ON   // Clock Out Enable (CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin)
#pragma config IESO = OFF       // Internal/External Switchover (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is disabled)

// CONFIG2
#pragma config WRT = OFF        // Flash Memory Self-Write Protection (Write protection off)
#pragma config PLLEN = OFF      // PLL Enable (4x PLL disabled)
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable (Stack Overflow or Underflow will cause a Reset)
#pragma config BORV = LO        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), low trip point selected.)
#pragma config LVP = OFF        // Low-Voltage Programming Enable (Low-voltage programming disabled)

#include "button_timer.h"

unsigned int display_index[10] = {
    0b00000011,         // 0
    0b10011111,         // 1
    0b00100101,         // 2
    0b00001101,         // 3
    0b10011001,         // 4
    0b01001001,         // 5
    0b01000001,         // 6
    0b00011111,         // 7
    0b00000001,         // 8
    0b00011001,         // 9
};

unsigned int button_state_integral = 0;
unsigned int button_debounce_threshold = 3;
unsigned int button_state_integral_max = 5;
unsigned char button_pushed_flag = 0;
unsigned int shift_register_data[2];
unsigned int display_update_scaler = 0;
unsigned int seconds_scaler = 0;         
unsigned int seconds_divisor = 25;     //set to overflow every tenth of a second
unsigned int seconds_clock = 0;
unsigned char PC_shadow = 0;      //shadow reg for PORTC to avoid R-M-W issues

void timer2_init(void) {
    // timer2 is used to clock the main program loop
    // the clock source is the system clock / 4, which is 2MHz
    T2CONbits.TMR2ON = 0;          //turn off timer2 for configuration
    PIR1bits.TMR2IF = 0;           //reset Timer2 overflow interrupt flag
    
    T2CONbits.T2CKPS = 0b01;       //pre-scaler set to 1:4
    T2CONbits.T2OUTPS = 0b1111;    //post-scaler set to 1:16
    TMR2 = 0;                      //clear timer2
    PR2 = 125;                     //set timer2 match reg to a "nice" value
    
    // the above sets the interrupt freq to ((((2MHz) / 4) / 125) / 16) = 250Hz
    T2CONbits.TMR2ON = 1;          //turn on Timer2
    
    return;
}

void tmr2_interrupt_handler(void){
    
    button_debouncer(PORTAbits.RA2);    //update the button state

    //keep counting the seconds for which button pushed?
    if (button_pushed_flag) {
        count_seconds();                //count up
        PC_shadow |= (1 << 2);
        PORTC = PC_shadow;
    } else {
        PC_shadow &= ~(1 << 2);
        PORTC = PC_shadow;
        seconds_scaler = 0;             //reset the clock
        seconds_clock = 0;              //reset the clock
    }
    
    // update the display at ~62.5Hz
    if (display_update_scaler < 8){
        update_display_values(seconds_clock);
        shiftout();
        display_update_scaler = 0;
    } else {
        display_update_scaler++;
    }

    return;
}

void count_seconds(void){     
    
    seconds_scaler++;           //increment the "seconds" clock

    if (seconds_scaler >= seconds_divisor){
        seconds_clock++;
        seconds_scaler = 0;
    }
    
    // prevent the seconds clock from overflowing
    if (seconds_clock == 100){
        seconds_clock = 0;
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
    
    //turn on the decimal point for the tens place
    shift_register_data[1] &= ~(1 << 0);
    
    return;
};

void pulse_sr_clock(void){
    PC_shadow |= (1 << 1);
    PORTC = PC_shadow;
    PC_shadow &= ~(1 << 1);
    PORTC = PC_shadow;
    return;
}

void shiftout(void){
    // toggle the shift register's clock line 16 times to put in the new
    // display values
    
    unsigned char w;  
    
    for (int i = 1; i >= 0; i--){
        for (int j = 0; j <= 7; j++) {
            // extract the relevant bit
            w = (shift_register_data[i] >> j) & 1;
            // put the value on the shift reg's data-in line (via shadow reg)
            PC_shadow ^= (-w ^ PC_shadow) & 1;
            PORTC = PC_shadow;
            // pulse the shift register's clock line. the system clock is 2MHz, 
            // and the old '595 shift registers can run up 20MHz, so this "dumb"
            // approach should be fine here
            pulse_sr_clock();
        }
    }
    
    // pulse the clock line once more to account for the lag inherent in the
    // '595 shift registers when SCLK and RCLK are tied together
    pulse_sr_clock();

    return;
}

int button_debouncer(unsigned int button_state){
    
    // "integrate" the state of the button over time
    if (button_state == 1 && button_state_integral < button_state_integral_max){
        button_state_integral++;
    } else if (button_state == 0 && button_state_integral > 0) {
        button_state_integral--;
    }
    
    // set the flag accordingly
    if (button_state_integral >= button_debounce_threshold){
        button_pushed_flag = 1;
    } else {
        button_pushed_flag = 0;
    }
    return 0;
}

void main(void) {
    
    // configure the internal program clock to run at 8MHz
    OSCCONbits.SPLLEN = 0b0;    // 4xPLL disabled
    OSCCONbits.IRCF = 0b1110;   // HFINTOSC set to 16MHz
    OSCCONbits.SCS = 0b00;      // clock source set by FOSC config word

    // configure the watchdog timer
    WDTCONbits.WDTPS = 0b01011; //set to 2s timer
    
    // configure the inputs and outputs
    TRISAbits.TRISA2 = 1;       //set RA2 (pin 11) as input
    TRISCbits.TRISC0 = 0;       //set RC0 (pin 10) as output
    TRISCbits.TRISC1 = 0;       //set RC1 (pin 9) as output
    TRISCbits.TRISC2 = 0;       //set RC2 (pin 8) as output
    ANSELA = 0b00000000;        //no analog inputs in this program!
    ANSELC = 0b00000000;        //no analog inputs in this program!
    
    // turn on interrupts
    PIE1bits.TMR2IE = 1;      //enable timer2 to PR2 match interrupt
    INTCONbits.PEIE = 1;      //enable peripheral interrupts
    INTCONbits.GIE = 1;       //general interrupts enabled
    
    // configure the timers so that the button duration can be counted with
    // some kind of accuracy
    timer2_init();
        
    while(1){
        CLRWDT();               //clear the Watchdog Timer to keep the PIC from
                                //resetting. sadly the program won't get here
                                //if the button is pressed...
    }
    return;
}

void interrupt ISR(void){
    // check for timer2 overflow interrupt
    if(PIR1bits.TMR2IF == 1){
        tmr2_interrupt_handler();
        PIR1bits.TMR2IF = 0;                 //reset the interrupt flag
    }
      
    return;
}