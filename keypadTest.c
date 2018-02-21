/*
 * File:   keypadTest.c
 * Author: llwyd
 *
 * Created on 18 February 2018, 17:29
 */
// CONFIG1
#pragma config FOSC = INTOSC        // Oscillator Selection Bits (HS Oscillator, High-speed crystal/resonator connected between OSC1 and OSC2 pins)
#pragma config WDTE = OFF       // Watchdog Timer Enable (WDT disabled)
#pragma config PWRTE = ON     // Power-up Timer Enable (PWRT disabled)
#pragma config MCLRE = ON       // MCLR Pin Function Select (MCLR/VPP pin function is MCLR)
#pragma config CP = OFF         // Flash Program Memory Code Protection (Program memory code protection is disabled)
#pragma config BOREN = OFF      // Brown-out Reset Enable (Brown-out Reset disabled)
#pragma config CLKOUTEN = OFF   // Clock Out Enable (CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin)
#pragma config IESO = ON        // Internal/External Switchover Mode (Internal/External Switchover Mode is enabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is disabled)

// CONFIG2
#pragma config WRT = OFF        // Flash Memory Self-Write Protection (Write protection off)
#pragma config PPS1WAY = ON     // Peripheral Pin Select one-way control (The PPSLOCK bit cannot be cleared once it is set by software)
#pragma config ZCD = OFF        // Zero-cross detect disable (Zero-cross detect circuit is disabled at POR)
#pragma config PLLEN = ON       // Phase Lock Loop enable (4x PLL is always enabled)
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable (Stack Overflow or Underflow will cause a Reset)
#pragma config BORV = LO        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), low trip point selected.)
#pragma config LPBOR = OFF      // Low-Power Brown Out Reset (Low-Power BOR is disabled)
#pragma config LVP = ON         // Low-Voltage Programming Enable (Low-voltage programming enabled)


#include <xc.h>
#include <pic16f1779.h>

#define _XTAL_FREQ 32000000

//Function Prototypes
void setColumnsInput(void);
void setColumnsOutput(void);
void setRowsInput(void);
void setRowsOutput(void);

//D rows
//C columns

void setColumnsOutput(void){
    TRISC=0x00;
    LATC=0x00;
}
void setColumnsInput(void){
    TRISC=0xFF;
}
void setRowsInput(void){
    TRISD=0xFF;
}
void setRowsOutout(void){
    TRISD=0x00;
    LATD=0x00;
}
void main(void) {
    //Configure Clock 32MHz
    OSCCON=0b11110000;
    TRISB=0x00;//output
    ANSELD=0x00;//Digital reads only
    ANSELC=0x00;//Digital reads only
    TRISBbits.TRISB4=1;
    setColumnsOutput();
    setRowsInput();
    while(1){
        setColumnsOutput();
        setRowsInput();
        if(PORTDbits.RD0==0){
            setColumnsInput();
            setRowsOutout();
            if(PORTCbits.RC0==0){
                LATBbits.LATB5=1;
            }
        }
        else{
            LATBbits.LATB5=0;
        }
        //LATB^=0xFF;
        //__delay_ms(500);
    }
    return;
}
