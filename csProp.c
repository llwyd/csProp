/*
 * File:   csProp.c
 * Author: llwyd
 *
 * Created on 18 February 2018, 17:29
 */
// CONFIG1
#pragma config FOSC = INTOSC        // Oscillator Selection Bits (HS Oscillator, High-speed crystal/resonator connected between OSC1 and OSC2 pins)
#pragma config WDTE = OFF       // Watchdog Timer Enable (WDT disabled)
#pragma config PWRTE = OFF     // Power-up Timer Enable (PWRT disabled)
#pragma config MCLRE = ON       // MCLR Pin Function Select (MCLR/VPP pin function is MCLR)
#pragma config CP = OFF       // Flash Program Memory Code Protection (Program memory code protection is disabled)
#pragma config BOREN = ON      // Brown-out Reset Enable (Brown-out Reset disabled)
#pragma config CLKOUTEN = OFF   // Clock Out Enable (CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin)
#pragma config IESO = ON        // Internal/External Switchover Mode (Internal/External Switchover Mode is enabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is disabled)

// CONFIG2
#pragma config WRT = OFF        // Flash Memory Self-Write Protection (Write protection off)
#pragma config PPS1WAY = OFF     // Peripheral Pin Select one-way control (The PPSLOCK bit cannot be cleared once it is set by software)
#pragma config ZCD = OFF        // Zero-cross detect disable (Zero-cross detect circuit is disabled at POR)
#pragma config PLLEN = ON       // Phase Lock Loop enable (4x PLL is always enabled)
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable (Stack Overflow or Underflow will cause a Reset)
#pragma config BORV = HI        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), low trip point selected.)
#pragma config LPBOR = ON      // Low-Power Brown Out Reset (Low-Power BOR is disabled)
#pragma config LVP = ON         // Low-Voltage Programming Enable (Low-voltage programming enabled)


#include <xc.h>
#include <pic16f1779.h>

#define _XTAL_FREQ 32000000

//Function Prototypes

//KeyPad
void setColumnsInput(void);
void setColumnsOutput(void);
void setRowsInput(void);
void setRowsOutput(void);
char scanCols(void);
char scan(void);
//LCD
void initialiseLCD(void);
void lcdWrite(void);
void writeChar(char c);
//Piezo

//DefuseButton?

//Finite State Machine
typedef enum{
    INPUT,
    TIMER,
}state;

//D rows
//C columns

//char arrays detailing number characters
char left[4]={'1','4','7','*'};
char mid[4]={'2','5','8','0'};
char right[4]={'3','6','9','#'};

volatile char tc;
volatile char ticks;
void __interrupt(high_priority) myIsr(void)
{
    if (TMR0IE && TMR0IF) {
        
        tc++;
                if(tc==61){
                    tc=0;
                    ticks++;
                    LATAbits.LATA0^=1;
                }
        TMR0IF=0;
    }
    return;
}

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
char scanCols(void){
    setColumnsInput();
    setRowsOutout();
    if(PORTCbits.RC0==0){
    //    LATBbits.LATB5=1;
        return 0;
    }
    else if(PORTCbits.RC1==0){
    //    LATBbits.LATB5=1;
        return 1;
    }
    else if(PORTCbits.RC2==0){
    //    LATBbits.LATB5=1;
        return 2;
    }
    else if(PORTCbits.RC3==0){
    //    LATBbits.LATB5=1;
        return 3;
    }
}
char scan(void){
    char colVal=0;
    char keyVal=0xFF;
    setColumnsOutput();
    setRowsInput();
    if(PORTDbits.RD0==0){
        colVal=scanCols();
        keyVal=left[colVal];
    }
    else if(PORTDbits.RD1==0){
        colVal=scanCols();
        keyVal=mid[colVal];
    }
    else if(PORTDbits.RD2==0){
        colVal=scanCols();
        keyVal=right[colVal];
    }
    else{
    //    LATBbits.LATB5=0;
    }
    return keyVal;
}



void setLCD(char lcdVal){  
    char delay=15;
    LATBbits.LATB4=1;
    __delay_ms(delay);
    //RS=0;
    LATBbits.LATB5=(lcdVal&0b00010000)>>4;
    LATBbits.LATB3=(lcdVal&0b00001000)>>3;
    LATBbits.LATB2=(lcdVal&0b00000100)>>2;
    LATBbits.LATB1=(lcdVal&0b00000010)>>1;
    LATBbits.LATB0=(lcdVal&0b00000001)>>0;
    __delay_ms(delay);
    LATBbits.LATB4=0;
    __delay_ms(delay);
}

void initialiseLCD(void){
    //RB5=Register select (RS)
    //RB4=Enable
    //RB3-0 =D4-7;
    //Initialise
    setLCD(0b00010);
    setLCD(0b00010);
    setLCD(0b00000);
    setLCD(0b01110);
    setLCD(0b00000);
    setLCD(0b00110);
}
void writeChar(char c){
    char outHigh=(0b10000)|(c>>4);
    char outLow=(0b10000)|(c&0xF);
    setLCD(outHigh);
    setLCD(outLow); 
}
void clearLCD(void){
    setLCD(0b00000);
    setLCD(0b00001);
}
void removeCursor(void){
    setLCD(0b00000);
    setLCD(0b01100);
}
void setCursor(void){
    setLCD(0b00000);
    setLCD(0b01110);
}
void configureTimer(void){
    TMR0=0x00;
    //enable timer 1 interrupt
    INTCONbits.TMR0IE=0;
    //prescalar
    OPTION_REG=0b00000111;
}
void configurePWM(void){
    //PWM3 to be used;
    //RA1 as PWM3 out
    GIE = 0;
    PPSLOCK = 0x55;
    PPSLOCK = 0xAA;
    PPSLOCK = 0;
    RA1PPS=0b011001;
    PPSLOCK = 0x55;
    PPSLOCK = 0xAA;
    PPSLOCK = 1;
    GIE = 1;  
    TRISAbits.TRISA1=0;
    //Reset Timer
    
    T2CON=0x02;
    TMR2=0x0;
    T2PR=0x45;
    T2CLKCON=0x01;
    //T2CONbits.CKPS=0b111;
    //Turn on the timer
    T2CONbits.ON=1;
    //
    CCPTMRS2bits.P3TSEL=0b00;
    //Enable PWM3
    PWM3CONbits.POL=0;
    PWM3DCH=0b00011111;
    PWM3DCL=0b11000000;
    PWM3CONbits.EN=1;


}


void main(void) {
    //Configure Clock 32MHz
    OSCCON=0b11110000;
    //pwmRedux();
    configurePWM();
    TRISB=0x00;// B OUTPUT
    //PORTB=0x00;
    TRISAbits.TRISA0=0;
    //TRISA=0x00;// A Output;
    ANSELD=0x00;//Digital reads only D
    ANSELC=0x00;//Digital reads only C
    //ANSELA=0x00;
    //TRISBbits.TRISB4=1;
    setColumnsOutput();
    setRowsInput();
    char pos=0;
    char count=0;
    __delay_ms(250);
    setLCD(0b00011);
    setLCD(0b00010);
    __delay_ms(100);
    initialiseLCD();
    clearLCD();
    __delay_ms(100);
    state s=INPUT;
    //enable global interrupts
    ei();
    configureTimer();
    ticks=0;
    //configurePWM();
    char keyVal=0xFF;
    while(1){
        switch (s)
        {
            case INPUT:
                keyVal=0xFF;
                keyVal=scan();
                if(keyVal!=0xFF){
                    //Check if activation key * has been pressed
                    if(keyVal=='*'){
                        clearLCD();
                        for (int i=0;i<pos-1;i++){
                            writeChar('*');
                        }
                        removeCursor();
                        pos=0;
                        s=TIMER;
                        INTCONbits.TMR0IE=1;
                    }
                    writeChar(keyVal);
                    pos++;
                    if(pos==16){
                        clearLCD();
                        pos=0;
                    }
                    __delay_ms(200);
                    
                }
                break;
            case TIMER:
//                LATAbits.LATA0^=1;
//                __delay_ms(500);
//                count++;
                
                if(ticks==10){
                    ticks=0;
                    INTCONbits.TMR0IE=0;
                    s=INPUT;
                    count=0;
                    clearLCD();
                    setCursor();
                }   
                break;
        //LATB^=0xFF;
        //__delay_ms(500);
        }
    }
    return;
}
