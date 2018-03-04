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

//DefuseButton


//D rows
//C columns

//char arrays detailing number characters
char left[4]={'1','4','7','*'};
char mid[4]={'2','5','8','0'};
char right[4]={'3','6','9','#'};


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
    char delay=50;
    LATBbits.LATB4=1;
    __delay_us(delay);
    //RS=0;
    LATBbits.LATB5=(lcdVal&0b00010000)>>4;
    LATBbits.LATB3=(lcdVal&0b00001000)>>3;
    LATBbits.LATB2=(lcdVal&0b00000100)>>2;
    LATBbits.LATB1=(lcdVal&0b00000010)>>1;
    LATBbits.LATB0=(lcdVal&0b00000001)>>0;
    __delay_us(delay);
    LATBbits.LATB4=0;
    __delay_us(delay);
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
    
    //Write values
//    writeChar('H');
//    writeChar('e');
//    writeChar('l');
//    writeChar('l');
//    writeChar('o');

    
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
    //writeChar(0x02);
}

void main(void) {
    //Configure Clock 32MHz
    OSCCON=0b11110000;
    TRISB=0x00;//output
    ANSELD=0x00;//Digital reads only[]
    ANSELC=0x00;//Digital reads only
    //TRISBbits.TRISB4=1;
    setColumnsOutput();
    setRowsInput();
    char pos=0;
    
    initialiseLCD();
    clearLCD();
    __delay_ms(100);
    char keyVal=0xFF;
    while(1){
        keyVal=0xFF;
        keyVal=scan();
        if(keyVal!=0xFF){
            writeChar(keyVal);
            pos++;
            if(pos==16){
                clearLCD();
                pos=0;
            }
            __delay_ms(200);
        }
        //LATB^=0xFF;
        //__delay_ms(500);
    }
    return;
}
