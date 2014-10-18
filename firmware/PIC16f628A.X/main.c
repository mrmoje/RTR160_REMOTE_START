/* 
 * File:   main.c
 * Author: james
 *
 * Created on August 9, 2014, 3:02 PM
 */

// PIC16F628A Configuration Bit Settings

// 'C' source line config statements

#include <xc.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <ctype.h>

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

// CONFIG
#pragma config FOSC = INTOSCIO  // Oscillator Selection bits (INTOSC oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled)
#pragma config PWRTE = ON       // Power-up Timer Enable bit (PWRT enabled)
#pragma config MCLRE = ON       // RA5/MCLR/VPP Pin Function Select bit (RA5/MCLR/VPP pin function is MCLR)
#pragma config BOREN = ON       // Brown-out Detect Enable bit (BOD enabled)
#pragma config LVP = OFF        // Low-Voltage Programming Enable bit (RB4/PGM pin has digital I/O function, HV on MCLR must be used for programming)
#pragma config CPD = OFF        // Data EE Memory Code Protection bit (Data memory code protection off)
#pragma config CP = OFF         // Flash Program Memory Code Protection bit (Code protection off)

#define DEBUG

#define _XTAL_FREQ   4000000 // needed by some macros
#define FCY _XTAL_FREQ/4

// Comm Setup
#define BAUDRATE 1200


#define BUZZER_PIN RB5
#define TOGGLE_BUZZER_PIN BUZZER_PIN = ~BUZZER_PIN;

//Globals
unsigned char 
    receive_buffer[5], //cmd reveive buffer
    sPORTA; //PORTA Shadow register


//Prototypes
void
    BOARD_init(void),
    USART_putc(unsigned char),
    buzz_ok(void),
    buzz_error(void),
    set_portAbit(unsigned char bt, bool val);
void interrupt ISR(void);
unsigned char USART_getc(void);

// Main function
void main()
{
    BOARD_init();
    printf("INIT!\r\n");
    buzz_ok();
    while(1);
}

void BOARD_init(void)
{
    //USART
    SPBRG = ((_XTAL_FREQ/16)/BAUDRATE) - 1;
    TRISB2 = 0; // TX Pin
    TRISB1 = 1; // RX Pin
    BRGH  = 1; 	// Fast baudrate
    SYNC  = 0;	// Asynchronous
    SPEN  = 1;	// Enable serial port pins
    CREN  = 1;	// Enable reception
    SREN  = 0;	// No effect
    TXIE  = 0;	// Disable tx interrupts
    RCIE  = 1;	// Enable rx interrupts
    TX9   = 0;	// 8-bit transmission
    RX9   = 0;	// 8-bit reception
    // Reset transmitter
    TXEN  = 0;
    TXEN  = 1;

    //I/O
    TRISB3 = 0; // Status LED Pin (O/P)
    TRISB5 = 0; // Buzzer Pin (O/P)
    TRISA0 = 0; // Key sw relay pin (O/P)
    TRISA1 = 0; // Ignition relay pin (O/P)
    TRISA2 = 0; // Engine kill sw relay pin (O/P)
    TRISA3 = 1; // Neutral Gear Sense Pin (I/P)

    //Interrupts
    GIE  = 1; 	// Enable global interrupts
    PEIE = 1;  	// Enable Peripheral Interrupts
}

void interrupt ISR(void)
{
    if (RCIF) { // check if receive interrupt has fired
        static unsigned char i = 0;
        unsigned char b = USART_getc();
        if ( isalpha(b) && i < 4 )
            receive_buffer[i++] = b; // append

        if ( b == '\r' ) {
            if(strcmp(receive_buffer,"KYON") == 0) {
                buzz_ok();
                set_portAbit(0,1); // power on / bypass keyswitch
            }
            
            if (strcmp(receive_buffer,"IGNT") == 0) {
                if(RA3) {
                    printf("ERR_NEUTRAL");
                    buzz_error();
                } else { //otherwise start the damn thing!
                    buzz_ok();
                    if(!RA0) {
                        set_portAbit(0,1); // power on / bypass keyswitch
                        __delay_ms(1000);
                    }
                    set_portAbit(2,1); // bypass engine cutout
                    __delay_ms(1000);
                    set_portAbit(1,1); // ignition
                    __delay_ms(2000);
                    set_portAbit(1,0); // release (should be running now)
                }
            }
            
            if (strcmp(receive_buffer,"KYOF") == 0) { PORTA = sPORTA = 0; }
            if (strcmp(receive_buffer,"BZOK") == 0) { buzz_ok(); }
            if (strcmp(receive_buffer,"BZER") == 0) { buzz_error(); }

            i = 0;
        }
#ifdef DEBUG
        printf("%s\r\n",receive_buffer);
#endif
        RCIF = 0;      // reset receive interrupt flag
    }
}

void USART_putc(unsigned char Byte)  // Writes a character to the serial port
{
    while(!TXIF);  // wait for previous transmission to finish
    TXREG = Byte;
}

unsigned char USART_getc(void)   // Reads a character from the serial port
{
    if(OERR) {
        buzz_error();
        // reset the receiver
        CREN = 0;
        CREN = 1;
    }
    
    if(FERR) {
        //buzz_error();
    }
    
    while(!RCIF);  // Wait for transmission to receive
    return RCREG;
}


void buzz_ok(void)
{
    unsigned int buzz = 600;
    while(--buzz){
        TOGGLE_BUZZER_PIN
        __delay_us(100);
    }
}

void buzz_error(void)
{
    unsigned char buzz = 20;
    while(--buzz){
        TOGGLE_BUZZER_PIN
        __delay_ms(4);
    }
    __delay_ms(100);
    buzz = 80;
    while(--buzz){
        TOGGLE_BUZZER_PIN
        __delay_ms(4);
    }
}

//xc8's printf calls this
void putch(unsigned char byte)
{
    USART_putc(byte);
}

//function sets
void set_portAbit(unsigned char bt, bool val)
{
    val ? ( sPORTA |= 1 << bt ) : ( sPORTA &= ~(1 << bt) );
    PORTA = sPORTA;
}