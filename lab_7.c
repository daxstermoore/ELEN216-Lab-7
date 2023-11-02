/* 
 * File:   lab_7.c
 * Author: concrete drinker
 *
 * Created on November 2, 2023, 7:46 AM
 */

#include <stdio.h>
#include <stdlib.h>
#include "elen216headernew.h"

/*
 * 
 */
int main(int argc, char** argv) {
    
    UART2_Initialize();
    
    
    /* The following code example will enable the Timer1 interrupt, load the Timer1
    Period register and start Timer1. When a Timer1 period match interrupt occurs,
    the interrupt service routine must clear the Timer1 interrupt status flag in software.
    */

    T1CON = 0x00;	//Stop Timer1 and reset the Timer1 control register
    TMR1 = 0x0000;	//Clears the contents of the timer register
    PR1 = 0xF424;	//Load the Period register with the value 0xF424

    IPC0bits.T1IP = 0x01;	//Setup Timer1 interrupt for desired priority level
    IFS0bits.T1IF = 0;		//Clear the Timer1 interrupt status flag
    IEC0bits.T1IE = 1;		//Enable Timer1 interrupts
    T1CONbits.TCKPS = 2;        //Set prescaler as 1:64
    T1CONbits.TON = 1;		//Start 16 bit timer
    TRISAbits.TRISA0 = 0;	//Set PortA as digital output
    INTCON2bits.GIE = 1;	//Enable global interrupts

    // Determine the frequency of the signal at RA0

    // fosc = 8 Mhz => Fcyc = 8 Mhz / 2 = 4 Mhz -> Tcyc = 0.25 us
    // time to interrupt = Tcyc * [PR1 - TMR1] * Prescaler = 0.25 us x [0xF424 - 0] * 64 = 1000 ms
    // period for the signal on RA0 is 1000 ms * 2 = 2000 ms
    // frequency for the signal on RA0 is = 1 / 2000 ms = 0.5 Hz
    
    while(1);
    return (0);
}

/* Example code for Timer1 ISR*/
void __attribute__((__interrupt__, __shadow__)) _T1Interrupt(void)
{
    TMR1 = 0x0000;		//Clears the contents of the timer register
    PORTA = PORTA ^ 0x0001;	//Toggle bit 0 on PortA
    IFS0bits.T1IF = 0;		//Reset Timer1 interrupt flag and return
}

