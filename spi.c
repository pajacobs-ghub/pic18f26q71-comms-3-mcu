// spi.c
// Functions to drive the SPI1 module on the PIC18F26Q71 microcontroller
// as a master device.
//
// PJ, 2026-03-11

#include <xc.h>
#include <stdint.h>
#include "global_defs.h"
#include "spi.h"

static uint8_t spi1_is_initialized = 0;
static uint8_t upin_ss = 0;

void spi1_init(uint8_t upin, uint8_t ckp, uint8_t cke, uint8_t smp)
{
    upin_ss = upin & 0b11; // Only values 0 through 3 are valid options.
    switch (upin_ss) {
#       if PCB_REV == 1
        case 0:
            LATAbits.LATA3 = 1;
            ANSELAbits.ANSELA3 = 0;
            TRISAbits.TRISA3 = 0;
            ODCONAbits.ODCA3 = 0;
            WPUAbits.WPUA3 = 0;
            break;
        case 1:
            LATAbits.LATA4 = 1;
            ANSELAbits.ANSELA4 = 0;
            TRISAbits.TRISA4 = 0;
            ODCONAbits.ODCA4 = 0;
            WPUAbits.WPUA4 = 0;
            break;
        case 2:
            LATAbits.LATA5 = 1;
            ANSELAbits.ANSELA5 = 0;
            TRISAbits.TRISA5 = 0;
            ODCONAbits.ODCA5 = 0;
            WPUAbits.WPUA5 = 0;
            break;
        case 3:
            LATAbits.LATA6 = 1;
            ANSELAbits.ANSELA6 = 0;
            TRISAbits.TRISA6 = 0;
            ODCONAbits.ODCA6 = 0;
            WPUAbits.WPUA6 = 0;
            break;
#       else
        case 0:
            LATAbits.LATA4 = 1;
            ANSELAbits.ANSELA4 = 0;
            TRISAbits.TRISA4 = 0;
            ODCONAbits.ODCA4 = 0;
            WPUAbits.WPUA4 = 0;
            break;
        case 1:
            LATAbits.LATA5 = 1;
            ANSELAbits.ANSELA5 = 0;
            TRISAbits.TRISA5 = 0;
            ODCONAbits.ODCA5 = 0;
            WPUAbits.WPUA5 = 0;
            break;
        case 2:
            LATAbits.LATA6 = 1;
            ANSELAbits.ANSELA6 = 0;
            TRISAbits.TRISA6 = 0;
            ODCONAbits.ODCA6 = 0;
            WPUAbits.WPUA6 = 0;
            break;
        case 3:
            LATAbits.LATA7 = 1;
            ANSELAbits.ANSELA7 = 0;
            TRISAbits.TRISA7 = 0;
            ODCONAbits.ODCA7 = 0;
            WPUAbits.WPUA7 = 0;
            break;
#       endif
    }
    //
    // Configure the SPI1 module as master and assign IO pins.
    // SCK1 -> RC2
    // SDO1 -> RC1
    // SDI1 <- RC0
    SPI1CON0bits.EN = 0;
    SPI1CLK = 0b00010; // MFINTOSC (500 kHz)
    SPI1BAUD = 0; // 250 kHz should be slow enough for all sorts of devices
    SPI1CON0bits.MST = 1;
    SPI1CON2bits.TXR = 1;
    SPI1CON2bits.RXR = 1;
    SPI1CON1bits.CKP = ckp;
    SPI1CON1bits.CKE = cke;
    SPI1CON1bits.SMP = smp;
    GIE = 0;
    PPSLOCK = 0x55;
    PPSLOCK = 0xaa;
    PPSLOCKED = 0;
    SPI1SCKPPS = 0b010010; // RC2
    RC2PPS = 0x1D; // SPI1 SCK
    RC1PPS = 0x1E; // SPI1 SDO
    SPI1SDIPPS = 0b010000; // RC0
    // Don't assign an output pin for SPI1SS.
    PPSLOCK = 0x55;
    PPSLOCK = 0xaa;
    PPSLOCKED = 1;
    ANSELCbits.ANSELC0 = 0; // allow digital use
    ANSELCbits.ANSELC1 = 0;
    ANSELCbits.ANSELC2 = 0;
    TRISCbits.TRISC0 = 1; // SDI
    WPUCbits.WPUC0 = 1; // pull-up the SDI, in case there is nothing attached
    TRISCbits.TRISC1 = 0; // SD0
    TRISCbits.TRISC2 = 0; // SCK
    //
    SPI1CON0bits.EN = 1;
    spi1_is_initialized = 1;
} // end spi1_init()

void spi1_close(void)
{
    if (!spi1_is_initialized) return;
    //
    SPI1CON0bits.EN = 0;
    // Leave assigned SPI port pins as they are.
    // Set chip-select pin back to a safe mode. 
    switch (upin_ss) {
#       if PCB_REV == 1
        case 0:
            ANSELAbits.ANSELA3 = 1;
            TRISAbits.TRISA3 = 1;
            ODCONAbits.ODCA3 = 0;
            WPUAbits.WPUA3 = 1;
            break;
        case 1:
            ANSELAbits.ANSELA4 = 1;
            TRISAbits.TRISA4 = 1;
            ODCONAbits.ODCA4 = 0;
            WPUAbits.WPUA4 = 1;
            break;
        case 2:
            ANSELAbits.ANSELA5 = 1;
            TRISAbits.TRISA5 = 1;
            ODCONAbits.ODCA5 = 0;
            WPUAbits.WPUA5 = 1;
            break;
        case 3:
            ANSELAbits.ANSELA6 = 1;
            TRISAbits.TRISA6 = 1;
            ODCONAbits.ODCA6 = 0;
            WPUAbits.WPUA6 = 1;
            break;
#       else
        case 0:
            ANSELAbits.ANSELA4 = 1;
            TRISAbits.TRISA4 = 1;
            ODCONAbits.ODCA4 = 0;
            WPUAbits.WPUA4 = 1;
            break;
        case 1:
            ANSELAbits.ANSELA5 = 1;
            TRISAbits.TRISA5 = 1;
            ODCONAbits.ODCA5 = 0;
            WPUAbits.WPUA5 = 1;
            break;
        case 2:
            ANSELAbits.ANSELA6 = 1;
            TRISAbits.TRISA6 = 1;
            ODCONAbits.ODCA6 = 0;
            WPUAbits.WPUA6 = 1;
            break;
        case 3:
            ANSELAbits.ANSELA7 = 1;
            TRISAbits.TRISA7 = 1;
            ODCONAbits.ODCA7 = 0;
            WPUAbits.WPUA7 = 1;
            break;
#       endif
    }
    spi1_is_initialized = 0;
} // end spi1_close()

uint8_t spi1_exchange(uint8_t n, uint8_t* buf)
{
    uint8_t n_sent = 0;
    if (!spi1_is_initialized) return 0;
    // Select slave device manually.
    switch (upin_ss) {
#       if PCB_REV == 1
        case 0:
            LATAbits.LATA3 = 0;
            break;
        case 1:
            LATAbits.LATA4 = 0;
            break;
        case 2:
            LATAbits.LATA5 = 0;
            break;
        case 3:
            LATAbits.LATA6 = 0;
            break;
#       else
        case 0:
            LATAbits.LATA4 = 0;
            break;
        case 1:
            LATAbits.LATA5 = 0;
            break;
        case 2:
            LATAbits.LATA6 = 0;
            break;
        case 3:
            LATAbits.LATA7 = 0;
            break;
#       endif
    }
    __delay_us(10);
    SPI1STATUSbits.CLRBF = 1; // Clear the buffers.
    for (uint8_t i=0; i < n; ++i) {
        while (SPI1CON2bits.BUSY) { /* just wait */ }
        SPI1TCNT = 1;
        SPI1TXB = buf[i];
        while (!SPI1INTFbits.SRMTIF) { /* wait for shift register to empty */ }
        buf[i] = SPI1RXB;
        ++n_sent;
        SPI1INTFbits.SRMTIF = 0;
    }
    __delay_us(10);
    // Deselect slave device.
    switch (upin_ss) {
#       if PCB_REV == 1
        case 0:
            LATAbits.LATA3 = 1;
            break;
        case 1:
            LATAbits.LATA4 = 1;
            break;
        case 2:
            LATAbits.LATA5 = 1;
            break;
        case 3:
            LATAbits.LATA6 = 1;
            break;
#       else
        case 0:
            LATAbits.LATA4 = 1;
            break;
        case 1:
            LATAbits.LATA5 = 1;
            break;
        case 2:
            LATAbits.LATA6 = 1;
            break;
        case 3:
            LATAbits.LATA7 = 1;
            break;
#       endif
    }
    return n_sent;
} // end spi1_exchange()
