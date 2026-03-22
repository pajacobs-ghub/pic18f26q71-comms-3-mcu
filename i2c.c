// i2c.c
// Functions to drive the I2C1 module on the PIC18F26Q71 microcontroller
// as a single master I2C device, with 7-bit addressing.
//
// PJ, 2026-03-09 

#include <xc.h>
#include <stdint.h>
#include "global_defs.h"
#include "i2c.h"

//
// We shall use TMR0 for our timeout monitor.
//
void tmr0_init(uint8_t count_ms)
{
    // Set up as an 8-bit timer, with millisecond ticks (approximately).
    T0CON0bits.EN = 0;
    T0CON1bits.CS = 0b101; // MFINTOSC (500kHz)
    T0CON1bits.CKPS = 0b1001; // 1:512
    PIR3bits.TMR0IF = 0;
    TMR0L = 255 - count_ms;
    T0CON0bits.EN = 1;
}

void tmr0_close(void)
{
    T0CON0bits.EN = 0;
    PIR3bits.TMR0IF = 0;
}

uint8_t tmr0_expired()
{
    return PIR3bits.TMR0IF;
}

//
// Follow the instructions in Section 37.4.2 of the data sheet.
// Code for the happy path but check for timeout, in case reality
// goes off the rails.
//
void i2c1_init(void)
{
    // Default pin mapping for PIC18F26Q71-I/SP
    // SCL1 = RC3, SDA1 = RC4
    GIE = 0;
    PPSLOCK = 0x55;
    PPSLOCK = 0xaa;
    PPSLOCKED = 0;
    I2C1SCLPPS = 0b010011; // RC3
    RC3PPS = 0x20; // I2C1 SCL
    I2C1SDAPPS = 0b010100; // RC4
    RC4PPS = 0x21; // I2C1 SDA
    PPSLOCK = 0x55;
    PPSLOCK = 0xaa;
    PPSLOCKED = 1;
    ANSELCbits.ANSELC3 = 0; // allow digital input
    ANSELCbits.ANSELC4 = 0;
    TRISCbits.TRISC3 = 0; // Set as output, as per data sheet section 37.3.2.
    TRISCbits.TRISC4 = 0;
    ODCONCbits.ODCC3 = 1; // Set to open-drain, as per data sheet instructions.
    ODCONCbits.ODCC4 = 1;
    // Leave the slew-rate and pull-up selection alone.
    // We have external resistors for pull-up.
    //
    // Configure module
    I2C1CON0bits.EN = 0; // Disable module.
    I2C1CON0bits.MODE = 0b100; // master mode, 7-bit
    I2C1CLKbits.CLK = 0b00011; // MFINTOSC 500kHz
    I2C1BAUD = 4; // to get 100 kHz clock out
    I2C1CON1bits.ACKCNT = 1; // Send a NACK at end of count when reading.
    I2C1CON1bits.ACKDT = 0; // Send ACK after each data byte, for reading more.
    PIR7bits.I2C1IF = 0;
    I2C1CON0bits.EN = 1; // Enable module.
    return;
}

void i2c1_close(void)
{
    I2C1CON0bits.EN = 0;
    return;
}

uint8_t i2c1_read(uint8_t addr7bit, uint8_t n, uint8_t* buf, uint8_t timeout_ms)
// Note that the value given for timeout_ms needs to allow enough time 
// for reception of all n bytes following the transmission of the address byte.
{
    uint8_t n_read = 0;
    I2C1STAT1bits.CLRBF = 1;
    I2C1ADB1 = (uint8_t)(addr7bit << 1) | 1; // R/Wn bit is 1
    I2C1CNT = n;
    I2C1CON0bits.S = 1;
    tmr0_init(timeout_ms);  // Allow enough time for full transmission.
    for (uint8_t i=0; i < n; ++i) {
        while (!I2C1STAT1bits.RXBF) {
            /* wait for incoming byte */
            if (tmr0_expired()) { goto Quit; }
        }
        buf[i] = I2C1RXB;
        ++n_read;
    }
    while (I2C1STAT0bits.MMA) {
        /* wait until Stop condition issued */
        if (tmr0_expired()) { goto Quit; }
    }
    Quit:
    tmr0_close();
    return n_read;
} // end i2c1_read()

uint8_t i2c1_write(uint8_t addr7bit, uint8_t n, uint8_t* buf, uint8_t timeout_ms)
// Note that the value given for timeout_ms needs to allow enough time 
// for transmission of all n bytes following the address byte.
{
    uint8_t n_sent = 0;
    I2C1STAT1bits.CLRBF = 1;
    I2C1ADB1 = (uint8_t)(addr7bit << 1); // R/Wn bit is 0
    I2C1CNT = n;
    I2C1CON0bits.S = 1;
    tmr0_init(timeout_ms);
    for (uint8_t i=0; i < n; ++i) {
        while (!I2C1STAT1bits.TXBE) {
            /* wait for space to put next byte */
            if (tmr0_expired()) { goto Quit; }
        }
        I2C1TXB = buf[i];
        //
        // Wait for slave device to acknowledge previous transaction.
        // On the first pass of this loop, this will be for the address byte
        // and the value of n_sent will be zero on return of this function.
        //
        // 1. Wait for acknowledge time, after 8th SCL falling edge.
        while (!I2C1CON1bits.ACKT) {
            if (tmr0_expired()) { goto Quit; }
        }
        // 2. Wait until acknowledge sequence is done.
        while (I2C1CON1bits.ACKT) {
            if (tmr0_expired()) { goto Quit; }
        }
        // 3. Quit if the slave device did not acknowledge.
        if (I2C1CON1bits.ACKSTAT) { goto Quit; }
        ++n_sent;
    }
    while (I2C1STAT0bits.MMA) {
        /* wait until Stop condition issued */
        if (tmr0_expired()) { goto Quit; }
    }
    Quit:
    tmr0_close();
    return n_sent;
} // end i2c1_write()
