// comms-3-mcu.c
// RS485 communications MCU using the PIC18F26Q71.
// A Pico2 is the DAQ-MCU.
//
// Peter J.
// 2025-10-07 First cut is to adapt the COMMS-2 firmware from the PIC18f16Q41
// 2025-10-04 Edit PCB tracks so that we can use negative-channel inputs to 
//            comparators 1 and 2 for the internal and external trigger signals.
// 2025-11-09 Clean up the functions and put them to use.
// 2025-11-11 Simple hardware trigger function, as used for PIC18F16Q41-COMMS-1.
//
// CONFIG1
#pragma config FEXTOSC = OFF
#pragma config RSTOSC = HFINTOSC_64MHZ

// CONFIG2
#pragma config CLKOUTEN = OFF
#pragma config PR1WAY = OFF
#pragma config BBEN = OFF
#pragma config CSWEN = OFF
#pragma config FCMEN = OFF
#pragma config FCMENP = OFF
#pragma config FCMENS = OFF

// CONFIG3
#pragma config MCLRE = EXTMCLR
#pragma config PWRTS = PWRT_64
#pragma config MVECEN = OFF
#pragma config IVT1WAY = OFF
#pragma config LPBOREN = OFF
#pragma config BOREN = SBORDIS

// CONFIG4
#pragma config BORV = VBOR_1P9
#pragma config ZCD = OFF
#pragma config PPS1WAY = OFF
#pragma config STVREN = ON
#pragma config LVP = ON
#pragma config DEBUG = OFF
#pragma config XINST = OFF

// CONFIG5
#pragma config WDTCPS = WDTCPS_31
#pragma config WDTE = ON

// CONFIG6
#pragma config WDTCWS = WDTCWS_7
#pragma config WDTCCS = SC

// CONFIG7
// BBSIZE = No Setting

// CONFIG8
#pragma config SAFSZ = SAFSZ_NONE

// CONFIG9
#pragma config WRTB = OFF
#pragma config WRTC = OFF
#pragma config WRTD = OFF
#pragma config WRTSAF = OFF
#pragma config WRTAPP = OFF

// CONFIG10
#pragma config CPD = OFF

// CONFIG11
#pragma config CP = OFF

#include <xc.h>
#include "global_defs.h"
#include <stdint.h>
#include <stdlib.h>

#include "uart.h"
#include <stdio.h>
#include <string.h>

#define VERSION_STR "v0.6 PIC18F26Q71 COMMS-3-MCU 2025-11-12"

// Each device on the RS485 network has a unique single-character identity.
// The master (PC) has identity '0'. Slave nodes may be 1-9A-Za-z.
// When programming each device, select a suitable value for MYID.
#define MYID 'E'

// The following definitions are for sensing.
#define PICO2_READY_PIN (PORTBbits.RB4)
#define EVENTn_PIN (PORTBbits.RB3)

// The following definition is for output.
#define PICO2_RESETn_PIN (LATBbits.LATB5)

void init_pins()
{
    // Utility bits RA6, RA5 RA4 and RA3 are all digital.
    ANSELAbits.ANSELA6 = 0;
    ANSELAbits.ANSELA5 = 0;
    ANSELAbits.ANSELA4 = 0;
    ANSELAbits.ANSELA3 = 0;
    //
    // RB4 as digital-input for DAQ-MCU Ready/Busy# signal.
    TRISBbits.TRISB4 = 1;
    ANSELBbits.ANSELB4 = 0;
    //
    // RB5 as digital-output for restart of Pico2 DAQ_MCU,
    // open-drain, weak pull-up is not needed.
    ODCONBbits.ODCB5 = 1;
    // WPUBbits.WPUB5 = 1;
    PICO2_RESETn_PIN = 1;
    TRISBbits.TRISB5 = 0;
    ANSELBbits.ANSELB5 = 0;
    //
    // Initially, we will be sensing the EVENTn line,
    // to see if other devices have pulled the line low
    // to indicate the event.
    // Later it may be driven low by software command or
    // it may be driven by hardware (comparator latched by CLC3).
    ANSELBbits.ANSELB3 = 0;
    TRISBbits.TRISB3 = 1; // input EVENTn line
    // The following lines are compatible with the un-asserted state,
    // for when the pin is driven by this device.
    ODCONBbits.ODCB3 = 1;
    LATBbits.LATB3 = 1;
    //
    // We will ignore Pico2_EVENT, 
    // but still set it up as a digital input.
    ANSELAbits.ANSELA7 = 0;
    TRISAbits.TRISA7 = 1;
    //
    return;
}

static inline void assert_event_pin()
{
    // Actively pull the event pin low.
    LATBbits.LATB3 = 0;
    ODCONBbits.ODCB3 = 1;
    TRISBbits.TRISB3 = 0; // output EVENTn line
}

static inline void release_event_pin()
{
    // Release the drive on the pin and allow it to be pulled high.
    LATBbits.LATB3 = 1;
    TRISBbits.TRISB3 = 1; // input EVENTn line
}

void FVR_init()
{
    // We want to supply both the ADC and the DAC with the same reference
    // voltage that is not too high for a 3v3 system.
    FVRCONbits.ADFVR = 2;  // 2v048
    FVRCONbits.CDAFVR = 2; // 2v048
    FVRCONbits.EN = 1;
    while (!FVRCONbits.RDY) { /* should be less than 25 microseconds */ }
    return;
}

void FVR_close()
{
    FVRCONbits.EN = 0;
    FVRCONbits.ADFVR = 0;
    FVRCONbits.CDAFVR = 0;
    return;
}

void ADC_init()
{
    // Set up the ADC to look at the V_ANALOG/2 input pin ANA0/RA0.
    TRISAbits.TRISA0 = 1;
    ANSELAbits.ANSELA0 = 1;
    //
    ADCON0bits.CS = 1; // use dedicated RC oscillator
    ADCON0bits.IC = 0; // single-ended input
    ADCON0bits.FM = 1; // right-justified result
    ADCON2bits.MD = 0b011; // burst-average mode
    PIR1bits.ADIF = 0;
    ADREFbits.NREF = 0; // negative reference is Vss
    ADREFbits.PREF = 0b11; // positive reference is FVR
    ADACQ = 0x10; // 16TAD acquisition period
    ADNCH = 0b1110111; // select VSS
    ADPCH = 0x00; // select RA0/ANA0
    ADCON0bits.ON = 1; // power on the device
    return;
}

void ADC_select(uint8_t byte_value)
{
    ADPCH = byte_value;
    return;
}

uint16_t ADC_read()
{
    // Returns the value from the ADC when looking at the input pin
    // for the comparator.
    ADCON0bits.GO = 1;
    NOP();
    while (ADCON0bits.GO) { /* wait, should be brief */ }
    PIR1bits.ADIF = 0;
    return ADRES;
}

void ADC_close()
{
    ADCON0bits.ON = 0;
    return;
}

uint8_t enable_external_trigger(uint8_t level, int8_t slope)
{
    // The external trigger signal is connected to C1IN1-/RA1.
    // It uses comparator 1 to sense the analog signal,
    // DAC2 as the reference, and CLC1 as the SR latch.
    //
    // Input:
    // level sets the trigger voltage
    // slope=1 to trigger on exceeding level
    // slope=0 to trigger on going below level
    //
    // Returns:
    // 0 if successfully set up,
    // 1 if the comparator is already high.
    //
    // Use DAC2 for the reference level.
    DAC2CONbits.PSS = 0b10; // FVR Buffer 2
    DAC2CONbits.NSS = 0; // VSS
    DAC2CONbits.EN = 1;
    DAC2DATL = level;
    __delay_ms(1);
    //
    // Our external signal goes into the inverting input of the comparator
    // and we use the DAC2 output as the reference on the noninverting input.
    CM1NCH = 0b001; // C1IN1- pin
    CM1PCH = 0b100; // DAC2_Output
    if (slope) {
        // Invert comparator output-polarity to get trigger on positive slope.
        CM1CON0bits.POL = 1;
    } else {
        CM1CON0bits.POL = 0;
    }
    CM1CON0bits.HYS = 0; // no hysteresis
    CM1CON0bits.SYNC = 0; // async output
    CM1CON0bits.EN = 1;
    // The signal out of the comparator should transition 0 to 1
    // as the external trigger voltage crosses the specified level.
    __delay_ms(1);
    if (CMOUTbits.MC1OUT) {
        // Fail early because the comparator is already triggered.
        return 1;
    }
    // Use CLC3 to latch the comparator output to RB3.
    //
    // Follow the set-up description in Section 22.6 of datasheet.
    CLCSELECT = 0b010; // To select CLC3 registers for the following settings.
    CLCnCONbits.EN = 0; // Disable while setting up.
    // Data select from outside world
    CLCnSEL0 = 0x20; // data1 gets CMP1_OUT as input
    CLCnSEL1 = 0; // data2 gets CLCIN0PPS as input, but gets ignored in logic select
    CLCnSEL2 = 0; // data3 as for data2
    CLCnSEL3 = 0; // data4 as for data2
    // Logic select into gates
    CLCnGLS0 = 0b10; // data1 goes through true to gate 1 (S-R set)
    CLCnGLS1 = 0; // gate 2 gets logic 0 (S-R set)
    CLCnGLS2 = 0; // gate 3 gets logic 0 (S-R reset)
    CLCnGLS3 = 0; // gate 4 gets logic 0 (S-R reset)
    // Gate output polarities
    CLCnPOLbits.G1POL = 0;
    CLCnPOLbits.G2POL = 0;
    CLCnPOLbits.G3POL = 0;
    CLCnPOLbits.G4POL = 0;
    // Logic function is S-R latch
    CLCnCONbits.MODE = 0b011;
    CLCnPOLbits.POL = 1; // Invert output; EVENT# is active low.
    // Now that the S-R latch is set up, enable it.
    CLCnCONbits.EN = 1;
    // Connect CLC1OUT to the RB3 pin.
    ODCONBbits.ODCB3 = 1;
    TRISBbits.TRISB3 = 0; // output
    GIE = 0;
    PPSLOCK = 0x55;
    PPSLOCK = 0xaa;
    PPSLOCKED = 0;
    RB3PPS = 0x03; // CLC3OUT to EVENTn_PIN
    PPSLOCK = 0x55;
    PPSLOCK = 0xaa;
    PPSLOCKED = 1;
    return 0; // Success is presumed.
}

uint8_t enable_internal_trigger(uint8_t level, int8_t slope)
{
    // The internal trigger signal comes from the analog buffer
    // for channel 0 and is connected to C2IN3-/RB1.
    // It uses comparator 2 to sense the analog signal,
    // DAC2 as the reference, and CLC1 as the SR latch.
    // Note that we can have only one of the triggers enabled
    // at any point in time because both use CLC1 as the SR latch.
    //
    // Input:
    // level sets the trigger voltage
    // slope=1 to trigger on exceeding level
    // slope=0 to trigger on going below level
    //
    // Returns:
    // 0 if successfully set up,
    // 1 if the comparator is already high.
    //
    // Use DAC2 for the reference level.
    DAC2CONbits.PSS = 0b10; // FVR Buffer 2
    DAC2CONbits.NSS = 0; // VSS
    DAC2CONbits.EN = 1;
    DAC2DATL = level;
    __delay_ms(1);
    //
    // Our external signal goes into the inverting input of the comparator
    // and we use the DAC2 output as the reference on the noninverting input.
    CM2NCH = 0b011; // C2IN3- pin
    CM2PCH = 0b100; // DAC2_Output
    if (slope) {
        // Invert comparator output-polarity to get trigger on positive slope.
        CM2CON0bits.POL = 1;
    } else {
        CM2CON0bits.POL = 0;
    }
    CM2CON0bits.HYS = 0; // no hysteresis
    CM2CON0bits.SYNC = 0; // async output
    CM2CON0bits.EN = 1;
    // The signal out of the comparator should transition 0 to 1
    // as the external trigger voltage crosses the specified level.
    __delay_ms(1);
    if (CMOUTbits.MC2OUT) {
        // Fail early because the comparator is already triggered.
        return 1;
    }
    // Use CLC3 to latch the comparator output to RB3.
    //
    // Follow the set-up description in Section 22.6 of datasheet.
    CLCSELECT = 0b010; // To select CLC3 registers for the following settings.
    CLCnCONbits.EN = 0; // Disable while setting up.
    // Data select from outside world
    CLCnSEL0 = 0x21; // data1 gets CMP2_OUT as input
    CLCnSEL1 = 0; // data2 gets CLCIN0PPS as input, but gets ignored in logic select
    CLCnSEL2 = 0; // data3 as for data2
    CLCnSEL3 = 0; // data4 as for data2
    // Logic select into gates
    CLCnGLS0 = 0b10; // data1 goes through true to gate 1 (S-R set)
    CLCnGLS1 = 0; // gate 2 gets logic 0 (S-R set)
    CLCnGLS2 = 0; // gate 3 gets logic 0 (S-R reset)
    CLCnGLS3 = 0; // gate 4 gets logic 0 (S-R reset)
    // Gate output polarities
    CLCnPOLbits.G1POL = 0;
    CLCnPOLbits.G2POL = 0;
    CLCnPOLbits.G3POL = 0;
    CLCnPOLbits.G4POL = 0;
    // Logic function is S-R latch
    CLCnCONbits.MODE = 0b011;
    CLCnPOLbits.POL = 1; // Invert output; EVENT# is active low.
    // Now that the S-R latch is set up, enable it.
    CLCnCONbits.EN = 1;
    // Connect CLC1OUT to the RB3 pin.
    ODCONBbits.ODCB3 = 1;
    TRISBbits.TRISB3 = 0; // output
    GIE = 0;
    PPSLOCK = 0x55;
    PPSLOCK = 0xaa;
    PPSLOCKED = 0;
    RB3PPS = 0x03; // CLC3OUT to EVENTn_PIN
    PPSLOCK = 0x55;
    PPSLOCK = 0xaa;
    PPSLOCKED = 1;
    return 0; // Success is presumed.
}

void disable_hardware_trigger()
{
    // Disables either/both internal and external triggers.
    TRISBbits.TRISB3 = 1; // input
    CLCSELECT = 0b010; // To select CLC3 for the following setting.
    CLCnCONbits.EN = 0;
    CM1CON0bits.EN = 0;
    CM2CON0bits.EN = 0;
    DAC2CONbits.EN = 0;
    // Reconnect LATB to the RB3 pin,
    // so we may drive EVENTn via software.
    GIE = 0;
    PPSLOCK = 0x55;
    PPSLOCK = 0xaa;
    PPSLOCKED = 0;
    RB3PPS = 0x00; // LATB3 to EVENTn_PIN
    PPSLOCK = 0x55;
    PPSLOCK = 0xaa;
    PPSLOCKED = 1;
    return;
}

uint8_t read_utility_pins()
{
    // Although the pins happen to be grouped together in the current design,
    // we'll set the bits one at a time, in case we ever reassign them.
    uint8_t bits = 0;
    if (PORTAbits.RA6) bits |= 0b1000;
    if (PORTAbits.RA5) bits |= 0b0100;
    if (PORTAbits.RA4) bits |= 0b0010;
    if (PORTAbits.RA3) bits |= 0b0001;
    return bits;
}

void write_utility_pins(char action, uint8_t bits)
{
    // Although the pins happen to be grouped together in the current design,
    // we'll set the bits one at a time, in case we ever reassign them.
    switch (action) {
    case 'T':
    case 't':
        TRISAbits.TRISA6 = (bits & 0b1000) ? 1 : 0;
        TRISAbits.TRISA5 = (bits & 0b0100) ? 1 : 0;
        TRISAbits.TRISA4 = (bits & 0b0010) ? 1 : 0;
        TRISAbits.TRISA3 = bits & 0b0001;
        break;
    case 'O':
    case 'o':
        ODCONAbits.ODCA6 = (bits & 0b1000) ? 1 : 0;
        ODCONAbits.ODCA5 = (bits & 0b0100) ? 1 : 0;
        ODCONAbits.ODCA4 = (bits & 0b0010) ? 1 : 0;
        ODCONAbits.ODCA3 = bits & 0b0001;
        break;
    case 'W':
    case 'w':
        WPUAbits.WPUA6 = (bits & 0b1000) ? 1 : 0;
        WPUAbits.WPUA5 = (bits & 0b0100) ? 1 : 0;
        WPUAbits.WPUA4 = (bits & 0b0010) ? 1 : 0;
        WPUAbits.WPUA3 = bits & 0b0001;
        break;
    case 'L':
    case 'l':
        LATAbits.LATA6 = (bits & 0b1000) ? 1 : 0;
        LATAbits.LATA5 = (bits & 0b0100) ? 1 : 0;
        LATAbits.LATA4 = (bits & 0b0010) ? 1 : 0;
        LATAbits.LATA3 = bits & 0b0001;
        break;
    default:
        /* do nothing */ ;
    }
    return;
}

// For incoming RS485 comms
#define NBUFA 80
char bufA[NBUFA];
// For outgoing RS485 comms
#define NBUFB 268
char bufB[NBUFB];
// For incoming AVR responses
#define NBUFC 256
char bufC[NBUFC];

int find_char(char* buf, int start, int end, char c)
// Returns the index of the character if found, -1 otherwise.
// start is the index of the first character to check.
// end is the index of the last character to check.
{
    for (int i = start; i <= end; i++) {
        if (buf[i] == '\0') return -1;
        if (buf[i] == c) return i;
    }
    return -1;
}

char* trim_RS485_command(char* buf, int nbytes)
// Returns a pointer to the command text string, within buf.
// The resulting string may be zero-length.
//
// A valid incoming command from the RS485 will be of the form
// "/cXXXXXXXX!"
// where the components are
//    / is the start character
//    ! is the end character
//    c is the MYID character, identifying the receiving node
//    XXXXXXX is the command text
//
// This format is described in the text:
// J.M. Hughes
// Real World Instrumentation
// O'Rielly 2010
// Chapter 11 Instrumentation Data I/O, Unique Protocols.
//
{
    // printf("DEBUG: buf=%s", buf);
    int start = find_char(buf, 0, nbytes-1, '/');
    if (start == -1) {
        // Did not find starting '/'
        buf[0] = '\0'; // make it a zero-length string
        return &buf[0];
    }
    int end = find_char(buf, start, nbytes-1, '!');
    if (end == -1) {
        // Did not find terminating '!'
        buf[0] = '\0'; // make it a zero-length string
        return &buf[0];
    }
    // At this point, we have a valid incoming command.
    if (buf[start+1] != MYID) {
        // The incoming message is not for this node, so discard it.
        buf[0] = '\0'; // make it a zero-length string
        return &buf[0];
    }
    // At this point, the message is for us.
    buf[end] = '\0'; // Trim off the '!' character.
    // On return, omit the MYID character from the front.
    return &buf[start+2];
} // end trim_command()

void interpret_RS485_command(char* cmdStr)
// We intend that valid commands are answered quickly
// so that the supervisory PC can infer the absence of a node
// by the lack of a prompt response.
// A command that does not do what is expected should return a message
// that includes the word "error".
{
    char* token_ptr;
    const char* sep_tok = ", ";
    int nchar;
    uint8_t i, j;
    // nchar = printf("DEBUG: cmdStr=%s", cmdStr);
    switch (cmdStr[0]) {
        case 'v':
            nchar = snprintf(bufB, NBUFB, "/0v %s#\n", VERSION_STR);
            uart1_putstr(bufB);
            break;
        case 't':
            // Software trigger to assert EVENTn line low.
            disable_hardware_trigger(); // Can enable only one at a time.
            assert_event_pin();
            nchar = snprintf(bufB, NBUFB, "/0t Software trigger#\n");
            uart1_putstr(bufB);
            break;
        case 'z':
            // Release EVENTn line (from software trigger).
            release_event_pin();
            nchar = snprintf(bufB, NBUFB, "/0z Release EVENTn line#\n");
            uart1_putstr(bufB);
            break;
        case 'Q':
            // Query the status signals.
            // READY/BUSYn is from the AVR.
            // EVENTn is a party line that anyone may pull low.
            nchar = snprintf(bufB, NBUFB, "/0Q %d %d#\n", EVENTn_PIN, PICO2_READY_PIN);
            uart1_putstr(bufB);
            break;
        case 'F':
            // Flush the RX2 buffer for incoming text from the DAQ MCU.
            uart2_flush_rx();
            nchar = snprintf(bufB, NBUFB, "/0F Flushed RX2 buffer#\n");
            uart1_putstr(bufB);
            break;
        case 'R':
            // Restart the attached DAQ MCU.
            PICO2_RESETn_PIN = 0;
            __delay_ms(1);
            PICO2_RESETn_PIN = 1;
            // Wait until we are reasonably sure that the DAQ MCU has restarted
            // and then flush the incoming serial buffer.
            __delay_ms(350);
            uart2_flush_rx();
            nchar = snprintf(bufB, NBUFB, "/0R DAQ_MCU restarted#\n");
            uart1_putstr(bufB);
            break;
        case 'a':
            // Report the ADC value for the analog signal on the selected pin.
            token_ptr = strtok(&cmdStr[1], sep_tok);
            if (token_ptr) {
                // Found some non-blank text; assume that it is an integer
                // that specifies which pin to sample.
                // 0x00  ANA0 (VA/2)
                // 0x01  C1IN1-/ANA1 (external trigger)
                // 0x09  C2IN3-/ANB1 (internal trigger)
                uint8_t pin = (uint8_t)atoi(token_ptr);
                ADC_select(pin);
                uint16_t aValue = ADC_read();
                nchar = snprintf(bufB, NBUFB, "/0a %u#\n", aValue);
            } else {
                nchar = snprintf(bufB, NBUFB, "/0a error: no pin specified#\n");
            }
            uart1_putstr(bufB);
            break;
        case 'e':
            // Enable external trigger, to pull EVENTn line low on the
            // external trigger signal.
            token_ptr = strtok(&cmdStr[1], sep_tok);
            if (token_ptr) {
                // Found some non-blank text; assume trigger level
                // and, maybe, slope flag.
                int16_t level = atoi(token_ptr);
                if (level > 255) level = 255;
                if (level < 0) level = 0;
                int8_t slope = 1;
                token_ptr = strtok(NULL, sep_tok);
                if (token_ptr) {
                    slope = (int8_t) atoi(token_ptr);
                }
                disable_hardware_trigger(); // Can enable only one at a time.
                uint8_t flag = enable_external_trigger((uint8_t)level, slope);
                if (flag) {
                    nchar = snprintf(bufB, NBUFB, "/0e error: comparator already triggered#\n");
                } else {
                    nchar = snprintf(bufB, NBUFB, "/0e %d %d#\n", level, slope);
                }
            } else {
                // There was no text to give a trigger level.
                nchar = snprintf(bufB, NBUFB, "/0e error: no level supplied#\n");
            }
            uart1_putstr(bufB);
            break;
        case 'i':
            // Enable internal trigger, to pull EVENTn line low on the
            // internal trigger signal on analog channel 0.
            token_ptr = strtok(&cmdStr[1], sep_tok);
            if (token_ptr) {
                // Found some non-blank text; assume trigger level
                // and, maybe, slope flag.
                int16_t level = atoi(token_ptr);
                if (level > 255) level = 255;
                if (level < 0) level = 0;
                int8_t slope = 1;
                token_ptr = strtok(NULL, sep_tok);
                if (token_ptr) {
                    slope = (int8_t) atoi(token_ptr);
                }
                disable_hardware_trigger(); // Can enable only one at a time.
                uint8_t flag = enable_internal_trigger((uint8_t)level, slope);
                if (flag) {
                    nchar = snprintf(bufB, NBUFB, "/0i error: comparator already triggered#\n");
                } else {
                    nchar = snprintf(bufB, NBUFB, "/0i %d %d#\n", level, slope);
                }
            } else {
                // There was no text to give a trigger level.
                nchar = snprintf(bufB, NBUFB, "/0i error: no level supplied#\n");
            }
            uart1_putstr(bufB);
            break;
        case 'd':
            // Disable trigger and release the EVENTn line.
            disable_hardware_trigger();
            release_event_pin();
            nchar = snprintf(bufB, NBUFB, "/0d Disable hardware trigger#\n");
            uart1_putstr(bufB);
            break;
            break;
        case 'u':
            // Interact with the 'utility' pins RA6, RA5, RA4 and RA3.
            //
            // The form of the command is
            // 'u' <action> <bits>
            // <action> is one of
            //   'P' read PORT bits (for input; <bits> is not required)
            //   'T' set TRIS bits (output-drive control 1=input 0=output)
            //   'O' set ODC bits (open-drain control)
            //   'W' set WPU bits (weak pull-up)
            //   'L' set LAT bits (latch for output)
            // <bits> are mapped to relevant pins as
            //   bit     7    6    5    4    3    2    1    0
            //   pin     X    X    X    X  RA6  RA5  RA4  RA3
            //
            // Note that the relevant register for each pin will be written
            // with new data for each call with a write action.
            // If you want to leave some pins unchanged from their current state,
            // you will need to send appropriate data.
            token_ptr = strtok(&cmdStr[1], sep_tok);
            if (token_ptr) {
                // Found some non-blank text; assume is is a single character
                // that specifies the action, as described above.
                char action = (char) *token_ptr;
                if (action == 'P' || action == 'p') {
                    uint8_t bits = read_utility_pins();
                    nchar = snprintf(bufB, NBUFB, "/0u %c 0x%02x#\n", action, bits);
                } else {
                    token_ptr = strtok(NULL, sep_tok);
                    if (token_ptr) {
                        uint8_t bits = (uint8_t) atoi(token_ptr);
                        write_utility_pins(action, bits);
                        nchar = snprintf(bufB, NBUFB, "/0u %c 0x%02x#\n", action, bits);
                    } else {
                        nchar = snprintf(bufB, NBUFB, "/0u error: no bits specified#\n");
                    }
                }
            } else {
                nchar = snprintf(bufB, NBUFB, "/0u error: no action specified#\n");
            }
            uart1_putstr(bufB);
            break;
        case 'X':
            // Pass through a command to the Pico2 DAQ MCU.
            if (PICO2_READY_PIN) {
                // DAQ MCU is ready and waiting for a command.
                // nchar = snprintf(bufB, NBUFB, "DEBUG About to send command:%s\n", &cmdStr[1]); uart1_putstr(bufB);
                uart2_putstr(&cmdStr[1]);
                uart2_putch('\n');
                // The DAQ MCU may start its response within 200us,
                // so start listening for that response immediately.
                uart2_getstr(bufC, NBUFC);
                nchar = snprintf(bufB, NBUFB, "/0X %s#\n", &bufC[0]);
            } else {
                // DAQ MCU is not ready for a command.
                nchar = snprintf(bufB, NBUFB, "/0X error: DAQ MCU busy#\n");
            }
            uart1_putstr(bufB);
            break;
        default:
            nchar = snprintf(bufB, NBUFB, "/0%c error: Unknown command#\n", cmdStr[0]);
            uart1_putstr(bufB);
    }
} // end interpret_RS485_command()

int main(void)
{
    int m;
    int n;
    init_pins();
    uart1_init(115200); // RS485 comms
    uart2_init(230400); // comms to Pico2 DAQ-MCU
    uart2_flush_rx(); // Discard any characters already arrived from Pico2 MCU
    FVR_init();
    ADC_init();
    // Wait until we are reasonably sure that the Pico2 has restarted
    // and then flush the incoming serial buffer.
    __delay_ms(100);
    uart2_flush_rx();
    // We will operate this COMMS_MCU as a slave,
    // waiting for commands and only responding when spoken to.
    while (1) {
        // Characters are not echoed as they are typed.
        // Backspace deleting is allowed.
        // NL (Ctrl-J) signals end of incoming string.
        m = uart1_getstr(bufA, NBUFA);
        if (m > 0) {
            char* cmd = trim_RS485_command(bufA, NBUFA);
            // Note that the cmd string may be of zero length,
            // with the null character in the first place.
            // If that is the case, do nothing with it.
            if (*cmd) {
                interpret_RS485_command(cmd);
            }
        }
    }
    disable_hardware_trigger();
    release_event_pin();
    ADC_close();
    FVR_close();
    uart2_flush_rx();
    uart2_close();
    uart1_flush_rx();
    uart1_close();
    return 0; // Expect that the MCU will reset.
} // end main
