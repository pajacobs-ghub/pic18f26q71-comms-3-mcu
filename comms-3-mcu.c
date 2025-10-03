// comms-3-mcu.c
// RS485 communications MCU using the PIC18F26Q71.
// A Pico2 is the DAQ-MCU.
//
// Peter J.
// 2025-04-07 First cut is to adapt the COMMS-2 firmware from the PIC18f16Q41
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

#define VERSION_STR "v0.1 PIC18F26Q71 COMMS-3-MCU 2025-10-03"

// Each device on the RS485 network has a unique single-character identity.
// The master (PC) has identity '0'. Slave nodes may be 1-9A-Za-z.
// When programming each device, select a suitable value for MYID.
#define MYID 'E'

#define LED (LATAbits.LATA6)
uint8_t override_led = 0;
#define READYPIN (PORTBbits.RB4)
#define EVENTPIN (PORTBbits.RB3)
#define RESTARTn (LATBbits.LATB5)

void init_pins()
{
    // RA6 as digital-output for LED.
    LED = 0;
    TRISAbits.TRISA6 = 0;
    ANSELAbits.ANSELA6 = 0;
    //
    // RB4 as digital-input for DAQ-MCU Ready/Busy# signal.
    TRISBbits.TRISB4 = 1;
    ANSELBbits.ANSELB4 = 0;
    //
    // RB5 as digital-output for restart of DAQ_MCU,
    // open-drain, weak pull-up not needed.
    ODCONBbits.ODCB5 = 1;
    // WPUBbits.WPUB5 = 1;
    RESTARTn = 1;
    TRISBbits.TRISB5 = 0;
    ANSELBbits.ANSELB5 = 0;
    //
    // RA2 as analog-in for external trigger.
    //
    // Combine the Pico2-EVENT (RA7) signal with the comparator-latch signal
    // and output the result to the system EVENTn line (RB3).
    // We will also be sensing the EVENTn line to see if other devices
    // have pulled the line low to indicate the event.
    // Later, it may be driven by this MCU, on software command.
    ANSELBbits.ANSELB3 = 0;
    ODCONBbits.ODCB3 = 1;
    LATBbits.LATB3 = 1;
    TRISBbits.TRISB3 = 0; // output to EVENTn line
    ANSELBbits.ANSELB6 = 0;
    TRISBbits.TRISB6 = 1; // input from Pico2
    // Use CLC3 to to combine the event signals.
    // Follow the set-up description in Section 22.6 of datasheet.
    CLCSELECT = 0b10; // To select CLC3 registers for the following settings.
    CLCnCONbits.EN = 0; // Disable while setting up.
    // Data select from outside world
    CLCnSEL0 = 0x01; // data1 gets CLCIN1PPS as input (Pico2-EVENT)
    CLCnSEL1 = 0x01; // data2 gets CLCIN1PPS as input, also
    CLCnSEL2 = 0x25; // data3 gets CLC1 (comparator latch)
    CLCnSEL3 = 0x25; // data4 gets CLC1, also
    // Logic select into gates
    CLCnGLS0 = 0b00000010; // data1 goes through true to gate 1
    CLCnGLS1 = 0b00001000; // data2 goes through true to gate 2
    CLCnGLS2 = 0b00100000; // data3 goes through true to gate 3
    CLCnGLS3 = 0b10000000; // data4 goes through true to gate 4
    // Gate output polarities
    CLCnPOLbits.G1POL = 0;
    CLCnPOLbits.G2POL = 0;
    CLCnPOLbits.G3POL = 0;
    CLCnPOLbits.G4POL = 0;
    // Logic function is AND-OR
    CLCnCONbits.MODE = 0b000;
    CLCnPOLbits.POL = 1; // Invert output; EVENT# is active low.
    // Connect CLC3 to the CLC1 and RB7 pin.
    GIE = 0;
    PPSLOCK = 0x55;
    PPSLOCK = 0xaa;
    PPSLOCKED = 0;
    CLCIN1PPS = 0b000111; // RA7
    RB3PPS = 0x03; // CLC3OUT
    PPSLOCK = 0x55;
    PPSLOCK = 0xaa;
    PPSLOCKED = 1;
    // Now that the connections are made and function selected, enable it.
    CLCnCONbits.EN = 1;
    //
    return;
}

static inline void assert_event_pin()
{
    // Actively pull the event pin low.
    // Of course, this will not work if the CLC1 has control of RB3.
    LATBbits.LATB3 = 0;
    ODCONBbits.ODCB3 = 1;
    TRISBbits.TRISB3 = 0;
}

static inline void release_event_pin()
{
    // Release the drive on the pin and allow it to be pulled high.
    // Should not be used if CLC1 has control of RB3.
    LATBbits.LATB3 = 1;
    TRISBbits.TRISB3 = 1;
}

void FVR_init()
{
    // We want to supply both the ADC and the DAC with 4V.
    FVRCONbits.ADFVR = 3;  // 4v096
    FVRCONbits.CDAFVR = 3; // 4v096
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
    // Set up the ADC to look at the comparator input pin C1IN0+/RA2.
    TRISAbits.TRISA2 = 1;
    ANSELAbits.ANSELA2 = 1;
    //
    ADCON0bits.CS = 1; // use dedicated RC oscillator
    ADCON0bits.FM = 1; // right-justified result
    ADCON2bits.ADMD = 0b000; // basic (legacy) behaviour
    PIR1bits.ADIF = 0;
    ADREFbits.NREF = 0; // negative reference is Vss
    ADREFbits.PREF = 0b11; // positive reference is FVR
    ADACQ = 0x10; // 16TAD acquisition period
    ADPCH = 0x82; // select RA2/ANA2
    ADCON0bits.ON = 1; // power on the device

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

uint8_t enable_comparator(uint8_t level, int8_t slope)
{
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
    // Our external signal goes into the noninverting input of the comparator.
    // To use the DAC2 output as the reference, we need to feed the external
    // signal into the inverting output.
    CM1NCH = 0b001; // C1IN1- pin
    CM1PCH = 0b100; // DAC2_Output
    if (slope) {
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
    // Use CLC1 to latch the comparator output.
    //
    // Follow the set-up description in Section 22.6 of datasheet.
    CLCSELECT = 0b00; // To select CLC1 registers for the following settings.
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
    CLCnPOLbits.POL = 0;
    // We don't directly connect the output of this latch to the 
    // outside world but combine it with the Pico2-EVENT signal
    // using CLC3, which was set up just after reset.
    //
    // Now that the S-R latch is set up, enable it.
    CLCnCONbits.EN = 1;
    return 0; // Success is presumed.
}

void disable_comparator()
{
    CM1CON0bits.EN = 0;
    CLCSELECT = 0b00; // To select CLC1 for the following setting.
    CLCnCONbits.EN = 0;
    CM1CON0bits.EN = 0;
    DAC2CONbits.EN = 0;
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
    if (!override_led) LED = 1; // To indicate start of interpreter activity.
    switch (cmdStr[0]) {
        case 'v':
            nchar = snprintf(bufB, NBUFB, "/0v %s#\n", VERSION_STR);
            uart1_putstr(bufB);
            break;
        case 't':
            // Software trigger to assert EVENTn line low.
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
            nchar = snprintf(bufB, NBUFB, "/0Q %d %d#\n", EVENTPIN, READYPIN);
            uart1_putstr(bufB);
            break;
        case 'F':
            // Flush the RX2 buffer for incoming text from the AVR.
            uart2_flush_rx();
            nchar = snprintf(bufB, NBUFB, "/0F Flushed RX2 buffer#\n");
            uart1_putstr(bufB);
            break;
        case 'R':
            // Restart the attached DAQ MCU.
            RESTARTn = 0;
            __delay_ms(1);
            RESTARTn = 1;
            // Wait until we are reasonably sure that the DAQ MCU has restarted
            // and then flush the incoming serial buffer.
            __delay_ms(350);
            uart2_flush_rx();
            nchar = snprintf(bufB, NBUFB, "/0R DAQ_MCU restarted#\n");
            uart1_putstr(bufB);
            break;
        case 'L':
            // Turn LED on or off.
            // Turning the LED on by command overrides its use
            // as an indicator of interpreter activity.
            token_ptr = strtok(&cmdStr[1], sep_tok);
            if (token_ptr) {
                // Found some non-blank text; assume on/off value.
                // Use just the least-significant bit.
                i = (uint8_t) (atoi(token_ptr) & 1);
                LED = i;
                override_led = i;
                nchar = snprintf(bufB, NBUFB, "/0L %d#\n", i);
            } else {
                // There was no text to give a value.
                nchar = snprintf(bufB, NBUFB, "/0L error: no value#\n");
            }
            uart1_putstr(bufB);
            break;
        case 'a': {
            // Report the ADC value for the analog signal on the comparator input.
            uint16_t aValue = ADC_read();
            nchar = snprintf(bufB, NBUFB, "/0a %u#\n", aValue);
            uart1_putstr(bufB); }
            break;
        case 'e':
            // Enable comparator, to pull EVENT# line low on the
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
                uint8_t flag = enable_comparator((uint8_t)level, slope);
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
        case 'd':
            // Disable comparator and, maybe, release the EVENTn line.
            // The state of the EVENTn line also depends on Pico2-EVENT.
            disable_comparator();
            nchar = snprintf(bufB, NBUFB, "/0d Disable comparator#\n");
            uart1_putstr(bufB);
            break;
            break;
        case 'X':
            // Pass through a command to the AVR MCU.
            if (READYPIN) {
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
    if (!override_led) LED = 0; // To indicate end of interpreter activity.    
} // end interpret_RS485_command()

int main(void)
{
    int m;
    int n;
    init_pins();
    uart1_init(115200); // RS485 comms
    uart2_init(230400); // comms to DAQ-MCU
    uart2_flush_rx(); // Discard any characters already arrived from Pico2 MCU
    FVR_init();
    ADC_init();
    __delay_ms(10);
    // Flash LED twice at start-up to indicate that the MCU is ready.
    for (int8_t i=0; i < 2; ++i) {
        LED = 1;
        __delay_ms(250);
        LED = 0;
        __delay_ms(250);
    }
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
    ADC_close();
    FVR_close();
    uart2_flush_rx();
    uart2_close();
    uart1_flush_rx();
    uart1_close();
    return 0; // Expect that the MCU will reset.
} // end main
