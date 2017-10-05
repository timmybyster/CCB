/*
 * File:   main.h
 * Author: Steven Burford
 *
 * Created on 28 January, 2015, 8:23 AM
 *
 * CCB V1.00 Code
 *
 */

#ifndef MAIN_H
#define	MAIN_H


// PIC16F1709 Configuration Bit Settings

#include <xc.h>

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.
// CONFIG1H
#pragma config FOSC = INTIO67   // Oscillator Selection bits (Internal oscillator block)
#pragma config PLLCFG = OFF     // 4X PLL Enable (Oscillator used directly)
#pragma config PRICLKEN = ON    // Primary clock enable bit (Primary clock enabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRTEN = OFF     // Power-up Timer Enable bit (Power up timer disabled)
#pragma config BOREN = OFF      // Brown-out Reset Enable bits (Brown-out Reset disabled in hardware and software)
#pragma config BORV = 190       // Brown Out Reset Voltage bits (VBOR set to 1.90 V nominal)

// CONFIG2H
#pragma config WDTEN = SWON //SWDTEN has no effect and is enabled //SWON      // Watchdog Timer Enable bits (Watch dog timer is always disabled. SWDTEN has no effect.)
//#pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)
#pragma config WDTPS = 2048    // Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3H
#pragma config CCP2MX = PORTC1  // CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)
#pragma config PBADEN = OFF     // PORTB A/D Enable bit (PORTB<5:0> pins are configured as digital I/O on Reset)
#pragma config CCP3MX = PORTB5  // P3A/CCP3 Mux bit (P3A/CCP3 input/output is multiplexed with RB5)
#pragma config HFOFST = OFF     // HFINTOSC Fast Start-up (HFINTOSC output and ready status are delayed by the oscillator stable status)
#pragma config T3CMX = PORTC0   // Timer3 Clock input mux bit (T3CKI is on RC0)
#pragma config P2BMX = PORTB5   // ECCP2 B output mux bit (P2B is on RB5)
#pragma config MCLRE = EXTMCLR  // MCLR Pin Enable bit (MCLR pin disabled, RE3 input pin enabled)

// CONFIG4L
#pragma config STVREN = OFF     // Stack Full/Underflow Reset Enable bit (Stack full/underflow will not cause Reset)
#pragma config LVP = OFF        // Single-Supply ICSP Enable bit (Single-Supply ICSP disabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection Block 0 (Block 0 (000800-003FFFh) not code-protected)
#pragma config CP1 = OFF        // Code Protection Block 1 (Block 1 (004000-007FFFh) not code-protected)
#pragma config CP2 = OFF        // Code Protection Block 2 (Block 2 (008000-00BFFFh) not code-protected)
#pragma config CP3 = OFF        // Code Protection Block 3 (Block 3 (00C000-00FFFFh) not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block (000000-0007FFh) not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection Block 0 (Block 0 (000800-003FFFh) not write-protected)
#pragma config WRT1 = OFF       // Write Protection Block 1 (Block 1 (004000-007FFFh) not write-protected)
#pragma config WRT2 = OFF       // Write Protection Block 2 (Block 2 (008000-00BFFFh) not write-protected)
#pragma config WRT3 = OFF       // Write Protection Block 3 (Block 3 (00C000-00FFFFh) not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot Block (000000-0007FFh) not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection Block 0 (Block 0 (000800-003FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection Block 1 (Block 1 (004000-007FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Table Read Protection Block 2 (Block 2 (008000-00BFFFh) not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF      // Table Read Protection Block 3 (Block 3 (00C000-00FFFFh) not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot Block (000000-0007FFh) not protected from table reads executed in other blocks)

//============================================================================//

#define FLAG_TICK       0b0000000000000001
#define _XTAL_FREQ      16000000

//Define ADC values to return

#define ARMED                   1
#define DISARMED                0

#define PI_COM_FAULT            1
#define PI_COM_GOOD             0

#define RX_TIMEOUT              30

#define FIRING                  2
#define TEST                    0

#define ADC_VOLT_EL             0
#define ADC_VOLT_CF             1
#define ADC_VOLT_ZC             2
#define ADC_VOLT_FO             3
#define ADC_VOLT_KS             4

#define VOLT_EL_HIGH            390 //For Rel = 2k
#define VOLT_CF_HIGH            375// calculateion get 389 but testing showed 375 to be more accurate. 275 //NEED TO CALCULATE
#define COUNTER_CF              40 //Amount of cable faults dettected to trigger alarm
#define COUNTER_EL              4  //Amount of EL faults detected to trigger alarm
#define START_EL                5 //Amount of cycles before it actually does the test


//#define VOLT_ZC_THRESHOLD       10 //Calculate
#define VOLT_ZC_THRESHOLD       1 //Calculate
#define VOLT_MAINS_THRESHOLD    800

#define CCB_SN                  0x0003 // must be changed in UART and main header. - UART seems to take priotiry
#define CBB_SN_BROADCAST_ADD    0x00
#define CBB_DEFAULT_SN          0x3FFE
#define CBB_NULL_SN             0x3FFF

#define CLEAR_UPDATE            0x00
#define DEFAULT_UPDATE          0x01
#define ERROR_UPDATE            0x02

#define CMD_SEND_DEFAULT        0b00000000
#define CMD_FORCE_DEFAULT       0b01000000
#define CMD_SEND_DC             0b00000001
#define CMD_FORCE_DC            0b01000001
#define CMD_SEND_AC             0b00000010
#define CMD_FORCE_AC            0b01000010
#define CMD_SEND_BLAST_VALUE    0b00000011
#define CMD_FORCE_BLAST_VALUE   0b01100011
#define CMD_SEND_TEMPERATURE    0b00000100
#define CMD_OPEN_RELAY          0b00000110
#define CMD_CLOSE_RELAY         0b00001101
#define CMD_GET_SN              0b00000111
#define CMD_SN_LIST_CHANGED     0b00001000
#define CMD_BLAST_COMMAND       0b00100101
#define PING_CBB                0b00101001
#define ARM_CBB                 0b00110001
#define DISARM_CBB              0b00110000
#define CMD_CABLE_FAULT         0b00100110
#define CMD_CBB_NEW_SN          0b00001001
#define CMD_NULL                0b11111111
#define CMD_AB1_UID             0b00001010
#define CMD_AB1_DATA            0b00001011

#define CMD_SEND_DEFAULT_B        0b10000000
#define CMD_FORCE_DEFAULT_B       0b11000000
#define CMD_SEND_DC_B             0b10000001
#define CMD_FORCE_DC_B            0b11000001
#define CMD_SEND_AC_B             0b10000010
#define CMD_FORCE_AC_B            0b11000010
#define CMD_SEND_BLAST_VALUE_B    0b10000011
#define CMD_FORCE_BLAST_VALUE_B   0b11000011
#define CMD_SEND_TEMPERATURE_B    0b10000100
#define CMD_OPEN_RELAY_B          0b10000110
#define CMD_CLOSE_RELAY_B         0b10000101
#define CMD_GET_SN_B              0b10000111
#define CMD_SN_LIST_CHANGED_B     0b10001000
#define CMD_BLAST_COMMAND_B       0b10100101
#define PING_CBB_B                0b10101001
#define ARM_CBB_B                 0b10110001
#define DISARM_CBB_B              0b10110000
#define CMD_CABLE_FAULT_B         0b10100110
#define CMD_CBB_NEW_SN_B          0b10001001

#define CMD_PI_SN_CBBS          0b00000001
#define CMD_PI_SN_IB651         0b00000010
#define CMD_PI_DEFAULT_DATA     0b00000011
#define CMD_PI_FORCE_DEFAULT    0b01000011
#define CMD_PI_BLAST_VALUE      0b00000110
#define CMD_PI_FORCE_BLAST_VAL  0b01000110
#define CMD_PI_CCB_DEFAULT      0b00001000
#define CMD_PI_CCB_ERRORS       0b00001100
#define CMD_PI_CBB_PARENT       0b00001111
#define CMD_PI_CLOSE_RELAY      0b00010001
#define CMD_PI_OPEN_RELAY       0b00010010
#define CMD_PI_CLEAR_CBB_LIST   0b00010011              //clear the CCB's list of CBB's
#define CMD_PI_MISSING_CBB_LIST 0b00010100              //gives the list of missing CBB's sent to PI
#define CMD_PI_CLEAR_ALARM      0b00010101              //Command to clear the alarm on the CCB
#define CMD_PI_BLAST_PERMISSION 0b10010000
#define CMD_PI_BLAST_DENIED     0b10010111
#define CMD_PI_BLAST_ACK        0b10100101
#define CMD_PI_PING_CBB         0b00101001
#define CMD_PI_ARM_CBB          0b10110001
#define CMD_PI_DISARM           0b10110000
#define CMD_PI_DC_DATA          0b00111110
#define CMD_PI_FORCE_DC         0b01111110
#define CMD_PI_AC_DATA          0b00111111
#define CMD_PI_FORCE_AC         0b01111111

#define CMD_PI_AB1_UID          0b00000100
#define CMD_PI_AB1_DATA         0b00000101

#define CMD_PI_DEFAULT_DATA_B   0b10000011              //COMPLETE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
#define CMD_PI_PING_CBB_B       0b10101001
//

#define FLAG_UART_TX_ACTIVE         1

#define PLM_RECEIVE_BUFFER_LENGTH   71
#define PLM_TRANSMIT_BUFFER_LENGTH  71

#define PRESSED                     1
#define DEPRESSED                   0

#define ON                          1
#define OFF                         0

#define INPUT                       1
#define OUTPUT                      0

#define HighCycle                   25
#define LowCycle                    1

#define EARTHFAULT                  2
#define CABLEFAULT                  1
#define CLEAR                       0
#define FAULT                       1

#define MAX_NO_CBB                  30

#define DATA_STATUS                 0
#define DATA_DC_VOLT                1
#define DATA_AC_VOLT                2
#define DATA_BL_VOLT                3
#define DATA_SERIAL_NUMBER          4
#define DATA_LENGTH                 5
#define DATA_TYPE_COUNT             6

#define CCB_COMSCRASH_COUNT         240     //tickes every 1 second amounts to 4 minures


//A2D Input definitions
#define TRIS_CFAULT_READ        TRISAbits.TRISA0        //Cable Fault Read
#define LAT_CFAULT_READ         LATAbits.LATA0
#define ANSEL_CFAULT_READ       ANSELAbits.ANSA0

#define TRIS_MAINS_ZEROCROSSING TRISAbits.TRISA1        //Mains/Zero Crossing detect
#define LAT_MAINS_ZEROCROSSING  LATAbits.LATA1        //Mains/Zero Crossing detect
#define ANSEL_MAINS_ZEROCROSSING             ANSELAbits.ANSA1
#define PORT_MAINS_ZEROCROSSING PORTAbits.RA1;

#define TRIS_EARTH_LEAKAGE      TRISCbits.TRISC2        //EARTH LEAKAGE
#define LAT_EARTH_LEAKAGE       LATCbits.LATC2
#define ANSEL_EARTH_LEAKAGE     ANSELCbits.ANSC2

#define TRIS_FIRE_OUT           TRISBbits.TRISB5        //FIRE OUT
#define LAT_FIRE_OUT            LATBbits.LATB5
#define ANSEL_FIRE_OUT          ANSELBbits.ANSB5

//STANDARD INPUT DEFINITIONS
#define TRIS_FIRE_SWITCH        TRISAbits.TRISA3        //FIRE SWITCH IN
#define LAT_FIRE_SWITCH         LATAbits.LATA3
#define PORT_FIRE_SWITCH        PORTAbits.RA3
#define ANSEL_FIRE_SWITCH       ANSELAbits.ANSA3

#define TRIS_RESET_SWITCH       TRISAbits.TRISA6        //RESET SWITCH IN 
#define LAT_RESET_SWITCH        LATAbits.LATA6
#define PORT_RESET_SWITCH       PORTAbits.RA6

//OUTPUTS
#define TRIS_BUZZER_ALARM_LED   TRISAbits.TRISA2        //BUZZER/ALARM LED
#define LAT_BUZZER_ALARM_LED    LATAbits.LATA2

#define TRIS_SSR1               TRISBbits.TRISB6        //SSR RELAY
#define LAT_SSR1                LATBbits.LATB6

#define TRIS_CABLE_FAULT        TRISBbits.TRISB7        //CABLE FAULT
#define LAT_CABLE_FAULT         LATBbits.LATB7

#define TRIS_RELAY1             TRISCbits.TRISC1        //RELAY 1
#define LAT_RELAY1              LATCbits.LATC1

//LED definitions
#define TRIS_CABLE_FAULT_LED    TRISAbits.TRISA5        //CABLE FAULT LED
#define LAT_CABLE_FAULT_LED     LATAbits.LATA5

//#define TRIS_TEST_LED           TRISAbits.TRISA6        //CHANGED TO RESET SWITCH
//#define LAT_TEST_LED            LATAbits.LATA6

#define TRIS_DIAG2_LED          TRISAbits.TRISA7        //DIAGNOSTIC LED 1
#define LAT_DIAG2_LED           LATAbits.LATA7

#define TRIS_DIAG1_LED          TRISCbits.TRISC0        //DIAGNOSTIC LED 2
#define LAT_DIAG1_LED           LATCbits.LATC0

//UART DEFINITIONS
#define TRIS_RX                 TRISCbits.TRISC7        //RX PIN
#define LAT_RX                  LATCbits.LATC7
#define ANSEL_RX                ANSELCbits.ANSC7

#define TRIS_TX                 TRISCbits.TRISC6        //TX PIN
#define LAT_TX                  LATCbits.LATC6
#define ANSEL_TX                ANSELCbits.ANSC6

#endif MAIN_H