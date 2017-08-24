/* 
 * File:   ST7540.h
 * Author: aece-engineering
 *
 * Created on November 26, 2014, 1:23 PM
 */

#ifndef ST7540_H
#define	ST7540_H

#include <xc.h>

#define FLAG_ST7540_TX_ACTIVE       1
#define FLAG_ST7540_RX_ACTIVE       2
#define FLAG_ST7540_DATA_READY      4

#define ST7540_PREAM_LEN            2
#define ST7540_HEADER_LEN           2
#define ST7540_MIN_PACKET_LEN       9
#define ST7540_MAX_PACKET_LEN       (unsigned long short) 75
#define ST7540_HEADER               (unsigned long short) 0x9b58

#define ST7540_DATA_LEN             0
#define ST7540_SOURCE               1
#define ST7540_DEST                 2
#define ST7540_NUMBER               3
#define ST7540_CMD                  4
#define ST7540_CRC_VALID            5

#define ST7540_FREQ_60KHZ           0b000000000000000000000000
#define ST7540_FREQ_66KHZ           0b000000000000000000000001
#define ST7540_FREQ_72KHZ           0b000000000000000000000010
#define ST7540_FREQ_76KHZ           0b000000000000000000000011
#define ST7540_FREQ_82K05HZ         0b000000000000000000000100
#define ST7540_FREQ_86KHZ           0b000000000000000000000101
#define ST7540_FREQ_110KHZ          0b000000000000000000000110
#define ST7540_FREQ_132K5HZ         0b000000000000000000000111

#define ST7540_BAUD_600             0b000000000000000000000000
#define ST7540_BAUD_1200            0b000000000000000000001000
#define ST7540_BAUD_2400            0b000000000000000000010000
#define ST7540_BAUD_4800            0b000000000000000000011000

#define ST7540_DEVIA_05             0b000000000000000000000000
#define ST7540_DEVIA_1              0b000000000000000000100000

#define ST7540_WATCHDOG_DIS         0b000000000000000000000000
#define ST7540_WATCHDOG_EN          0b000000000000000001000000

#define ST7540_TRANS_TOUT_DIS       0b000000000000000000000000
#define ST7540_TRANS_TOUT_1S        0b000000000000000010000000
#define ST7540_TRANS_TOUT_3S        0b000000000000000100000000
//#define ST7540_TRANS_TOUT_NU        0b000000000000000110000000

#define ST7540_FREQ_DET_TIME_05MS   0b000000000000000000000000
#define ST7540_FREQ_DET_TIME_1MS    0b000000000000001000000000
#define ST7540_FREQ_DET_TIME_3MS    0b000000000000010000000000
#define ST7540_FREQ_DET_TIME_5MS    0b000000000000011000000000

//#define ST7540_RESERVED             0b000000000000100000000000

#define ST7540_PREAM_WO_COND        0b000000000000000000000000
#define ST7540_PREAM_W_COND         0b000000000001000000000000
#define ST7540_CARRIER_WO_COND      0b000000000010000000000000
#define ST7540_CARRIER_W_COND       0b000000000011000000000000

#define ST7540_SYNC                 0b000000000000000000000000
#define ST7540_ASYNC                0b000000000100000000000000

#define ST7540_OUTP_CLOCK_16MHZ     0b000000000000000000000000
#define ST7540_OUTP_CLOCK_8MHZ      0b000000001000000000000000
#define ST7540_OUTP_CLOCK_4MHZ      0b000000010000000000000000
#define ST7540_OUTP_CLOCK_OFF       0b000000011000000000000000

#define ST7540_OUTP_VOLT_FRZ_EN     0b000000000000000000000000
#define ST7540_OUTP_VOLT_FRZ_DIS    0b000000100000000000000000

#define ST7540_HEADER_RECOG_DIS     0b000000000000000000000000
#define ST7540_HEADER_RECOG_EN      0b000001000000000000000000

#define ST7540_FRAME_LEN_CNT_DIS    0b000000000000000000000000
#define ST7540_FRAME_LEN_CNT_EN     0b000010000000000000000000

#define ST7540_HEADER_LENGTH_8      0b000000000000000000000000
#define ST7540_HEADER_LENGTH_16     0b000100000000000000000000

#define ST7540_EXTENDED_REG_DIS_24  0b000000000000000000000000
#define ST7540_EXTENDED_REG_EN_48   0b001000000000000000000000

#define ST7540_SENSITIV_NORM        0b000000000000000000000000
#define ST7540_SENSITIV_HIGH        0b010000000000000000000000

#define ST7540_INP_FILTER_DIS       0b000000000000000000000000
#define ST7540_INP_FILTER_EN        0b100000000000000000000000 

#define UNSIGNED_LONG_SHORT_23_16   0b111111110000000000000000
#define UNSIGNED_LONG_SHORT_15_8    0b000000001111111100000000
#define UNSIGNED_LONG_SHORT_7_0     0b000000000000000011111111

/*
#define TRIS_n_CD_PD            TRISCbits.TRISC7
#define ANSEL_n_CD_PD           ANSELCbits.ANSC7
#define PORT_n_CD_PD            PORTCbits.RC7
#define LAT_n_CD_PD             LATCbits.LATC7

#define TRIS_REGnDATA           TRISCbits.TRISC6
#define ANSEL_REGnDATA          ANSELCbits.ANSC6
#define PORT_REGnDATA           PORTCbits.RC6
#define LAT_REGnDATA            LATCbits.LATC6

#define TRIS_RXD                TRISCbits.TRISC4
#define ANSEL_RXD               ANSELCbits.ANSC4
#define PORT_RXD                PORTCbits.RC4
#define LAT_RXD                 LATCbits.LATC4

#define TRIS_TXD                TRISCbits.TRISC5
#define ANSEL_TXD               ANSELCbits.ANSC5
#define PORT_TXD                PORTCbits.RC5
#define LAT_TXD                 LATCbits.LATC5

#define TRIS_RXnTX              TRISAbits.TRISA6
//#define ANSEL_RXnTX             ANSELAbits.ANSA6
#define PORT_RXnTX              PORTAbits.RA6
#define LAT_RXnTX               LATAbits.LATA6

#define TRIS_BU_THERM           TRISCbits.TRISC0
//#define ANSEL_BU_THERM          ANSELCbits.ANSC0
#define PORT_BU_THERM           PORTCbits.RC0
#define LAT_BU_THERM            LATCbits.LATC0

#define TRIS_CLR_T              TRISCbits.TRISC3
#define ANSEL_CLR_T             ANSELCbits.ANSC3
#define PORT_CLR_T              PORTCbits.RC3
#define LAT_CLR_T               LATCbits.LATC3

#define TRIS_UARTnSPI           TRISCbits.TRISC1
//#define ANSEL_UARTnSPI          ANSELCbits.ANSC1
#define PORT_UARTnSPI           PORTCbits.RC1
#define LAT_UARTnSPI            LATCbits.LATC1
*/


#define TRIS_n_CD_PD            TRISBbits.TRISB0
#define ANSEL_n_CD_PD           ANSELBbits.ANSB0
#define PORT_n_CD_PD            PORTBbits.RB0
#define LAT_n_CD_PD             LATBbits.LATB0

#define TRIS_REGnDATA           TRISCbits.TRISC4
#define ANSEL_REGnDATA          ANSELCbits.ANSC4
#define PORT_REGnDATA           PORTCbits.RC4
#define LAT_REGnDATA            LATCbits.LATC4

#define TRIS_RXD                TRISBbits.TRISB2
#define ANSEL_RXD               ANSELBbits.ANSB2
#define PORT_RXD                PORTBbits.RB2
#define LAT_RXD                 LATBbits.LATB2

#define TRIS_TXD                TRISBbits.TRISB3
#define ANSEL_TXD               ANSELBbits.ANSB3
#define PORT_TXD                PORTBbits.RB3
#define LAT_TXD                 LATBbits.LATB3

#define TRIS_RXnTX              TRISCbits.TRISC5
#define ANSEL_RXnTX             ANSELCbits.ANSC5
#define PORT_RXnTX              PORTCbits.RC5
#define LAT_RXnTX               LATCbits.LATC5

#define TRIS_BU_THERM           TRISCbits.TRISC3
#define ANSEL_BU_THERM          ANSELCbits.ANSC3
#define PORT_BU_THERM           PORTCbits.RC3
#define LAT_BU_THERM            LATCbits.LATC3

#define TRIS_CLR_T              TRISBbits.TRISB1
#define ANSEL_CLR_T             ANSELBbits.ANSB1
#define PORT_CLR_T              PORTBbits.RB1
#define LAT_CLR_T               LATBbits.LATB1

#define TRIS_CLR_T_IOC          TRISBbits.TRISB4
#define ANSEL_CLR_T_IOC         ANSELBbits.ANSB4
#define PORT_CLR_T_IOC          PORTBbits.RB4
#define LAT_CLR_T_IOC           LATBbits.LATB4

/* //UART/SPI not on Modem interface for IBC-1
#define TRIS_UARTnSPI           TRISCbits.TRISC1
//#define ANSEL_UARTnSPI          ANSELCbits.ANSC1
#define PORT_UARTnSPI           PORTCbits.RC1
#define LAT_UARTnSPI            LATCbits.LATC1
 */


//#define TRIS_
//#define ANSEL_
//#define PORT_
//#define LATCH_

//LED definitions
//#define TRIS_STATUS_LED         TRISBbits.TRISB2	//Status LED
//#define TRIS_CFAULT_LED         TRISAbits.TRISA7	//Cable Fault LED
//#define ANSEL_STATUS_LED        ANSELBbits.ANSB2        //Status LED
//#define ANSEL_CFAULT_LED        ANSELAbits.ANSA7        //Cable Fault LED
//#define LAT_STATUS_LED          LATBbits.LATB2		//Status LED
//#define LAT_CFAULT_LED          LATAbits.LATA7		//Cable Fault LED

#endif	/* ST7540_H */

