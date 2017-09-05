/* 
 * File:   UART.h
 * Author: Steven Burford
 *
 * Created on 29 January 2015, 10:22 AM
 */

#ifndef UART_H
#define	UART_H

#include <xc.h>

#define MAXRECLENGTH 120
#define PI_RECEIVE_BUFFER_LENGTH 7
#define PI_TRANSMIT_MIN_BUFFER_LENGTH 8                                         //CHANGE BACK TO 8!!!! USE 7 FOR TESTING ONLY
#define PI_TRANSMIT_BUFFER_LENGTH 72

#define IBC_SN                  0x0001                                        // must be changed in UART and main header. - UART seems to take priotiry
#define FLAG_UART_TX_ACTIVE     1

#define PLM_RECEIVE_BUFFER_LENGTH 71
#define PLM_TRANSMIT_BUFFER_LENGTH 71

#define UART_HEADER             0
#define UART_CMD                1
#define UART_SN                 2
#define UART_CRC_VALID          3

#define RX_PACKET_TIMEOUT       100                                             //RX pointer will restart with 0xAA 0xAA just after timeout (each unit:500us & Max:200)

#endif	/* UART_H */

