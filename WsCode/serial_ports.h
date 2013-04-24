/*******************************************************************************
*
*	TITLE:		serial_ports.h
*
*	VERSION:	0.2 (Beta)
*
*	DATE:		28-Dec-2004
*
*	AUTHOR:		R. Kevin Watson
*				kevinw@jpl.nasa.gov
*
*	COMMENTS:	You are free to use this source code for any non-commercial
*				use. Please do not make copies of this source code, modified
*				or un-modified, publicly available on the internet or elsewhere
*				without permission. Thanks.
*
*				Copyright ©2004 R. Kevin Watson. All rights are reserved.
*
********************************************************************************
*
*	CHANGE LOG:
*
*	DATE         REV  DESCRIPTION
*	-----------  ---  ----------------------------------------------------------
*	22-Dec-2004  0.1  RKW - Original code.
*	28-Dec-2004  0.2  RKW - Using preprocessor directives, added the ability
*	                  to enable/disable individual serial port receive and
*	                  transmit code. Updated documentation.
*
*******************************************************************************/
#ifndef _SERIAL_PORTS_H
#define _SERIAL_PORTS_H

// comment out the next line to disable serial port one
// receive functionality
#define ENABLE_SERIAL_PORT_ONE_RX

// comment out the next line to disable serial port one
// transmit functionality
#define ENABLE_SERIAL_PORT_ONE_TX

// comment out the next line to disable serial port two
// receive functionality
#define ENABLE_SERIAL_PORT_TWO_RX

// comment out the next line to disable serial port two
// transmit functionality
#define ENABLE_SERIAL_PORT_TWO_TX

// Sample values that can be plugged into the SPBRGx register to program the
// baud rate generator for a specific baud rate. Make sure to also set the BRGH
// bit accordingly. See the Init_Serial_Port_One() and Init_Serial_Port_Two()
// functions in the serial_ports.c source file for more information.
#define BAUD_4800  129	// set BRGH = 0
#define BAUD_9600 	64	// set BRGH = 0
#define BAUD_14400  42	// set BRGH = 0
#define BAUD_19200 129	// set BRGH = 1
#define BAUD_38400  64 	// set BRGH = 1
#define BAUD_57600  42	// set BRGH = 1
#define BAUD_115200 21	// set BRGH = 1
#define BAUD_230400 10  // set BRGH = 1

// These values define the size, in bytes, of the four circular queues used to
// buffer incoming and outgoing serial port data. The size of the receive queue
// is critical to prevent data loss. As an example, if you're continuously
// receiving data at 38,400 baud and only call Read_Serial_Port_xxx() in the
// Process_Data_From_Master_uP() loop, which is called every 17ms, you'll need
// to size the queue to accept all of the data you expect to receive in those
// 17ms. To do this, multiply the rate at which bytes of data are being received
// times the amount of time between calls to Read_Serial_Port_xxx(). In the
// above example, we're receiving data at a rate of 3840 bytes per second
// (8 bits of data plus 2 bits of overhead per byte transferred) and we need to
// store at least 17ms-worth of that data for a calculated queue size of 66
// bytes. Because the queue size must be a power of two, we'll need to round-up
// to a queue size of 128. Another solution is to check for received serial data
// at a higher rate by putting the call to Read_Serial_Port_xxx() in the much
// faster Process_Data_From_Local_IO() loop. As mentioned above, these values
// must be a power of two (i.e.,8,16,32,64,128) for the circular queue algorithm
// to function correctly.
#define RX_1_QUEUE_SIZE 32
#define TX_1_QUEUE_SIZE 16
#define RX_2_QUEUE_SIZE 32
#define TX_2_QUEUE_SIZE 16

// The circular queue algorithm will break if these values are altered.
#define RX_1_QUEUE_INDEX_MASK RX_1_QUEUE_SIZE-1
#define TX_1_QUEUE_INDEX_MASK TX_1_QUEUE_SIZE-1
#define RX_2_QUEUE_INDEX_MASK RX_2_QUEUE_SIZE-1
#define TX_2_QUEUE_INDEX_MASK TX_2_QUEUE_SIZE-1

/*
#define TRUE 1
#define FALSE 0
*/


#define SERIAL_PORT_ONE 1
#define SERIAL_PORT_TWO 2

// if needed, declare functions and global variables that
// are specific to serial port one receiver functionality
#ifdef ENABLE_SERIAL_PORT_ONE_RX
void Init_Serial_Port_One(void);
unsigned char Serial_Port_One_Byte_Count(void);
unsigned char Read_Serial_Port_One(void);
void clear_serial_port_one_rx(void);
void Rx_1_Int_Handler(void);
extern volatile unsigned char RX_1_Framing_Errors;
extern volatile unsigned char RX_1_Overrun_Errors;
#endif

// if needed, declare functions that are specific to serial
// port one transmit functionality
#ifdef ENABLE_SERIAL_PORT_ONE_TX
void Init_Serial_Port_One(void);
void Write_Serial_Port_One(unsigned char);
void Tx_1_Int_Handler(void);
#endif

// if needed, declare functions and global variables that
// are specific to serial port two receiver functionality
#ifdef ENABLE_SERIAL_PORT_TWO_RX
void Init_Serial_Port_Two(void);
unsigned char Serial_Port_Two_Byte_Count(void);
unsigned char Read_Serial_Port_Two(void);
void clear_serial_port_two_rx(void);
void Rx_2_Int_Handler(void);
extern volatile unsigned char RX_2_Framing_Errors;
extern volatile unsigned char RX_2_Overrun_Errors;
#endif

// if needed, declare functions that are specific to serial
// port two transmit functionality
#ifdef ENABLE_SERIAL_PORT_TWO_TX
void Init_Serial_Port_Two(void);
void Write_Serial_Port_Two(unsigned char);
void Tx_2_Int_Handler(void);
#endif

#if 0
#if defined(ENABLE_SERIAL_PORT_ONE_TX) || defined(ENABLE_SERIAL_PORT_TWO_TX)
#define MAX_SERIAL_BUFFER 32
void write_serial_port_str(UINT8, unsigned char *, UINT8);
UINT8 read_serial_port_str(UINT8, unsigned char *);
#endif
#endif

#endif
