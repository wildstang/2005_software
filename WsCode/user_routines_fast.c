/*******************************************************************************
* FILE NAME: user_routines_fast.c <FRC VERSION>
*
* DESCRIPTION:
*  This file is where the user can add their custom code within the framework
*  of the routines below.
*
* USAGE:
*  You can either modify this file to fit your needs, or remove it from your
*  project and replace it with a modified copy.
*
* OPTIONS:  Interrupts are disabled and not used by default.
*
*******************************************************************************/

#include "ifi_aliases.h"
#include "ifi_default.h"
#include "ifi_utilities.h"
#include "ws_includes.h"
#include "user_routines.h"
#include "user_Serialdrv.h"
#include "user_camera.h"

#include "ws_encoder.h"

#ifndef USE_CMU_CAMERA
#include "serial_ports.h"
#endif


/****************************
**   VISION VARIABLES      **
*****************************/
#ifdef USE_CMU_CAMERA
extern unsigned char cam_uart_buffer[];
volatile unsigned int data_rdy;
volatile unsigned int cam_index_ptr = 0;  // Zero start of buffer
unsigned int parse_mode = 0;              // Tells it to look for '\r' instead of T packets
#endif


/*******************************************************************************
* FUNCTION NAME: InterruptVectorLow
* PURPOSE:       Low priority interrupt vector
* CALLED FROM:   nowhere by default
* ARGUMENTS:     none
* RETURNS:       void
* DO NOT MODIFY OR DELETE THIS FUNCTION
*******************************************************************************/
#pragma code InterruptVectorLow = LOW_INT_VECTOR
void InterruptVectorLow (void)
{
  _asm
    goto InterruptHandlerLow  /*jump to interrupt routine*/
  _endasm
}


/*******************************************************************************
* FUNCTION NAME: InterruptHandlerLow
* PURPOSE:       Low priority interrupt handler
* If you want to use these external low priority interrupts or any of the
* peripheral interrupts then you must enable them in your initialization
* routine.  Innovation First, Inc. will not provide support for using these
* interrupts, so be careful.  There is great potential for glitchy code if good
* interrupt programming practices are not followed.  Especially read p. 28 of
* the "MPLAB(R) C18 C Compiler User's Guide" for information on context saving.
* CALLED FROM:   this file, InterruptVectorLow routine
* ARGUMENTS:     none
* RETURNS:       void
*******************************************************************************/
#pragma code
#pragma interruptlow InterruptHandlerLow save=PROD,section(".tmpdata") /* You may want to save additional symbols. */

void InterruptHandlerLow ()
{
  unsigned char int_byte;

#ifdef USE_CMU_CAMERA
  if (INTCON3bits.INT2IF && INTCON3bits.INT2IE)       /* The INT2 pin is RB2/DIG I/O 1. */
  {
    INTCON3bits.INT2IF = 0;
  }
  else if (INTCON3bits.INT3IF && INTCON3bits.INT3IE)  /* The INT3 pin is RB3/DIG I/O 2. */
  {
    INTCON3bits.INT3IF = 0;
  }
  else if (INTCONbits.RBIF && INTCONbits.RBIE)  /* DIG I/O 3-6 (RB4, RB5, RB6, or RB7) changed. */
  {
    int_byte = PORTB;          /* You must read or write to PORTB */
    INTCONbits.RBIF = 0;     /*     and clear the interrupt flag         */
  }                                        /*     to clear the interrupt condition.  */
  else
  {
    CheckUartInts();    /* For Dynamic Debug Tool or buffered printf features. */
  }
#else
  if (PIR1bits.RCIF && PIE1bits.RC1IE) // rx1 interrupt?
  {
    #ifdef ENABLE_SERIAL_PORT_ONE_RX
    Rx_1_Int_Handler(); // call the rx1 interrupt handler (in serial_ports.c)
    #endif
  }
  else if (PIR3bits.RC2IF && PIE3bits.RC2IE) // rx2 interrupt?
  {
    #ifdef ENABLE_SERIAL_PORT_TWO_RX
    Rx_2_Int_Handler(); // call the rx2 interrupt handler (in serial_ports.c)
    #endif
  }
  else if (PIR1bits.TX1IF && PIE1bits.TX1IE) // tx1 interrupt?
  {
    #ifdef ENABLE_SERIAL_PORT_ONE_TX
    Tx_1_Int_Handler(); // call the tx1 interrupt handler (in serial_ports.c)
    #endif
  }
  else if (PIR3bits.TX2IF && PIE3bits.TX2IE) // tx2 interrupt?
  {
    #ifdef ENABLE_SERIAL_PORT_TWO_TX
    Tx_2_Int_Handler(); // call the tx2 interrupt handler (in serial_ports.c)
    #endif
  }
#endif

#ifndef NO_ENCODERS
  /* lift encoder interrupt */
  if (INTCON3bits.INT2IF && INTCON3bits.INT2IE)
  {
    INTCON3bits.INT2IF = 0;       // clear the interrupt flag [91]
    lift_encoder_int_handler();   // call the lift encoder interrupt handler
  }
  /* tilt encoder interrupt */
  else if (INTCON3bits.INT3IF && INTCON3bits.INT3IE)
  {
    INTCON3bits.INT3IF = 0;       // clear the interrupt flag [91]
    tilt_encoder_int_handler();   // call the tilt encoder interrupt handler
  }
#endif
}


/*******************************************************************************
* FUNCTION NAME: User_Autonomous_Code
* PURPOSE:       Execute user's code during autonomous robot operation.
* You should modify this routine by adding code which you wish to run in
* autonomous mode.  It will be executed every program loop, and not
* wait for or use any data from the Operator Interface.
* CALLED FROM:   main.c file, main() routine when in Autonomous mode
* ARGUMENTS:     none
* RETURNS:       void
*******************************************************************************/
#if 0
void User_Autonomous_Code(void)
{
  /* Initialize all PWMs and Relays when entering Autonomous mode, or else it
     will be stuck with the last values mapped from the joysticks.  Remember,
     even when Disabled it is reading inputs from the Operator Interface.
  */
    pwm01 = pwm02 = pwm03 = pwm04 = pwm05 = pwm06 = pwm07 = pwm08 = 127;
    pwm09 = pwm10 = pwm11 = pwm12 = pwm13 = pwm14 = pwm15 = pwm16 = 127;
    relay1_fwd = relay1_rev = relay2_fwd = relay2_rev = 0;
    relay3_fwd = relay3_rev = relay4_fwd = relay4_rev = 0;
    relay5_fwd = relay5_rev = relay6_fwd = relay6_rev = 0;
    relay7_fwd = relay7_rev = relay8_fwd = relay8_rev = 0;

  while (autonomous_mode)   /* DO NOT CHANGE! */
  {
    if (statusflag.NEW_SPI_DATA)      /* 26.2ms loop area */
    {
        Getdata(&rxdata);   /* DO NOT DELETE, or you will be stuck here forever! */

        /* Add your own autonomous code here. */

        Generate_Pwms(pwm13,pwm14,pwm15,pwm16);

        Putdata(&txdata);   /* DO NOT DELETE, or you will get no PWM outputs! */
    }
  }
}
#endif

/*******************************************************************************
* FUNCTION NAME: Process_Data_From_Local_IO
* PURPOSE:       Execute user's realtime code.
* You should modify this routine by adding code which you wish to run fast.
* It will be executed every program loop, and not wait for fresh data
* from the Operator Interface.
* CALLED FROM:   main.c
* ARGUMENTS:     none
* RETURNS:       void
*******************************************************************************/
void Process_Data_From_Local_IO(void)
{
  /* Add code here that you want to be executed every program loop. */

}

/*******************************************************************************
* FUNCTION NAME: Serial_Char_Callback
* PURPOSE:       Interrupt handler for the TTL_PORT.
* CALLED FROM:   user_SerialDrv.c
* ARGUMENTS:
*     Argument             Type    IO   Description
*     --------             ----    --   -----------
*     data        unsigned char    I    Data received from the TTL_PORT
* RETURNS:       void
*******************************************************************************/

#ifdef USE_CMU_CAMERA
void Serial_Char_Callback(unsigned char tmp)
{
  /* Add code to handle incomming data (remember, interrupts are still active) */

/*******************************************************************************
	This is the section of the UART rx interrupt routine that handle the camera
	buffer. There are two different modes parse_mode=0 and parse_mode=1.
	
	parse_mode=0
	is for reading ACKs back from the camera.  It will buffer an entire line until
	it reads a '\r' character.  It should be used for non streaming data from the
    camera.

	parse_mode=1
	is for reading tracking packets that are streaming from the camera. It assumes
	the camera is in raw mode and will read until it gets a 'T' character.  It then
	buffers until it gets a 255 bytes which in raw mode signifies the end of line.

	cam_index_ptr - is the counter that keeps track of the current location in the
					buffer.

	data_rdy - is a flag that remains 0 until the entire data packet is ready at
				which point it becomes 1.  Once data_rdy is 1, new packets will be
				ignored.
********************************************************************************/
  if (data_rdy==0)
  {
    if (parse_mode==0)  // Grab single line packets, such as ACKS or NCKS
    {
      if (tmp=='\r' || cam_index_ptr==MAX_BUF_SIZE)
      {
        // Once the end of a packet is found, assert data_rdy and wait
        cam_uart_buffer[cam_index_ptr]=0;
        data_rdy=1;
      }
      else
      {
        cam_uart_buffer[cam_index_ptr]=tmp;
        cam_index_ptr++;
      }
    }
    if (parse_mode==1)   // Grab streaming packets
    {
      if (tmp=='T' )
      {
        data_rdy=0;
        cam_uart_buffer[0]=tmp;
        cam_index_ptr=1;
        return;
      }
      if (cam_index_ptr>0) // Added this to potentially stop unnecessary delays
      {
        if (tmp==255 || cam_index_ptr==MAX_BUF_SIZE)
        {
          if (cam_index_ptr==0 )return;

          // Once the end of a packet is found, assert data_rdy and wait
          cam_uart_buffer[cam_index_ptr]=0;
          data_rdy=1;
        }
        else
        {
          cam_uart_buffer[cam_index_ptr]=tmp;
          cam_index_ptr++;
        }
      }
    }
  }
}
#endif


/******************************************************************************/
/******************************************************************************/
/******************************************************************************/
