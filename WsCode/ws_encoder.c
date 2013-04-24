/*******************************************************************************
*
* TITLE:   ws_encoder.c
*
* VERSION: 0.4 (Beta)
*
* DATE:    06-Jan-2005
*
* AUTHOR:  R. Kevin Watson
*          kevinw@jpl.nasa.gov
*
* COMMENTS: This source code in this file implements an interface to two
*           quadrature output encoders with the assumption that they are
*           coupled to the robot's drive train, one on the left side, the
*           other on the right. Used with suitable PID control software,
*           encoders can be used to control the position and velocity of
*           of your robot. This software was tested with Grayhill 63R256
*           and 61K128 quadrature output optical encoders.
*
*           This source code will work with the Robovation (A/K/A EDU-RC)
*           robot controller and the FIRST Robotics robot controller.
*
*                              ** IMPORTANT **
*
*           On a 40MHz PIC18F8520, this software can track peak encoder
*           count rates as high as a few thousand counts per second, which
*           should be more than adequate for most applications. To meet
*           your performance expectations, selecting the proper Counts Per
*           Revolution (CPR) parameter of your encoder is very important.
*           If the CPR is too high, the robot controller will spend too
*           much time counting encoder "ticks" and not enough time on
*           other tasks. At the extreme, you will see very wacky behavior
*           in your robot controller including corrupted data, the red-
*           light-of-death or the controller may even think the robot is
*           traveling in a direction that it isn't. Selecting a CPR that
*           is too low will not give you the resolution you desire. The
*           CPR should be optimized to minimize the number of interrupts
*           your robot controller will have to service yet meet your
*           resolution expectations (yes, millimeter position resolution
*           to too much to ask for).
*
*           Five things must be done before this software will work
*           correctly on the FRC-RC:
*
*           1) The left encoder's phase-A output is wired to digital input
*           one and the phase-B output is wired to digital I/O six.
*
*           2) The right encoder's phase-A output is wired to digital input
*           two and the phase-B output is wired to digital I/O 8.
*
*           3) Digital I/O pins one, two, six and eight are configured as
*           inputs in user_routines.c/User_Initialization(). If you notice
*           that the encoder only counts in one direction, you forgot to
*           do this step.
*
*           4) A #include statement for the encoder.h header file must be
*           included at the beginning of each source file that calls the
*           functions in this source file. The statement should look like
*           this: #include "encoder.h".
*
*           5) Initialize_Encoders() must be called from user_routines.c/
*           User_Initialization().
*
*
*           Copyright ©2004-2005 R. Kevin Watson. All rights are reserved.
*
********************************************************************************
*
*  CHANGE LOG:
*
*  DATE         REV  DESCRIPTION
*  -----------  ---  ----------------------------------------------------------
*  20-Dec-2003  0.1  RKW - Original code.
*  18-Feb-2004  0.2  RKW - Reassigned the encoder digital inputs to run
*                    on the FRC robot controller too.
*  01-Jan-2005  0.3  RKW - Get_Left_Encoder_Count(), Get_Right_Encoder_Count(),
*                    Set_Left_Encoder_Count() and Set_Right_Encoder_Count()
*                    functions added.
*  01-Jan-2005  0.3  RKW - Renamed Int_1_Handler() and Int_2_Handler() to
*                    Left_Encoder_Int_Handler() and Right_Encoder_Int_Handler
*                    respectively.
*  01-Jan-2005  0.3  RKW - Altered the interrupt service routines to easily
*                    flip the direction the encoders count by altering the
*                    RIGHT_ENCODER_TICK_DELTA and LEFT_ENCODER_TICK_DELTA
*                    #defines found in encoder.h
*  06-Jan-2005  0.4  RKW - Rebuilt with C18 version 3.40.
*
*******************************************************************************/

#include "ifi_picdefs.h"
#include "ifi_aliases.h"
/* can remove below when encoder is in */
#include "ifi_utilities.h"
#include "ws_includes.h"
#include "ws_io.h"
/* can remove above when encoder is in */
#include "ws_encoder.h"

// These variables are used to keep track of the
// encoder position over time. Though these are
// global variables, they shouldn't be modified
// directly. Functions to modify these variables
// are included below.
volatile UINT16 lift_encoder_count = 0;
volatile UINT16 tilt_encoder_count = 0;

/*******************************************************************************
*
* FUNCTION:   Initialize_Encoders()
*
* PURPOSE:    Initializes the encoder software.
*
* PARAMETERS: None
*
* RETURNS:    Nothing
*
* COMMENTS:   Numbers within brackets refer to the PIC18F8520	data
*             sheet page number where more information can be found.
*             This document can be found at microchip's website at
*             http://www.microchip.com or at the author's website at
*             http://www.kevin.org/frc
*
*******************************************************************************/
void initialize_encoders(void)
{
#ifndef NO_ENCODERS
  // Initialize the lift encoder's interrupt. INT2 on the user PIC18F8520
  // is mapped to the interrupt one pin on the EDU-RC and digital I/O one
  // on the FRC-RC.
  TRISBbits.TRISB2 = 1;     // make sure the RB2/INT2 pin is configured as
                            // an input [108]
                            //
  INTCON3bits.INT2IP = 0;   // 0: interrupt 1 is low priority (leave at 0
                            // for IFI controllers) [91]
                            // 1: interrupt 1 is high priority
                            //
  INTCON2bits.INTEDG2 = 1;  // 0: trigger on the falling-edge [90]
                            // 1: trigger on the rising-edge
                            //
  INTCON3bits.INT2IE = 1;   // 0: disable interrupt 1 [91]
                            // 1: enable interrupt 1

  // Initialize the tilt encoder's interrupt. INT3 on the user PIC18F8520
  // is mapped to the interrupt two pin on the EDU-RC and digital I/O two
  // on the FRC-RC.
  TRISBbits.TRISB3 = 1;     // make sure the RB3/CCP2/INT3 pin is configured
                            // as an input [108]
                            //
  INTCON2bits.INT3IP = 0;   // 0: interrupt 2 is low priority (leave at 0
                            // for IFI controllers) [90]
                            // 1: interrupt 2 is high priority
                            //
  INTCON2bits.INTEDG3 = 1;  // 0: trigger on the falling-edge [90]
                            // 1: trigger on the rising-edge
                            //
  INTCON3bits.INT3IE = 1;   // 0: disable interrupt	2 [91]
                            // 1: enable interrupt 2

#endif
}


/*******************************************************************************
*
* FUNCTION:   get_lift_encoder_count()
*
* PURPOSE:    Gets the current number of lift encoder "ticks".
*
* PARAMETERS: None
*
* RETURNS:    Long
*
* COMMENTS:   This function demonstrates the proper way to access
*             a variable that can also be changed by an interrupt
*             service routine.
*
*******************************************************************************/
UINT16 get_lift_encoder_count(void)
{
  UINT16 count;

#ifndef NO_ENCODERS
  // Since we're about to access the lift_encoder_count variable,
  // which can also be modified in the interrupt service routine,
  // let's briefly disable the lift encoder's interrupt to make
  // sure that the lift_encoder_count variable doesn't get altered
  // while we're using it.
  INTCON3bits.INT2IE = 0;

  // Now we can get a local copy of the encoder count without fear
  // that we'll get corrupted data.
  count = lift_encoder_count;

  // Okay, we have a local copy of the encoder count, so turn the
  // lift encoder's interrupt back on.
  INTCON3bits.INT2IE = 1;
#endif

  // Return the encoder count to the caller.
  return(count);
}


/*******************************************************************************
*
* FUNCTION:   set_lift_encoder_count()
*
* PURPOSE:    Sets the current number of lift encoder "ticks".
*
* PARAMETERS: None
*
* RETURNS:    Long
*
* COMMENTS:   This function demonstrates the proper way to access
*             a variable that can also be changed by an interrupt
*             service routine.
*
*******************************************************************************/
void set_lift_encoder_count(UINT16 count)
{
#ifndef NO_ENCODERS
  // Since we're about to access the lift_encoder_count variable,
  // which can also be modified in the interrupt service routine,
  // let's briefly disable the lift encoder's interrupt to make
  // sure that the lift_encoder_count variable doesn't get altered
  // while we're using it.
  INTCON3bits.INT2IE = 0;

  // Now we can set the value of the encoder count without fear
  // that we'll write corrupted data.
  lift_encoder_count = count;

  // Okay, we're done updating the encoder count, so turn the
  // lift encoder's interrupt back on.
  INTCON3bits.INT2IE = 1;
#endif
}


/*******************************************************************************
*
* FUNCTION:   lift_encoder_int_handler()
*
* PURPOSE:    If enabled, the interrupt 1 handler is called when the
*             interrupt 1 pin changes logic level. The edge that the
*             interrupt 1 pin reacts to is programmable (see comments
*             in the Initialize_Encoders() function, above)
*
* PARAMETERS: None
*
* RETURNS:    Nothing
*
* COMMENTS:
*
*******************************************************************************/
void lift_encoder_int_handler(void)
{
  // The lift encoder's phase-A signal just transitioned
  // from zero to one, causing this interrupt service
  // routine to be called. We know that the encoder just
  // rotated one count or "tick" so now check the logical
  // state of the phase-B signal and increment or decrement
  // the lift_encoder_count variable.
  if (LIFT_ENCODER_PHASE_B_PIN == 0)
  {
    lift_encoder_count -= LIFT_ENCODER_TICK_DELTA;
  }
  else
  {
    lift_encoder_count += LIFT_ENCODER_TICK_DELTA;
  }
}


/*******************************************************************************
*
* FUNCTION:   get_tilt_encoder_count()
*
* PURPOSE:    Gets the current number of tilt encoder "ticks".
*
* PARAMETERS: None
*
* RETURNS:    Long
*
* COMMENTS:   This function demonstrates the proper way to access
*             a variable that can also be changed by an interrupt
*             service routine.
*
*******************************************************************************/
UINT16 get_tilt_encoder_count(void)
{
  UINT16 count;

#ifndef NO_ENCODERS
  // Since we're about to access the tilt_encoder_count variable,
  // which can also be modified in the interrupt service routine,
  // let's briefly disable the tilt encoder's interrupt to make
  // sure that the tilt_encoder_count variable doesn't get altered
  // while we're using it.
  INTCON3bits.INT2IE = 0;

  // Now we can get a local copy of the encoder count without fear
  // that we'll get corrupted data.
  count = tilt_encoder_count;

  // Okay, we have a local copy of the encoder count, so turn the
  // tilt encoder's interrupt back on.
  INTCON3bits.INT2IE = 1;
#endif

  // Return the encoder count to the caller.
  return(count);
}


/*******************************************************************************
*
* FUNCTION:   set_tilt_encoder_count()
*
* PURPOSE:    Sets the current number of tilt encoder "ticks".
*
* PARAMETERS: None
*
* RETURNS:    Long
*
* COMMENTS:   This function demonstrates the proper way to access
*             a variable that can also be changed by an interrupt
*             service routine.
*
*******************************************************************************/
void set_tilt_encoder_count(UINT16 count)
{
#ifndef NO_ENCODERS
  // Since we're about to access the tilt_encoder_count variable,
  // which can also be modified in the interrupt service routine,
  // let's briefly disable the tilt encoder's interrupt to make
  // sure that the tilt_encoder_count variable doesn't get altered
  // while we're using it.
  INTCON3bits.INT2IE = 0;

  // Now we can set the value of the encoder count without fear
  // that we'll write corrupted data.
  tilt_encoder_count = count;

  // Okay, we're done updating the encoder count, so turn the
  // tilt encoder's interrupt back on.
  INTCON3bits.INT2IE = 1;
#endif
}


/*******************************************************************************
*
* FUNCTION:   tilt_encoder_int_handler()
*
* PURPOSE:    If enabled, the interrupt 1 handler is called when the
*             interrupt 1 pin changes logic level. The edge that the
*             interrupt 1 pin reacts to is programmable (see comments
*             in the Initialize_Encoders() function, above)
*
* PARAMETERS: None
*
* RETURNS:    Nothing
*
* COMMENTS:
*
*******************************************************************************/
void tilt_encoder_int_handler(void)
{
  // The tilt encoder's phase-A signal just transitioned
  // from zero to one, causing this interrupt service
  // routine to be called. We know that the encoder just
  // rotated one count or "tick" so now check the logical
  // state of the phase-B signal and increment or decrement
  // the tilt_encoder_count variable.
#if 1
  if (TILT_ENCODER_PHASE_B_PIN == 0)
  {
    tilt_encoder_count -= TILT_ENCODER_TICK_DELTA;
  }
  else
  {
    tilt_encoder_count += TILT_ENCODER_TICK_DELTA;
  }

#else

  if ((INTCON2bits.INTEDG3 == 1) && (TILT_ENCODER_PHASE_B_PIN == 0) ||
      (INTCON2bits.INTEDG3 == 0) && (TILT_ENCODER_PHASE_B_PIN == 1))
  {
    tilt_encoder_count -= TILT_ENCODER_TICK_DELTA;
  }
  else
  {
    tilt_encoder_count += TILT_ENCODER_TICK_DELTA;
  }

  // toggle interrupt between triggering on rising & falling edges
  INTCON2bits.INTEDG3 = !INTCON2bits.INTEDG3;
#endif
}

