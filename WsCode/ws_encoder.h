/*******************************************************************************
*
* TITLE:    ws_encoder.h
*
* VERSION:  0.4 (Beta)
*
* DATE:	    06-Jan-2005
*
* AUTHOR:   R. Kevin Watson
*
* COMMENTS: You are free to use this source code for any non-commercial
*           use. Please do not make copies of this source code, modified
*           or un-modified, publicly available on the internet or
*           elsewhere without permission. Thanks.
*
*           Copyright ©2004-2005 R. Kevin Watson. All rights are reserved.
*
********************************************************************************
*
* CHANGE LOG:
*
* DATE         REV  DESCRIPTION
* -----------  ---  ----------------------------------------------------------
* 20-Dec-2003  0.1  RKW - Original code.
* 18-Feb-2004  0.2  RKW - Reassigned the encoder digital inputs to run
*                   on the FRC robot controller too.
* 01-Jan-2005  0.3  RKW - Get_Left_Encoder_Count(), Get_Right_Encoder_Count(),
*                   Set_Left_Encoder_Count() and Set_Right_Encoder_Count()
*                   functions added.
* 01-Jan-2005  0.3  RKW - Renamed Int_1_Handler() and Int_2_Handler() to
*                   Left_Encoder_Int_Handler() and Right_Encoder_Int_Handler
*                   respectively.
* 01-Jan-2005  0.3  RKW - Altered the interrupt service routines to easily
*                   flip the direction the encoders count by altering the
*                   RIGHT_ENCODER_TICK_DELTA and LEFT_ENCODER_TICK_DELTA
*                   #defines found in encoder.h
* 06-Jan-2005  0.4  RKW - Rebuilt with C18 version 3.40.
*
*******************************************************************************/

#ifndef __ws_encoder_h_
#define __ws_encoder_h_

// Digital input pin assigned to the lift
// encoder's phase-B output. Make sure this pin
// is configured as an input in user_routines.c/
// User_Initialization().
#define LIFT_ENCODER_PHASE_B_PIN    Dig_in_lift_encoder_2
#define TILT_ENCODER_PHASE_B_PIN    Dig_in_tilt_encoder_2

// change the sign of these if you need to flip
// the way the encoders count. For instance, if
// a given encoder count increases in the positive
// direction when rotating counter-clockwise, but
// you want it to count in the negative direction
// when rotating in the counter-clockwise direction,
// flip the sign and it'll work the way you need it
// to. By default, the left encoder is -1 because
// left/right mounted encoders will count in the
// opposite direction when the 'bot is moving in
// a straight line.
#define LIFT_ENCODER_TICK_DELTA      -1
#define TILT_ENCODER_TICK_DELTA      -1

// function prototypes
void initialize_encoders(void);        // call this to initialize the encoder
                                       // software

UINT16 get_lift_encoder_count(void);   // call this to get the current lift
                                       // encoder count

void set_lift_encoder_count(UINT16);   // call this to set the lift encoder
                                       // count

void lift_encoder_int_handler(void);   // lift encoder interrupt handler

UINT16 get_tilt_encoder_count(void);   // call this to get the current tilt
                                       // encoder count

void set_tilt_encoder_count(UINT16);   // call this to set the tilt encoder
                                       // count

void tilt_encoder_int_handler(void);   // tilt encoder interrupt handler

#endif

