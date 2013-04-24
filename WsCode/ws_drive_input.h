/*******************************************************************************
* FILE NAME: ws_drive_input.h
*
* DESCRIPTION:
*  This is the include file which corresponds to ws_drive_input.c
*
*  It contains some aliases and function prototypes used in those files.
*
* USAGE:
*  If you add your own routines to those files, this is a good place to add
*  your custom macros (aliases), type definitions, and function prototypes.
*******************************************************************************/

#ifndef __ws_drive_input_h_
#define __ws_drive_input_h_

/******************************* TYPEDEFS *************************************/

/******************************** MACROS **************************************/

/***************************** DEFINITIONS ************************************/

/*** Scaling constants ***/
#define DRIVE_SCALE_NUMERATOR     2  /* these two constants make up the */
#define DRIVE_SCALE_DENOMINATOR   3  /* scaling factor for anti-turbo */

#define DRIVE_DEADZONE            5
#define DRIVE_ACCEL_RATE          35

/****************************** STRUCTURES ************************************/

/************************* FUNCTION PROTOTYPES ********************************/
extern void  drive_stick_input(UINT8);
extern INT16 drive_acceleration_adjust(INT16, INT16);

#endif /* __ws_drive_input_h_ */

