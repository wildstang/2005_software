/*******************************************************************************
* FILE NAME: ws_calibrate.h
*
* DESCRIPTION:
*  This is the include file which corresponds to ws_calibrate.c
*
*  It contains some aliases and function prototypes used in those files.
*
* USAGE:
*  If you add your own routines to those files, this is a good place to add
*  your custom macros (aliases), type definitions, and function prototypes.
*******************************************************************************/

#ifndef __ws_calibrate_h_
#define __ws_calibrate_h_

/******************************* TYPEDEFS *************************************/
typedef enum
{
  DISPLAY_NONE,
  DISPLAY_DRIVE_X,
  DISPLAY_DRIVE_Y,
  DISPLAY_LIFT_Y,
  DISPLAY_LIFT_X,
  DISPLAY_LIFT_ENCODER,
  DISPLAY_TILT_ENCODER
} DISPLAY_TYPE;

/******************************** MACROS **************************************/

/***************************** DEFINITIONS ************************************/
#define CALIBRATE_ZONE      50

#define CALIBRATE_KNOWN_BYTE_1  0xCD
#define CALIBRATE_KNOWN_BYTE_2  0xEF

/* bitmasks to determine if calibration value has been set */
#define CAL_MASK_LIFT_HEIGHT_BOTTOM       0x01
#define CAL_MASK_LIFT_HEIGHT_TOP          0x02
#define CAL_MASK_LIFT_HEIGHT_LOAD_BOTTOM  0x04
#define CAL_MASK_LIFT_HEIGHT_LOAD_TOP     0x08
#define CAL_MASK_LIFT_HEIGHT_SCORE        0x10

#define CAL_MASK_LIFT_TILT_LEFT           0x01
#define CAL_MASK_LIFT_TILT_VERTICAL       0x02


#define DEFAULT_LIFT_TILT_LEFT            50
#define DEFAULT_LIFT_TILT_VERTICAL        100

/****************************** STRUCTURES ************************************/

/************************* FUNCTION PROTOTYPES ********************************/
#if 0
extern void calibrate_pots(void);
extern void retrieve_calibration(void);
#endif
extern void display_calibration(void);

#endif /* __ws_calibrate_h_ */

