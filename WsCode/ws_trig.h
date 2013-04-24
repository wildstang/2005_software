/*******************************************************************************
* FILE NAME: ws_trig.h
*
* DESCRIPTION:
*  This is the include file which corresponds to ws_trig.c
*
*  It contains some aliases and function prototypes used in those files.
*
* USAGE:
*  If you add your own routines to those files, this is a good place to add
*  your custom macros (aliases), type definitions, and function prototypes.
*******************************************************************************/

#ifndef __ws_trig_h_
#define __ws_trig_h_

/******************************* TYPEDEFS *************************************/

/******************************** MACROS **************************************/

/***************************** DEFINITIONS ************************************/
#define COS_SHIFT_FACTOR     8

#ifdef CYGWIN_IFI_SIMULATOR
/* For the simulator, redefine cos to be ws_cos so it doesn't conflict with the
 * built-in "C" cos.
 */
#define cos ws_cos
#endif

/****************************** STRUCTURES ************************************/

/************************* FUNCTION PROTOTYPES ********************************/
extern UINT8 arctan(UINT8, UINT8);
extern UINT8 arctan16(UINT16 opposite, UINT16 adjacent);
extern INT16 cos(UINT8);
extern INT16 sin(UINT8);

#endif /* __ws_trig_h_ */

