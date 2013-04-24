/*******************************************************************************
* FILE NAME: ws_general.h
*
* DESCRIPTION:
*  This is the include file which corresponds to ws_general.c
*
*  It contains some aliases and function prototypes used in those files.
*
* USAGE:
*  If you add your own routines to those files, this is a good place to add
*  your custom macros (aliases), type definitions, and function prototypes.
*******************************************************************************/

#ifndef __ws_general_h_
#define __ws_general_h_

/******************************* TYPEDEFS *************************************/

typedef enum
{
  DISPLAY_DATA_PSI,
  DISPLAY_DATA_AUTO,
  DISPLAY_DATA_CALIBRATE,
  DISPLAY_DATA_OVERRIDE
} DisplayDataType;

/******************************** MACROS **************************************/

/***************************** DEFINITIONS ************************************/
#define PUMP_HI_VAL   223     /* point where we turn pump off */
#define PUMP_LO_VAL   196     /* point where we turn pump on */
#define PUMP_TOP_OFF  300     /* Number of cycles to keep pump on after
                                 digital switch triggers.*/

#define WING_LOCK_HAT     HAT_DOWN
#define WING_UNLOCK_HAT   HAT_UP

/****************************** STRUCTURES ************************************/

/************************* FUNCTION PROTOTYPES ********************************/
extern void display_oi_data(UINT8, DisplayDataType);
extern void pump_control(void);
extern void wing_control(void);
extern UINT8 toggle_on_tap( UINT8 cur_button, UINT8 prev_button,
                            UINT8 prev_state, UINT8 state1,
                            UINT8 state2);
#endif /* __ws_general_h_ */

