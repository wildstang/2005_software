/*******************************************************************************
* FILE NAME: ws_auto_drive.h
*
* DESCRIPTION:
*  This is the include file which corresponds to ws_auto_drive.c
*
*  It contains some aliases and function prototypes used in those files.
*
* USAGE:
*  If you add your own routines to those files, this is a good place to add
*  your custom macros (aliases), type definitions, and function prototypes.
*******************************************************************************/

#ifndef __ws_auto_drive_h_
#define __ws_auto_drive_h_

/******************************* TYPEDEFS *************************************/
typedef enum
{
  DRIVE_TASK_NOT_DONE,
  DRIVE_TASK_DONE,
  DRIVE_TASK_CC_FAIL
} DriveTaskStatusType;

typedef enum
{
  DRIVE_TO_OBJ,
  TURN_TO_OBJ
} TurnHysteresisType;

/******************************** MACROS **************************************/

/***************************** DEFINITIONS ************************************/

/****************************** STRUCTURES ************************************/

/************************* FUNCTION PROTOTYPES ********************************/

extern UINT8 turn_to_orient(UINT8);
extern UINT8 drive_to_approach(INT8, INT8, UINT8, UINT8, UINT8);
extern UINT8 drive_next_to_object(UINT8, UINT8);
extern UINT8 drive_past_object(UINT8, UINT8, UINT8);
extern UINT8 drive_to_distance(INT16, UINT8, UINT8);

#endif /* __ws_auto_drive_h_ */

