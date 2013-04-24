/*******************************************************************************
* FILE NAME: ws_camera.h
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

#ifndef __ws_camera_h_
#define __ws_camera_h_

/******************************* TYPEDEFS *************************************/
typedef enum
{
  CAM_STATE_BOOT,
  CAM_STATE_INIT,
  CAM_STATE_AUTO_SERVO,
  CAM_STATE_PROCESSING,
  CAM_STATE_DELAY
} CAM_STATE;

/******************************** MACROS **************************************/

/***************************** DEFINITIONS ************************************/

#define CAMERA_EXPOSURE_TETRA  85

/****************************** STRUCTURES ************************************/

/************************* FUNCTION PROTOTYPES ********************************/
extern void camera_state_machine(int *, int *, int *);
extern void camera_process(int *, int *, int *);

#endif /* __ws_camera_h_ */

