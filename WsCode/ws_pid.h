/*******************************************************************************
* FILE NAME: ws_pid.h
*
* DESCRIPTION:
*  This is the include file which corresponds to ws_pid.c
*
*  It contains some aliases and function prototypes used in those files.
*
* USAGE:
*  If you add your own routines to those files, this is a good place to add
*  your custom macros (aliases), type definitions, and function prototypes.
*******************************************************************************/

#ifndef __ws_pid_h_
#define __ws_pid_h_

/******************************* TYPEDEFS *************************************/
typedef struct
{
  INT16  prop_gain;
  INT16  int_gain;
  INT16  deriv_gain;
  UINT16 scale_factor;
  INT16  integral;
  INT16  max_integral;
  INT16  last_error;
  INT16  last_last_error;
  INT16  min_val;
  INT16  max_val;
} PidValsType;

/******************************** MACROS **************************************/

/***************************** DEFINITIONS ************************************/
#ifdef PROTO_ROBOT
/* proto */
#define SET_SPEED_PID_VISION() \
       g_drive_speed_pid_vals.scale_factor = 1; \
       g_drive_speed_pid_vals.prop_gain = 2 * g_drive_speed_pid_vals.scale_factor;\
       g_drive_speed_pid_vals.int_gain = 0;\
       g_drive_speed_pid_vals.deriv_gain = -6;\
       g_drive_speed_pid_vals.max_val = 60;\
       g_drive_speed_pid_vals.min_val = -60;\
       g_drive_speed_pid_vals.max_integral = g_drive_speed_pid_vals.max_val;

#define SET_SPEED_PID_VISION_LOW_GEAR() \
       g_drive_speed_pid_vals.scale_factor = 1; \
       g_drive_speed_pid_vals.prop_gain = 7 * g_drive_speed_pid_vals.scale_factor;\
       g_drive_speed_pid_vals.int_gain = 0;\
       g_drive_speed_pid_vals.deriv_gain = -6;\
       g_drive_speed_pid_vals.max_val = 127;\
       g_drive_speed_pid_vals.min_val = -127;\
       g_drive_speed_pid_vals.max_integral = g_drive_speed_pid_vals.max_val;

#define SET_SPEED_PID_DRIVE_DIST() \
       g_drive_speed_pid_vals.scale_factor = 1; \
       g_drive_speed_pid_vals.prop_gain = 2 * g_drive_speed_pid_vals.scale_factor;\
       g_drive_speed_pid_vals.int_gain = 0;\
       g_drive_speed_pid_vals.deriv_gain = -6;\
       g_drive_speed_pid_vals.max_val = 127;\
       g_drive_speed_pid_vals.min_val = -127;\
       g_drive_speed_pid_vals.max_integral = g_drive_speed_pid_vals.max_val;

#define SET_SPEED_PID_DRIVE_DIST_LOW_GEAR() \
       g_drive_speed_pid_vals.scale_factor = 1; \
       g_drive_speed_pid_vals.prop_gain = 7 * g_drive_speed_pid_vals.scale_factor;\
       g_drive_speed_pid_vals.int_gain = 0;\
       g_drive_speed_pid_vals.deriv_gain = -6;\
       g_drive_speed_pid_vals.max_val = 127;\
       g_drive_speed_pid_vals.min_val = -127;\
       g_drive_speed_pid_vals.max_integral = g_drive_speed_pid_vals.max_val;

#define SET_SPEED_PID_SCORE() \
       g_drive_speed_pid_vals.scale_factor = 2; \
       g_drive_speed_pid_vals.prop_gain = 3 * g_drive_speed_pid_vals.scale_factor;\
       g_drive_speed_pid_vals.int_gain = 0;\
       g_drive_speed_pid_vals.deriv_gain = -3;\
       g_drive_speed_pid_vals.max_val = 50;\
       g_drive_speed_pid_vals.min_val = -50;\
       g_drive_speed_pid_vals.max_integral = g_drive_speed_pid_vals.max_val;

#define SET_TURN_PID_SCORE() \
      g_turn_drive_pid_vals.scale_factor = 1; \
      g_turn_drive_pid_vals.prop_gain = 4 * g_turn_drive_pid_vals.scale_factor; \
      g_turn_drive_pid_vals.int_gain = 1; \
      g_turn_drive_pid_vals.deriv_gain = 0; \
      g_turn_drive_pid_vals.max_integral = 10; \
      g_turn_drive_pid_vals.max_val = 60; \
      g_turn_drive_pid_vals.min_val = -60; \

#else
/* real robot */
#define SET_SPEED_PID_VISION() \
       g_drive_speed_pid_vals.scale_factor = 1; \
       g_drive_speed_pid_vals.prop_gain = 2 * g_drive_speed_pid_vals.scale_factor;\
       g_drive_speed_pid_vals.int_gain = 0;\
       g_drive_speed_pid_vals.deriv_gain = -7;\
       g_drive_speed_pid_vals.max_val = 55;\
       g_drive_speed_pid_vals.min_val = -55;\
       g_drive_speed_pid_vals.max_integral = g_drive_speed_pid_vals.max_val;

#define SET_SPEED_PID_VISION_LOW_GEAR() \
       g_drive_speed_pid_vals.scale_factor = 1; \
       g_drive_speed_pid_vals.prop_gain = 5 * g_drive_speed_pid_vals.scale_factor;\
       g_drive_speed_pid_vals.int_gain = 0;\
       g_drive_speed_pid_vals.deriv_gain = -8;\
       g_drive_speed_pid_vals.max_val = 70;\
       g_drive_speed_pid_vals.min_val = -70;\
       g_drive_speed_pid_vals.max_integral = g_drive_speed_pid_vals.max_val;

#define SET_SPEED_PID_DRIVE_DIST() \
       g_drive_speed_pid_vals.scale_factor = 1; \
       g_drive_speed_pid_vals.prop_gain = 2 * g_drive_speed_pid_vals.scale_factor;\
       g_drive_speed_pid_vals.int_gain = 0;\
       g_drive_speed_pid_vals.deriv_gain = -6;\
       g_drive_speed_pid_vals.max_val = 127;\
       g_drive_speed_pid_vals.min_val = -127;\
       g_drive_speed_pid_vals.max_integral = g_drive_speed_pid_vals.max_val;

#define SET_SPEED_PID_DRIVE_DIST_LOW_GEAR() \
       g_drive_speed_pid_vals.scale_factor = 1; \
       g_drive_speed_pid_vals.prop_gain = 7 * g_drive_speed_pid_vals.scale_factor;\
       g_drive_speed_pid_vals.int_gain = 0;\
       g_drive_speed_pid_vals.deriv_gain = -6;\
       g_drive_speed_pid_vals.max_val = 127;\
       g_drive_speed_pid_vals.min_val = -127;\
       g_drive_speed_pid_vals.max_integral = g_drive_speed_pid_vals.max_val;

#define SET_SPEED_PID_SCORE() \
       g_drive_speed_pid_vals.scale_factor = 2; \
       g_drive_speed_pid_vals.prop_gain = 2 * g_drive_speed_pid_vals.scale_factor;\
       g_drive_speed_pid_vals.int_gain = 0;\
       g_drive_speed_pid_vals.deriv_gain = -4;\
       g_drive_speed_pid_vals.max_val = 60;\
       g_drive_speed_pid_vals.min_val = -60;\
       g_drive_speed_pid_vals.max_integral = g_drive_speed_pid_vals.max_val;

#define SET_TURN_PID_SCORE() \
       g_turn_drive_pid_vals.scale_factor = 1; \
       g_turn_drive_pid_vals.prop_gain = 2 * g_turn_drive_pid_vals.scale_factor; \
       g_turn_drive_pid_vals.int_gain = 1; \
       g_turn_drive_pid_vals.deriv_gain = -5; \
       g_turn_drive_pid_vals.max_integral = 10; \
       g_turn_drive_pid_vals.max_val = 50; \
       g_turn_drive_pid_vals.min_val = -50; \

#if 0
/* values tuned before ship */
#define SET_SPEED_PID_VISION() \
       g_drive_speed_pid_vals.scale_factor = 1; \
       g_drive_speed_pid_vals.prop_gain = 2 * g_drive_speed_pid_vals.scale_factor;\
       g_drive_speed_pid_vals.int_gain = 0; /*2;*/ \
       g_drive_speed_pid_vals.deriv_gain = -5; \
       g_drive_speed_pid_vals.max_val = 80;\
       g_drive_speed_pid_vals.min_val = -80;\
       g_drive_speed_pid_vals.max_integral = 30;


#define SET_SPEED_PID_SCORE() \
       g_drive_speed_pid_vals.scale_factor = 2; \
       g_drive_speed_pid_vals.prop_gain = 2 * g_drive_speed_pid_vals.scale_factor;\
       g_drive_speed_pid_vals.int_gain = 0;\
       g_drive_speed_pid_vals.deriv_gain = 0;\
       g_drive_speed_pid_vals.max_val = 50;\
       g_drive_speed_pid_vals.min_val = -50;\
       g_drive_speed_pid_vals.max_integral = g_drive_speed_pid_vals.max_val;

#endif
#endif




/****************************** STRUCTURES ************************************/

/************************* FUNCTION PROTOTYPES ********************************/
extern INT16 ws_pid(PidValsType *, INT16, INT16);

#endif /* __ws_pid_h_ */

