#include <stdio.h>
#include "ifi_aliases.h"
#include "ifi_default.h"
#include "ifi_utilities.h"
#include "ws_includes.h"
#include "user_routines.h"
#include "delays.h"       /*defined locally*/
#include "ws_io.h"
#include "ws_cc.h"
#include "ws_lift.h"
#include "ws_encoder.h"

#include "ws_autonomous.h"
#include "ws_autonomous_tasks.h"
#include "ws_auto_drive.h"


#define KNOCK_HANG_LEFT_DRIVE_DIST   -260
#define KNOCK_HANG_LEFT_DRIVE_SPEED   -40
#define KNOCK_HANG_RIGHT_DRIVE_DIST   220
#define KNOCK_HANG_RIGHT_DRIVE_SPEED   40

#define KNOCK_HANG_LIFT_HEIGHT   LIFT_DOWN_ENCODER_VAL + 225


#define KNOCK_HANG_LEFT_DRIVE_AWAY_DIST   -600
#define KNOCK_HANG_LEFT_DRIVE_AWAY_SPEED   -70
#define KNOCK_HANG_RIGHT_DRIVE_AWAY_DIST   600
#define KNOCK_HANG_RIGHT_DRIVE_AWAY_SPEED   70


#define KNOCK_HANG_SPEAR_RETRACT_DELAY   35
#define KNOCK_HANG_SPEAR_DRIVE_AWAY_DELAY 42

extern UINT8 g_starting_pos;

UINT8 auto_task_knock_hang(void *params)
{
#if AUTO_USE_KNOCK
  static UINT8 local_state = KNOCK_HANG_INITIALIZE;
  static UINT8 loop_count = 0;
  static UINT8 orig_orient;
  static INT16 drive_dist_tgt;
  static UINT8 drive_speed;
  static INT16 drive_away_dist_tgt;
  static UINT8 drive_away_speed;
  UINT8 ret_state;
  UINT8 ret_val;
  UINT16 pos;
  EncoderValsType encoder_vals;

  if(local_state == KNOCK_HANG_INITIALIZE)
  {
    loop_count = 0;
    /* Reset the CC values by requesting the current
     * values - the result isn't used here */
    ret_val = cc_get_encoder_vals(&encoder_vals);

    if(ret_val == CC_SUCCESS)
    {
#if PRINT_TASK_INFO
    printf("TASK STARTED: KNOCK HANG\r");
#endif
      ret_state = TASK_STATE_PROCESSING;

      orig_orient = encoder_vals.orient;
      /* Set the parameters based on starting position */
      switch(g_starting_pos)
      {
        case STARTING_POS_BLUE_LEFT:
        case STARTING_POS_RED_LEFT:
          drive_away_dist_tgt = KNOCK_HANG_LEFT_DRIVE_AWAY_DIST;
          drive_away_speed = KNOCK_HANG_LEFT_DRIVE_AWAY_SPEED;
          drive_dist_tgt = KNOCK_HANG_LEFT_DRIVE_DIST;
          drive_speed = KNOCK_HANG_LEFT_DRIVE_SPEED;
          break;
        case STARTING_POS_BLUE_CENTER:
        case STARTING_POS_RED_CENTER:
          drive_away_dist_tgt = 0;
          drive_away_speed = 0;
          drive_dist_tgt = 0;
          drive_speed = 0;
          break;
        case STARTING_POS_BLUE_RIGHT:
        case STARTING_POS_RED_RIGHT:
          drive_away_dist_tgt = KNOCK_HANG_RIGHT_DRIVE_AWAY_DIST;
          drive_away_speed = KNOCK_HANG_RIGHT_DRIVE_AWAY_SPEED;
          drive_dist_tgt = KNOCK_HANG_RIGHT_DRIVE_DIST;
          drive_speed = KNOCK_HANG_RIGHT_DRIVE_SPEED;
          break;
        default:
          break;
      }

      if(drive_dist_tgt == 0)
      {
        local_state = KNOCK_HANG_DONE;
      }
      else
      {
        /* Set local state to the next state to be run */
        local_state = KNOCK_HANG_DRIVE_FWD;
      }
    }
  }

  switch(local_state)
  {
    case KNOCK_HANG_DRIVE_FWD:
      ret_state = TASK_STATE_PROCESSING;
      ret_val = drive_to_distance(drive_dist_tgt,drive_speed,SHIFT_SWITCH_LOW);

      if(ret_val == DRIVE_TASK_DONE)
      {
#if PRINT_TASK_INFO
          printf("DRIVE DONE\r");
#endif
          local_state = KNOCK_HANG_POSITION_LIFT;
          loop_count = 0;
      }
      else if(ret_val == DRIVE_TASK_CC_FAIL)
      {
        motor_vals.left_drive = 0;
        motor_vals.right_drive = 0;
      }
      else
      {
#if PRINT_TASK_INFO
        printf("DRIVE %d %d\r",(int)g_autodrive_ticks,(int)drive_dist_tgt);
#endif
      }
      break;
    case KNOCK_HANG_POSITION_LIFT:
      /*  Get the current lift position */
      pos = get_lift_encoder_count();
#if PRINT_TASK_INFO
      printf("Lifting (%d)...\r",pos);
#endif
      /* Untilt the lift */
      tilt_set_pos(TILT_POSITION_STRAIGHT_UP);

      /* Raise the lift -
       * Go a little higher so that we make sure the
       * spear goes out*/
      lift_set_height(KNOCK_HANG_LIFT_HEIGHT);
      if(pos >= (KNOCK_HANG_LIFT_HEIGHT))
      {
#if PRINT_TASK_INFO
        printf("LIFT DONE\r");
#endif
        local_state = KNOCK_HANG_SET_SPEAR;
        loop_count = 0;
      }
      break;
    case KNOCK_HANG_SET_SPEAR:

      ret_state = TASK_STATE_PROCESSING;
      loop_count++;
      if ((loop_count > KNOCK_HANG_SPEAR_RETRACT_DELAY) &&
          (loop_count < (KNOCK_HANG_SPEAR_RETRACT_DELAY + KNOCK_HANG_SPEAR_DRIVE_AWAY_DELAY)))
      {
#if PRINT_TASK_INFO
        printf("Pull Spear In %d\r",loop_count);
#endif
        motor_vals.bot_spear_retract =  SPEAR_IN;
      }
      else if(loop_count >= (KNOCK_HANG_SPEAR_RETRACT_DELAY + KNOCK_HANG_SPEAR_DRIVE_AWAY_DELAY))
      {
#if PRINT_TASK_INFO
        printf("Time to drive away %d\r",loop_count);
#endif
        motor_vals.bot_spear_retract =  SPEAR_IN;
        local_state = KNOCK_HANG_DRIVE_AWAY;
        loop_count = 0;
        g_autodrive_ticks = 0;
      }
      else
      {
#if PRINT_TASK_INFO
      printf("Opening...\r");
#endif
        motor_vals.bot_spear_retract =  SPEAR_OUT;
      }
      break;
   case KNOCK_HANG_DRIVE_AWAY:
      ret_val = drive_to_distance(drive_away_dist_tgt,drive_away_speed,SHIFT_SWITCH_LOW);

      if(ret_val == DRIVE_TASK_DONE)
      {
#if PRINT_TASK_INFO
          printf("DRIVE PAST GOAL\r");
#endif
          local_state = KNOCK_HANG_LOWER_LIFT;
          loop_count = 0;
      }
      else if(ret_val == DRIVE_TASK_CC_FAIL)
      {
        motor_vals.left_drive = 0;
        motor_vals.right_drive = 0;
      }
      else
      {
#if PRINT_TASK_INFO
        printf("DRIVE %d %d\r",(int)g_autodrive_ticks,(int)drive_away_dist_tgt);
#endif
      }
    case KNOCK_HANG_LOWER_LIFT:
#if PRINT_TASK_INFO
      printf("Lifting...\r");
#endif

      /* Lower the lift -
       * Go a little higher so that we make sure the
       * spear goes out*/
      pos = get_lift_encoder_count();
      lift_set_height(LIFT_HEIGHT_LIMIT_BOTTOM);
      ret_state = TASK_STATE_PROCESSING;
      if(pos <= (LIFT_HEIGHT_LIMIT_BOTTOM))
      {
        local_state = KNOCK_HANG_DONE;
      }
      break;
   case KNOCK_HANG_DONE:
#if PRINT_TASK_INFO
    printf("KNOCK HANG TASK DONE\r");
#endif
      ret_state = TASK_STATE_DONE;
      local_state = KNOCK_HANG_INITIALIZE;
      break;
   default:
      break;
  }

  return ret_state;
#else
#if PRINT_TASK_INFO
    printf("TASK SKIPPED: KNOCK HANG\r");
#endif
  return TASK_STATE_DONE;
#endif
}

UINT8 auto_task_grab_hang(void *params)
{
#if AUTO_USE_GRAB_HANG
  static UINT8 local_state = GRAB_HANG_INITIALIZE;
  static UINT8 goal_id;
  UINT8 ret_state;
  if(local_state == GRAB_HANG_INITIALIZE)
  {
    /* Do anything that needs to be done the
       first time here */
    goal_id = ((GrabHangParamType *)params)->goal_id;
    ret_state = TASK_STATE_PROCESSING;
    local_state = GRAB_HANG_DONE;
#if PRINT_TASK_INFO
    printf("TASK STARTED: GRAB HANG - goal (%d)\r",(int) goal_id);
#endif
  }

  /* Add code that will do whatever this
     task is supposed to do

     When done, the ret_state should be set to
     TASK_STATE_DONE and the local_state should
     be set to TASK_STATE_INITIALIZE
  */
#if PRINT_TASK_INFO
    printf("GRAB HANG TASK DONE\r");
#endif
  ret_state = TASK_STATE_DONE;
  local_state = GRAB_HANG_INITIALIZE;

  return ret_state;
#else
#if PRINT_TASK_INFO
    printf("TASK SKIPPED: Grab Hang\r");
#endif
  return TASK_STATE_DONE;
#endif
}

