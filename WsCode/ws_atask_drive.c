#include <stdio.h>
#include "ifi_aliases.h"
#include "ifi_default.h"
#include "ifi_utilities.h"
#include "ws_includes.h"
#include "user_routines.h"
#include "delays.h"       /*defined locally*/
#include "ws_cc.h"
#include "ws_pid.h"
#include "ws_io.h"

#include "ws_autonomous.h"
#include "ws_autonomous_tasks.h"
#include "ws_auto_drive.h"
#include "ws_drive_input.h"

extern PidValsType g_drive_speed_pid_vals;
UINT8 auto_task_drive_dist(void *,UINT8);

UINT8 auto_task_drive_point(void *params)
{
#if AUTO_USE_DRIVE_POINT
  static UINT8 local_state = DRIVE_POINT_INITIALIZE;
  static UINT8 target_x,target_y,target_orient;

  UINT8 ret_state;
  if(local_state == DRIVE_POINT_INITIALIZE)
  {
    /* Do anything that needs to be done the
       first time here */
    target_x = ((DrivePointParamType *)params)->target_x;
    target_y = ((DrivePointParamType *)params)->target_y;
    target_orient = ((DrivePointParamType *)params)->target_orient;
    ret_state = TASK_STATE_PROCESSING;
    /* Set local state to the next state to be run */
    local_state = DRIVE_POINT_DONE;

#if PRINT_TASK_INFO
    printf("TASK STARTED: DRIVE POINT - x (%d)  y (%d)  orient (%d)\r",(int) target_x, (int)target_y,(int)target_orient);
#endif
  }

  /* Add code that will do whatever this
     task is supposed to do

     When done, the ret_state should be set to
     TASK_STATE_DONE and the local_state should
     be set to TASK_STATE_INITIALIZE
  */
#if PRINT_TASK_INFO
    printf("DRIVE POINT TASK DONE\r");
#endif
  ret_state = TASK_STATE_DONE;
  local_state = DRIVE_POINT_INITIALIZE;

  return ret_state;
#else
#if PRINT_TASK_INFO
    printf("TASK SKIPPED: Drive Point\r");
#endif
  return TASK_STATE_DONE;
#endif
}

UINT8 auto_task_drive_direct(void *params)
{
#if AUTO_USE_DRIVE_DIRECT
  static UINT8 local_state = DRIVE_DIRECT_INITIALIZE;
  static UINT16 counter = 0;
  UINT8 ret_state;
  static INT8 speed;
  static UINT16 run_cycles;
  static UINT8 orient,seconds;


  if(local_state == DRIVE_DIRECT_INITIALIZE)
  {
    counter = 0;
    seconds = ((DriveDirectParamType *)params)->seconds;
    run_cycles = NUM_PACKETS_PER_SEC * (UINT16) (seconds);
    speed = ((DriveDirectParamType *)params)->speed;
    orient = ((DriveDirectParamType *)params)->orient;
    MIN_MAX(speed,-127,127);
#if PRINT_TASK_INFO
    printf("TASK STARTED: DRIVE DIRECT (%d) seconds at speed (%d) orient (%d)\r",(int)seconds,(int) speed, (int) orient);
#endif
    ret_state = TASK_STATE_PROCESSING;
    /* Set local state to the next state to be run */
    local_state = DRIVE_DIRECT_DRIVE;
  }

  if(counter >= run_cycles)
  {
#if PRINT_TASK_INFO
    printf("DRIVE DIRECT TASK DONE\r");
#endif
    ret_state = TASK_STATE_DONE;
    local_state = DRIVE_DIRECT_INITIALIZE;
  }
  else
  {
    counter++;
    ret_state = TASK_STATE_PROCESSING;
  }

  motor_vals.left_drive = speed;
  motor_vals.right_drive = speed;
  return ret_state;
#else
#if PRINT_TASK_INFO
    printf("TASK SKIPPED: Drive Direct\r");
#endif
  return TASK_STATE_DONE;
#endif
}

UINT8 auto_task_drive_tank(void *params)
{
#if AUTO_USE_DRIVE_TANK
  static UINT8 local_state = DRIVE_TANK_INITIALIZE;
  static UINT16 counter = 0;
  UINT8 ret_state,seconds;
  static INT8 left_speed,right_speed;
  static UINT16 run_cycles;

  if(local_state == DRIVE_TANK_INITIALIZE)
  {
    counter = 0;
    seconds = ((DriveTankParamType *)params)->seconds;
    run_cycles = NUM_PACKETS_PER_SEC * (UINT16)seconds;
    left_speed = ((DriveTankParamType *)params)->left_speed;
    right_speed = ((DriveTankParamType *)params)->right_speed;
    MIN_MAX(left_speed,-127,127);
    MIN_MAX(right_speed,-127,127);
#if PRINT_TASK_INFO
    printf("TASK STARTED: DRIVE TANK(%d) seconds L_speed (%d) R_speed (%d)\r",(int)seconds,(int) left_speed, (int) right_speed);
#endif
    ret_state = TASK_STATE_PROCESSING;
    local_state = DRIVE_TANK_DRIVE;
  }

  if(counter >= run_cycles)
  {
#if PRINT_TASK_INFO
    printf("DRIVE TANK TASK DONE\r");
#endif
    ret_state = TASK_STATE_DONE;
    local_state = DRIVE_TANK_DONE;
  }
  else
  {
    counter++;
    ret_state = TASK_STATE_PROCESSING;
    motor_vals.left_drive = left_speed;
    motor_vals.right_drive = right_speed;
  }

  return ret_state;
#else
#if PRINT_TASK_INFO
    printf("TASK SKIPPED: Drive Tank\r");
#endif
  return TASK_STATE_DONE;
#endif
}

UINT8 auto_task_drive_dist_low(void *params)
{
  return auto_task_drive_dist(params,SHIFT_SWITCH_LOW);
}

UINT8 auto_task_drive_dist_high(void *params)
{
  return auto_task_drive_dist(params,SHIFT_SWITCH_HIGH);
}

UINT8 auto_task_drive_dist(void *params,UINT8 gear)
{
#if AUTO_USE_DRIVE_DIST
  static UINT8 local_state = DRIVE_DIST_INITIALIZE;
  static INT16 dist;
  static INT8 speed;
  UINT8 ret_state = TASK_STATE_INITIALIZE;
  UINT8 ret_val;
  EncoderValsType encoder_vals;

  if(local_state == DRIVE_DIST_INITIALIZE)
  {
    speed = ((DriveDistParamType *)params)->speed;
    dist = ((DriveDistParamType *)params)->dist;
    if(speed < 0)
    {
      printf("INVERT DISTANCE\r");
      dist = -dist;
    }
    MIN_MAX(speed,-127,127);

    ret_val = cc_get_encoder_vals(&encoder_vals);
    g_autodrive_ticks = 0;
#if PRINT_TASK_INFO
    printf("TASK STARTED: DRIVE DIST (%d) ticks at speed (%d) (%d)\r",(int)dist,(int) speed,(int)g_autodrive_ticks);
#endif

    if(ret_val == CC_SUCCESS)
    {
      ret_state = TASK_STATE_PROCESSING;
      /* Set local state to the next state to be run */
      local_state = DRIVE_DIST_DRIVE;
    }
  }

  switch(local_state)
  {
    case DRIVE_DIST_DRIVE:
      ret_state = TASK_STATE_PROCESSING;
      ret_val = drive_to_distance((INT16)dist,speed,gear);
      if(ret_val == DRIVE_TASK_DONE)
      {
#if PRINT_TASK_INFO
          printf("DRIVE DISTANCE DONE\r");
#endif
          local_state = DRIVE_DIST_DONE;
      }
      else if(ret_val == DRIVE_TASK_CC_FAIL)
      {
        //local_state = DRIVE_DIST_INITIALIZE;
        //ret_state = TASK_STATE_ABORT;
        motor_vals.left_drive = 0;
        motor_vals.right_drive = 0;
      }
      else
      {
#if PRINT_TASK_INFO
        printf("DRIVE DIST %d %d\r",(int)g_autodrive_ticks,(int)dist);
#endif
      }
      break;
    case DRIVE_DIST_DONE:
        motor_vals.left_drive = 0;
        motor_vals.right_drive = 0;
        local_state = DRIVE_DIST_INITIALIZE;
        ret_state = TASK_STATE_DONE;
#if PRINT_TASK_INFO
    printf("DRIVE DIST TASK DONE\r");
#endif
      break;
    default:
      break;
  }
  /*
  printf("Cur: %d Tgt: %d\r", cur_dist,dist);
  */
  return ret_state;
#else
#if PRINT_TASK_INFO
    printf("TASK SKIPPED: Drive Distance\r");
#endif
  return TASK_STATE_DONE;
#endif
}

UINT8 auto_task_rotate(void *params)
{
#if AUTO_USE_ROTATE
  static UINT8 local_state = ROTATE_INITIALIZE;
  static UINT8 angle;
  UINT8 ret_state = TASK_STATE_INITIALIZE;
  UINT8 ret_val;
  EncoderValsType encoder_vals;

  if(local_state == ROTATE_INITIALIZE)
  {
    ret_val = cc_get_encoder_vals(&encoder_vals);

    if(ret_val == CC_SUCCESS)
    {
      angle = encoder_vals.orient + ((RotateParamType *)params)->angle;
#if PRINT_TASK_INFO
    printf("TASK STARTED: ROTATE (%d) BRADS TO %d\r",(int)encoder_vals.orient,(int)angle);
#endif
      ret_state = TASK_STATE_PROCESSING;
      /* Set local state to the next state to be run */
      local_state = ROTATE_DRIVE;
    }
  }

  switch(local_state)
  {
    case ROTATE_DRIVE:
      if (turn_to_orient(angle) == DRIVE_TASK_DONE)
      {
        local_state = ROTATE_DONE;
      }
      ret_state = TASK_STATE_PROCESSING;
      break;
    case ROTATE_DONE:
        motor_vals.left_drive = 0;
        motor_vals.right_drive = 0;
        ret_state = TASK_STATE_DONE;
        local_state = ROTATE_INITIALIZE;
      break;
    default:
      break;
  }
  /*
  printf("Cur: %d Tgt: %d\r", cur_dist,dist);
  */
  return ret_state;
#else
#if PRINT_TASK_INFO
    printf("TASK SKIPPED: Rotate\r");
#endif
  return TASK_STATE_DONE;
#endif
}

