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

#include "ws_drive_input.h"
#include "ws_encoder.h"
#include "ws_lift.h"
#include "ws_autonomous.h"
#include "ws_autonomous_tasks.h"
#include "ws_auto_drive.h"

#define DELIVER_TILT 22
#define DELIVER_HEIGHT 10500
#define DELIVER_HEIGHT_SUCCESS 10420
#define DELIVER_HEIGHT_OVERSHOOT (DELIVER_HEIGHT + 50)
#define DELIVER_HEIGHT_OVERSHOOT_SUCCESS (DELIVER_HEIGHT_SUCCESS + 50)
#define DELIVER_HEIGHT_DROP 10250

#define DELIVER_LEFT_ROTATE_ORIENT 60
#define DELIVER_LEFT_ROTATE_SPEED 100
/*
#define DELIVER_LEFT_DRIVE_DIST -340
*/
#define DELIVER_LEFT_DRIVE_DIST -275
#define DELIVER_LEFT_DRIVE_SPEED -40
#define DELIVER_BLUE_LEFT_DRIVE_AWAY_DIST -650
#define DELIVER_BLUE_LEFT_DRIVE_AWAY_SPEED -127
#define DELIVER_RED_LEFT_TURN_TO_DRIVE_AWAY 40
/*
#define DELIVER_RED_LEFT_DRIVE_AWAY_DIST -1500
*/
#define DELIVER_RED_LEFT_DRIVE_AWAY_DIST -750
#define DELIVER_RED_LEFT_DRIVE_AWAY_SPEED -127

#define DELIVER_CENTER_ROTATE_ORIENT 0
#define DELIVER_CENTER_ROTATE_SPEED 0
#define DELIVER_CENTER_DRIVE_DIST 0
#define DELIVER_CENTER_DRIVE_SPEED 0
#define DELIVER_CENTER_DRIVE_AWAY_DIST 0
#define DELIVER_CENTER_DRIVE_AWAY_SPEED 0

#define DELIVER_RIGHT_ROTATE_ORIENT 0
#define DELIVER_RIGHT_ROTATE_SPEED 0
/*
#define DELIVER_RIGHT_DRIVE_DIST 250
*/
#define DELIVER_RIGHT_DRIVE_DIST 115
#define DELIVER_RIGHT_DRIVE_SPEED 40
#define DELIVER_RED_RIGHT_TURN_TO_AUTOLOADER 50

#define DELIVER_BLUE_RIGHT_DRIVE_AWAY_DIST 1
#define DELIVER_BLUE_RIGHT_DRIVE_AWAY_SPEED 127
/*
#define DELIVER_RED_RIGHT_DRIVE_AWAY_DIST 700
*/
#define DELIVER_RED_RIGHT_DRIVE_AWAY_DIST_NO_CROSS 0
//#define DELIVER_RED_RIGHT_DRIVE_AWAY_DIST_CROSS 1300
#define DELIVER_RED_RIGHT_DRIVE_AWAY_DIST_CROSS 700
#define DELIVER_RED_RIGHT_DRIVE_AWAY_SPEED 127

#define DELIVER_WAIT_BEFORE_DROP_CYCLES 80
#define DELIVER_DROP_WAIT_CYCLES 25
#define DELIVER_WAIT_TO_RESET_LIFT 30
#define DELIVER_WAIT_FOR_STABLE_TETRA 70

#define DELIVER_DRIVE_PAST_GOAL_DIST   780
#define DELIVER_DRIVE_PAST_GOAL_SPEED  127


extern UINT8 g_starting_pos;
extern PidValsType g_turn_orient_pid_vals;

UINT8 auto_task_deliver(void *params)
{
#if AUTO_USE_DELIVER
  static UINT8 local_state = DELIVER_INITIALIZE;
  UINT8 ret_state = TASK_STATE_INITIALIZE;
  UINT8 ret_val;
  EncoderValsType encoder_vals;
  static INT8 loop_count;
  UINT16 pos;

  static UINT8 rotate_orient_tgt;
  static INT16 drive_dist_tgt;
  static UINT8 drive_speed;
  static INT16 drive_away_dist_tgt;
  static UINT8 drive_away_speed;
  static UINT8 orig_orient;
  static UINT8 cross_to_autoloader;

  if(local_state == DELIVER_INITIALIZE)
  {
    loop_count = 0;
    /* Reset the CC values by requesting the current
     * values - the result isn't used here */
    ret_val = cc_get_encoder_vals(&encoder_vals);

    if(ret_val == CC_SUCCESS)
    {
#if PRINT_TASK_INFO
    printf("TASK STARTED: DELIVER\r");
#endif
      ret_state = TASK_STATE_PROCESSING;
      cross_to_autoloader = ((DeliverParamType *)params)->cross_to_autoloader;
      orig_orient = encoder_vals.orient;
      /* Set the parameters based on starting position */
      switch(g_starting_pos)
      {
        case STARTING_POS_BLUE_LEFT:
        case STARTING_POS_RED_LEFT:
          rotate_orient_tgt = orig_orient + DELIVER_LEFT_ROTATE_ORIENT;
          drive_dist_tgt = DELIVER_LEFT_DRIVE_DIST;
          drive_speed = DELIVER_LEFT_DRIVE_SPEED;
          if(g_starting_pos == STARTING_POS_BLUE_LEFT)
          {
            drive_away_dist_tgt = DELIVER_BLUE_LEFT_DRIVE_AWAY_DIST;
            drive_away_speed = DELIVER_BLUE_LEFT_DRIVE_AWAY_SPEED;
          }
          else
          {
            drive_away_dist_tgt = DELIVER_RED_LEFT_DRIVE_AWAY_DIST;
            drive_away_speed = DELIVER_RED_LEFT_DRIVE_AWAY_SPEED;
          }
          break;
        case STARTING_POS_BLUE_CENTER:
        case STARTING_POS_RED_CENTER:
          rotate_orient_tgt = orig_orient + DELIVER_CENTER_ROTATE_ORIENT;
          drive_dist_tgt = DELIVER_CENTER_DRIVE_DIST;
          drive_speed = DELIVER_CENTER_DRIVE_SPEED;
          drive_away_dist_tgt = DELIVER_CENTER_DRIVE_AWAY_DIST;
          drive_away_speed = DELIVER_CENTER_DRIVE_AWAY_SPEED;
          break;
        case STARTING_POS_BLUE_RIGHT:
        case STARTING_POS_RED_RIGHT:
          rotate_orient_tgt = orig_orient + DELIVER_RIGHT_ROTATE_ORIENT;
          drive_dist_tgt = DELIVER_RIGHT_DRIVE_DIST;
          drive_speed = DELIVER_RIGHT_DRIVE_SPEED;
          if(g_starting_pos == STARTING_POS_BLUE_RIGHT)
          {
            drive_away_dist_tgt = DELIVER_BLUE_RIGHT_DRIVE_AWAY_DIST;
            drive_away_speed = DELIVER_BLUE_RIGHT_DRIVE_AWAY_SPEED;
          }
          else
          {
            if(cross_to_autoloader == 1)
            {
              drive_away_dist_tgt = DELIVER_RED_RIGHT_DRIVE_AWAY_DIST_CROSS;
            }
            else
            {
              drive_away_dist_tgt = DELIVER_RED_RIGHT_DRIVE_AWAY_DIST_NO_CROSS;
            }
            drive_away_speed = DELIVER_RED_RIGHT_DRIVE_AWAY_SPEED;
          }
          break;
        default:
          break;
      }

      if(drive_dist_tgt == 0)
      {
        local_state = DELIVER_DONE;
      }
      else
      {
        /* Set local state to the next state to be run */
        local_state = DELIVER_POSITION_LIFT;
      }
    }

    /* set PID vals for turning */
    g_turn_orient_pid_vals.scale_factor = 3;
    g_turn_orient_pid_vals.prop_gain = 2 * g_turn_orient_pid_vals.scale_factor;
    g_turn_orient_pid_vals.int_gain = 1;
    g_turn_orient_pid_vals.deriv_gain = -2;
    g_turn_orient_pid_vals.max_integral = 127;
    g_turn_orient_pid_vals.max_val = 80;
    g_turn_orient_pid_vals.min_val = -80;
  }

  switch(local_state)
  {
    case DELIVER_POSITION_LIFT:
      ret_state = TASK_STATE_PROCESSING;
      /* Send the top spear out */
      motor_vals.top_spear_retract =  SPEAR_OUT;
      motor_vals.bot_spear_grabber =  SPEAR_GRAB;

      /* Untilt the lift */
      tilt_set_pos(TILT_POSITION_PAST_VERTICAL);
      /*  Get the current lift position */
      pos = get_lift_encoder_count();
#if PRINT_TASK_INFO
      printf("Lifting (%d)...\r",pos);
#endif
      /* Raise the lift -
       * Go a little higher so that we make sure the
       * spear goes out*/
      lift_set_height(DELIVER_HEIGHT_OVERSHOOT);
      if(pos >= (DELIVER_HEIGHT_OVERSHOOT_SUCCESS))
      {
#if PRINT_TASK_INFO
        printf("LIFT DONE\r");
#endif
        local_state = DELIVER_ROTATE_TO_DROP;
      }
      break;
    case DELIVER_ROTATE_TO_DROP:
      /* Send the top spear out */
      motor_vals.top_spear_retract =  SPEAR_OUT;

      /* Set the lift height to the correct height
       * this should be slightly lower than set
       * in the previous state so that the momentum
       * causes the top spear to come out */
#if USE_LIFT
      lift_set_height(DELIVER_HEIGHT);
#endif
      ret_state = TASK_STATE_PROCESSING;

      /* Turn to align with the goal */
      if (turn_to_orient(rotate_orient_tgt) == DRIVE_TASK_DONE)
      {
        /* Get the current encoder values
         * This is done to clear out the CC counters
         * The actual values are ignored this time */
        ret_val = cc_get_encoder_vals(&encoder_vals);
        g_autodrive_ticks = 0;

        if(ret_val == CC_SUCCESS)
        {
#if PRINT_TASK_INFO
          printf("ROTATE TO DROP DONE\r");
#endif
          /* set turn PID values for driving */
          local_state = DELIVER_DRIVE_TO_DROP;
          loop_count = 0;
        }
        else
        {
          motor_vals.left_drive = 0;
          motor_vals.right_drive = 0;
        }
      }
      else
      {
        ret_state = TASK_STATE_PROCESSING;
      }
      break;
    case DELIVER_DRIVE_TO_DROP:
      ret_state = TASK_STATE_PROCESSING;
      ret_val = drive_to_distance(drive_dist_tgt,drive_speed,SHIFT_SWITCH_LOW);

      if(ret_val == DRIVE_TASK_DONE)
      {
#if PRINT_TASK_INFO
          printf("DRIVE DONE\r");
#endif
          if((rotate_orient_tgt == orig_orient) ||
             (loop_count++ > DELIVER_WAIT_FOR_STABLE_TETRA))
          {
            local_state = DELIVER_POSITION_LIFT_FOR_DROP;
            loop_count = 0;
          }
          else
          {
#if PRINT_TASK_INFO
          printf("WAIT FOR STABLE TETRA <%d>\r",loop_count);
#endif
          }
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
    case DELIVER_POSITION_LIFT_FOR_DROP:
      /* By lowering the lift before dropping, we give ourselves
       * a better chance of being successful by opening up our
       * window for error */

      /* Get the lift height */
      pos = get_lift_encoder_count();
#if PRINT_TASK_INFO
      printf("Lowering (%d)...\r",pos);
#endif

      /* Set the lift height to be just above the goal */
      lift_set_height(DELIVER_HEIGHT_DROP);
      if(pos <= (DELIVER_HEIGHT_DROP))
      {
#if PRINT_TASK_INFO
        printf("LOWER DONE\r");
#endif
        local_state = DELIVER_WAIT_BEFORE_DROP;
      }
      break;
    case DELIVER_WAIT_BEFORE_DROP:
      /* Wait for a little bit so that the tetra stops rocking */
      if(loop_count++ >= DELIVER_WAIT_BEFORE_DROP_CYCLES)
      {
        loop_count = 0;
        local_state = DELIVER_DROP;
      }
      ret_state = TASK_STATE_PROCESSING;
      break;
    case DELIVER_DROP:
      /* Do the drop - Top spear - out, down, no grab */
#if PRINT_TASK_INFO
      printf("Dropping\r");
#endif
      motor_vals.top_spear_retract =  SPEAR_OUT;
      motor_vals.top_spear_tilt =  SPEAR_TILT_DOWN;
      motor_vals.top_spear_grabber =  SPEAR_NO_GRAB;
      /* Wait so that we make sure the tetra is dropped */
      if(loop_count++ >= DELIVER_DROP_WAIT_CYCLES)
      {
        local_state = DELIVER_COLLAPSE_LIFT;
      }
      ret_state = TASK_STATE_PROCESSING;
      break;
    case DELIVER_COLLAPSE_LIFT:
      /* Bring the lift/spears to the state that we want them in
       * when the drivers take over */
#if PRINT_TASK_INFO
      printf("Collapsing\r");
#endif
      /* Put the tilt at vertical */
      tilt_set_pos(TILT_POSITION_STRAIGHT_UP);
      lift_set_height(LIFT_HEIGHT_LIMIT_BOTTOM);
      motor_vals.top_spear_retract =  SPEAR_IN;
      motor_vals.top_spear_tilt =  SPEAR_NO_TILT;
      motor_vals.top_spear_grabber =  SPEAR_GRAB;

      g_autodrive_ticks = 0;
      local_state = DELIVER_DRIVE_PAST_GOAL;
      ret_state = TASK_STATE_PROCESSING;
      break;

    case DELIVER_DRIVE_PAST_GOAL:
      if(g_starting_pos == STARTING_POS_RED_RIGHT ||
         g_starting_pos == STARTING_POS_BLUE_RIGHT)
      {
        ret_val = drive_to_distance(DELIVER_DRIVE_PAST_GOAL_DIST,DELIVER_DRIVE_PAST_GOAL_SPEED,
                                    SHIFT_SWITCH_LOW);

        if(ret_val == DRIVE_TASK_DONE)
        {
#if PRINT_TASK_INFO
            printf("DRIVE PAST GOAL DONE\r");
#endif
            local_state = DELIVER_ROTATE_TO_DRIVE_AWAY;
        }
        else if(ret_val == DRIVE_TASK_CC_FAIL)
        {
          ret_state = TASK_STATE_PROCESSING;
          motor_vals.left_drive = 0;
          motor_vals.right_drive = 0;
        }
        else
        {
#if PRINT_TASK_INFO
          printf("DRIVE PAST GOAL %d %d\r",(int)g_autodrive_ticks,(int)DELIVER_DRIVE_PAST_GOAL_DIST);
#endif
        }
      }
      else
      {
          local_state = DELIVER_DRIVE_AWAY;
          ret_state = TASK_STATE_PROCESSING;
      }
      break;
    case DELIVER_ROTATE_TO_DRIVE_AWAY:
      if(g_starting_pos == STARTING_POS_RED_LEFT)
      {
#if PRINT_TASK_INFO
      printf("Final rotate\r");
#endif
        if (turn_to_orient(orig_orient + DELIVER_RED_LEFT_TURN_TO_DRIVE_AWAY) == DRIVE_TASK_DONE)
        {
          local_state = DELIVER_DRIVE_AWAY;
          g_autodrive_ticks = 0;
          loop_count = 0;
        }
      }
      else if(g_starting_pos == STARTING_POS_RED_RIGHT)
      {
        if (turn_to_orient(orig_orient + DELIVER_RED_RIGHT_TURN_TO_AUTOLOADER) == DRIVE_TASK_DONE)
        {
          if(cross_to_autoloader)
          {
            local_state = DELIVER_DRIVE_AWAY;
          }
          else
          {
            local_state = DELIVER_DONE;
          }
          g_autodrive_ticks = 0;
          loop_count = 0;
        }
      }
      else
      {
        local_state = DELIVER_DRIVE_AWAY;
        g_autodrive_ticks = 0;
        loop_count = 0;
      }
      ret_state = TASK_STATE_PROCESSING;
      break;
    case DELIVER_DRIVE_AWAY:
      ret_val = drive_to_distance(drive_away_dist_tgt,drive_away_speed,
                                  SHIFT_SWITCH_HIGH);

      if(ret_val == DRIVE_TASK_DONE)
      {
#if PRINT_TASK_INFO
          printf("DRIVE AWAY DONE\r");
#endif
          local_state = DELIVER_DONE;
      }
      else if(ret_val == DRIVE_TASK_CC_FAIL)
      {
        ret_state = TASK_STATE_PROCESSING;
        motor_vals.left_drive = 0;
        motor_vals.right_drive = 0;
      }
      else
      {
#if PRINT_TASK_INFO
        printf("DRIVE AWAY %d %d\r",(int)g_autodrive_ticks,(int)drive_away_dist_tgt);
#endif
      }
      break;
    case DELIVER_DONE:
      pos = get_lift_encoder_count();
      if(pos <= LIFT_HEIGHT_TOP_AUTOLOAD)
      {
        local_state = DELIVER_INITIALIZE;
        ret_state = TASK_STATE_DONE;
      }
      break;
    default:
      break;
  }
  return ret_state;
#else
#if PRINT_TASK_INFO
    printf("TASK SKIPPED: Deliver Tetra\r");
#endif
  return TASK_STATE_DONE;
#endif
}

