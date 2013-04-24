#include <stdio.h>
#include "ifi_aliases.h"
#include "ifi_default.h"
#include "ifi_utilities.h"
#include "ws_includes.h"
#include "user_routines.h"
#include "delays.h"       /*defined locally*/
#include "ws_io.h"
#include "ws_pid.h"
#include "ws_calibrate.h"
#include "ws_autonomous.h"
#include "ws_autonomous_tasks.h"
#include "ws_includes.h"
#include "ws_lift.h"
#include "ws_auto_drive.h"
#include "ws_drive_input.h"
#include "ws_cc.h"
#include "ws_encoder.h"

extern PidValsType g_turn_drive_pid_vals;
extern PidValsType g_drive_speed_pid_vals;
extern UINT8 g_chosen_tetra;
extern UINT8 g_starting_pos;
extern UINT8 g_goal_acked;
UINT8 g_goal_id;

const rom ScoreParamsType g_score_params[6] =
    { {BLUE_CENTER_L1_SCORE_APPROACH_X, BLUE_CENTER_L1_SCORE_APPROACH_Y,
       BLUE_CENTER_L1_SCORE_APPROACH_RANGE, BLUE_CENTER_L1_SCORE_ORIENT,
       BLUE_CENTER_L1_SCORE_DISTANCE, BLUE_CENTER_L1_SCORE_RANGE},
      {BLUE_CENTER_L2_SCORE_APPROACH_X, BLUE_CENTER_L2_SCORE_APPROACH_Y,
       BLUE_CENTER_L2_SCORE_APPROACH_RANGE, BLUE_CENTER_L2_SCORE_ORIENT,
       BLUE_CENTER_L2_SCORE_DISTANCE, BLUE_CENTER_L2_SCORE_RANGE},
      {RED_CENTER_F1_SCORE_APPROACH_X, RED_CENTER_F1_SCORE_APPROACH_Y,
       RED_CENTER_F1_SCORE_APPROACH_RANGE, RED_CENTER_F1_SCORE_ORIENT,
       RED_CENTER_F1_SCORE_DISTANCE, RED_CENTER_F1_SCORE_RANGE},
      {RED_CENTER_F2_SCORE_APPROACH_X, RED_CENTER_F2_SCORE_APPROACH_Y,
       RED_CENTER_F2_SCORE_APPROACH_RANGE, RED_CENTER_F2_SCORE_ORIENT,
       RED_CENTER_F2_SCORE_DISTANCE, RED_CENTER_F2_SCORE_RANGE},
      {RED_CENTER_B_SCORE_APPROACH_X, RED_CENTER_B_SCORE_APPROACH_Y,
       RED_CENTER_B_SCORE_APPROACH_RANGE, RED_CENTER_B_SCORE_ORIENT,
       RED_CENTER_B_SCORE_DISTANCE, RED_CENTER_B_SCORE_RANGE},
      {RIGHT_SCORE_APPROACH_X, RIGHT_SCORE_APPROACH_Y,
       RIGHT_SCORE_APPROACH_RANGE, RIGHT_SCORE_ORIENT,
       RIGHT_SCORE_DISTANCE, RIGHT_SCORE_RANGE} };

#define GOAL_SCORE_HEADING  198
#define GOAL_SCORE_SPEED    180


UINT8 auto_task_score(void *params)
{
#if AUTO_USE_SCORE
  static UINT8 local_state = SCORE_INITIALIZE;
  static UINT16 goal_height;
  static UINT8 wait_count;
  static UINT8 score_idx;

  UINT8 ret_state;

  if (local_state == SCORE_INITIALIZE)
  {
    /* Do anything that needs to be done the
       first time here */
    if(g_chosen_tetra == 0)
    {
      if(((ScoreParamType *)params)->skip_on_fail)
      {
        local_state = SCORE_SKIP;
      }
      else
      {
        local_state = SCORE_DONE;
      }
    }
    else if (g_goal_id == GOAL_MIDDLE_CENTER)
    {
      /* score on center goal */
      if ((g_starting_pos == STARTING_POS_RED_LEFT) ||
          (g_starting_pos == STARTING_POS_RED_CENTER) ||
          (g_starting_pos == STARTING_POS_RED_RIGHT))
      {
        if (g_chosen_tetra == 3)
        {
          score_idx = SCORE_CENTER_RED_BACK;
        }
        else if (g_chosen_tetra == 7)
        {
          score_idx = SCORE_CENTER_RED_FRONT_2;
        }
        else
        {
          score_idx = SCORE_CENTER_RED_FRONT_1;
        }
      }
      else
      {
        if ((g_chosen_tetra == 6) || (g_chosen_tetra == 7))
        {
          score_idx = SCORE_CENTER_BLUE_LEFT2;
        }
        else
        {
          score_idx = SCORE_CENTER_BLUE_LEFT1;
        }
      }

      goal_height = LIFT_HEIGHT_SCORE_CENTER;
      local_state = SCORE_CC_COMM;
    }
    else if (g_goal_id == GOAL_MIDDLE_RIGHT)
    {
      /* score on right goal */
      score_idx = SCORE_RIGHT;
      local_state = SCORE_CC_COMM;
      goal_height = LIFT_HEIGHT_SCORE_OUTSIDE;
    }
    else
    {
      local_state = SCORE_DONE;
    }

    /*
    printf("GOAL %d\r", score_idx);
    printf("ax %d ay %d ar %d do %d dd %d dr %d\r",
           g_score_params[score_idx].approach_x,
           g_score_params[score_idx].approach_y,
           g_score_params[score_idx].approach_range,
           g_score_params[score_idx].drop_orient,
           g_score_params[score_idx].drop_dist,
           g_score_params[score_idx].drop_range);
    */

    SET_SPEED_PID_SCORE();

    ret_state = TASK_STATE_PROCESSING;
#if PRINT_TASK_INFO
    printf("TASK STARTED: SCORE - goal (%d)\r", (int)g_goal_id);
#endif
  }

  switch(local_state)
  {
    case SCORE_CC_COMM:
      lift_set_height(LIFT_HEIGHT_DRIVE_TO_APPROACH);

      /* only advance to next state if CC ack's the object */
      if ((g_goal_acked == TRUE) ||
          (cc_set_track_object(g_goal_id) == CC_SUCCESS))
      {
        local_state = SCORE_DRIVE_TO_APPROACH;
      }

      ret_state = TASK_STATE_PROCESSING;

      break;

    case SCORE_DRIVE_TO_APPROACH:
      motor_vals.bot_spear_grabber = SPEAR_GRAB;

      if (drive_to_approach(g_score_params[score_idx].approach_x,
                            g_score_params[score_idx].approach_y,
                            g_score_params[score_idx].approach_range,
                            SHIFT_SWITCH_HIGH, CC_CMD_REQ_DIST_HDG_XY) ==
          DRIVE_TASK_DONE)
      {
        local_state = SCORE_TURN_TO_DROPOFF;
      }
      ret_state = TASK_STATE_PROCESSING;
      break;

    case SCORE_TURN_TO_DROPOFF:
      lift_set_height(LIFT_HEIGHT_TURN_TO_GOAL);

      if (turn_to_orient(g_score_params[score_idx].drop_orient) ==
          DRIVE_TASK_DONE)
      {
        local_state = SCORE_DRIVE_TO_DROPOFF;
        SET_TURN_PID_SCORE();
      }
      ret_state = TASK_STATE_PROCESSING;
      break;

    case SCORE_DRIVE_TO_DROPOFF:
      lift_set_height(LIFT_HEIGHT_DRIVE_NEXT_TO_GOAL);

      if (drive_next_to_object(g_score_params[score_idx].drop_dist,
                               g_score_params[score_idx].drop_range) ==
          DRIVE_TASK_DONE)
      {
        local_state = SCORE_DRIVE_THROUGH_DROPOFF;
      }
      ret_state = TASK_STATE_PROCESSING;
      break;

    case SCORE_DRIVE_THROUGH_DROPOFF:

      /*
      lift_set_height(LIFT_HEIGHT_DRIVE_NEXT_TO_GOAL);
      */
      lift_set_height(goal_height);

      if (drive_past_object(g_score_params[score_idx].drop_orient,
                            GOAL_SCORE_SPEED, GOAL_SCORE_HEADING) ==
          DRIVE_TASK_DONE)
      {
        local_state = SCORE_SET_SPEAR_BEFORE_DROP;
      }

      ret_state = TASK_STATE_PROCESSING;

      break;

    case SCORE_SET_SPEAR_BEFORE_DROP:

      /* Need to decide which spear to work with in all states!!! */
      /* Bring bottom spear in */
      motor_vals.bot_spear_grabber = SPEAR_GRAB;
      motor_vals.bot_spear_tilt = SPEAR_NO_TILT;
      motor_vals.bot_spear_retract = SPEAR_OUT;

      /* set drive speed */
      /* Don't be moving for now... */
      Oi_drive_x = 127;
      Oi_drive_y = 127;
      drive_stick_input(FALSE);

#if 1
      /* goal height is purposfully 300 higher than the height needed to
         drop in order to give us enough P to keep going fast enough */
      if (get_lift_encoder_count() >= (goal_height - 300))
      {
        local_state = SCORE_DO_DROP;
      }
#else
      local_state = SCORE_DO_DROP;
#endif
      ret_state = TASK_STATE_PROCESSING;
      break;

    case SCORE_DO_DROP:
      /* Need to decide which spear to work with in all states!!! */
      /* Set the bottom spear to drop */
      motor_vals.bot_spear_grabber = SPEAR_NO_GRAB;
      motor_vals.bot_spear_tilt = SPEAR_TILT_DOWN;
      motor_vals.bot_spear_retract = SPEAR_OUT;

      Oi_drive_x = 127;
      Oi_drive_y = 127;
      drive_stick_input(FALSE);

      wait_count = 0;
      local_state = SCORE_WAIT_AFTER_DROP;
      ret_state = TASK_STATE_PROCESSING;
      break;

    case SCORE_WAIT_AFTER_DROP:
      if(wait_count++ >= 40)
      {
        local_state = SCORE_SET_SPEAR_AFTER_DROP;
      }

      Oi_drive_x = 127;
      Oi_drive_y = 127;
      drive_stick_input(FALSE);

      ret_state = TASK_STATE_PROCESSING;
      break;

    case SCORE_SET_SPEAR_AFTER_DROP:
      /* Bring bottom spear in */
      motor_vals.bot_spear_grabber = SPEAR_NO_GRAB;
      motor_vals.bot_spear_tilt = SPEAR_NO_TILT;
      motor_vals.bot_spear_retract = SPEAR_IN;

      /* Open top spear */
      motor_vals.top_spear_grabber = SPEAR_GRAB;
      motor_vals.top_spear_tilt = SPEAR_NO_TILT;
      motor_vals.top_spear_retract = SPEAR_IN;

      local_state = SCORE_LOWER_LIFT;
      ret_state = TASK_STATE_PROCESSING;
      break;

    case SCORE_LOWER_LIFT:
      /* Set lift height to spear tetra on autoloader with top spear  */
      lift_set_height(LIFT_HEIGHT_SCORE_DONE);
      if(motor_vals.lift_height <= LIFT_HEIGHT_STOW)
      {
        local_state = SCORE_DONE;
      }
      ret_state = TASK_STATE_PROCESSING;
      break;

    case SCORE_SKIP:
#if PRINT_TASK_INFO
      printf("SCORE TASK DONE - SKIP NEXT\r");
#endif
      local_state = SCORE_INITIALIZE;
      ret_state = TASK_STATE_SKIP_NEXT;
      break;
    case SCORE_DONE:
#if PRINT_TASK_INFO
      printf("SCORE TASK DONE\r");
#endif
      ret_state = TASK_STATE_DONE;
      local_state = SCORE_INITIALIZE;
      break;
  }
  return ret_state;
#else
#if PRINT_TASK_INFO
    printf("TASK SKIPPED: Score\r");
#endif
  return TASK_STATE_DONE;
#endif
}
