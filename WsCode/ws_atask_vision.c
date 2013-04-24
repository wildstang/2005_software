#include <stdio.h>
#include "ifi_aliases.h"
#include "ifi_default.h"
#include "ifi_utilities.h"
#include "ws_includes.h"
#include "user_routines.h"
#include "delays.h"       /*defined locally*/
#include "ws_io.h"

#include "ws_autonomous.h"
#include "ws_autonomous_tasks.h"
#include "ws_pid.h"
#include "ws_cc.h"
#include "ws_drive_input.h"
#include "ws_lift.h"
#include "ws_trig.h"
#include "ws_auto_drive.h"

#define APPROACH_X_THRESH 5
#define APPROACH_Y_THRESH 5

UINT8 g_goal_acked = FALSE;
extern UINT8 g_goal_id;
#if USE_NEW_PICKUP_METHOD
extern UINT8 g_starting_pos;
#endif
extern UINT8 g_chosen_tetra;
extern PidValsType g_turn_drive_pid_vals;
extern PidValsType g_drive_speed_pid_vals;

#define PICKUP_SPEED      180
//#define PICKUP_SPEED      230

/*******************************************************************************
* FUNCTION NAME: auto_task_vision
* PURPOSE:       State machine to pick up vision tetra
* CALLED FROM:
* ARGUMENTS:
* RETURNS:
*******************************************************************************/
UINT8 auto_task_vision(void *params)
{
#if AUTO_USE_VISION
  static UINT8 local_state = VISION_INITIALIZE;
  static UINT8 drive_counter = 0;
  UINT8        ret_state;
  UINT8        cc_ret_val;
  DistHdgType  curr_dist_hdg;
  static UINT8 orig_orient;
  EncoderValsType encoder_vals;


  if(local_state == VISION_INITIALIZE)
  {
    /* Do anything that needs to be done the
       first time here */

       SET_SPEED_PID_VISION();

#if PRINT_TASK_INFO
    printf("TASK STARTED: VISION\r");
#endif

    ret_state = TASK_STATE_PROCESSING;

#if 0
    // Use starting position to select tetra
    switch(g_starting_pos)
    {
      case STARTING_POS_RED_LEFT:
        g_chosen_tetra = 7;
        break;
      case STARTING_POS_RED_CENTER:
        g_chosen_tetra = 8;
        break;
      case STARTING_POS_RED_RIGHT:
        g_chosen_tetra = 3;
        break;
      case STARTING_POS_BLUE_LEFT:
        g_chosen_tetra = 4;
        break;
      case STARTING_POS_BLUE_CENTER:
        g_chosen_tetra = 5;
        break;
      case STARTING_POS_BLUE_RIGHT:
        g_chosen_tetra = 6;
        break;
      default:
        g_chosen_tetra = 0;
        break;
    }
#endif

    if (g_chosen_tetra != 0)
    {
      local_state = VISION_CC_COMM;
    }
    else
    {
      local_state = VISION_SKIP;
    }
  }

  switch (local_state)
  {
    case VISION_CC_COMM:

      /* tilt lift vertical */
      tilt_set_pos(TILT_POSITION_STRAIGHT_UP);
#if 1 // Use this to test autonomous tilting...
      lift_set_height(LIFT_HEIGHT_BOT_FLOOR_PICKUP);

      motor_vals.fwing = WING_OUT;

      /* only advance to next state if CC ack's the object */
      if (cc_set_track_object(g_chosen_tetra) == CC_SUCCESS)
      {
        local_state = VISION_DRIVE_TO_TETRA;
      }

#if 0
      if (drive_counter <= 6)
      {
        Oi_drive_y = 254;
        Oi_drive_x = 127;
        drive_counter++;
        printf("drive fwd\r");
      }
      else
      {
        Oi_drive_y = 127;
        Oi_drive_x = 127;
        printf("no drive\r");
      }

      Oi_sw_turbo = 1;
      drive_stick_input(FALSE);
#endif
#endif
      ret_state = TASK_STATE_PROCESSING;
      break;

#if 0
    case VISION_DRIVE_FORWARD:

      local_state = VISION_TURN_TO_APPROACH;
      ret_state = TASK_STATE_PROCESSING;
      break;

    case VISION_TURN_TO_APPROACH:
      /*
      if (turn_to_orient(TETRA_PICKUP_ORIENT) == DRIVE_TASK_DONE)
      {
      */
        local_state = VISION_DRIVE_TO_APPROACH_HI;
      /*
      }
      */
      ret_state = TASK_STATE_PROCESSING;
      break;
#endif

    case VISION_DRIVE_TO_TETRA:

      motor_vals.top_spear_retract = SPEAR_OUT;

#if 0
      if (drive_counter <= 6)
      {
        Oi_drive_y = 254;
        Oi_drive_x = 127;
        drive_counter++;
        printf("drive fwd\r");
        Oi_sw_turbo = 1;
        drive_stick_input(FALSE);
      }
      else
      {
#endif
        if (drive_to_approach(TETRA_APPROACH_X, TETRA_APPROACH_Y,
                              TETRA_APPROACH_RANGE * 3, SHIFT_SWITCH_HIGH,
                              CC_CMD_REQ_DIST_HDG_XY) == DRIVE_TASK_DONE)
        {
          local_state = VISION_DRIVE_TO_APPROACH_LO;
          drive_counter = 0;
        }
#if 0
      }
#endif

      ret_state = TASK_STATE_PROCESSING;
      break;

    case VISION_DRIVE_TO_APPROACH_LO:

#if 0
      /* disable tilt feedback */
      tilt_set_pos(0);

      if (drive_to_approach(TETRA_APPROACH_X, TETRA_APPROACH_Y,
                            TETRA_APPROACH_RANGE, SHIFT_SWITCH_LOW,
                            CC_CMD_REQ_DIST_HDG_XY) == DRIVE_TASK_DONE)
      {
        local_state = VISION_REORIENT;
        drive_counter = 0;
      }
#else
      drive_counter++;
      if(drive_counter < 20)
      {
        //Coast in
        Oi_drive_x = 127;
        Oi_drive_y = 127;
        drive_stick_input(FALSE);
      }
      else if(drive_counter >= 40 && drive_counter < 45)
      {
        // Back up a bit
        Oi_drive_y = 60;
        Oi_drive_x = 127;
        drive_stick_input(FALSE);
      }
      else
      {
        // Read encoders from CC to get the orientation
        if(cc_get_encoder_vals(&encoder_vals) == CC_SUCCESS)
        {
          orig_orient = encoder_vals.orient;
          drive_counter = 0;
          local_state = VISION_REORIENT;
        }
      }
#endif
      ret_state = TASK_STATE_PROCESSING;
      break;

    case VISION_REORIENT:

      motor_vals.fwing = WING_IN;
      motor_vals.top_spear_retract = SPEAR_OUT;
      motor_vals.bot_spear_grabber =  SPEAR_NO_GRAB;
      motor_vals.bot_spear_tilt = SPEAR_NO_TILT;
      motor_vals.bot_spear_retract = SPEAR_OUT;

      if (turn_to_orient(orig_orient + TETRA_PICKUP_ORIENT) == DRIVE_TASK_DONE)
      {
        /* set turn PID values for driving */
        SET_SPEED_PID_VISION_LOW_GEAR();
        local_state = VISION_DRIVE_THROUGH_TETRA;
      }
      ret_state = TASK_STATE_PROCESSING;
      break;

    case VISION_DRIVE_THROUGH_TETRA:

      motor_vals.bwing = WING_OUT;
#if 1
      if (g_goal_id != 0)
      {
        if ((g_goal_acked == FALSE) &&
            (cc_set_track_object(g_goal_id) == CC_SUCCESS))
        {
          g_goal_acked = TRUE;
        }
      }
#endif
//      lift_set_height(LIFT_HEIGHT_BOT_FLOOR_PICKUP2);

      Oi_drive_x = 127;
      Oi_drive_y = PICKUP_SPEED;
      Oi_sw_shifter = SHIFT_SWITCH_LOW;
      drive_stick_input(FALSE);

      drive_counter++;

      if (drive_counter > 30)
      {
        local_state = VISION_DRIVE_THROUGH_TETRA2;
        drive_counter = 0;
      }

      ret_state = TASK_STATE_PROCESSING;
      break;

    case VISION_DRIVE_THROUGH_TETRA2:

      motor_vals.fwing = WING_OUT;
#if 1
      if (g_goal_id != 0)
      {
        if ((g_goal_acked == FALSE) &&
            (cc_set_track_object(g_goal_id) == CC_SUCCESS))
        {
          g_goal_acked = TRUE;
        }
      }
#endif
      lift_set_height(LIFT_HEIGHT_BOT_FLOOR_PICKUP3);

      Oi_drive_x = 127;
      Oi_drive_y = PICKUP_SPEED;
      Oi_sw_shifter = SHIFT_SWITCH_LOW;
      drive_stick_input(FALSE);

      drive_counter++;

      if (drive_counter > 6)
      {
        local_state = VISION_PICKUP_TETRA;
        drive_counter = 0;
      }

      ret_state = TASK_STATE_PROCESSING;
      break;

    case VISION_PICKUP_TETRA:

      motor_vals.bot_spear_grabber = SPEAR_GRAB;
#if 0
      motor_vals.bot_spear_tilt = SPEAR_NO_TILT;
      lift_set_height(LIFT_HEIGHT_BOT_FLOOR_PICKUP + 30);
#else
      lift_set_height(LIFT_HEIGHT_BOT_FLOOR_PICKUP + 150);
#endif

      Oi_drive_y = 110;
      Oi_drive_x = 127;
      Oi_sw_shifter = SHIFT_SWITCH_LOW;
      drive_stick_input(FALSE);

      if (drive_counter > 5)
      {
        local_state = VISION_DONE;
      }

      drive_counter++;

      ret_state = TASK_STATE_PROCESSING;
      break;

    case VISION_SKIP:
      /* tilt lift vertical */
      tilt_set_pos(TILT_POSITION_STRAIGHT_UP);
      local_state = VISION_INITIALIZE;
      ret_state = TASK_STATE_DONE;
#if PRINT_TASK_INFO
      printf("VISION TASK - No tetra, move on\r");
#endif
      break;

    case VISION_DONE:
      local_state = VISION_INITIALIZE;
      ret_state = TASK_STATE_DONE;
#if PRINT_TASK_INFO
      printf("VISION TASK DONE\r");
#endif
      break;

    default:
      break;
  }

  return ret_state;
#else
#if PRINT_TASK_INFO
    printf("TASK SKIPPED: Vision\r");
#endif
  return TASK_STATE_DONE;
#endif
}

