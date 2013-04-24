/*******************************************************************************
* FILE NAME: ws_autonomous.c
*
* DESCRIPTION:
*
* USAGE:
*
*******************************************************************************/

#include <stdio.h>
#include "ifi_aliases.h"
#include "ifi_default.h"
#include "ifi_utilities.h"
#include "ws_includes.h"
#include "user_routines.h"
#include "delays.h"       /*defined locally*/
#include "ws_io.h"
#include "ws_cc.h"
#include "ws_general.h"
#include "ws_pid.h"
#include "ws_lift.h"
#include "ws_encoder.h"

#include "ws_autonomous.h"
#include "ws_autonomous_tasks.h"

UINT8 g_chosen_tetra = 0;
UINT8 g_starting_pos = STARTING_POS_UNINIT;
PidValsType g_turn_drive_pid_vals;
PidValsType g_turn_orient_pid_vals;
PidValsType g_drive_speed_pid_vals;
INT16 g_autodrive_ticks;
extern UINT8 g_goal_id;
extern UINT16 g_lift_encoder_val_prev[];
extern UINT8 g_lift_encoder_val_prev_idx;

static UINT8 s_prog_num = 0;
static AutoProgramType *program = AUTO_DO_NOTHING;
static UINT8 s_auto_locked_in = FALSE;

/*
 * Starting tetra postions;
 *
 *    2       5       8
 *
 *        3       6
 *
 *    1       4       7
 *
 *
 *          Center
 *    Left         Right
 */

/*              starting tetra positions   0   1   2   3   4   5   6   7   8 */
const rom UINT8 pos_left_priority[9] =   {20, 18, 19,  1,  2, 15,  3, 16, 17};
const rom UINT8 pos_center_r_priority[9] = {20, 18, 19, 15,  1, 16,  2, 14, 17};
const rom UINT8 pos_center_b_priority[9] = {20, 18, 19,  3,  1, 16,  2, 15, 17};
const rom UINT8 pos_right_priority[9] =  {20, 18, 19, 17, 14, 16,  1,  2, 15};


/*******************************************************************************
* FUNCTION NAME: autonomous_init
* PURPOSE:       Initialize the autonomous system...called only once
* CALLED FROM:
* ARGUMENTS:
* RETURNS:
*******************************************************************************/
void autonomous_init(void)
{
  return;
}


/*******************************************************************************
* FUNCTION NAME: auto_lock_in
* PURPOSE:       Lock in autonomous program & pass info to CC
* CALLED FROM:
* ARGUMENTS:
* RETURNS:
*******************************************************************************/
void auto_lock_in(void)
{
  static UINT8 pos_init_state = STARTING_POS_NOT_SET;
  static TetraPosType tetra_pos;

  if (Oi_sw_auto_lockin == 1)
  {
    if(program->use_vision == USE_VISION)
    {
      /* tell CC starting position */
      if ((pos_init_state == STARTING_POS_NOT_SET) &&
          (cc_set_position(g_starting_pos) == CC_SUCCESS))
      {
        pos_init_state = STARTING_POS_SET;
      }

      if (pos_init_state == STARTING_POS_SET)
      {
        if ((rxdata.packet_num % 10) == 0)
        {
          /* request starting tetra positions */
          cc_get_tetra_pos(&tetra_pos);

          /* choose tetra to track */
          tetra_chooser(tetra_pos.tetra1, tetra_pos.tetra2);

          printf("tetras %d %d chosen %d\r", tetra_pos.tetra1, tetra_pos.tetra2,
                 g_chosen_tetra);
        }
      }
    }
    else
    {
      //cc_set_camera_pos(CC_CMD_SET_CAMERA_STORE);
    }
    s_auto_locked_in = TRUE;
  }
  else
  {
    /* lock in program selector & robot position selectors */
    auto_chooser();
    tetra_pos.tetra1 = 0;
    tetra_pos.tetra2 = 0;
    s_auto_locked_in = FALSE;
    pos_init_state = STARTING_POS_NOT_SET;
  }

  return;
}


/*******************************************************************************
* FUNCTION NAME: auto_chooser
* PURPOSE:       set autonomous program, side & starting position;
*                lock in forced auto
* ARGUMENTS:     none
* RETURNS:       none
*******************************************************************************/
void auto_chooser(void)
{
  /*
  printf("auto_prog %d, auto_pos %d ", (int)Oi_auto_prog_select,
         (int)Oi_auto_pos_select);
  */


  /* read autonomous program dial */
  if ((Oi_auto_prog_select >= (WAYPOINT_OI_SEL1 - WAYPOINT_OI_DIFF)) &&
      (Oi_auto_prog_select <= (WAYPOINT_OI_SEL1 + WAYPOINT_OI_DIFF)))
  {
    s_prog_num = 1;
    program = &program1;
  }
  else if ((Oi_auto_prog_select >= (WAYPOINT_OI_SEL2 - WAYPOINT_OI_DIFF)) &&
           (Oi_auto_prog_select <= (WAYPOINT_OI_SEL2 + WAYPOINT_OI_DIFF)))
  {
    s_prog_num = 2;
    program = &program2;
  }
  else if ((Oi_auto_prog_select >= (WAYPOINT_OI_SEL3 - WAYPOINT_OI_DIFF)) &&
           (Oi_auto_prog_select <= (WAYPOINT_OI_SEL3 + WAYPOINT_OI_DIFF)))
  {
    s_prog_num = 3;
    program = &program3;
  }
  else if ((Oi_auto_prog_select >= (WAYPOINT_OI_SEL4 - WAYPOINT_OI_DIFF)) &&
           (Oi_auto_prog_select <= (WAYPOINT_OI_SEL4 + WAYPOINT_OI_DIFF)))
  {
    s_prog_num = 4;
    program = &program4;
  }
  else if ((Oi_auto_prog_select >= (WAYPOINT_OI_SEL5 - WAYPOINT_OI_DIFF)) &&
           (Oi_auto_prog_select <= (WAYPOINT_OI_SEL5 + WAYPOINT_OI_DIFF)))
  {
    s_prog_num = 5;
    program = &program5;
  }
  else if ((Oi_auto_prog_select >= (WAYPOINT_OI_SEL6 - WAYPOINT_OI_DIFF)) &&
           (Oi_auto_prog_select <= (WAYPOINT_OI_SEL6 + WAYPOINT_OI_DIFF)))
  {
    s_prog_num = 6;
    program = &program6;
  }
  else
  {
    /* default to dead auto program */
    s_prog_num = 0;
    program = AUTO_DO_NOTHING;
  }

  /* read starting position dial */
  if ((Oi_auto_pos_select >= (OI_SEL_START_RED_L - OI_SEL_START_DIFF)) &&
      (Oi_auto_pos_select <= (OI_SEL_START_RED_L + OI_SEL_START_DIFF)))
  {
    g_starting_pos = STARTING_POS_RED_LEFT;
  }
  else if ((Oi_auto_pos_select >= (OI_SEL_START_RED_C - OI_SEL_START_DIFF)) &&
           (Oi_auto_pos_select <= (OI_SEL_START_RED_C + OI_SEL_START_DIFF)))
  {
    g_starting_pos = STARTING_POS_RED_CENTER;
  }
  else if ((Oi_auto_pos_select >= (OI_SEL_START_RED_R - OI_SEL_START_DIFF)) &&
           (Oi_auto_pos_select <= (OI_SEL_START_RED_R + OI_SEL_START_DIFF)))
  {
    g_starting_pos = STARTING_POS_RED_RIGHT;
  }
  else if ((Oi_auto_pos_select >= (OI_SEL_START_BLUE_L - OI_SEL_START_DIFF)) &&
           (Oi_auto_pos_select <= (OI_SEL_START_BLUE_L + OI_SEL_START_DIFF)))
  {
    g_starting_pos = STARTING_POS_BLUE_LEFT;
  }
  else if ((Oi_auto_pos_select >= (OI_SEL_START_BLUE_C - OI_SEL_START_DIFF)) &&
           (Oi_auto_pos_select <= (OI_SEL_START_BLUE_C + OI_SEL_START_DIFF)))
  {
    g_starting_pos = STARTING_POS_BLUE_CENTER;
  }
  else if ((Oi_auto_pos_select >= (OI_SEL_START_BLUE_R - OI_SEL_START_DIFF)) &&
           (Oi_auto_pos_select <= (OI_SEL_START_BLUE_R + OI_SEL_START_DIFF)))
  {
    g_starting_pos = STARTING_POS_BLUE_RIGHT;
  }
  else
  {
    /* default to uninitialized */
    g_starting_pos = STARTING_POS_UNINIT;
  }

  /*
  printf("prog %d, pos %d\r", (int)s_prog_num, (int)g_starting_pos);
  */

  return;
}

/*******************************************************************************
* FUNCTION NAME: auto_main
* PURPOSE:       main loop for autonomous
* ARGUMENTS:     none
* RETURNS:       none
*******************************************************************************/
void auto_main()
{
  static UINT8 task_state = TASK_STATE_DONE;
  static INT8 task_id = AUTO_PROGRAM_START;  // First time through gets incremented to 0

#if 0
  /* Initialize all PWMs and Relays when entering Autonomous mode, or else it
     will be stuck with the last values mapped from the joysticks. */
  pwm01 = pwm02 = pwm03 = pwm04 = pwm05 = pwm06 = pwm07 = pwm08 = 127;
  pwm09 = pwm10 = pwm11 = pwm12 = pwm13 = pwm14 = pwm15 = pwm16 = 127;
  relay1_fwd = relay1_rev = relay2_fwd = relay2_rev = 0;
  relay3_fwd = relay3_rev = relay4_fwd = relay4_rev = 0;
  relay5_fwd = relay5_rev = relay6_fwd = relay6_rev = 0;
  relay7_fwd = relay7_rev = relay8_fwd = relay8_rev = 0;
#endif

  /* initialize autonomous PID values */
  g_turn_orient_pid_vals.last_error = 0;
  g_turn_orient_pid_vals.last_last_error = 0;
  g_turn_drive_pid_vals.last_error = 0;
  g_turn_drive_pid_vals.last_last_error = 0;
  g_drive_speed_pid_vals.last_error = 0;
  g_drive_speed_pid_vals.last_last_error = 0;

#ifdef PROTO_ROBOT
  /* turning PID vals while reorienting */
  g_turn_orient_pid_vals.scale_factor = 2;
  g_turn_orient_pid_vals.prop_gain = 4 * g_turn_orient_pid_vals.scale_factor;
  g_turn_orient_pid_vals.int_gain = 1;
  g_turn_orient_pid_vals.deriv_gain = -3;
  g_turn_orient_pid_vals.max_integral = 127;
  g_turn_orient_pid_vals.max_val = 90;
  g_turn_orient_pid_vals.min_val = -70;

  /* turning PID vals while driving */
  g_turn_drive_pid_vals.scale_factor = 1;
  g_turn_drive_pid_vals.prop_gain = 7 * g_turn_drive_pid_vals.scale_factor;
  g_turn_drive_pid_vals.int_gain = 1;
  g_turn_drive_pid_vals.deriv_gain = 0;
  g_turn_drive_pid_vals.max_integral = 40;
  g_turn_drive_pid_vals.max_val = 60;
  g_turn_drive_pid_vals.min_val = -60;
#else
  /* turning PID vals while reorienting */
  g_turn_orient_pid_vals.scale_factor = 2;
  g_turn_orient_pid_vals.prop_gain = 2 * g_turn_orient_pid_vals.scale_factor;
  g_turn_orient_pid_vals.int_gain = 1;
  g_turn_orient_pid_vals.deriv_gain = -3;
  g_turn_orient_pid_vals.max_integral = 30;
  g_turn_orient_pid_vals.max_val = 80;
  g_turn_orient_pid_vals.min_val = -80;

  /* turning PID vals while driving */
  g_turn_drive_pid_vals.scale_factor = 1;
  g_turn_drive_pid_vals.prop_gain = 4 * g_turn_drive_pid_vals.scale_factor;
  g_turn_drive_pid_vals.int_gain = 1;
  g_turn_drive_pid_vals.deriv_gain = -5;
  g_turn_drive_pid_vals.max_integral = 10;
  g_turn_drive_pid_vals.max_val = 60;
  g_turn_drive_pid_vals.min_val = -60;
#if 0
  /* values tuned before ship */
  /* turning PID vals while reorienting */
  g_turn_orient_pid_vals.scale_factor = 3;
  g_turn_orient_pid_vals.prop_gain = 6 * g_turn_orient_pid_vals.scale_factor;
  g_turn_orient_pid_vals.int_gain = 1;
  g_turn_orient_pid_vals.deriv_gain = -9;
  g_turn_orient_pid_vals.max_val = 70;
  g_turn_orient_pid_vals.min_val = -70;
  g_turn_orient_pid_vals.max_integral = g_turn_orient_pid_vals.max_val;

  /* turning PID vals while driving */
  g_turn_drive_pid_vals.scale_factor = 1;
  g_turn_drive_pid_vals.prop_gain = 2 * g_turn_drive_pid_vals.scale_factor;
  g_turn_drive_pid_vals.int_gain = 0;
  g_turn_drive_pid_vals.deriv_gain = -4;
  g_turn_drive_pid_vals.max_integral = 20;
  g_turn_drive_pid_vals.max_val = 40;
  g_turn_drive_pid_vals.min_val = -40;
#endif
#endif

  auto_output_off();
  lift_set_height(0);

  printf("Run auto program %d\r",(int)s_prog_num);

  /*
  if ((g_chosen_tetra == 3) || (g_chosen_tetra == 4) ||
      (g_chosen_tetra == 6) || (g_chosen_tetra == 7))
  {
    g_goal_id = GOAL_MIDDLE_CENTER;
  }
  */
  if ((g_chosen_tetra == 4) || (g_chosen_tetra == 6))
  {
    g_goal_id = GOAL_MIDDLE_CENTER;
  }
  else if ((g_chosen_tetra == 3) &&
           ((g_starting_pos == STARTING_POS_BLUE_LEFT) ||
            (g_starting_pos == STARTING_POS_BLUE_CENTER) ||
            (g_starting_pos == STARTING_POS_BLUE_RIGHT)))
  {
    g_goal_id = GOAL_MIDDLE_CENTER;
  }
  else if ((g_chosen_tetra == 7) &&
           ((g_starting_pos == STARTING_POS_RED_LEFT) ||
            (g_starting_pos == STARTING_POS_RED_CENTER) ||
            (g_starting_pos == STARTING_POS_RED_RIGHT)))
  {
    g_goal_id = GOAL_MIDDLE_CENTER;
  }
  else
  {
    g_goal_id = 0;
  }

  while (autonomous_mode == AUTO_ENABLED)
  {
    if (statusflag.NEW_SPI_DATA)      /* 26.2ms loop area */
    {
      Getdata(&rxdata); /* DO NOT DELETE, or it will be stuck here forever */

      /* Wildstang autonomous code */

      g_lift_encoder_val_prev_idx =
        (g_lift_encoder_val_prev_idx + 1) % LIFT_ENCODER_BUFFER_SZE;
      g_lift_encoder_val_prev[g_lift_encoder_val_prev_idx] =
        get_lift_encoder_count();

      if ((task_id != AUTO_PROGRAM_DONE) &&
         ((task_state == TASK_STATE_DONE) || (task_state == TASK_STATE_SKIP_NEXT)))
      {
        printf("GET NEXT TASK...<%d> <%d>\r",(int)task_id,(int)task_state);
        /* Current task is done, get next task */
        if(task_id+1 <  MAX_AUTO_TASKS &&
           program->task_list[task_id+1].function != NULL)
        {
          task_id++;
          printf("GOT PROG (%d) TASK (%d)\n",(int)s_prog_num,(int)task_id);
          if(task_state == TASK_STATE_SKIP_NEXT)
          {
            printf("SKIP NEXT TASK...<%d> <%d>\r",(int)task_id,(int)task_state);
            task_state = TASK_STATE_DONE;
            continue;
          }
        }
        else
        {
          task_id = AUTO_PROGRAM_DONE;
        }
      }

      if ((task_id != AUTO_PROGRAM_DONE) && (task_state != TASK_STATE_ABORT) &&
          (s_auto_locked_in == TRUE))
      {
        task_state = program->task_list[task_id].function(&(program->task_list[task_id].parameters));
      }
      else
      {
        printf("PROGRAM DONE!!!\n");
         /* PROGRAM DONE DO NOTHING !!! */
        auto_output_off();
      }

      pump_control();
      motor_vals.lift_height = lift_height_feedback();
      motor_vals.lift_tilt   = tilt_pos_feedback();
      assign_outputs_slow();

      Putdata(&txdata);   /* DO NOT DELETE, or you will get no PWM outputs! */
    }

    assign_outputs_fast();
    Generate_Pwms(pwm13,pwm14,pwm15,pwm16);

    display_auto_data();
  }

  if(program->use_vision == USE_VISION)
  {
    cc_set_camera_pos(CC_CMD_SET_CAMERA_STORE);
  }

  return;
}

/*******************************************************************************
* FUNCTION NAME: display_auto_data
* PURPOSE:       display autonomous data on the OI user display
* ARGUMENTS:     none
* RETURNS:       none
*******************************************************************************/
void display_auto_data(void)
{
  static UINT8 num_blinks = 0;
  static UINT16 num_cycles = 0;
  static UINT8 blink_state = BLINK_ON;
  //Default LEDs to off
  UINT8 do_led_on = 0;

  if ((Oi_calibrate < OI_CALIBRATE_JOYSTICKS) &&
      (Oi_calibrate > OI_CALIBRATE_ENCODERS))
  {
    /* only display auto data when not in joystick or encoder calibrate mode */
    LEFT_POS_LED = 0;
    CENTER_POS_LED = 0;
    RIGHT_POS_LED = 0;
    RED_SIDE_LED = 0;
    BLUE_SIDE_LED = 0;

    if(disabled_mode == ROBOT_DISABLED)
    {
      if(Oi_sw_auto_lockin == 1)
      {
        do_led_on = 1;
      }
      else
      {
        //  We are disabled and not locked in
        //  blink the program number (1-based)
        switch(blink_state)
        {
          case BLINK_ON:
            if(num_cycles == AUTO_LED_BLINK_END)
            {
              // We've been in the ON part of a blink
              // long enough
              // Move to off state, reset cycle count
              blink_state = BLINK_OFF;
              num_cycles = 0;
            }
            else
            {
              // Turn the LEDs on
              do_led_on = 1;
            }
            break;
          case BLINK_OFF:
            if(num_cycles == AUTO_LED_BLINK_END)
            {
              // We've been in the OFF part of a blink
              // long enough

              if(++num_blinks < (s_prog_num+1))
              {
                // We haven't blinked the number of times
                // for the current program
                // Reset the cycle count and start another blink
                num_cycles = 0;
                blink_state = BLINK_ON;
                do_led_on = 1;
              }
              else
              {
                // We've blinked the number of times for
                // the current program
                // Reset the number of blinks and move
                // to the delay state
                num_blinks = 0;
                blink_state = BLINK_DELAY;
              }
            }
            break;
          case BLINK_DELAY:
            if(num_cycles == AUTO_LED_BLINK_DELAY_END)
            {
              // We've delayed long enough
              // Start a new blink sequence
              num_cycles = 0;
              blink_state = BLINK_ON;
              do_led_on = 1;
            }
            break;
          default:
            break;
        }
        // Increment the cycle count
        num_cycles++;
      }
    }
    else if(autonomous_mode == AUTO_ENABLED)
    {
      do_led_on = 1;
    }

    if (do_led_on)
    {
      /* turn LEDs on when locked in or in auto; blink LEDs otherwise */
      switch (g_starting_pos)
      {
        case STARTING_POS_RED_LEFT:
          LEFT_POS_LED = 1;
          RED_SIDE_LED = 1;
          break;
        case STARTING_POS_RED_CENTER:
          CENTER_POS_LED = 1;
          RED_SIDE_LED = 1;
          break;
        case STARTING_POS_RED_RIGHT:
          RIGHT_POS_LED = 1;
          RED_SIDE_LED = 1;
          break;
        case STARTING_POS_BLUE_LEFT:
          LEFT_POS_LED = 1;
          BLUE_SIDE_LED = 1;
          break;
        case STARTING_POS_BLUE_CENTER:
          CENTER_POS_LED = 1;
          BLUE_SIDE_LED = 1;
          break;
        case STARTING_POS_BLUE_RIGHT:
          RIGHT_POS_LED = 1;
          BLUE_SIDE_LED = 1;
          break;
        default:
          break;
      }
    }
  }
}


/*******************************************************************************
* FUNCTION NAME: tetra_chooser
* PURPOSE:       chooses which tetra to track based on priority
* ARGUMENTS:
* RETURNS:
*******************************************************************************/
UINT8 tetra_chooser(UINT8 tetra1, UINT8 tetra2)
{
  /* tetra position of 0 means no tetra was found */

  if (tetra1 > 8)
  {
    tetra1 = 0;
  }

  if (tetra2 > 8)
  {
    tetra2 = 0;
  }

  if ((g_starting_pos == STARTING_POS_RED_LEFT) ||
      (g_starting_pos == STARTING_POS_BLUE_LEFT))
  {
    if (pos_left_priority[tetra1] <
        pos_left_priority[tetra2])
    {
      g_chosen_tetra = tetra1;
    }
    else
    {
      g_chosen_tetra = tetra2;
    }
  }
  else if ((g_starting_pos == STARTING_POS_RED_RIGHT) ||
           (g_starting_pos == STARTING_POS_BLUE_RIGHT))
  {
    if (pos_right_priority[tetra1] <
        pos_right_priority[tetra2])
    {
      g_chosen_tetra = tetra1;
    }
    else
    {
      g_chosen_tetra = tetra2;
    }
  }
  else if (g_starting_pos == STARTING_POS_RED_CENTER)
  {
    if (pos_center_r_priority[tetra1] <
        pos_center_r_priority[tetra2])
    {
      g_chosen_tetra = tetra1;
    }
    else
    {
      g_chosen_tetra = tetra2;
    }

    if (pos_center_r_priority[g_chosen_tetra] > TETRA_PRIORITY_THRESHOLD)
    {
      g_chosen_tetra = 0;
    }
  }
  else if (g_starting_pos == STARTING_POS_BLUE_CENTER)
  {
    if (pos_center_b_priority[tetra1] <
        pos_center_b_priority[tetra2])
    {
      g_chosen_tetra = tetra1;
    }
    else
    {
      g_chosen_tetra = tetra2;
    }

    if (pos_center_b_priority[g_chosen_tetra] > TETRA_PRIORITY_THRESHOLD)
    {
      g_chosen_tetra = 0;
    }
  }
  else
  {
    g_chosen_tetra = 0;
  }
}



/*******************************************************************************
* FUNCTION NAME: auto_output_off
* PURPOSE:       Sets the outputs to a safe state.  Used for initialization
*                as well as at the end of autonomous
* ARGUMENTS:
* RETURNS:       none
*******************************************************************************/
void auto_output_off()
{
  motor_vals.left_drive = 0;
  motor_vals.right_drive = 0;
  //motor_vals.lift_height = 0;
  motor_vals.lift_tilt = 0;
  tilt_set_pos(0);
  /*
  motor_vals.shifter_position = SHIFTER_HIGH;
  motor_vals.fwing = WING_IN;
  motor_vals.bwing = WING_IN;
  lift_set_height(0);
  */
}

