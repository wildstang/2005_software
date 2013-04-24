/*******************************************************************************
* FILE NAME: ws_lift.c
*
* DESCRIPTION: This file contains all of the functions for controlling
*              the lift; manual and automatic positioning
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
#include "ws_general.h"
#include "ws_io.h"
#include "ws_calibrate.h"
#include "ws_lift.h"
#include "ws_encoder.h"
#include "ws_pid.h"


static UINT8 lift_encoder_state = LIFT_ENCODER_UNINITIALIZED;
UINT8 lift_mode = LIFT_MANU_MODE;
UINT8 tilt_mode = TILT_MANU_MODE;
UINT16 g_lift_encoder_val_prev[LIFT_ENCODER_BUFFER_SZE];
UINT8  g_lift_encoder_val_prev_idx = 0;
static PidValsType lift_pid_vals;
static PidValsType lift_speed_pid_vals;
static PidValsType tilt_pid_vals;
static UINT16 lift_desired_height = 0;
static UINT16 tilt_desired_pos = 0;
static UINT8 lift_same_counter = 0;
extern UINT8 show_pid;

/*******************************************************************************
* FUNCTION NAME: lift_init()
* PURPOSE:       Initializes PID constants
* ARGUMENTS:     none
* RETURNS:       none
* DOL:
*******************************************************************************/
void lift_init()
{
#ifdef PROTO_ROBOT
  /* Proto Values */
  lift_pid_vals.scale_factor = 1;
  lift_pid_vals.prop_gain = 3 * lift_pid_vals.scale_factor;
  lift_pid_vals.int_gain = 1;
  lift_pid_vals.deriv_gain = -4;
  lift_pid_vals.max_integral = 80;
  lift_pid_vals.min_val = -127;
  lift_pid_vals.max_val = 127;

  lift_speed_pid_vals.scale_factor = 7;
  lift_speed_pid_vals.prop_gain = 1 * lift_speed_pid_vals.scale_factor;
  lift_speed_pid_vals.int_gain = 1;
  lift_speed_pid_vals.deriv_gain = 1;
  lift_speed_pid_vals.max_integral = 127 * lift_speed_pid_vals.scale_factor;
  lift_speed_pid_vals.min_val = -127;
  lift_speed_pid_vals.max_val = 127;
#else
  /* Real Values */
  lift_pid_vals.scale_factor = 1;
  lift_pid_vals.prop_gain = 2 * lift_pid_vals.scale_factor;
  lift_pid_vals.int_gain = 1;
  lift_pid_vals.deriv_gain = -7;
  lift_pid_vals.max_integral = 80;
  lift_pid_vals.min_val = -127;
  lift_pid_vals.max_val = 127;

  lift_speed_pid_vals.scale_factor = 7;
  lift_speed_pid_vals.prop_gain = 1 * lift_speed_pid_vals.scale_factor;
  lift_speed_pid_vals.int_gain = 1;
  lift_speed_pid_vals.deriv_gain = 1;
  lift_speed_pid_vals.max_integral = 127 * lift_speed_pid_vals.scale_factor;
  lift_speed_pid_vals.min_val = -127;
  lift_speed_pid_vals.max_val = 127;
#endif

  for (g_lift_encoder_val_prev_idx = 0;
       g_lift_encoder_val_prev_idx < LIFT_ENCODER_BUFFER_SZE;
       g_lift_encoder_val_prev_idx++)
  {
    g_lift_encoder_val_prev[g_lift_encoder_val_prev_idx] =
      LIFT_DOWN_ENCODER_VAL;
  }

  g_lift_encoder_val_prev_idx = 0;

  set_lift_encoder_count(LIFT_DOWN_ENCODER_VAL);
}

/*******************************************************************************
* FUNCTION NAME: tilt_init()
* PURPOSE:       Initializes PID constants
* ARGUMENTS:     none
* RETURNS:       none
* DOL:
*******************************************************************************/
void tilt_init()
{
#ifdef PROTO_ROBOT
  /* Proto Values */
  tilt_pid_vals.scale_factor = 8;
  tilt_pid_vals.prop_gain = (tilt_pid_vals.scale_factor/4);
  tilt_pid_vals.int_gain = 0;
  tilt_pid_vals.deriv_gain = -0;
  tilt_pid_vals.max_integral = 127;
  tilt_pid_vals.min_val = -127;
  tilt_pid_vals.max_val = 127;
#else
  tilt_pid_vals.scale_factor = 8;
  tilt_pid_vals.prop_gain = (tilt_pid_vals.scale_factor/4);
  tilt_pid_vals.int_gain = 0;
  tilt_pid_vals.deriv_gain = -0;
  tilt_pid_vals.max_integral = 127;
  tilt_pid_vals.min_val = -127;
  tilt_pid_vals.max_val = 127;
  /* Real Values */
  /*
  tilt_pid_vals.scale_factor = 3;
  tilt_pid_vals.prop_gain = 1 * tilt_pid_vals.scale_factor;
  tilt_pid_vals.int_gain = 0;
  tilt_pid_vals.deriv_gain = -5;
  tilt_pid_vals.max_integral = 110;
  tilt_pid_vals.min_val = -0;
  tilt_pid_vals.max_val = 0;
  */
#endif

  set_tilt_encoder_count(TILT_DOWN_ENCODER_VAL);
}


/*******************************************************************************
* FUNCTION NAME: lift_oi_input()
* PURPOSE:       Handles data from joystick for lift
* ARGUMENTS:     OIData - lift joystick
* RETURNS:       none
* DOL:
*   Create a dead zone in the X & Y axis of the lift stick
*   IF in manual mode (driver moves joystick up or down)
*      process inputs of joystick and set lift height motors
*   ELSE if in auto mode (button tapped)
*      set height of lift to auto position
*      (each button tap will put the lift at the next heighest auto
*       position)
*   ENDIF
*******************************************************************************/
void lift_oi_input()
{

  /*
  printf("lift stick %d\r", (int)Oi_lift_height);
  */

  /* reset lift counter if limit switch at bottom is pressed */
  if (Dig_in_lift_bottom == 1)
  {
    set_lift_encoder_count(LIFT_DOWN_ENCODER_VAL);
    lift_encoder_state = LIFT_ENCODER_INITIALIZED;
  }

  if (Oi_sw_lift_tilt_move == 0)
  {
    /* Create a deadzone in the lift stick */
    DEADZONE(Oi_lift_height, LIFT_STICK_HEIGHT_DEADZONE);

    /* joystick movement puts us in manual mode */
    if (lift_mode != LIFT_OVERRIDE_MODE)
    {
      if ((Oi_lift_height > (127 + LIFT_STICK_SWITCH_DEADZONE)) ||
          (Oi_lift_height < (127 - LIFT_STICK_SWITCH_DEADZONE)) ||
          (Oi_sw_top_spear_grab == 1) || (Oi_sw_bot_spear_grab == 1) ||
          (Oi_sw_top_spear_tilt == 1) || (Oi_sw_bot_spear_tilt == 1) ||
          (Oi_sw_top_spear_retract == 1) || (Oi_sw_bot_spear_retract == 1))
      {
        lift_mode = LIFT_MANU_MODE;
      }

      /* disallow auto mode when encoder is uninitialized */
      if (lift_encoder_state == LIFT_ENCODER_INITIALIZED)
      {
#if 0
        /* button press puts us in auto mode */
        if ((Oi_sw_lift_auto_floor == 1) || (Oi_sw_lift_auto_loader == 1))
        {
          lift_mode = LIFT_AUTO_MODE;
        }
#endif

        /* hat press runs auto lift program */
        if (HAT_RANGE_CHECK(Oi_lift_programs, LIFT_HAT_AUTO_LOADER_RESET))
        {
          lift_mode = LIFT_AUTO_MODE_TETRA_GRABBER;
        }
      }
    }

    /* transition into and out of override mode */
    if ((Oi_sw_lift_override == 1) &&
        (Oi_sw_lift_override_prev == 0))
    {
      if (lift_mode != LIFT_OVERRIDE_MODE)
      {
        lift_mode = LIFT_OVERRIDE_MODE;
      }
      else
      {
        lift_mode = LIFT_MANU_MODE;
      }
    }
  }


  if (((lift_mode == LIFT_MANU_MODE) || (lift_mode == LIFT_OVERRIDE_MODE)) &&
      (Oi_sw_lift_tilt_move == 0))
  {
    lift_height_manual();
    lift_set_height(0);
    lift_same_counter = 0;
  }
  else if (lift_mode == LIFT_AUTO_MODE)
  {
    lift_height_auto();
    motor_vals.lift_height = lift_height_feedback();
  }
  else if (lift_mode == LIFT_AUTO_MODE_TETRA_GRABBER)
  {
    lift_program_auto_loader();
    motor_vals.lift_height = lift_height_feedback();
  }

  /*
  printf(" mode %d speed %d\r", (int)lift_mode, (int)motor_vals.lift_height);
  */

  return;
}


/*******************************************************************************
* FUNCTION NAME: tilt_oi_input()
* PURPOSE:       Handles data from joystick for tilt
* ARGUMENTS:
* RETURNS:
*******************************************************************************/
void tilt_oi_input()
{
  UINT16         tilt_encoder;

  DEADZONE(Oi_lift_tilt, LIFT_STICK_TILT_DEADZONE);

  /* only move tilt if button is pressed */
  if (Oi_sw_lift_tilt_move == 1)
  {
    /* switch to manual mode if joystick is moved */
    if (tilt_mode != TILT_OVERRIDE_MODE)
    {
      if ((Oi_lift_tilt > (127 + TILT_STICK_SWITCH_DEADZONE)) ||
          (Oi_lift_tilt < (127 - TILT_STICK_SWITCH_DEADZONE)))
      {
        tilt_mode = TILT_MANU_MODE;
      }

      /* switch to auto mode if hat is pressed */
      if (HAT_RANGE_CHECK(Oi_tilt_auto_pos, TILT_HAT_AUTO_POS_TILT))
      {
        lift_mode = LIFT_AUTO_MODE;
        tilt_set_pos(TILT_POSITION_DRIVER_AUTO_TILT);
      }
      else if (HAT_RANGE_CHECK(Oi_tilt_auto_pos, TILT_HAT_AUTO_POS_VERT))
      {
        lift_mode = LIFT_AUTO_MODE;
        tilt_set_pos(TILT_POSITION_STRAIGHT_UP);
      }
    }


    if ((tilt_mode == TILT_MANU_MODE) || (tilt_mode == TILT_OVERRIDE_MODE))
    {
      tilt_set_pos(0);
      motor_vals.lift_tilt = 127 - (int)Oi_lift_tilt;

      if (motor_vals.lift_tilt > 0)
      {
        /* scale by 1/4 when tilting out */
        motor_vals.lift_tilt = motor_vals.lift_tilt / 4;
      }
      else
      {
        /* scale by 1/2 when tilting in */
        motor_vals.lift_tilt = motor_vals.lift_tilt / 2;
      }

#if 0
      if (tilt_mode != TILT_OVERRIDE_MODE)
      {
        tilt_encoder = Get_Analog_Value(Analog_in_lift_tilt_pot);

        if ((Oi_lift_tilt < 127) && (tilt_encoder > LIFT_TILT_POT_MIN))
        {
          motor_vals.lift_tilt = motor_vals.lift_tilt;
        }
        else if ((Oi_lift_tilt > 127) && (tilt_encoder < LIFT_TILT_POT_MAX))
        {
          motor_vals.lift_tilt = motor_vals.lift_tilt;
        }
        else
        {
          motor_vals.lift_tilt = 0;
        }
      }
#endif
    }
  }
  else
  {
    /* don't move lift tilt */
    motor_vals.lift_tilt = 0;
  }

  /* transition into and out of override mode */
  if ((Oi_sw_tilt_override == 1) &&
      (Oi_sw_tilt_override_prev == 0))
  {
    if (tilt_mode != TILT_OVERRIDE_MODE)
    {
      tilt_mode = TILT_OVERRIDE_MODE;
    }
    else
    {
      tilt_mode = TILT_MANU_MODE;
    }
  }

  /*
  if (tilt_mode == TILT_AUTO_MODE)
  {
    motor_vals.lift_tilt = tilt_pos_feedback();
  }
  */
}

/*******************************************************************************
* FUNCTION NAME: tilt_set_pos()
* PURPOSE:       set tilt pos for feedback routine
* CALLED FROM:
* ARGUMENTS:
* RETURNS:
*******************************************************************************/
void tilt_set_pos(UINT16 desired_post)
{
  /* height of 0 means no feedback */
  tilt_desired_pos = desired_post;
}

/*******************************************************************************
* FUNCTION NAME: tilt_pos_feedback()
* PURPOSE:
* CALLED FROM:
* ARGUMENTS:     desired position (encoder value) of the tilt
* RETURNS:       speed
*******************************************************************************/
INT8 tilt_pos_feedback()
{
  INT16    speed = 0;
  UINT16   curr_pos;

  if (tilt_desired_pos != 0)
  {
    /* get the current lift position from encoder */
    curr_pos = get_tilt_encoder_count();

    if (disabled_mode == ROBOT_ENABLED)
    {
      if(((tilt_desired_pos - curr_pos) < TILT_AUTO_DEADBAND) ||
         ((curr_pos - tilt_desired_pos)  > -TILT_AUTO_DEADBAND))
      {
        speed = 0;
      }
      else
      {
        show_pid = 0;
        speed = ws_pid(&tilt_pid_vals, tilt_desired_pos, curr_pos);
      }
    }

    /*
    printf("d %d c %d s %d\r", (int)tilt_desired_pos, (int)curr_pos, speed);
    */
  }

  return speed;
}



/*******************************************************************************
* FUNCTION NAME: lift_height_manual()
* PURPOSE:       Handles joystick inputs and sets motor vals
* CALLED FROM:   lift_oi_input()
* ARGUMENTS:     none
* RETURNS:       void
*******************************************************************************/
void lift_height_manual()
{
  UINT16 lift_height;

  lift_height = get_lift_encoder_count();

  /*
  printf(" height %d top %d bot %d", (int)lift_height,
         (int)LIFT_HEIGHT_LIMIT_TOP, (int)LIFT_HEIGHT_LIMIT_BOTTOM);
  printf(" lh %d\r", lift_height);
  */

#if 1
  /* make sure lift height does not overrun limits */
  if ((Oi_lift_height > 127) &&
      (((lift_encoder_state == LIFT_ENCODER_INITIALIZED)  &&
        (lift_height < LIFT_HEIGHT_LIMIT_TOP)) ||
       (lift_mode == LIFT_OVERRIDE_MODE)))
  {
    /* move lift up */
    motor_vals.lift_height = (int)Oi_lift_height - 127;
  }
  else if ((Oi_lift_height < 127) &&
           ((lift_height > LIFT_HEIGHT_LIMIT_BOTTOM) ||
            (lift_encoder_state == LIFT_ENCODER_UNINITIALIZED)) ||
            (lift_mode == LIFT_OVERRIDE_MODE))
  {
    /* move lift down */
    motor_vals.lift_height = (int)Oi_lift_height - 127;
  }
  else if ((Oi_lift_height < 127) &&
           (lift_height < LIFT_HEIGHT_LIMIT_BOTTOM) &&
           (Dig_in_lift_bottom == 0))
  {
    /* move lift down slowly until it hits limit switch */
    motor_vals.lift_height = MIN_RETURN(((int)Oi_lift_height - 127),
                                        LIFT_DOWN_SLOW_SPEED);
  }
  else
  {
    /* don't move lift */
    motor_vals.lift_height = 0;
  }
#else
  /* move lift */
  motor_vals.lift_height = (int)Oi_lift_height - 127;
#endif

  //motor_vals.lift_height = lift_speed_feedback(motor_vals.lift_height);
}

/*******************************************************************************
* FUNCTION NAME: lift_height_auto()
* PURPOSE:       Handles auto position input and sets motor vals
* CALLED FROM:   lift_oi_input()
* ARGUMENTS:     none
* RETURNS:       void
*******************************************************************************/
void lift_height_auto()
{
  static UINT16 lift_height;


#if 0
  if (Oi_sw_lift_auto_floor == 1)
  {
    lift_height = LIFT_BOTTOM;
  }
  else if (Oi_sw_lift_auto_loader == 1)
  {
    lift_height = LIFT_LOAD_BOTTOM;
  }
#endif

  /* set desired lift height */
  lift_set_height(lift_height);
}


/*******************************************************************************
* FUNCTION NAME: lift_set_height()
* PURPOSE:       set lift height for feedback routine
* CALLED FROM:
* ARGUMENTS:
* RETURNS:
*******************************************************************************/
void lift_set_height(UINT16 desired_height)
{
  /* height of 0 means no feedback */
  lift_desired_height = desired_height;
}


/*******************************************************************************
* FUNCTION NAME: lift_height_feedback()
* PURPOSE:
* CALLED FROM:   lift_height_auto()
* ARGUMENTS:     desired position (encoder value) of the lift
* RETURNS:       speed
*******************************************************************************/
INT8 lift_height_feedback()
{
  static UINT8  hysteresis = HYST_REACHED_TGT;
  INT16    speed = 0;
  UINT16   lift_pos;

  /* reset lift counter if limit switch at bottom is pressed */
  if (Dig_in_lift_bottom == 1)
  {
    set_lift_encoder_count(LIFT_DOWN_ENCODER_VAL);
    lift_encoder_state = LIFT_ENCODER_INITIALIZED;
  }

  if ((lift_encoder_state == LIFT_ENCODER_INITIALIZED) &&
      (lift_desired_height != 0))
  {
    /* get the current lift position from encoder */
    lift_pos = get_lift_encoder_count();

    /*
    printf("d %d c %d\r", (int)lift_desired_height, (int)lift_pos);
    */

    if (disabled_mode == ROBOT_ENABLED)
    {
      if (lift_pos > lift_desired_height)
      {
        hysteresis = HYST_REACHED_TGT;
      }
      else if (lift_pos < (lift_desired_height - LIFT_HEIGHT_CLOSE_DIST))
      {
        hysteresis = HYST_BELOW_THRESHOLD;
      }

      if ((lift_desired_height == LIFT_HEIGHT_LIMIT_BOTTOM) &&
          (lift_pos <= LIFT_HEIGHT_LIMIT_BOTTOM) &&
          (Dig_in_lift_bottom == 0))
      {
        /* move lift down slowly until it hits limit switch */
        speed = LIFT_DOWN_SLOW_SPEED;
      }
      else if ((hysteresis == HYST_REACHED_TGT) &&
               (lift_pos <= lift_desired_height) &&
               (lift_pos >= (lift_desired_height - LIFT_HEIGHT_CLOSE_DIST)))
      {
        speed = 0;
      }
      else if ((lift_pos == LIFT_HEIGHT_LIMIT_BOTTOM) &&
               (lift_desired_height <= LIFT_HEIGHT_LIMIT_BOTTOM))
      {
        speed = 0;
      }
      else
      {
        speed = ws_pid(&lift_pid_vals, lift_pos, lift_desired_height);
      }

#if 0
      /* check for dead encoder */
      if ((speed != 0) &&
          (lift_pos == g_lift_encoder_val_prev[g_lift_encoder_val_prev_idx]))
      {
        /* trying to drive lift & lift is not moving */
        lift_same_counter++;

        if (lift_same_counter > LIFT_DEAD_ENCODER_COUNT)
        {
          /* dead encoder */
          lift_mode = LIFT_OVERRIDE_MODE;
        }
      }
      else
      {
        lift_same_counter = 0;
      }
#endif
    }
  }



  /*
  printf("lift h %d d %d hy %d s %d\r", lift_pos, lift_desired_height,
         hysteresis, speed);
  */

  return speed;
}


/*******************************************************************************
* FUNCTION NAME: lift_speed_feedback()
* PURPOSE:
* ARGUMENTS:
* RETURNS:       motor speed
*******************************************************************************/
INT8 lift_speed_feedback(INT8 desired_speed)
{
  INT16  lift_speed;

#if 0
  if (lift_mode != LIFT_OVERRIDE_MODE)
  {
    /* lift speed is (current encoder value) - (previous encoder value)
       need to multiply by scaling factor to convert from encoder ticks per
       loop units to 'joystick' units */
    lift_speed =
      (INT16)g_lift_encoder_val_prev[g_lift_encoder_val_prev_idx] -
      (INT16)g_lift_encoder_val_prev[(g_lift_encoder_val_prev_idx + 1) % 4];

    /*
    printf("lift p %d %d %d %d i %d s %d\r",
           g_lift_encoder_val_prev[(g_lift_encoder_val_prev_idx + 1) % 4],
           g_lift_encoder_val_prev[(g_lift_encoder_val_prev_idx + 2) % 4],
           g_lift_encoder_val_prev[(g_lift_encoder_val_prev_idx + 3) % 4],
           g_lift_encoder_val_prev[g_lift_encoder_val_prev_idx],
           g_lift_encoder_val_prev_idx, lift_speed);
    */

    lift_speed = (lift_speed * LIFT_SPEED_SCALE_NUMERATOR) /
                 LIFT_SPEED_SCALE_DENOMINATOR;

    /*
    printf("c %d t %d  ", lift_speed, desired_speed);
    */

    /* clear integral term if desired speed is 0 and error is 0 */
    if ((desired_speed == 0) && (lift_speed_pid_vals.last_error == 0))
    {
      lift_speed_pid_vals.integral = 0;
    }

    lift_speed = ws_pid(&lift_speed_pid_vals, lift_speed, desired_speed);
  }
  else
  {
#endif
    lift_speed = desired_speed;
#if 0
  }
#endif

  return((INT8)lift_speed);
}


/*******************************************************************************
* FUNCTION NAME: spear_oi_input()
* PURPOSE:       Handles data from joystick/button box for spear
* ARGUMENTS:     OIData - spear joystick
* RETURNS:       none
* DOL:
*******************************************************************************/
void spear_control()
{
  /* if Oi_sw_top_spear_grab is tapped, change state of top grabber */
  motor_vals.top_spear_grabber = toggle_on_tap(Oi_sw_top_spear_grab,
                                               Oi_sw_top_spear_grab_prev,
                                               motor_vals.top_spear_grabber,
                                               SPEAR_GRAB,
                                               SPEAR_NO_GRAB);

  /* if Oi_sw_bot_spear_grab is tapped, change state of bottom grabber */
  motor_vals.bot_spear_grabber = toggle_on_tap(Oi_sw_bot_spear_grab,
                                               Oi_sw_bot_spear_grab_prev,
                                               motor_vals.bot_spear_grabber,
                                               SPEAR_GRAB,
                                               SPEAR_NO_GRAB);

  /* if Oi_sw_top_spear_tilt is tapped, change state of top tilt */
  motor_vals.top_spear_tilt = toggle_on_tap(Oi_sw_top_spear_tilt,
                                            Oi_sw_top_spear_tilt_prev,
                                            motor_vals.top_spear_tilt,
                                            SPEAR_NO_TILT,
                                            SPEAR_TILT_DOWN);

  /* if Oi_sw_bot_spear_tilt is tapped, change state of bottom tilt */
  motor_vals.bot_spear_tilt = toggle_on_tap(Oi_sw_bot_spear_tilt,
                                            Oi_sw_bot_spear_tilt_prev,
                                            motor_vals.bot_spear_tilt,
                                            SPEAR_NO_TILT,
                                            SPEAR_TILT_DOWN);

  /* if Oi_sw_top_spear_retract is tapped, change state of top retractor */
  motor_vals.top_spear_retract = toggle_on_tap(Oi_sw_top_spear_retract,
                                               Oi_sw_top_spear_retract_prev,
                                               motor_vals.top_spear_retract,
                                               SPEAR_IN,
                                               SPEAR_OUT);

  /* if Oi_sw_bot_spear_retract is tapped, change state of bottom retractor */
  motor_vals.bot_spear_retract = toggle_on_tap(Oi_sw_bot_spear_retract,
                                               Oi_sw_bot_spear_retract_prev,
                                               motor_vals.bot_spear_retract,
                                               SPEAR_IN,
                                               SPEAR_OUT);
}

/*******************************************************************************
* FUNCTION NAME: lift_program_auto_loader()
* PURPOSE:       runs lift program that automatically controls lift & spears to
*                pick up two tetras from the auto loaders
* ARGUMENTS:
* RETURNS:       none
* DOL:
*******************************************************************************/

void lift_program_auto_loader()
{
  static UINT8 lift_prog_state = LOADER_PROG_SET_TOP;
  static UINT8 raise_wait_counter = 0;

  if( HAT_RANGE_CHECK(Oi_lift_programs,LIFT_HAT_AUTO_LOADER_RESET))
  {
    lift_prog_state = LOADER_PROG_SET_TOP;
    lift_pid_vals.integral = 0;
  }
  else if(HAT_RANGE_CHECK(Oi_lift_programs_prev,HAT_NONE) &&
          HAT_RANGE_CHECK(Oi_lift_programs,LIFT_HAT_AUTO_LOADER_STEP) &&
          lift_prog_state != LOADER_PROG_DONE)
  {
    lift_prog_state++;
    lift_pid_vals.integral = 0;
  }

  switch (lift_prog_state)
  {
    case LOADER_PROG_SET_TOP:

      /* Bring bottom spear in */
      motor_vals.bot_spear_grabber = SPEAR_GRAB;
      motor_vals.bot_spear_tilt = SPEAR_NO_TILT;
      motor_vals.bot_spear_retract = SPEAR_IN;

      /* Open top spear */
      motor_vals.top_spear_grabber = SPEAR_NO_GRAB;
      motor_vals.top_spear_tilt = SPEAR_NO_TILT;
      motor_vals.top_spear_retract = SPEAR_IN;

      /* Set lift height to spear tetra on autoloader with top spear  */
      lift_set_height(LIFT_HEIGHT_TOP_AUTOLOAD);
      raise_wait_counter = 0;

      break;

    case LOADER_PROG_RAISE_FOR_BOTTOM:

      /* Bring bottom spear in */
      motor_vals.bot_spear_grabber = SPEAR_GRAB;
      motor_vals.bot_spear_tilt = SPEAR_NO_TILT;
      motor_vals.bot_spear_retract = SPEAR_IN;

      /* Close Top Grabber */
      motor_vals.top_spear_grabber = SPEAR_GRAB;
      motor_vals.top_spear_tilt = SPEAR_NO_TILT;
      motor_vals.top_spear_retract = SPEAR_OUT;

      if (raise_wait_counter < 5)
      {
        lift_set_height(LIFT_HEIGHT_TOP_AUTOLOAD);
        raise_wait_counter++;
      }
      else
      {
        /* Set lift height to clear auto loader */
        lift_set_height(LIFT_HEIGHT_TOP_CLEAR_AUTOLOAD);
      }
#if 0
        /* Once lift clears the top spear, move to LOADER_PROG_SET_BOTTOM */
      if(motor_vals.lift_height >= LIFT_HEIGHT_TOP_CLEAR_AUTOLOAD)
      {
        lift_prog_state = LOADER_PROG_SET_BOTTOM;
      }
#endif
      break;

    case LOADER_PROG_SET_BOTTOM:

      /* Open Bottom Spear */
      motor_vals.bot_spear_grabber = SPEAR_NO_GRAB;
      motor_vals.bot_spear_tilt = SPEAR_NO_TILT;
      motor_vals.bot_spear_retract = SPEAR_OUT;

      /* Close Top Grabber */
      motor_vals.top_spear_grabber = SPEAR_GRAB;
      motor_vals.top_spear_tilt = SPEAR_NO_TILT;
      motor_vals.top_spear_retract = SPEAR_OUT;

      /* Set lift height to spear tetra on autoloader with bottom spear */
      lift_set_height(LIFT_HEIGHT_BOT_AUTOLOAD);

      break;

    case LOADER_PROG_GOT_BOTTOM:
      /* Close Bottom Grabber */
      motor_vals.bot_spear_grabber = SPEAR_GRAB;
      motor_vals.bot_spear_tilt = SPEAR_NO_TILT;
      motor_vals.bot_spear_retract = SPEAR_OUT;

      /* Close Top Grabber */
      motor_vals.top_spear_grabber = SPEAR_GRAB;
      motor_vals.top_spear_tilt = SPEAR_NO_TILT;
      motor_vals.top_spear_retract = SPEAR_OUT;

      /* Raise lift to pick up tetra with bottom spear */
      lift_set_height(LIFT_HEIGHT_BOT_CLEAR_AUTOLOAD);

      break;

    case LOADER_PROG_TUCK_BOTTOM:
      /* Close Bottom Grabber */
      motor_vals.bot_spear_grabber = SPEAR_GRAB;
      motor_vals.bot_spear_tilt = SPEAR_NO_TILT;
      /* Pull the Bottom Spear in */
      motor_vals.bot_spear_retract = SPEAR_IN;

      /* Close Top Grabber */
      motor_vals.top_spear_grabber = SPEAR_GRAB;
      motor_vals.top_spear_tilt = SPEAR_NO_TILT;
      motor_vals.top_spear_retract = SPEAR_OUT;

      /* Raise lift to pick up tetra with bottom spear */
      lift_set_height(LIFT_HEIGHT_BOT_CLEAR_AUTOLOAD);
      break;
    default:
      break;
  }
/*
  printf("S: %d TR %d TG %d TT %d BR %d BG %d BT %d\r",
          lift_prog_state,
          motor_vals.top_spear_retract,
          motor_vals.top_spear_grabber,
          motor_vals.top_spear_tilt,
          motor_vals.bot_spear_retract,
          motor_vals.bot_spear_grabber,
          motor_vals.bot_spear_tilt
          );
*/
}
