/*******************************************************************************
* FILE NAME: ws_general.c
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

#include "ws_general.h"
#include "ws_io.h"

/*******************************************************************************
* FUNCTION NAME: display_oi_data
* PURPOSE:       Displays data on OI user display when in user mode
* ARGUMENTS:     print_data - data to print
*                type
* RETURNS:       none
*******************************************************************************/
void display_oi_data(UINT8 print_data, DisplayDataType type)
{
  if ((Oi_calibrate > OI_CALIBRATE_ENCODERS) &&
      (Oi_calibrate < OI_CALIBRATE_JOYSTICKS))
  {
    /* not in pot or joystick calibration mode */
    if ((disabled_mode == ROBOT_ENABLED) && (autonomous_mode == AUTO_DISABLED))
    {
      /* robot is enabled  and autonomous is disabled */
      if (type == DISPLAY_DATA_OVERRIDE)
      {
        /* display the psi data */
        User_Mode_byte = print_data;
      }
    }
    else
    {
      if (type == DISPLAY_DATA_AUTO)
      {
        /* display autonomous data */
        User_Mode_byte = print_data;
      }
    }
  }
  else
  {
    if (type == DISPLAY_DATA_CALIBRATE)
    {
      User_Mode_byte = print_data;
    }
  }

  return;
}


/*******************************************************************************
* FUNCTION NAME: pump_control
* PURPOSE:       turn pump on & off depending on pressure
* ARGUMENTS:
* RETURNS:       none
*******************************************************************************/
void pump_control()
{
  /* This variable represents the number
     of cycles the pump is on after the
     switch tells us that we're at 120 PSI*/
  static int cycles_after_sensor = 0;

  /* Check to see if pressure switch is off
     This means that we are less than 120 PSI.
     In this case we want the pump to turn on.

     If the pressure switch indicates that we are
     greater than 120PSI, we want to run for PUMP_TOP_OFF
     cycles before it turns off

     Input variables:
     -------------------
     Dig_in_pressure - The variable containing the state
                       of the pressure switch
                       0 = Less than or equal to 120PSI
                       1 = Greater than 120PSI

     competition_mode - The variable containing the state
                        of the robot.  This will have one of
                        the following values.
                        ROBOT_ENABLED
                        ROBOT_DISABLED

     Output variables:
     -------------------
     motor_vals.pump - The output variable that will control
                       the pump.  It can be set to the following
                       values - PUMP_ON, PUMP_OFF
  */

/*
    if ((Dig_in_pressure == PRESSURE_BELOW_120) ||
        (cycles_after_sensor < 300))
    {
      motor_vals.pump = PUMP_ON;
    }
    else
    {
      motor_vals.pump = PUMP_OFF;
    }
    */

  /* if pressure sensor is < 120, set counter to 0 */
  if (Dig_in_pressure == PRESSURE_BELOW_120)
  {
   cycles_after_sensor = 0;
  }
  else if ((motor_vals.pump == PUMP_ON) && (disabled_mode == ROBOT_ENABLED))
  {
    cycles_after_sensor = cycles_after_sensor + 1;
  }


  /* else pump is on & robot is enabled increment counter */



  if (cycles_after_sensor < 300)
  {
    /* if counter is < 300 turn pump on */
    motor_vals.pump = PUMP_ON;
  }
  else
  {
    /* else turn pump off */
    motor_vals.pump = PUMP_OFF;
  }

  return;
}


/*******************************************************************************
* FUNCTION NAME: wing_control
* PURPOSE:       Set wing positions (in/out) based on OI inputs
* ARGUMENTS:
* RETURNS:       none
*******************************************************************************/
void wing_control()
{

  /*
     Front Wing Control

     Input variables:
     -------------------
     Oi_sw_fwing - The variable containing the state of the joystick button
                   that controls the front wing.

     Output variables:
     -------------------
     motor_vals.fwing - The output variable that will control
                        the front wing.  It can be set to the following
                        values:
                        WING_IN
                        WING_OUT

     Back Wing Control

     Input variables:
     -------------------
     Oi_sw_bwing - The variable containing the state of the joystick button
                   that controls the back wing.

     Output variables:
     -------------------
     motor_vals.bwing - The output variable that will control
                        the back wing.  It can be set to the following
                        values:
                        WING_IN
                        WING_OUT
  */

  /* toggle front wing position on button tap */
  motor_vals.fwing = toggle_on_tap(Oi_sw_fwing,
                                   Oi_sw_fwing_prev,
                                   motor_vals.fwing,
                                   WING_IN,
                                   WING_OUT);

  /* toggle back wing position on button tap */
  motor_vals.bwing = toggle_on_tap(Oi_sw_bwing,
                                   Oi_sw_bwing_prev,
                                   motor_vals.bwing,
                                   WING_IN,
                                   WING_OUT);

  if (HAT_RANGE_CHECK(Oi_drive_wing_lock, WING_LOCK_HAT))
  {
    motor_vals.wing_lock = WING_LOCKED;
  }
  else if (HAT_RANGE_CHECK(Oi_drive_wing_lock, WING_UNLOCK_HAT))
  {
    motor_vals.wing_lock = WING_UNLOCKED;
  }
}

/*******************************************************************************
* FUNCTION NAME: toggle_on_tap
* PURPOSE:       Toggle a parameter when a button is tapped
* ARGUMENTS:     UINT8 cur_button - The current button state
*                UINT8 prev_button - The button state for the previous loop
*                UINT8 prev_state - The parameter state for the previous loop
*                UINT8 state1 - A state that the parameter can have
*                UINT8 state2 - The other state that the parameter can have
* RETURNS:       UINT8 new_state - The new state of the parameter
*******************************************************************************/
UINT8 toggle_on_tap(  UINT8 cur_button,
                      UINT8 prev_button,
                      UINT8 prev_state,
                      UINT8 state1,
                      UINT8 state2)
{
  UINT8 new_state = prev_state;
  if ((cur_button == 1) && (prev_button == 0))
  {
    if (prev_state == state1)
    {
      new_state = state2;
    }
    else
    {
      new_state = state1;
    }
  }
//  printf("Cur %d PrevB %d  PrevS %d  NewS %d\r",cur_button,prev_button,prev_state,new_state);
  return new_state;
}

