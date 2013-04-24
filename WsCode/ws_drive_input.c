/*******************************************************************************
* FILE NAME: ws_drive_input.c
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
#include "ws_drive_input.h"

/*******************************************************************************
* FUNCTION NAME: drive_stick_input
* PURPOSE:       Process position of drive joystick
* CALLED FROM:   Process_Data_From_Master_uP()
* ARGUMENTS:
* RETURNS:       none
*******************************************************************************/
void drive_stick_input(UINT8 use_deadzone)
{
  static INT16 right_drive_speed_prev = 0;
  static INT16 left_drive_speed_prev = 0;
  INT16  right_drive_speed, left_drive_speed;

  /*
  printf("x %d y %d t %d s %d", (int)Oi_drive_x, (int)Oi_drive_y,
                  (int) Oi_sw_turbo,(int) Oi_sw_shifter);
  */

  /* create deadzone in drive stick */
  if (use_deadzone == TRUE)
  {
    DEADZONE(Oi_drive_y, DRIVE_DEADZONE);
    DEADZONE(Oi_drive_x, DRIVE_DEADZONE);
  }

  /* Calculate left & right motor speeds for single stick drive */
  /* right speed = joystick_y + joystick_x - 127
     left speed  = joystick_y - joystick_x + 127
     check for a min of 0 and a max of 254 */
  right_drive_speed = (INT16)Oi_drive_y + (INT16)Oi_drive_x - 127;
  MIN(right_drive_speed, 0);
  MAX(right_drive_speed, 254);

  left_drive_speed = (INT16)Oi_drive_y - (INT16)Oi_drive_x + 127;
  MIN(left_drive_speed, 0);
  MAX(left_drive_speed, 254);

  /* remap left & right drive speed from 0 to 254 range to -127 to 127 range */
  left_drive_speed -= 127;
  right_drive_speed -= 127;


  /* Anti-Turbo - if the turbo button is not pressed while in
                  high gear, slow the drive motors down by
                  (100*DRIVE_SCALE_NUMERATOR/DRIVE_SCALE_DENOMINATOR)%
  */
  if ((Oi_sw_turbo == 0) &&
      (Oi_sw_shifter == SHIFT_SWITCH_HIGH))
  {
    left_drive_speed = (left_drive_speed * DRIVE_SCALE_NUMERATOR) /
                       DRIVE_SCALE_DENOMINATOR;
    right_drive_speed = (right_drive_speed * DRIVE_SCALE_NUMERATOR) /
                        DRIVE_SCALE_DENOMINATOR;
  }

  /* Adjust right & left drive speeds to accelerate/decelerate */
  right_drive_speed = drive_acceleration_adjust(right_drive_speed,
                                                right_drive_speed_prev);
  left_drive_speed = drive_acceleration_adjust(left_drive_speed,
                                               left_drive_speed_prev);

  /* assign motor values from right & left drive speeds */
  motor_vals.left_drive = left_drive_speed;
  motor_vals.right_drive = right_drive_speed;

  /* assign shifter position from shifter_position */
  if (Oi_sw_shifter == SHIFT_SWITCH_LOW)
  {
    motor_vals.shifter_position = SHIFTER_LOW;
  }
  else
  {
    motor_vals.shifter_position = SHIFTER_HIGH;
  }

  /*
  printf("l %d r %d\r", (int)left_drive_speed, (int)right_drive_speed);
  */

  /* save right & left drive speeds for next loop */
  right_drive_speed_prev = right_drive_speed;
  left_drive_speed_prev = left_drive_speed;

  return;
}


/*******************************************************************************
* FUNCTION NAME: drive_acceleration_adjust
* PURPOSE:       Ramp up / down wheel speed
* ARGUMENTS:
*
* RETURNS:       none
* DOL:
******************************************************************************/
INT16 drive_acceleration_adjust(INT16 desired_wheel_speed,
                                INT16 curr_wheel_speed)
{
  INT16 new_wheel_speed = 127;


  if ((desired_wheel_speed + DRIVE_ACCEL_RATE) < curr_wheel_speed)
  {
    new_wheel_speed = curr_wheel_speed - DRIVE_ACCEL_RATE;
  }
  else if ((desired_wheel_speed - DRIVE_ACCEL_RATE) > curr_wheel_speed)
  {
    new_wheel_speed = curr_wheel_speed + DRIVE_ACCEL_RATE;
  }
  else
  {
    new_wheel_speed = desired_wheel_speed;
  }

  /*
  printf("D: %d C: %d N: %d\r", desired_wheel_speed, curr_wheel_speed,
      new_wheel_speed);
  */

  return new_wheel_speed;
}


