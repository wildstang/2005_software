/*******************************************************************************
* FILE NAME: ws_io.c
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
#include "ws_io.h"
#include "user_routines.h"


extern UINT8 g_chosen_tetra;

/*******************************************************************************
* FUNCTION NAME: assign_outputs_slow
* PURPOSE:       assign motor speeds to pwm outputs
* CALLED FROM:
* ARGUMENTS:
* RETURNS:       none
*******************************************************************************/
void assign_outputs_slow()
{
  static UINT8 bot_spear_retract_prev = SPEAR_IN;
  static UINT8 bot_spear_retract_counter = BOT_SPEAR_MOVE_COUNT;
  static UINT8 tilt_ratchet_counter = 0;


  pwm01 = 127;
  pwm02 = 127;
  pwm03 = 127;
  pwm04 = 127;
  pwm05 = 127;
  pwm06 = 127;
  pwm07 = 127;
  pwm08 = 127;
  pwm09 = 127;
  pwm10 = 127;
  pwm11 = 127;
  pwm12 = 127;

#ifdef MOTOR_TEST
  if ((Oi_drive_y > (127 - SC_CALIB_STICK_DEADZONE)) &&
      (Oi_drive_y < (127 + SC_CALIB_STICK_DEADZONE)))
  {
    Oi_drive_y = 127;
  }

  /*
  Rc_analog_out_l_drive_1 = Oi_drive_y;
  Rc_analog_out_l_drive_2 = Oi_drive_y;
  Rc_analog_out_r_drive_1 = Oi_drive_y;
  Rc_analog_out_r_drive_2 = Oi_drive_y;
  Rc_analog_out_lift_height = Oi_drive_y;
  Rc_analog_out_lift_tilt = Oi_drive_y;

  printf(" driveY=%d\r", (int)Oi_drive_y);
  */

#else

  Rc_analog_out_l_drive_1 = 127 + motor_vals.left_drive;
  Rc_analog_out_l_drive_2 = 127 + motor_vals.left_drive;
  Rc_analog_out_r_drive_1 = 127 + motor_vals.right_drive;
  Rc_analog_out_r_drive_2 = 127 + motor_vals.right_drive;

  Rc_analog_out_lift_height = 127 + motor_vals.lift_height;

  if(motor_vals.wing_lock == WING_LOCKED)
  {
    Rc_analog_out_bwing_lock = motor_vals.wing_lock;
    Rc_analog_out_fwing_lock = 255 - motor_vals.wing_lock;
  }
  else
  {
    Rc_analog_out_bwing_lock = WING_UNLOCKED;
    Rc_analog_out_fwing_lock = 255 - WING_UNLOCKED + 30;
  }


  if (motor_vals.lift_tilt > 0)
  {
    /* tilt is supposed to be moving out */
    /* disengage ratchet */
    Rc_analog_out_tilt_ratchet = 255;

    if (tilt_ratchet_counter < 5)
    {
      /* bring tilt in a little */
      motor_vals.lift_tilt = -30;
      tilt_ratchet_counter++;
    }
  }
  else
  {
    /* tilt is either stopped or moving in */
    /* engage ratchet */
    Rc_analog_out_tilt_ratchet = 127;
    tilt_ratchet_counter = 0;
  }

  Rc_analog_out_lift_tilt = 127 + motor_vals.lift_tilt;

  /*
  printf("tilt %d servo %d count %d\r", Rc_analog_out_lift_tilt,
         Rc_analog_out_tilt_ratchet, tilt_ratchet_counter);
  */
#endif


  /*
  printf(" drive: l %d r %d  lift: h %d t %d lock %d\r", motor_vals.left_drive,
         motor_vals.right_drive, motor_vals.lift_height, motor_vals.lift_tilt,
         motor_vals.wing_lock);
  */


  relay2_fwd = 0;
  relay2_rev = 0;
  relay3_fwd = 0;
  relay3_rev = 0;
  relay4_fwd = 0;
  relay4_rev = 0;
  Rc_relay_fwing = 0;
  Rc_relay_bwing = 0;
  Rc_relay_pump_on = 0;
  Rc_relay_pump_not_used = 0;
  relay7_fwd = 0;
  relay7_rev = 0;
  relay8_fwd = 0;
  relay8_rev = 0;

#ifdef MOTOR_TEST

#else
  /* assign relay outputs */

  /* Gear shift */
  if (motor_vals.shifter_position == SHIFTER_LOW)
  {
    Rc_relay_shifter_left = 1;
    Rc_relay_shifter_right = 1;
  }
  else
  {
    Rc_relay_shifter_left = 0;
    Rc_relay_shifter_right = 0;
  }

  /*
  printf("shifter l %d r %d\r", (int)Rc_relay_shifter_left,
         (int)Rc_relay_shifter_right);
  */

  /* pump */
  if (motor_vals.pump == PUMP_ON)
  {
    Rc_relay_pump_on = 1;
    Rc_relay_pump_not_used = 0;
  }
  else
  {
    Rc_relay_pump_on = 0;
    Rc_relay_pump_not_used = 0;
  }

  /*
  printf("pump %d\r", (int)Rc_relay_pump_on);
  */

  /* Front Wing */
  if (motor_vals.fwing == WING_IN)
  {
    Rc_relay_fwing = 0;
  }
  else
  {
    Rc_relay_fwing = 1;
  }

  /* Back Wing */
  if (motor_vals.bwing == WING_IN)
  {
    Rc_relay_bwing = 0;
  }
  else
  {
    Rc_relay_bwing = 1;
  }

  /*
  printf("wing f%d b%d\r", (int)Rc_relay_fwing, (int)Rc_relay_bwing);
  */

  /* Top Spear Grabber*/
  if (motor_vals.top_spear_grabber == SPEAR_NO_GRAB)
  {
    Rc_relay_grabber_top = 1;
  }
  else
  {
    Rc_relay_grabber_top = 0;
  }

  /* Top Spear Tilt*/
  if (motor_vals.top_spear_tilt == SPEAR_NO_TILT)
  {
    Rc_relay_hook_top = 0;
  }
  else
  {
    Rc_relay_hook_top = 1;
  }

  /* Top Spear Retract*/
  if (motor_vals.top_spear_retract == SPEAR_IN)
  {
    Rc_relay_spear_pos_top = 0;
  }
  else
  {
    Rc_relay_spear_pos_top = 1;
  }

  /* Bottom Spear Grabber*/
  if (motor_vals.bot_spear_grabber == SPEAR_NO_GRAB)
  {
    Rc_relay_grabber_bottom = 1;
  }
  else
  {
    Rc_relay_grabber_bottom = 0;
  }

  /* Bottom Spear Tilt*/
  if (motor_vals.bot_spear_tilt == SPEAR_NO_TILT)
  {
    Rc_relay_hook_bottom = 0;
  }
  else
  {
    Rc_relay_hook_bottom = 1;
  }


  /* Bottom Spear Retract*/
  if ((motor_vals.bot_spear_retract == SPEAR_IN) &&
      (bot_spear_retract_prev == SPEAR_OUT))
  {
    bot_spear_retract_counter = 0;
  }
  else if ((motor_vals.bot_spear_retract == SPEAR_OUT) &&
           (bot_spear_retract_prev == SPEAR_IN))
  {
    bot_spear_retract_counter = 0;
  }

  if (bot_spear_retract_counter < BOT_SPEAR_MOVE_COUNT)
  {
    if(motor_vals.bot_spear_retract == SPEAR_IN)
    {
      /* motor in */
      Rc_relay_spear_pos_bot_in = 1;
      Rc_relay_spear_pos_bot_out = 0;
    }
    else
    {
      /* motor out */
      Rc_relay_spear_pos_bot_in = 0;
      Rc_relay_spear_pos_bot_out = 1;
    }

    bot_spear_retract_counter++;
  }
  else
  {
    /* motor stop */
    Rc_relay_spear_pos_bot_in = 0;
    Rc_relay_spear_pos_bot_out = 0;
  }

  bot_spear_retract_prev = motor_vals.bot_spear_retract;


#if 0
  printf("Proxy: %d\r",Get_Analog_Value(Analog_in_front_proxy));
#endif

  /*
  printf("state %d in %d out %d\r", motor_vals.bot_spear_retract,
         Rc_relay_spear_pos_bot_in, Rc_relay_spear_pos_bot_out);
  */


/*
  printf("TG: %d TT: %d TR: %d BG: %d BT: %d\r",
          Rc_relay_grabber_top,Rc_relay_hook_top,Rc_relay_spear_pos_top,
          Rc_relay_grabber_bottom,Rc_relay_hook_bottom);
*/

  if (g_chosen_tetra != 0)
  {
    /* turn LED on */
    digital_io_18 = 0;
  }
  else
  {
    /* turn LED off */
    digital_io_18 = 1;
  }

  digital_io_17 = 0;

#endif

  return;
}


/*******************************************************************************
* FUNCTION NAME: assign_outputs_fast
* PURPOSE:       assign motor speeds to pwm outputs
* CALLED FROM:
* ARGUMENTS:
* RETURNS:       none
*******************************************************************************/
void assign_outputs_fast()
{
  pwm13 = 127;
  pwm14 = 127;
  pwm15 = 127;
  pwm16 = 127;

#ifdef MOTOR_TEST
  if ((Oi_drive_y > (127 - SC_CALIB_STICK_DEADZONE)) &&
      (Oi_drive_y < (127 + SC_CALIB_STICK_DEADZONE)))
  {
    Oi_drive_y = 127;
  }

  /*
  Rc_analog_out_pwm13 = Oi_drive_y;

  printf(" driveY %d\r", (int)Oi_drive_y);
  */

#else


#endif

  return;
}



/*******************************************************************************
* FUNCTION NAME: joystick_scaling
* PURPOSE:       Scale joystick so the x & y ranges are 0-254
* ARGUMENTS:     none
* RETURNS:       none
*******************************************************************************/
void joystick_scaling(UINT8 *joystick_val, UINT8 stick_min_val,
                      UINT8 stick_mid_val, UINT8 stick_max_val)
{
  int tmp_val;

  /************************************************************
  Since no two joysticks are the same, we want to make sure that
  all joysticks give a consistent 0-254 input to the rest of
  the code.

  This is done by scaling the joystick so that the value is in the
  the maps to the following range
        Input          Output
     stick_min_val ==>   0
     stick_mid_val ==>  127
     stick_max_val ==>  254

  The scaled joystick value is stored back into joystick_val

  !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  It is important to make sure that the return value is never
  less than 0 or greater than 254
  !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  ************************************************************/

  /*
  printf("in %d ", (int)*joystick_val);
  */

  if (*joystick_val > stick_mid_val)
  {
    tmp_val = ((int)(*joystick_val - stick_mid_val) * (int)(254 - 127)) /
              (int)(stick_max_val - stick_mid_val);

    /* prevent overflow if tmp_val is > 127 */
    *joystick_val = 127 + MAX_RETURN(tmp_val, 127);
  }
  else
  {
    tmp_val = ((int)(stick_mid_val - *joystick_val) * (int)(127 - 0)) /
              (int)(stick_mid_val - stick_min_val);

    /* prevent underflow if tmp_val is > 127 */
    *joystick_val = 127 - MAX_RETURN(tmp_val, 127);
  }

  /*
  printf("out %d\r", (int)*joystick_val);
  */

  return;
}



void io_print_oi_inputs()
{
  /*
  printf(" Drive x %d y %d ", (int)Oi_drive_x, (int)Oi_drive_y);
  printf("\r");
  */

  /*
  printf("pressure %d lift sensor %d\r", Dig_in_pressure, Dig_in_lift_bottom);
  */

  /*
  printf("p1 x %3d y %3d wheel %3d aux %3d top %d trig %d aux1 %d aux2 %d\r",
         (int)p1_x, (int)p1_y, (int)p1_wheel, (int)p1_aux,
         (int)p1_sw_top, (int)p1_sw_trig, (int)p1_sw_aux1, (int)p1_sw_aux2);
  printf("p2 x %3d y %3d wheel %3d aux %3d top %d trig %d aux1 %d aux2 %d\r",
         (int)p2_x, (int)p2_y, (int)p2_wheel, (int)p2_aux,
         (int)p2_sw_top, (int)p2_sw_trig, (int)p2_sw_aux1, (int)p2_sw_aux2);
  printf("p3 x %3d y %3d wheel %3d aux %3d top %d trig %d aux1 %d aux2 %d\r",
         (int)p3_x, (int)p3_y, (int)p3_wheel, (int)p3_aux,
         (int)p3_sw_top, (int)p3_sw_trig, (int)p3_sw_aux1, (int)p3_sw_aux2);
  printf("p4 x %3d y %3d wheel %3d aux %3d top %d trig %d aux1 %d aux2 %d\r",
         (int)p4_x, (int)p4_y, (int)p4_wheel, (int)p4_aux,
         (int)p4_sw_top, (int)p4_sw_trig, (int)p4_sw_aux1, (int)p4_sw_aux2);
  */

  return;
}


void io_print_rc_inputs()
{

  /*
  printf("dig %d %d %d %d\r", (int)Dig_in_lift_encoder_1,
         (int)Dig_in_tilt_encoder_1, (int)Dig_in_lift_encoder_2,
         (int)Dig_in_tilt_encoder_2);
  */

  /*
  printf("pressure %3d\r",
         (int)GET_ANALOG_VALUE_SHIFT(Analog_in_pressure_sensor));
  */

  return;
}


