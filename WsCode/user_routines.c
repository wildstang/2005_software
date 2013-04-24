/*******************************************************************************
* FILE NAME: user_routines.c <FRC VERSION>
*
* DESCRIPTION:
*  This file contains the default mappings of inputs
*  (like switches, joysticks, and buttons) to outputs on the RC.
*
* USAGE:
*  You can either modify this file to fit your needs, or remove it from your
*  project and replace it with a modified copy.
*
*******************************************************************************/
#include <stdio.h>
#include "ifi_aliases.h"
#include "ifi_default.h"
#include "ifi_utilities.h"
#include "ws_includes.h"
#include "ws_io.h"
#include "user_routines.h"
#include "user_Serialdrv.h"
#include "ws_drive_input.h"
#include "ws_io.h"
#include "ws_camera.h"
#include "ws_calibrate.h"
#include "ws_general.h"
#include "ws_lift.h"
#include "ws_encoder.h"

#include "ws_cc.h"
#include "ws_pid.h"
#include "ws_trig.h"


#ifndef USE_CMU_CAMERA
#include "serial_ports.h"
#endif

#define DEBUG_CAM_TIMEOUTS 0

extern unsigned char aBreakerWasTripped;
extern UINT8 lift_mode;
extern UINT8 tilt_mode;
extern UINT16 g_lift_encoder_val_prev[];
extern UINT8 g_lift_encoder_val_prev_idx;


/*******************************************************************************
* FUNCTION NAME: User_Initialization
* PURPOSE:       This routine is called first (and only once) in the Main function.
*                You may modify and add to this function.
* CALLED FROM:   main.c
* ARGUMENTS:     none
* RETURNS:       void
*******************************************************************************/
void User_Initialization (void)
{
  Set_Number_of_Analog_Channels(SIXTEEN_ANALOG);    /* DO NOT CHANGE! */

  /* FIRST: Set up the I/O pins you want to use as digital INPUTS. */
  digital_io_01 = digital_io_02 = digital_io_03 = digital_io_04 = INPUT;
  digital_io_05 = digital_io_06 = digital_io_07 = digital_io_08 = INPUT;
  digital_io_09 = digital_io_10 = digital_io_11 = digital_io_12 = INPUT;
  digital_io_13 = digital_io_14 = digital_io_15 = digital_io_16 = INPUT;

  digital_io_17 = digital_io_18 = OUTPUT;
    /*
     Note: digital_io_01 = digital_io_02 = ... digital_io_04 = INPUT;
           is the same as the following:

           digital_io_01 = INPUT;
           digital_io_02 = INPUT;
           ...
           digital_io_04 = INPUT;
    */

  /* SECOND: Set up the I/O pins you want to use as digital OUTPUTS. */
  /* digital_io_17 = OUTPUT; */

  /* THIRD: Initialize the values on the digital outputs. */
  /* rc_dig_out17 = 0; */

  /* FOURTH: Set your initial PWM values.  Neutral is 127. */
  pwm01 = pwm02 = pwm03 = pwm04 = pwm05 = pwm06 = pwm07 = pwm08 = 127;
  pwm09 = pwm10 = pwm11 = pwm12 = pwm13 = pwm14 = pwm15 = pwm16 = 127;

  /* FIFTH: Set your PWM output types for PWM OUTPUTS 13-16. */
  /*   Choose from these parameters for PWM 13-16 respectively:               */
  /*     IFI_PWM  - Standard IFI PWM output generated with Generate_Pwms(...) */
  /*     USER_CCP - User can use PWM pin as digital I/O or CCP pin.           */
  Setup_PWM_Output_Type(IFI_PWM,IFI_PWM,IFI_PWM,IFI_PWM);

  /* Example: The following would generate a 40KHz PWM with a 50% duty cycle on the CCP2 pin:

         CCP2CON = 0x3C;
         PR2 = 0xF9;
         CCPR2L = 0x7F;
         T2CON = 0;
         T2CONbits.TMR2ON = 1;

         Setup_PWM_Output_Type(USER_CCP,IFI_PWM,IFI_PWM,IFI_PWM);
  */

  /* Add any other initialization code here. */

  Putdata(&txdata);             /* DO NOT CHANGE! */

#ifdef USE_CMU_CAMERA
  Serial_Driver_Initialize();
#else
  Init_Serial_Port_One();
  Init_Serial_Port_Two();
#endif

  initialize_encoders();

  printf("IFI 2005 User Processor Initialized ...\r");
  /* Note:  use a '\r' rather than a '\n' with the new compiler (v2.4) */

  User_Proc_Is_Ready(); /* DO NOT CHANGE! - last line of User_Initialization */
}

/*******************************************************************************
* FUNCTION NAME: Process_Data_From_Master_uP
* PURPOSE:       Executes every 26.2ms when it gets new data from the master
*                microprocessor.
* CALLED FROM:   main.c
* ARGUMENTS:     none
* RETURNS:       void
*******************************************************************************/
void Process_Data_From_Master_uP()
{
  static UINT8 encoder_debug_state = ENCODER_DEBUG_NONE;
  UINT8        ret_val;
  DistHdgType  curr_dist_hdg;
  EncoderValsType debug_encoder_vals;

  UINT8        hdg_brads;
  INT16        distance;


  Getdata(&rxdata);   /* Get fresh data from the master microprocessor. */

  io_print_oi_inputs();
  io_print_rc_inputs();


  if (Oi_calibrate < OI_CALIBRATE_ENCODERS)
  {
    if ((Oi_sw_lift_debug == 1) && (Oi_sw_lift_debug_prev == 0))
    {
      if (encoder_debug_state == ENCODER_DEBUG_LIFT)
      {
        encoder_debug_state = ENCODER_DEBUG_NONE;
      }
      else
      {
        encoder_debug_state = ENCODER_DEBUG_LIFT;
      }
    }
    else if ((Oi_sw_tilt_debug == 1) && (Oi_sw_tilt_debug_prev == 0))
    {
      if (encoder_debug_state == ENCODER_DEBUG_TILT)
      {
        encoder_debug_state = ENCODER_DEBUG_NONE;
      }
      else
      {
        encoder_debug_state = ENCODER_DEBUG_TILT;
      }
    }
  }

  if (encoder_debug_state == ENCODER_DEBUG_LIFT)
  {
    printf("lift encoder %u lift switch %u\r", get_lift_encoder_count(),Dig_in_lift_bottom);
  }
  else if (encoder_debug_state == ENCODER_DEBUG_TILT)
  {
    printf("tilt encoder %u\r", get_tilt_encoder_count());
  }


  if (autonomous_mode != AUTO_ENABLED)
  {
    g_lift_encoder_val_prev_idx =
      (g_lift_encoder_val_prev_idx + 1) % LIFT_ENCODER_BUFFER_SZE;
    g_lift_encoder_val_prev[g_lift_encoder_val_prev_idx] =
      get_lift_encoder_count();

    if ((Oi_calibrate < OI_CALIBRATE_ENCODERS) && (user_display_mode == 0))
    {

      if ((rxdata.packet_num % 8) < 4)
      {
        display_oi_data(255, DISPLAY_DATA_CALIBRATE);
      }
      else
      {
        display_oi_data(0, DISPLAY_DATA_CALIBRATE);
      }


      /* allow us to talk to CC in non-autonomous */
      if (Oi_sw_vision_enable == 1)
      {
#if DEBUG_CAM_TIMEOUTS
        if ((rxdata.packet_num % 10) == 0)
        {
          TetraPosType tetra_pos;
          /* request starting tetra positions */
          cc_get_tetra_pos(&tetra_pos);

          /* choose tetra to track */
          tetra_chooser(tetra_pos.tetra1, tetra_pos.tetra2);

          printf("tetras %d %d\r", tetra_pos.tetra1, tetra_pos.tetra2);
        }
#else
        ret_val = cc_get_dist_hdg(&curr_dist_hdg, CC_CMD_REQ_DIST_HDG);

        if (ret_val == CC_SUCCESS)
        {
          printf("h %3u d %3u o %u", curr_dist_hdg.hdg, curr_dist_hdg.dist,
                 curr_dist_hdg.orient);
        }

        printf("\r");
#endif
      }
#if DEBUG_CAM_TIMEOUTS
      else
      {
        if(Oi_sw_vision_enable_prev == 1)
        {
           UINT8 res;
           unsigned char  ack_data[CC_RESP_TETRA_CHOICE_SIZE];
           UINT8 object_id = 4;
           res = readCcReg(&(ack_data[0]), CC_CMD_SET_TRACK_OBJECT, object_id,
                  CC_RESP_TETRA_CHOICE_SIZE, CC_LOOP_CNT_TIMEOUT);

           if (res < CC_RESP_TETRA_CHOICE_SIZE)
           {
             /* No response from the CC */
             printf("CC TO3 %d %d",object_id,res);
           }
           else
           {
             printf("CC GOOD %d %d",object_id,res);
           }
           printf("\r");
         }
      }
#endif

      if (Oi_sw_encoder_debug == 1)
      {
        ret_val = cc_get_encoder_vals(&debug_encoder_vals);

        if (ret_val == CC_SUCCESS)
        {
          printf("Left: %04d Right: %04d Orient %03d\r",
                 debug_encoder_vals.left, debug_encoder_vals.right,
                 debug_encoder_vals.orient);
        }
      }

      if ((Oi_sw_center_camera == 1) && (Oi_sw_center_camera_prev == 0))
      {
        cc_set_camera_pos(CC_CMD_SET_CAMERA_CENTER);
      }

      if ((Oi_sw_9090_camera == 1) && (Oi_sw_9090_camera_prev == 0))
      {
        cc_set_camera_pos(CC_CMD_SET_CAMERA_9090);
      }
    }
    else
    {
      if ((Oi_calibrate > OI_CALIBRATE_JOYSTICKS) ||
          (Oi_calibrate < OI_CALIBRATE_ENCODERS))
      {
        display_calibration();
      }

      /* scale lift stick x */
      joystick_scaling(&Oi_lift_tilt,
                       LIFT_STICK_X_MIN,
                       LIFT_STICK_X_MIDDLE,
                       LIFT_STICK_X_MAX);

      /* scale lift stick y */
      joystick_scaling(&Oi_lift_height,
                       LIFT_STICK_Y_MIN,
                       LIFT_STICK_Y_MIDDLE,
                       LIFT_STICK_Y_MAX);

      /* Scale drive stick X */
      joystick_scaling(&Oi_drive_x,
                       DRIVE_STICK_X_MIN,
                       DRIVE_STICK_X_MIDDLE,
                       DRIVE_STICK_X_MAX);

      /* Scale drive stick Y */
      joystick_scaling(&Oi_drive_y,
                       DRIVE_STICK_Y_MIN,
                       DRIVE_STICK_Y_MIDDLE,
                       DRIVE_STICK_Y_MAX);

      drive_stick_input(TRUE);
      pump_control();
      wing_control();

      lift_oi_input();
      spear_control();
      tilt_oi_input();
    }

    if (disabled_mode == ROBOT_DISABLED)
    {
      /* reset pneumatics to default state so they don't jump out when robot
         is enabled */
      motor_vals.fwing = WING_IN;
      motor_vals.bwing = WING_IN;
      motor_vals.top_spear_grabber = SPEAR_GRAB;
      motor_vals.top_spear_tilt = SPEAR_NO_TILT;
      motor_vals.top_spear_retract = SPEAR_IN;
      motor_vals.bot_spear_grabber = SPEAR_GRAB;
      motor_vals.bot_spear_tilt = SPEAR_NO_TILT;
    }

    assign_outputs_fast();
    Generate_Pwms(pwm13,pwm14,pwm15,pwm16);


#if 0
    /* Eample code to check if a breaker was ever tripped. */
    if (aBreakerWasTripped)
    {
      for (i=1;i<29;i++)
      {
        if (Breaker_Tripped(i))
          User_Byte1 = i;  /* Update the last breaker tripped on User_Byte1 (to demonstrate the use of a user byte)
                              Normally, you do something else if a breaker got tripped (ex: limit a PWM output)     */
      }
    }
#endif
    display_oi_data(((lift_mode == LIFT_OVERRIDE_MODE) <<
                     LIFT_OVERRIDE_LED_POS) |
                    ((tilt_mode == TILT_OVERRIDE_MODE) <<
                     TILT_OVERRIDE_LED_POS), DISPLAY_DATA_OVERRIDE);

    assign_outputs_slow();
    Putdata(&txdata);             /* DO NOT CHANGE! */
  }
}


