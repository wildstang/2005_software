/*******************************************************************************
* FILE NAME: main.c <FRC VERSION>
*
* DESCRIPTION:
*  This file contains the main program loop.
*
* USAGE:
*  You should not need to modify this file.
*  Note the different loop speed for the two routines:
*     Process_Data_From_Master_uP
*     Process_Data_From_Local_IO
*******************************************************************************/

#include "ifi_aliases.h"
#include "ifi_default.h"
#include "ifi_utilities.h"
#include "ws_includes.h"
#include "user_routines.h"
#include "ws_autonomous.h"
#include "ws_calibrate.h"
#include "ws_lift.h"
#include "ws_io.h"

tx_data_record txdata;          /* DO NOT CHANGE! */
rx_data_record rxdata;          /* DO NOT CHANGE! */
packed_struct statusflag;       /* DO NOT CHANGE! */
RcButtonsType oi_swA_byte_prev;
RcButtonsType oi_swB_byte_prev;
UINT8         oi_analog14_prev;

MotorValsType       motor_vals;

/*******************************************************************************
* FUNCTION NAME: main
* PURPOSE:       Main program function.
* CALLED FROM:   ifi_startup.c
* ARGUMENTS:     none
* RETURNS:       void
* DO NOT DELETE THIS FUNCTION
*******************************************************************************/
void main (void)
{

#ifdef UNCHANGEABLE_DEFINITION_AREA
  IFI_Initialization ();        /* DO NOT CHANGE! */
#endif

  User_Initialization();        /* You edit this in user_routines.c */

  statusflag.NEW_SPI_DATA = 0;  /* DO NOT CHANGE! */

  /*
  retrieve_calibration();
  */

  /* initialize PID values for lift */
  lift_init();
  /* initialize PID values for tilt and set bottom positon */
  tilt_init();

  motor_vals.fwing = WING_IN;
  motor_vals.bwing = WING_IN;
  motor_vals.wing_lock = WING_UNLOCKED;
  motor_vals.top_spear_grabber = SPEAR_GRAB;
  motor_vals.bot_spear_grabber = SPEAR_GRAB;
  motor_vals.top_spear_tilt = SPEAR_NO_TILT;
  motor_vals.bot_spear_tilt = SPEAR_NO_TILT;
  motor_vals.top_spear_retract = SPEAR_IN;
  motor_vals.bot_spear_retract = SPEAR_IN;
  motor_vals.shifter_position = SHIFTER_HIGH;


  while (1)   /* This loop will repeat indefinitely. */
  {
#ifdef _SIMULATOR
    statusflag.NEW_SPI_DATA = 1;
#endif

    if (statusflag.NEW_SPI_DATA)
    {
      /* 26.2ms loop area */
      /* I'm slow!  I only execute every 26.2ms because */
      /* that's how fast the Master uP gives me data. */

      Process_Data_From_Master_uP();

      if (autonomous_mode == AUTO_DISABLED)
      {
        auto_lock_in();
      }

      /* set auto-lock-in LEDs */
      display_auto_data();

      /* display OI data */

#if FAKE_AUTON
      if (p1_sw_trig == TRUE || autonomous_mode == AUTO_ENABLED)
#else
      if (autonomous_mode == AUTO_ENABLED)
#endif
      {
        auto_main();
      }

      oi_analog14_prev = rxdata.oi_analog14;
      oi_swA_byte_prev = rxdata.oi_swA_byte;
      oi_swB_byte_prev = rxdata.oi_swB_byte;

      /* THE NEXT LINE IS NECESSARY TO PREVENT A COMPILER CORE DUMP */
      Oi_lift_programs_prev = 0;
      /* END NECESSARY LINE */
      Oi_lift_programs_prev = Oi_lift_programs;
    }

    /* You edit this in user_routines_fast.c */
    /* I'm fast!  I execute during every loop.*/
    Process_Data_From_Local_IO();
  }
}


/******************************************************************************/
/******************************************************************************/
/******************************************************************************/
