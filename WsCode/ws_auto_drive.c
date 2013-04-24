/*******************************************************************************
* FILE NAME: ws_auto_drive.c
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
#include "ws_autonomous.h"
#include "ws_cc.h"
#include "ws_pid.h"
#include "ws_drive_input.h"
#include "ws_trig.h"
#include "ws_auto_drive.h"


extern UINT8 show_pid;
extern PidValsType g_turn_orient_pid_vals;
extern PidValsType g_turn_drive_pid_vals;
extern PidValsType g_drive_speed_pid_vals;
extern INT16 g_autodrive_ticks;

static UINT8 success_counter = 0;

/*******************************************************************************
* FUNCTION NAME: turn_to_orient
* PURPOSE:       turn robot to specified orientation
* CALLED FROM:
* ARGUMENTS:
* RETURNS:
*******************************************************************************/
UINT8 turn_to_orient(UINT8 orient_target)
{
  UINT8        cc_ret_val;
  EncoderValsType encoder_vals_cur;
  INT8         hdg_delta;
  UINT8        ret = DRIVE_TASK_NOT_DONE;
  INT8         orient_delta;

  cc_ret_val = cc_get_encoder_vals(&encoder_vals_cur);

  if (cc_ret_val == CC_SUCCESS)
  {
    printf("  Ot %u Oc %u  ", orient_target, encoder_vals_cur.orient);

    Oi_drive_y = 127;

    /* need to take care of wrap around from 0 <--> 255 */
    hdg_delta = (INT8)encoder_vals_cur.orient - (INT8)orient_target;

    Oi_drive_x = 127 + ws_pid(&g_turn_orient_pid_vals, hdg_delta, 0);
  }
  else
  {
    Oi_drive_y = 127;
    Oi_drive_x = 127;
  }

  printf("turn x %d y %d\r", (int)Oi_drive_x, (int)Oi_drive_y);

  Oi_sw_turbo = 1;
  Oi_sw_shifter = SHIFT_SWITCH_LOW;
  drive_stick_input(FALSE);

  orient_delta = (INT8)(encoder_vals_cur.orient - orient_target);

  if ((cc_ret_val == CC_SUCCESS) &&
      (orient_delta <= 1) && (orient_delta >= -1))
  {
    ret = DRIVE_TASK_DONE;
  }

  return ret;
}



/*******************************************************************************
* FUNCTION NAME: drive_to_approach
* PURPOSE:       drive robot to approach point
* CALLED FROM:
* ARGUMENTS:
* RETURNS:
*******************************************************************************/
UINT8 drive_to_approach(INT8 approach_point_x, INT8 approach_point_y,
                        UINT8 target_distance, UINT8 shifter_pos, UINT8 cmd)
{
  static UINT8 turn_hyst = DRIVE_TO_OBJ;
  UINT8        cc_ret_val;
  DistHdgType  curr_dist_hdg;
  UINT8        beta;
  UINT8        heading;
  INT16        opposite, adjacent;
  INT16        distance;
  INT8         hdg_delta;
  UINT8        ret = DRIVE_TASK_NOT_DONE;


  /*
   * Refer to this diagram when reading the following code.
   *
   * Quadrants on the field compass-wise.
   *
   *            (positive y)
   *
   *                 63
   *         Quad 2  |   Quad 1
   *                 |
   *                 |
   *        127 ----------- 0 / 255  (positive x)
   *                 |
   *                 |
   *         Quad 3  |   Quad 4
   *                191
   */

  cc_ret_val = cc_get_dist_hdg(&curr_dist_hdg, cmd);

  if (cc_ret_val == CC_SUCCESS)
  {
    printf(" h %3u d %3u o %u  ", curr_dist_hdg.hdg, curr_dist_hdg.dist,
           curr_dist_hdg.orient);

    /* beta = camera angle + robot orientation */
    beta = curr_dist_hdg.hdg + curr_dist_hdg.orient;

    /* calculate angle to approach point */
    /* angle = arctan((dist * sin(beta) + APPROACH) /
       (dist * cos(beta) + PICKUP)) */
    opposite = (((INT32)curr_dist_hdg.dist * sin(beta)) / 256) +
               approach_point_y;
    adjacent = (((INT32)curr_dist_hdg.dist * cos(beta)) / 256) -
               approach_point_x;

    if (opposite >= 0)
    {
      if (adjacent >= 0)
      {
        /* quadrent 1
         * opposite is positive
         * adjacent is positive
         */
        heading = arctan16(opposite, adjacent);
      }
      else
      {
        /* quadrent 2
         * opposite is positive
         * adjacent is negative
         */
        heading = 127 - arctan16(opposite, -(adjacent));
      }
    }
    else
    {
      if (adjacent >= 0)
      {
        /* quadrent 4
         * opposite is negative
         * adjacent is positive
         */
        heading = 255 - arctan16(-(opposite), adjacent);
      }
      else
      {
        /* quadrent 3
         * opposite is negative
         * adjacent is negative
         */
        heading = 127 + arctan16(-(opposite), -(adjacent));
      }
    }

    /* calculate distance to approach point */
    /* if opposite is 0, we don't have a triangle and the distance is
       simply the adjacent component */
    if (sin(heading) != 0)
    {
      /* distance = (dist * sin(beta) + APPROACH) / sin(heading) */
      distance = (((((INT32)curr_dist_hdg.dist * sin(beta)) / 256) +
                   approach_point_y) * 256) / (sin(heading));
    }
    else
    {
      distance = adjacent;
    }

#if 1
    printf(" cb %d sb %d sh %d opp %d adj %d hdg %d dst %d ",
           (int)cos(beta), (int)sin(beta), (int)sin(heading),
           (int)opposite, (int)adjacent, (int)heading, (int)distance);
#endif

    /* need to take care of wrap around from 0 <--> 255 */
    hdg_delta = (INT8)curr_dist_hdg.orient - (INT8)heading;

    /*
    printf("dlt %d ", hdg_delta);
    */

    if ((distance < 20) && (distance > -20))
    {
      /* always drive forward when close to object */
      turn_hyst = DRIVE_TO_OBJ;
    }
    else if ((hdg_delta > 17) || (hdg_delta < -17))
    {
      /* reorient when far away */
      turn_hyst = TURN_TO_OBJ;
    }
    else if ((hdg_delta < 14) && (hdg_delta > -14))
    {
      /* start driving again when closer, but add in some hysteresis */
      turn_hyst = DRIVE_TO_OBJ;
    }

    if (turn_hyst == DRIVE_TO_OBJ)
    {
      Oi_drive_y = 127 - ws_pid(&g_drive_speed_pid_vals, distance, 0);
      Oi_drive_x = 127 + ws_pid(&g_turn_drive_pid_vals, hdg_delta, 0);
      Oi_sw_shifter = SHIFT_SWITCH_HIGH;
    }
    else
    {
      Oi_drive_y = 127;
      Oi_drive_x = 127 + ws_pid(&g_turn_orient_pid_vals, hdg_delta, 0);

      if ((hdg_delta > 25) || (hdg_delta < -25))
      {
        Oi_sw_shifter = SHIFT_SWITCH_LOW;
      }
      else
      {
        Oi_sw_shifter = shifter_pos;
      }
    }
  }
  else
  {
    Oi_drive_y = 127;
    Oi_drive_x = 127;
  }

#if 1
  printf("x %d y %d", (int)Oi_drive_x, (int)Oi_drive_y);
  printf("\r");
#endif

  Oi_sw_turbo = 1;
  drive_stick_input(FALSE);

  if (distance < 0)
  {
    distance = -distance;
  }

  if ((cc_ret_val == CC_SUCCESS) && (distance <= target_distance))
  {
    success_counter++;

    if (success_counter >= 2)
    {
      ret = DRIVE_TASK_DONE;
      success_counter = 0;
    }
  }

  return ret;
}



/*******************************************************************************
* FUNCTION NAME: drive_next_to_object
* PURPOSE:       drive robot next to object
* CALLED FROM:
* ARGUMENTS:
* RETURNS:
*******************************************************************************/
UINT8 drive_next_to_object(UINT8 pickup_distance, UINT8 target_distance)
{
  UINT8        ret = DRIVE_TASK_NOT_DONE;
  UINT8        cc_ret_val;
  DistHdgType  curr_dist_hdg;
  INT16        distance;


  cc_ret_val = cc_get_dist_hdg(&curr_dist_hdg, CC_CMD_REQ_DIST_HDG);

  if (cc_ret_val == CC_SUCCESS)
  {
#if 1
    printf(" h %3u d %3u o %u  ", curr_dist_hdg.hdg, curr_dist_hdg.dist,
           curr_dist_hdg.orient);
#endif
    show_pid = 0; // Use to debug Y PID values
    Oi_drive_y = 127 - ws_pid(&g_drive_speed_pid_vals, curr_dist_hdg.dist,
                              pickup_distance);

    /* calculate path's perpendicular distance from tetra */
    distance = ((INT16)curr_dist_hdg.dist *
                (INT32)sin(curr_dist_hdg.hdg)) / 256;

    show_pid = 0; // Use to debug X PID values
    /* control Oi_drive_x using PID loop on distance */
    Oi_drive_x = 127 - ws_pid(&g_turn_drive_pid_vals, distance,
                              -(INT16)pickup_distance);

#if 1
    printf("sin %d dist %d ", (int)sin(curr_dist_hdg.hdg), (int)distance);
#endif

  }
  else
  {
    Oi_drive_y = 127;
    Oi_drive_x = 127;
  }

#if 1
  printf("drive x %d y %d  ", (int)Oi_drive_x, (int)Oi_drive_y);
  printf("\r");
#endif

  Oi_sw_turbo = 1;
  Oi_sw_shifter = SHIFT_SWITCH_LOW;
  drive_stick_input(FALSE);

  if ((cc_ret_val == CC_SUCCESS) && (curr_dist_hdg.dist <= target_distance))
  {
    success_counter++;

    if (success_counter >= 2)
    {
      ret = DRIVE_TASK_DONE;
      success_counter = 0;
    }
  }

  return ret;
}


/*******************************************************************************
* FUNCTION NAME: drive_past_object
* PURPOSE:       drive at constant heading until object is at correct angle
* CALLED FROM:
* ARGUMENTS:
* RETURNS:
*******************************************************************************/
UINT8 drive_past_object(UINT8 desired_orient, UINT8 drive_speed,
                        UINT8 target_hdg)
{
  UINT8        ret = DRIVE_TASK_NOT_DONE;
  UINT8        cc_ret_val;
  DistHdgType  curr_dist_hdg;
  INT8         hdg_delta;


  cc_ret_val = cc_get_dist_hdg(&curr_dist_hdg, CC_CMD_REQ_DIST_HDG);

  if (cc_ret_val == CC_SUCCESS)
  {
#if 1
    printf(" h %3u d %3u o %u  ", curr_dist_hdg.hdg, curr_dist_hdg.dist,
           curr_dist_hdg.orient);
#endif

    /* need to take care of wrap around from 0 <--> 255 */
    hdg_delta = (INT8)curr_dist_hdg.orient - (INT8)desired_orient;

    Oi_drive_x = 127 + ws_pid(&g_turn_drive_pid_vals, hdg_delta, 0);
    Oi_drive_y = drive_speed;
  }
  else
  {
    Oi_drive_y = 127;
    Oi_drive_x = 127;
  }

  Oi_sw_shifter = SHIFT_SWITCH_LOW;
  drive_stick_input(FALSE);

  printf("side hdg %d x %d y %d\r", curr_dist_hdg.hdg, Oi_drive_x, Oi_drive_y);

  /* move to next state when tetra is next to robot */
  if ((cc_ret_val == CC_SUCCESS) && (curr_dist_hdg.hdg <= target_hdg))
  {
    success_counter++;

    if (success_counter >= 2)
    {
      ret = DRIVE_TASK_DONE;
      success_counter = 0;
    }
  }

  return ret;
}





UINT8 drive_to_distance(INT16 desired_ticks, UINT8 speed,UINT8 shifter)
{
  UINT8        ret = DRIVE_TASK_NOT_DONE;
  UINT8        cc_ret_val;
  EncoderValsType encoder_vals;

  cc_ret_val = cc_get_encoder_vals(&encoder_vals);

  printf("dist d %d c %d\r", desired_ticks, g_autodrive_ticks);

  if(cc_ret_val == CC_SUCCESS)
  {
    g_autodrive_ticks += encoder_vals.right;
#if 1
    if(((desired_ticks > 0) && (g_autodrive_ticks >= desired_ticks)) ||
        ((desired_ticks < 0) && (g_autodrive_ticks <= desired_ticks)))
    {
      /* Stop the motors from moving */
      motor_vals.left_drive = 0;
      motor_vals.right_drive = 0;
      ret = DRIVE_TASK_DONE;
    }
    else
    {
      Oi_drive_y = 127+speed;
      Oi_drive_x = 127;
      Oi_sw_shifter = shifter;
      drive_stick_input(FALSE);
    }
#else
    if(shifter == SHIFT_SWITCH_HIGH)
    {
      SET_SPEED_PID_DRIVE_DIST();
    }
    else
    {
      SET_SPEED_PID_DRIVE_DIST_LOW_GEAR();
    }
    show_pid = 1;
    Oi_drive_y = 127 + ws_pid(&g_drive_speed_pid_vals,
                              g_autodrive_ticks,
                              desired_ticks);
    show_pid = 0;

    Oi_drive_x = 127;

    Oi_sw_shifter = shifter;

    printf("d %d x %d\r", Oi_drive_y,Oi_drive_x);
    drive_stick_input(FALSE);

    if(((desired_ticks > 0) && (g_autodrive_ticks >= desired_ticks)) ||
       ((desired_ticks < 0) && (g_autodrive_ticks <= desired_ticks)))
    {
      motor_vals.left_drive = 0;
      motor_vals.right_drive = 0;
      ret = DRIVE_TASK_DONE;
    }
#endif
  }
  else
  {
    ret = DRIVE_TASK_CC_FAIL;
  }
  return ret;
}


