/*******************************************************************************
* FILE NAME: ws_auto_misc.c
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
#include "ws_atask_misc.h"

UINT8 auto_task_set_wings(void *params)
{
  UINT8 bitmap = ((BitmapParamType *)params)->bitmap;
  printf("SET WINGS %d\r",bitmap);
  motor_vals.fwing = (bitmap & ATASK_FWING_MASK) >> ATASK_FWING_ORDER;
  motor_vals.bwing = (bitmap & ATASK_BWING_MASK) >> ATASK_BWING_ORDER;
  return TASK_STATE_DONE;
}

UINT8 auto_task_set_top_spear(void *params)
{
  UINT8 bitmap = ((BitmapParamType *)params)->bitmap;
  printf("SET TOP SPEAR %d\r",bitmap);
  motor_vals.top_spear_retract = (bitmap & ATASK_SPEAR_RETRACT_MASK) >> ATASK_SPEAR_RETRACT_ORDER;
  motor_vals.top_spear_tilt = (bitmap & ATASK_SPEAR_TILT_MASK) >> ATASK_SPEAR_TILT_ORDER;
  motor_vals.top_spear_grabber = (bitmap & ATASK_SPEAR_GRABBER_MASK) >> ATASK_SPEAR_GRAB_ORDER;
  return TASK_STATE_DONE;
}

UINT8 auto_task_set_bot_spear(void *params)
{
  UINT8 bitmap = ((BitmapParamType *)params)->bitmap;
  printf("SET BOT SPEAR %d >> ",bitmap);
  motor_vals.bot_spear_retract = (bitmap & ATASK_SPEAR_RETRACT_MASK) >> ATASK_SPEAR_RETRACT_ORDER;
  motor_vals.bot_spear_tilt = (bitmap & ATASK_SPEAR_TILT_MASK) >> ATASK_SPEAR_TILT_ORDER;
  motor_vals.bot_spear_grabber = (bitmap & ATASK_SPEAR_GRABBER_MASK) >> ATASK_SPEAR_GRAB_ORDER;
  printf(" %d %d %d\r",motor_vals.bot_spear_retract,
                      motor_vals.bot_spear_tilt,
                      motor_vals.bot_spear_grabber);
  return TASK_STATE_DONE;
}

UINT8 auto_task_set_lift_height(void *params)
{
  UINT8 ret_state = TASK_STATE_PROCESSING;
  UINT16 height = ((EncoderPosParamType *)params)->encoder_val;
  UINT8 wait_for_feedback = ((EncoderPosParamType *)params)->wait_for_feedback;
  UINT16 pos;

#if USE_LIFT
  lift_set_height(height);
  if(wait_for_feedback)
  {
    pos = get_lift_encoder_count();
    if(pos <= height)
    {
      ret_state = TASK_STATE_DONE;
    }
  }
  else
  {
    ret_state = TASK_STATE_DONE;
  }
#endif
  return ret_state;
}

UINT8 auto_task_set_tilt_pos(void *params)
{
  UINT8 ret_state = TASK_STATE_PROCESSING;
  UINT16 tilt = ((EncoderPosParamType *)params)->encoder_val;
  UINT8 wait_for_feedback = ((EncoderPosParamType *)params)->wait_for_feedback;
  UINT16 pos;

#if USE_TILT
  tilt_set_pos(tilt);
  if(wait_for_feedback)
  {
    // Need to use tilt feedback to know when
    // we're there
    ret_state = TASK_STATE_DONE;
  }
  else
  {
    ret_state = TASK_STATE_DONE;
  }
#endif
  return ret_state;
}

