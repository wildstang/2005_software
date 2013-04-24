/*******************************************************************************
* FILE NAME: ws_camera.c
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
#include "ws_camera.h"
#include "user_camera.h"

#ifdef USE_CMU_CAMERA

/*******************************************************************************
* FUNCTION NAME: camera_state_machine
* PURPOSE:
* ARGUMENTS:
* RETURNS:       none
*******************************************************************************/
void camera_state_machine(int *tracking, int *pan_position, int *tilt_position)
{
  static UINT8 cam_state = CAM_STATE_BOOT;
  static UINT8 delay;

  switch (cam_state)
  {
    case CAM_STATE_BOOT:
      /* Wait for ~1 sec before talking to camera */
      delay = 38;
      cam_state = CAM_STATE_DELAY;
      break;

    case CAM_STATE_INIT:
      /* Set the 3 exposure values yellow, green and red */
      if (camera_init(64,85,50))
      {
        cam_state = CAM_STATE_AUTO_SERVO;

        /* turn LEDs off */
        Pwm1_red = Pwm2_red = Relay1_red = Relay2_red = 0;
      }
      else
      {
        /* Issue continual retries until camera responds */
        cam_state = CAM_STATE_DELAY;

        /* Wait for ~.5 sec */
        delay = 19;

        /* Flash All Color LED Indicators when camera is not responding */
        Pwm1_red ^= 1;
        Pwm2_red ^= 1;
        Relay1_red ^= 1;
        Relay2_red ^= 1;
      }
      break;

    case CAM_STATE_AUTO_SERVO:
      /* Turn on auto-servo mode and set servo parameters */
      camera_auto_servo(1);
      cam_state = CAM_STATE_PROCESSING;
      break;

    case CAM_STATE_PROCESSING:
      camera_process(tracking, pan_position, tilt_position);
      break;

    case CAM_STATE_DELAY:
      if (delay > 0)
      {
        delay--;
      }
      else
      {
        cam_state = CAM_STATE_INIT;
      }
      break;
  }
}


/*******************************************************************************
* FUNCTION NAME: Camera_Processing
* PURPOSE:       Handles trigger buttons from OI and track updates
* CALLED FROM:   Process_Data_From_Master_uP
* ARGUMENTS:     none
* RETURNS:       void
*******************************************************************************/
void camera_process(int *tracking, int *pan_position, int *tilt_position)
{
  static unsigned char i;
  static UINT8 Oi_sw_vision_prev = 0;
  cam_struct cam;


  if ((Oi_sw_vision == 1) && (Oi_sw_vision_prev == 0))
  {
    /* tell the camera what color to look for when trigger is tapped */
    i = camera_find_color(GREEN);
  }

  Oi_sw_vision_prev = Oi_sw_vision;


  if (camera_track_update(&cam) == 1)
  {
    // Put vision processing here, because we have a good frame!
//    printf("Got T packet %d %d %d %d servo: %d %d\r", cam.x, cam.y, cam.size,
//           cam.conf, cam.pan_servo, cam.tilt_servo);

    if (cam.size > 0)
    {
      //Check to see if camera is tracking
      *tracking = 1;               // if yes then set 'tracking' flag
      *pan_position = cam.pan_servo;
      *tilt_position = cam.tilt_servo;
    }
    else
    {
      *tracking = 0;               // if there is no track, clear the flag
    }
  }
}

#endif

