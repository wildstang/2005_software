/*******************************************************************************
* FILE NAME: ws_calibrate.c
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

#include "ws_calibrate.h"
#include "ws_io.h"
#include "ws_general.h"
#include "ws_encoder.h"
#include "eeprom.h"


/*******************************************************************************
* FUNCTION NAME: calibrate_pots
* PURPOSE:       store calibration values for crab & arms in EEPROM
* ARGUMENTS:     none
* RETURNS:       none
*
* To calibrate a pot position:
*   - set the pot to the desired position
*   - enable pot calibration mode
*
*******************************************************************************/
#if 0
void calibrate_pots(void)
{
  static UINT8 eeprom_init_check_flag = FALSE;
  UINT8 bitmask;

#ifdef CALIBRATION_PRINTF
  printf("calibrate func  ");
#endif

  /* only check once if EEPROM has been initialized */
  if (eeprom_init_check_flag == FALSE)
  {
    /* only initialize EEPROM if the 'known' bits are not correct */
    if ((readEE(ADDR_KNOWN_BYTE1) != CALIBRATE_KNOWN_BYTE_1) ||
        (readEE(ADDR_KNOWN_BYTE2) != CALIBRATE_KNOWN_BYTE_2))
    {
#ifdef CALIBRATION_PRINTF
      printf("initializing EEPROM  ");
#endif
      /* initialize bitmasks to 0 */
      writeEE(ADDR_DATA_BITMASK_LIFT_HEIGHT, 0);
      writeEE(ADDR_DATA_BITMASK_LIFT_TILT, 0);
      writeEE(ADDR_DATA_BITMASK3, 0);
      writeEE(ADDR_DATA_BITMASK4, 0);

      /* set 'known' bytes */
      writeEE(ADDR_KNOWN_BYTE1, CALIBRATE_KNOWN_BYTE_1);
      writeEE(ADDR_KNOWN_BYTE2, CALIBRATE_KNOWN_BYTE_2);
    }

    eeprom_init_check_flag = TRUE;
  }

  /* calibrate lift height */
  if (0)
  {
    bitmask = readEE(ADDR_DATA_BITMASK_LIFT_HEIGHT);
  }

  /* calibrate lift tilt */
  if (0)
  {
    bitmask = readEE(ADDR_DATA_BITMASK_LIFT_TILT);
  }

#ifdef CALIBRATION_PRINTF
  printf("\n");
#endif

  return;
}
#endif



/*******************************************************************************
* FUNCTION NAME: retrieve_calibration
* PURPOSE:       retrieve calibration values for crab & arms from EEPROM
* ARGUMENTS:     none
* RETURNS:       none
*******************************************************************************/
#if 0
void retrieve_calibration()
{
  UINT8 verify1, verify2;
  UINT8 bitmask;

  verify1 = readEE(ADDR_KNOWN_BYTE1);
  verify2 = readEE(ADDR_KNOWN_BYTE2);

  if ((verify1 == CALIBRATE_KNOWN_BYTE_1) &&
      (verify2 == CALIBRATE_KNOWN_BYTE_2))
  {
    /* the bitmask bytes can be trusted, so check each bit */
    bitmask = readEE(ADDR_DATA_BITMASK_LIFT_HEIGHT);

    /* lift height bottom */

    /* lift height top */

    /* lift height load lo */

    /* lift height load hi */

    /* lift height score */


    /* lift tilt left */

    /* lift tilt vertical */
  }
  else
  {
    /* use default values */
  }

  return;
}
#endif



/*******************************************************************************
* FUNCTION NAME: display_calibration
* PURPOSE:       display joystick vals & pot values for crab & arms on OI
* ARGUMENTS:     none
* RETURNS:       none
*
* To display pot values on the OI:
*   - enable calibration mode
*   - move the joystick axis that corresponds to the pot (crab stick X,
*     big arm Y, big arm X, small arm X) full forward or left (depending on
*     the axis) and allow it to return to the middle
*   - now the OI displays the pot value
*
* To display joystick values on the OI:
*   - enable calibration mode
*   - enable manual crab mode
*   - move the joystick axis that you want to display full forward or left
*     (depending on the axis) and allow it to return to the middle
*   - now the OI displays the X/Y axis of the stick
*
*******************************************************************************/
void display_calibration(void)
{
  static DISPLAY_TYPE display_type = DISPLAY_NONE;
  UINT8 display_data = 0;

  if (Oi_calibrate > OI_CALIBRATE_JOYSTICKS)
  {
    /* print joystick values when switch is to joystick side */
    if (Oi_drive_x >= (127 + CALIBRATE_ZONE))
    {
      display_type = DISPLAY_DRIVE_X;
    }
    else if (Oi_drive_y >= (127 + CALIBRATE_ZONE))
    {
      display_type = DISPLAY_DRIVE_Y;
    }
    else if (Oi_lift_height >= (127 + CALIBRATE_ZONE))
    {
      display_type = DISPLAY_LIFT_Y;
    }
    else if (Oi_lift_tilt >= (127 + CALIBRATE_ZONE))
    {
      display_type = DISPLAY_LIFT_X;
    }
  }
  else
  {
    /* print encoder values when switch is to calibrate encoder side */
    if (Oi_lift_height >= (127 + CALIBRATE_ZONE))
    {
      display_type = DISPLAY_LIFT_ENCODER;
    }
    else if (Oi_lift_tilt >= (127 + CALIBRATE_ZONE))
    {
      display_type = DISPLAY_TILT_ENCODER;
    }
  }

  switch (display_type)
  {
    case DISPLAY_DRIVE_X:
      display_data = Oi_drive_x;
      break;

    case DISPLAY_DRIVE_Y:
      display_data = Oi_drive_y;
      break;

    case DISPLAY_LIFT_Y:
      display_data = Oi_lift_height;
      break;

    case DISPLAY_LIFT_X:
      display_data = Oi_lift_tilt;
      break;

    case DISPLAY_LIFT_ENCODER:
      display_data = (UINT8)(get_lift_encoder_count() & 0x00FF);
      printf("lift %d %d\r", get_lift_encoder_count(), display_data);
      break;

#if USING_TILT_ENCODER
    case DISPLAY_TILT_ENCODER:
      display_data = (UINT8)(get_tilt_encoder_count() & 0x00FF);
      break;
#endif

    default:
      break;
  }

  display_oi_data(display_data, DISPLAY_DATA_CALIBRATE);

  return;
}


