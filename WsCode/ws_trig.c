/*******************************************************************************
* FILE NAME: ws_trig.c
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
#include "ws_trig.h"


const rom UINT8 arctan_lookup[65] =
     { 0,  1,  1,  2,  3,  3,  4,  4,  5,  6,
       6,  7,  8,  8,  9,  9, 10, 11, 11, 12,
      12, 13, 13, 14, 15, 15, 16, 16, 17, 17,
      18, 18, 19, 19, 20, 20, 21, 21, 22, 22,
      23, 23, 24, 24, 25, 25, 25, 26, 26, 27,
      27, 27, 28, 28, 29, 29, 29, 30, 30, 30,
      31, 31, 31, 32, 32};

const rom UINT8 cos_lookup[65] =
     { 255, 255, 255, 255, 255, 254, 254, 252, 251, 250,
       248, 247, 245, 243, 241, 239, 237, 234, 231, 229,
       226, 223, 220, 216, 213, 209, 206, 202, 198, 194,
       190, 185, 181, 177, 172, 167, 162, 157, 152, 147,
       142, 137, 132, 126, 121, 115, 109, 104,  98,  92,
        86,  80,  74,  68,  62,  56,  50,  44,  38,  31,
        25,  19,  13,   6,   0};


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
 *
 */


/*******************************************************************************
* FUNCTION NAME: arctan16
* PURPOSE:       return arctan of opposite / adjecent pair
* CALLED FROM:   two_axis_crab_steering()
* ARGUMENTS:     opposite - opposite side of triangle
*                adjacent - adjacent side of triangle
* RETURNS:       arctan of opposite / adjacent pair
*******************************************************************************/
UINT8 arctan16(UINT16 opposite, UINT16 adjacent)
{
  /* arctan takes two UINT8s, so we need to shift down our diffs to below 255.
   * To do this, we will just keep shifting off a bit until we get to two nums
   * below 255.  This is only really needed when our target point is greater
   * than 255 away...that is quite far.
   */
  while((opposite>255) || (adjacent>255))
  {
     /* Shift off a bit to divide the number by 2 */
     opposite = opposite >> 1;
     adjacent = adjacent >> 1;
  }

  /* Now that opp,adj are under 255 call the UINT8 arctan */
  return(arctan((UINT8)opposite, (UINT8)adjacent));
}

/*******************************************************************************
* FUNCTION NAME: arctan
* PURPOSE:       return arctan of opposite / adjecent pair
* CALLED FROM:
* ARGUMENTS:     opposite - opposite side of triangle
*                adjacent - adjacent side of triangle
* RETURNS:       arctan of opposite / adjacent pair
*******************************************************************************/
UINT8 arctan(UINT8 opposite, UINT8 adjacent)
{
  UINT8 arctan_val;
  UINT8 tmp;
  UINT8 swap_angle_flag;

  if ((opposite == 0) && (adjacent == 0))
  {
    /* passed in opposite & adjacent are both zero...should not happen but keep
       from a divide by zero and just return angle of zero */
    arctan_val = 0;
  }
  else
  {
    /* We always want our opposite to be smaller than our adjacent so we only
       have to deal with 45 degrees or smaller.  If the passed in opposite is
       larger, we simply reverse them and calculate the other angle prior to
       output. */
    if (opposite < adjacent)
    {
      /* passed in opposite is smaller than adjacent...perfect! */
      swap_angle_flag = FALSE;
    }
    else
    {
      /* passed in opposite is larger than adjacent...swap it and set the swap
         angle flag so we will calculate the exact angle prior to output */
      tmp = opposite;
      opposite = adjacent;
      adjacent = tmp;
      swap_angle_flag = TRUE;
    }

    /* this is a TAN lookup table that will convert (opposite/adjacent)*64 into
       an angle from 0-32 brads (45 degrees).  Since the angle must be less
       than 45 degrees, the opposite must be less than the adjacent side.
       NOTE: (a << 6) is the same as (a * 64), but the shifting is much faster
       in hardware */

    arctan_val = arctan_lookup[((int)opposite << 6) / (int)adjacent];

    /* if we swapped the opposite & adjacent then we have to convert to the
       other angle in the triange.  This is simple, just subtract 90 degrees
       (60 brads). */
    if (swap_angle_flag == TRUE)
    {
      /* swap flag was set so calculate the other angle in the triangle */
      arctan_val = 64 - arctan_val;
    }
  }

  return (arctan_val);
}




/*******************************************************************************
* FUNCTION NAME: cos
* PURPOSE:       return 256 * (cos angle)
* CALLED FROM:
* ARGUMENTS:     angle - angle to take cos of
* RETURNS:       256 * (cos of angle)
*******************************************************************************/
INT16 cos(UINT8 angle)
{
  INT16 cos_val;

  if (angle <= 64)
  {
    cos_val = (INT16)cos_lookup[angle];
  }
  else if (angle <= 128)
  {
    angle = 128 - angle;
    cos_val = -(INT16)(cos_lookup[angle]);
  }
  else if (angle <= 192)
  {
    angle = angle - 128;
    cos_val = -(INT16)(cos_lookup[angle]);
  }
  else
  {
    angle = 256 - angle;
    cos_val = (INT16)cos_lookup[angle];
  }

  return (cos_val);
}


/*******************************************************************************
* FUNCTION NAME: sin
* PURPOSE:       return 256 * (sin angle)
* CALLED FROM:
* ARGUMENTS:     angle - angle to take cos of
* RETURNS:       256 * (sin of angle)
*******************************************************************************/
INT16 sin(UINT8 angle)
{
  /* sin(theta) = cos(64brads - theta) */
  return (cos(64 - angle));
}


