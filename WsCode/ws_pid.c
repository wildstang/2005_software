/*******************************************************************************
* FILE NAME: ws_pid.c
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
#include "delays.h"

#include "ws_pid.h"
#include "ws_io.h"
#include "ws_general.h"

UINT8 show_pid = 0;
/*******************************************************************************
* FUNCTION NAME: pid
* PURPOSE:
* ARGUMENTS:
* RETURNS:
*
*******************************************************************************/
INT16 ws_pid(PidValsType *pid_vals, INT16 current, INT16 target)
{
  INT16 pid_val;
  INT16 error;
  INT16 prop_term;
  INT16 int_term;
  INT16 deriv_term;

  /*
  printf("c %d t %d  ", (int)current, (int)target);
  */

  /* calculate the current error */
  error = target - current;

  /* calculate the proportional term of the PID equation */
  prop_term = pid_vals->prop_gain * error;

  /* add the current error to the running integrated value */
  pid_vals->integral += error;

  /* prevent integral wind-up */
  if (pid_vals->max_integral < pid_vals->integral)
  {
    pid_vals->integral = pid_vals->max_integral;
  }
  else if (-(pid_vals->max_integral) > pid_vals->integral)
  {
    pid_vals->integral = -(pid_vals->max_integral);
  }

  /* calculate the integral term using the integrated value & gain */
  int_term = pid_vals->int_gain * pid_vals->integral;

  /* calculate the differential term */
  deriv_term = pid_vals->deriv_gain * (error - pid_vals->last_last_error);

  /* calculate the PID value using the previously calculate P, I, and D terms */
  pid_val = prop_term + int_term - deriv_term;

  pid_val = (int)pid_val / (int)pid_vals->scale_factor;

  /* limit PID value to max & min values */
  MIN(pid_val, pid_vals->min_val);
  MAX(pid_val, pid_vals->max_val);

#if 1
  if(show_pid == 1)
  {
    printf("le %d e %d p %d i %d d %d s %d",
           (int)pid_vals->last_last_error, (int)error, (int)prop_term, (int)int_term,
           (int)deriv_term, (int)pid_val);
    printf("\r");
  }

#endif
  pid_vals->last_last_error = pid_vals->last_error;
  pid_vals->last_error = error;

  return (pid_val);
}



