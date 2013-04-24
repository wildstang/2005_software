#include <stdio.h>
#include "ifi_aliases.h"
#include "ifi_default.h"
#include "ifi_utilities.h"
#include "ws_includes.h"
#include "user_routines.h"
#include "delays.h"       /*defined locally*/
#include "ws_io.h"

#include "ws_autonomous.h"
#include "ws_autonomous_tasks.h"

UINT8 auto_task_wait(void *params)
{
  static UINT8 local_state = WAIT_INITIALIZE;
  static UINT16 counter = 0;
  static UINT16 wait_cycles;
  static UINT8 seconds;
  UINT8 ret_state;

#if AUTO_USE_WAIT
  if(local_state == WAIT_INITIALIZE)
  {
    counter = 0;
    seconds = ((WaitParamType *) params)->seconds;
    wait_cycles = NUM_PACKETS_PER_SEC * (UINT16)seconds;
#if PRINT_TASK_INFO
    printf("TASK STARTED: WAIT (%d) cycles (%d) seconds\r",wait_cycles,seconds);
#endif
    local_state = WAIT_WAITING;
  }

  if(counter >= wait_cycles)
  {
#if PRINT_TASK_INFO
    printf("WAIT TASK DONE\r");
#endif
    ret_state = TASK_STATE_DONE;
    local_state = WAIT_INITIALIZE;
  }
  else
  {
    counter++;
    ret_state = TASK_STATE_PROCESSING;
  }

  motor_vals.left_drive = 0;
  motor_vals.right_drive = 0;
  return ret_state;
#else
#if PRINT_TASK_INFO
    printf("TASK SKIPPED: Wait\r");
#endif
  return TASK_STATE_DONE;
#endif
}
