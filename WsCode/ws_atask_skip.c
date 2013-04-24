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
extern UINT8 g_starting_pos;

UINT8 auto_task_skip_color(void *params)
{
#if AUTO_USE_SKIP_COLOR
  UINT8 ret_state;

#if PRINT_TASK_INFO
    printf("START SKIP COLOR\r");
#endif
  if((((SkipColorParamType *)params)->color == START_COLOR_RED) &&
      ((g_starting_pos == STARTING_POS_RED_LEFT) ||
       (g_starting_pos == STARTING_POS_RED_CENTER) ||
       (g_starting_pos == STARTING_POS_RED_RIGHT)))
  {
#if PRINT_TASK_INFO
    printf("SKIP TASK - START IS RED\r");
#endif
    ret_state = TASK_STATE_SKIP_NEXT;
  }
  else if((((SkipColorParamType *)params)->color == START_COLOR_BLUE) &&
       ((g_starting_pos == STARTING_POS_BLUE_LEFT) ||
       (g_starting_pos == STARTING_POS_BLUE_CENTER) ||
       (g_starting_pos == STARTING_POS_BLUE_RIGHT)))
  {
#if PRINT_TASK_INFO
    printf("SKIP TASK - START IS BLUE\r");
#endif
    ret_state = TASK_STATE_SKIP_NEXT;
  }
  else
  {
#if PRINT_TASK_INFO
    printf("SKIP TASK - NEXT TASK WILL RUN\r");
#endif
    ret_state = TASK_STATE_DONE;
  }
  return ret_state;
#else
#if PRINT_TASK_INFO
    printf("TASK SKIPPED: Skip Color\r");
#endif
  return TASK_STATE_DONE;
#endif
}
