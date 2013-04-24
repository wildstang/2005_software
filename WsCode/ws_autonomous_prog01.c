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

/***********************************
 * Program 1
 * The Sleeper
 * ********************************/
AutoProgramType program0 =
{
  {
    NULL
  },
  NO_VISION
};

/***********************************
 * Program 2
 * The Runner
 * ********************************/
AutoProgramType program1 =
{
  {
    AUTO_TASK_DRIVE_DIST_HIGH(800,127),
    NULL
  },
  NO_VISION
};
