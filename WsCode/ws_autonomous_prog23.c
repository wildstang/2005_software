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
#include "ws_atask_misc.h"

/***********************************
 * Program 3
 * The Snatcher Capper
 * ********************************/
AutoProgramType program2 =
{
  {
    // If a valid vision tetra is found, go pick
    // it up and score it....
    AUTO_TASK_VISION(),
    AUTO_TASK_SCORE(NO_SKIP_ON_FAIL),
    AUTO_TASK_WAIT(15),
    NULL
  },
  USE_VISION
};

/***********************************
 * Program 4
 * The Dumper
 * ********************************/
AutoProgramType program3 =
{
  {
    AUTO_TASK_DELIVER(NO_CROSS_TO_AUTOLOADER),
    AUTO_TASK_WAIT(15),
    NULL
  },
  NO_VISION
};
