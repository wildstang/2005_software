#include <stdio.h>
#include "ifi_aliases.h"
#include "ifi_default.h"
#include "ifi_utilities.h"
#include "ws_includes.h"
#include "user_routines.h"
#include "delays.h"       /*defined locally*/
#include "ws_io.h"
#include "ws_lift.h"

#include "ws_autonomous.h"
#include "ws_autonomous_tasks.h"

#include "ws_atask_misc.h"
/***********************************
 * Program 5
 * The Dumper Slider
 * ********************************/
AutoProgramType program4 =
{
  {
    AUTO_TASK_DELIVER(CROSS_TO_AUTOLOADER),
    NULL
  },
  NO_VISION
};

/***********************************
 * Program 6
 * The Slapper
 * ********************************/
AutoProgramType program5 =
{
  {
    AUTO_TASK_KNOCK_HANG(),
#if 0
    AUTO_TASK_SET_WING_POS( ATASK_FWING_OUT | ATASK_BWING_IN ),
    AUTO_TASK_WAIT(1),
    AUTO_TASK_SET_WING_POS( ATASK_FWING_OUT | ATASK_BWING_OUT ),
    AUTO_TASK_WAIT(1),
    AUTO_TASK_SET_WING_POS( ATASK_FWING_IN | ATASK_BWING_OUT ),
    AUTO_TASK_WAIT(1),
    AUTO_TASK_SET_WING_POS( ATASK_FWING_IN | ATASK_BWING_IN ),
    AUTO_TASK_WAIT(1),
    AUTO_TASK_SET_TOP_SPEAR_POS( ATASK_SPEAR_OUT | ATASK_SPEAR_NO_TILT | ATASK_SPEAR_GRAB ),
    AUTO_TASK_WAIT(1),
    AUTO_TASK_SET_TOP_SPEAR_POS( ATASK_SPEAR_OUT | ATASK_SPEAR_TILT | ATASK_SPEAR_GRAB ),
    AUTO_TASK_WAIT(1),
    AUTO_TASK_SET_TOP_SPEAR_POS( ATASK_SPEAR_OUT | ATASK_SPEAR_TILT | ATASK_SPEAR_NO_GRAB ),
    AUTO_TASK_WAIT(1),
    AUTO_TASK_SET_TOP_SPEAR_POS( ATASK_SPEAR_IN | ATASK_SPEAR_NO_TILT | ATASK_SPEAR_GRAB ),
    AUTO_TASK_WAIT(1),
    AUTO_TASK_SET_BOT_SPEAR_POS( ATASK_SPEAR_OUT | ATASK_SPEAR_NO_TILT | ATASK_SPEAR_GRAB ),
    AUTO_TASK_WAIT(1),
    AUTO_TASK_SET_BOT_SPEAR_POS( ATASK_SPEAR_OUT | ATASK_SPEAR_TILT | ATASK_SPEAR_GRAB ),
    AUTO_TASK_WAIT(1),
    AUTO_TASK_SET_BOT_SPEAR_POS( ATASK_SPEAR_OUT | ATASK_SPEAR_TILT | ATASK_SPEAR_NO_GRAB ),
    AUTO_TASK_WAIT(1),
    AUTO_TASK_SET_BOT_SPEAR_POS( ATASK_SPEAR_IN | ATASK_SPEAR_NO_TILT | ATASK_SPEAR_GRAB ),
    AUTO_TASK_WAIT(1),
#endif
    AUTO_TASK_WAIT(15),
    NULL
  },
  NO_VISION
};
