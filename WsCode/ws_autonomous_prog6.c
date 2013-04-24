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
 * Program 7
 * The Snatcher Skipper
 * ********************************/
AutoProgramType program6 =
{
  {
    AUTO_TASK_VISION(),
    AUTO_TASK_SCORE(SKIP_ON_FAIL),
    AUTO_TASK_WAIT(15),
    // This won't be reached unless we don't
    // find a valid vision tetra.
    // If we get here, drive to the autoloader
    //
#ifdef REAL_ROBOT
    AUTO_TASK_SKIP_IF_BLUE(),
    AUTO_TASK_DRIVE_DIST_LOW(400,127),
    AUTO_TASK_SKIP_IF_BLUE(),
    AUTO_TASK_ROTATE(27),
    AUTO_TASK_SKIP_IF_BLUE(),
    AUTO_TASK_DRIVE_DIST_LOW(400,127),
    AUTO_TASK_SKIP_IF_BLUE(),
    AUTO_TASK_ROTATE(50),
    AUTO_TASK_SKIP_IF_BLUE(),
    AUTO_TASK_DRIVE_DIST_LOW(100,127),

    AUTO_TASK_SKIP_IF_RED(),
    AUTO_TASK_DRIVE_DIST_LOW(200,95),
    AUTO_TASK_SKIP_IF_RED(),
    AUTO_TASK_ROTATE(200),
    AUTO_TASK_SKIP_IF_RED(),
    AUTO_TASK_DRIVE_DIST_LOW(1000,127),
    AUTO_TASK_SKIP_IF_RED(),
    AUTO_TASK_ROTATE(20),
    AUTO_TASK_SKIP_IF_RED(),
    AUTO_TASK_DRIVE_DIST_LOW(100,127),
#else
    AUTO_TASK_SKIP_IF_BLUE(),
    AUTO_TASK_DRIVE_DIST_LOW(400,127),
    AUTO_TASK_SKIP_IF_BLUE(),
    AUTO_TASK_ROTATE(27),
    AUTO_TASK_SKIP_IF_BLUE(),
    AUTO_TASK_DRIVE_DIST_LOW(1400,127),
    AUTO_TASK_SKIP_IF_BLUE(),
    AUTO_TASK_ROTATE(50),
    AUTO_TASK_SKIP_IF_BLUE(),
    AUTO_TASK_DRIVE_DIST_LOW(100,127),

    AUTO_TASK_SKIP_IF_RED(),
    AUTO_TASK_DRIVE_DIST_LOW(300,95),
    AUTO_TASK_SKIP_IF_RED(),
    AUTO_TASK_ROTATE(210),
    AUTO_TASK_SKIP_IF_RED(),
    AUTO_TASK_DRIVE_DIST_LOW(1000,127),
    AUTO_TASK_SKIP_IF_RED(),
    AUTO_TASK_ROTATE(20),
    AUTO_TASK_SKIP_IF_RED(),
    AUTO_TASK_DRIVE_DIST_LOW(100,127),
#endif

    NULL
  },
  USE_VISION
};
