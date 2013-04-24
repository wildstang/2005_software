/*******************************************************************************
* FILE NAME: ws_atask_misc.*
* DESCRIPTION:
*  This is the include file which corresponds to ws_atask_misc.c
*
*  It contains some aliases and function prototypes used in those files.
*
* USAGE:
*  If you add your own routines to those files, this is a good place to add
*  your custom macros (aliases), type definitions, and function prototypes.
*******************************************************************************/

#ifndef __ws_atask_misc_h__
#define __ws_atask_misc_h__

#include "ws_includes.h"

#define ATASK_BWING_ORDER 0
#define ATASK_FWING_ORDER 1

#define ATASK_SPEAR_RETRACT_ORDER 2
#define ATASK_SPEAR_TILT_ORDER 1
#define ATASK_SPEAR_GRAB_ORDER 0

#define ATASK_FWING_IN   ((WING_IN) << (ATASK_FWING_ORDER))
#define ATASK_FWING_OUT  ((WING_OUT) << (ATASK_FWING_ORDER))

#define ATASK_BWING_IN   ((WING_IN) << (ATASK_BWING_ORDER))
#define ATASK_BWING_OUT  ((WING_OUT) << (ATASK_BWING_ORDER))

#define ATASK_SPEAR_OUT  ((SPEAR_OUT) << (ATASK_SPEAR_RETRACT_ORDER))
#define ATASK_SPEAR_IN   ((SPEAR_IN) << (ATASK_SPEAR_RETRACT_ORDER))

#define ATASK_SPEAR_NO_TILT  ((SPEAR_NO_TILT) << (ATASK_SPEAR_TILT_ORDER))
#define ATASK_SPEAR_TILT     ((SPEAR_TILT_DOWN) << (ATASK_SPEAR_TILT_ORDER))

#define ATASK_SPEAR_NO_GRAB  ((SPEAR_NO_GRAB) << (ATASK_SPEAR_GRAB_ORDER))
#define ATASK_SPEAR_GRAB     ((SPEAR_GRAB) << (ATASK_SPEAR_GRAB_ORDER))

#define ATASK_FWING_MASK   (1 << (ATASK_FWING_ORDER))
#define ATASK_BWING_MASK   (1 << (ATASK_BWING_ORDER))

#define ATASK_SPEAR_GRABBER_MASK   (1 << (ATASK_SPEAR_GRAB_ORDER))
#define ATASK_SPEAR_TILT_MASK  (1 << (ATASK_SPEAR_TILT_ORDER))
#define ATASK_SPEAR_RETRACT_MASK   (1 << (ATASK_SPEAR_RETRACT_ORDER))

#endif /* __ws_atask_misc_h__ */

