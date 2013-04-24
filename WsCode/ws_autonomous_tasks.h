#ifndef __ws_autonomous_tasks_h_
#define __ws_autonomous_tasks_h_

#define AUTO_USE_VISION            1
#define AUTO_USE_KNOCK             1
#define AUTO_USE_WAIT              1
#define AUTO_USE_SCORE             1
#define AUTO_USE_DRIVE_POINT       0
#define AUTO_USE_DRIVE_DIRECT      1
#define AUTO_USE_DRIVE_TANK        1
#define AUTO_USE_ROT_L_TANK        0
#define AUTO_USE_ROT_R_TANK        0
#define AUTO_USE_GRAB_HANG         0
#define AUTO_USE_DRIVE_DIST        1
#define AUTO_USE_DELIVER           1
#define AUTO_USE_ROTATE            1
#define AUTO_USE_SKIP_COLOR        1

#define AUTO_TASK_VISION()                        {&auto_task_vision,{(0)}}
#define AUTO_TASK_KNOCK_HANG()                    {&auto_task_knock_hang,{(0)}}
#define AUTO_TASK_WAIT(sec)                       {&auto_task_wait,{(sec)}}
#define AUTO_TASK_SCORE(skip_on_fail)             {&auto_task_score,{(skip_on_fail)}}
#define AUTO_TASK_DRIVE_DIRECT(speed,sec,orient)  {&auto_task_drive_direct,{(speed),(sec),(orient)}}
#define AUTO_TASK_DRIVE_POINT(x,y,orient)         {&auto_task_drive_point,{(x),(y),(orient)}}
#define AUTO_TASK_DRIVE_TANK(left,right,sec)      {&auto_task_drive_tank,{(left),(right),(sec)}}
#define AUTO_TASK_ROT_L_TANK(speed,sec)           AUTO_TASK_DRIVE_TANK(-(speed),(speed),(sec))
#define AUTO_TASK_ROT_R_TANK(speed,sec)           AUTO_TASK_DRIVE_TANK((speed),-(speed),(sec))
#define AUTO_TASK_GRAB_HANG(goal)                 {&auto_task_grab_hang,{(goal)}}
#define AUTO_TASK_DRIVE_DIST_LOW(speed,dist)      {&auto_task_drive_dist_low,{(speed),(dist)}}
#define AUTO_TASK_DRIVE_DIST_HIGH(speed,dist)     {&auto_task_drive_dist_high,{(speed),(dist)}}
#define AUTO_TASK_SET_WING_POS(wing_bitmap)       {&auto_task_set_wings,{(wing_bitmap)}}
#define AUTO_TASK_SET_TOP_SPEAR_POS(spear_bitmap) {&auto_task_set_top_spear,{(spear_bitmap)}}
#define AUTO_TASK_SET_BOT_SPEAR_POS(spear_bitmap) {&auto_task_set_bot_spear,{(spear_bitmap)}}
#define AUTO_TASK_SET_LIFT_HEIGHT(height,wait)    {&auto_task_set_lift_height,{(height),(wait)}}
#define AUTO_TASK_SET_TILT_POS(pos,wait)          {&auto_task_set_tilt_pos,{(pos),(wait)}}
#define AUTO_TASK_DELIVER(cross_to_autoloader)    {&auto_task_deliver,{(cross_to_autoloader)}}
#define AUTO_TASK_ROTATE(angle)                   {&auto_task_rotate,{(angle)}}
#define AUTO_TASK_SKIP_IF_COLOR(color)            {&auto_task_skip_color,{(color)}}
#define AUTO_TASK_SKIP_IF_RED()                   AUTO_TASK_SKIP_IF_COLOR((START_COLOR_RED))
#define AUTO_TASK_SKIP_IF_BLUE()                  AUTO_TASK_SKIP_IF_COLOR((START_COLOR_BLUE))

/*** State Machine states ***/
/* vision */
typedef enum
{
  VISION_INITIALIZE = 0,
  VISION_CC_COMM,
  VISION_DRIVE_TO_TETRA,
  VISION_DRIVE_TO_APPROACH_LO,
  VISION_REORIENT,
  VISION_DRIVE_THROUGH_TETRA,
  VISION_DRIVE_THROUGH_TETRA2,
  VISION_PICKUP_TETRA,
  VISION_SKIP,
  VISION_DONE
} VISION_STATES;

typedef enum
{
  DRIVE_POINT_INITIALIZE = 0,
  DRIVE_POINT_DONE
} DRIVE_POINT_STATES;

typedef enum
{
  DRIVE_DIRECT_INITIALIZE = 0,
  DRIVE_DIRECT_DRIVE,
  DRIVE_DIRECT_DONE
} DRIVE_DIRECT_STATES;

typedef enum
{
  DRIVE_TANK_INITIALIZE = 0,
  DRIVE_TANK_DRIVE,
  DRIVE_TANK_DONE
} DRIVE_TANK_STATES;

typedef enum
{
  DRIVE_DIST_INITIALIZE = 0,
  DRIVE_DIST_DRIVE,
  DRIVE_DIST_DONE
} DRIVE_DIST_STATES;

typedef enum
{
  ROTATE_INITIALIZE = 0,
  ROTATE_DRIVE,
  ROTATE_DONE
} ROTATE_STATES;

typedef enum
{
  KNOCK_HANG_INITIALIZE = 0,
  KNOCK_HANG_DRIVE_FWD,
  KNOCK_HANG_POSITION_LIFT,
  KNOCK_HANG_SET_SPEAR,
  KNOCK_HANG_DRIVE_AWAY,
  KNOCK_HANG_LOWER_LIFT,
  KNOCK_HANG_DONE
} KNOCK_HANG_STATES;

typedef enum
{
  GRAB_HANG_INITIALIZE = 0,
  GRAB_HANG_DONE
} GRAB_HANG_STATES;

typedef enum
{
  SCORE_INITIALIZE = 0,
  SCORE_CC_COMM,
  SCORE_DRIVE_TO_APPROACH,
  SCORE_TURN_TO_DROPOFF,
  SCORE_DRIVE_TO_DROPOFF,
  SCORE_DRIVE_THROUGH_DROPOFF,
  SCORE_SET_SPEAR_BEFORE_DROP,
  SCORE_DO_DROP,
  SCORE_WAIT_AFTER_DROP,
  SCORE_SET_SPEAR_AFTER_DROP,
  SCORE_LOWER_LIFT,
  SCORE_SKIP,
  SCORE_DONE
} SCORE_STATES;

typedef enum
{
  WAIT_INITIALIZE = 0,
  WAIT_WAITING,
  WAIT_DONE
} WAIT_STATES;

typedef enum
{
  DELIVER_INITIALIZE = 0,
  DELIVER_POSITION_LIFT,
  DELIVER_ROTATE_TO_DROP,
  DELIVER_DRIVE_TO_DROP,
  DELIVER_POSITION_LIFT_FOR_DROP,
  DELIVER_WAIT_BEFORE_DROP,
  DELIVER_DROP,
  DELIVER_COLLAPSE_LIFT,
  DELIVER_DRIVE_PAST_GOAL,
  DELIVER_ROTATE_TO_DRIVE_AWAY,
  DELIVER_DRIVE_AWAY,
  DELIVER_DONE
} DELIVER_STATES;

UINT8 auto_task_knock_hang(void *);
UINT8 auto_task_vision(void *);
UINT8 auto_task_wait(void *);
UINT8 auto_task_score(void *);
UINT8 auto_task_drive_point(void *);
UINT8 auto_task_drive_tank(void *);
UINT8 auto_task_drive_direct(void *);
UINT8 auto_task_drive_dist_low(void *);
UINT8 auto_task_drive_dist_high(void *);
UINT8 auto_task_grab_hang(void *);
UINT8 auto_task_deliver(void *);
UINT8 auto_task_rotate(void *);
UINT8 auto_task_skip_color(void *);
UINT8 auto_task_set_wings(void *);
UINT8 auto_task_set_top_spear(void *);
UINT8 auto_task_set_bot_spear(void *);
UINT8 auto_task_set_lift_height(void *);
UINT8 auto_task_set_tilt_pos(void *);

#endif /* __ws_autonomous_tasks_h_ */
