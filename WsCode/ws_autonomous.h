/*******************************************************************************
* FILE NAME: ws_autonomous.h
*
* DESCRIPTION:
*  This is the include file which corresponds to autonomous.c
*
*  It contains some aliases and function prototypes used in those files.
*
* USAGE:
*  If you add your own routines to those files, this is a good place to add
*  your custom macros (aliases), type definitions, and function prototypes.
*******************************************************************************/

#ifndef __ws_autonomous_h_
#define __ws_autonomous_h_

#define FAKE_AUTON 0
#define PRINT_TASK_INFO 1

// Set this to 1 to enable picking up by driving
// directly into tetra and turning
#define USE_NEW_PICKUP_METHOD 1

/******************************* TYPEDEFS *************************************/
typedef enum
{
  AUTO_OFF = 0,
  AUTO_ON
} AutoStateType;

typedef enum
{
  STARTING_POS_NOT_SET,
  STARTING_POS_SET
} StartPosSetType;

typedef enum
{
  START_COLOR_RED = 0,
  START_COLOR_BLUE
} StartColorType;
/******************************** MACROS **************************************/

/***************************** DEFINITIONS ************************************/

/* Camera calibration
 *
 *  Feet    'd'
 *    1      20
 *    2      32
 *    3      42
 *    4      52
 *    5      63
 *    6      73
 *    7      86
 *    8      97
 *    9     109
 *   10     117
 *   11     125
 *   12     134
 */

/* WARNING WARNING WARNING
   Any time that APPROACH_DISTANCE or PICKUP_DISTANCE are changed, the
   X & Y constants MUST be re-calculated using the "auto points.xls"
   spreadsheet */
/* tetra - either side */
#define TETRA_PICKUP_DISTANCE           32
#define TETRA_APPROACH_DISTANCE         42

#if USE_NEW_PICKUP_METHOD
/* Set points to be 0 to drive right into tetra */
#define TETRA_APPROACH_X                0
#define TETRA_APPROACH_Y                0
#define TETRA_APPROACH_RANGE            7
#else
#define TETRA_APPROACH_X                16
#define TETRA_APPROACH_Y                56
#define TETRA_APPROACH_RANGE            5
#endif

#if USE_NEW_PICKUP_METHOD
#define TETRA_PICKUP_ORIENT             60
#define TETRA_PICKUP_HEADING            210
#define TETRA_PICKUP_RANGE              (TETRA_PICKUP_DISTANCE + \
                                         (TETRA_PICKUP_DISTANCE / 4))
#else
#define TETRA_PICKUP_ORIENT             235
#define TETRA_PICKUP_HEADING            210
#define TETRA_PICKUP_RANGE              (TETRA_PICKUP_DISTANCE + \
                                         (TETRA_PICKUP_DISTANCE / 4))
#endif

/* center goal left (path 1 - pickup from 3 & 4) - blue side */
#define BLUE_CENTER_L1_SCORE_DISTANCE       26
#define BLUE_CENTER_L1_SCORE_APPROACH_DIST  60
#define BLUE_CENTER_L1_SCORE_APPROACH_X     60
#define BLUE_CENTER_L1_SCORE_APPROACH_Y     36
#define BLUE_CENTER_L1_SCORE_APPROACH_RANGE 5
#define BLUE_CENTER_L1_SCORE_ORIENT         254
#define BLUE_CENTER_L1_SCORE_HEADING        200
#define BLUE_CENTER_L1_SCORE_RANGE          (BLUE_CENTER_L1_SCORE_DISTANCE + \
                                             (BLUE_CENTER_L1_SCORE_DISTANCE / 6))

/* center goal front (path 2 - pickup from 6 & 7) - blue side */
#define BLUE_CENTER_L2_SCORE_DISTANCE       27
#define BLUE_CENTER_L2_SCORE_APPROACH_DIST  80
#define BLUE_CENTER_L2_SCORE_APPROACH_X     80
#define BLUE_CENTER_L2_SCORE_APPROACH_Y     34
#define BLUE_CENTER_L2_SCORE_APPROACH_RANGE 5
#define BLUE_CENTER_L2_SCORE_ORIENT         254
#define BLUE_CENTER_L2_SCORE_HEADING        200
#define BLUE_CENTER_L2_SCORE_RANGE          (BLUE_CENTER_L2_SCORE_DISTANCE + \
                                             (BLUE_CENTER_L2_SCORE_DISTANCE / 6))

/* center goal front - red side (Pos 4 & 6) */
#define RED_CENTER_F1_SCORE_DISTANCE        29
#define RED_CENTER_F1_SCORE_APPROACH_DIST   60
#define RED_CENTER_F1_SCORE_APPROACH_X      63
#define RED_CENTER_F1_SCORE_APPROACH_Y     -33
#define RED_CENTER_F1_SCORE_APPROACH_RANGE  5
#define RED_CENTER_F1_SCORE_ORIENT          43
#define RED_CENTER_F1_SCORE_HEADING         200
#define RED_CENTER_F1_SCORE_RANGE           (RED_CENTER_F1_SCORE_DISTANCE + \
                                             (RED_CENTER_F1_SCORE_DISTANCE / 6))

/* center goal front - red side (Pos 7) */
#define RED_CENTER_F2_SCORE_DISTANCE        45
#define RED_CENTER_F2_SCORE_APPROACH_DIST   70
#define RED_CENTER_F2_SCORE_APPROACH_X      91
#define RED_CENTER_F2_SCORE_APPROACH_Y     -28
#define RED_CENTER_F2_SCORE_APPROACH_RANGE  5
#define RED_CENTER_F2_SCORE_ORIENT          43
#define RED_CENTER_F2_SCORE_HEADING         200
#define RED_CENTER_F2_SCORE_RANGE           (RED_CENTER_F2_SCORE_DISTANCE + \
                                             (RED_CENTER_F2_SCORE_DISTANCE / 6))

/* center goal back - red side (Pos 3) */
#define RED_CENTER_B_SCORE_DISTANCE        31
#define RED_CENTER_B_SCORE_APPROACH_DIST   65
#define RED_CENTER_B_SCORE_APPROACH_X      0
#define RED_CENTER_B_SCORE_APPROACH_Y      75
#define RED_CENTER_B_SCORE_APPROACH_RANGE  5
#define RED_CENTER_B_SCORE_ORIENT          213
#define RED_CENTER_B_SCORE_HEADING         200
#define RED_CENTER_B_SCORE_RANGE           (RED_CENTER_B_SCORE_DISTANCE + \
                                            (RED_CENTER_B_SCORE_DISTANCE / 6))

/* right goal - either side */
#define RIGHT_SCORE_DISTANCE             20
#define RIGHT_SCORE_APPROACH_DIST        15
#define RIGHT_SCORE_APPROACH_X           25
#define RIGHT_SCORE_APPROACH_Y          -2
#define RIGHT_SCORE_APPROACH_RANGE       5
#define RIGHT_SCORE_ORIENT               43
#define RIGHT_SCORE_HEADING              200
#define RIGHT_SCORE_RANGE                (RIGHT_SCORE_DISTANCE + \
                                          (RIGHT_SCORE_DISTANCE / 6))


/* autonomous program selector knob */
#define WAYPOINT_OI_SEL0   250
#define WAYPOINT_OI_SEL1   229
#define WAYPOINT_OI_SEL2   203
#define WAYPOINT_OI_SEL3   176
#define WAYPOINT_OI_SEL4   152
#define WAYPOINT_OI_SEL5   127
#define WAYPOINT_OI_SEL6   100
#define WAYPOINT_OI_SEL7    75
#define WAYPOINT_OI_SEL8    49
#define WAYPOINT_OI_SEL9    22
#define WAYPOINT_OI_SEL10    4
#define WAYPOINT_OI_DIFF     4

/* starting position selector */
#define OI_SEL_START_RED_L   250
#define OI_SEL_START_RED_C   230
#define OI_SEL_START_RED_R   203
#define OI_SEL_START_BLUE_L  176
#define OI_SEL_START_BLUE_C  151
#define OI_SEL_START_BLUE_R  125
#define OI_SEL_START_DIFF      4


/* Constant for the size of auto programs */
#define MAX_AUTO_PROGRAMS  1
#define MAX_AUTO_TASKS     25
#define AUTO_PROGRAM_START (UINT8) -1
#define AUTO_PROGRAM_DONE  (UINT8) -2

#define AUTO_LED_BLINK_END 8
#define AUTO_LED_BLINK_DELAY_END (AUTO_LED_BLINK_END + 28)

#define TETRA_PRIORITY_THRESHOLD  10

#define GOAL_ID_CENTER  5

/****************************** STRUCTURES ************************************/

typedef struct disthdg_
{
  UINT16 dist;
  UINT16 hdg;
  UINT8  orient;
} DistHdgType;

typedef struct tetrapos_
{
  UINT8 tetra1;
  UINT8 tetra2;
} TetraPosType;

typedef struct encodervals_
{
  INT8 left;
  INT8 right;
  UINT8 orient;
} EncoderValsType;

typedef struct score_params_
{
  INT8  approach_x;
  INT8  approach_y;
  UINT8 approach_range;
  UINT8 drop_orient;
  UINT8 drop_dist;
  UINT8 drop_range;
} ScoreParamsType;


typedef struct position_
{
  UINT16 x;
  UINT16 y;
  UINT8  theta;
} PositionType;

typedef struct waypoint_data_
{
  INT8  speed;
  UINT8 option_flags;
  UINT8 s_arm_pos;
} WaypointDataType;

typedef struct waypoint_common_
{
  UINT8 orientation; /* compass reading of the robot in brads, 0 if fwd */
  INT8  speed;       /* speed to get to the point, 127 if full speed    */
  UINT8 range;       /* size of the waypoint, determines how close we need
                      * to be to the point before moving on to the next point */
  UINT8 option_flags; /* options for each waypoint */
  UINT8 max_time;    /* max time allocated for each waypoint */
} WaypointInfo;

typedef struct waypoint_pos_
{
  UINT16 x_pos;
  UINT16 y_pos;
} WaypointPos;

typedef struct waypoint_
{
  WaypointPos*  p_waypoint_pos;
  WaypointInfo* p_waypoint_info;
} Waypoints;



/*******************************************
2005 Autonomous Constants and Data Types
*******************************************/
typedef enum
{
  STARTING_POS_UNINIT = 21,
  STARTING_POS_BLUE_LEFT,
  STARTING_POS_BLUE_CENTER,
  STARTING_POS_BLUE_RIGHT,
  STARTING_POS_RED_LEFT,
  STARTING_POS_RED_CENTER,
  STARTING_POS_RED_RIGHT
} StartingPosType;

typedef enum
{
  GOAL_BACK_LEFT = 9,
  GOAL_BACK_CENTER,
  GOAL_BACK_RIGHT,
  GOAL_MIDDLE_LEFT,
  GOAL_MIDDLE_CENTER,
  GOAL_MIDDLE_RIGHT,
  GOAL_FAR_LEFT,
  GOAL_FAR_CENTER,
  GOAL_FAR_RIGHT
} GoalType;


typedef enum
{
  TASK_STATE_INITIALIZE = 0,
  TASK_STATE_PROCESSING,
  TASK_STATE_DONE,
  TASK_STATE_ABORT,
  TASK_STATE_SKIP_NEXT
} TaskStateType;

typedef enum
{
  SCORE_CENTER_BLUE_LEFT1 = 0,
  SCORE_CENTER_BLUE_LEFT2,
  SCORE_CENTER_RED_FRONT_1,
  SCORE_CENTER_RED_FRONT_2,
  SCORE_CENTER_RED_BACK,
  SCORE_RIGHT
} ScoreGoalType;

typedef enum
{
  BLINK_ON,
  BLINK_OFF,
  BLINK_DELAY
} LEDBlinkStates;

typedef enum
{
  NO_VISION,
  USE_VISION
} UseVisionType;

typedef enum
{
  NO_SKIP_ON_FAIL,
  SKIP_ON_FAIL
} ScoreSkipType;

typedef enum
{
  NO_CROSS_TO_AUTOLOADER,
  CROSS_TO_AUTOLOADER
} CrossFieldType;

typedef struct vision_param_
{
  UINT8 dummy;
} VisionParamType;

#if 0
typedef struct auto_load_param_
{
} AutoLoadParamType;
#endif

typedef struct knock_hang_
{
  UINT8 goal_id;
} KnockHangParamType;

typedef struct score_param_
{
  UINT8 skip_on_fail;
} ScoreParamType;

typedef struct wait_param_
{
  UINT8 seconds;
} WaitParamType;

typedef struct drive_point_param_
{
  UINT8  target_x;
  UINT8  target_y;
  UINT8  target_orient;
} DrivePointParamType;

typedef struct drive_direct_param_
{
  INT8   speed;
  UINT8  seconds;
  UINT8  orient;
} DriveDirectParamType;

typedef struct drive_tank_param_
{
  INT8   left_speed;
  INT8   right_speed;
  UINT8  seconds;
} DriveTankParamType;

typedef struct drive_dist_param_
{
  UINT16   dist;
  INT8   speed;
} DriveDistParamType;

typedef struct encoder_val_param_
{
  UINT16 encoder_val;
  INT8   wait_for_feedback;
} EncoderPosParamType;

typedef struct wing_pos_param_
{
  UINT8 bitmap;
} BitmapParamType;

typedef struct rotate_param_
{
  UINT8   angle;
} RotateParamType;

typedef struct grab_hang_param_
{
  UINT8 goal_id;
} GrabHangParamType;

typedef struct deliver_param_
{
  UINT8 cross_to_autoloader;
} DeliverParamType;

typedef struct skip_color_param_
{
  UINT8 color;
} SkipColorParamType;


typedef struct auto_task_
{
  UINT8 (*function) (void *);
  union
  {
  /* WHY DO THE LARGE PARAMETERS NEED TO BE ON TOP??? */
    DriveDistParamType  driveDist;
    DrivePointParamType drivePoint;
    DriveDirectParamType  driveDirect;
    DriveTankParamType  driveTank;
    RotateParamType     rotate;
    BitmapParamType     bitmap;
    EncoderPosParamType  encoderPos;
    SkipColorParamType  skipColor;
    VisionParamType     vision;
#if 0
    AutoLoadParamType   auto_load;
#endif
    KnockHangParamType  knock_hang;
    ScoreParamType      score;
    WaitParamType       wait;
    GrabHangParamType   grab_hang;
    DeliverParamType    deliver;
  } parameters;
} AutoTaskType;

typedef struct auto_program_
{
  AutoTaskType    task_list[MAX_AUTO_TASKS];
  UINT8 use_vision;
} AutoProgramType;




/************************* FUNCTION PROTOTYPES ********************************/

extern void autonomous_init(void);
extern void auto_lock_in(void);
extern void auto_main(void);
extern void auto_chooser(void);
extern void auto_output_off(void);
extern UINT8 tetra_chooser(UINT8, UINT8);
extern void display_auto_data(void);

#define AUTO_DO_NOTHING &program0
extern AutoProgramType program0;
extern AutoProgramType program1;
extern AutoProgramType program2;
extern AutoProgramType program3;
extern AutoProgramType program4;
extern AutoProgramType program5;
extern AutoProgramType program6;

extern INT16 g_autodrive_ticks;
#endif /* __ws_autonomous_h_ */

