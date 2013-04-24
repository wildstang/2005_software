/*******************************************************************************
* FILE NAME: ws_lift.h
*
* DESCRIPTION:
*  This is the include file which corresponds to ws_lift.c
*
*  It contains some aliases and function prototypes used in those files.
*
* USAGE:
*  If you add your own routines to those files, this is a good place to add
*  your custom macros (aliases), type definitions, and function prototypes.
*******************************************************************************/

#ifndef __ws_lift_h_
#define __ws_lift_h_

/******************************* TYPEDEFS *************************************/
typedef enum
{
  LIFT_AUTO_MODE,
  LIFT_OVERRIDE_MODE,
  LIFT_MANU_MODE,
  LIFT_AUTO_MODE_TETRA_GRABBER
} LiftModeType;

typedef enum
{
  TILT_AUTO_MODE,
  TILT_OVERRIDE_MODE,
  TILT_MANU_MODE
} LiftModeType;

typedef enum
{
  LIFT_ENCODER_UNINITIALIZED,
  LIFT_ENCODER_INITIALIZED
} LiftEncoderStateType;

typedef enum
{
  LOADER_PROG_SET_TOP,
  LOADER_PROG_RAISE_FOR_BOTTOM,
  LOADER_PROG_SET_BOTTOM,
  LOADER_PROG_GOT_BOTTOM,
  LOADER_PROG_TUCK_BOTTOM,
  LOADER_PROG_DONE = LOADER_PROG_TUCK_BOTTOM
} LiftLoaderProgStateType;

typedef enum
{
  AUTO_SPEAR_CONTROL,
  MANUAL_SPEAR_CONTROL
} SpearControlType;

typedef enum
{
  HYST_BELOW_THRESHOLD,
  HYST_REACHED_TGT
} LiftHystType;

/******************************** MACROS **************************************/

/***************************** DEFINITIONS ************************************/

/* lift speeds */
#define LIFT_DOWN_ENCODER_VAL 10000 /* encoder value of lift in full down pos */
#define TILT_DOWN_ENCODER_VAL 10000 /* encoder value of tilt in full down pos */

#define LIFT_DEAD_ENCODER_COUNT     20

#define LIFT_STICK_HEIGHT_DEADZONE  10
#define LIFT_STICK_TILT_DEADZONE    5
#define LIFT_STICK_SWITCH_DEADZONE  30
#define TILT_STICK_SWITCH_DEADZONE  30

#define LIFT_HAT_AUTO_LOADER_RESET  HAT_DOWN
#define LIFT_HAT_AUTO_LOADER_STEP   HAT_UP
#define TILT_HAT_AUTO_POS_TILT      HAT_LEFT
#define TILT_HAT_AUTO_POS_VERT      HAT_RIGHT

#define LIFT_TILT_SCALE_UP_NUMERATOR    1
#define LIFT_TILT_SCALE_UP_DENOMINATOR  2
#define LIFT_TILT_SCALE_DN_NUMERATOR    1
#define LIFT_TILT_SCALE_DN_DENOMINATOR  2

#define LIFT_TILT_POT_MIN   60
#define LIFT_TILT_POT_MAX  100

#ifdef PROTO_ROBOT
#define LIFT_SPEED_SCALE_NUMERATOR    127
#define LIFT_SPEED_SCALE_DENOMINATOR  28
#else
#define LIFT_SPEED_SCALE_NUMERATOR    127
#define LIFT_SPEED_SCALE_DENOMINATOR  28
#endif

#define LIFT_ENCODER_BUFFER_SZE       4


#define LIFT_DOWN_SLOW_SPEED              -70

#define LIFT_HEIGHT_CLOSE_DIST            15

/* lift height positions */
#define LIFT_HEIGHT_LIMIT_BOTTOM          (LIFT_DOWN_ENCODER_VAL + 0)
#define LIFT_HEIGHT_LIMIT_TOP             (LIFT_DOWN_ENCODER_VAL + 1160)

#define LIFT_HEIGHT_BOT_FLOOR_PICKUP      (LIFT_DOWN_ENCODER_VAL + 111)
#define LIFT_HEIGHT_BOT_FLOOR_PICKUP2     (LIFT_DOWN_ENCODER_VAL + 140)
#define LIFT_HEIGHT_BOT_FLOOR_PICKUP3     (LIFT_DOWN_ENCODER_VAL + 170)
#define LIFT_HEIGHT_DRIVE_TO_APPROACH     (LIFT_DOWN_ENCODER_VAL + 300)
#define LIFT_HEIGHT_TURN_TO_GOAL          (LIFT_DOWN_ENCODER_VAL + 450)
#define LIFT_HEIGHT_DRIVE_NEXT_TO_GOAL    (LIFT_DOWN_ENCODER_VAL + 1100)
#define LIFT_HEIGHT_SCORE_CENTER          (LIFT_DOWN_ENCODER_VAL + 1437)
#define LIFT_HEIGHT_SCORE_OUTSIDE         (LIFT_DOWN_ENCODER_VAL + 860)
#define LIFT_HEIGHT_SCORE_DONE            (LIFT_DOWN_ENCODER_VAL + 600)

#define LIFT_HEIGHT_TOP_AUTOLOAD          (LIFT_DOWN_ENCODER_VAL + 0)
#define LIFT_HEIGHT_TOP_CLEAR_AUTOLOAD    (LIFT_DOWN_ENCODER_VAL + 183)

#define LIFT_HEIGHT_BOT_AUTOLOAD          (LIFT_DOWN_ENCODER_VAL + 470)
#define LIFT_HEIGHT_BOT_CLEAR_AUTOLOAD    (LIFT_DOWN_ENCODER_VAL + 622)
#define LIFT_HEIGHT_STOW                  (LIFT_DOWN_ENCODER_VAL + 350)

/* tilt positions */
#define TILT_POS_TILTED_BRADS             20
#ifdef PROTO_ROBOT
#define TILT_AUTO_DEADBAND                (10)
#define TILT_POSITION_STRAIGHT_UP         (TILT_DOWN_ENCODER_VAL - 255)
#define TILT_POSITION_PAST_VERTICAL       (TILT_DOWN_ENCODER_VAL - 270)
#define TILT_POSITION_DRIVER_AUTO_TILT    (TILT_DOWN_ENCODER_VAL - 120)
#else
#define TILT_AUTO_DEADBAND                (10)
#define TILT_POSITION_STRAIGHT_UP         (TILT_DOWN_ENCODER_VAL - 270)
#define TILT_POSITION_PAST_VERTICAL       (TILT_DOWN_ENCODER_VAL - 280)
#define TILT_POSITION_DRIVER_AUTO_TILT    (TILT_DOWN_ENCODER_VAL - 120)
#endif


/****************************** STRUCTURES ************************************/

/************************* FUNCTION PROTOTYPES ********************************/
void lift_init(void);
void tilt_init(void);
void lift_oi_input(void);
void tilt_oi_input(void);
void lift_height_manual(void);
void lift_height_auto(void);
INT8 lift_height_feedback(void);
INT8 lift_speed_feedback(INT8);
INT8 tilt_pos_feedback(void);
void lift_set_height(UINT16);
void tilt_set_pos(UINT16);
void spear_control(void);
void lift_program_auto_loader(void);


#endif /* __ws_lift_h_ */
