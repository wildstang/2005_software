/*******************************************************************************
* FILE NAME: ws_includes.h
*
* DESCRIPTION:
*  General structures & enumerations
*
*******************************************************************************/

#ifndef __ws_includes_h_
#define __ws_includes_h_

/******************************* TYPEDEFS *************************************/

typedef enum
{
  POTS_GOOD,
  POTS_BAD
} PotStateType;

typedef enum
{
  RELAY_OFF = 0,
  RELAY_FWD,
  RELAY_REV
} RelayValsType;

typedef enum
{
  PUMP_OFF = 0,
  PUMP_ON
} PumpRunType;

typedef enum
{
  SHIFTER_LOW = 0,
  SHIFTER_HIGH
} ShifterPosType;

typedef enum
{
  WING_IN = 0,
  WING_OUT
} WingPosType;

typedef enum
{
  SPEAR_NO_GRAB = 0,
  SPEAR_GRAB
} SpearGrabType;

typedef enum
{
  SPEAR_NO_TILT = 0,
  SPEAR_TILT_DOWN
} SpearTiltType;

typedef enum
{
  SPEAR_IN = 0,
  SPEAR_OUT
} SpearRetractType;

/******************************** MACROS **************************************/


#define MIN_MAX(variable,min_val,max_val)  MIN((variable),(min_val)); MAX((variable),(max_val))

#define MAX(variable, max_val)  if (variable > (max_val)) variable = (max_val)
#define MIN(variable, min_val)  if (variable < (min_val)) variable = (min_val)

#define MAX_RETURN(value, max_val) ((value) > (max_val) ? (max_val) : (value))
#define MIN_RETURN(value, min_val) ((value) < (min_val) ? (min_val) : (value))

#define DEADZONE(var, zone)  if ((var > (127 - (zone))) && \
                                 (var < (127 + (zone)))) \
                               var = 127

#define GET_ANALOG_VALUE_SHIFT(a) (Get_Analog_Value(a) >> 2)

#define HAT_RANGE_CHECK(hat, value) \
   ((((hat) >= ((int)(value) - HAT_RANGE)) && \
     ((hat) <= ((int)(value) + HAT_RANGE))) ? \
    TRUE : FALSE)

/***************************** DEFINITIONS ************************************/

#define SUCCESS 1
#define FAIL    0

#define ROBOT_ENABLED  0
#define ROBOT_DISABLED 1

#define AUTO_ENABLED   1
#define AUTO_DISABLED  0

#define NUM_PACKETS_PER_SEC 40

/* some auto positions for the lift */
#define LIFT_BOTTOM            0
#define LIFT_TOP               1
#define LIFT_LOAD_BOTTOM       2
#define LIFT_LOAD_TOP          3
#define LIFT_SCORE             4
#define MAX_LIFT_NUM_POSITIONS 5

#define HAT_RANGE    10

#define HAT_NONE     127
#define HAT_UP       HAT_RANGE
#define HAT_DOWN     254-HAT_RANGE
#define HAT_LEFT     185
#define HAT_RIGHT    65

#define ENCODER_DEBUG_NONE  0
#define ENCODER_DEBUG_LIFT  1
#define ENCODER_DEBUG_TILT  2
/****************************** STRUCTURES ************************************/
typedef struct motor_vals_
{
  INT8                left_drive;
  INT8                right_drive;
  PumpRunType         pump;
  INT8                lift_tilt;
  INT8                lift_height;
  UINT8               wing_lock;
  ShifterPosType      shifter_position;
  WingPosType         fwing;
  WingPosType         bwing;
  SpearGrabType       top_spear_grabber;
  SpearGrabType       bot_spear_grabber;
  SpearTiltType       top_spear_tilt;
  SpearTiltType       bot_spear_tilt;
  SpearRetractType    top_spear_retract;
  SpearRetractType    bot_spear_retract;
} MotorValsType;

typedef struct calibration_vals_
{
  UINT8  lift_tilt_left;
  UINT8  lift_tilt_vertical;
  UINT16 lift_height[MAX_LIFT_NUM_POSITIONS];
} CalibrationValsType;

extern MotorValsType motor_vals;

/************************* FUNCTION PROTOTYPES ********************************/

#endif /* __ws_includes_h_ */

