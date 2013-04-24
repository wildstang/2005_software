/*******************************************************************************
* FILE NAME: ws_cc.h
*
* DESCRIPTION:
*  This is the include file which corresponds to ws_cc.c
*
*  It contains some aliases and function prototypes used in those files.
*
* USAGE:
*  If you add your own routines to those files, this is a good place to add
*  your custom macros (aliases), type definitions, and function prototypes.
*******************************************************************************/

#ifndef __ws_cc_h_
#define __ws_cc_h_

#include "ws_autonomous.h"  /* Needed for PositionType */
/******************************* TYPEDEFS *************************************/

typedef enum
{
   CC_SUCCESS=1,
   CC_TIMEOUT,
   CC_FAIL
} CcReturnType;

/******************************** MACROS **************************************/

/***************************** DEFINITIONS ************************************/

/* Command bytes sent to CC to request data */
/* Dist(16bit), Hdg(16bit), Orient(8bit) */
#define  CC_CMD_REQ_DIST_HDG               1
/* Tetra1(8bit), Tetra2(8bit) */
#define  CC_CMD_REQ_TETRA_POS              2
/* Left Encoder(8bit),Right Encoder(8bit), Orient (8bit) */
#define  CC_CMD_REQ_ENCODER                3
/* Dist(16bit), Hdg(16bit), Orient(8bit) */
#define  CC_CMD_REQ_DIST_HDG_XY            4

/* Command bytes sent to CC to set data */
/* position_id(8bit)  */
#define  CC_CMD_SET_POSITION               101
/* request gyro calibration*/
#define  RECALIBRATE_GYRO                  102
/* set lift angle (16bit) */
#define  CC_CMD_SET_LIFT_ANGLE             103
/* tell it what object to track (8bit) */
#define  CC_CMD_SET_TRACK_OBJECT           104

/* ack from CC for object track command */
#define  CC_ACK_TRACK_OBJECT               151
/* ack from CC for starting pos command */
#define  CC_ACK_STARTING_POS               152

/* Command bytes sent to CC for debugging purposes */
/* request raw 32-bit X value for debug */
#define  CC_CMD_REQ_RAW_X                  201
/* request raw 32-bit Y value for debug */
#define  CC_CMD_REQ_RAW_Y                  202
/* request Quad decode errors */
#define  CC_CMD_REQ_QUAD_ERRS              203
/* set camera position to center */
#define  CC_CMD_SET_CAMERA_CENTER          204
/* set camera position to 90 degrees right & down */
#define  CC_CMD_SET_CAMERA_9090            205
/* set camera position to store position */
#define  CC_CMD_SET_CAMERA_STORE           206


/* Num of loops before giving up on a resp from the CC */
#define  CC_LOOP_CNT_TIMEOUT  65534


/* Size of the response to various commands */
#define CC_RESP_DIST_HDG_SIZE     9  /* 2 bytes dist, 2 bytes hdg,
                                        1 byte orient */
#define CC_RESP_TETRA_POS_SIZE    2  /* 1 byte for each tetra */
#define CC_RESP_QUAD_ERRS_SIZE    1  /* Quad Decode Errors is 1 byte */
#define CC_RESP_RAWY_SIZE         4  /* Raw Y is 4 bytes */
#define CC_RESP_RAWX_SIZE         4  /* Raw X is 4 bytes */
#define CC_RESP_TETRA_CHOICE_SIZE 2  /* 1 byte cmd, 1 byte object */
#define CC_RESP_ENCODER_VAL_SIZE  3  /* 1 byte left, 1 byte right, 1 byte orient*/
#define CC_RESP_STARTING_POS_SIZE 1  /* 1 byte cmd */


/* IDs of objects to track */
#define OBJ_TRACK_TETRA_BASE      0
#define OBJ_TRACK_GOAL_BASE      10


/****************************** STRUCTURES ************************************/

/************************* FUNCTION PROTOTYPES ********************************/

extern UINT8 cc_set_position(UINT8);
extern void cc_set_lift_angle(void);
extern UINT8 cc_set_track_object(UINT8);
extern void cc_set_camera_pos(UINT8);
extern void cc_gyro_calibrate(void);
extern UINT8 cc_get_dist_hdg(DistHdgType *, UINT8);
extern UINT8 cc_get_tetra_pos(TetraPosType *);
extern UINT8 cc_get_encoder_vals(EncoderValsType *);
extern UINT8 readCcReg(unsigned char *, UINT8, UINT8, UINT8, UINT16);
extern void print_cc_debug(void);


#endif /* __ws_cc_h_ */

