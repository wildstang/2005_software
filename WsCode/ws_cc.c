/*******************************************************************************
* FILE NAME: ws_cc.c
*
* DESCRIPTION:
*
* USAGE:
*
*******************************************************************************/

#include <stdio.h>
#include "ifi_aliases.h"
#include "ifi_default.h"
#include "ifi_utilities.h"
#include "ws_includes.h"
#include "user_routines.h"
#include "delays.h"       /*defined locally*/
#include "ws_io.h"
#include "serial_ports.h"
#include "ws_cc.h"
#include "ws_autonomous.h"
#include "ws_lift.h"
#include "ws_encoder.h"


/* Use these defines to dump debug info from the CC in input_data */
/*#define CC_DEBUG_RAYX*/
/*#define CC_DEBUG_RAYY*/
/*#define CC_DEBUG_QUAD_ERRS*/



/*******************************************************************************
* FUNCTION NAME: cc_set_position
* PURPOSE:       Tell the CC the current position of the robot.
* CALLED FROM:
* ARGUMENTS:
* RETURNS:
*******************************************************************************/
UINT8 cc_set_position(UINT8 starting_position_id)
{
  CcReturnType   cc_ret_val = CC_SUCCESS;
  unsigned char  ack_data[CC_RESP_TETRA_CHOICE_SIZE];

  cc_set_lift_angle();

  if (readCcReg(&(ack_data[0]), CC_CMD_SET_POSITION, starting_position_id,
                CC_RESP_STARTING_POS_SIZE, CC_LOOP_CNT_TIMEOUT) <
      CC_RESP_STARTING_POS_SIZE)
  {
    /* No response from the CC */
    cc_ret_val = CC_TIMEOUT;
    printf("CC TO4 ");
  }
  else
  {
    /*
    printf("pos_data=%X ", (int)ack_data[0]);
    */
    /* ack_data is |cmd|, verify the data */
    if (ack_data[0] != CC_ACK_STARTING_POS)
    {
      cc_ret_val = CC_FAIL;
    }
  }

  return (cc_ret_val);
}


/*******************************************************************************
* FUNCTION NAME: cc_set_lift_angle
* PURPOSE:       Tell the CC the angle of the lift
* CALLED FROM:
* ARGUMENTS:
* RETURNS:
*******************************************************************************/
void cc_set_lift_angle(void)
{
  UINT16 lift_encoder;
  UINT8  lift_angle_brads;

  /* convert lift angle to BRADS */
  lift_encoder = get_tilt_encoder_count();

  //printf("lift encoder %d ", lift_encoder);

  if (lift_encoder < TILT_POSITION_STRAIGHT_UP)
  {
    lift_encoder = TILT_POSITION_STRAIGHT_UP;
  }

  lift_angle_brads = ((lift_encoder - TILT_POSITION_STRAIGHT_UP) * 20) /
                     (TILT_DOWN_ENCODER_VAL - TILT_POSITION_STRAIGHT_UP);
  //printf("brads %d\r", lift_angle_brads);

  /* send starting postition to CC */
  Write_Serial_Port_Two(CC_CMD_SET_LIFT_ANGLE);
  Delay10TCY();

  /* 8bit lift_angle */
  Write_Serial_Port_Two(lift_angle_brads);

  return;
}


/*******************************************************************************
* FUNCTION NAME: cc_set_track_object
* PURPOSE:       Tell the CC the current position of the robot.
* CALLED FROM:
* ARGUMENTS:
* RETURNS:
*******************************************************************************/
UINT8 cc_set_track_object(UINT8 object_id)
{
  CcReturnType   cc_ret_val = CC_SUCCESS;
  unsigned char  ack_data[CC_RESP_TETRA_CHOICE_SIZE];

  UINT8 res;

  res = readCcReg(&(ack_data[0]), CC_CMD_SET_TRACK_OBJECT, object_id,
                CC_RESP_TETRA_CHOICE_SIZE, CC_LOOP_CNT_TIMEOUT);

  if (res < CC_RESP_TETRA_CHOICE_SIZE)
  {
    /* No response from the CC */
    cc_ret_val = CC_TIMEOUT;
    printf("CC TO3 ");
  }
  else
  {
    /*
    printf("obj_data=%02X%02X ", (int)ack_data[0], (int)ack_data[1]);
    */
    /* ack_data is |cmd|obj|, verify the data */
    if ((ack_data[0] != CC_ACK_TRACK_OBJECT) ||
        (ack_data[1] != object_id))
    {
      cc_ret_val = CC_FAIL;
    }
  }

  return (cc_ret_val);
}


/*******************************************************************************
* FUNCTION NAME: cc_set_camera_center
* PURPOSE:       Tell the CC to set the camera to center position
* CALLED FROM:
* ARGUMENTS:
* RETURNS:
*******************************************************************************/
void cc_set_camera_pos(UINT8 cmd)
{
  Write_Serial_Port_Two(cmd);

  return;
}


/*******************************************************************************
* FUNCTION NAME: cc_gyro_calibrate
* PURPOSE:       Cause the CC to recalibrate the gyro to get the current
*                center position of the gyro.  Make sure the robot isn't moving
*                before calling the function.
* CALLED FROM:
* ARGUMENTS:
* RETURNS:
*******************************************************************************/
void cc_gyro_calibrate(void)
{
    /* recalibrate gyro (send twice as a double check) */
    Write_Serial_Port_Two(RECALIBRATE_GYRO);  /* 8bit theta */
    Write_Serial_Port_Two(RECALIBRATE_GYRO);  /* 8bit theta */

    return;
}


/*******************************************************************************
* FUNCTION NAME: cc_get_dist_hdg
* PURPOSE:       Read the distance & heading to the object from the CC and
*                the robot's orientation
* CALLED FROM:
* ARGUMENTS:
* RETURNS:
*******************************************************************************/
UINT8 cc_get_dist_hdg(DistHdgType *p_curr_dist_hdg, UINT8 cmd)
{
  CcReturnType cc_ret_val = CC_SUCCESS;

  unsigned char dist_hdg_data[CC_RESP_DIST_HDG_SIZE];
  UINT16 x, y;

  cc_set_lift_angle();

  /* request Distance & Heading to target */
  /*printf("Send1 ");*/
  if (readCcReg(&(dist_hdg_data[0]), cmd, 0,
                CC_RESP_DIST_HDG_SIZE, CC_LOOP_CNT_TIMEOUT) <
      CC_RESP_DIST_HDG_SIZE)
  {
    /* No response from the CC...tell the auton code not to run */
    cc_ret_val = CC_TIMEOUT;
    printf("CC TO1 ");
  }
  else
  {
    /* dist_hdg_data is |Dhigh|Dlow|Hhigh|Hlow|Orient, break out the data */
    /*
    printf("dist_hdg_data=%02X%02X%02X%02X%02X%02X%02X%02X%02X ",
           dist_hdg_data[0], dist_hdg_data[1], dist_hdg_data[2],
           dist_hdg_data[3], dist_hdg_data[4], dist_hdg_data[5],
           dist_hdg_data[6], dist_hdg_data[7], dist_hdg_data[8]);
    */

    /* Dist is byte 0 and 1 */
    p_curr_dist_hdg->dist = dist_hdg_data[1];
    p_curr_dist_hdg->dist = p_curr_dist_hdg->dist +
                            ((int)dist_hdg_data[0] << 8);

    /* Heading is byte 2 and 3 */
    p_curr_dist_hdg->hdg = dist_hdg_data[3];
    p_curr_dist_hdg->hdg = p_curr_dist_hdg->hdg +
                           ((int)dist_hdg_data[2] << 8);

    /* Orientation is byte 4 */
    p_curr_dist_hdg->orient = dist_hdg_data[4];

#if 0
    /* x is byte 5 and 6 */
    x = dist_hdg_data[6];
    x = x + ((int)dist_hdg_data[5] << 8);

    /* y is byte 7 and 8 */
    y = dist_hdg_data[8];
    y = y + ((int)dist_hdg_data[7] << 8);
#endif

    /* Print out various CC debug information */
    print_cc_debug();

    if ((p_curr_dist_hdg->hdg > 256) || (p_curr_dist_hdg->dist > 1000))
    {
      cc_ret_val = CC_TIMEOUT;
      printf("CC BV01");
    }
  }

  /*
  printf("D=%d H=%d O=%d x=%d y=%d\r", p_curr_dist_hdg->dist,
         p_curr_dist_hdg->hdg, p_curr_dist_hdg->orient, x, y);
  */

  return(cc_ret_val);
}


/*******************************************************************************
* FUNCTION NAME: cc_get_tetra_pos
* PURPOSE:       Read the starting tetra positions from the CC
* CALLED FROM:
* ARGUMENTS:
* RETURNS:
*******************************************************************************/
UINT8 cc_get_tetra_pos(TetraPosType *p_tetra_pos)
{
  CcReturnType cc_ret_val = CC_SUCCESS;

  unsigned char tetra_pos_data[CC_RESP_TETRA_POS_SIZE];

  /* request starting tetra positions */
  /*printf("Send2 ");*/
  if (readCcReg(&(tetra_pos_data[0]), CC_CMD_REQ_TETRA_POS, 0,
                CC_RESP_TETRA_POS_SIZE, CC_LOOP_CNT_TIMEOUT) <
      CC_RESP_TETRA_POS_SIZE)
  {
    /* No response from the CC...tell the auton code not to run */
    cc_ret_val = CC_TIMEOUT;
    printf("CC TO2 ");
  }
  else
  {
    /* tetra_pos_data is |T1|T2|, break out the data */
    /*
    printf("tetra_pos_data=%X%X ", (int)tetra_pos_data[0],
           (int)tetra_pos_data[1]);
    */

    /* Tetra 1 is byte 0 */
    p_tetra_pos->tetra1 = tetra_pos_data[0];

    /* Tetra 2 is byte 1 */
    p_tetra_pos->tetra2 = tetra_pos_data[1];
  }

  /*
  printf("T1=%d T2=%d\r", (int)p_tetra_pos->tetra1, (int)p_tetra_pos->tetra2);
  */

  return(cc_ret_val);
}

/*******************************************************************************
* FUNCTION NAME: cc_get_encoder_vals
* PURPOSE:       Read the encoder values from the CC
* CALLED FROM:
* ARGUMENTS:
* RETURNS:
*******************************************************************************/
UINT8 cc_get_encoder_vals(EncoderValsType *p_encoder_vals)
{
  CcReturnType cc_ret_val = CC_SUCCESS;

  unsigned char encoder_vals_data[CC_RESP_ENCODER_VAL_SIZE];

  /* request encoder values */
  if (readCcReg(&(encoder_vals_data[0]), CC_CMD_REQ_ENCODER, 0,
                CC_RESP_ENCODER_VAL_SIZE, CC_LOOP_CNT_TIMEOUT) <
      CC_RESP_ENCODER_VAL_SIZE)
  {
    /* No response from the CC...tell the auton code not to run */
    cc_ret_val = CC_TIMEOUT;
    printf("CC Encoder TO");
  }
  else
  {
    /* encoder_vals_data is |Left|Right|Orient, break out the data */
    /*
    printf("encoder_vals_data=%X%X%X ", (int)encoder_vals_data[0],
           (int)encoder_vals_data[1],(int)encoder_vals_data[2]);
    */

    /* Left encoder is byte 0 */
    p_encoder_vals->left = encoder_vals_data[0];

    /* Right encoder is byte 1 */
    p_encoder_vals->right = encoder_vals_data[1];

    /* Orient is byte 2 */
    p_encoder_vals->orient = encoder_vals_data[2];
  }

  /*
  printf("L=%d R=%d O=%d\r",
          (int)p_encoder_vals->left,
          (int)p_encoder_vals->right,
          (int)p_encoder_vals->orient);
  */

  return(cc_ret_val);
}


/*******************************************************************************
* FUNCTION NAME: readCcReg
* PURPOSE:       Read a register on the CC.
* CALLED FROM:
* ARGUMENTS:
*            p_data    - Pointer to the buffer where the contents of the reg will be put
*            cmd       - Which specific register is being read
*            resp_size - Size of the register being read
*            timeout   - Timeout to wait for each char of the register being read
* RETURNS:
*            Number of bytes read from the register
*******************************************************************************/
UINT8 readCcReg(unsigned char* p_data, UINT8 cmd, UINT8 data, UINT8 resp_size,
                UINT16 timeout_per_char)
{
  UINT8  byte_recvd;    /* Was a byte recvd in this loop */
  UINT8  bytes_recvd=0; /* How many bytes were recvd total */
  UINT8  i;
  UINT16    loop_count;

  /* Clear any extra chars out from the input buffer */
  clear_serial_port_two_rx();

  /* Send the command */
  Write_Serial_Port_Two(cmd);

  if (data != 0)
  {
    Delay10TCY();
    Write_Serial_Port_Two(data);
  }

  /* Now recv all the expected response data, break out after all the
   * expected bytes are recvd or until we don't recv a byte.
   */
  byte_recvd=1;   /* Init to 1 so the first loop works */
  for (i=0; (i < resp_size) && (byte_recvd == 1); i++)
  {
    loop_count=timeout_per_char;
    byte_recvd = 0;

    do
    {
      /* only read from buffer if there is data waiting */
      if (Serial_Port_Two_Byte_Count() > 0)
      {
        /* put data recieved into byte_recvd */
        p_data[i] = Read_Serial_Port_Two();
        byte_recvd = 1;
      }

      loop_count--;
    } while (byte_recvd == 0 && loop_count > 0);

    /* Add the byte we recvd to the total count */
    bytes_recvd += byte_recvd;
  }

  return(bytes_recvd);
}

/*******************************************************************************
* FUNCTION NAME: print_cc_debug
* PURPOSE:       Print out various debug information from the CC.  The info
*                printed is controled by various #defines
* CALLED FROM:
* ARGUMENTS:
* RETURNS:
*******************************************************************************/
void print_cc_debug(void)
{
/* TEMP DEBUG, GET THE QUAD DECODE ERRORS */
#ifdef CC_DEBUG_QUAD_ERRS
  {
    unsigned char quad_errs;
    if(readCcReg(&(quad_errs), CC_CMD_REQ_QUAD_ERRS, 0,
                 CC_RESP_QUAD_ERRS_SIZE, CC_LOOP_CNT_TIMEOUT)
                                              < CC_RESP_QUAD_ERRS_SIZE)
    {
       /* No response from the CC...tell the auton code not to run */
       printf("CC TOQERR ");
    }
    printf("QERR=%d ", (int)quad_errs);
  }
#endif

/* TEMP DEBUG, GET THE FULL Y DATA */
#ifdef CC_DEBUG_RAYY
  {
    unsigned char rawy[CC_RESP_RAWY_SIZE];
    if(readCcReg(&(rawy[0]), CC_CMD_REQ_RAW_Y, 0,
                 CC_RESP_RAWY_SIZE, CC_LOOP_CNT_TIMEOUT)
                                             < CC_RESP_RAWY_SIZE)
   {
       /* No response from the CC...tell the auton code not to run */
       printf("CC TOYRAW ");
    }
    printf("RAWY=%2X%2X%2X%2X ",(int)rawy[0], (int)rawy[1], (int)rawy[2],
           (int)rawy[3]);
  }
#endif

/* TEMP DEBUG, GET THE FULL X DATA */
#ifdef CC_DEBUG_RAYX
  {
    unsigned char rawx[CC_RESP_RAWX_SIZE];
    if(readCcReg(&(rawx[0]), CC_CMD_REQ_RAW_X, 0,
                 CC_RESP_RAWX_SIZE, CC_LOOP_CNT_TIMEOUT)
                                              < CC_RESP_RAWX_SIZE)
    {
       /* No response from the CC...tell the auton code not to run */
       printf("CC TOXRAW ");
    }
    printf("RAWX=%2X%2X%2X%2X ",(int)rawx[0], (int)rawx[1], (int)rawx[2],
           (int)rawx[3]);
  }
#endif

  return;
}

