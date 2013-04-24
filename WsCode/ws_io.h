/*******************************************************************************
* FILE NAME: ws_io.h
*
* DESCRIPTION:
*  This is the include file which corresponds to io_code.c
*
*  It contains some aliases and function prototypes used in those files.
*
* USAGE:
*  If you add your own routines to those files, this is a good place to add
*  your custom macros (aliases), type definitions, and function prototypes.
*******************************************************************************/

#ifndef __ws_io_h_
#define __ws_io_h_

/******************************* TYPEDEFS *************************************/

/******************************** MACROS **************************************/

/***************************** DEFINITIONS ************************************/
#define OI_CALIBRATE_ENCODERS    50
#define OI_CALIBRATE_JOYSTICKS   200

#define SHIFT_SWITCH_LOW          1
#define SHIFT_SWITCH_HIGH         0

#define FWING_SWITCH_IN           0
#define FWING_SWITCH_OUT          1

#define BWING_SWITCH_IN           0
#define BWING_SWITCH_OUT          1

#define WING_LOCKED               254
#define WING_UNLOCKED             127

#define PRESSURE_BELOW_120        0
#define PRESSURE_ABOVE_120        1

/**************************************************************
 * Inputs
 **************************************************************/

/***** RC Analog Inputs *****/
#define Analog_in_pressure_sensor        rc_ana_in01
#define Analog_in_02                     rc_ana_in02
#define Analog_in_03                     rc_ana_in03
#define Analog_in_04                     rc_ana_in04
#define Analog_in_05                     rc_ana_in05
#define Analog_in_lift_tilt_pot          rc_ana_in06
#define Analog_in_07                     rc_ana_in07
#define Analog_in_08                     rc_ana_in08
#define Analog_in_09                     rc_ana_in09
#define Analog_in_10                     rc_ana_in10
#define Analog_in_11                     rc_ana_in11
#define Analog_in_12                     rc_ana_in12
#define Analog_in_front_proxy            rc_ana_in13
#define Analog_in_14                     rc_ana_in14
#define Analog_in_15                     rc_ana_in15
#define Analog_in_16                     rc_ana_in16


/***** RC Digital Inputs *****/
#define Dig_in_lift_encoder_1     rc_dig_in01
#define Dig_in_tilt_encoder_1     rc_dig_in02
#define Dig_in_03                 rc_dig_in03
#define Dig_in_04                 rc_dig_in04
#define Dig_in_05                 rc_dig_in05
#define Dig_in_06                 rc_dig_in06
#define Dig_in_lift_encoder_2     rc_dig_in07
#define Dig_in_tilt_encoder_2     rc_dig_in08
#define Dig_in_lift_bottom        rc_dig_in09
#define Dig_in_pressure           rc_dig_in10
#define Dig_in_11                 rc_dig_in11
#define Dig_in_12                 rc_dig_in12
#define Dig_in_13                 rc_dig_in13
#define Dig_in_14                 rc_dig_in14
#define Dig_in_15                 rc_dig_in15
#define Dig_in_16                 rc_dig_in16
#define Dig_in_17                 rc_dig_in17
#define Dig_in_18                 rc_dig_in18


/***** Drive Joystick Analog Inputs *****/
#define Oi_drive_x                    p4_x
#define Oi_drive_y                    p4_y
#define Oi_drive_wing_lock            p4_aux

/***** Drive Joystick Digital Inputs *****/
#define Oi_sw_turbo                   p4_sw_trig /* drive stick trigger */
#define Oi_sw_bwing                   p4_sw_top  /* drive stick left button */
#define Oi_sw_bwing_prev              p4_sw_top_prev
#define Oi_sw_fwing                   p4_sw_aux1 /* drive stick right button */
#define Oi_sw_fwing_prev              p4_sw_aux1_prev


/***** Manipulator Joystick Analog Inputs *****/
#define Oi_lift_height                p2_y
#define Oi_lift_tilt                  p2_x
#define Oi_lift_programs              p2_aux  /* manipulator stick hat */
#define Oi_tilt_auto_pos              p2_aux  /* manipulator stick hat */
#define Oi_lift_tilt_debug            p2_aux  /* manipulator stick hat */
#define Oi_lift_programs_prev         p2_aux_prev
#define Oi_lift_tilt_debug_prev       p2_aux_prev

/***** Manipulator Joystick Digital Inputs *****/
#define Oi_sw_tilt_override           p2_sw_aux1
#define Oi_sw_tilt_override_prev      p2_sw_aux1_prev
#define Oi_sw_lift_override           p2_sw_top
#define Oi_sw_lift_override_prev      p2_sw_top_prev
#define Oi_sw_lift_tilt_move          p2_sw_trig


/***** Button Box Inputs *****/
/* Driver */
#define Oi_sw_shifter                 p3_sw_trig

/* Manipulator */
#define Oi_sw_top_spear_grab          p3_sw_aux2
#define Oi_sw_top_spear_grab_prev     p3_sw_aux2_prev
#define Oi_sw_top_spear_tilt          p1_sw_trig
#define Oi_sw_top_spear_tilt_prev     p1_sw_trig_prev
#define Oi_sw_top_spear_retract       p1_sw_aux2
#define Oi_sw_top_spear_retract_prev  p1_sw_aux2_prev

#define Oi_sw_bot_spear_grab          p3_sw_aux1
#define Oi_sw_bot_spear_grab_prev     p3_sw_aux1_prev
#define Oi_sw_bot_spear_tilt          p1_sw_top
#define Oi_sw_bot_spear_tilt_prev     p1_sw_top_prev
#define Oi_sw_bot_spear_retract       p1_sw_aux1
#define Oi_sw_bot_spear_retract_prev  p1_sw_aux1_prev

/* General */
#define Oi_auto_prog_select           p3_y
#define Oi_auto_pos_select            p3_x
#define Oi_sw_auto_lockin             p3_sw_top
#define Oi_calibrate                  p3_wheel

/* debug buttons */
#define Oi_sw_vision_enable           Oi_sw_top_spear_retract
#define Oi_sw_vision_enable_prev      Oi_sw_top_spear_retract_prev
#define Oi_sw_center_camera           Oi_sw_bot_spear_retract
#define Oi_sw_center_camera_prev      Oi_sw_bot_spear_retract_prev
#define Oi_sw_9090_camera             Oi_sw_bot_spear_tilt
#define Oi_sw_9090_camera_prev        Oi_sw_bot_spear_tilt_prev
#define Oi_sw_lift_debug              Oi_sw_top_spear_grab
#define Oi_sw_lift_debug_prev         Oi_sw_top_spear_grab_prev
#define Oi_sw_tilt_debug              Oi_sw_bot_spear_grab
#define Oi_sw_tilt_debug_prev         Oi_sw_bot_spear_grab_prev
#define Oi_sw_encoder_debug           Oi_sw_top_spear_tilt



/**************************************************************
 * Outputs
 **************************************************************/
/***** RC Digital Outputs *****/
#define Rc_relay_bwing               relay1_fwd   /* blue/black   */
#define Rc_relay_fwing               relay1_rev
#define Rc_relay_grabber_top         relay2_fwd   /* green/black  */
#define Rc_relay_grabber_bottom      relay2_rev
#define Rc_relay_hook_top            relay3_fwd   /* orange/black */
#define Rc_relay_hook_bottom         relay3_rev
#define Rc_relay_shifter_left        relay4_fwd   /* red/black    */
#define Rc_relay_shifter_right       relay4_rev
#define Rc_relay_spear_pos_top       relay5_fwd   /* yellow/black */
#define Rc_relay_spear_pos_not_used  relay5_rev
#define Rc_relay_pump_on             relay6_fwd   /* red/green    */
#define Rc_relay_pump_not_used       relay6_rev
#define Rc_relay_spear_pos_bot_out   relay7_fwd   /* grey/black   */
#define Rc_relay_spear_pos_bot_in    relay7_rev
#define Rc_relay_8_fwd               relay8_fwd   /* black/black  */
#define Rc_relay_8_rev               relay8_rev

/***** RC Analog Outputs *****/
#define Rc_analog_out_r_drive_1           pwm01   /* blue   */
#define Rc_analog_out_r_drive_2           pwm02   /* green  */
#define Rc_analog_out_l_drive_1           pwm03   /* orange */
#define Rc_analog_out_l_drive_2           pwm04   /* red    */
#define Rc_analog_out_lift_height         pwm05   /* yellow */
#define Rc_analog_out_lift_tilt           pwm06   /* grey   */
#define Rc_analog_out_fwing_lock          pwm07   /* white  */
#define Rc_analog_out_bwing_lock          pwm08   /* black  */
#define Rc_analog_out_tilt_ratchet        pwm09   /* brown  */
#define Rc_analog_out_pwm10               pwm10   /* purple */
#define Rc_analog_out_pwm11               pwm11   /* ?      */
#define Rc_analog_out_pwm12               pwm12   /* ?      */
#define Rc_analog_out_pwm13               pwm13   /* ?      */
#define Rc_analog_out_pwm14               pwm14   /* ?      */
#define Rc_analog_out_pwm15               pwm15   /* ?      */
#define Rc_analog_out_pwm16               pwm16   /* ?      */

/**************************************************************
 * LEDs
 **************************************************************/
/* #defines for auton LEDs */
#define LEFT_POS_LED      Pwm2_red
#define CENTER_POS_LED    Pwm1_red
#define RIGHT_POS_LED     Pwm1_green
#define RED_SIDE_LED      Relay1_red
#define BLUE_SIDE_LED     Relay2_red
#define LEAST_BIT_LED     Relay1_green
#define MID_BIT_LED       Relay2_green
#define MOST_BIT_LED      Pwm2_green

#define LIFT_OVERRIDE_LED_POS 4
#define TILT_OVERRIDE_LED_POS 6


/***** Input Scaling *****/
/* drive joystick scaling constants */
#define DRIVE_STICK_X_MIN        25
#define DRIVE_STICK_X_MIDDLE     127
#define DRIVE_STICK_X_MAX        253
#define DRIVE_STICK_Y_MIN        41
#define DRIVE_STICK_Y_MIDDLE     127
#define DRIVE_STICK_Y_MAX        240

/* lift joystick scaling constants */
#define LIFT_STICK_X_MIN        4
#define LIFT_STICK_X_MIDDLE     127
#define LIFT_STICK_X_MAX        253
#define LIFT_STICK_Y_MIN        33
#define LIFT_STICK_Y_MIDDLE     127
#define LIFT_STICK_Y_MAX        254

#define SC_CALIB_STICK_DEADZONE   5

#define BOT_SPEAR_MOVE_COUNT    80

/****************************** STRUCTURES ************************************/

/************************* FUNCTION PROTOTYPES ********************************/
extern void assign_outputs_slow(void);
extern void assign_outputs_fast(void);
extern void joystick_scaling(UINT8 *, UINT8, UINT8, UINT8);
extern void io_print_oi_inputs(void);
extern void io_print_rc_inputs(void);

#endif /* __ws_io_h_ */

