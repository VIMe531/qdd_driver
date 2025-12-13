#ifndef GIM81088_ID_DEF_H
#define GIM81088_ID_DEF_H

//==================================================
// GIM81088-8 / ODrive CAN-Simple constants
//   11-bit CAN_ID = (node_id << 5) | cmd_id
//   (from SteadyWin GIM81088-8 使用手册 rev1.4)
//==================================================

#include <cstdint>

#define GIM81088_MAKE_CAN_ID(node_id, cmd_id)  ( (uint16_t)(((uint16_t)(node_id) << 5) | ((uint16_t)(cmd_id) & 0x1F)) )

// ---- cmd_id list ----
#define GIM81088_CMD_HEARTBEAT                 0x001
#define GIM81088_CMD_ESTOP                     0x002
#define GIM81088_CMD_GET_ERROR                 0x003
#define GIM81088_CMD_RXSDO                     0x004
#define GIM81088_CMD_TXSDO                     0x005
#define GIM81088_CMD_SET_AXIS_NODE_ID          0x006
#define GIM81088_CMD_SET_AXIS_STATE            0x007
#define GIM81088_CMD_MIT_CONTROL               0x008
#define GIM81088_CMD_GET_ENCODER_ESTIMATES     0x009
#define GIM81088_CMD_GET_ENCODER_COUNT         0x00A
#define GIM81088_CMD_SET_CONTROLLER_MODE       0x00B
#define GIM81088_CMD_SET_INPUT_POS             0x00C
#define GIM81088_CMD_SET_INPUT_VEL             0x00D
#define GIM81088_CMD_SET_INPUT_TORQUE          0x00E
#define GIM81088_CMD_SET_LIMITS                0x00F
#define GIM81088_CMD_START_ANTICOGGING         0x010
#define GIM81088_CMD_SET_TRAJ_VEL_LIMIT        0x011
#define GIM81088_CMD_SET_TRAJ_ACCEL_LIMITS     0x012
#define GIM81088_CMD_SET_TRAJ_INERTIA          0x013
#define GIM81088_CMD_GET_IQ                    0x014
#define GIM81088_CMD_GET_TEMPERATURE           0x015
#define GIM81088_CMD_REBOOT                    0x016
#define GIM81088_CMD_GET_BUS_VOLTAGE_CURRENT   0x017
#define GIM81088_CMD_CLEAR_ERRORS              0x018
#define GIM81088_CMD_SET_LINEAR_COUNT          0x019  // manual name: Set_Linear_Count / Set_Absolute_Position (firmware dependent)
#define GIM81088_CMD_SET_POS_GAIN              0x01A
#define GIM81088_CMD_SET_VEL_GAINS             0x01B
#define GIM81088_CMD_GET_TORQUES               0x01C
#define GIM81088_CMD_GET_POWERS                0x01D
#define GIM81088_CMD_DISABLE_CAN               0x01E
#define GIM81088_CMD_SAVE_CONFIGURATION        0x01F

// ---- Axis State (Set_Axis_State payload) ----
#define GIM81088_AXIS_STATE_UNDEFINED          0
#define GIM81088_AXIS_STATE_IDLE               1
#define GIM81088_AXIS_STATE_STARTUP_SEQUENCE   2
#define GIM81088_AXIS_STATE_FULL_CALIBRATION   3
#define GIM81088_AXIS_STATE_MOTOR_CALIBRATION  4
#define GIM81088_AXIS_STATE_SENSORLESS_CONTROL 5
#define GIM81088_AXIS_STATE_ENCODER_INDEX_SEARCH 6
#define GIM81088_AXIS_STATE_ENCODER_OFFSET_CALIBRATION 7
#define GIM81088_AXIS_STATE_CLOSED_LOOP        8

// ---- Control Mode (Set_Controller_Mode payload) ----
#define GIM81088_CONTROL_MODE_VOLTAGE_CONTROL  0
#define GIM81088_CONTROL_MODE_TORQUE_CONTROL   1
#define GIM81088_CONTROL_MODE_VELOCITY_CONTROL 2
#define GIM81088_CONTROL_MODE_POSITION_CONTROL 3

// ---- Input Mode (Set_Controller_Mode payload) ----
#define GIM81088_INPUT_MODE_INACTIVE           0
#define GIM81088_INPUT_MODE_PASSTHROUGH        1
#define GIM81088_INPUT_MODE_VEL_RAMP           2
#define GIM81088_INPUT_MODE_POS_FILTER         3
#define GIM81088_INPUT_MODE_MIX_CHANNELS       4
#define GIM81088_INPUT_MODE_TRAP_TRAJ          5
#define GIM81088_INPUT_MODE_TORQUE_RAMP        6
#define GIM81088_INPUT_MODE_MIRROR             7
#define GIM81088_INPUT_MODE_TUNING             8
#define GIM81088_INPUT_MODE_MIT                9

#define GIM81088_DEFAULT_NODE_ID               0x01

#endif // GIM81088_ID_DEF_H