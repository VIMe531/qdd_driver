#ifndef GIM8108_8_ID_DEF_H
#define GIM8108_8_ID_DEF_H

//==================================================
// GIM8108-8 / ODrive CAN-Simple 用 定数定義
//==================================================

//--------------------------------------------------
// 11bit CAN-ID の生成ヘルパ
//   CAN_ID = (node_id << 5) | cmd_id
//--------------------------------------------------
#define GIM8108_MAKE_CAN_ID(node_id, cmd_id)  ( ((node_id) << 5) | (cmd_id) )

//--------------------------------------------------
// cmd_id: CAN-Simple コマンド ID 群
//   下位 5bit (0..4) に入る値
//--------------------------------------------------

// モニタ・状態系
#define GIM8108_CMD_HEARTBEAT                 0x001  // Heartbeat (Axis_Error, Axis_State, etc.)
#define GIM8108_CMD_ESTOP                     0x002  // Emergency stop
#define GIM8108_CMD_GET_ERROR                 0x003  // Get_Error
// 0x004, 0x005 は SDO 関連(RxSdo/TxSdo)として予約されている実装が多い

// 軸・ノード設定
#define GIM8108_CMD_SET_AXIS_NODE_ID          0x006  // Set_Axis_Node_ID
#define GIM8108_CMD_SET_AXIS_STATE            0x007  // Set_Axis_State

// MIT 制御
#define GIM8108_CMD_MIT_CONTROL               0x008  // Mit_Control (詳細フォーマットは別途)

// エンコーダ
#define GIM8108_CMD_GET_ENCODER_ESTIMATES     0x009  // Pos_Estimate, Vel_Estimate
#define GIM8108_CMD_GET_ENCODER_COUNT         0x00A  // Shadow_Count, Count_In_Cpr

// 制御モード・入力
#define GIM8108_CMD_SET_CONTROLLER_MODE       0x00B  // Control_Mode, Input_Mode
#define GIM8108_CMD_SET_INPUT_POS             0x00C  // Input_Pos, Vel_FF, Torque_FF
#define GIM8108_CMD_SET_INPUT_VEL             0x00D  // Input_Vel, Torque_FF
#define GIM8108_CMD_SET_INPUT_TORQUE          0x00E  // Input_Torque
#define GIM8108_CMD_SET_LIMITS                0x00F  // Velocity_Limit, Current_Limit

// ゲイン・トルク・バス電圧など
#define GIM8108_CMD_SET_TRAJ_VEL_LIMIT        0x011  // Set_Traj_Vel_Limit
#define GIM8108_CMD_SET_TRAJ_ACCEL_LIMITS     0x012  // Set_Traj_Accel_Limits
#define GIM8108_CMD_SET_TRAJ_INERTIA          0x013  // Set_Traj_Inertia

#define GIM8108_CMD_GET_IQ                    0x014  // Iq_Setpoint, Iq_Measured
#define GIM8108_CMD_GET_TEMPERATURE           0x015  // FET/Motor temperature
#define GIM8108_CMD_REBOOT                    0x016  // Reboot / save / erase / DFU
#define GIM8108_CMD_GET_BUS_VOLTAGE_CURRENT   0x017  // Bus_Voltage, Bus_Current

#define GIM8108_CMD_SET_ABSOLUTE_POSITION     0x019  // Set_Absolute_Position
#define GIM8108_CMD_SET_POS_GAIN              0x01A  // Set_Pos_Gain
#define GIM8108_CMD_SET_VEL_GAINS             0x01B  // Set_Vel_Gains
#define GIM8108_CMD_GET_TORQUES               0x01C  // Get_Torques
#define GIM8108_CMD_GET_POWERS                0x01D  // Get_Powers

//--------------------------------------------------
// Axis State (Set_Axis_State で使う値)
//   odrv0.axis0.requested_state と同じ値
//--------------------------------------------------
#define GIM8108_AXIS_STATE_UNDEFINED          0   // 未定義
#define GIM8108_AXIS_STATE_IDLE               1   // Idle
#define GIM8108_AXIS_STATE_STARTUP_SEQUENCE   2   // Startup sequence (ODrive 世代依存)
#define GIM8108_AXIS_STATE_FULL_CALIBRATION   3   // Motor + Encoder calibration
#define GIM8108_AXIS_STATE_MOTOR_CALIBRATION  4   // Motor only calibration
#define GIM8108_AXIS_STATE_SENSORLESS_CONTROL 5   // Sensorless control
#define GIM8108_AXIS_STATE_ENCODER_INDEX_SEARCH 6 // Index search (必要な場合)
#define GIM8108_AXIS_STATE_ENCODER_OFFSET_CALIBRATION 7
#define GIM8108_AXIS_STATE_CLOSED_LOOP        8   // Closed loop control

//--------------------------------------------------
// Control Mode (Set_Controller_Mode 用)
//   odrv0.axis0.controller.config.control_mode
//--------------------------------------------------
#define GIM8108_CONTROL_MODE_VOLTAGE_CONTROL  0
#define GIM8108_CONTROL_MODE_TORQUE_CONTROL   1
#define GIM8108_CONTROL_MODE_VELOCITY_CONTROL 2
#define GIM8108_CONTROL_MODE_POSITION_CONTROL 3

//--------------------------------------------------
// Input Mode (Set_Controller_Mode 用)
//   odrv0.axis0.controller.config.input_mode
//--------------------------------------------------
#define GIM8108_INPUT_MODE_INACTIVE           0
#define GIM8108_INPUT_MODE_PASSTHROUGH        1  // 直接入力
#define GIM8108_INPUT_MODE_VEL_RAMP           2
#define GIM8108_INPUT_MODE_POS_FILTER         3
#define GIM8108_INPUT_MODE_MIX_CHANNELS       4
#define GIM8108_INPUT_MODE_TRAP_TRAJ          5
#define GIM8108_INPUT_MODE_TORQUE_RAMP        6
#define GIM8108_INPUT_MODE_MIRROR             7
#define GIM8108_INPUT_MODE_TUNING             8
#define GIM8108_INPUT_MODE_MIT                9  // MIT 制御入力

//--------------------------------------------------
// 便利なデフォルト設定
//--------------------------------------------------

// 典型的な node_id の例 (必要に応じてアプリ側で上書き)
#define GIM8108_DEFAULT_NODE_ID               0x01

#endif // GIM8108_8_ID_DEF_H
