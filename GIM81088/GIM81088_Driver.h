#ifndef GIM810888_DRIVER_H
#define GIM810888_DRIVER_H

#include <cstdint>
#include <cstring>
#include <string>

#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/select.h>

#include "GIM81088_id_def.h"

//==================================================
// GIM81088-8 CAN-Simple driver (SocketCAN, 11-bit ID)
//  - CAN_ID = (node_id << 5) | cmd_id
//  - Data layout follows "SteadyWin GIM81088-8 使用手册 rev1.4"
//==================================================

struct GIM81088Heartbeat
{
  uint32_t axis_error = 0;
  uint8_t  axis_state = 0;
  uint8_t  flags = 0;     // firmware >=0.5.12
  uint8_t  life  = 0;     // firmware >=0.5.12
};

struct GIM81088EncoderEstimates
{
  float pos_estimate_rev = 0.0f;
  float vel_estimate_rev_s = 0.0f;
};

struct GIM81088Iq
{
  float iq_setpoint_A = 0.0f;
  float iq_measured_A = 0.0f;
};

struct GIM81088Bus
{
  float bus_voltage_V = 0.0f;
  float bus_current_A = 0.0f;
};

struct GIM81088MitFeedback
{
  uint8_t node_id = 0;
  float pos_rad = 0.0f;
  float vel_rad_s = 0.0f;
  float torque_Nm = 0.0f;
};

class GIM81088Driver
{
public:
  // can_iface: "can0" etc.
  // node_id  : target motor CAN node id
  explicit GIM81088Driver(std::string can_iface = "can0", uint8_t node_id = GIM81088_DEFAULT_NODE_ID);
  ~GIM81088Driver();

  GIM81088Driver(const GIM81088Driver&) = delete;
  GIM81088Driver& operator=(const GIM81088Driver&) = delete;

  int  init_can(const char* ifname = nullptr);
  void close_can();
  bool is_open() const { return sock_ >= 0; }

  void set_node_id(uint8_t node_id) { node_id_ = node_id; }
  uint8_t node_id() const { return node_id_; }

  // Low-level send (11-bit std CAN id)
  bool send_frame(uint16_t can_id11, const uint8_t data[8]);

  // Receive one frame with timeout. Returns true if a frame was read.
  bool recv_frame(struct can_frame& out_frame, int timeout_ms = 100);

  // Poll one frame and update cached status/telemetry.
  // Returns true if a frame was consumed and parsed (known cmd_id).
  bool poll(int timeout_ms = 0);

  // Cached latest data (updated by poll()).
// Returns true if cached data exists.
bool get_last_heartbeat(GIM81088Heartbeat& out) const;
bool get_last_encoder_estimates(GIM81088EncoderEstimates& out) const;
bool get_last_iq(GIM81088Iq& out) const;
bool get_last_bus(GIM81088Bus& out) const;
bool get_last_mit_feedback(GIM81088MitFeedback& out) const;
  //========================
  // CAN-Simple commands
  //========================

  // CMD 0x006: Set_Axis_Node_ID (uint32 Axis_Node_ID)
  bool setAxisNodeId(uint8_t new_node_id);

  // CMD 0x007: Set_Axis_State (uint32 Axis_Requested_State)
  bool setAxisState(uint32_t requested_state);

  // CMD 0x00B: Set_Controller_Mode (uint32 Control_Mode, uint32 Input_Mode)
  bool setControllerMode(uint32_t control_mode, uint32_t input_mode);

  // CMD 0x00C: Set_Input_Pos (float32 Input_Pos[rev], int16 Vel_FF[0.001rev/s], int16 Torque_FF[0.001Nm])
  bool setInputPos(float input_pos_rev, float vel_ff_rev_s = 0.0f, float torque_ff_Nm = 0.0f);

  // CMD 0x00D: Set_Input_Vel (float32 Input_Vel[rev/s], float32 Torque_FF[Nm])
  bool setInputVel(float input_vel_rev_s, float torque_ff_Nm = 0.0f);

  // CMD 0x00E: Set_Input_Torque (float32 Input_Torque[Nm])
  bool setInputTorque(float input_torque_Nm);

  // CMD 0x00F: Set_Limits (float32 Velocity_Limit[rev/s], float32 Current_Limit[A])
  bool setLimits(float vel_limit_rev_s, float current_limit_A);

  // CMD 0x011: Set_Traj_Vel_Limit (float32 Traj_Vel_Limit[rev/s])
  bool setTrajVelLimit(float traj_vel_limit_rev_s);

  // CMD 0x012: Set_Traj_Accel_Limits (float32 accel[rev/s^2], float32 decel[rev/s^2])
  bool setTrajAccelLimits(float traj_accel_rev_s2, float traj_decel_rev_s2);

  // CMD 0x013: Set_Traj_Inertia (float32 inertia[Nm/(rev/s^2)])
  bool setTrajInertia(float traj_inertia);

  // CMD 0x018: Clear_Errors (no payload)
  bool clearErrors();

  // CMD 0x016: Reboot (no payload per rev1.4)
  bool reboot();

  // CMD 0x01F: Save_Configuration (no payload)
  bool saveConfiguration();

  // CMD 0x01E: Disable_Can (no payload) - disables CAN interface on the drive (use with caution)
  bool disableCan();

  // CMD 0x009/0x014/0x017 etc are "motor->host" messages.
  // In CAN-Simple, you typically request them via SDO, but some firmwares stream them.
  // This driver parses them when received.

  //========================
  // MIT Control (CMD 0x008)
  //========================
  // Per manual, CAN MIT uses output-shaft side units:
  //  pos [rad] in [-12.5, 12.5]
  //  vel [rad/s] in [-65, 65]
  //  kp  [0..500]
  //  kd  [0..5]
  //  torque [Nm] in [-50, 50]
  bool mitControl(float pos_rad, float vel_rad_s, float kp, float kd, float torque_Nm);

private:
  std::string can_iface_;
  int sock_ = -1;
  uint8_t node_id_ = GIM81088_DEFAULT_NODE_ID;

  // cached parsed states
bool hb_valid_ = false;
bool enc_valid_ = false;
bool iq_valid_ = false;
bool bus_valid_ = false;
bool mit_fb_valid_ = false;

GIM81088Heartbeat        hb_{};
GIM81088EncoderEstimates enc_{};
GIM81088Iq               iq_{};
GIM81088Bus              bus_{};
GIM81088MitFeedback      mit_fb_{};

  uint16_t build_can_id11(uint16_t cmd_id) const;
  static void pack_u32_le(uint8_t out[4], uint32_t v);
  static void pack_i16_le(uint8_t out[2], int16_t v);
  static void pack_f32_le(uint8_t out[4], float v);

  static uint16_t clamp_u16(int v);
  static uint16_t float_to_uN(float x, float x_min, float x_max, int bits);

  bool parse_frame(const struct can_frame& f);
  bool parse_heartbeat(uint16_t cmd_id, const struct can_frame& f);
  bool parse_encoder_estimates(uint16_t cmd_id, const struct can_frame& f);
  bool parse_iq(uint16_t cmd_id, const struct can_frame& f);
  bool parse_bus(uint16_t cmd_id, const struct can_frame& f);
  bool parse_mit_feedback(uint16_t cmd_id, const struct can_frame& f);
};

#endif // GIM810888_DRIVER_H
