#ifndef GIM810888_DRIVER_H
#define GIM810888_DRIVER_H

#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

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
  GIM81088Driver(std::string can_iface, std::vector<uint8_t> motor_id);
  ~GIM81088Driver();

  GIM81088Driver(const GIM81088Driver&) = delete;
  GIM81088Driver& operator=(const GIM81088Driver&) = delete;

  int  init_can(const char* ifname = nullptr);
  void close_can();
  bool is_open() const { return sock_ >= 0; }
  bool send_frame(uint16_t can_id, const uint8_t data[8]);
  bool recv_frame(struct can_frame& out_frame, int timeout_ms = 100);
  bool poll(uint8_t node_id, int timeout_ms = 0);

  bool get_last_heartbeat(GIM81088Heartbeat& out) const;
  bool get_last_encoder_estimates(GIM81088EncoderEstimates& out) const;
  bool get_last_iq(GIM81088Iq& out) const;
  bool get_last_bus(GIM81088Bus& out) const;
  bool get_last_mit_feedback(GIM81088MitFeedback& out) const;

  bool setAxisNodeId(uint8_t node_id, uint8_t new_id);
  bool setAxisState(uint8_t node_id, uint32_t requested_state);
  bool setControllerMode(uint8_t node_id, uint32_t control_mode, uint32_t input_mode);
  bool setInputPos(uint8_t node_id, float input_pos_rad, float vel_ff_rad_s, float torque_ff_Nm);
  bool setInputVel(uint8_t node_id, float input_vel_rad_s, float torque_ff_Nm);
  bool setInputTorque(uint8_t node_id, float input_torque_Nm);
  bool setLimits(uint8_t node_id, float vel_limit_rad_s, float current_limit_A);
  bool setTrajVelLimit(uint8_t node_id, float traj_vel_limit_rad_s);
  bool setTrajAccelLimits(uint8_t node_id, float traj_accel_rad_s2, float traj_decel_rad_s2);
  bool setTrajInertia(uint8_t node_id, float traj_inertia);
  bool clearErrors(uint8_t node_id);
  bool reboot(uint8_t node_id);
  bool saveConfiguration(uint8_t node_id);
  bool disableCan(uint8_t node_id);
  bool mitControl(uint8_t node_id, float pos_rad, float vel_rad_s, float kp, float kd, float torque_Nm);
  bool requestEncoderEstimates(uint8_t node_id);

private:
  std::string can_iface_;
  int sock_ = -1;
  std::vector<uint8_t> motor_id;

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

  uint16_t build_can_id(uint8_t node_id, uint16_t cmd_id) const;
  static void pack_u32_le(uint8_t out[4], uint32_t v);
  static void pack_i16_le(uint8_t out[2], int16_t v);
  static void pack_f32_le(uint8_t out[4], float v);

  static uint16_t clamp_u16(int v);
  static uint16_t float_to_uint(float x, float x_min, float x_max, int bits);

  bool parse_frame(uint8_t node_id, const struct can_frame& frame);
  bool parse_heartbeat(uint16_t cmd_id, const struct can_frame& frame);
  bool parse_encoder_estimates(uint16_t cmd_id, const struct can_frame& frame);
  bool parse_iq(uint16_t cmd_id, const struct can_frame& frame);
  bool parse_bus(uint16_t cmd_id, const struct can_frame& frame);
  bool parse_mit_feedback(uint16_t cmd_id, const struct can_frame& frame, uint8_t expected_node_id);
};

#endif // GIM810888_DRIVER_H
