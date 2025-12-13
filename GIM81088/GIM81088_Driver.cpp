#include "GIM81088_Driver.h"

#include <iostream>
#include <cmath>
#include <algorithm>


// C++11 compatible clamp (std::clamp is C++17)
template <typename T>
static inline T clamp_cxx11(T v, T lo, T hi) {
  return (v < lo) ? lo : (v > hi) ? hi : v;
}

// MIT Control ranges per manual rev1.4
static constexpr float MIT_POS_MIN = -12.5f;
static constexpr float MIT_POS_MAX =  12.5f;
static constexpr float MIT_VEL_MIN = -65.0f;
static constexpr float MIT_VEL_MAX =  65.0f;
static constexpr float MIT_KP_MIN  =   0.0f;
static constexpr float MIT_KP_MAX  = 500.0f;
static constexpr float MIT_KD_MIN  =   0.0f;
static constexpr float MIT_KD_MAX  =   5.0f;
static constexpr float MIT_T_MIN   = -50.0f;
static constexpr float MIT_T_MAX   =  50.0f;

GIM81088Driver::GIM81088Driver(std::string can_iface, uint8_t node_id)
: can_iface_(std::move(can_iface)), node_id_(node_id)
{
}

GIM81088Driver::~GIM81088Driver()
{
  close_can();
}


bool GIM81088Driver::get_last_heartbeat(GIM81088Heartbeat& out) const
{
  if (!hb_valid_) return false;
  out = hb_;
  return true;
}

bool GIM81088Driver::get_last_encoder_estimates(GIM81088EncoderEstimates& out) const
{
  if (!enc_valid_) return false;
  out = enc_;
  return true;
}

bool GIM81088Driver::get_last_iq(GIM81088Iq& out) const
{
  if (!iq_valid_) return false;
  out = iq_;
  return true;
}

bool GIM81088Driver::get_last_bus(GIM81088Bus& out) const
{
  if (!bus_valid_) return false;
  out = bus_;
  return true;
}

bool GIM81088Driver::get_last_mit_feedback(GIM81088MitFeedback& out) const
{
  if (!mit_fb_valid_) return false;
  out = mit_fb_;
  return true;
}

int GIM81088Driver::init_can(const char* ifname)
{
  const char* iface = ifname ? ifname : can_iface_.c_str();

  int s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (s < 0) {
    perror("socket");
    return -1;
  }

  struct ifreq ifr{};
  std::strncpy(ifr.ifr_name, iface, IFNAMSIZ);
  ifr.ifr_name[IFNAMSIZ - 1] = '\0';

  if (ioctl(s, SIOCGIFINDEX, &ifr) < 0) {
    perror("ioctl");
    close(s);
    return -1;
  }

  struct sockaddr_can addr{};
  addr.can_family  = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  if (bind(s, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
    perror("bind");
    close(s);
    return -1;
  }

  // Only 11-bit standard frames
  // (CAN_RAW_FILTER could be set here if desired.)

  sock_ = s;
  return sock_;
}

void GIM81088Driver::close_can()
{
  if (sock_ >= 0) {
    close(sock_);
    sock_ = -1;
  }
}

bool GIM81088Driver::send_frame(uint16_t can_id11, const uint8_t data[8])
{
  if (sock_ < 0) {
    std::cerr << "CAN socket not initialized\n";
    return false;
  }

  struct can_frame f{};
  f.can_id  = (can_id11 & CAN_SFF_MASK);  // standard 11-bit
  f.can_dlc = 8;
  std::memcpy(f.data, data, 8);

  if (write(sock_, &f, sizeof(f)) != (ssize_t)sizeof(f)) {
    perror("write");
    return false;
  }
  usleep(1000);
  return true;
}

bool GIM81088Driver::recv_frame(struct can_frame& out_frame, int timeout_ms)
{
  if (sock_ < 0) return false;

  fd_set rfds;
  FD_ZERO(&rfds);
  FD_SET(sock_, &rfds);

  struct timeval tv{};
  tv.tv_sec  = timeout_ms / 1000;
  tv.tv_usec = (timeout_ms % 1000) * 1000;

  int ret = select(sock_ + 1, &rfds, nullptr, nullptr, (timeout_ms >= 0) ? &tv : nullptr);
  if (ret < 0) {
    perror("select");
    return false;
  } else if (ret == 0) {
    return false; // timeout
  }

  struct can_frame f{};
  ssize_t n = read(sock_, &f, sizeof(f));
  if (n != (ssize_t)sizeof(f)) {
    perror("read");
    return false;
  }
  out_frame = f;
  return true;
}

bool GIM81088Driver::poll(int timeout_ms)
{
  struct can_frame f{};
  if (!recv_frame(f, timeout_ms)) return false;
  return parse_frame(f);
}

uint16_t GIM81088Driver::build_can_id11(uint16_t cmd_id) const
{
  return (uint16_t)GIM81088_MAKE_CAN_ID(node_id_, cmd_id) & 0x7FF;
}

void GIM81088Driver::pack_u32_le(uint8_t out[4], uint32_t v)
{
  out[0] = (uint8_t)(v & 0xFF);
  out[1] = (uint8_t)((v >> 8) & 0xFF);
  out[2] = (uint8_t)((v >> 16) & 0xFF);
  out[3] = (uint8_t)((v >> 24) & 0xFF);
}

void GIM81088Driver::pack_i16_le(uint8_t out[2], int16_t v)
{
  uint16_t uv = (uint16_t)v;
  out[0] = (uint8_t)(uv & 0xFF);
  out[1] = (uint8_t)((uv >> 8) & 0xFF);
}

void GIM81088Driver::pack_f32_le(uint8_t out[4], float v)
{
  static_assert(sizeof(float) == 4, "float must be 32-bit");
  std::memcpy(out, &v, 4);
}

uint16_t GIM81088Driver::clamp_u16(int v)
{
  if (v < 0) return 0;
  if (v > 0xFFFF) return 0xFFFF;
  return (uint16_t)v;
}

uint16_t GIM81088Driver::float_to_uN(float x, float x_min, float x_max, int bits)
{
  if (x > x_max) x = x_max;
  if (x < x_min) x = x_min;
  const float span = x_max - x_min;
  const float norm = (x - x_min) / span;
  const int maxv = (1 << bits) - 1;
  const int v = (int)(norm * (float)maxv + 0.5f);
  return (uint16_t)clamp_cxx11(v, 0, maxv);
}

bool GIM81088Driver::setAxisNodeId(uint8_t new_node_id)
{
  uint8_t d[8]{}; // zero
  pack_u32_le(&d[0], (uint32_t)new_node_id);
  return send_frame(build_can_id11(GIM81088_CMD_SET_AXIS_NODE_ID), d);
}

bool GIM81088Driver::setAxisState(uint32_t requested_state)
{
  uint8_t d[8]{};
  pack_u32_le(&d[0], requested_state);
  return send_frame(build_can_id11(GIM81088_CMD_SET_AXIS_STATE), d);
}

bool GIM81088Driver::setControllerMode(uint32_t control_mode, uint32_t input_mode)
{
  uint8_t d[8]{};
  pack_u32_le(&d[0], control_mode);
  pack_u32_le(&d[4], input_mode);
  return send_frame(build_can_id11(GIM81088_CMD_SET_CONTROLLER_MODE), d);
}

bool GIM81088Driver::setInputPos(float input_pos_rev, float vel_ff_rev_s, float torque_ff_Nm)
{
  uint8_t d[8]{};

  // Input_Pos float32 rev
  pack_f32_le(&d[0], input_pos_rev);

  // Vel_FF int16 in 0.001 rev/s
  const int16_t vel_ff_i16 = (int16_t)clamp_cxx11((int)std::lround(vel_ff_rev_s * 1000.0f), -32768, 32767);
  pack_i16_le(&d[4], vel_ff_i16);

  // Torque_FF int16 in 0.001 Nm
  const int16_t tq_ff_i16 = (int16_t)clamp_cxx11((int)std::lround(torque_ff_Nm * 1000.0f), -32768, 32767);
  pack_i16_le(&d[6], tq_ff_i16);

  return send_frame(build_can_id11(GIM81088_CMD_SET_INPUT_POS), d);
}

bool GIM81088Driver::setInputVel(float input_vel_rev_s, float torque_ff_Nm)
{
  uint8_t d[8]{};
  pack_f32_le(&d[0], input_vel_rev_s);
  pack_f32_le(&d[4], torque_ff_Nm);
  return send_frame(build_can_id11(GIM81088_CMD_SET_INPUT_VEL), d);
}

bool GIM81088Driver::setInputTorque(float input_torque_Nm)
{
  uint8_t d[8]{};
  pack_f32_le(&d[0], input_torque_Nm);
  return send_frame(build_can_id11(GIM81088_CMD_SET_INPUT_TORQUE), d);
}

bool GIM81088Driver::setLimits(float vel_limit_rev_s, float current_limit_A)
{
  uint8_t d[8]{};
  pack_f32_le(&d[0], vel_limit_rev_s);
  pack_f32_le(&d[4], current_limit_A);
  return send_frame(build_can_id11(GIM81088_CMD_SET_LIMITS), d);
}

bool GIM81088Driver::setTrajVelLimit(float traj_vel_limit_rev_s)
{
  uint8_t d[8]{};
  pack_f32_le(&d[0], traj_vel_limit_rev_s);
  return send_frame(build_can_id11(GIM81088_CMD_SET_TRAJ_VEL_LIMIT), d);
}

bool GIM81088Driver::setTrajAccelLimits(float traj_accel_rev_s2, float traj_decel_rev_s2)
{
  uint8_t d[8]{};
  pack_f32_le(&d[0], traj_accel_rev_s2);
  pack_f32_le(&d[4], traj_decel_rev_s2);
  return send_frame(build_can_id11(GIM81088_CMD_SET_TRAJ_ACCEL_LIMITS), d);
}

bool GIM81088Driver::setTrajInertia(float traj_inertia)
{
  uint8_t d[8]{};
  pack_f32_le(&d[0], traj_inertia);
  return send_frame(build_can_id11(GIM81088_CMD_SET_TRAJ_INERTIA), d);
}

bool GIM81088Driver::clearErrors()
{
  uint8_t d[8]{};
  return send_frame(build_can_id11(GIM81088_CMD_CLEAR_ERRORS), d);
}

bool GIM81088Driver::reboot()
{
  uint8_t d[8]{};
  return send_frame(build_can_id11(GIM81088_CMD_REBOOT), d);
}

bool GIM81088Driver::saveConfiguration()
{
  uint8_t d[8]{};
  return send_frame(build_can_id11(GIM81088_CMD_SAVE_CONFIGURATION), d);
}

bool GIM81088Driver::disableCan()
{
  uint8_t d[8]{};
  return send_frame(build_can_id11(GIM81088_CMD_DISABLE_CAN), d);
}

bool GIM81088Driver::mitControl(float pos_rad, float vel_rad_s, float kp, float kd, float torque_Nm)
{
  // Pack as described in the manual:
  // BYTE0-1: pos (16)
  // BYTE2: vel (8 high bits)
  // BYTE3: vel (low 4 bits in [7-4]) | kp (high 4 bits in [3-0])
  // BYTE4: kp (low 8 bits)
  // BYTE5: kd (8 high bits)
  // BYTE6: kd (low 4 bits in [7-4]) | torque (high 4 bits in [3-0])
  // BYTE7: torque (low 8 bits)

  const uint16_t p_int  = float_to_uN(pos_rad,    MIT_POS_MIN, MIT_POS_MAX, 16);
  const uint16_t v_int  = float_to_uN(vel_rad_s,  MIT_VEL_MIN, MIT_VEL_MAX, 12);
  const uint16_t kp_int = float_to_uN(kp,         MIT_KP_MIN,  MIT_KP_MAX,  12);
  const uint16_t kd_int = float_to_uN(kd,         MIT_KD_MIN,  MIT_KD_MAX,  12);
  const uint16_t t_int  = float_to_uN(torque_Nm,  MIT_T_MIN,   MIT_T_MAX,   12);

  uint8_t d[8]{};
  d[0] = (uint8_t)((p_int >> 8) & 0xFF);
  d[1] = (uint8_t)( p_int       & 0xFF);

  d[2] = (uint8_t)((v_int >> 4) & 0xFF);
  d[3] = (uint8_t)(((v_int & 0x0F) << 4) | ((kp_int >> 8) & 0x0F));
  d[4] = (uint8_t)( kp_int & 0xFF);

  d[5] = (uint8_t)((kd_int >> 4) & 0xFF);
  d[6] = (uint8_t)(((kd_int & 0x0F) << 4) | ((t_int >> 8) & 0x0F));
  d[7] = (uint8_t)( t_int & 0xFF);

  return send_frame(build_can_id11(GIM81088_CMD_MIT_CONTROL), d);
}

//========================
// Parsing
//========================

static inline uint16_t get_sff_id(const struct can_frame& f) { return (uint16_t)(f.can_id & CAN_SFF_MASK); }
static inline uint16_t extract_cmd_id(uint16_t can_id11) { return (uint16_t)(can_id11 & 0x1F); }
static inline uint8_t  extract_node_id(uint16_t can_id11) { return (uint8_t)((can_id11 >> 5) & 0x3F); }

bool GIM81088Driver::parse_frame(const struct can_frame& f)
{
  const uint16_t id11 = get_sff_id(f);
  const uint8_t  nid  = extract_node_id(id11);
  const uint16_t cmd  = extract_cmd_id(id11);

  // Most messages are node-specific.
  // Accept MIT feedback where node_id is also in data[0] and can_id11 may still be node-specific.
  if (nid != node_id_ && cmd != GIM81088_CMD_MIT_CONTROL) {
    return false;
  }

  if (parse_heartbeat(cmd, f)) return true;
  if (parse_encoder_estimates(cmd, f)) return true;
  if (parse_iq(cmd, f)) return true;
  if (parse_bus(cmd, f)) return true;
  if (parse_mit_feedback(cmd, f)) return true;

  return false;
}

bool GIM81088Driver::parse_heartbeat(uint16_t cmd_id, const struct can_frame& f)
{
  if (cmd_id != GIM81088_CMD_HEARTBEAT) return false;

  GIM81088Heartbeat hb{};
  // firmware >=0.5.12 format:
  // [0..3] Axis_Error (u32)
  // [4]    Axis_State (u8)
  // [5]    Flags (u8)
  // [6]    Reserved
  // [7]    Life
  std::memcpy(&hb.axis_error, &f.data[0], 4);
  hb.axis_state = f.data[4];
  hb.flags      = f.data[5];
  hb.life       = f.data[7];
  hb_ = hb;
  hb_valid_ = true;
  return true;
}

bool GIM81088Driver::parse_encoder_estimates(uint16_t cmd_id, const struct can_frame& f)
{
  if (cmd_id != GIM81088_CMD_GET_ENCODER_ESTIMATES) return false;

  GIM81088EncoderEstimates e{};
  std::memcpy(&e.pos_estimate_rev,   &f.data[0], 4);
  std::memcpy(&e.vel_estimate_rev_s, &f.data[4], 4);
  enc_ = e;
  return true;
}

bool GIM81088Driver::parse_iq(uint16_t cmd_id, const struct can_frame& f)
{
  if (cmd_id != GIM81088_CMD_GET_IQ) return false;

  GIM81088Iq iq{};
  std::memcpy(&iq.iq_setpoint_A, &f.data[0], 4);
  std::memcpy(&iq.iq_measured_A, &f.data[4], 4);
  iq_ = iq;
  iq_valid_ = true;
  return true;
}

bool GIM81088Driver::parse_bus(uint16_t cmd_id, const struct can_frame& f)
{
  if (cmd_id != GIM81088_CMD_GET_BUS_VOLTAGE_CURRENT) return false;

  GIM81088Bus b{};
  std::memcpy(&b.bus_voltage_V, &f.data[0], 4);
  std::memcpy(&b.bus_current_A, &f.data[4], 4);
  bus_ = b;
  return true;
}

bool GIM81088Driver::parse_mit_feedback(uint16_t cmd_id, const struct can_frame& f)
{
  if (cmd_id != GIM81088_CMD_MIT_CONTROL) return false;

  // motor->host MIT feedback frame described in manual:
  // BYTE0: node id
  // BYTE1-2: position (16)
  // BYTE3: vel high 8
  // BYTE4: vel low4 [7-4], torque high4 [3-0]
  // BYTE5: torque low8
  const uint8_t node = f.data[0];

  uint16_t pos_int = ((uint16_t)f.data[1] << 8) | (uint16_t)f.data[2];
  uint16_t vel_int = ((uint16_t)f.data[3] << 4) | ((uint16_t)(f.data[4] >> 4) & 0x0F);
  uint16_t t_int   = ((uint16_t)(f.data[4] & 0x0F) << 8) | (uint16_t)f.data[5];

  // Conversion per manual:
  // pos_double = pos_int * 25 / 65535 - 12.5
  // vel_double = vel_int * 130 / 4095 - 65
  // t_double   = t_int   * 100 / 4095 - 50
  GIM81088MitFeedback m{};
  m.node_id   = node;
  m.pos_rad   = (float)pos_int * 25.0f / 65535.0f - 12.5f;
  m.vel_rad_s = (float)vel_int * 130.0f / 4095.0f - 65.0f;
  m.torque_Nm = (float)t_int   * 100.0f / 4095.0f - 50.0f;

  mit_fb_ = m;
  return true;
}