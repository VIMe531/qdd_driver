#ifndef RS03_DRIVER_H
#define RS03_DRIVER_H

#include <vector>
#include <iostream>
#include <cstring>
#include <cstdint>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/select.h>

#include "RS03_id_def.h"

struct RS03Feedback {
    uint8_t motor_id;      // フレーム内のモータID
    uint8_t mode_state;    // 0:Reset, 1:Cali, 2:Motor
    uint8_t fault_bits;    // Bit集合 (過温/過流/欠圧など)
    float position;        // [rad]
    float velocity;        // [rad/s]
    float torque;          // [Nm]
    float temperature;     // [℃]
};

class RS03Driver {
public:
    // can_iface : "can0" など
    // motor_id  : モータの CAN_ID (0-127)
    // master_id : PC/マイコン側の ID (任意 0-65535)
    RS03Driver(const char* can_iface, std::vector<uint8_t> motor_id, uint16_t master_id);
    ~RS03Driver(void);

    int init_can(const char* ifname = nullptr);
    void close_can(void);

    bool send_frame(uint32_t id, const uint8_t data[8]);
    bool recv_feedback(RS03Feedback &out, uint8_t target_id, int timeout_ms = 100);

    bool write_param_u8(uint16_t index, uint8_t value, uint8_t target_id);
    bool write_param_float(uint16_t index, float value, uint8_t target_id);
    bool read_param(uint16_t index, uint8_t target_id);

    bool enable_motor(uint8_t target_id);
    bool disable_motor(uint8_t target_id);
    bool clear_fault(uint8_t target_id);
    bool set_mech_zero(uint8_t target_id);
    bool change_can_id(uint8_t target_id, uint8_t new_id);
    bool set_run_mode(uint8_t mode, uint8_t target_id);

    // 制御モード
    bool commandPosition(float pos_rad, float limit_speed_rad, uint8_t target_id);
    bool commandVelocity(float vel_rad_s, float limit_current_A, uint8_t target_id);
    bool commandCurrent(float iq_A, uint8_t target_id);
    bool commandMIT(float pos_rad, float vel_rad_s, float kp, float kd, float torque_Nm, uint8_t target_id);
    bool commandCSP(float pos_rad, float limit_speed_rad, uint8_t target_id);
    bool commandPP(float pos_rad, float vel_max_rad_s, float acc_set_rad_s2, uint8_t target_id);

    bool set_limit_speed_raw(uint16_t raw_spd, uint8_t target_id);
    bool set_locref_raw(uint16_t raw_pos, uint8_t target_id);

private:
    const char* RS03_CAN_IFACE = "can0";
    int sock = -1;
    std::vector<uint8_t> motor_id = {127};
    uint16_t master_id = 0xFD;

    uint32_t build_can_id(uint8_t type, uint8_t target_id) const;
};

#endif // RS03_DRIVER_H
