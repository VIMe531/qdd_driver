#include "RS03_id_def.h"
#include "RS03_Driver.h"
#include <vector>
#include <cstdint>
#include <algorithm>
#include <iostream>

// ROBSTRIDE 03 (RS03) パラメータ限界値
#define P_MIN   -12.57f
#define P_MAX    12.57f
#define V_MIN   -20.0f
#define V_MAX    20.0f
#define KP_MIN    0.0f
#define KP_MAX 5000.0f
#define KD_MIN    0.0f
#define KD_MAX  100.0f
#define T_MIN   -60.0f
#define T_MAX    60.0f

static int float_to_uint(float x, float x_min, float x_max, int bits) {
    float span   = x_max - x_min;
    float offset = x_min;
    if (x > x_max) x = x_max;
    else if (x < x_min) x = x_min;
    return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

static float uint_to_float(uint16_t x, float x_min, float x_max, int bits) {
    float span   = x_max - x_min;
    float offset = x_min;
    return (float)x * span / (float)((1 << bits) - 1) + offset;
}

int findUint8IndexRS03(const std::vector<uint8_t>& data, uint8_t target) {
    auto it = std::find(data.begin(), data.end(), target);
    if (it == data.end()) {
        return -1;
    }
    return static_cast<int>(std::distance(data.begin(), it));
}

// コンストラクタ
RS03Driver::RS03Driver(const char* can_iface, std::vector<uint8_t> motor_id, uint16_t master_id) {
    if (can_iface) {
        this->RS03_CAN_IFACE = can_iface;
    }
    this->motor_id  = motor_id;
    this->master_id = master_id;
}

// デストラクタ
RS03Driver::~RS03Driver(void) {
    if (sock >= 0) {
        close_can();
    }
}

// initialize can interface
int RS03Driver::init_can(const char* ifname) {
    const char* iface = ifname ? ifname : RS03_CAN_IFACE;
    struct sockaddr_can addr;
    struct ifreq ifr;

    sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sock < 0) {
        std::cerr << "Error opening CAN socket\n";
        return -1;
    }

    std::strncpy(ifr.ifr_name, iface, IFNAMSIZ - 1);
    if (ioctl(sock, SIOCGIFINDEX, &ifr) < 0) {
        std::cerr << "Error in ioctl finding CAN interface\n";
        close(sock);
        sock = -1;
        return -1;
    }

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        std::cerr << "Error binding CAN socket\n";
        close(sock);
        sock = -1;
        return -1;
    }

    std::cout << "CAN interface " << iface << " initialized successfully\n";
    return 0;
}

// close can interface
void RS03Driver::close_can(void) {
    if (this->sock >= 0) {
        close(this->sock);
        this->sock = -1;
        std::cout << "CAN socket closed\n";
    }
}

uint32_t RS03Driver::build_can_id(uint8_t type, uint8_t target_id) const {
    uint32_t id = 0;
    uint8_t master = master_id & 0xFF;
    id |= (uint32_t(type & 0x1F) << 24);   // Bit28..24 通信タイプ
    id |= (uint32_t(0x00) << 16);          // Bit23..16 予約(0)
    id |= (uint32_t(master) << 8);         // Bit15..8  主機ID
    id |= (uint32_t(target_id) & 0xFF);    // Bit7..0   目標電機ID
    return id;
}

bool RS03Driver::send_frame(uint32_t id, const uint8_t data[8]) {
    if (sock < 0) {
        std::cerr << "CAN socket not initialized\n";
        return false;
    }
    struct can_frame frame;
    frame.can_id = id | CAN_EFF_FLAG; // Extended frame
    frame.can_dlc = 8;
    std::memcpy(frame.data, data, 8);

    ssize_t nbytes = write(sock, &frame, sizeof(struct can_frame));
    if (nbytes != sizeof(struct can_frame)) {
        std::cerr << "Error sending CAN frame\n";
        return false;
    }
    return true;
}

bool RS03Driver::recv_feedback(RS03Feedback &out, uint8_t target_id, int timeout_ms) {
    if (sock < 0) {
        std::cerr << "CAN socket not initialized\n";
        return false;
    }

    struct timeval tv;
    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;

    fd_set rdfs;
    FD_ZERO(&rdfs);
    FD_SET(sock, &rdfs);

    int ret = select(sock + 1, &rdfs, NULL, NULL, &tv);
    if (ret <= 0) {
        return false;
    }

    struct can_frame frame;
    ssize_t nbytes = read(sock, &frame, sizeof(struct can_frame));
    if (nbytes < 0) {
        std::cerr << "Read error on CAN socket\n";
        return false;
    }

    // Parse extended frame ID
    uint32_t id = frame.can_id & CAN_EFF_MASK;
    uint8_t type = (id >> 24) & 0x1F;
    uint8_t motor_id_in_frame = (id >> 8) & 0xFF; // Bit15..8 が送信元モーターID

    if (motor_id_in_frame != target_id) {
        return false;
    }

    if (type == RS03_FEEDBACK) { // type 2
        out.motor_id = motor_id_in_frame;
        out.mode_state = (id >> 22) & 0x03; // Bit22..23
        out.fault_bits = (id >> 16) & 0x3F; // Bit16..21

        uint16_t pos_raw    = (frame.data[0] << 8) | frame.data[1];
        uint16_t vel_raw    = (frame.data[2] << 8) | frame.data[3];
        uint16_t torque_raw = (frame.data[4] << 8) | frame.data[5];
        int16_t temp_raw    = (frame.data[6] << 8) | frame.data[7];

        out.position    = uint_to_float(pos_raw, P_MIN, P_MAX, 16);
        out.velocity    = uint_to_float(vel_raw, V_MIN, V_MAX, 16);
        out.torque      = uint_to_float(torque_raw, T_MIN, T_MAX, 16);
        out.temperature = (float)temp_raw / 10.0f;

        return true;
    }

    return false;
}

bool RS03Driver::write_param_u8(uint16_t index, uint8_t value, uint8_t target_id) {
    uint8_t data[8] = { (uint8_t)(index & 0xFF), (uint8_t)((index >> 8) & 0xFF), 0x00, 0x00, value, 0x00, 0x00, 0x00 };
    uint32_t id = build_can_id(RS03_WRITE_PARAMS, target_id);
    return send_frame(id, data);
}

bool RS03Driver::write_param_float(uint16_t index, float value, uint8_t target_id) {
    uint8_t data[8] = { (uint8_t)(index & 0xFF), (uint8_t)((index >> 8) & 0xFF), 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    std::memcpy(&data[4], &value, sizeof(float));
    uint32_t id = build_can_id(RS03_WRITE_PARAMS, target_id);
    return send_frame(id, data);
}

bool RS03Driver::read_param(uint16_t index, uint8_t target_id) {
    uint8_t data[8] = { (uint8_t)(index & 0xFF), (uint8_t)((index >> 8) & 0xFF), 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    uint32_t id = build_can_id(RS03_READ_PARAMS, target_id);
    return send_frame(id, data);
}

bool RS03Driver::enable_motor(uint8_t target_id) {
    uint8_t data[8] = {0};
    uint32_t id = build_can_id(RS03_ID_ENABLE, target_id); // type 3
    std::cout << "[Enable] transmitted\n";
    return send_frame(id, data);
}

bool RS03Driver::disable_motor(uint8_t target_id) {
    uint8_t data[8] = {0};
    uint32_t id = build_can_id(RS03_ESTOP, target_id); // type 4
    std::cout << "[E-Stop] transmitted\n";
    return send_frame(id, data);
}

bool RS03Driver::clear_fault(uint8_t target_id) {
    uint8_t data[8] = {0};
    data[0] = 1; // Byte0 = 1 で fault clear
    uint32_t id = build_can_id(RS03_ESTOP, target_id);
    std::cout << "[Clear fault] transmitted\n";
    return send_frame(id, data);
}

bool RS03Driver::set_mech_zero(uint8_t target_id) {
    uint8_t data[8] = {0};
    data[0] = 1; // Byte0 = 1 で現在位置をゼロに
    uint32_t id = build_can_id(RS03_SET_MECH_ZERO, target_id); // type 6
    std::cout << "[Set mechanical zero] transmitted\n";
    return send_frame(id, data);
}

bool RS03Driver::change_can_id(uint8_t target_id, uint8_t new_id) {
    uint8_t data[8] = {0};
    uint32_t id = 0;
    uint8_t master = master_id & 0xFF;
    id |= (uint32_t(RS03_MOTOR_ID_CHANGE & 0x1F) << 24);
    id |= (uint32_t(new_id) << 16);
    id |= (uint32_t(master) << 8);
    id |= (uint32_t(target_id) & 0xFF);
    std::cout << "[Change CAN ID] target_id=" << int(target_id) << " -> new_id=" << int(new_id) << "\n";
    return send_frame(id, data);
}

bool RS03Driver::set_run_mode(uint8_t mode, uint8_t target_id) {
    std::cout << "[run_mode=" << int(mode) << "] transmitted\n";
    return write_param_u8(RS03_RUN_MODE, mode, target_id);
}

// 位置モード (PP)
bool RS03Driver::commandPosition(float pos_rad, float limit_speed_rad, uint8_t target_id) {
    if (!set_run_mode(1, target_id)) {
        std::cout << "[set_run_mode:1] failed" << std::endl;
        return false;
    }
    if (!enable_motor(target_id)) {
        std::cout << "[enable_motor] failed" << std::endl;
        return false;
    }
    if (!write_param_float(RS03_LIMIT_SPEED, limit_speed_rad, target_id)) {
        std::cout << "[write_param_float] failed" << std::endl;
        return false;
    }
    return write_param_float(RS03_LOC_REF, pos_rad, target_id);
}

// 速度モード
bool RS03Driver::commandVelocity(float vel_rad_s, float limit_current_A, uint8_t target_id) {
    if (!set_run_mode(2, target_id)) {
        std::cout << "[set_run_mode:2] failed" << std::endl;
        return false;
    }
    if (!enable_motor(target_id))  {
        std::cout << "[enable_motor] failed" << std::endl;
        return false;
    }
    if (!write_param_float(RS03_LIMIT_CURRENT, limit_current_A, target_id)) {
        std::cout << "[write_param_float] failed" << std::endl;
        return false;
    }
    return write_param_float(RS03_SPD_REF, vel_rad_s, target_id);
}

// 電流モード
bool RS03Driver::commandCurrent(float iq_A, uint8_t target_id) {
    if (!set_run_mode(3, target_id)) {
        std::cout << "[set_run_mode:3] failed" << std::endl;
        return false;
    }
    if (!enable_motor(target_id)) {
        std::cout << "[enable_motor] failed" << std::endl;
        return false;
    }
    return write_param_float(RS03_IQ_REF, iq_A, target_id);
}

// MIT制御モード
bool RS03Driver::commandMIT(float pos_rad, float vel_rad_s, float kp, float kd, float torque_Nm, uint8_t target_id) {
    uint8_t data[8];
    uint16_t pos_raw = float_to_uint(pos_rad, P_MIN, P_MAX, 16);
    uint16_t vel_raw = float_to_uint(vel_rad_s, V_MIN, V_MAX, 16);
    uint16_t kp_raw  = float_to_uint(kp, KP_MIN, KP_MAX, 16);
    uint16_t kd_raw  = float_to_uint(kd, KD_MIN, KD_MAX, 16);
    uint16_t t_raw   = float_to_uint(torque_Nm, T_MIN, T_MAX, 16);

    data[0] = (pos_raw >> 8) & 0xFF;
    data[1] = pos_raw & 0xFF;
    data[2] = (vel_raw >> 8) & 0xFF;
    data[3] = vel_raw & 0xFF;
    data[4] = (kp_raw >> 8) & 0xFF;
    data[5] = kp_raw & 0xFF;
    data[6] = (kd_raw >> 8) & 0xFF;
    data[7] = kd_raw & 0xFF;

    uint32_t id = 0;
    id |= (uint32_t(RS03_MIT_CTRL & 0x1F) << 24); // type = 1
    id |= (uint32_t(t_raw) << 8);
    id |= (uint32_t(target_id) & 0xFF);

    return send_frame(id, data);
}

// CSPモード (補間同期位置制御)
bool RS03Driver::commandCSP(float pos_rad, float limit_speed_rad, uint8_t target_id) {
    if (!set_run_mode(5, target_id)) {
        std::cout << "[set_run_mode:5] failed" << std::endl;
        return false;
    }
    if (!enable_motor(target_id)) {
        std::cout << "[enable_motor] failed" << std::endl;
        return false;
    }
    if (!write_param_float(RS03_LIMIT_SPEED, limit_speed_rad, target_id)) {
        std::cout << "[write_param_float] failed" << std::endl;
        return false;
    }
    return write_param_float(RS03_LOC_REF, pos_rad, target_id);
}

// PPモード (台形加減速位置制御)
bool RS03Driver::commandPP(float pos_rad, float vel_max_rad_s, float acc_set_rad_s2, uint8_t target_id) {
    if (!set_run_mode(1, target_id)) {
        std::cout << "[set_run_mode:1] failed" << std::endl;
        return false;
    }
    if (!enable_motor(target_id)) {
        std::cout << "[enable_motor] failed" << std::endl;
        return false;
    }
    if (!write_param_float(RS03_VEL_MAX, vel_max_rad_s, target_id)) {
        std::cout << "[write_param_float:vel_max] failed" << std::endl;
        return false;
    }
    if (!write_param_float(RS03_ACC_SET, acc_set_rad_s2, target_id)) {
        std::cout << "[write_param_float:acc_set] failed" << std::endl;
        return false;
    }
    return write_param_float(RS03_LOC_REF, pos_rad, target_id);
}

bool RS03Driver::set_limit_speed_raw(uint16_t raw_spd, uint8_t target_id) {
    uint8_t data[8] = {
        (uint8_t)(RS03_LIMIT_SPEED & 0xFF), (uint8_t)((RS03_LIMIT_SPEED >> 8) & 0xFF),
        0x00, 0x00,
        (uint8_t)(raw_spd & 0xFF), (uint8_t)((raw_spd >> 8) & 0xFF),
        0x00, 0x00
    };
    std::cout << "[limit_spd(raw)=" << raw_spd << "] transmitted\n";
    uint32_t id = build_can_id(RS03_WRITE_PARAMS, target_id);
    return send_frame(id, data);
}

bool RS03Driver::set_locref_raw(uint16_t raw_pos, uint8_t target_id) {
    uint8_t data[8] = {
        (uint8_t)(RS03_LOC_REF & 0xFF), (uint8_t)((RS03_LOC_REF >> 8) & 0xFF),
        0x00, 0x00,
        (uint8_t)(raw_pos & 0xFF), (uint8_t)((raw_pos >> 8) & 0xFF),
        0x00, 0x00
    };
    std::cout << "[loc_ref(raw)=" << raw_pos << "] transmitted\n";
    uint32_t id = build_can_id(RS03_WRITE_PARAMS, target_id);
    return send_frame(id, data);
}
