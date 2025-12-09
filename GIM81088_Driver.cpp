#include "GIM81088_id_def.h"
#include "GIM81088_Driver.h"
#include <sys/select.h>

#define P_MIN  -12.5f
#define P_MAX   12.5f
#define V_MIN  -30.0f
#define V_MAX   30.0f
#define KP_MIN   0.0f
#define KP_MAX 500.0f
#define KD_MIN   0.0f
#define KD_MAX   5.0f
#define T_MIN  -12.0f
#define T_MAX   12.0f

// float → 16bit unsigned
static int float_to_uint(float x, float x_min, float x_max, int bits)
{
	float span   = x_max - x_min;
	float offset = x_min;
	
	if (x > x_max) x = x_max;
	else if (x < x_min) x = x_min;
	
	return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

// 16bit unsigned → float
static float uint_to_float(uint16_t x, float x_min, float x_max, int bits)
{
	float span   = x_max - x_min;
	float offset = x_min;
	return (float)x * span / (float)((1 << bits) - 1) + offset;
}

// 拡張CAN ID 組み立て
uint32_t GIM81088Driver::build_can_id(uint8_t type) const
{
	uint32_t id = 0;
	id |= ((uint32_t)type		& 0x1F) << 24;	// Bit28-24: 通信タイプ
	id |= ((uint32_t)master_id_	& 0xFFFF) << 8;	// Bit23-8 : master_id
	id |= (uint32_t)(motor_id_	& 0xFF);		// Bit7-0  : motor_id
	return id;
}

GIM81088Driver::GIM81088Driver(const char* can_iface, uint8_t motor_id, uint16_t master_id)
	:	can_iface_(can_iface),
		sock_(-1),
		motor_id_(motor_id),
		master_id_(master_id)
{}

GIM81088Driver::~GIM81088Driver()
{
	if (sock_ >= 0) {
		close_can();
	}
}

int GIM81088Driver::init_can(const char* ifname)
{
	const char* iface = ifname ? ifname : can_iface_;
	
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
	
	sock_ = s;
	return s;
}

void GIM81088Driver::close_can()
{
	if (sock_ >= 0) {
		close(sock_);
		sock_ = -1;
	}
}

bool GIM81088Driver::send_frame(uint32_t id, const uint8_t data[8])
{
	if (sock_ < 0) {
		std::cerr << "CAN socket not initialized\n";
		return false;
	}
	
	struct can_frame frame{};
	frame.can_id  = id | CAN_EFF_FLAG; // 拡張ID
	frame.can_dlc = 8;
	std::memcpy(frame.data, data, 8);
	
	if (write(sock_, &frame, sizeof(frame)) != sizeof(frame)) {
		perror("write");
		return false;
	}
	
	// 必要に応じて少し待つ
	usleep(1000); // 1ms
	return true;
}

bool GIM81088Driver::recv_feedback(GIM8108Feedback &out, int timeout_ms)
{
	if (sock_ < 0) {
		std::cerr << "CAN socket not initialized\n";
		return false;
	}
	
	fd_set readfds;
	FD_ZERO(&readfds);
	FD_SET(sock_, &readfds);
	
	struct timeval tv;
	struct timeval* ptv = nullptr;
	if (timeout_ms >= 0) {
		tv.tv_sec  = timeout_ms / 1000;
		tv.tv_usec = (timeout_ms % 1000) * 1000;
		ptv = &tv;
	}
	
	int ret = select(sock_ + 1, &readfds, nullptr, nullptr, ptv);
	if (ret < 0) {
		perror("select");
		return false;
	} else if (ret == 0) {
		// timeout
		return false;
	}
	
	struct can_frame frame{};
	ssize_t n = read(sock_, &frame, sizeof(frame));
	if (n != sizeof(frame)) {
		perror("read");
		return false;
	}
	
	if (!(frame.can_id & CAN_EFF_FLAG)) {
		// 標準IDは無視
		return false;
	}
	
	uint32_t eff_id = frame.can_id & CAN_EFF_MASK;
	
	// 通信タイプ抽出
	uint8_t type = (eff_id >> 24) & 0x1F;
	if (type != GIM81088_FEEDBACK) {
		return false;
	}
	
	// 上位16bit（Bit23-8）を仮にヘッダとして解釈
	uint16_t header = (eff_id >> 8) & 0xFFFF;
	uint8_t motor_id_in_frame	= header & 0xFF;
	uint8_t fault_bits			= (header >> 8) & 0x3F;
	uint8_t mode_state			= (header >> 14) & 0x03;
	
	if (motor_id_in_frame != motor_id_) {
		return false;
	}
	
	uint16_t raw_pos	= (uint16_t(frame.data[0]) << 8) | frame.data[1];
	uint16_t raw_vel	= (uint16_t(frame.data[2]) << 8) | frame.data[3];
	uint16_t raw_torque	= (uint16_t(frame.data[4]) << 8) | frame.data[5];
	int16_t  raw_temp	= (int16_t)((uint16_t(frame.data[6]) << 8) | frame.data[7]);
	
	float pos	= uint_to_float(raw_pos, P_MIN, P_MAX, 16);
	float vel	= uint_to_float(raw_vel, V_MIN, V_MAX, 16);
	float tq	= uint_to_float(raw_torque,T_MIN, T_MAX, 16);
	float temp	= raw_temp / 10.0f;
	
	out.motor_id			= motor_id_in_frame;
	out.mode_state			= mode_state;
	out.fault_bits			= fault_bits;
	out.position_rad		= pos;
	out.velocity_rad_s		= vel;
	out.torque_Nm			= tq;
	out.temperature_degC	= temp;
	
	return true;
}

// ===== 汎用パラメータアクセス =====

bool GIM81088Driver::write_param_u8(uint16_t index, uint8_t value)
{
	uint8_t data[8] = {
		uint8_t(index & 0xFF),
		uint8_t((index >> 8) & 0xFF),
		0x00,
		value,
		0x00,
		0x00,
		0x00,
		0x00
	};
	uint32_t id = build_can_id(GIM81088_WRITE_PARAMS);
	return send_frame(id, data);
}

bool GIM81088Driver::write_param_float(uint16_t index, float value)
{
	uint8_t data[8] = {
		uint8_t(index & 0xFF),
		uint8_t((index >> 8) & 0xFF),
		0x00,
		0x00,
		0x00,
		0x00,
		0x00,
		0x00
	};
	std::memcpy(&data[4], &value, sizeof(float));
	uint32_t id = build_can_id(GIM81088_WRITE_PARAMS);
	return send_frame(id, data);
}

bool GIM81088Driver::read_param(uint16_t index)
{
	uint8_t data[8] = {
		uint8_t(index & 0xFF),
		uint8_t((index >> 8) & 0xFF),
		0x00,
		0x00,
		0x00,
		0x00,
		0x00,
		0x00
	};
	uint32_t id = build_can_id(GIM81088_READ_PARAMS);
	return send_frame(id, data);
}

// ===== 基本操作 =====

bool GIM81088Driver::enable_motor()
{
	uint8_t data[8] = {0};
	uint32_t id = build_can_id(GIM81088_MOTOR_ENABLE);
	std::cout << "[Enable] transmitted\n";
	return send_frame(id, data);
}

bool GIM81088Driver::disable_motor()
{
	uint8_t data[8] = {0};
	uint32_t id = build_can_id(GIM81088_MOTOR_DISABLE);
	std::cout << "[Disable] transmitted\n";
	return send_frame(id, data);
}

bool GIM81088Driver::set_mech_zero()
{
	uint8_t data[8] = {0};
	data[0] = 1; // 現在位置をゼロとするフラグ
	uint32_t id = build_can_id(GIM81088_ZERO_SET);
	std::cout << "[Set mechanical zero] transmitted\n";
	return send_frame(id, data);
}

bool GIM81088Driver::change_can_id(uint8_t new_id)
{
	uint8_t data[8] = {0};
	
	// Bit16-23 に new_id を載せる例（Cybergear と同様のやり方）
	uint32_t id = 0;
	id |= ((uint32_t)GIM81088_MOTOR_ID_CHANGE & 0x1F) << 24;
	id |= ((uint32_t)new_id & 0xFF) << 16;
	id |= (uint32_t)(motor_id_ & 0xFF);
	
	std::cout << "[Change CAN ID " << int(motor_id_) << " -> " << int(new_id) << "] transmitted\n";
	bool ok = send_frame(id, data);
	if (ok) {
		motor_id_ = new_id;
	}
	return ok;
}

// run_mode 設定
//	0: MIT/運動制御
//	1: 位置
//	2: 速度
//	3: 電流
bool GIM81088Driver::set_run_mode(uint8_t mode)
{
	std::cout << "[run_mode=" << int(mode) << "] transmitted\n";
	return write_param_u8(GIM81088_RUN_MODE, mode);
}

// ===== 高レベル API =====

// 位置モード: run_mode=1 → enable → limit_speed[rad/s] → pos_ref[rad]
bool GIM81088Driver::commandPosition(float pos_rad, float limit_speed_rad)
{
	if (!set_run_mode(GIM81088_POSSITION_MODE))	return false;
	if (!enable_motor())						return false;
	if (!write_param_float(GIM81088_SET_LIMIT_SPEED, limit_speed_rad))	return false;
	return write_param_float(GIM81088_SET_POS_REF, pos_rad);
}

// 速度モード: run_mode=2 → enable → limit_current[A] → spd_ref[rad/s]
bool GIM81088Driver::commandVelocity(float vel_rad_s, float limit_current_A)
{
	if (!set_run_mode(GIM81088_SPEED_MODE))	return false;
	if (!enable_motor())					return false;
	if (!write_param_float(GIM81088_SET_LIMIT_CURRENT, limit_current_A))	return false;
	return write_param_float(GIM81088_SPD_REF, vel_rad_s);
}

// 電流モード: run_mode=3 → enable → iq_ref[A]
bool GIM81088Driver::commandCurrent(float iq_A)
{
	if (!set_run_mode(GIM81088_CURRENT_MODE))	return false;
	if (!enable_motor())						return false;
	return write_param_float(GIM81088_IQ_REF, iq_A);
}

// MIT制御モード: run_mode=0 → enable → 通信タイプ1フレーム
bool GIM81088Driver::commandMIT(float pos_rad, float vel_rad_s, float kp, float kd, float torque_Nm)
{
	// run_mode=0 を MIT/運控とみなす
	if (!set_run_mode(0))		return false;
	if (!enable_motor())		return false;
	
	uint8_t data[8];
	
	int p_int  = float_to_uint(pos_rad,		P_MIN, P_MAX, 16);
	int v_int  = float_to_uint(vel_rad_s,	V_MIN, V_MAX, 16);
	int kp_int = float_to_uint(kp,			KP_MIN, KP_MAX, 16);
	int kd_int = float_to_uint(kd,			KD_MIN, KD_MAX, 16);
	int t_int  = float_to_uint(torque_Nm,	T_MIN, T_MAX, 16);
	
	data[0] = uint8_t((p_int >> 8)	& 0xFF);
	data[1] = uint8_t( p_int		& 0xFF);
	data[2] = uint8_t((v_int >> 8)	& 0xFF);
	data[3] = uint8_t( v_int		& 0xFF);
	data[4] = uint8_t((kp_int >> 8)	& 0xFF);
	data[5] = uint8_t( kp_int		& 0xFF);
	data[6] = uint8_t((kd_int >> 8)	& 0xFF);
	data[7] = uint8_t( kd_int		& 0xFF);
	
	// torque は ID の data 部に載せる実装もあるが、
	// ここでは master_id_ の代わりに t_int を載せるなど、
	// 実機仕様に合わせて必要なら build_can_id を拡張する。
	uint32_t id = build_can_id(GIM81088_CTRL_MODE);
	
	std::cout	<< "[MIT] p=" << pos_rad
				<< " v=" << vel_rad_s
				<< " kp=" << kp
				<< " kd=" << kd
				<< " t=" << torque_Nm << "\n";
				
	return send_frame(id, data);
}
