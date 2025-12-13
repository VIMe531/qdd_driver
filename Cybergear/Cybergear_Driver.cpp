#include "Cybergear_id_def.h"
#include "Cybergear_Driver.h"
#include <vector>
#include <cstdint>
#include <algorithm>

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

static int float_to_uint(float x, float x_min, float x_max, int bits)
{
	float span   = x_max - x_min;
	float offset = x_min;
	
	if (x > x_max) x = x_max;
	else if (x < x_min) x = x_min;
	
	return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

static float uint_to_float(uint16_t x, float x_min, float x_max, int bits)
{
	float span   = x_max - x_min;
	float offset = x_min;
	return (float)x * span / (float)((1 << bits) - 1) + offset;
}

int findUint8Index(const std::vector<uint8_t>& data, uint8_t target)
{
    auto it = std::find(data.begin(), data.end(), target);
    if (it == data.end()) {
        return -1;
    }
    return static_cast<int>(std::distance(data.begin(), it));
}

// コンストラクタ
CybergearDriver::CybergearDriver(const char* can_iface, std::vector<uint8_t> motor_id, uint16_t master_id)
{
	if (can_iface) {
		this->CYBERGEAR_CAN_IFACE = can_iface;
	}
	this->motor_id  = motor_id;
	this->master_id = master_id;
}

CybergearDriver::~CybergearDriver(void)
{
	if (sock >= 0) {
		close_can();
	}
}

// initialize can interface
int CybergearDriver::init_can(const char* ifname)
{
    const char* iface = ifname ? ifname : CYBERGEAR_CAN_IFACE;

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

    this->sock = s;
    return this->sock;
}


// close can interface
void CybergearDriver::close_can(void)
{
	if (this->sock >= 0) {
		close(this->sock);
		this->sock = -1;
	}
}

uint32_t CybergearDriver::build_can_id(uint8_t type, uint8_t target_id) const
{
    uint32_t id = 0;
    uint8_t master = master_id & 0xFF;
    id |= (uint32_t(type & 0x1F) << 24);   // Bit28..24 通信タイプ
    id |= (uint32_t(0x00) << 16);          // Bit23..16 予約(0)
    id |= (uint32_t(master) << 8);         // Bit15..8  主機ID
    id |= (uint32_t(target_id) & 0xFF);     // Bit7..0   目标电机ID
    return id;
}

bool CybergearDriver::send_frame(uint32_t id, const uint8_t data[8])
{
	if (sock < 0) {
		std::cerr << "CAN socket not initialized\n";
		return false;
	}
	
	struct can_frame frame {};
	frame.can_id  = id | CAN_EFF_FLAG;  // 拡張IDフラグ
	frame.can_dlc = 8;
	std::memcpy(frame.data, data, 8);
	
	if (write(sock, &frame, sizeof(frame)) != sizeof(frame)) {
		perror("write");
		return false;
	}
	
	usleep(1000);
	return true;
}

bool CybergearDriver::recv_feedback(CybergearFeedback &out, uint8_t target_id, int timeout_ms)
{
	if (sock < 0) {
	    std::cerr << "CAN socket not initialized\n";
	    return false;
	}
	
	// --- select で待つ（タイムアウト付き） ---
	fd_set readfds;
	FD_ZERO(&readfds);
	FD_SET(sock, &readfds);
	
	struct timeval tv;
	if (timeout_ms >= 0) {
		tv.tv_sec  = timeout_ms / 1000;
		tv.tv_usec = (timeout_ms % 1000) * 1000;
	}
	
	int ret = select(sock + 1, &readfds, nullptr, nullptr,
					(timeout_ms >= 0) ? &tv : nullptr);
	if (ret < 0) {
		perror("select");
		return false;
	} else if (ret == 0) {
		// timeout
		return false;
	}
	
	// --- 1フレーム受信 ---
	struct can_frame frame {};
	ssize_t n = read(sock, &frame, sizeof(frame));
	if (n != sizeof(frame)) {
		perror("read");
		return false;
	}
	
	// 拡張IDのみ対象
	if (!(frame.can_id & CAN_EFF_FLAG)) {
		return false;
	}
	
	uint32_t eff_id = frame.can_id & CAN_EFF_MASK;
	
	// 通信タイプを抽出 (Bit28-24)
	uint8_t type = (eff_id >> 24) & 0x1F;
	if (type != CYBERGEAR_FEEDBACK) {
		// 必要であればここで他種フレームも解析してよい
		return false;
	}
	
	// 上位16bit (Bit23-8)
	uint16_t header_data = (eff_id >> 8) & 0xFFFF;
	uint8_t  motor_id_in_frame	= header_data & 0xFF;
	uint8_t  fault_bits			= (header_data >> 8) & 0x3F;  // Bit8-13
	uint8_t  mode_state			= (header_data >> 14) & 0x03; // Bit14-15
	
	// このインスタンスが担当するモータだけ扱いたい場合
	if (motor_id_in_frame != target_id) {
		// 他モータのフィードバックだった場合は必要に応じて無視
		// （マルチモータ対応したければここを変える）
		// ひとまず無視して false を返す
		return false;
	}
	
	// データ部を 16bit にまとめる
	uint16_t raw_pos   = (uint16_t(frame.data[0]) << 8) | frame.data[1];
	uint16_t raw_vel   = (uint16_t(frame.data[2]) << 8) | frame.data[3];
	uint16_t raw_torque= (uint16_t(frame.data[4]) << 8) | frame.data[5];
	int16_t  raw_temp  = (int16_t)((uint16_t(frame.data[6]) << 8) | frame.data[7]);
	
	// 実数に変換（送信時と同じレンジを使用）
	float pos = uint_to_float(raw_pos, P_MIN, P_MAX, 16);
	float vel = uint_to_float(raw_vel, V_MIN, V_MAX, 16);
	float tq  = uint_to_float(raw_torque, T_MIN, T_MAX, 16);
	float temp_deg = raw_temp / 10.0f;
	
	// 構造体へ格納
	out.motor_id = motor_id_in_frame;
	out.mode_state = mode_state;
	out.fault_bits = fault_bits;
	out.position_rad = pos;
	out.velocity_rad_s = vel;
	out.torque_Nm = tq;
	out.temperature_degC = temp_deg;
	
	return true;
}


bool CybergearDriver::write_param_u8(uint16_t index, uint8_t value, uint8_t target_id)
{
	uint8_t data[8] = {
		(uint8_t)(index & 0xFF),
		(uint8_t)((index >> 8) & 0xFF),
		0x00,
		0x00,
		value,
		0x00,
		0x00,
		0x00
	};
	uint32_t id = build_can_id(CYBERGEAR_WRITE_PARAMS, target_id);
	return send_frame(id, data);
}

bool CybergearDriver::write_param_float(uint16_t index, float value, uint8_t target_id)
{
	uint8_t data[8] = {
		(uint8_t)(index & 0xFF),
		(uint8_t)((index >> 8) & 0xFF),
		0x00,
		0x00,
		0x00,
		0x00,
		0x00,
		0x00
	};
	std::memcpy(&data[4], &value, sizeof(float));
	uint32_t id = build_can_id(CYBERGEAR_WRITE_PARAMS, target_id);
	
	std::cout << "id:" << id << std::endl;
	for(int i=0;i<8;i++){
		std::cout << static_cast<uint16_t>(data[i]);
	}
	std::cout << std::endl;
	
	return send_frame(id, data);
}

bool CybergearDriver::read_param(uint16_t index, uint8_t target_id)
{
	uint8_t data[8] = {
		(uint8_t)(index & 0xFF),
		(uint8_t)((index >> 8) & 0xFF),
		0x00,
		0x00,
		0x00,
		0x00,
		0x00,
		0x00
	};
	uint32_t id = build_can_id(CYBERGEAR_READ_PARAMS, target_id);
	return send_frame(id, data);
}

bool CybergearDriver::enable_motor(uint8_t target_id)
{
    uint8_t data[8] = {0};
    uint32_t id = build_can_id(CYBERGEAR_ID_ENABLE, target_id);
    std::cout << "[Enable] transmitted\n";
    return send_frame(id, data);
}


bool CybergearDriver::disable_motor(uint8_t target_id)
{
	uint8_t data[8] = {0};
	uint32_t id = build_can_id(CYBERGEAR_ESTOP, target_id);
	std::cout << "[E-Stop] transmitted\n";
	return send_frame(id, data);
}

bool CybergearDriver::clear_fault(uint8_t target_id)
{
	uint8_t data[8] = {0};
	data[0] = 1; // Byte0 = 1 で fault clear
	uint32_t id = build_can_id(CYBERGEAR_ESTOP, target_id);
	std::cout << "[Clear fault] transmitted\n";
	return send_frame(id, data);
}

bool CybergearDriver::set_mech_zero(uint8_t target_id)
{
	uint8_t data[8] = {0};
	data[0] = 1; // Byte0 = 1 で現在位置をゼロに
	uint32_t id = build_can_id(CYBERGEAR_SET_MECH_ZERO, target_id);
	std::cout << "[Set mechanical zero] transmitted\n";
	return send_frame(id, data);
}

bool CybergearDriver::change_can_id(uint8_t target_id, uint8_t new_id)
{
	uint8_t data[8] = {0};
	
	// PDF 4.1.7: Bit16-23 に new_id を入れる仕様
	// → ここでは data には何も入れず，拡張IDの master_id 部分に new_id を載せる実装も可能．
	// シンプルにするため data は0のまま送る．
	uint32_t id = 0;
	id |= ((uint32_t)CYBERGEAR_MOTOR_ID_CHANGE & 0x1F) << 24;
	id |= ((uint32_t)new_id & 0xFF) << 16;   // 予約: pre-set CAN_ID
	id |= (uint32_t)(target_id & 0xFF);
	
	std::cout << "[Change CAN ID " << int(target_id) << " -> " << int(new_id) << "] transmitted\n";
	bool ok = send_frame(id, data);
	if (ok) {
		int idx = findUint8Index(this->motor_id, target_id);
		if(idx<0){
			return false;
		}
		else{
			motor_id[idx] = new_id;
		}
	}
	return ok;
}

bool CybergearDriver::set_run_mode(uint8_t mode, uint8_t target_id)
{
	std::cout << "[run_mode=" << int(mode) << "] transmitted\n";
	return write_param_u8(CYBERGEAR_RUN_MODE, mode, target_id);
}

// 位置モード: run_mode=1 → enable → limit_spd → loc_ref
bool CybergearDriver::commandPosition(float pos_rad, float limit_speed_rad, uint8_t target_id)
{
	if (!set_run_mode(1, target_id)) {
		std::cout << "[set_run_mode:1] failed" << std::endl;
		return false;
	}
	if (!enable_motor(target_id)) {
		std::cout << "[enable_motor] failed" << std::endl;
		return false;
	}
	if (!write_param_float(CYBERGEAR_LIMIT_SPEED, limit_speed_rad, target_id)) {
		std::cout << "[write_param_float] failed" << std::endl;
		return false;
	}
	return write_param_float(CYBERGEAR_LOC_REF, pos_rad, target_id);
}

// 速度モード: run_mode=2 → enable → limit_cur → spd_ref
bool CybergearDriver::commandVelocity(float vel_rad_s, float limit_current_A, uint8_t target_id)
{
	if (!set_run_mode(2, target_id)) {
		std::cout << "[set_run_mode:2] failed" << std::endl;
		return false;
	}
	if (!enable_motor(target_id))  {
		std::cout << "[enable_motor] failed" << std::endl;
		return false;
	}
	if (!write_param_float(CYBERGEAR_LIMIT_CURRENT, limit_current_A, target_id)) {
		std::cout << "[write_param_float] failed" << std::endl;
		return false;
	}
	return write_param_float(CYBERGEAR_SPD_REF, vel_rad_s, target_id);
}

// 電流モード: run_mode=3 → enable → iq_ref
bool CybergearDriver::commandCurrent(float iq_A, uint8_t target_id)
{
	if (!set_run_mode(3, target_id)) {
		std::cout << "[set_run_mode:3] failed" << std::endl;
		return false;
	}
	if (!enable_motor(target_id)) {
		std::cout << "[enable_motor] failed" << std::endl;
		return false;
	}
	return write_param_float(CYBERGEAR_IQ_REF, iq_A, target_id);
}

// MIT制御モード: 通信タイプ1フレーム
bool CybergearDriver::commandMIT(float pos_rad, float vel_rad_s, float kp, float kd, float torque_Nm, uint8_t target_id)
{
	uint8_t data[8];
	
	int p_int  = float_to_uint(pos_rad,		P_MIN, P_MAX, 16);
	int v_int  = float_to_uint(vel_rad_s,	V_MIN, V_MAX, 16);
	int kp_int = float_to_uint(kp,			KP_MIN, KP_MAX, 16);
	int kd_int = float_to_uint(kd,			KD_MIN, KD_MAX, 16);
	int t_int  = float_to_uint(torque_Nm,	T_MIN, T_MAX, 16);
	
	data[0] = (uint8_t)((p_int >> 8)	& 0xFF);
	data[1] = (uint8_t)( p_int			& 0xFF);
	data[2] = (uint8_t)((v_int >> 8)	& 0xFF);
	data[3] = (uint8_t)( v_int			& 0xFF);
	data[4] = (uint8_t)((kp_int >> 8)	& 0xFF);
	data[5] = (uint8_t)( kp_int			& 0xFF);
	data[6] = (uint8_t)((kd_int >> 8)	& 0xFF);
	data[7] = (uint8_t)( kd_int			& 0xFF);
	
	uint32_t id = build_can_id(CYBERGEAR_MIT_CTRL, target_id);
	// torque は ID の data 部に入る仕様なので，build_can_id の data 部に
	// t_int を渡したい場合は build_can_id を拡張してもよい．
	// ここでは、master_id を torque として使う場合の実装など，
	// 運用方針に合わせて調整する．
	
	std::cout << "[MIT] p=" << pos_rad << " v=" << vel_rad_s << 
				 " kp=" << kp << " kd=" << kd << " t=" << torque_Nm << "\n";
	
	return send_frame(id, data);
}

bool CybergearDriver::set_limit_speed_raw(uint16_t raw_spd, uint8_t target_id)
{
    uint8_t data[8] = {
        (uint8_t)(CYBERGEAR_LIMIT_SPEED & 0xFF),
        (uint8_t)((CYBERGEAR_LIMIT_SPEED >> 8) & 0xFF),
        0x00,
        0x00,
        (uint8_t)(raw_spd & 0xFF),
        (uint8_t)((raw_spd >> 8) & 0xFF),
        0x00,
        0x00
    };
    std::cout << "[limit_spd(raw)=" << raw_spd << "] transmitted\n";
    uint32_t id = build_can_id(CYBERGEAR_WRITE_PARAMS, target_id);
    return send_frame(id, data);
}

bool CybergearDriver::set_locref_raw(uint16_t raw_pos, uint8_t target_id)
{
    uint8_t data[8] = {
        (uint8_t)(CYBERGEAR_LOC_REF & 0xFF),
        (uint8_t)((CYBERGEAR_LOC_REF >> 8) & 0xFF),
        0x00,
        0x00,
        (uint8_t)(raw_pos & 0xFF),
        (uint8_t)((raw_pos >> 8) & 0xFF),
        0x00,
        0x00
    };
    std::cout << "[loc_ref(raw)=" << raw_pos << "] transmitted\n";
    uint32_t id = build_can_id(CYBERGEAR_WRITE_PARAMS, target_id);
    return send_frame(id, data);
}
