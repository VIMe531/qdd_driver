#ifndef CYBERGEAR_DRIVER_H
#define CYBERGEAR_DRIVER_H

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

#include "Cybergear_id_def.h"

struct CybergearFeedback
{
	uint8_t	motor_id;			// フレーム内のモータID
	uint8_t	mode_state;			// 0:Reset, 1:Cali, 2:Motor
	uint8_t	fault_bits;			// Bit集合 (過温/過流/欠圧など)
	
	float	position_rad;		// 出力軸の角度 [rad]
	float	velocity_rad_s;		// 出力軸の角速度 [rad/s]
	float	torque_Nm;			// トルク [Nm]
	float	temperature_degC;	// 温度 [℃]
};

class CybergearDriver
{
public:
	//  can_iface : "can0" など
	//  motor_id  : モータの CAN_ID (0-127)
	//  master_id : PC/マイコン側の ID (任意 0-65535)
	CybergearDriver(const char* can_iface, std::vector<uint8_t> motor_id, uint16_t master_id);
	~CybergearDriver(void);
	
	// CAN 初期化 (成功時: ソケットFD, 失敗時: -1)
	int  init_can(const char* ifname = nullptr);
	void close_can(void);
	bool send_frame(uint32_t can_id, const uint8_t data[8]);
	bool recv_feedback(CybergearFeedback &out, uint8_t target_id, int timeout_ms = 100);
	bool write_param_u8   (uint16_t index, uint8_t value, uint8_t target_id);
	bool write_param_float(uint16_t index, float   value, uint8_t target_id);
	bool read_param       (uint16_t index, uint8_t target_id);  // 受信処理は別途実装想定
	bool enable_motor(uint8_t target_id);      // 通信タイプ 3
	bool disable_motor(uint8_t target_id);     // 通信タイプ 4 (非常停止)
	bool clear_fault(uint8_t target_id);       // 通信タイプ 4 + Byte0 = 1
	bool set_mech_zero(uint8_t target_id);     // 通信タイプ 6
	bool change_can_id(uint8_t target_id, uint8_t new_id); // 通信タイプ 7
	bool set_run_mode(uint8_t mode, uint8_t target_id); // 0:MIT 1:位置 2:速度 3:電流
	bool commandPosition(float pos_rad, float limit_speed_rad, uint8_t target_id);
	bool commandVelocity(float vel_rad_s, float limit_current_A, uint8_t target_id);
	bool commandCurrent (float iq_A, uint8_t target_id);
	bool commandMIT(float pos_rad, float vel_rad_s, float kp, float kd, float torque_Nm, uint8_t target_id);
	bool set_limit_speed_raw(uint16_t raw_spd, uint8_t target_id);
	bool set_locref_raw     (uint16_t raw_pos, uint8_t target_id);

private:
	const char* CYBERGEAR_CAN_IFACE = "can0";
	int						sock = -1;
	std::vector<uint8_t>	motor_id = {1};
	uint16_t				master_id = 0;
	uint32_t build_can_id(uint8_t type, uint8_t target_id) const;
};

#endif // CYBERGEAR_DRIVER_H
