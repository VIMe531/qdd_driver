#ifndef GIM81088_DRIVER_H
#define GIM81088_DRIVER_H

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

#include "GIM81088_id_def.h"

// フィードバック用の構造体（必要に応じて拡張）
struct GIM81088Feedback
{
	uint8_t	motor_id;			// モータID
	uint8_t	mode_state;			// モード状態（実装次第）
	uint8_t	fault_bits;			// エラービット
	
	float	position_rad;		// 位置 [rad]
	float	velocity_rad_s;		// 速度 [rad/s]
	float	torque_Nm;			// トルク [Nm]
	float	temperature_degC;	// 温度 [℃]
};

class GIM81088Driver
{
public:
	// コンストラクタ:
	//  can_iface : "can0" など
	//  motor_id  : モータの CAN_ID (0-255想定)
	//  master_id : PC/マイコン側識別用 ID (0-65535想定)
	GIM8108Driver(const char* can_iface, uint8_t motor_id, uint16_t master_id);
	~GIM8108Driver();
	
	// CAN 初期化 / クローズ
	int  init_can(const char* ifname = nullptr);
	void close_can();
	
	// 送信（拡張ID）
	bool send_frame(uint32_t can_id, const uint8_t data[8]);
	
	// フィードバック受信（type=GIM81088_FEEDBACK のフレームのみ拾う想定）
	// timeout_ms < 0 ならブロッキング
	bool recv_feedback(GIM81088Feedback &out, int timeout_ms = 100);
	
	// 汎用パラメータアクセス
	bool write_param_u8   (uint16_t index, uint8_t value);
	bool write_param_float(uint16_t index, float   value);
	bool read_param       (uint16_t index);
	
	// 基本操作
	bool enable_motor();				// 通信タイプ GIM81088_MOTOR_ENABLE
	bool disable_motor();				// 通信タイプ GIM81088_MOTOR_DISABLE
	bool set_mech_zero();				// 通信タイプ GIM81088_ZERO_SET
	bool change_can_id(uint8_t new_id);	// 通信タイプ GIM81088_MOTOR_ID_CHANGE
	
	// run_mode 設定: 0:MIT/運控, 1:位置, 2:速度, 3:電流
	bool set_run_mode(uint8_t mode);
	
	// 位置制御モード:
	//   run_mode=1, enable, limit_speed[rad/s], pos_ref[rad]
	bool commandPosition(float pos_rad, float limit_speed_rad);
	
	// 速度制御モード:
	//   run_mode=2, enable, limit_current[A], spd_ref[rad/s]
	bool commandVelocity(float vel_rad_s, float limit_current_A);
	
	// 電流制御モード:
	//   run_mode=3, enable, iq_ref[A]
	bool commandCurrent(float iq_A);
	
	// MIT制御モード（運控/通信タイプ1）:
	//   run_mode=0, enable, MITフレーム1発
	bool commandMIT(float pos_rad, float vel_rad_s, float kp, float kd, float torque_Nm);

private:
	const char* can_iface_;
	int			sock_;
	uint8_t		motor_id_;
	uint16_t	master_id_;
	
	uint32_t build_can_id(uint8_t type) const;
};

#endif // GIM81088_DRIVER_H
