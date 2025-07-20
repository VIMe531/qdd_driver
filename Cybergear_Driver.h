#ifndef CYBERGEAR_DRIVER_H
#define CYBERGEAR_DRIVER_H

#include <iostream>
#include <cstring>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>

#include "Cybergear_id_def.h"

class CybergearDriver
{
public:
	const char* CAN_IFACE      = "can0";
	uint32_t  ID_RUN_MODE = 0;
	uint32_t  ID_ENABLE = 0;
	int sock = 0;
public:
	CybergearDriver( const char*, uint32_t, uint32_t );
	~CybergearDriver( void );
	int init_can( const char* ifname );
	void close_can( void );
	bool send_frame( uint32_t, const uint8_t* );
	bool set_can_id( void );
	bool read_params( void );
	bool write_params( void );
	bool get_feedback( void );
	bool enable_motor( void );
	bool disable_motor( void );
	bool set_mech_zero( void );
	bool set_limit_speed( uint16_t );
	bool set_run_mode( uint8_t );
	bool set_motion( uint16_t );
};

#endif // CYBERGEAR_DRIVER_H
