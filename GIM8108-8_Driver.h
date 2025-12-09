#ifndef GIM81088_DRIVER_H
#define GIM81088_DRIVER_H

#include <iostream>
#include <cstring>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>

#include "GIM8108-8_id_def.h"

class GIM81088Driver
{
public:
	const char* CAN_IFACE      = "can0";
	uint32_t  ID_RUN_MODE = 0;
	uint32_t  ID_ENABLE = 0;
	int sock = 0;
public:
	GIM81088Driver( const char*, uint32_t, uint32_t );
	~GIM81088Driver( void );
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

#endif // GIM81088_DRIVER_H
