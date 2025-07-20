#include "Cybergear_id_def.h"
#include "Cybergear_Driver.h"

CybergearDriver::CybergearDriver( const char* can, uint32_t id_run, uint32_t id_enable )
{
	this->CAN_IFACE = can;
	this->ID_RUN_MODE = id_run;
	this->ID_ENABLE = id_enable;
}

CybergearDriver::~CybergearDriver( void )
{}

// initialize can interface
int CybergearDriver::init_can(const char* ifname) {
    int s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (s < 0) {
        perror("socket");
        return -1;
    }
    struct ifreq ifr;
    std::strncpy(ifr.ifr_name, ifname, IFNAMSIZ);
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
    return s;
}

// close can interface
void CybergearDriver::close_can( void ) {
    close(this->sock);
}

bool CybergearDriver::send_frame( uint32_t id, const uint8_t data[8] ) {
    struct can_frame frame{};
    frame.can_id  = id | CAN_EFF_FLAG;  // exp_ID flag
    frame.can_dlc = 8;
    std::memcpy(frame.data, data, 8);
    if (write(this->sock, &frame, sizeof(frame)) != sizeof(frame)) {
        perror("write");
        return false;
    }
    usleep(100000);  // 100ms
    return true;
}

bool CybergearDriver::set_run_mode( uint8_t mode ) {
    uint8_t data[8] = { 0x00, 0x70, 0x00, mode, 0,0,0,0 };
    std::cout << "[run_mode=" << int(mode) << "] transmmited\n";
    return send_frame(ID_RUN_MODE, data);
}

bool CybergearDriver::enable_motor( void ) {
    uint8_t data[8] = {0};
    std::cout << "[Enable] transmmited\n";
    return send_frame(ID_ENABLE, data);
}

bool CybergearDriver::set_limit_speed( uint16_t raw_spd ) {
    uint8_t data[8] = {
        0x17, 0x70,
        0x00,
        uint8_t(raw_spd & 0xFF), uint8_t((raw_spd >> 8) & 0xFF),
        0,0,0
    };
    std::cout << "[limit_spd=" << raw_spd << "] transmmited\n";
    return send_frame(ID_RUN_MODE, data);
}

bool CybergearDriver::set_motion( uint16_t raw_pos ) {
    uint8_t data[8] = {
        0x16, 0x70,
        0x00,
        uint8_t(raw_pos & 0xFF), uint8_t((raw_pos >> 8) & 0xFF),
        0,0,0
    };
    std::cout << "[loc_ref=" << raw_pos << "] transmmited\n";
    return send_frame(ID_RUN_MODE, data);
}

