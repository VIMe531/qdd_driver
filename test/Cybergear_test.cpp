#include "Cybergear_Driver.h"
#include <iostream>
#include <unistd.h>
#include <termios.h>
#include <vector>

class KeyboardReader
{
public:
    KeyboardReader()
    {
        tcgetattr(STDIN_FILENO, &orig_);
        termios raw = orig_;
        raw.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &raw);
    }

    ~KeyboardReader()
    {
        tcsetattr(STDIN_FILENO, TCSANOW, &orig_);
    }

    char readKey()
    {
        char c;
        if (read(STDIN_FILENO, &c, 1) < 0) {
            return 0;
        }
        return c;
    }

private:
    termios orig_;
};

int main( void ){
    std::vector<uint8_t> motor_id = {0x7F};
    uint8_t master_id = 1;

	CybergearDriver motor("can0", motor_id, master_id);
    CybergearFeedback fb;

    KeyboardReader keyboard;
	
	float value = 0.0f;
	
	if (motor.init_can() < 0) {
		std::cerr << "CAN init failed\n";
		return -1;
	}
	
	while (true) {
        char key = keyboard.readKey();

        if (key == 'i') {
            value += 0.5f;
        } else if (key == 'k') {
            value -= 0.5f;
        } else if (key == 'q') {
            break;
        } else {
            continue;
        }

        std::cout << "\rvalue = " << value << "    " << std::flush;
        
        // 位置 value [rad] へ移動（最大速度 1 rad/s）
		motor.commandPosition(value, 1.0f, motor_id[0]);
        motor.recv_feedback(fb, motor_id[0], 20);
		
        std::cout	<< "pos=" << fb.position_rad
							<< " vel=" << fb.velocity_rad_s
							<< " tq="  << fb.torque_Nm
							<< " T="   << fb.temperature_degC
							<< " mode="<< int(fb.mode_state)
							<< " fault_bits=0x" << std::hex << int(fb.fault_bits)
							<< std::dec << std::endl;
    }
}
