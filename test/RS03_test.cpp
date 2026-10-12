#include "RS03_Driver.h"
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
    std::vector<uint8_t> motor_id = {0x7F}; // デフォルト ID = 127
    uint8_t master_id = 0xFD;              // ホスト ID = 253

    RS03Driver motor("can0", motor_id, master_id);
    RS03Feedback fb;

    KeyboardReader keyboard;
    
    float value = 0.0f;
    
    if (motor.init_can() < 0) {
        std::cerr << "CAN init failed\n";
        return -1;
    }

    std::cout << "==================================================" << std::endl;
    std::cout << "  ROBSTRIDE 03 (RS03) Driver Interactive Test Demo" << std::endl;
    std::cout << "==================================================" << std::endl;
    std::cout << "操作方法:\n  'i' : 位置 +0.5 rad\n  'k' : 位置 -0.5 rad\n  'q' : 終了\n";

    // 起動時の故障クリア
    motor.clear_fault(motor_id[0]);
    usleep(100000);

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
        
        // PP (Profile Position) モードで位置指定（最大速度 2.0 rad/s, 加速度 5.0 rad/s^2）
        motor.commandPP(value, 2.0f, 5.0f, motor_id[0]);
        motor.recv_feedback(fb, motor_id[0], 20);
        
        std::cout << "pos=" << fb.position
                  << " vel=" << fb.velocity
                  << " tq="  << fb.torque
                  << " T="   << fb.temperature
                  << " mode="<< int(fb.mode_state)
                  << " fault_bits=0x" << std::hex << int(fb.fault_bits)
                  << std::dec << std::endl;
    }

    // 終了処理（モーター脱力）
    motor.disable_motor(motor_id[0]);
    motor.close_can();
    std::cout << "\nMotor disabled and CAN closed safely.\n";
    return 0;
}
