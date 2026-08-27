#include "RS00_Driver.h"
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

int main(void) {
    // 制御対象のモーターIDのリスト（ROBSTRIDE 00 デフォルトは 0x7F = 127）
    std::vector<uint8_t> motor_id = {0x7F};
    uint8_t master_id = 0xFD; // ホスト（マスター）のID

    // RS00Driverクラスのインスタンス化
    RS00Driver motor("can0", motor_id, master_id);
    RS00Feedback fb;

    KeyboardReader keyboard;
    
    float value = 0.0f;
    
    if (motor.init_can() < 0) {
        std::cerr << "CAN init failed\n";
        return -1;
    }

    std::cout << "==================================================" << std::endl;
    std::cout << "  ROBSTRIDE 00 (RS00) Interactive Position Demo   " << std::endl;
    std::cout << "  [i] : Target +0.5 rad   [k] : Target -0.5 rad   " << std::endl;
    std::cout << "  [q] : Quit Program                              " << std::endl;
    std::cout << "==================================================" << std::endl;

    // 最初にエラーをクリアしてクリアランスを確保
    motor.clear_fault(motor_id[0]);
    usleep(200000); // 200ms待機
    
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
        
        // 移植したRS00用の位置制御コマンドを送信（PPモードを適用）
        // 内部で自動的にモード設定・有効化（使能）されます
        motor.commandPosition(value, 1.0f, motor_id[0]);
        
        // フィードバックを受信して表示
        if (motor.recv_feedback(fb, motor_id[0], 20)) {
            std::cout   << " | pos=" << fb.position
                        << " vel=" << fb.velocity
                        << " tq="  << fb.torque
                        << " T="   << fb.temperature
                        << " mode="<< int(fb.mode_state)
                        << " fault_bits=0x" << std::hex << int(fb.fault_bits)
                        << std::dec << std::endl;
        } else {
            std::cout << " | [FB] No data" << std::endl;
        }
    }

    // 安全のため、終了時にモーターを脱力（Disable）
    motor.disable_motor(motor_id[0]);
    motor.close_can();
    std::cout << "\nProgram exited safely.\n";
    return 0;
}