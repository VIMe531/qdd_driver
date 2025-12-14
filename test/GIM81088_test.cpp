#include "GIM81088_Driver.h"
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

int main() {
    std::vector<uint8_t> gim_id = { 0x0F };

    GIM81088Driver motor("can0", gim_id);
    GIM81088EncoderEstimates encoder;

    KeyboardReader keyboard;
    char key;
    float value = 0.0f;
    bool commu_flag = true;

    if (motor.init_can() < 0) {
		std::cerr << "[init_can] failed\n";
		return -1;
	}

     // 1) エラークリア
    if(!motor.clearErrors(gim_id[0])){
        std::cerr << "[clearErrors] failed\n";
		return -1;
    }

    // 2) 位置制御モードにする
    if(!motor.setControllerMode(gim_id[0], GIM81088_CONTROL_MODE_POSITION_CONTROL, GIM81088_INPUT_MODE_PASSTHROUGH)){
        std::cerr << "[setControllerMode] failed\n";
		return -1;
    }

    // 3) 閉ループへ（CLOSED_LOOP_CONTROL）
    if(!motor.setAxisState(gim_id[0], GIM81088_AXIS_STATE_CLOSED_LOOP)){
        std::cerr << "[setAxisState] failed\n";
		return -1;
    }

    while(true){
        // 受信処理
        motor.poll(100);

        key = keyboard.readKey();

        if (key == 'i') {
            value += 0.5f;
        } else if (key == 'k') {
            value -= 0.5f;
        } else if (key == 'q') {
            break;
        } else {
            continue;
        }

        std::cout << "\rvalue = " << value << std::endl;

        commu_flag = motor.setInputPos(gim_id[0], value, 0.0f, 0.0f);
        if(!commu_flag){
            std::cerr << "[setInputPos] Error\n";
            return -1;
        }
        commu_flag = motor.requestEncoderEstimates(gim_id[0]);
        if(!commu_flag){
            std::cerr << "[requestEncoderEstimates] Error\n";
            return -1;
        }
  
        // Heartbeatを取り出す
        if (motor.get_last_encoder_estimates(encoder)) {
        // hbを使用
            std::cout << encoder.pos_estimate_rev << std::endl;
        }
    }  
    motor.close_can();
}
