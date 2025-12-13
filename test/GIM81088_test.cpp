#include "GIM81088_Driver.h"
#include <iostream>

int main() {
    GIM81088Driver motor("can0", 1);
    GIM81088EncoderEstimates encoder;
    motor.init_can();
  
    std::cout << "test point" << std::endl;

    while(true){
        // 受信処理
        motor.poll(100);
  
        // Heartbeatを取り出す
        if (motor.get_last_encoder_estimates(encoder)) {
        // hbを使用
            std::cout << encoder.pos_estimate_rev << std::endl;
        }
    }
  
    motor.close_can();
  }
  