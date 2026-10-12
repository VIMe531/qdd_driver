#ifndef RS03_ID_DEF_H
#define RS03_ID_DEF_H

/*
 * RS03 ID & CAN communication protocol definitions
 * For ROBSTRIDE 03 (RS03) Motor Module
 */

#define RS03_READ_DEV_ID          0x00
#define RS03_MIT_CTRL             0x01
#define RS03_FEEDBACK             0x02
#define RS03_ID_ENABLE            0x03
#define RS03_ESTOP                0x04
#define RS03_SET_MECH_ZERO        0x06
#define RS03_MOTOR_ID_CHANGE      0x07
#define RS03_READ_PARAMS          0x11
#define RS03_WRITE_PARAMS         0x12
#define RS03_NOTIFY_FAILURE       0x21
#define RS03_CHANGE_BAUDRATE      0x22

// Parameter indexes (0x7000 segment)
#define RS03_RUN_MODE             0x7005
#define RS03_IQ_REF               0x7006
#define RS03_SPD_REF              0x700A
#define RS03_LIMIT_TORQUE         0x700B
#define RS03_CUR_KP               0x7010
#define RS03_CUR_KI               0x7011
#define RS03_CUR_FILT_GAIN        0x7014
#define RS03_LOC_REF              0x7016
#define RS03_LIMIT_SPEED          0x7017
#define RS03_LIMIT_CURRENT        0x7018
#define RS03_GET_MECHPOS          0x7019
#define RS03_GET_IQF              0x701A
#define RS03_GET_MECHVEL          0x701B
#define RS03_GET_VBUS             0x701C
#define RS03_ROTATION             0x701D
#define RS03_POS_KP               0x701E
#define RS03_SPD_KP               0x701F
#define RS03_SPD_KI               0x7020
#define RS03_VEL_MAX              0x7024
#define RS03_ACC_SET              0x7025
#define RS03_DCC_SET              0x702E

#endif // RS03_ID_DEF_H
