#ifndef RS00_ID_DEF_H
#define RS00_ID_DEF_H

/*
 * RS00 ID & CAN communication protocol definitions
 * Modified from Cybergear to ROBSTRIDE 00 (RS00)
 */

#define RS00_READ_DEV_ID          0x00
#define RS00_MIT_CTRL             0x01
#define RS00_FEEDBACK             0x02
#define RS00_ID_ENABLE            0x03
#define RS00_ESTOP                0x04
#define RS00_SET_MECH_ZERO        0x06
#define RS00_MOTOR_ID_CHANGE      0x07
#define RS00_READ_PARAMS          0x11
#define RS00_WRITE_PARAMS         0x12
#define RS00_NOTIFY_FAILURE       0x21
#define RS00_CHANGE_BAUDRATE      0x22

// Parameter indexes (0x7000 segment)
#define RS00_RUN_MODE             0x7005
#define RS00_IQ_REF               0x7006
#define RS00_SPD_REF              0x700A
#define RS00_LIMIT_TORQUE         0x700B
#define RS00_CUR_KP               0x7010
#define RS00_CUR_KI               0x7011
#define RS00_CUR_FILT_GAIN        0x7014
#define RS00_LOC_REF              0x7016
#define RS00_LIMIT_SPEED          0x7017
#define RS00_LIMIT_CURRENT        0x7018
#define RS00_GET_MECHPOS          0x7019
#define RS00_GET_IQF              0x701A
#define RS00_GET_MECHVEL          0x701B
#define RS00_GET_VBUS             0x701C
#define RS00_ROTATION             0x701D
#define RS00_POS_KP               0x701E
#define RS00_SPD_KP               0x701F
#define RS00_SPD_KI               0x7020
#define RS00_VEL_MAX              0x7024
#define RS00_ACC_SET              0x7025
#define RS00_DCC_SET              0x702E

#endif // RS00_ID_DEF_H
