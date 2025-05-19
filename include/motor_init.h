#ifndef MOTOR_INIT_H
#define MOTOR_INIT_H

#include "udp_comm.h"

/**
 * @brief 标定六足机器人电机，使所有电机回到零位置点
 * 
 * @param udp_comm UDP通信对象，用于发送和接收数据
 * @param send_data 发送数据结构，包含电机目标参数
 * @param receive_data 接收数据结构，包含电机当前状态
 */
void InitMotors(UdpComm &udp_comm, udp::SendData &send_data, udp::ReceiveData &receive_data);

#endif // MOTOR_INIT_H