/**
 * @file test_udp.cpp
 * @brief non-blocking UDP communication test and motor initialization
 * @author Haowen Liang(1224559437@qq.com)
 * @date 2025-5-20
 *
 * @copyright Copyright (C) 2025.
 *
 */
/* related header files */
#include "udp_comm.h"
#include "motor_init.h"
#include <iostream>
#include <string>
#include <unistd.h>

UdpComm udp_comm(7, "192.168.1.10", 10);
udp::ReceiveData udp_receive_data = {};
udp::SendData udp_send_data = {};


int main(int argc, char *argv[])
{
    if (!udp_comm.init())
    {
        printf("udp init failed\n");
        exit(1);
    }    

    udp_send_data.state = static_cast<uint8_t>(CommBoardState::kNormal);
    //InitMotors(udp_comm, udp_send_data, udp_receive_data);
    //InitLegMotors(udp_comm, udp_send_data, udp_receive_data, 0);
    ProtectMotors(udp_comm, udp_send_data, udp_receive_data);
    // udp_send_data.udp_motor_send[16].torque = -0.35;
    // udp_send_data.udp_motor_send[16].kp = 0.0;
    // udp_send_data.udp_motor_send[16].kd = 0.5;

    //std::cout << "电机初始化和保护完成，开始主循环..." << std::endl;

    while (true)
    {
        udp_send_data.state = static_cast<uint8_t>(CommBoardState::kNormal);
        udp_comm.setSendData(udp_send_data);
        udp_comm.send();
        if (udp_comm.receive(1000000))
        {
            udp_receive_data = udp_comm.getReceiveData();
            std::cout << "Motor  position: " << udp_receive_data.udp_motor_receive[12].pos << std::endl;
        }

        usleep(1000000);
    }
    return 0;
}