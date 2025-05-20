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
    InitMotors(udp_comm, udp_send_data, udp_receive_data);
    ProtectMotors(udp_comm, udp_send_data, udp_receive_data);

    std::cout << "电机初始化和保护完成，开始主循环..." << std::endl;

    while (true)
    {
        udp_send_data.state = static_cast<uint8_t>(CommBoardState::kNormal);
        udp_comm.setSendData(udp_send_data);
        udp_comm.send();
        if (udp_comm.receive(1000000))
        {
            udp_receive_data = udp_comm.getReceiveData();
            std::cout << "receive successfully" << std::endl;

            std::cout << "Header: " << udp_receive_data.header[0] << ", " << udp_receive_data.header[1] << std::endl;
            std::cout << "State: " << static_cast<int>(udp_receive_data.state) << std::endl;
            for (int i = 0; i < 18; ++i)
            {
                std::cout << "Motor " << i << " - Pos: " << udp_receive_data.udp_motor_receive[i].pos
                          << ", Vel: " << udp_receive_data.udp_motor_receive[i].vel
                          << ", Acc: " << udp_receive_data.udp_motor_receive[i].acc
                          << ", Torque: " << udp_receive_data.udp_motor_receive[i].torque
                          << ", Temp: " << udp_receive_data.udp_motor_receive[i].temp << std::endl;
            }
        }
        //std::cout << "is running" << std::endl;

        usleep(1000000);
    }
    return 0;
}