/**
 * @file test_udp_non_blocking.cpp
 * @brief non-blocking UDP communication test
 * @author Haoyu Wang (qrpucp@qq.com)
 * @date 2022-09-19
 *
 * @copyright Copyright (C) 2022.
 *
 */
/* related header files */
#include "udp_comm.h"
#include "motor_init.h"
#include <iostream>
#include <string>
#include <unistd.h>  // 添加 usleep 函数的头文件

/* external project header files */
//#include <ros/ros.h>
//#include <sensor_msgs/JointState.h>

/* internal project header files */
//#include "robot_state.h"

UdpComm udp_comm(7, "192.168.1.10", 10);
udp::ReceiveData udp_receive_data = {};
udp::SendData udp_send_data = {};
//sensor_msgs::JointState joint_state;

int main(int argc, char *argv[])
{
    if (!udp_comm.init())
    {
        printf("udp init failed\n");
        exit(1);
    }    

    while (true)
    {
        udp_send_data.state = static_cast<uint8_t>(CommBoardState::kNormal);
        
        // 调用电机标定函数
        //InitMotors(udp_comm, udp_send_data, udp_receive_data);
        //ProtectMotors(udp_comm, udp_send_data, udp_receive_data);
        InitLegMotors(udp_comm, udp_send_data, udp_receive_data, 4);
        //udp_send_data.udp_motor_send[2].torque = 0.2;

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
        
        // 添加 1 秒延时
        usleep(1000000);  // 1000000 微秒 = 1 秒
    }
    return 0;
}