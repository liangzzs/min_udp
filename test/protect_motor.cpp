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
/* c system header files */

/* c++ standard library header files */
#include <iostream>
#include <string>

/* external project header files */
//#include <ros/ros.h>
//#include <sensor_msgs/JointState.h>

/* internal project header files */
//#include "robot_state.h"

UdpComm udp_comm(8888, "192.168.1.10", 10);
udp::ReceiveData udp_receive_data = {};
udp::SendData udp_send_data = {};
//sensor_msgs::JointState joint_state;

int main(int argc, char *argv[])
{
    //ros::init(argc, argv, "test_motor_udp");
    //ros::Rate loop_rate(100);
    // initialization
    if (!udp_comm.init())
    {
        printf("udp init failed\n");
        exit(1);
    }    

    while (true)
    {
        udp_send_data.state = static_cast<uint8_t>(CommBoardState::kNormal);
        ProtectMotors(udp_comm, udp_send_data, udp_receive_data);
        udp_comm.setSendData(udp_send_data);
        udp_comm.send();
        // 添加一些延时防止CPU占用过高
        usleep(10000); // 10ms延时
    }
    return 0;
}