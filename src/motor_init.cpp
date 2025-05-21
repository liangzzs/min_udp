/**
 * @file test_udp.cpp
 * @brief non-blocking UDP communication test and motor initialization
 * @author Haowen Liang(1224559437@qq.com)
 * @date 2025-5-20
 *
 * @copyright Copyright (C) 2025.
 *
 */
#include "motor_init.h"
#include "udp_comm.h"
#include <cmath>
#include <iostream>
#include <vector>
#include <algorithm> 
#include <unistd.h> 

void InitMotors(UdpComm &udp_comm, udp::SendData &send_data, udp::ReceiveData &receive_data)
{
    const double kTolerance = 0.01;
    const double kTorque1 = 0.35;   // knee joint motors
    const double kTorque2 = 0.15;   // hip joint motors
    const double kTorque3 = 0.15;   // heel joint motors
    const int kStableCount = 50;
    bool allMotorsStable = false;
    int stableCounter = 0;

    // reverse motors list
    std::vector<int> reverseMotors = {1, 2, 4, 5, 9, 12, 15, 16, 17};
    std::vector<double> lastPositions(18, 0.000);

    while (!allMotorsStable)
    {
        bool currentlyStable = false;

        for (int i = 0; i < 18; ++i)
        {
            double baseTorque;
            // set different torque for different motors
            if (i == 1 || i == 4 || i == 7 || i == 10 || i == 13 || i == 16) {
                baseTorque = kTorque1;
                send_data.udp_motor_send[i].kd = 0.5;  // 设置膝关节电机的kd值
            } else if (i == 0 || i == 3 || i == 6 || i == 9 || i == 12 || i == 15) {
                baseTorque = kTorque2;
                send_data.udp_motor_send[i].kd = 0.0;  // 设置髋关节电机的kd值
            } else {
                baseTorque = kTorque3;
                send_data.udp_motor_send[i].kd = 0.0;  // 设置踝关节电机的kd值
            }

            // whether the motor is in reverse
            bool isReverseMotor = std::find(reverseMotors.begin(), reverseMotors.end(), i) != reverseMotors.end();
            double appliedTorque = isReverseMotor ? -baseTorque : baseTorque;
            send_data.udp_motor_send[i].torque = appliedTorque;
            send_data.udp_motor_send[i].kp = 0.0;
        }
        // send torque command
        udp_comm.setSendData(send_data);
        udp_comm.send();
        // receive feedback data
        if (!udp_comm.receive(1000))
        {
            std::cerr << "Failed to receive data!" << std::endl;
            return;
        }
        receive_data = udp_comm.getReceiveData();

        for (int i = 0; i < 18; ++i)
        {
            double currentPos = receive_data.udp_motor_receive[i].pos;
            double posDiff = std::fabs(currentPos - lastPositions[i]);
            
            if (posDiff <= kTolerance)
            {
                currentlyStable = true;
            }
            
            lastPositions[i] = currentPos;
        }

        if (currentlyStable)
        {
            stableCounter++;
            if (stableCounter >= kStableCount)
            {
                allMotorsStable = true;
                std::cout << "All motors have reached stable positions." << std::endl;
            }
        }
        
        usleep(10000);
    }

    std::cout << "Setting zero positions for all motors..." << std::endl;
    for (int i = 0; i < 18; ++i)
    {
        double zeroPosition = receive_data.udp_motor_receive[i].pos;
        std::cout << "Motor " << i << " zero position set to: " << zeroPosition << std::endl;
    }
    std::cout << "All motors initialization completed." << std::endl;
}

// void InitMotors2(UdpComm &udp_comm, udp::SendData &send_data, udp::ReceiveData &receive_data)
// {
//     Timer timer_;
//     int calibrationStep = -1;
//     int forceControlTime = 0;
//     int joint_num = 0;
//     int sum_flag = 0;
//     Eigen::Matrix<bool, 3, 6> init_finished_flag;
//     Eigen::MatrixXd current_pos(3, 6);
//     Eigen::MatrixXd last_pos(3, 6);
//     // reverse motors list
//     std::vector<int> reverseMotors = {1, 2, 4, 5, 9, 12, 15, 16, 17};

//     while (true)
//     {
//         timer_.start();        
//         //  标定程序
//         switch (calibrationStep)
//         {
//             case -1: // 给定力控命令
//                 forceControlTime++;
//                 for (int j = 0; j < 6; ++j)
//                 {
//                     udp_send_data.udp_motor_send[joint_num+6*j].kp = 0.0;
//                     if (joint_num == 0) // there exit difference between on-board calibration and lying calibration
//                     {
//                         // whether the motor is in reverse
//                         bool isReverseMotor = std::find(reverseMotors.begin(), reverseMotors.end(), i) != reverseMotors.end();
//                         double appliedTorque = isReverseMotor ? -0.15 : 0.15;                
//                         udp_send_data.udp_motor_send[joint_num+3*j].torque = appliedTorque;
//                         udp_send_data.udp_motor_send[joint_num+3*j].kd = 0.5;
//                         std::cout << "forceControlTime: " << forceControlTime << std::endl;
//                     }
//                 }
//                 if(forceControlTime > 2)
//                 {
//                     // calibrationStep = 0;
//                     forceControlTime = 0;
//                 }
//             break;

//             case 0:  // 判断是否到达机械限位
//                 init_finished_flag.setZero();

//                 //收取关节数据
//                 if (udp_comm.receive(1000000))
//                 {
//                     udp_receive_data = udp_comm.getReceiveData();            

//                     // 遍历所有电机并给矩阵赋值
//                     for(int leg = 0; leg < 6; leg++) {       // 遍历6条腿
//                         for(int joint = 0; joint < 3; joint++) { // 遍历每条腿的3个关节
//                             // 计算电机索引 (每条腿3个关节)
//                             int motor_idx = leg * 3 + joint;
//                             // 将电机位置值赋给对应矩阵元素
//                             current_pos(joint, leg) = udp_receive_data.udp_motor_receive[motor_idx].pos;
//                         }
//                     }

//                 }

//                 for (int j = 0; j < 6; ++j)
//                 {
//                     if (std::abs(current_pos(joint_num, j) - last_pos(joint_num, j)) < 0.002)
//                     {
//                         init_finished_flag(joint_num, j) = true;
//                     }
//                 }
//                 //  determine whether all six joints in the same position reached the mechanical limit
//                 sum_flag = 0;
//                 for (int j = 0; j < 6; ++j)
//                 {
//                     sum_flag += init_finished_flag(joint_num, j);
//                 }
//                 if (sum_flag == 6)
//                 {
//                     std::cout << "test ok " << std::endl;
//                     // 保存电机零位
//                     for (int j = 0; j < 6; ++j)
//                     {
//                         // 记录零位位置
//                         double zeroPosition = current_pos(joint_num, j);
//                         std::cout << "Joint " << joint_num << " Leg " << j << " zero position set to: " << zeroPosition << std::endl;
//                     }                    
//                     // 进入下一个关节
//                     ++joint_num;
//                     if (joint_num == 1) // 力控阶段结束,进入根关节回0度case
//                     {
//                         calibrationStep = 1;
//                     }
//                     else if(joint_num == 2) // 膝关节继续力控
//                     {
//                         calibrationStep = -1;
//                     }
//                     else if(joint_num == 3) // 所有关节力控矫正结束，开始控制关节回到指定角度
//                     {
//                         calibrationStep = 2;
//                     }
//                 }
//             case 1: // 根关节回0度位置
                
//             break;
//         }


//         last_pos = current_pos;

//         udp_send_data.state = static_cast<uint8_t>(CommBoardState::kNormal);
//         udp_comm.setSendData(udp_send_data);
//         udp_comm.send();

//         timer_.stop();
//         float control_frequency = 1.0; 
//         double time_compensate = 
//             1000. / control_frequency - timer_.elapsedMilliseconds();

//         if (time_compensate > 0)
//         {
//             usleep(static_cast<int>(time_compensate * 1000));  // 将毫秒转换为微秒
//         }
//     }
//     return 0;
// }

void ProtectMotors(UdpComm &udp_comm, udp::SendData &send_data, udp::ReceiveData &receive_data)
{
    for(int i = 0; i < 18; ++i)
    {
        send_data.udp_motor_send[i].torque = 0.0;
        send_data.udp_motor_send[i].kp = 0.0;
        send_data.udp_motor_send[i].kd = 0.0;
    }    
    udp_comm.setSendData(send_data);
    udp_comm.send();
}