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
#include <ctime>
#include <chrono>
#include <Eigen/Dense>

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
    ProtectMotors(udp_comm, udp_send_data, udp_receive_data);
    std::cout << "电机初始化和保护完成，开始主循环..." << std::endl;

    // Now this will compile fine
    Timer timer_;
    int calibrationStep = -1;
    int forceControlTime = 0;
    int joint_num = 0;
    int sum_flag = 0;

    Eigen::Matrix<bool, 3, 6> init_finished_flag;
    Eigen::MatrixXd current_pos(3, 6);
    Eigen::MatrixXd last_pos(3, 6);
    std::vector<int> reverseMotors = {1, 2, 4, 5, 9, 12, 15, 16, 17};

    while (true)
    {
        timer_.start();
        
        //  标定程序
        switch (calibrationStep)
        {
            case -1: // 给定力控命令                
                forceControlTime++;
                for (int j = 0; j < 6; ++j)
                {
                    int motorIndex = joint_num + 3*j;  // 计算当前电机索引
                    udp_send_data.udp_motor_send[motorIndex].kp = 0.0;
                    if (joint_num == 0)
                    {
                        bool isReverseMotor = std::find(reverseMotors.begin(), reverseMotors.end(), motorIndex) != reverseMotors.end();
                        double appliedTorque = isReverseMotor ? -0.15 : 0.15;                
                        udp_send_data.udp_motor_send[motorIndex].torque = appliedTorque;
                        udp_send_data.udp_motor_send[motorIndex].kd = 0.5;
                        std::cout << "forceControlTime: " << forceControlTime << std::endl;
                    }
                }
                if(forceControlTime > 200)
                {
                    calibrationStep = 0;
                    forceControlTime = 0;
                }                
                break;

            case 0:  // 判断是否到达机械限位                
                init_finished_flag.setZero();
                //收取关节数据
                if (udp_comm.receive(1000000))
                {
                    udp_receive_data = udp_comm.getReceiveData();
                    
                    // 遍历所有电机并给矩阵赋值
                    for(int leg = 0; leg < 6; leg++) {       // 遍历6条腿
                        for(int joint = 0; joint < 3; joint++) { // 遍历每条腿的3个关节
                            // 计算电机索引 (每条腿3个关节)
                            int motor_idx = leg * 3 + joint;
                            // 将电机位置值赋给对应矩阵元素
                            current_pos(joint, leg) = udp_receive_data.udp_motor_receive[motor_idx].pos;
                        }
                    }
                }

                for (int j = 0; j < 6; ++j)
                {
                    if (std::abs(current_pos(joint_num, j) - last_pos(joint_num, j)) < 0.002)
                    {
                        init_finished_flag(joint_num, j) = true;
                    }
                }
                //  determine whether all six joints in the same position reached the mechanical limit
                sum_flag = 0;
                for (int j = 0; j < 6; ++j)
                {
                    sum_flag += init_finished_flag(joint_num, j);
                }
                if (sum_flag == 6)
                {
                    std::cout << "test ok " << std::endl;
                    // 保存电机零位
                    for (int j = 0; j < 6; ++j)
                    {
                        int motorIndex = joint_num + 3*j;  // 在循环内部声明motorIndex
                        // 记录零位位置
                        double zeroPosition = current_pos(joint_num, j);
                        udp_send_data.udp_motor_send[motorIndex].torque = 0.0;
                        udp_send_data.udp_motor_send[motorIndex].kp = 0.0;
                        udp_send_data.udp_motor_send[motorIndex].kd = 0.0;
                        std::cout << "Joint " << joint_num << " Leg " << j << " zero position set to: " << zeroPosition << std::endl;
                    }
                    calibrationStep = 1; // 力控阶段结束,进入根关节回0度case
                }                
                break;

            case 1: // 根关节回0度位置
                {
                    const int maxK = 100;  // 定义最大插值次数
                    static int kk = 0;     // 静态计数器用于插值
                    kk++;
                    if(kk > maxK) kk = maxK;
                    
                    for (int j = 0; j < 6; ++j)  // 6条腿
                    {
                        int motorIndex = joint_num + 3*j;  // 计算电机索引
                        float targetDiff;
                        
                        // 根据电机编号设置目标位置差值
                        if (motorIndex == 0 || motorIndex == 3 || motorIndex == 6) {
                            targetDiff = -7.0f;
                            

                        } else if (motorIndex == 9 || motorIndex == 12 || motorIndex == 15) {
                            targetDiff = 7.0f;
                        }
                        
                        // 进行线性插值
                        float tmpTgt = current_pos(joint_num, j) + (targetDiff / float(maxK)) * kk;
                        
                        // 设置电机命令
                        udp_send_data.udp_motor_send[motorIndex].pos = tmpTgt;
                        udp_send_data.udp_motor_send[motorIndex].kp = 0.15;  // 位置环增益
                        udp_send_data.udp_motor_send[motorIndex].kd = 0.1;
                    }
                    
                    if (kk == maxK)  // 完成回零过程
                    {
                        calibrationStep = 2;
                    }
                }
                break;
            case 2:
                {
                    ProtectMotors(udp_comm, udp_send_data, udp_receive_data);
                }
               break;
        }
        last_pos = current_pos;

        udp_send_data.state = static_cast<uint8_t>(CommBoardState::kNormal);
        udp_comm.setSendData(udp_send_data);
        udp_comm.send();

        timer_.stop();
        float control_frequency = 100.0; 
        double time_compensate = 
            1000. / control_frequency - timer_.elapsedMilliseconds();

        if (time_compensate > 0)
        {
            usleep(static_cast<int>(time_compensate * 1000));  // 将毫秒转换为微秒
        }
    }
    return 0;
}