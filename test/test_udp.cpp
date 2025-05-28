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
            udp_comm.receive(10);
            for (int j = 0; j < 6; ++j)
            {
                int motorIndex = joint_num + 3 * j; // 计算当前电机索引
                udp_send_data.udp_motor_send[motorIndex].kp = 0.0;
                if (joint_num == 0)
                {
                    bool isReverseMotor = std::find(reverseMotors.begin(), reverseMotors.end(), motorIndex) != reverseMotors.end();
                    double appliedTorque = isReverseMotor ? -0.2 : 0.2;
                    udp_send_data.udp_motor_send[motorIndex].torque = appliedTorque;
                    udp_send_data.udp_motor_send[motorIndex].kd = 0.5;
                }
                std::cout << "forceControlTime: " << forceControlTime << std::endl;
            }
            if (forceControlTime > 200)
            {
                calibrationStep = 0;
                forceControlTime = 0;
            }
            break;

        case 0: // 判断是否到达机械限位
            init_finished_flag.setZero();
            // 收取关节数据
            if (udp_comm.receive(10))
            {
                udp_receive_data = udp_comm.getReceiveData();
                // 遍历所有电机并给矩阵赋值
                for (int leg = 0; leg < 6; leg++)
                { // 遍历6条腿
                    for (int joint = 0; joint < 1; joint++)
                    { // 遍历每条腿的3个关节
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
                    int motorIndex = joint_num + 3 * j; // 在循环内部声明motorIndex
                    // 记录零位位置
                    double zeroPosition = current_pos(joint_num, j);
                    udp_send_data.udp_motor_send[motorIndex].torque = 0.0;
                    udp_send_data.udp_motor_send[motorIndex].kp = 0.0;
                    udp_send_data.udp_motor_send[motorIndex].kd = 0.0;
                    std::cout << "Joint " << joint_num << " Leg " << j << " zero position set to: " << zeroPosition << std::endl;
                }
                calibrationStep = 1; // 进入根关节回0度case
            }
            break;

        case 1: // 根关节回0度位置
        {
            std::cout << "进入根关节回0度状态" << std::endl;

            const int maxK = 200; // 定义最大插值次数
            static int kk = 0;    // 静态计数器用于插值
            kk++;
            if (kk > maxK)
                kk = maxK;
            if (udp_comm.receive(10))
            {

                udp_receive_data = udp_comm.getReceiveData();

                for (int j = 0; j < 6; ++j) // 6条腿
                {
                    int motorIndex = 3 * j; // 计算电机索引
                    float targetDiff;

                    // 根据电机编号设置目标位置差值
                    if (motorIndex == 0 || motorIndex == 3 || motorIndex == 6)
                    {
                        targetDiff = -7.25f;
                    }
                    else if (motorIndex == 9 || motorIndex == 12 || motorIndex == 15)
                    {
                        targetDiff = 7.25f;
                    }

                    // 进行线性插值
                    float tmpTgt = current_pos(joint_num, j) + (targetDiff / float(maxK)) * kk;

                    // 设置电机命令
                    udp_send_data.udp_motor_send[motorIndex].pos = tmpTgt;
                    udp_send_data.udp_motor_send[motorIndex].kp = 0.05; // 位置环增益
                    udp_send_data.udp_motor_send[motorIndex].kd = 0.5;
                }
            }
            if (kk == maxK) // 完成回零过程
            {
                calibrationStep = 2;
            }
        }
        break;

        case 2:
            std::cout << "进入膝关节和踝关节初始化状态" << std::endl;
            forceControlTime++;
            udp_comm.receive(10);
            
            for (int j = 0; j < 6; ++j)
            {
                // 处理膝关节(joint_num = 1)
                int kneeMotorIndex = 1 + 3 * j;  // 计算膝关节电机索引
                udp_send_data.udp_motor_send[kneeMotorIndex].kp = 0.0;
                bool isKneeReverse = std::find(reverseMotors.begin(), reverseMotors.end(), kneeMotorIndex) != reverseMotors.end();
                double kneeAppliedTorque = isKneeReverse ? -0.5 : 0.5;
                udp_send_data.udp_motor_send[kneeMotorIndex].torque = kneeAppliedTorque;
                udp_send_data.udp_motor_send[kneeMotorIndex].kd = 1.5;
                
                // 处理踝关节(joint_num = 2)
                int ankleMotorIndex = 2 + 3 * j;  // 计算踝关节电机索引
                udp_send_data.udp_motor_send[ankleMotorIndex].kp = 0.0;
                bool isAnkleReverse = std::find(reverseMotors.begin(), reverseMotors.end(), ankleMotorIndex) != reverseMotors.end();
                double ankleAppliedTorque = isAnkleReverse ? -0.25 : 0.25;
                udp_send_data.udp_motor_send[ankleMotorIndex].torque = ankleAppliedTorque;
                udp_send_data.udp_motor_send[ankleMotorIndex].kd = 0.5;
            }
            
            if (forceControlTime > 200)
            {
                calibrationStep = 3;  // 完成所有关节初始化，进入保护状态
                forceControlTime = 0;
            }        
        break;
        
        case 3:
            std::cout << "进入膝关节和踝关节机械限位状态" << std::endl;            
            init_finished_flag.setZero();
            if (udp_comm.receive(10))
            {
                udp_receive_data = udp_comm.getReceiveData();
                for (int leg = 0; leg < 6; leg++)
                { 
                    for (int joint = 1; joint < 3; joint++)
                    {                         
                        int motor_idx = leg * 3 + joint;                        
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
            sum_flag = 0;
            for (int j = 0; j < 6; ++j)
            {
                sum_flag += init_finished_flag(joint_num, j);
            }
            if (sum_flag == 6)
            {
                std::cout << "test ok " << std::endl;
                for (int j = 0; j < 6; ++j)
                {
                    int motorIndex = joint_num + 3 * j; 
                    double zeroPosition = current_pos(joint_num, j);
                    udp_send_data.udp_motor_send[motorIndex].torque = 0.0;
                    udp_send_data.udp_motor_send[motorIndex].kp = 0.0;
                    udp_send_data.udp_motor_send[motorIndex].kd = 0.0;
                    std::cout << "Joint " << joint_num << " Leg " << j << " zero position set to: " << zeroPosition << std::endl;
                }
                calibrationStep = 4; 
            }
            break;

        case 4: // 膝关节和踝关节回0度位置            
            std::cout << "进入膝关节和踝关节回0度位置状态" << std::endl;
            const int maxK2 = 200;
            static int kk2 = 0;
            kk2++;
            if(kk2 > maxK2) kk2 = maxK2;            
            
            for(int curr_joint = 1; curr_joint <= 2; curr_joint++) 
            {
                for (int j = 0; j < 6; ++j)
                {
                    int motorIndex = curr_joint + 3*j;
                    float targetDiff;                    
                    bool isReverseMotor = std::find(reverseMotors.begin(), 
                                                reverseMotors.end(), 
                                                motorIndex) != reverseMotors.end();
                    if (curr_joint == 1)                    
                        targetDiff = isReverseMotor ? 5.25f : -5.25f;                    
                    else                     
                        targetDiff = isReverseMotor ? 27.0f : -27.0f;                    

                    float tmpTgt = current_pos(curr_joint, j) + (targetDiff / float(maxK2)) * kk2;

                    udp_send_data.udp_motor_send[motorIndex].pos = tmpTgt;
                    if (curr_joint == 1) //knee joint
                    {
                        udp_send_data.udp_motor_send[motorIndex].kp = 0.05;   
                        udp_send_data.udp_motor_send[motorIndex].kd = 0.5;
                    } 
                    else                 //ankle joint
                    { 
                        udp_send_data.udp_motor_send[motorIndex].kp = 0.05;  
                        udp_send_data.udp_motor_send[motorIndex].kd = 0.5;  
                    }
                }
            }
            
        break;

        // case 99:        
        //     ProtectMotors(udp_comm, udp_send_data, udp_receive_data);
        //     std::cout << "进入保护状态" << std::endl;        
        // break;
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
            usleep(static_cast<int>(time_compensate * 1000)); // 将毫秒转换为微秒
        }
    }
    return 0;
}