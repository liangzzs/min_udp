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

        // 标定程序
        switch (calibrationStep)
        {
        case -1: // 给定力控命令
            forceControlTime++;
            udp_comm.receive(10);
            for (int j = 0; j < 6; ++j)
            {
                int motorIndex = joint_num + 3 * j;
                send_data.udp_motor_send[motorIndex].kp = 0.0;
                if (joint_num == 0)
                {
                    bool isReverseMotor = std::find(reverseMotors.begin(), reverseMotors.end(), motorIndex) != reverseMotors.end();
                    double appliedTorque = isReverseMotor ? -0.15 : 0.15;
                    send_data.udp_motor_send[motorIndex].torque = appliedTorque;
                    send_data.udp_motor_send[motorIndex].kd = 0.5;
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
            if (udp_comm.receive(10))
            {
                receive_data = udp_comm.getReceiveData();
                for (int leg = 0; leg < 6; leg++)
                {
                    for (int joint = 0; joint < 3; joint++)
                    {
                        int motor_idx = leg * 3 + joint;
                        current_pos(joint, leg) = receive_data.udp_motor_receive[motor_idx].pos;
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
                    send_data.udp_motor_send[motorIndex].torque = 0.0;
                    send_data.udp_motor_send[motorIndex].kp = 0.0;
                    send_data.udp_motor_send[motorIndex].kd = 0.0;
                    std::cout << "Joint " << joint_num << " Leg " << j << " zero position set to: " << zeroPosition << std::endl;
                }
                calibrationStep = 1;
            }
            break;

        case 1: // 根关节回0度位置
        {
            std::cout << "进入根关节回0度状态" << std::endl;
            const int maxK = 200;
            static int kk = 0;
            kk++;
            if (kk > maxK)
                kk = maxK;
            if (udp_comm.receive(10))
            {
                receive_data = udp_comm.getReceiveData();
                for (int j = 0; j < 6; ++j)
                {
                    int motorIndex = 3 * j;
                    float targetDiff;

                    if (motorIndex == 0 || motorIndex == 3 || motorIndex == 6)
                    {
                        targetDiff = -7.25f;
                    }
                    else if (motorIndex == 9 || motorIndex == 12 || motorIndex == 15)
                    {
                        targetDiff = 7.25f;
                    }

                    float tmpTgt = current_pos(joint_num, j) + (targetDiff / float(maxK)) * kk;

                    send_data.udp_motor_send[motorIndex].pos = tmpTgt;
                    send_data.udp_motor_send[motorIndex].kp = 0.05;
                    send_data.udp_motor_send[motorIndex].kd = 0.5;
                }
            }
            if (kk == maxK)
            {
                calibrationStep = 2;
            }
        }
        break;

        case 2: // 膝关节和踝关节力控初始化
            std::cout << "进入膝关节和踝关节初始化状态" << std::endl;
            forceControlTime++;
            udp_comm.receive(10);
            
            for (int j = 0; j < 6; ++j)
            {
                // 处理膝关节
                int kneeMotorIndex = 1 + 3 * j;
                send_data.udp_motor_send[kneeMotorIndex].kp = 0.0;
                bool isKneeReverse = std::find(reverseMotors.begin(), reverseMotors.end(), kneeMotorIndex) != reverseMotors.end();
                double kneeAppliedTorque = isKneeReverse ? -0.5 : 0.5;
                send_data.udp_motor_send[kneeMotorIndex].torque = kneeAppliedTorque;
                send_data.udp_motor_send[kneeMotorIndex].kd = 0.5;
                
                // 处理踝关节
                int ankleMotorIndex = 2 + 3 * j;
                send_data.udp_motor_send[ankleMotorIndex].kp = 0.0;
                bool isAnkleReverse = std::find(reverseMotors.begin(), reverseMotors.end(), ankleMotorIndex) != reverseMotors.end();
                double ankleAppliedTorque = isAnkleReverse ? -0.2 : 0.2;
                send_data.udp_motor_send[ankleMotorIndex].torque = ankleAppliedTorque;
                send_data.udp_motor_send[ankleMotorIndex].kd = 0.5;
            }
            
            if (forceControlTime > 200)
            {
                calibrationStep = 3;
                forceControlTime = 0;
            }
            break;
            
        case 3: // 检测膝关节和踝关节机械限位
            std::cout << "进入膝关节和踝关节机械限位状态" << std::endl;
            init_finished_flag.setZero();
            if (udp_comm.receive(10))
            {
                receive_data = udp_comm.getReceiveData();
                for(int leg = 0; leg < 6; leg++) {
                    for(int joint = 1; joint < 3; joint++) {
                        int motor_idx = leg * 3 + joint;
                        current_pos(joint, leg) = receive_data.udp_motor_receive[motor_idx].pos;
                    }
                }
            }

            for (int joint = 1; joint < 3; joint++) {
                for (int leg = 0; leg < 6; leg++) {
                    if (std::abs(current_pos(joint, leg) - last_pos(joint, leg)) < 0.002)
                    {
                        init_finished_flag(joint, leg) = true;
                    }
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
                    int motorIndex = joint_num + 3*j;
                    double zeroPosition = current_pos(joint_num, j);
                    send_data.udp_motor_send[motorIndex].torque = 0.0;
                    send_data.udp_motor_send[motorIndex].kp = 0.0;
                    send_data.udp_motor_send[motorIndex].kd = 0.0;
                    std::cout << "Joint " << joint_num << " Leg " << j << " zero position set to: " << zeroPosition << std::endl;
                }
                calibrationStep = 4;
            }
            break;

        case 4: // 膝关节和踝关节回0度位置
            std::cout << "进入膝关节和踝关节回0度位置状态" << std::endl;
            {
                const int maxK2 = 100;
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
                            targetDiff = isReverseMotor ? 5.0f : -5.0f;
                        else
                            targetDiff = isReverseMotor ? 8.0f : -8.0f;

                        float tmpTgt = current_pos(curr_joint, j) + (targetDiff / float(maxK2)) * kk2;

                        send_data.udp_motor_send[motorIndex].pos = tmpTgt;
                        if (curr_joint == 1) //knee joint
                        {
                            send_data.udp_motor_send[motorIndex].kp = 0.2;
                            send_data.udp_motor_send[motorIndex].kd = 0.15;
                        } 
                        else //ankle joint
                        {
                            send_data.udp_motor_send[motorIndex].kp = 0.12;
                            send_data.udp_motor_send[motorIndex].kd = 0.08;
                        }
                    }
                }
                
                if (kk2 == maxK2)
                {
                    // 初始化完成，退出函数
                    std::cout << "所有关节初始化完成" << std::endl;
                    return;
                }
            }
            break;
        }

        last_pos = current_pos;

        send_data.state = static_cast<uint8_t>(CommBoardState::kNormal);
        udp_comm.setSendData(send_data);
        udp_comm.send();

        timer_.stop();
        float control_frequency = 100.0;
        double time_compensate =
            1000. / control_frequency - timer_.elapsedMilliseconds();

        if (time_compensate > 0)
        {
            usleep(static_cast<int>(time_compensate * 1000));
        }
    }
}

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