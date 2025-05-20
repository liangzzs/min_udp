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
    const double kTorque1 = 0.35;  // knee joint motors
    const double kTorque2 = 0.3;   // basic joint motors
    const double kTorque3 = 0.2;   // 踝关节motors
    const int kStableCount = 50;
    bool allMotorsStable = false;
    int stableCounter = 0;

    //reverse motors list
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
            } else if (i == 0 || i == 3 || i == 6 || i == 9 || i == 12 || i == 15) {
                baseTorque = kTorque2;
            } else {
                baseTorque = kTorque3;
            }

            // whether the motor is in reverse
            bool isReverseMotor = std::find(reverseMotors.begin(), reverseMotors.end(), i) != reverseMotors.end();
            double appliedTorque = isReverseMotor ? -baseTorque : baseTorque;
            send_data.udp_motor_send[i].torque = appliedTorque;
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

void ProtectMotors(UdpComm &udp_comm, udp::SendData &send_data, udp::ReceiveData &receive_data)
{
    for(int i = 0; i < 18; ++i)
    {
        send_data.udp_motor_send[i].torque = 0.0;
    }    
    udp_comm.setSendData(send_data);
    udp_comm.send();
}
void InitLegMotors(UdpComm &udp_comm, udp::SendData &send_data, udp::ReceiveData &receive_data, int leg_index)
{
    const double kTolerance = 0.02;
    const double kTorque = 0.35;
    const int kStableCount = 50;
    bool allMotorsStable = false;
    int stableCounter = 0;
    int motor_start_index = leg_index * 3;
    int motor_end_index = motor_start_index + 3;

    std::vector<int> reverseMotors = {1, 2, 4, 5, 9, 12, 15, 16, 17};
    std::vector<double> lastPositions(3, 0.0);
    
    while (!allMotorsStable)
    {
        bool currentlyStable = true;

        for (int i = motor_start_index; i < motor_end_index; ++i)
        {

            bool isReverseMotor = std::find(reverseMotors.begin(), reverseMotors.end(), i) != reverseMotors.end();
            double appliedTorque = isReverseMotor ? -kTorque : kTorque;
            send_data.udp_motor_send[i].torque = appliedTorque;
        }

        udp_comm.setSendData(send_data);
        udp_comm.send();

        if (!udp_comm.receive(1000))
        {
            std::cerr << "Failed to receive data!" << std::endl;
            return;
        }
        receive_data = udp_comm.getReceiveData();

        for (int i = motor_start_index; i < motor_end_index; ++i)
        {
            double currentPos = receive_data.udp_motor_receive[i].pos;
            double posDiff = std::fabs(currentPos - lastPositions[i - motor_start_index]);
            
            if (posDiff > kTolerance)
            {
                currentlyStable = false;
                stableCounter = 0;
            }
            
            lastPositions[i - motor_start_index] = currentPos;
        }

        if (currentlyStable)
        {
            stableCounter++;
            if (stableCounter >= kStableCount)
            {
                allMotorsStable = true;
                std::cout << "Leg " << leg_index << " has reached stable position." << std::endl;
            }
        }
        
        usleep(10000);
    }

    // 第二阶段：将当前位置标记为零点
    std::vector<double> zeroPositions(3);
    for (int i = motor_start_index; i < motor_end_index; ++i)
    {
        zeroPositions[i - motor_start_index] = receive_data.udp_motor_receive[i].pos;
        std::cout << "Motor " << i << " zero position set to: " << zeroPositions[i - motor_start_index] << std::endl;
    }
    std::cout << "Leg " << leg_index << " initialization completed." << std::endl;
}