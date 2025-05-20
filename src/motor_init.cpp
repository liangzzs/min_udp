#include "motor_init.h"
#include "udp_comm.h"
#include <cmath>
#include <iostream>
#include <vector>
#include <algorithm>  // 添加这行以支持 std::find
#include <unistd.h>  // for usleep

void InitMotors(UdpComm &udp_comm, udp::SendData &send_data, udp::ReceiveData &receive_data)
{
    const double kTolerance = 0.01;
    const double kTorque = 0.35;  // 使用与 InitLegMotors 相同的力矩值
    const int kStableCount = 50;
    bool allMotorsStable = false;
    int stableCounter = 0;

    // 需要反向力矩的电机列表
    std::vector<int> reverseMotors = {1, 4, 16, 17};
    
    // 记录上一次的位置
    std::vector<double> lastPositions(18, 0.000);

    while (!allMotorsStable)
    {
        bool currentlyStable = false;

        // 为每个电机施加恒定力矩
        for (int i = 0; i < 18; ++i)
        {
            // 检查当前电机是否需要反向力矩
            bool isReverseMotor = std::find(reverseMotors.begin(), reverseMotors.end(), i) != reverseMotors.end();
            
            // 根据是否需要反向来决定力矩方向
            double appliedTorque = isReverseMotor ? -kTorque : kTorque;
            send_data.udp_motor_send[i].torque = appliedTorque;
        }

        // 发送力矩命令
        udp_comm.setSendData(send_data);
        udp_comm.send();

        // 接收反馈数据
        if (!udp_comm.receive(1000))
        {
            std::cerr << "Failed to receive data!" << std::endl;
            return;
        }
        receive_data = udp_comm.getReceiveData();

        // 检查是否所有电机都稳定在极限位置
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
        
        usleep(10000); // 10ms延时，防止CPU占用过高
    }

    // 记录当前位置作为零点
    std::cout << "Setting zero positions for all motors..." << std::endl;
    for (int i = 0; i < 18; ++i)
    {
        double zeroPosition = receive_data.udp_motor_receive[i].pos;
        std::cout << "Motor " << i << " zero position set to: " << zeroPosition << std::endl;
    }

    // 发送零力矩命令
    for (int i = 0; i < 18; ++i)
    {
        send_data.udp_motor_send[i].torque = 0.0;
    }
    
    udp_comm.setSendData(send_data);
    udp_comm.send();

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

    // 需要反向力矩的电机列表
    std::vector<int> reverseMotors = {1, 4, 16, 17};

    // 计算该腿的电机索引范围
    int motor_start_index = leg_index * 3;
    int motor_end_index = motor_start_index + 3;

    // 记录上一次的位置
    std::vector<double> lastPositions(3, 0.0);
    
    // 第一阶段：施加恒定力矩直到到达极限位置
    while (!allMotorsStable)
    {
        bool currentlyStable = true;

        // 为每个电机施加恒定力矩
        for (int i = motor_start_index; i < motor_end_index; ++i)
        {
            // 检查当前电机是否需要反向力矩
            bool isReverseMotor = std::find(reverseMotors.begin(), reverseMotors.end(), i) != reverseMotors.end();
            
            // 根据是否需要反向来决定力矩方向
            double appliedTorque = isReverseMotor ? -kTorque : kTorque;
            send_data.udp_motor_send[i].torque = appliedTorque;
        }

        // 发送力矩命令
        udp_comm.setSendData(send_data);
        udp_comm.send();

        // 接收反馈数据
        if (!udp_comm.receive(1000))
        {
            std::cerr << "Failed to receive data!" << std::endl;
            return;
        }
        receive_data = udp_comm.getReceiveData();

        // 检查是否所有电机都稳定在极限位置
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
        
        usleep(10000); // 10ms延时，防止CPU占用过高
    }

    // 第二阶段：将当前位置标记为零点
    std::vector<double> zeroPositions(3);
    for (int i = motor_start_index; i < motor_end_index; ++i)
    {
        zeroPositions[i - motor_start_index] = receive_data.udp_motor_receive[i].pos;
        std::cout << "Motor " << i << " zero position set to: " << zeroPositions[i - motor_start_index] << std::endl;
    }

    // 第三阶段：发送零力矩命令
    for (int i = motor_start_index; i < motor_end_index; ++i)
    {
        send_data.udp_motor_send[i].torque = 0.0;
    }
    
    udp_comm.setSendData(send_data);
    udp_comm.send();

    std::cout << "Leg " << leg_index << " initialization completed." << std::endl;
}