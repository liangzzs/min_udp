#include "motor_init.h"
#include "udp_comm.h"
#include <cmath>
#include <iostream>
#include <vector>
#include <unistd.h>  // for usleep

void InitMotors(UdpComm &udp_comm, udp::SendData &send_data, udp::ReceiveData &receive_data)
{
    const double kTolerance = 0.02; // 判断是否到达极限位置的误差范围
    const double kTorqueStep = 0.1; // 每次调整的力矩步长
    const double kMaxTorque = 0.3;  // 最大允许力矩
    bool allMotorsReachedLimit = false;

    while (!allMotorsReachedLimit)
    {
        allMotorsReachedLimit = true; // 假设所有电机都已到达极限位置

        for (int i = 0; i < 18; ++i)
        {
            double currentPos = receive_data.udp_motor_receive[i].pos;
            double targetPos = 0.0; // 初始化目标位置为零点

            // 如果当前位置与目标位置的差值大于误差范围，继续施加力矩
            if (std::fabs(currentPos - targetPos) > kTolerance)
            {
                allMotorsReachedLimit = false;

                // 根据当前位置与目标位置的差值施加力矩
                double torque = (currentPos > targetPos) ? -kTorqueStep : kTorqueStep;

                // 限制力矩的最大值
                if (std::fabs(torque) > kMaxTorque)
                {
                    torque = (torque > 0) ? kMaxTorque : -kMaxTorque;
                }

                send_data.udp_motor_send[i].torque = torque;
            }
            else
            {
                // 如果电机已经到达目标位置，停止施加力矩
                send_data.udp_motor_send[i].torque = 0.0;
            }
        }

        // 发送调整后的数据
        udp_comm.setSendData(send_data);
        udp_comm.send();

        // 接收反馈数据
        if (udp_comm.receive(1000))
        {
            receive_data = udp_comm.getReceiveData();
        }
        else
        {
            std::cerr << "Failed to receive!" << std::endl;
            break;
        }
    }

    // 检查是否所有电机完成初始化
    bool allMotorsInitialized = true;
    for (int i = 0; i < 18; ++i)
    {
        if (std::fabs(receive_data.udp_motor_receive[i].pos) > kTolerance)
        {
            allMotorsInitialized = false;
            std::cerr << "Motor " << i << " failed to initialize properly!" << std::endl;
        }
    }

    if (allMotorsInitialized)
        std::cout << "All motors inited to zero position successfully!" << std::endl;
    else
        std::cerr << "Some motors failed to initialize properly!" << std::endl;
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
    const double kTolerance = 0.02;    // 判断是否到达极限位置的误差范围
    const double kTorque = 0.35;        // 恒定力矩大小
    const int kStableCount = 50;       // 需要保持稳定的周期数
    bool allMotorsStable = false;
    int stableCounter = 0;

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
            send_data.udp_motor_send[i].torque = kTorque;
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