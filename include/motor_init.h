#ifndef MOTOR_INIT_H
#define MOTOR_INIT_H

#include "udp_comm.h"
#include <iostream>
#include <string>
#include <unistd.h>
#include <ctime>
#include <chrono>
#include <Eigen/Dense>
/**
 * @brief 标定六足机器人所有电机，使所有电机回到零位置点
 * 
 * @param udp_comm UDP通信对象，用于发送和接收数据
 * @param send_data 发送数据结构，包含电机目标参数
 * @param receive_data 接收数据结构，包含电机当前状态
 */
void InitMotors(UdpComm &udp_comm, udp::SendData &send_data, udp::ReceiveData &receive_data);

void InitLegMotors(UdpComm &udp_comm, udp::SendData &send_data, udp::ReceiveData &receive_data, int leg_index);

void ProtectMotors(UdpComm &udp_comm, udp::SendData &send_data, udp::ReceiveData &receive_data);

void InitMotors2(UdpComm &udp_comm, udp::SendData &send_data, udp::ReceiveData &receive_data);

#endif // MOTOR_INIT_H

enum class TimerSourceType
{
    kNone,
    kRos,
    kSystem
};

class Timer
{
public:
    // Modified constructor with default argument
    explicit Timer(TimerSourceType source_type = TimerSourceType::kNone) 
        : source_type_(source_type) {}
    
    ~Timer() = default;

    void start()
    {
        sys_start_time_ = std::chrono::high_resolution_clock::now();
        running_flag_ = true;
    }

    void stop()
    {
        sys_stop_time_ = std::chrono::high_resolution_clock::now();
        running_flag_ = false;
    }

    [[nodiscard]] double elapsedMilliseconds() const
    {
        const auto end_time = running_flag_ 
            ? std::chrono::high_resolution_clock::now() 
            : sys_stop_time_;
        
        return std::chrono::duration<double, std::milli>(end_time - sys_start_time_).count();
    }

    [[nodiscard]] double elapsedSeconds() const
    {
        return elapsedMilliseconds() / 1000.0;
    }

private:
    TimerSourceType source_type_{TimerSourceType::kNone};
    std::chrono::time_point<std::chrono::high_resolution_clock> sys_start_time_{};
    std::chrono::time_point<std::chrono::high_resolution_clock> sys_stop_time_{};
    bool running_flag_{false};
};