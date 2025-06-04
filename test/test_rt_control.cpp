/**
 * @file test_rt_control.cpp
 * @brief 测试Xenomai实时控制功能
 * @author Based on work by Haowen Liang
         // 添加标定状态信息
        is_calibrated = is_calibration_completed();
        std::cout << " | \033[1m标定状态:\033[0m " 
                  << (is_calibrated ? "\033[32m已完成\033[0m" : "\033[33m进行中\033[0m")
                  << std::flush;  // 使用flush来实现同行更新
        
        // 如果刚刚完成标定，显示零点位置
        if (is_calibrated && !showedCalibrationComplete)
        {
            std::cout << std::endl << std::endl;
            std::cout << "\033[32m标定已完成！\033[0m" << std::endl;
            showedCalibrationComplete = true;
            
            // 显示标定后的零点位置
            std::cout << "标定后的电机零点位置:" << std::endl;
            double* zeroPositions = getMotorZeroPositions();
            std::cout << "+-----+-----+---------------+---------------+---------------+" << std::endl;
            std::cout << "| 腿  | 关节 | 零点位置      | 关节类型      | 电机索引      |" << std::endl;
            std::cout << "+-----+-----+---------------+---------------+---------------+" << std::endl;
            
            for (int leg = 0; leg < 6; leg++) {
                for (int joint = 0; joint < 3; joint++) {
                    int motorIndex = leg * 3 + joint;
                    std::string jointType;
                    if (joint == 0) jointType = "髋关节(Hip)";
                    else if (joint == 1) jointType = "膝关节(Knee)";
                    else jointType = "踝关节(Ankle)";
                    
                    std::cout << "| " << std::setw(3) << leg 
                              << " | " << std::setw(3) << joint 
                              << " | " << std::fixed << std::setprecision(6) << std::setw(13) << zeroPositions[motorIndex] 
                              << " | " << std::setw(13) << jointType
                              << " | " << std::setw(13) << motorIndex
                              << " |" << std::endl;
                }
            }
            std::cout << "+-----+-----+---------------+---------------+---------------+" << std::endl;
            std::cout << std::endl << "继续显示实时控制周期信息..." << std::endl << std::endl;
        }
        
        usleep(50000); // 缩短休眠时间到50ms，以更频繁更新显示
 * @copyright Copyright (C) 2025.
 *
 */
#include "udp_comm.h"
#include "motor_init.h"
#include "rt_control.h"
#include <iostream>
#include <string>
#include <unistd.h>
#include <signal.h>
#include <atomic>
#include <iomanip>  // 用于格式化输出
#include <climits>  // 定义LONG_MAX

// 全局变量定义
UdpComm udp_comm(7, "192.168.1.10", 10);
udp::ReceiveData udp_receive_data = {};
udp::SendData udp_send_data = {};
std::atomic<bool> running(true);

// 信号处理函数
void signal_handler(int sig)
{
    running = false;
    std::cout << "捕获到信号 " << sig << "，准备退出..." << std::endl;
}

int main(int argc, char *argv[])
{
    // 注册信号处理函数
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    // 初始化UDP通信
    if (!udp_comm.init())
    {
        std::cerr << "udp init failed" << std::endl;
        return 1;
    }
    
    // 重置标定状态
    reset_calibration();
    
    // 初始化实时标定控制系统
    if (!init_rt_control())
    {
        std::cerr << "实时标定控制系统初始化失败！" << std::endl;
        return 1;
    }
    
    std::cout << "实时标定控制系统已启动，开始执行标定流程..." << std::endl;
    
    // 主线程等待退出信号或标定完成
    bool showedCalibrationComplete = false;
    bool is_calibrated = false;
    unsigned long last_cycle_count = 0;
    double freq_hz = 0.0;
    
    // 周期统计信息显示的变量
    long min_jitter, max_jitter, avg_jitter, last_jitter;
    unsigned long cycle_count;
    
    // 清除屏幕
    std::cout << "\033[2J\033[1;1H";
    std::cout << "====== 六足机器人实时标定控制系统 ======" << std::endl;
    std::cout << "实时控制线程周期: 1ms (1000Hz)" << std::endl;
    
    while (running)
    {
        // 获取实时控制周期统计信息
        get_cycle_stats(&min_jitter, &max_jitter, &avg_jitter, &last_jitter, &cycle_count);
        
        // 计算实际频率 (每100ms更新一次)
        if (cycle_count > last_cycle_count) {
            freq_hz = 1000000000.0 / (1000000.0 + std::abs(avg_jitter));
            last_cycle_count = cycle_count;
        }
        
        // 清除当前行，显示控制周期信息，带有彩色输出和格式化数字
        std::cout << "\r\033[K"; // 清除当前行
        
        // 循环计数和频率
        std::cout << "\033[1m控制循环:\033[0m " << std::setw(10) << cycle_count 
                  << " | \033[1m频率:\033[0m " << std::fixed << std::setprecision(2) 
                  << std::setw(7) << freq_hz << " Hz";
        
        // 抖动信息
        std::cout << " | \033[1m抖动(ns):\033[0m 当前:" << std::setw(7) << last_jitter 
                  << " 最小:" << std::setw(7) << min_jitter 
                  << " 最大:" << std::setw(7) << max_jitter
                  << " 平均:" << std::setw(7) << avg_jitter;
        
        // 检查标定状态
        is_calibrated = is_calibration_completed();
        if (is_calibrated && !showedCalibrationComplete)
        {
            std::cout << std::endl << "标定已完成！" << std::endl;
            showedCalibrationComplete = true;
            
            // 显示标定后的零点位置
            std::cout << "标定后的电机零点位置:" << std::endl;
            double* zeroPositions = getMotorZeroPositions();
            for (int leg = 0; leg < 6; leg++) {
                for (int joint = 0; joint < 3; joint++) {
                    int motorIndex = leg * 3 + joint;
                    std::cout << "腿 " << leg << ", 关节 " << joint 
                              << ": " << zeroPositions[motorIndex] << std::endl;
                }
            }
            std::cout << "继续显示实时控制周期信息..." << std::endl;
        }
        
        usleep(100000); // 缩短休眠时间到100ms，以更频繁更新显示
    }
    
    // 清理实时控制系统
    std::cout << "正在清理实时控制系统..." << std::endl;
    cleanup_rt_control();
    
    // 保护电机
    ProtectMotors(udp_comm, udp_send_data, udp_receive_data);
    std::cout << "程序已安全退出" << std::endl;
    
    return 0;
}
