/**
 * @file rt_control.cpp
 * @brief Xenomai实时控制程序 - 六足机器人标定控制
 * @author Based on work by Haowen Liang
 * @date 2025-6-4
 *
 * @co        if (display_counter % 1000 == 0 && !calibration_completed)
        {
            std::cout << "标定控制循环计数: " << cycle_stats.cycle_count 
                      << ", 当前抖动: " << cycle_stats.last_jitter_ns << " ns"
                      << ", 最小抖动: " << cycle_stats.min_jitter_ns << " ns"
                      << ", 最大抖动: " << cycle_stats.max_jitter_ns << " ns"
                      << ", 平均抖动: " << cycle_stats.avg_jitter_ns << " ns" << std::endl;
        }t Copyright (C) 2025.
 *
 */
#include "motor_init.h"
#include "udp_comm.h"
#include <cmath>
#include <iostream>
#include <vector>
#include <algorithm> 
#include <unistd.h> 
#include <signal.h>
#include <string.h>
#include <Eigen/Dense>

/* Xenomai 头文件 */
#include <alchemy/task.h>
#include <alchemy/timer.h>
#include <alchemy/mutex.h>

/* 外部变量声明 */
extern UdpComm udp_comm;
extern udp::ReceiveData udp_receive_data;
extern udp::SendData udp_send_data;
extern double motorZeroPositions[18];
extern std::vector<int> reverseMotors;

/* Xenomai 任务和互斥锁句柄 */
RT_TASK rt_control_task;
RT_MUTEX udp_mutex;

/* 控制参数 */
const RTIME control_period_ns = 1000000; // 1ms = 1000us = 1000000ns
static volatile bool running = true;
static volatile bool calibration_completed = false;

/* 控制周期统计信息 */
typedef struct {
    long min_jitter_ns;      // 最小抖动（纳秒）
    long max_jitter_ns;      // 最大抖动（纳秒）
    long avg_jitter_ns;      // 平均抖动（纳秒）
    long jitter_sum_ns;      // 抖动累计
    unsigned long cycle_count; // 循环计数
    long last_jitter_ns;     // 最近一次抖动
    RTIME last_cycle_ns;     // 最近一次循环时间
} CycleStats;

/* 标定状态变量 */
static int calibrationStep = -1;
static int forceControlTime = 0;
static int joint_num = 0;
static int sum_flag = 0;
static const int maxK = 200;  // 根关节最大插值次数
static int kk = 0;            // 根关节静态计数器
static const int maxK2 = 200; // 膝关节和踝关节最大插值次数
static int kk2 = 0;           // 膝关节和踝关节静态计数器

/* 信号处理函数，用于安全退出 */
static void signal_handler(int sig)
{
    running = false;
    std::cout << "捕获到信号 " << sig << "，准备退出..." << std::endl;
}

/* 全局周期统计信息，允许外部访问 */
static CycleStats cycle_stats = {
    .min_jitter_ns = LONG_MAX,
    .max_jitter_ns = 0,
    .avg_jitter_ns = 0,
    .jitter_sum_ns = 0,
    .cycle_count = 0,
    .last_jitter_ns = 0,
    .last_cycle_ns = 0
};

/* 实时控制任务函数 */
void rt_control_thread(void *arg)
{
    RTIME now, previous;
    RTIME task_period_ns = control_period_ns;
    
    /* 设置任务为周期性执行 */
    rt_task_set_periodic(NULL, TM_NOW, task_period_ns);
    
    /* 记录起始时间 */
    previous = rt_timer_read();
    
    unsigned long display_counter = 0;
    
    std::cout << "实时标定控制任务开始运行..." << std::endl;
    
    // 初始化标定相关矩阵
    Eigen::Matrix<bool, 3, 6> init_finished_flag;
    Eigen::MatrixXd current_pos(3, 6);
    Eigen::MatrixXd last_pos(3, 6);
    
    init_finished_flag.setZero();
    current_pos.setZero();
    last_pos.setZero();
    
    while (running)
    {
        /* 等待下一个周期 */
        rt_task_wait_period(NULL);
        
        /* 计算时间抖动和统计信息 */
        now = rt_timer_read();
        long jitter = now - previous - task_period_ns;
        previous = now;
        
        /* 更新周期统计信息 */
        cycle_stats.cycle_count++;
        cycle_stats.last_jitter_ns = jitter;
        cycle_stats.last_cycle_ns = now - previous;
        cycle_stats.jitter_sum_ns += jitter > 0 ? jitter : -jitter; // 累计绝对值抖动
        
        if (jitter < cycle_stats.min_jitter_ns)
            cycle_stats.min_jitter_ns = jitter;
            
        if (jitter > cycle_stats.max_jitter_ns)
            cycle_stats.max_jitter_ns = jitter;
            
        cycle_stats.avg_jitter_ns = cycle_stats.jitter_sum_ns / cycle_stats.cycle_count;
        
        display_counter++;
        
        /* 每秒显示一次抖动信息 - 注释掉，改为由主线程获取并显示 */
        // 实时线程中不打印，避免影响实时性能
        // if (display_counter % 1000 == 0 && !calibration_completed)
        // {
        //     std::cout << "标定控制循环计数: " << cycle_count 
        //               << ", 最大抖动: " << max_jitter << " ns" << std::endl;
        //     max_jitter = 0;
        // }
        
        /* 获取互斥锁 */
        rt_mutex_acquire(&udp_mutex, TM_INFINITE);
        
        /* 电机标定控制状态机 */
        if (!calibration_completed) 
        {
            switch (calibrationStep)
            {
            case -1: // 给定力控命令
                forceControlTime++;
                udp_comm.receive(1);
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
                    if (forceControlTime % 100 == 0)
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
                if (udp_comm.receive(1))
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
                    std::cout << "Hip joints reached limits, saving zero positions..." << std::endl;
                    // 保存电机零位
                    for (int j = 0; j < 6; ++j)
                    {
                        int motorIndex = joint_num + 3 * j;
                        // 记录零位位置
                        double zeroPosition = current_pos(joint_num, j);
                        motorZeroPositions[motorIndex] = zeroPosition; // 保存到全局零点数组
                        udp_send_data.udp_motor_send[motorIndex].torque = 0.0;
                        udp_send_data.udp_motor_send[motorIndex].kp = 0.0;
                        udp_send_data.udp_motor_send[motorIndex].kd = 0.0;
                        std::cout << "Joint " << joint_num << " Leg " << j << " zero position set to: " << zeroPosition << std::endl;
                    }
                    calibrationStep = 1; // 进入根关节回0度case
                }
                break;

            case 1: // 根关节回0度位置        
                //std::cout << "进入根关节回0度状态" << std::endl;
                
                kk++;
                if (kk > maxK)
                    kk = maxK;
                if (udp_comm.receive(1))
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
                break;

            case 2: // 膝关节和踝关节初始化状态
                //std::cout << "进入膝关节和踝关节初始化状态" << std::endl;
                forceControlTime++;
                udp_comm.receive(1);
                
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
            
            case 3: // 膝关节和踝关节机械限位状态
                //std::cout << "进入膝关节和踝关节机械限位状态" << std::endl;            
                init_finished_flag.setZero();
                if (udp_comm.receive(1))
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

                // 合并检查膝关节和踝关节是否到达限位的循环
                for (int joint = 1; joint < 3; joint++)
                {
                    for (int j = 0; j < 6; ++j)
                    {
                        if (std::abs(current_pos(joint, j) - last_pos(joint, j)) < 0.002)
                        {
                            init_finished_flag(joint, j) = true;
                        }
                    }
                }
                
                sum_flag = 0;
                // 计算膝关节和踝关节限位标志总和
                for (int joint = 1; joint < 3; joint++)
                {
                    for (int j = 0; j < 6; ++j)
                    {
                        sum_flag += init_finished_flag(joint, j);
                    }
                }
                
                if (sum_flag == 12) // 6条腿*2个关节=12个关节都到达限位
                {
                    std::cout << "Knee and ankle joints reached limits, saving zero positions..." << std::endl;
                    // 保存膝关节和踝关节零位
                    for (int joint = 1; joint < 3; joint++)
                    {
                        for (int j = 0; j < 6; ++j)
                        {
                            int motorIndex = joint + 3 * j; 
                            double zeroPosition = current_pos(joint, j);
                            motorZeroPositions[motorIndex] = zeroPosition; // 保存到全局零点数组
                            udp_send_data.udp_motor_send[motorIndex].torque = 0.0;
                            udp_send_data.udp_motor_send[motorIndex].kp = 0.0;
                            udp_send_data.udp_motor_send[motorIndex].kd = 0.0;
                            std::cout << "Joint " << joint << " Leg " << j << " zero position set to: " << zeroPosition << std::endl;
                        }
                    }
                    
                    calibrationStep = 4; 
                }
                break;

            case 4: // 膝关节和踝关节回0度位置            
                //std::cout << "进入膝关节和踝关节回0度位置状态" << std::endl;
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
                
                if(kk2 == maxK2) {
                    std::cout << "标定完成！所有关节已回到零位。" << std::endl;
                    calibration_completed = true;
                }
                break;
            }
            
            last_pos = current_pos;
        }
        // else {
        //     /* 标定完成后的正常控制逻辑 */
        //     /* 使所有电机处于标定后的原始位置（零点位置） */
        //     if (udp_comm.receive(1))
        //     {
        //         udp_receive_data = udp_comm.getReceiveData();
                
        //         /* 直接使用零点位置（不添加任何角度偏移）维持电机位置 */
        //         for (int i = 0; i < 18; ++i)
        //         {
        //             // 直接设置为零点位置，kp不为0确保位置控制有效
        //             udp_send_data.udp_motor_send[i].pos = motorZeroPositions[i];
        //             udp_send_data.udp_motor_send[i].kp = 0.1;
        //             udp_send_data.udp_motor_send[i].kd = 0.5;
        //             udp_send_data.udp_motor_send[i].torque = 0.0;
        //         }
        //     }
        // }

        udp_send_data.state = static_cast<uint8_t>(CommBoardState::kNormal);
        udp_comm.setSendData(udp_send_data);
        udp_comm.send();
        
        /* 释放互斥锁 */
        rt_mutex_release(&udp_mutex);
    }    
    /* 退出实时控制任务 */
    std::cout << "实时控制任务已安全退出。" << std::endl;
}

/* 获取标定完成状态 */
extern "C" bool is_calibration_completed()
{
    return calibration_completed;
}

/* 重置标定状态 */
extern "C" void reset_calibration()
{
    calibrationStep = -1;
    forceControlTime = 0;
    kk = 0;
    kk2 = 0;
    calibration_completed = false;
    
    // 同时重置统计信息
    cycle_stats.min_jitter_ns = LONG_MAX;
    cycle_stats.max_jitter_ns = 0;
    cycle_stats.avg_jitter_ns = 0;
    cycle_stats.jitter_sum_ns = 0;
    cycle_stats.cycle_count = 0;
    cycle_stats.last_jitter_ns = 0;
    cycle_stats.last_cycle_ns = 0;
}

/* 获取控制周期统计信息 */
extern "C" void get_cycle_stats(long* min_jitter, long* max_jitter, 
                               long* avg_jitter, long* last_jitter,
                               unsigned long* count)
{
    *min_jitter = cycle_stats.min_jitter_ns;
    *max_jitter = cycle_stats.max_jitter_ns;
    *avg_jitter = cycle_stats.avg_jitter_ns;
    *last_jitter = cycle_stats.last_jitter_ns;
    *count = cycle_stats.cycle_count;
}

/* 初始化实时控制系统 */
extern "C" bool init_rt_control()
{
    /* 初始化互斥锁 */
    if (rt_mutex_create(&udp_mutex, "UDP_MUTEX"))
    {
        std::cerr << "无法创建互斥锁！" << std::endl;
        return false;
    }
    
    /* 创建实时任务 */
    if (rt_task_create(&rt_control_task, "RT_CONTROL", 0, 99, T_JOINABLE))
    {
        std::cerr << "无法创建实时任务！" << std::endl;
        rt_mutex_delete(&udp_mutex);
        return false;
    }
    
    /* 启动实时任务 */
    if (rt_task_start(&rt_control_task, &rt_control_thread, NULL))
    {
        std::cerr << "无法启动实时任务！" << std::endl;
        rt_task_delete(&rt_control_task);
        rt_mutex_delete(&udp_mutex);
        return false;
    }
    
    return true;
}

/* 清理实时控制系统 */
extern "C" void cleanup_rt_control()
{
    running = false;
    rt_task_join(&rt_control_task);
    rt_task_delete(&rt_control_task);
    rt_mutex_delete(&udp_mutex);
}
