#include "motor_init.h"
#include "udp_comm.h"
#include <cmath> // 用于fabs函数
#include <iostream>

// void InitMotors(UdpComm &udp_comm, udp::SendData &send_data, udp::ReceiveData &receive_data)
// {
//     const double kTolerance = 0.01; // 允许的零点误差
//     const double kStep = 0.1;       // 每次调整的步长
//     bool allMotorsInited = false;

//     while (!allMotorsInited)
//     {
//         allMotorsInited = true;

//         for (int i = 0; i < 18; ++i)
//         {
//             double currentPos = receive_data.udp_motor_receive[i].pos;

//             // 如果当前位置不在零点附近，调整位置
//             if (std::fabs(currentPos) > kTolerance)
//             {
//                 allMotorsInited = false;

//                 // 根据当前位置调整目标位置
//                 if (currentPos > 0)
//                 {
//                     send_data.udp_motor_send[i].pos -= kStep;
//                 }
//                 else
//                 {
//                     send_data.udp_motor_send[i].pos += kStep;
//                 }
//             }
//         }

//         // 发送调整后的数据
//         udp_comm.setSendData(send_data);
//         udp_comm.send();

//         // 接收反馈数据
//         if (udp_comm.receive(1000))
//         {
//             receive_data = udp_comm.getReceiveData();
//         }
//         else
//         {
//             std::cerr << "Failed to receive data!" << std::endl;
//             break;
//         }
//     }

//     if (allMotorsInited)
//     {
//         std::cout << "All motors inited to zero position successfully!" << std::endl;
//     }
// }

void InitMotors(UdpComm &udp_comm, udp::SendData &send_data, udp::ReceiveData &receive_data)
{
    const double kTolerance = 0.01; // 判断是否到达极限位置的误差范围
    const double kStep = 0.1;       // 每次调整的步长
    bool allMotorsReachedLimit = false;

    while (!allMotorsReachedLimit)
    {     

        for (int i = 0; i < 18; ++i)
        {
            double currentPos = receive_data.udp_motor_receive[i].pos;
            double targetPos = send_data.udp_motor_send[i].pos;

            // 如果当前位置与目标位置的差值小于误差范围，认为电机到达极限位置
            if (std::fabs(currentPos - targetPos) > kTolerance)
            {
                allMotorsReachedLimit = false;

                // 继续向目标方向移动
                send_data.udp_motor_send[i].pos += kStep;
            }
            else
            {
                // 如果电机已经到达目标位置，设置目标位置为当前值
                send_data.udp_motor_send[i].pos = currentPos;
                allMotorsReachedLimit = true;
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

    // 标记当前极限位置为零点
    for (int i = 0; i < 18; ++i)
    {
        send_data.udp_motor_send[i].pos = 0.0; // 将当前位置标记为零点
    }
    // 检查是否所有电机完成初始化
    bool allMotorsInitialized = false;
    for (int i = 0; i < 18; ++i)
    {
        if (std::fabs(receive_data.udp_motor_receive[i].pos) > kTolerance)
        {
            allMotorsInitialized = false;
            std::cerr << "Motor " << i << " failed to initialize properly!" << std::endl;
        }
        else        
            allMotorsInitialized = true;
        
    }

    if (allMotorsInitialized)    
        std::cout << "All motors inited to zero position successfully!" << std::endl;
    
    else    
        std::cerr << "Some motors failed to initialize properly!" << std::endl;  
    
}