/**
 * @file rt_control.h
 * @brief Xenomai实时控制程序头文件 - 六足机器人标定控制
 * @author Based on work by Haowen Liang
 * @date 2025-6-4
 *
 * @copyright Copyright (C) 2025.
 *
 */
#ifndef RT_CONTROL_H
#define RT_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

/* 初始化实时控制系统 */
bool init_rt_control();

/* 清理实时控制系统 */
void cleanup_rt_control();

/* 获取标定完成状态 */
bool is_calibration_completed();

/* 重置标定状态 */
void reset_calibration();

/* 获取控制周期统计信息 */
void get_cycle_stats(long* min_jitter, long* max_jitter, 
                    long* avg_jitter, long* last_jitter,
                    unsigned long* count);

#ifdef __cplusplus
}
#endif

#endif /* RT_CONTROL_H */
