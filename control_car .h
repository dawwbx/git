#ifndef CONTROL_CAR_H
#define CONTROL_CAR_H

#include "main.h"
#include "motor_tb6612.h"
#include "pid.h"

/* ================= 车辆物理参数定义 (请务必根据实车修改) ================= */

// 轮子直径 (单位: cm)
#define WHEEL_DIAMETER  6.5f

// 轮距 (左右轮中心间距, 单位: cm)，用于差速转弯计算
#define TRACK_WIDTH     15.0f

// 编码器相关
// 减速比 * 编码器线数 * 4 (倍频)
// 例如：减速比30，线数13，4倍频 = 30*13*4 = 1560
#define ENCODER_TICKS_PER_REV  1560.0f

/* ================= 函数声明 ================= */

/**
 * @brief 初始化控制模块（包括PID参数初始化）
 */
void Car_Control_Init(void);

/**
 * @brief 小车直行前进固定距离
 * @param distance 距离(cm)
 * @param target_heading 目标朝向(-180~180)
 * @param tolerance 允许距离误差(cm)
 */
void Car_Go_Straight(float distance, float target_heading, float tolerance);

/**
 * @brief 小车原地转向
 * @param dir 1:左转, -1:右转
 * @param target_abs_angle 目标绝对角度(-180~180)
 */
void Car_Turn_In_Place(int8_t dir, float target_abs_angle);

/**
 * @brief 小车转直角弯和直行融合函数
 * @param distance 行驶距离 或 转弯半径(cm)
 * @param target_angle 运动后的目标朝向
 * @param dist_allow_err 允许距离误差
 * @param angle_allow_err 允许角度误差
 * * @note 逻辑：
 * 1. 若当前角度 == target_angle: 直行 distance 距离。
 * 2. 若当前角度 != target_angle: 以 distance 为半径，差速转弯直到朝向为 target_angle。
 */
void Car_Smart_Move(float distance, float target_angle, float dist_allow_err, float angle_allow_err);

#endif // CONTROL_CAR_H
