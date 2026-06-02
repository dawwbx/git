#ifndef CONTROL_CAR_H
#define CONTROL_CAR_H

#include "main.h"
#include "pid.h"
#include "motor_tb6612.h"
#include <math.h>
#include <stdlib.h>

// ================= 用户参数配置 (需根据实车修改) =================

// 1. 物理参数
#define WHEEL_DIAMETER_CM    6.0f    // 麦轮直径 (cm)
#define ENCODER_PPR          2496.0f // 电机转一圈的编码器总脉冲数 (线数*减速比*4倍频)
#define PI                   3.1415926f

// 2. 距离转换比例: 1cm 对应的编码器脉冲数
// Calculation: Total_Pulse / (PI * Diameter)
#define PULSE_PER_CM         (ENCODER_PPR / (PI * WHEEL_DIAMETER_CM))

// 3. 速度限制 (PWM 占空比或编码器速度，取决于你的motor_set定义)
#define MAX_SPEED_PWM        7200.0f  // 最大输出限制
#define MOVE_SPEED_BASE      1000.0f  // 默认巡航速度

// 4. 阈值
#define ANGLE_DEADZONE       1.0f    // 角度死区
#define DIST_DEADZONE        0.5f    // 距离死区 (cm)

// ================= 全局变量声明 =================
// 假设这些变量在 main.c 或其他地方定义并实时更新
extern float angle;            // 当前IMU角度 (-180.0 ~ 180.0)
extern int32_t motor_encoder_1; // 左前 FL
extern int32_t motor_encoder_2; // 右前 FR
extern int32_t motor_encoder_3; // 左后 RL
extern int32_t motor_encoder_4; // 右后 RR

// ================= 控制结构体 =================
typedef struct {
    PID speed_pid;      // 速度环/距离环 PID
    PID heading_pid;    // 角度/航向环 PID
    
    // 目标状态
    float target_dist;
    float target_angle;
    
    // 当前累计距离 (通过编码器计算)
    float current_dist_x;
    float current_dist_y;
} Car_Control_t;

// ================= 函数接口 =================

/**
 * @brief 初始化小车控制参数和PID
 */
void Car_Control_Init(void);

/**
 * @brief 麦轮运动学解算与电机输出 (底层核心)
 * @param vx  X轴速度 (平移: 左- 右+)
 * @param vy  Y轴速度 (直行: 后- 前+)
 * @param vw  W轴速度 (旋转: 顺- 逆+)
 */
void Car_Set_Velocity(float vx, float vy, float vw);

/**
 * @brief 1. 小车左右平移函数 (阻塞式)
 * @param distance_cm 距离 (正数向右，负数向左)
 */
void Car_Move_Strafe(float distance_cm);

/**
 * @brief 2. 小车直行前进函数 (阻塞式)
 * @param distance_cm 距离 (正数向前，负数向后)
 * @param tolerance_cm 允许偏差 (cm)
 */
void Car_Move_Straight(float distance_cm, float tolerance_cm);

/**
 * @brief 3. 小车原地转向函数 (阻塞式)
 * @param target_angle 目标绝对角度 (-180 ~ 180)
 */
void Car_Turn_Spot(float target_angle);

/**
 * @brief 4. 小车直角转弯 (圆弧运动)
 * @param direction 1: 向右转, -1: 向左转
 * @param radius_cm 转弯半径
 * @param target_angle_diff 转弯后的角度变化量 (通常是 90 或 -90)
 */
void Car_Turn_Arc(int8_t direction, float radius_cm, float target_angle_diff);

/**
 * @brief 停止所有电机
 */
void Car_Stop(void);

#endif
