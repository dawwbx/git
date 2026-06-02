/**
 * @file    smooth_movement_control.h
 * @brief   小车平滑运动控制模块 —— 边走边转，丝滑到达目标角度和距离
 *
 * 功能：
 *  1. 单阶段运动：同时控制距离和角度，边走边转
 *  2. 距离优先：距离到达即停止，不管角度是否完全到达
 *  3. 并级 PID：距离环 + 角度转向环
 *  4. 非阻塞设计：SmoothMovement_Update() 在定时器中断中周期调用
 *  5. 适合连续蛇形走位，无停顿
 *
 * 与 movement_control 的区别：
 *  - movement_control: 先原地旋转到目标角度，再直线前进（两阶段，有停顿）
 *  - smooth_movement_control: 边走边转，一气呵成（单阶段，无停顿）
 *
 * 使用场景：
 *  - 蛇形走位、绕障、连续转弯
 *  - 需要流畅运动轨迹的场合
 */

#ifndef SMOOTH_MOVEMENT_CONTROL_H
#define SMOOTH_MOVEMENT_CONTROL_H

#include "pid_optimized.h"
#include <stdint.h>
#include <stdbool.h>

/* ---- 运动状态机 ---- */
typedef enum {
    SMOOTH_IDLE = 0,    /* 未激活 */
    SMOOTH_TURNING,     /* 阶段1：边走边转，优先角度（转弯阶段） */
    SMOOTH_STRAIGHT,    /* 阶段2：角度到位，保持直行（直行阶段） */
    SMOOTH_DONE         /* 运动完成 */
} SmoothMovementState;

/* ---- 平滑运动控制器 ---- */
typedef struct {
    /* 目标参数 */
    float target_angle;         /* 目标角度 0~360° */
    float target_distance_cm;   /* 目标距离（厘米） */

    /* 当前状态 */
    float current_angle;        /* 当前偏航角 0~360° */
    float current_distance_cm;  /* 当前已行驶距离（厘米） */
    SmoothMovementState state;  /* 当前状态 */

    /* PID 控制器 */
    ParallelPID parallel_pid;   /* 并级 PID：距离环 + 角度转向环 */

    /* 编码器起始值（用于计算相对距离） */
    int32_t start_encoder_1;
    int32_t start_encoder_2;
    int32_t start_encoder_3;
    int32_t start_encoder_4;

    /* 完成判定 */
    uint16_t settle_count;      /* 连续在容差范围内的周期计数 */
    uint16_t settle_threshold;  /* 判定完成的连续周期数（默认 25 = 200ms @125Hz） */
    float distance_tolerance;   /* 距离容差（厘米），默认 1.0cm */

    /* 速度限制 */
    float max_speed;            /* 最大行驶速度（PWM 值），默认 6000 */
    float min_speed;            /* 最小驱动速度（克服静摩擦），默认 2000 */

    /* 角度控制参数 */
    float angle_turn_rate;      /* 角度转向速率系数，默认 1.0 */
    float angle_threshold;      /* 角度到位阈值（度），默认 3.0° */
} SmoothMovementController;

/* ---- 全局控制器实例 ---- */
extern SmoothMovementController g_smooth_movement;

/* ---- API ---- */

/**
 * @brief 初始化平滑运动控制器
 * @param move 平滑运动控制器实例
 */
void SmoothMovement_Init(SmoothMovementController *move);

/**
 * @brief 设置目标角度和距离，启动平滑运动
 * @param move 平滑运动控制器实例
 * @param angle_deg 目标角度 0~360°（相对于世界坐标系）
 * @param distance_cm 目标距离（厘米，正数表示前进，负数表示后退）
 *
 * 注意：非阻塞函数，需要在 TIM9 中断中周期调用 SmoothMovement_Update()
 */
void SmoothMovement_SetTarget(SmoothMovementController *move, float angle_deg, float distance_cm);

/**
 * @brief 设置目标角度和距离，阻塞等待直到运动完成（推荐使用）
 * @param move 平滑运动控制器实例
 * @param angle_deg 目标角度 0~360°（相对于世界坐标系）
 * @param distance_cm 目标距离（厘米，正数表示前进，负数表示后退）
 * @param yaw_deg_ptr 指向全局 yaw_deg 变量的指针（从串口 DMA 中断更新）
 *
 * 注意：阻塞函数，会等待直到运动完成才返回，适合连续运动指令
 *
 * 使用示例：
 *   SmoothMovement_SetTargetBlocking(&g_smooth_movement, 0.0f, 30.0f, &yaw_deg);
 *   SmoothMovement_SetTargetBlocking(&g_smooth_movement, 45.0f, 30.0f, &yaw_deg);
 *   SmoothMovement_SetTargetBlocking(&g_smooth_movement, 90.0f, 30.0f, &yaw_deg);
 */
void SmoothMovement_SetTargetBlocking(SmoothMovementController *move, float angle_deg,
                                      float distance_cm, volatile float *yaw_deg_ptr);

/**
 * @brief 每控制周期调用（TIM9 ISR, 125Hz），dt=0.008f
 * @param move 平滑运动控制器实例
 * @param yaw_deg 当前偏航角（陀螺仪读数，-180~180°）
 * @param dt 时间间隔（秒）
 */
void SmoothMovement_Update(SmoothMovementController *move, float yaw_deg, float dt);

/**
 * @brief 停止运动，电机归零，回到 IDLE 状态
 * @param move 平滑运动控制器实例
 */
void SmoothMovement_Stop(SmoothMovementController *move);

/**
 * @brief 运行时调整距离环 PID 参数
 * @param move 平滑运动控制器实例
 * @param kp, ki, kd PID 参数
 */
void SmoothMovement_SetDistanceTuning(SmoothMovementController *move, float kp, float ki, float kd);

/**
 * @brief 运行时调整角度转向环 PID 参数
 * @param move 平滑运动控制器实例
 * @param kp, ki, kd PID 参数
 */
void SmoothMovement_SetAngleTuning(SmoothMovementController *move, float kp, float ki, float kd);

/**
 * @brief 获取当前状态
 * @param move 平滑运动控制器实例
 * @return 当前运动状态
 */
SmoothMovementState SmoothMovement_GetState(SmoothMovementController *move);

/**
 * @brief 获取当前已行驶距离（厘米）
 * @param move 平滑运动控制器实例
 * @return 已行驶距离（厘米）
 */
float SmoothMovement_GetCurrentDistance(SmoothMovementController *move);

/**
 * @brief 获取当前角度误差（度）
 * @param move 平滑运动控制器实例
 * @return 角度误差（度），正数表示需要右转，负数表示需要左转
 */
float SmoothMovement_GetAngleError(SmoothMovementController *move);

/**
 * @brief 检查运动是否完成
 * @param move 平滑运动控制器实例
 * @return true 表示完成，false 表示未完成
 */
bool SmoothMovement_IsDone(SmoothMovementController *move);

#endif /* SMOOTH_MOVEMENT_CONTROL_H */
