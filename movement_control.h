/**
 * @file    movement_control.h
 * @brief   小车运动控制模块 —— 以指定角度行驶指定距离
 *
 * 功能：
 *  1. 先旋转到目标角度（复用 rotation_control 模块）
 *  2. 保持角度直线行驶指定距离（并级 PID：距离环 + 角度保持环）
 *  3. 非阻塞设计：Movement_Update() 在定时器中断中周期调用
 *
 * 使用示例：
 *  Movement_Init(&g_movement);
 *  Movement_SetTarget(&g_movement, 90.0f, 50.0f);  // 90度方向行驶50cm
 *  // 在 TIM9 中断 (125Hz) 中调用：
 *  Movement_Update(&g_movement, yaw_deg, 0.008f);
 */

#ifndef MOVEMENT_CONTROL_H
#define MOVEMENT_CONTROL_H

#include "pid_optimized.h"
#include "rotation_control.h"
#include <stdint.h>
#include <stdbool.h>

/* ---- 运动状态机 ---- */
typedef enum {
    MOVE_IDLE = 0,      /* 未激活 */
    MOVE_ROTATING,      /* 正在旋转到目标角度 */
    MOVE_MOVING,        /* 正在直线行驶 */
    MOVE_DONE           /* 运动完成 */
} MovementState;

/* ---- 运动控制器 ---- */
typedef struct {
    /* 目标参数 */
    float target_angle;         /* 目标角度 0~360° */
    float target_distance_cm;   /* 目标距离（厘米） */

    /* 当前状态 */
    float current_angle;        /* 当前偏航角 0~360° */
    float current_distance_cm;  /* 当前已行驶距离（厘米） */
    MovementState state;        /* 当前状态 */

    /* PID 控制器 */
    ParallelPID parallel_pid;   /* 并级 PID：距离环 + 角度保持环 */

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

    /* 旋转控制器引用 */
    RotationController *rotation_ctrl;
} MovementController;

/* ---- 全局控制器实例 ---- */
extern MovementController g_movement;

/* ---- API ---- */

/**
 * @brief 初始化运动控制器
 * @param move 运动控制器实例
 * @param rot_ctrl 旋转控制器实例（用于旋转阶段）
 */
void Movement_Init(MovementController *move, RotationController *rot_ctrl);

/**
 * @brief 设置目标角度和距离，启动运动
 * @param move 运动控制器实例
 * @param angle_deg 目标角度 0~360°（相对于世界坐标系）
 * @param distance_cm 目标距离（厘米，正数表示前进，负数表示后退）
 *
 * 注意：非阻塞函数，需要在 TIM9 中断中周期调用 Movement_Update()
 */
void Movement_SetTarget(MovementController *move, float angle_deg, float distance_cm);

/**
 * @brief 设置目标角度和距离，阻塞等待直到运动完成（推荐使用）
 * @param move 运动控制器实例
 * @param angle_deg 目标角度 0~360°（相对于世界坐标系）
 * @param distance_cm 目标距离（厘米，正数表示前进，负数表示后退）
 * @param yaw_deg_ptr 指向全局 yaw_deg 变量的指针（从串口 DMA 中断更新）
 *
 * 注意：阻塞函数，会等待直到运动完成才返回，适合连续运动指令
 *
 * 使用示例：
 *   Movement_SetTargetBlocking(&g_movement, 0.0f, 30.0f, &yaw_deg);
 *   Movement_SetTargetBlocking(&g_movement, 90.0f, 40.0f, &yaw_deg);
 */
void Movement_SetTargetBlocking(MovementController *move, float angle_deg,
                                 float distance_cm, volatile float *yaw_deg_ptr);

/**
 * @brief 每控制周期调用（TIM9 ISR, 125Hz），dt=0.008f
 * @param move 运动控制器实例
 * @param yaw_deg 当前偏航角（陀螺仪读数，-180~180°）
 * @param dt 时间间隔（秒）
 */
void Movement_Update(MovementController *move, float yaw_deg, float dt);

/**
 * @brief 停止运动，电机归零，回到 IDLE 状态
 * @param move 运动控制器实例
 */
void Movement_Stop(MovementController *move);

/**
 * @brief 运行时调整距离环 PID 参数
 * @param move 运动控制器实例
 * @param kp, ki, kd PID 参数
 */
void Movement_SetDistanceTuning(MovementController *move, float kp, float ki, float kd);

/**
 * @brief 运行时调整角度保持环 PID 参数
 * @param move 运动控制器实例
 * @param kp, ki, kd PID 参数
 */
void Movement_SetAngleTuning(MovementController *move, float kp, float ki, float kd);

/**
 * @brief 获取当前状态
 * @param move 运动控制器实例
 * @return 当前运动状态
 */
MovementState Movement_GetState(MovementController *move);

/**
 * @brief 获取当前已行驶距离（厘米）
 * @param move 运动控制器实例
 * @return 已行驶距离（厘米）
 */
float Movement_GetCurrentDistance(MovementController *move);

/**
 * @brief 检查运动是否完成
 * @param move 运动控制器实例
 * @return true 表示完成，false 表示未完成
 */
bool Movement_IsDone(MovementController *move);

#endif /* MOVEMENT_CONTROL_H */
