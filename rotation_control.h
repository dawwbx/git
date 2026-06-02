/**
 * @file    rotation_control.h
 * @brief   小车 PID 旋转控制模块 —— 控制车头旋转到指定角度 (0~360°)
 *
 * 使用差速转向：左侧电机(1,2) 与 右侧电机(3,4) 反向旋转，实现原地转向。
 * 非阻塞设计：Rotation_Update() 在 TIM9 中断 (125Hz) 中周期调用。
 * 复用 fun/pid_optimized.h 的单级 PID 控制器。
 */

#ifndef ROTATION_CONTROL_H
#define ROTATION_CONTROL_H

#include "pid_optimized.h"
#include <stdint.h>

/* ---- 旋转状态机 ---- */
typedef enum {
    ROT_IDLE = 0,       /* 未激活，不影响正常行驶 */
    ROT_ACTIVE,         /* 正在旋转到目标角度 */
    ROT_DONE            /* 旋转完成（已稳定在目标角度容差范围内） */
} RotationState;

/* ---- 旋转控制器 ---- */
typedef struct {
    PID pid;                        /* 角度 PID 控制器 */
    float target_angle;             /* 目标角度 0~360° */
    float current_angle;            /* 当前偏航角 0~360°（从 yaw_deg 转换） */
    float output;                   /* PID 输出 = 旋转速度 (PWM 值) */
    RotationState state;            /* 当前状态 */
    uint16_t settle_count;          /* 连续在容差范围内的周期计数 */
    uint16_t settle_threshold;      /* 判定完成的连续周期数 (默认 25 = 200ms @125Hz) */
    float angle_tolerance;          /* 完成判定的角度容差 (度)，默认 2° */
    float max_rotation_speed;       /* 最大旋转速度 (PWM 值)，默认 4500 */
} RotationController;

/* ---- 全局控制器实例 ---- */
extern RotationController g_rotation;

/* ---- API ---- */

/** 初始化控制器，设置默认 PID 参数和阈值 */
void Rotation_Init(RotationController *rot);

/** 设置目标角度 (0~360°)，自动启动旋转，复位 PID 状态 */
void Rotation_SetTarget(RotationController *rot, float target_deg);

/** 每控制周期调用 (TIM9 ISR, 125Hz)，dt=0.008f */
void Rotation_Update(RotationController *rot, float yaw_deg, float dt);

/** 停止旋转，电机归零，回到 IDLE 状态 */
void Rotation_Stop(RotationController *rot);

/** 运行时调整 PID 参数 (方便在线调参) */
void Rotation_SetTuning(RotationController *rot, float kp, float ki, float kd);

/** 获取当前状态 */
RotationState Rotation_GetState(RotationController *rot);

/** 获取当前 PID 输出 (调试用) */
float Rotation_GetOutput(RotationController *rot);

#endif /* ROTATION_CONTROL_H */
