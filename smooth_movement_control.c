/**
 * @file    smooth_movement_control.c
 * @brief   小车平滑运动控制模块实现 —— 边走边转，丝滑蛇形走位
 *
 * 核心逻辑：
 *  1. 单阶段运动：同时控制距离和角度
 *  2. 距离环：控制 4 个电机的平均行驶距离 → 基础速度
 *  3. 角度转向环：计算角度误差 → 差速修正量
 *  4. 电机输出：基础速度 ± 角度修正 → 左右电机差速实现边走边转
 *  5. 距离优先：距离到达即停止，不管角度是否完全到达
 */

#include "smooth_movement_control.h"
#include "motor_tb6612.h"
#include "encoder.h"
#include <math.h>

/* ---- 内部配置 ---- */
#define SMOOTH_DEADBAND_CM    (0.5f)   /* 距离死区（厘米） */

/* ---- 全局实例 ---- */
SmoothMovementController g_smooth_movement;

/* ---- 内部工具函数 ---- */

/** 对称限幅 */
static float clamp_symmetric(float x, float limit)
{
    if (x > limit)  return limit;
    if (x < -limit) return -limit;
    return x;
}

/** yaw 从 [-180,180] 转换为 [0,360] */
static float yaw_to_360(float yaw_deg)
{
    if (yaw_deg < 0.0f) {
        return yaw_deg + 360.0f;
    }
    return yaw_deg;
}

/** 计算最短路径角度误差 [-180, 180] */
static float angle_error(float target, float current)
{
    float err = target - current;
    if (err > 180.0f) {
        err -= 360.0f;
    } else if (err < -180.0f) {
        err += 360.0f;
    }
    return err;
}

/** 获取 4 个电机的平均行驶距离（厘米） */
static float get_average_distance_cm(SmoothMovementController *move)
{
    /* 计算每个电机相对于起始点的编码器增量 */
    int32_t delta_1 = motor_encoder_1 - move->start_encoder_1;
    int32_t delta_2 = motor_encoder_2 - move->start_encoder_2;
    int32_t delta_3 = motor_encoder_3 - move->start_encoder_3;
    int32_t delta_4 = motor_encoder_4 - move->start_encoder_4;

    /* 转换为距离（厘米） - Get_Motor_Distance() 返回厘米 */
    float dist_1 = Get_Motor_Distance(delta_1);
    float dist_2 = Get_Motor_Distance(delta_2);
    float dist_3 = Get_Motor_Distance(delta_3);
    float dist_4 = Get_Motor_Distance(delta_4);

    /* 返回平均值（厘米） */
    return (dist_1 + dist_2 + dist_3 + dist_4) / 4.0f;
}

/* ================================================================
 *  API 实现
 * ================================================================ */

void SmoothMovement_Init(SmoothMovementController *move)
{
    /* 初始化并级 PID
     * 回路1：距离环（位置控制）
     * 回路2：角度转向环（边走边转） */
    parallel_pid_init(&move->parallel_pid,
        /* 距离环 PID 参数（提速版：更快加速和行驶） */
        500.0f,    /* kp: 提高到 1200，加速更快 */
        8.0f,       /* ki: 提高积分，消除稳态误差 */
        100.0f,     /* kd: 降低阻尼，允许更快响应 */
        10000.0f,    /* max_out: 提高到 9000，最大速度更快 */
        3000.0f,    /* max_i: 积分限幅 */

        /* 角度转向环 PID 参数（加强版：更快转向，更小转弯半径） */
        200.0f,     /* kp: 角度误差 10° → 1500 PWM 差速修正（提高转向力度） */
        5.0f,       /* ki: 消除长期偏航 */
        10.0f,      /* kd: 阻尼（配合 kp 提高） */
        8000.0f,    /* max_out: 角度环最大输出（提高差速修正上限） */
        4000.0f);   /* max_i: 积分限幅 */

    /* 初始化状态 */
    move->target_angle = 0.0f;
    move->target_distance_cm = 0.0f;
    move->current_angle = 0.0f;
    move->current_distance_cm = 0.0f;
    move->state = SMOOTH_IDLE;

    move->start_encoder_1 = 0;
    move->start_encoder_2 = 0;
    move->start_encoder_3 = 0;
    move->start_encoder_4 = 0;

    move->settle_count = 0;
    move->settle_threshold = 12;        /* 缩短到 96ms @ 125Hz，减少卡顿 */
    move->distance_tolerance = 1.0f;    /* 恢复到 1cm，保证精度 */

    move->max_speed = 9000.0f;          /* 提高最大速度 */
    move->min_speed = 2500.0f;          /* 提高最小速度 */
    move->angle_turn_rate = 1.0f;
    move->angle_threshold = 1.0f;       /* 角度到位阈值：3° */
}

void SmoothMovement_SetTarget(SmoothMovementController *move, float angle_deg, float distance_cm)
{
    /* 规范化目标角度到 [0, 360) */
    while (angle_deg >= 360.0f) angle_deg -= 360.0f;
    while (angle_deg < 0.0f)   angle_deg += 360.0f;

    move->target_angle = angle_deg;
    move->target_distance_cm = distance_cm;
    move->current_distance_cm = 0.0f;
    move->settle_count = 0;

    /* 记录编码器起始值 */
    move->start_encoder_1 = motor_encoder_1;
    move->start_encoder_2 = motor_encoder_2;
    move->start_encoder_3 = motor_encoder_3;
    move->start_encoder_4 = motor_encoder_4;

    /* 复位 PID 状态 */
    pid_reset(&move->parallel_pid.branch1);
    pid_reset(&move->parallel_pid.branch2);

    /* 启动运动：阶段1 - 边走边转 */
    move->state = SMOOTH_TURNING;
}

void SmoothMovement_SetTargetBlocking(SmoothMovementController *move, float angle_deg,
                                      float distance_cm, volatile float *yaw_deg_ptr)
{
    /* 启动运动 */
    SmoothMovement_SetTarget(move, angle_deg, distance_cm);

    /* 阻塞等待直到运动完成 */
    while (move->state != SMOOTH_DONE) {
        /* 在主循环中手动调用更新函数
         * 注意：TIM9 中断中也会调用 SmoothMovement_Update()，
         * 但为了确保阻塞期间也能响应，这里也调用一次 */
        __NOP();  /* 等待 TIM9 中断更新 */
    }
}

void SmoothMovement_Update(SmoothMovementController *move, float yaw_deg, float dt)
{
    /* 空闲或完成状态：不干预 */
    if (move->state == SMOOTH_IDLE || move->state == SMOOTH_DONE) {
        return;
    }

    /* 更新当前角度 */
    move->current_angle = yaw_to_360(yaw_deg);

    /* 计算当前已行驶距离（厘米） */
    move->current_distance_cm = get_average_distance_cm(move);

    /* 计算距离误差 */
    float distance_error_cm = move->target_distance_cm - move->current_distance_cm;
    float abs_distance_error = fabsf(distance_error_cm);

    /* 计算角度误差（目标角度 - 当前角度） */
    float angle_err = angle_error(move->target_angle, move->current_angle);
    float abs_angle_err = fabsf(angle_err);

    /* ================================================================
     *  阶段 1：边走边转，优先角度（SMOOTH_TURNING）
     * ================================================================ */
    if (move->state == SMOOTH_TURNING) {
        /* 并级 PID 计算 */
        float base_speed = pid_calc(&move->parallel_pid.branch1,
                                     move->target_distance_cm,
                                     move->current_distance_cm,
                                     dt);

        float angle_correction = pid_calc(&move->parallel_pid.branch2,
                                          0.0f,
                                          -angle_err,
                                          dt);

        angle_correction *= move->angle_turn_rate;

        /* 限幅基础速度 */
        base_speed = clamp_symmetric(base_speed, move->max_speed);

        /* 转弯时保持高速，确保转弯半径小 */
        if (abs_angle_err > 5.0f) {
            float min_turn_speed = move->max_speed * 0.75f;
            if (base_speed > 0.0f && base_speed < min_turn_speed) {
                base_speed = min_turn_speed;
            } else if (base_speed < 0.0f && base_speed > -min_turn_speed) {
                base_speed = -min_turn_speed;
            }
        }

        /* 静摩擦补偿 */
        if (base_speed > 0.0f && base_speed < move->min_speed) {
            base_speed = move->min_speed;
        } else if (base_speed < 0.0f && base_speed > -move->min_speed) {
            base_speed = -move->min_speed;
        }

        /* 计算左右电机速度 */
        float left_speed = base_speed - angle_correction;
        float right_speed = base_speed + angle_correction;

        left_speed = clamp_symmetric(left_speed, move->max_speed);
        right_speed = clamp_symmetric(right_speed, move->max_speed);

        /* 输出到电机 */
        motor_set(1, (int16_t)left_speed);
        motor_set(2, (int16_t)left_speed);
        motor_set(3, (int16_t)right_speed);
        motor_set(4, (int16_t)right_speed);

        /* 判断角度是否到位 */
        if (abs_angle_err < move->angle_threshold) {
            /* 角度到位，立即切换到阶段2，不等待稳定时间 */
            move->state = SMOOTH_STRAIGHT;
            move->settle_count = 0;

            /* 重新记录编码器起始值（用于计算剩余距离） */
            move->start_encoder_1 = motor_encoder_1;
            move->start_encoder_2 = motor_encoder_2;
            move->start_encoder_3 = motor_encoder_3;
            move->start_encoder_4 = motor_encoder_4;

            /* 复位距离环 PID，但保留角度环状态（保持连贯性） */
            pid_reset(&move->parallel_pid.branch1);
            /* 注意：不复位 branch2，保持角度修正的连续性 */
        }

        /* 距离用完但角度还没到位：停止（距离不足） */
        if (abs_distance_error < SMOOTH_DEADBAND_CM) {
            move->state = SMOOTH_DONE;
            motor_set(1, 0);
            motor_set(2, 0);
            motor_set(3, 0);
            motor_set(4, 0);
            pid_reset(&move->parallel_pid.branch1);
            pid_reset(&move->parallel_pid.branch2);
        }
    }

    /* ================================================================
     *  阶段 2：角度到位，保持直行（SMOOTH_STRAIGHT）
     * ================================================================ */
    else if (move->state == SMOOTH_STRAIGHT) {
        /* 重新计算剩余距离（从阶段2开始点算起） */
        float straight_distance = get_average_distance_cm(move);
        float remaining_distance = move->target_distance_cm -
                                   (move->current_distance_cm - straight_distance);

        /* 距离环 PID：控制直行速度 */
        float base_speed = pid_calc(&move->parallel_pid.branch1,
                                     remaining_distance,
                                     straight_distance,
                                     dt);

        /* 角度保持环：保持目标角度直行 */
        float angle_correction = pid_calc(&move->parallel_pid.branch2,
                                          0.0f,
                                          -angle_err,
                                          dt);

        angle_correction *= 0.7f;  /* 直行阶段角度修正降低到 60%，更稳定 */

        /* 限幅 */
        base_speed = clamp_symmetric(base_speed, move->max_speed);

        /* 静摩擦补偿 */
        if (base_speed > 0.0f && base_speed < move->min_speed) {
            base_speed = move->min_speed;
        } else if (base_speed < 0.0f && base_speed > -move->min_speed) {
            base_speed = -move->min_speed;
        }

        /* 计算左右电机速度 */
        float left_speed = base_speed - angle_correction;
        float right_speed = base_speed + angle_correction;

        left_speed = clamp_symmetric(left_speed, move->max_speed);
        right_speed = clamp_symmetric(right_speed, move->max_speed);

        /* 输出到电机 */
        motor_set(1, (int16_t)left_speed);
        motor_set(2, (int16_t)left_speed);
        motor_set(3, (int16_t)right_speed);
        motor_set(4, (int16_t)right_speed);

        /* 完成判定：剩余距离到达 */
        float remaining_error = fabsf(remaining_distance - straight_distance);
        if (remaining_error < move->distance_tolerance) {
            move->settle_count++;
            if (move->settle_count >= move->settle_threshold) {
                /* 运动完成 */
                move->state = SMOOTH_DONE;
                motor_set(1, 0);
                motor_set(2, 0);
                motor_set(3, 0);
                motor_set(4, 0);
                pid_reset(&move->parallel_pid.branch1);
                pid_reset(&move->parallel_pid.branch2);
            }
        } else {
            move->settle_count = 0;
        }
    }
}

void SmoothMovement_Stop(SmoothMovementController *move)
{
    move->state = SMOOTH_IDLE;
    move->settle_count = 0;

    /* 复位 PID */
    pid_reset(&move->parallel_pid.branch1);
    pid_reset(&move->parallel_pid.branch2);

    /* 停止所有电机 */
    motor_set(1, 0);
    motor_set(2, 0);
    motor_set(3, 0);
    motor_set(4, 0);
}

void SmoothMovement_SetDistanceTuning(SmoothMovementController *move, float kp, float ki, float kd)
{
    pid_set_tuning(&move->parallel_pid.branch1, kp, ki, kd);
}

void SmoothMovement_SetAngleTuning(SmoothMovementController *move, float kp, float ki, float kd)
{
    pid_set_tuning(&move->parallel_pid.branch2, kp, ki, kd);
}

SmoothMovementState SmoothMovement_GetState(SmoothMovementController *move)
{
    return move->state;
}

float SmoothMovement_GetCurrentDistance(SmoothMovementController *move)
{
    return move->current_distance_cm;
}

float SmoothMovement_GetAngleError(SmoothMovementController *move)
{
    return angle_error(move->target_angle, move->current_angle);
}

bool SmoothMovement_IsDone(SmoothMovementController *move)
{
    return (move->state == SMOOTH_DONE);
}

/* ========================================================================
 *  使用说明
 * ========================================================================
 *
 * 【功能】
 *   小车边走边转，丝滑到达目标角度和距离
 *   - 单阶段运动：同时控制距离和角度，无停顿
 *   - 距离优先：距离到达即停止，不管角度是否完全到达
 *   - 适合蛇形走位、连续转弯
 *
 * 【与 movement_control 的区别】
 *
 *   movement_control（两阶段模式）：
 *     1. 先原地旋转到目标角度
 *     2. 再直线前进到目标距离
 *     优点：精确到达目标角度和距离
 *     缺点：有停顿，不够流畅
 *
 *   smooth_movement_control（单阶段模式）：
 *     1. 边走边转，一气呵成
 *     2. 距离到达即停止
 *     优点：流畅无停顿，适合蛇形走位
 *     缺点：如果距离太短，可能无法完全到达目标角度
 *
 * 【快速开始】
 *
 *   // 1. 在 main() 中初始化
 *   Encoder_Init();
 *   motor_init();
 *   SmoothMovement_Init(&g_smooth_movement);
 *   HAL_TIM_Base_Start_IT(&htim9);  // 启动 125Hz 定时器
 *
 *   // 2. 设置目标：90度方向前进50厘米（边走边转）
 *   SmoothMovement_SetTarget(&g_smooth_movement, 90.0f, 50.0f);
 *
 *   // 3. 在 TIM9 中断中更新（125Hz）
 *   void TIM9_IRQHandler(void) {
 *       if (__HAL_TIM_GET_FLAG(&htim9, TIM_FLAG_UPDATE)) {
 *           __HAL_TIM_CLEAR_FLAG(&htim9, TIM_FLAG_UPDATE);
 *           Encoder_Update_Counts();
 *           SmoothMovement_Update(&g_smooth_movement, yaw, 0.008f);
 *       }
 *   }
 *
 *   // 4. 检查是否完成
 *   if (SmoothMovement_IsDone(&g_smooth_movement)) {
 *       // 运动完成，执行下一个任务
 *   }
 *
 * 【蛇形走位示例】
 *
 *   // 连续蛇形走位，无停顿
 *   typedef enum {
 *       SNAKE_IDLE, SNAKE_1, SNAKE_2, SNAKE_3, SNAKE_4, SNAKE_DONE
 *   } SnakeState;
 *   SnakeState snake = SNAKE_IDLE;
 *
 *   void Snake_Update(void) {
 *       switch (snake) {
 *           case SNAKE_IDLE:
 *               // 0度方向前进30cm
 *               SmoothMovement_SetTarget(&g_smooth_movement, 0.0f, 30.0f);
 *               snake = SNAKE_1;
 *               break;
 *           case SNAKE_1:
 *               if (SmoothMovement_IsDone(&g_smooth_movement)) {
 *                   // 45度方向前进30cm（右转边走）
 *                   SmoothMovement_SetTarget(&g_smooth_movement, 45.0f, 30.0f);
 *                   snake = SNAKE_2;
 *               }
 *               break;
 *           case SNAKE_2:
 *               if (SmoothMovement_IsDone(&g_smooth_movement)) {
 *                   // 315度方向前进30cm（左转边走）
 *                   SmoothMovement_SetTarget(&g_smooth_movement, 315.0f, 30.0f);
 *                   snake = SNAKE_3;
 *               }
 *               break;
 *           case SNAKE_3:
 *               if (SmoothMovement_IsDone(&g_smooth_movement)) {
 *                   // 0度方向前进30cm（回正边走）
 *                   SmoothMovement_SetTarget(&g_smooth_movement, 0.0f, 30.0f);
 *                   snake = SNAKE_4;
 *               }
 *               break;
 *           case SNAKE_4:
 *               if (SmoothMovement_IsDone(&g_smooth_movement)) {
 *                   snake = SNAKE_DONE;
 *               }
 *               break;
 *       }
 *   }
 *
 * 【API 说明】
 *
 *   SmoothMovement_SetTarget(move, angle_deg, distance_cm)
 *     - angle_deg: 目标角度 0~360° (0°正前方, 90°右侧, 180°后方, 270°左侧)
 *     - distance_cm: 目标距离，厘米（正数前进，负数后退）
 *     - 注意：距离优先，距离到达即停止
 *
 *   SmoothMovement_Update(move, yaw_deg, dt)
 *     - yaw_deg: 陀螺仪偏航角 -180~180°
 *     - dt: 时间间隔，秒（125Hz → 0.008s）
 *
 *   SmoothMovement_Stop(move)
 *     - 立即停止运动，复位所有状态
 *
 *   SmoothMovement_IsDone(move)
 *     - 返回 true 表示运动完成
 *
 *   SmoothMovement_GetCurrentDistance(move)
 *     - 返回当前已行驶距离（厘米）
 *
 *   SmoothMovement_GetAngleError(move)
 *     - 返回当前角度误差（度），正数需要右转，负数需要左转
 *
 * 【在线调参】
 *
 *   // 调整距离环 PID（控制行驶距离）
 *   SmoothMovement_SetDistanceTuning(&g_smooth_movement, 800.0f, 5.0f, 150.0f);
 *
 *   // 调整角度转向环 PID（控制转向速度）
 *   SmoothMovement_SetAngleTuning(&g_smooth_movement, 80.0f, 2.0f, 20.0f);
 *
 * 【默认 PID 参数】
 *
 *   距离环：Kp=800, Ki=5, Kd=150, max_out=6000, max_i=2000
 *   角度转向环：Kp=80, Ki=2, Kd=20, max_out=4000, max_i=1500
 *
 * 【调参建议】
 *
 *   转向太慢（弯转不过来）：
 *     - 增加角度转向环 Kp（如 100）
 *     - 增加 max_out（如 5000）
 *
 *   转向太快（摆动严重）：
 *     - 降低角度转向环 Kp（如 60）
 *     - 增加角度转向环 Kd（如 30）
 *
 *   距离不准：
 *     - 调整距离环参数（同 movement_control）
 *
 * 【注意事项】
 *
 *   1. 距离优先：距离到达即停止，不管角度是否完全到达
 *   2. 如果需要精确到达目标角度，请使用 movement_control（两阶段模式）
 *   3. 转弯半径取决于距离和角度差：
 *      - 距离长 + 角度差小 = 大半径缓弯
 *      - 距离短 + 角度差大 = 小半径急弯（可能转不过来）
 *   4. 陀螺仪必须正常工作
 *   5. 适合连续蛇形走位，每段距离建议 > 20cm
 *
 * 【工作原理】
 *
 *   距离环：根据 4 个编码器平均值计算已行驶距离，PID 输出基础速度
 *   角度转向环：根据陀螺仪计算角度误差，PID 输出差速修正量
 *   左侧电机 = 基础速度 - 角度修正（角度误差为正时，左侧加速）
 *   右侧电机 = 基础速度 + 角度修正（角度误差为正时，右侧减速）
 *   完成判定：距离误差 < 1cm 且稳定 200ms → 停止电机 → SMOOTH_DONE
 *
 * ======================================================================== */
