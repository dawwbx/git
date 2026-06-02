/**
 * @file    movement_control.c
 * @brief   小车运动控制模块实现
 *
 * 核心逻辑：
 *  1. ROTATING 阶段：调用 rotation_control 模块旋转到目标角度
 *  2. MOVING 阶段：使用并级 PID 控制
 *     - 距离环：控制 4 个电机的平均行驶距离
 *     - 角度保持环：保持目标角度，输出差速修正量
 *  3. 电机输出：基础速度 ± 角度修正 → 左右电机差速
 *  4. 完成判定：距离误差在容差内且稳定 → DONE
 */

#include "movement_control.h"
#include "motor_tb6612.h"
#include "encoder.h"
#include <math.h>

/* ---- 内部配置 ---- */
#define MOVE_DEADBAND_CM    (0.5f)   /* 距离死区（厘米） */
#define ANGLE_DEADBAND_DEG  (1.0f)   /* 角度死区（度） */

/* ---- 全局实例 ---- */
MovementController g_movement;

/* ---- 内部工具函数 ---- */

/** 对称限幅 */
static float clamp_symmetric(float x, float limit)
{
    if (x > limit)  return limit;
    if (x < -limit) return -limit;
    return x;
}

/** 限幅到指定范围 */
static float clamp(float x, float min, float max)
{
    if (x < min) return min;
    if (x > max) return max;
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
static float get_average_distance_cm(MovementController *move)
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

void Movement_Init(MovementController *move, RotationController *rot_ctrl)
{
    /* 保存旋转控制器引用 */
    move->rotation_ctrl = rot_ctrl;

    /* 初始化并级 PID
     * 回路1：距离环（位置控制）
     * 回路2：角度保持环（方向修正） */
    parallel_pid_init(&move->parallel_pid,
        /* 距离环 PID 参数 */
        800.0f,     /* kp: 距离误差 10cm → 8000 PWM */
        5.0f,       /* ki: 消除稳态误差 */
        150.0f,     /* kd: 阻尼，防止冲过头 */
        6000.0f,    /* max_out: 距离环最大输出 */
        2000.0f,    /* max_i: 积分限幅 */

        /* 角度保持环 PID 参数 */
        60.0f,      /* kp: 角度误差 5° → 300 PWM 差速修正 */
        1.0f,       /* ki: 消除长期偏航 */
        15.0f,      /* kd: 阻尼 */
        3000.0f,    /* max_out: 角度环最大输出（差速修正量） */
        1000.0f);   /* max_i: 积分限幅 */

    /* 初始化状态 */
    move->target_angle = 0.0f;
    move->target_distance_cm = 0.0f;
    move->current_angle = 0.0f;
    move->current_distance_cm = 0.0f;
    move->state = MOVE_IDLE;

    move->start_encoder_1 = 0;
    move->start_encoder_2 = 0;
    move->start_encoder_3 = 0;
    move->start_encoder_4 = 0;

    move->settle_count = 0;
    move->settle_threshold = 25;        /* 200ms @ 125Hz */
    move->distance_tolerance = 1.0f;    /* 1cm */

    move->max_speed = 6000.0f;
    move->min_speed = 2000.0f;
}

void Movement_SetTarget(MovementController *move, float angle_deg, float distance_cm)
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

    /* 启动旋转阶段 */
    move->state = MOVE_ROTATING;
    Rotation_SetTarget(move->rotation_ctrl, angle_deg);
}

void Movement_SetTargetBlocking(MovementController *move, float angle_deg,
                                 float distance_cm, volatile float *yaw_deg_ptr)
{
    /* 启动运动 */
    Movement_SetTarget(move, angle_deg, distance_cm);

    /* 阻塞等待直到运动完成 */
    while (move->state != MOVE_DONE) {
        /* 等待 TIM9 中断更新 */
        __NOP();
    }
}

void Movement_Update(MovementController *move, float yaw_deg, float dt)
{
    /* 空闲或完成状态：不干预 */
    if (move->state == MOVE_IDLE || move->state == MOVE_DONE) {
        return;
    }

    /* 更新当前角度 */
    move->current_angle = yaw_to_360(yaw_deg);

    /* ---- 阶段 1：旋转到目标角度 ---- */
    if (move->state == MOVE_ROTATING) {
        /* 调用旋转控制器更新 */
        Rotation_Update(move->rotation_ctrl, yaw_deg, dt);

        /* 检查旋转是否完成 */
        if (Rotation_GetState(move->rotation_ctrl) == ROT_DONE) {
            /* 旋转完成，进入直线行驶阶段 */
            move->state = MOVE_MOVING;
            move->settle_count = 0;

            /* 重新记录编码器起始值（旋转过程中编码器会变化） */
            move->start_encoder_1 = motor_encoder_1;
            move->start_encoder_2 = motor_encoder_2;
            move->start_encoder_3 = motor_encoder_3;
            move->start_encoder_4 = motor_encoder_4;

            /* 复位 PID */
            pid_reset(&move->parallel_pid.branch1);
            pid_reset(&move->parallel_pid.branch2);
        }
        return;
    }

    /* ---- 阶段 2：直线行驶 ---- */
    if (move->state == MOVE_MOVING) {
        /* 1. 计算当前已行驶距离（厘米） */
        move->current_distance_cm = get_average_distance_cm(move);

        /* 2. 计算距离误差 */
        float distance_error_cm = move->target_distance_cm - move->current_distance_cm;
        float abs_distance_error = fabsf(distance_error_cm);

        /* 3. 计算角度误差（保持目标角度） */
        float angle_err = angle_error(move->target_angle, move->current_angle);

        /* 4. 并级 PID 计算
         *    回路1：距离控制 → 基础速度
         *    回路2：角度保持 → 差速修正量 */
        float base_speed = pid_calc(&move->parallel_pid.branch1,
                                     move->target_distance_cm,  /* 目标距离（厘米） */
                                     move->current_distance_cm, /* 当前距离（厘米） */
                                     dt);

        float angle_correction = pid_calc(&move->parallel_pid.branch2,
                                          0.0f,        /* 目标：角度误差为 0 */
                                          -angle_err,  /* 当前：负角度误差 */
                                          dt);

        /* 5. 限幅基础速度 */
        base_speed = clamp_symmetric(base_speed, move->max_speed);

        /* 6. 死区判断 */
        if (abs_distance_error < MOVE_DEADBAND_CM) {
            base_speed = 0.0f;
            angle_correction = 0.0f;
        }
        /* 静摩擦补偿 */
        else if (base_speed > 0.0f && base_speed < move->min_speed) {
            base_speed = move->min_speed;
        }
        else if (base_speed < 0.0f && base_speed > -move->min_speed) {
            base_speed = -move->min_speed;
        }

        /* 7. 计算左右电机速度（差速转向修正） */
        float left_speed = base_speed - angle_correction;
        float right_speed = base_speed + angle_correction;

        /* 限幅 */
        left_speed = clamp_symmetric(left_speed, move->max_speed);
        right_speed = clamp_symmetric(right_speed, move->max_speed);

        /* 8. 输出到电机 */
        int16_t left_pwm = (int16_t)left_speed;
        int16_t right_pwm = (int16_t)right_speed;

        motor_set(1, left_pwm);   /* 左后 */
        motor_set(2, left_pwm);   /* 左前 */
        motor_set(3, right_pwm);  /* 右前 */
        motor_set(4, right_pwm);  /* 右后 */

        /* 9. 完成判定 */
        if (abs_distance_error < move->distance_tolerance) {
            move->settle_count++;
            if (move->settle_count >= move->settle_threshold) {
                /* 运动完成：停止电机 */
                move->state = MOVE_DONE;
                motor_set(1, 0);
                motor_set(2, 0);
                motor_set(3, 0);
                motor_set(4, 0);

                /* 复位 PID */
                pid_reset(&move->parallel_pid.branch1);
                pid_reset(&move->parallel_pid.branch2);
            }
        } else {
            move->settle_count = 0;  /* 超出容差，重置计数 */
        }
    }
}

void Movement_Stop(MovementController *move)
{
    move->state = MOVE_IDLE;
    move->settle_count = 0;

    /* 停止旋转控制器 */
    if (move->rotation_ctrl != NULL) {
        Rotation_Stop(move->rotation_ctrl);
    }

    /* 复位 PID */
    pid_reset(&move->parallel_pid.branch1);
    pid_reset(&move->parallel_pid.branch2);

    /* 停止所有电机 */
    motor_set(1, 0);
    motor_set(2, 0);
    motor_set(3, 0);
    motor_set(4, 0);
}

void Movement_SetDistanceTuning(MovementController *move, float kp, float ki, float kd)
{
    pid_set_tuning(&move->parallel_pid.branch1, kp, ki, kd);
}

void Movement_SetAngleTuning(MovementController *move, float kp, float ki, float kd)
{
    pid_set_tuning(&move->parallel_pid.branch2, kp, ki, kd);
}

MovementState Movement_GetState(MovementController *move)
{
    return move->state;
}

float Movement_GetCurrentDistance(MovementController *move)
{
    return move->current_distance_cm;
}

bool Movement_IsDone(MovementController *move)
{
    return (move->state == MOVE_DONE);
}

/* ========================================================================
 *  使用说明
 * ========================================================================
 *
 * 【功能】
 *   小车以指定角度行驶指定距离，输入单位为厘米
 *   - 两阶段运动：先旋转到目标角度，再直线行驶
 *   - 并级 PID 控制：距离环 + 角度保持环
 *   - 非阻塞设计：在 125Hz 定时器中断中调用
 *
 * 【快速开始】
 *
 *   // 1. 在 main() 中初始化
 *   Encoder_Init();
 *   motor_init();
 *   Rotation_Init(&g_rotation);
 *   Movement_Init(&g_movement, &g_rotation);
 *   HAL_TIM_Base_Start_IT(&htim9);  // 启动 125Hz 定时器
 *
 *   // 2. 设置目标：90度方向前进50厘米
 *   Movement_SetTarget(&g_movement, 90.0f, 50.0f);
 *
 *   // 3. 在 TIM9 中断中更新（125Hz）
 *   void TIM9_IRQHandler(void) {
 *       if (__HAL_TIM_GET_FLAG(&htim9, TIM_FLAG_UPDATE)) {
 *           __HAL_TIM_CLEAR_FLAG(&htim9, TIM_FLAG_UPDATE);
 *           Encoder_Update_Counts();
 *           float yaw = Get_Gyro_Yaw();  // 获取陀螺仪偏航角 -180~180°
 *           Movement_Update(&g_movement, yaw, 0.008f);
 *       }
 *   }
 *
 *   // 4. 检查是否完成
 *   if (Movement_IsDone(&g_movement)) {
 *       // 运动完成，执行下一个任务
 *   }
 *
 * 【API 说明】
 *
 *   Movement_SetTarget(move, angle_deg, distance_cm)
 *     - angle_deg: 目标角度 0~360° (0°正前方, 90°右侧, 180°后方, 270°左侧)
 *     - distance_cm: 目标距离，厘米（正数前进，负数后退）
 *
 *   Movement_Update(move, yaw_deg, dt)
 *     - yaw_deg: 陀螺仪偏航角 -180~180°
 *     - dt: 时间间隔，秒（125Hz → 0.008s）
 *
 *   Movement_Stop(move)
 *     - 立即停止运动，复位所有状态
 *
 *   Movement_IsDone(move)
 *     - 返回 true 表示运动完成
 *
 *   Movement_GetCurrentDistance(move)
 *     - 返回当前已行驶距离（厘米）
 *
 *   Movement_GetState(move)
 *     - 返回当前状态：MOVE_IDLE / MOVE_ROTATING / MOVE_MOVING / MOVE_DONE
 *
 * 【在线调参】
 *
 *   // 调整距离环 PID（控制行驶距离）
 *   Movement_SetDistanceTuning(&g_movement, 800.0f, 5.0f, 150.0f);
 *
 *   // 调整角度保持环 PID（保持直线）
 *   Movement_SetAngleTuning(&g_movement, 60.0f, 1.0f, 15.0f);
 *
 * 【默认 PID 参数】
 *
 *   距离环：Kp=800, Ki=5, Kd=150, max_out=6000, max_i=2000
 *   角度保持环：Kp=60, Ki=1, Kd=15, max_out=3000, max_i=1000
 *
 * 【调参建议】
 *
 *   小车冲过头（超调）：
 *     - 降低距离环 Kp（如 600）
 *     - 增加距离环 Kd（如 200）
 *
 *   小车到不了目标距离：
 *     - 增加距离环 Kp（如 1000）
 *     - 增加距离环 Ki（如 10）
 *
 *   小车走不直（偏航）：
 *     - 增加角度保持环 Kp（如 80）
 *     - 检查陀螺仪是否正常
 *
 *   小车摆动严重：
 *     - 降低所有 Kp 值
 *     - 增加所有 Kd 值
 *
 * 【连续运动示例】
 *
 *   typedef enum {
 *       TASK_IDLE, TASK_MOVE_1, TASK_MOVE_2, TASK_DONE
 *   } TaskState;
 *   TaskState task = TASK_IDLE;
 *
 *   void Task_Update(void) {
 *       switch (task) {
 *           case TASK_IDLE:
 *               Movement_SetTarget(&g_movement, 0.0f, 30.0f);
 *               task = TASK_MOVE_1;
 *               break;
 *           case TASK_MOVE_1:
 *               if (Movement_IsDone(&g_movement)) {
 *                   HAL_Delay(500);
 *                   Movement_SetTarget(&g_movement, 90.0f, 40.0f);
 *                   task = TASK_MOVE_2;
 *               }
 *               break;
 *           case TASK_MOVE_2:
 *               if (Movement_IsDone(&g_movement)) {
 *                   task = TASK_DONE;
 *               }
 *               break;
 *       }
 *   }
 *
 * 【注意事项】
 *
 *   1. 陀螺仪必须正常工作，提供准确的偏航角
 *   2. 编码器方向必须正确设置（encoder.h 中的 MOTOR_X_DIR）
 *   3. 电机方向必须正确设置（motor_tb6612.c）
 *   4. 必须在 125Hz 定时器中断中调用 Movement_Update()
 *   5. 地面打滑会影响距离精度
 *
 * 【工作原理】
 *
 *   阶段1 - 旋转：
 *     调用 rotation_control 模块旋转到目标角度
 *
 *   阶段2 - 直线行驶：
 *     距离环：根据 4 个编码器平均值计算已行驶距离，PID 输出基础速度
 *     角度保持环：根据陀螺仪保持目标角度，PID 输出差速修正量
 *     左侧电机 = 基础速度 - 角度修正
 *     右侧电机 = 基础速度 + 角度修正
 *
 *   完成判定：
 *     距离误差 < 1cm 且稳定 200ms → 停止电机 → MOVE_DONE
 *
 * ======================================================================== */
