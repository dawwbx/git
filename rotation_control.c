/**
 * @file    rotation_control.c
 * @brief   小车 PID 旋转控制模块实现
 *
 * 核心逻辑：
 *  1. 将陀螺仪 yaw 从 [-180,180] 转换为 [0,360]
 *  2. 计算最短路径角度误差（处理 360° 环绕）
 *  3. PID 计算旋转速度
 *  4. 差速转向输出到 4 个电机
 *  5. 稳定判定 → ROT_DONE → 电机停止
 */

#include "rotation_control.h"
#include "motor_tb6612.h"

/* ============================================================
 *  方向极性开关  ⚠️ 调试关键！
 *  ------------------------------------------------------------
 *  如果烧录后小车一直朝一个方向转停不下来 → 把这里改成 -1 再烧
 *  原理：PID 推电机后，yaw 朝反方向变化 → 误差越拉越大 → 转个不停
 *  这是「反馈极性反了」，把符号翻一下就好。
 * ============================================================ */
#define ROT_DIR_SIGN  (-1)        /* 改 +1 或 -1 */

/* 死区：误差在 ±DEADBAND_DEG 内不再驱动电机，消除小幅来回摆动 */
#define ROT_DEADBAND_DEG  (0.8f)

/* 启动最小驱动力：克服电机静摩擦力，PWM 太小电机不会动
 * 如果输出非零但 |output| < MIN_DRIVE 就提升到 MIN_DRIVE
 * 2400 ≈ 24% 占空比，提高终点段力矩 */
#define ROT_MIN_DRIVE     (2400.0f)

/* ---- 全局实例 ---- */
RotationController g_rotation;

/* ---- 内部工具：对称限幅 ---- */
static float clamp_symmetric(float x, float limit)
{
    if (x > limit)  return limit;
    if (x < -limit) return -limit;
    return x;
}

/* ---- 内部工具：yaw 从 [-180,180] 转换为 [0,360] ---- */
static float yaw_to_360(float yaw_deg)
{
    if (yaw_deg < 0.0f) {
        return yaw_deg + 360.0f;
    }
    return yaw_deg;
}

/* ---- 内部工具：计算最短路径角度误差 [-180, 180] ---- */
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

/* ================================================================
 *  API 实现
 * ================================================================ */

void Rotation_Init(RotationController *rot)
{
    /* 默认 PID 参数 —— 已调到中高速度，靠近目标力矩充足
     * 注意：kp 越大反应越快但越容易摆动；kd 提供阻尼 */
    pid_init(&rot->pid,
             80.0f,      /* kp:  误差 10° → 800 PWM, 90° → 7200 PWM */
             2.0f,       /* ki:  消除稳态漂移，不能太大否则会过冲 */
             28.0f,      /* kd:  阻尼，与 kp 配比 1:2 ~ 1:3 较稳 */
             8500.0f,    /* max_out: PID 内部最大输出 */
             2500.0f);    /* max_i:   积分限幅，防止饱和卷绕 */

    rot->target_angle      = 0.0f;
    rot->current_angle     = 0.0f;
    rot->output            = 0.0f;
    rot->state             = ROT_IDLE;
    rot->settle_count      = 0;
    rot->settle_threshold  = 30;     /* 240ms @ 125Hz 内稳定即完成 */
    rot->angle_tolerance   = 2.5f;   /* ±2.5° 内认为到达 */
    rot->max_rotation_speed = 8500.0f;  /* 实际输出限幅，与 max_out 一致 */
}

void Rotation_SetTarget(RotationController *rot, float target_deg)
{
    /* 规范化目标角度到 [0, 360) */
    while (target_deg >= 360.0f) target_deg -= 360.0f;
    while (target_deg < 0.0f)   target_deg += 360.0f;

    rot->target_angle = target_deg;
    rot->settle_count = 0;
    pid_reset(&rot->pid);         /* 清除上一次旋转的积分和历史 */
    rot->state = ROT_ACTIVE;
}

void Rotation_Update(RotationController *rot, float yaw_deg, float dt)
{
    /* 空闲状态：不干预电机 */
    if (rot->state == ROT_IDLE || rot->state == ROT_DONE) {
        return;
    }

    /* 1. 转换 yaw 到 0~360 范围 */
    rot->current_angle = yaw_to_360(yaw_deg);

    /* 2. 计算最短路径角度误差 */
    float error = angle_error(rot->target_angle, rot->current_angle);
    float abs_error = (error < 0.0f) ? -error : error;

    /* 3. PID 计算旋转速度
     *    技巧: setpoint=0, measurement=-error → PID 内部 error = 实际角度误差 */
    rot->output = pid_calc(&rot->pid, 0.0f, -error, dt);

    /* 4. 输出限幅 + 死区 + 静摩擦补偿 + 方向极性 */
    float out = clamp_symmetric(rot->output, rot->max_rotation_speed);

    /* 死区：误差很小时直接断电，避免微抖 */
    if (abs_error < ROT_DEADBAND_DEG) {
        out = 0.0f;
    }
    /* 静摩擦补偿：非零输出但太小，提升到最小驱动力 */
    else if (out > 0.0f && out < ROT_MIN_DRIVE) {
        out = ROT_MIN_DRIVE;
    }
    else if (out < 0.0f && out > -ROT_MIN_DRIVE) {
        out = -ROT_MIN_DRIVE;
    }

    /* 方向极性开关（顶部 ROT_DIR_SIGN 控制） */
    int16_t speed = (int16_t)(out * (float)ROT_DIR_SIGN);

    motor_set(1,  speed);   /* 左后 */
    motor_set(2,  speed);   /* 左前 */
    motor_set(3, -speed);   /* 右前 */
    motor_set(4, -speed);   /* 右后 */

    /* 5. 稳定判定 */
    if (abs_error < rot->angle_tolerance) {
        rot->settle_count++;
        if (rot->settle_count >= rot->settle_threshold) {
            /* 旋转完成：停止电机，复位 PID */
            rot->state = ROT_DONE;
            motor_set(1, 0);
            motor_set(2, 0);
            motor_set(3, 0);
            motor_set(4, 0);
            pid_reset(&rot->pid);
        }
    } else {
        rot->settle_count = 0;  /* 超出容差，重置计数 */
    }
}

void Rotation_Stop(RotationController *rot)
{
    rot->state = ROT_IDLE;
    rot->settle_count = 0;
    pid_reset(&rot->pid);

    /* 停止所有电机 */
    motor_set(1, 0);
    motor_set(2, 0);
    motor_set(3, 0);
    motor_set(4, 0);
}

void Rotation_SetTuning(RotationController *rot, float kp, float ki, float kd)
{
    pid_set_tuning(&rot->pid, kp, ki, kd);
}

RotationState Rotation_GetState(RotationController *rot)
{
    return rot->state;
}

float Rotation_GetOutput(RotationController *rot)
{
    return rot->output;
}
