/**
 * @file    pid_optimized.c
 * @brief   优化版 PID 控制器实现
 *
 * 核心改进点：
 *  - dt 时间因子：积分项 *= dt，微分项 /= dt，控制周期变化时参数无需重调
 *  - 微分项基于测量值（derivative-on-measurement）：设定值突变不会产生输出尖峰
 *  - 条件积分（conditional integration）：输出饱和时暂停积分，配合限幅双重抗饱和
 *  - pid_reset()：模式切换时清零状态，避免历史积分干扰
 *  - pid_set_tuning()：运行时在线调参，无需重新 init
 */

#include "pid_optimized.h"

/* ========================================================================
 *  内部工具函数
 * ======================================================================== */

/** 对称限幅：将 x 钳位在 [-limit, limit] */
static float clamp_symmetric(float x, float limit)
{
    if (x > limit)  return limit;
    if (x < -limit) return -limit;
    return x;
}

/* ========================================================================
 *  单级 PID
 * ======================================================================== */

void pid_init(PID *pid, float kp, float ki, float kd,
              float max_out, float max_i)
{
    pid->kp      = kp;
    pid->ki      = ki;
    pid->kd      = kd;
    pid->max_out = max_out;
    pid->max_i   = max_i;

    pid->setpoint         = 0.0f;
    pid->measurement      = 0.0f;
    pid->last_measurement = 0.0f;
    pid->output           = 0.0f;

    pid->error.now      = 0.0f;
    pid->error.last     = 0.0f;
    pid->error.integral = 0.0f;
}

void pid_set_tuning(PID *pid, float kp, float ki, float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

void pid_reset(PID *pid)
{
    pid->error.integral   = 0.0f;
    pid->error.last       = 0.0f;
    pid->last_measurement = 0.0f;
    pid->output           = 0.0f;
}

float pid_calc(PID *pid, float setpoint, float measurement, float dt)
{
    pid->setpoint    = setpoint;
    pid->measurement = measurement;

    /* ---- 1. 误差计算 ---- */
    pid->error.now = setpoint - measurement;

    /* ---- 2. 比例项 ---- */
    float p_term = pid->kp * pid->error.now;

    /* ---- 3. 积分项（带 dt 缩放） ---- */
    if (dt > 0.0f) {
        pid->error.integral += pid->error.now * dt;
    } else {
        pid->error.integral += pid->error.now;
    }
    pid->error.integral = clamp_symmetric(pid->error.integral, pid->max_i);

    float i_term = pid->ki * pid->error.integral;

    /* ---- 4. 微分项（基于测量值，避免微分 kick） ----
     *    d(error)/dt = d(setpoint - measurement)/dt
     *    setpoint 通常不变 → d(error)/dt ≈ -d(measurement)/dt
     *    所以实际使用：-kd * (measurement - last_measurement) / dt
     */
    float d_term = 0.0f;
    if (dt > 0.0f) {
        float deriv = (measurement - pid->last_measurement) / dt;
        d_term = -pid->kd * deriv;
    } else {
        d_term = pid->kd * (pid->error.now - pid->error.last);
    }

    pid->last_measurement = measurement;
    pid->error.last       = pid->error.now;

    /* ---- 5. 输出合成 + 条件积分判断 ---- */
    float out = p_term + i_term + d_term;

    /* 判断是否饱和：如果即将饱和且误差同向，回退本次积分增量，防止继续向饱和方向累积 */
    if ((out > pid->max_out && pid->error.now > 0.0f) ||
        (out < -pid->max_out && pid->error.now < 0.0f)) {
        if (dt > 0.0f) {
            pid->error.integral -= pid->error.now * dt;
        } else {
            pid->error.integral -= pid->error.now;
        }
    }

    pid->output = clamp_symmetric(out, pid->max_out);
    return pid->output;
}

/* ========================================================================
 *  串级 PID
 * ======================================================================== */

void cascade_pid_init(CascadePID *cpid,
                      float kp_out, float ki_out, float kd_out,
                      float max_out_out, float max_i_out,
                      float kp_in, float ki_in, float kd_in,
                      float max_out_in, float max_i_in)
{
    pid_init(&cpid->outer, kp_out, ki_out, kd_out, max_out_out, max_i_out);
    pid_init(&cpid->inner, kp_in, ki_in, kd_in, max_out_in, max_i_in);
}

float cascade_pid_calc(CascadePID *cpid,
                       float pos_set, float pos_fdb,
                       float vel_fdb, float dt)
{
    /* 外环：位置 → 速度设定 */
    float vel_set = pid_calc(&cpid->outer, pos_set, pos_fdb, dt);

    /* 内环：速度设定 → 控制量 */
    return pid_calc(&cpid->inner, vel_set, vel_fdb, dt);
}

/* ========================================================================
 *  并级 PID
 * ======================================================================== */

void parallel_pid_init(ParallelPID *ppid,
                       float kp1, float ki1, float kd1, float max_out1, float max_i1,
                       float kp2, float ki2, float kd2, float max_out2, float max_i2)
{
    pid_init(&ppid->branch1, kp1, ki1, kd1, max_out1, max_i1);
    pid_init(&ppid->branch2, kp2, ki2, kd2, max_out2, max_i2);
}

float parallel_pid_calc(ParallelPID *ppid,
                        float set1, float fdb1,
                        float set2, float fdb2, float dt)
{
    float out1 = pid_calc(&ppid->branch1, set1, fdb1, dt);
    float out2 = pid_calc(&ppid->branch2, set2, fdb2, dt);

    float total_max = ppid->branch1.max_out + ppid->branch2.max_out;
    return clamp_symmetric(out1 + out2, total_max);
}


/* ========================================================================
 *  使用示例（注释）
 * ========================================================================
 *
 * // ---- 单环速度控制 ----
 * PID vel_pid;
 * pid_init(&vel_pid, 22.5f, 0.0f, 5.0f, 1000.0f, 100.0f);
 *
 * // 在定时器中调用（假设 1kHz → dt = 0.001f）
 * float pwm = pid_calc(&vel_pid, 目标速度, 编码器速度, 0.001f);
 *
 *
 * // ---- 串级 PID（位置外环 + 速度内环） ----
 * CascadePID cpid;
 * cascade_pid_init(&cpid,
 *     9.8f, 0.0f, 1.5f,    // 外环: Kp, Ki, Kd
 *     800.0f, 1000.0f,      // 外环: max_out, max_i
 *     22.5f, 0.0f, 5.0f,    // 内环: Kp, Ki, Kd
 *     1000.0f, 100.0f);     // 内环: max_out, max_i
 *
 * float pwm = cascade_pid_calc(&cpid, 目标位置, 编码器位置, 编码器速度, 0.001f);
 *
 *
 * // ---- 并级 PID（角度环 + 速度环并行） ----
 * ParallelPID ppid;
 * parallel_pid_init(&ppid,
 *     30.0f, 1.0f, 0.5f, 200.0f, 100.0f,   // 回路1: 角度环
 *     5.0f,  0.2f, 0.1f, 100.0f, 50.0f);   // 回路2: 速度环
 *
 * float pwm = parallel_pid_calc(&ppid,
 *     目标角度, 陀螺仪角度,   // 回路1
 *     目标速度, 编码器速度,   // 回路2
 *     0.001f);
 *
 *
 * // ---- 在线调参 ----
 * pid_set_tuning(&vel_pid, 18.0f, 0.5f, 3.0f);  // 运行时修改 Kp/Ki/Kd
 *
 * // ---- 模式切换时复位 ----
 * pid_reset(&vel_pid);  // 切换控制模式前清零积分和历史
 */
