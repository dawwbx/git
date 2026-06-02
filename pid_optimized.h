/**
 * @file    pid_optimized.h
 * @brief   优化版 PID 控制器 —— 单级 / 串级 / 并级
 *
 * 相比原始版本的改进：
 *  1. 加入 dt（时间间隔）支持，积分/微分项不再依赖固定调用周期
 *  2. 微分项基于"测量值"计算（derivative-on-measurement），避免设定值突变时的微分 kick
 *  3. 条件积分抗饱和：输出饱和时暂停积分累积，防止 windup
 *  4. 更清晰的命名与结构，符合嵌入式 C 规范
 */

#ifndef PID_OPTIMIZED_H
#define PID_OPTIMIZED_H

#ifdef __cplusplus
extern "C" {
#endif

/*-------------------------------------------------------------------------*/
/*  单级 PID                                                               */
/*-------------------------------------------------------------------------*/

typedef struct {
    float now;       /* 当前误差 */
    float last;      /* 上一次误差 */
    float integral;  /* 误差积分 */
} PidError;

typedef struct {
    /* --- 可调参数 --- */
    float kp, ki, kd;           /* PID 系数 */
    float max_out;              /* 输出限幅 (对称: -max_out ~ +max_out) */
    float max_i;                /* 积分限幅 */

    /* --- 状态变量 --- */
    float setpoint;             /* 设定值 */
    float measurement;          /* 测量值（反馈） */
    float last_measurement;     /* 上一次测量值（用于微分项） */
    float output;               /* 当前输出 */
    PidError error;             /* 误差状态 */
} PID;

/* 初始化：设置系数 + 限幅 */
void pid_init(PID *pid, float kp, float ki, float kd,
              float max_out, float max_i);

/* 运行时修改系数（方便在线调参） */
void pid_set_tuning(PID *pid, float kp, float ki, float kd);

/* 复位积分与历史状态（模式切换/启动时调用） */
void pid_reset(PID *pid);

/**
 * @brief 单次 PID 计算
 * @param pid         PID 实例
 * @param setpoint    设定值（目标）
 * @param measurement 测量值（反馈）
 * @param dt          距上次调用的时间间隔（秒），传 0 则退化为无 dt 的原始行为
 * @return            计算后的控制输出
 */
float pid_calc(PID *pid, float setpoint, float measurement, float dt);

/*-------------------------------------------------------------------------*/
/*  串级 PID（外环位置 + 内环速度）                                        */
/*-------------------------------------------------------------------------*/

typedef struct {
    PID outer;  /* 外环 — 位置 */
    PID inner;  /* 内环 — 速度 */
} CascadePID;

void cascade_pid_init(CascadePID *cpid,
                      float kp_out, float ki_out, float kd_out,
                      float max_out_out, float max_i_out,
                      float kp_in, float ki_in, float kd_in,
                      float max_out_in, float max_i_in);

/**
 * @brief 串级 PID 计算
 * @param cpid     串级 PID 实例
 * @param pos_set  位置设定值
 * @param pos_fdb  位置反馈值
 * @param vel_fdb  速度反馈值
 * @param dt       时间间隔（秒）
 * @return         内环输出控制量
 */
float cascade_pid_calc(CascadePID *cpid,
                       float pos_set, float pos_fdb, float vel_fdb, float dt);

/*-------------------------------------------------------------------------*/
/*  并级 PID（两回路独立控制，输出叠加）                                    */
/*-------------------------------------------------------------------------*/

typedef struct {
    PID branch1;
    PID branch2;
} ParallelPID;

void parallel_pid_init(ParallelPID *ppid,
                       float kp1, float ki1, float kd1, float max_out1, float max_i1,
                       float kp2, float ki2, float kd2, float max_out2, float max_i2);

/**
 * @brief 并级 PID 计算
 * @return 两回路输出之和
 */
float parallel_pid_calc(ParallelPID *ppid,
                        float set1, float fdb1,
                        float set2, float fdb2, float dt);

#ifdef __cplusplus
}
#endif

#endif /* PID_OPTIMIZED_H */
