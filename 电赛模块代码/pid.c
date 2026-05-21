#include "pid.h"

// 初始化PID控制器
static float limit_max(float x, float max)
{
    if (x > max) {
        return max;
    }
    if (x < -max) {
        return -max;
    }
    return x;
}

void pid_init(PID* pid, float kp, float ki, float kd, const float max_out, const float max_error_integral)
{

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->max_out = max_out;
    pid->max_error_integral = max_error_integral;

    pid->target = 0.0f;
    pid->input = 0.0f;
    pid->output = 0.0f;

    pid->error.now = 0.0f;
    pid->error.last = 0.0f;
    pid->error.integral = 0.0f;
}

float pid_calc(PID* pid, float set, float fdb)
{


    // 更新设定值和反馈值
    pid->target = set;
    pid->input = fdb;

    // 计算误差
    pid->error.now = pid->target - pid->input;

    // 误差积分，与抗饱和限幅
    pid->error.integral += pid->error.now;
    pid->error.integral = limit_max(pid->error.integral, pid->max_error_integral);

    // 微分项 = 当前误差 - 上次误差
    float derivative = pid->error.now - pid->error.last;

    // PID 输出
    float out = pid->kp * pid->error.now
              + pid->ki * pid->error.integral
              + pid->kd * derivative;

    // 对称限幅输出
    pid->output = limit_max(out, pid->max_out);

    // 保存本次误差，用于下次微分计算
    pid->error.last = pid->error.now;

    return pid->output;
}

/**
 * @brief 串级PID结构体，常见的速度环（inner）+位置环（outer）
 */

/**
 * @brief 初始化串级PID
 */
void cascade_pid_init(CascadePID* cpid,
                      float kp_out, float ki_out, float kd_out,
                      float max_out_out, float max_i_out,
                      float kp_in, float ki_in, float kd_in,
                      float max_out_in, float max_i_in)
{
    pid_init(&cpid->outer, kp_out, ki_out, kd_out, max_out_out, max_i_out);
    pid_init(&cpid->inner, kp_in, ki_in, kd_in, max_out_in, max_i_in);
}

/**
 * @brief 串级PID计算函数
 * @param cpid      串级PID实例
 * @param pos_set   位置设定值
 * @param pos_fdb   位置反馈值
 * @param vel_fdb   速度反馈值
 * @return          内环输出控制量
 */
float cascade_pid_calc(CascadePID* cpid, float pos_set, float pos_fdb, float vel_fdb)
{
    // 外环：计算所需速度设定
    float vel_set = pid_calc(&cpid->outer, pos_set, pos_fdb);
    // 内环：根据速度设定和速度反馈计算控制量
    float control = pid_calc(&cpid->inner, vel_set, vel_fdb);
    return control;
}

/**
 * @brief 并级PID结构体，两个独立并行回路，共同输出相加
 */

/**
 * @brief 初始化并级PID
 */
void parallel_pid_init(ParallelPID* ppid,
                       float kp1, float ki1, float kd1, float max_out1, float max_i1,
                       float kp2, float ki2, float kd2, float max_out2, float max_i2)
{
    pid_init(&ppid->branch1, kp1, ki1, kd1, max_out1, max_i1);
    pid_init(&ppid->branch2, kp2, ki2, kd2, max_out2, max_i2);
}

/**
 * @brief 并级PID计算函数
 * @param ppid      并级PID实例
 * @param set1      回路1目标
 * @param fdb1      回路1反馈
 * @param set2      回路2目标
 * @param fdb2      回路2反馈
 * @return          两回路输出之和
 */
float parallel_pid_calc(ParallelPID* ppid,
                        float set1, float fdb1,
                        float set2, float fdb2)
{
    float out1 = pid_calc(&ppid->branch1, set1, fdb1);
    float out2 = pid_calc(&ppid->branch2, set2, fdb2);
    // 将两个输出相加，并对总输出限幅为两者限幅最大值之和
    float total_max = ppid->branch1.max_out + ppid->branch2.max_out;
    return limit_max(out1 + out2, total_max);
}


//使用方法：
//
// 	 PID vel_pid; 单环速度环
//   PID pos_pid; 单环位置环
//   CascadePID cpid; 串级PID
//   ParallelPID ppid; 并级PID
// cascade_pid_init(&cpid,  //串级PID初始化
//     9.8f, 0.0f, 1.5f,    // 外环KP超小，KI/KD=0
//    800.0f, 1000.0f,      // 只要够大，不限幅
//    22.5f, 0.0f, 5.0f,    // 内环先只测试KP
//    1000.0f, 100.0f); 
// 回路1：角度环KP=30,KI=1,KD=0.5, max_out=200, max_int=100
// 回路2：速度环KP=5,KI=0.2,KD=0.1, max_out=100, max_int=50
// parallel_pid_init(&ppid,30, 1, 0.5, 200.0f, 100.0f,5, 0.2, 0.1, 1000.0f, 50.0f); //并级PID初始化
// float pwm = cascade_pid_calc(&cpid, 目标值, 外环反馈值, 内环反馈值);  //串级PID计算
// float pwm = parallel_pid_calc(&ppid, 目标值1, 反馈值1, 目标值2, 反馈值2);  //并级PID计算
