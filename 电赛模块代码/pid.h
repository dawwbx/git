#ifndef PID_HPP
#define PID_HPP

#include <stdint.h>

typedef struct 
{
  float now;
  float last;
  float integral;
}Error;

typedef struct 
{
  float kp, ki, kd;     
  float target,input,output;
  Error error;
  float max_out, max_error_integral;
}PID;

typedef struct {
    PID outer;    // 外环（位置）
    PID inner;    // 内环（速度）
} CascadePID;


typedef struct {
    PID branch1;  // 回路1
    PID branch2;  // 回路2
}ParallelPID;

void pid_init(PID* pid, float kp, float ki, float kd, const float max_out, const float max_error_integral);
float pid_calc(PID* pid, float set, float fdb);
void cascade_pid_init(CascadePID* cpid,
                      float kp_out, float ki_out, float kd_out,
                      float max_out_out, float max_i_out,
                      float kp_in, float ki_in, float kd_in,
                      float max_out_in, float max_i_in);
float cascade_pid_calc(CascadePID* cpid, float pos_set, float pos_fdb, float vel_fdb);

void parallel_pid_init(ParallelPID* ppid,
                       float kp1, float ki1, float kd1, float max_out1, float max_i1,
                       float kp2, float ki2, float kd2, float max_out2, float max_i2);

float parallel_pid_calc(ParallelPID* ppid,
                        float set1, float fdb1,
                        float set2, float fdb2);
#endif  // PID_HPP
