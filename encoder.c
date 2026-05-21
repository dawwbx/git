#include "encoder.h"

// ================= 全局变量定义 =================
// 记录从上电（或Reset）开始累计的总脉冲数，int32范围足够大，跑几天都不会溢出
int32_t motor_encoder_1 = 0;
int32_t motor_encoder_2 = 0;
int32_t motor_encoder_3 = 0;
int32_t motor_encoder_4 = 0;

// ================= 内部静态变量 =================
// 用于记录上一次读取的定时器原始值(0-65535)
static uint16_t last_cnt_1 = 0;
static uint16_t last_cnt_2 = 0;
static uint16_t last_cnt_3 = 0;
static uint16_t last_cnt_4 = 0;

// ================= 函数实现 =================

void Encoder_Init(void)
{
    // 开启编码器模式的定时器计数
    HAL_TIM_Encoder_Start(&ENCODER_TIM_1, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&ENCODER_TIM_2, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&ENCODER_TIM_3, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&ENCODER_TIM_4, TIM_CHANNEL_ALL);

    // 初始化上一次的值，防止刚启动时的瞬间突变
    last_cnt_1 = __HAL_TIM_GET_COUNTER(&ENCODER_TIM_1);
    last_cnt_2 = __HAL_TIM_GET_COUNTER(&ENCODER_TIM_2);
    last_cnt_3 = __HAL_TIM_GET_COUNTER(&ENCODER_TIM_3);
    last_cnt_4 = __HAL_TIM_GET_COUNTER(&ENCODER_TIM_4);
    
    motor_encoder_1 = 0;
    motor_encoder_2 = 0;
    motor_encoder_3 = 0;
    motor_encoder_4 = 0;
}

// 内部辅助函数：计算差值并处理溢出
static int16_t Calculate_Diff(uint16_t current, uint16_t last) {
    // 核心逻辑：利用 int16_t 的溢出特性
    // 例如：当前 10，上次 65530。
    // (uint16_t)(10 - 65530) = 16 (二进制 0x0010)
    // 强制转换为 int16_t 后就是 16。
    // 例如：当前 65530，上次 10。
    // (uint16_t)(65530 - 10) = 65520 (二进制 0xFFF0)
    // 强制转换为 int16_t 后就是 -16。
    return (int16_t)(current - last);
}

void Encoder_Update_Counts(void)
{
    // --- 电机 1 ---
    uint16_t curr_1 = __HAL_TIM_GET_COUNTER(&ENCODER_TIM_1);
    motor_encoder_1 += Calculate_Diff(curr_1, last_cnt_1) * MOTOR_1_DIR;
    last_cnt_1 = curr_1;

    // --- 电机 2 ---
    uint16_t curr_2 = __HAL_TIM_GET_COUNTER(&ENCODER_TIM_2);
    motor_encoder_2 += Calculate_Diff(curr_2, last_cnt_2) * MOTOR_2_DIR;
    last_cnt_2 = curr_2;

    // --- 电机 3 ---
    uint16_t curr_3 = __HAL_TIM_GET_COUNTER(&ENCODER_TIM_3);
    motor_encoder_3 += Calculate_Diff(curr_3, last_cnt_3) * MOTOR_3_DIR;
    last_cnt_3 = curr_3;

    // --- 电机 4 ---
    uint16_t curr_4 = __HAL_TIM_GET_COUNTER(&ENCODER_TIM_4);
    motor_encoder_4 += Calculate_Diff(curr_4, last_cnt_4) * MOTOR_4_DIR;
    last_cnt_4 = curr_4;
}

void Encoder_Reset(void)
{
    motor_encoder_1 = 0;
    motor_encoder_2 = 0;
    motor_encoder_3 = 0;
    motor_encoder_4 = 0;
    
    // 同时更新last值，避免下次Update时产生巨大跳变
    last_cnt_1 = __HAL_TIM_GET_COUNTER(&ENCODER_TIM_1);
    last_cnt_2 = __HAL_TIM_GET_COUNTER(&ENCODER_TIM_2);
    last_cnt_3 = __HAL_TIM_GET_COUNTER(&ENCODER_TIM_3);
    last_cnt_4 = __HAL_TIM_GET_COUNTER(&ENCODER_TIM_4);
}
