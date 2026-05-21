#ifndef ENCODER_H
#define ENCODER_H

#include "main.h"
#include "tim.h" 

// ================= 配置区域 (请根据CubeMX配置修改) =================

// 假设你使用了TIM2, TIM3, TIM4, TIM5作为四个轮子的编码器接口
// 必须确保这些定时器在CubeMX中开启了 "Encoder Mode"
#define ENCODER_TIM_1   htim2  // 左前 FL
#define ENCODER_TIM_2   htim3  // 左后 FR
#define ENCODER_TIM_3   htim4  // 右后 RL
#define ENCODER_TIM_4   htim8  // 右前 RR

// 如果你的定时器是32位的(如STM32F4的TIM2/TIM5)，此逻辑依然通用且安全
// 如果你的电机方向和实际相反，可以在这里定义 -1 来软件反转
#define MOTOR_1_DIR     -1
#define MOTOR_2_DIR     -1     // 很多时候右侧电机安装是镜像的，需要反转
#define MOTOR_3_DIR     1
#define MOTOR_4_DIR     1

// ================= 全局变量声明 =================
// 这些变量记录的是总路程（脉冲数），可以直接被PID和control_car.c调用
extern int32_t motor_encoder_1;
extern int32_t motor_encoder_2;
extern int32_t motor_encoder_3;
extern int32_t motor_encoder_4;

// ================= 函数接口 =================

/**
 * @brief 初始化编码器定时器
 */
void Encoder_Init(void);

/**
 * @brief 读取并累加编码器数值
 * @note  必须在定时器中断(如TIM9, 20ms)中周期性调用
 */
void Encoder_Update_Counts(void);

/**
 * @brief 清零编码器数值 (可选，用于重置状态)
 */
void Encoder_Reset(void);

#endif
