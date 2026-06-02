#ifndef ENCODER_H
#define ENCODER_H

#include "main.h"
#include "tim.h" 


#define ENCODER_TIM_1   htim2  
#define ENCODER_TIM_2   htim3  
#define ENCODER_TIM_3   htim4  
#define ENCODER_TIM_4   htim8  


#define MOTOR_1_DIR     -1
#define MOTOR_2_DIR     -1     
#define MOTOR_3_DIR     1
#define MOTOR_4_DIR     1

// ================= 物理参数定义 =================
#define WHEEL_TICKS_PER_REV  68000.0f    // 500线 * 4倍频 * 34减速比
#define WHEEL_CIRCUMFERENCE_CM  20.42035f  // 轮胎周长：PI * 0.065 (单位：米)


extern int32_t motor_encoder_1;
extern int32_t motor_encoder_2;
extern int32_t motor_encoder_3;
extern int32_t motor_encoder_4;


void Encoder_Init(void);


void Encoder_Update_Counts(void);


void Encoder_Reset(void);

float Get_Motor_Distance(int32_t encoder_count);
float Get_Motor_Velocity(int16_t tick_delta, float dt);
#endif
