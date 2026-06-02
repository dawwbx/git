#ifndef __SERVO_H
#define __SERVO_H

#include "stm32f4xx_hal.h"

// 舵机控制结构体
typedef struct {
    TIM_HandleTypeDef *htim;  // 定时器句柄
    uint32_t channel;         // PWM通道
    uint16_t current_angle;   // 当前角度
} Servo_Controller;

// 函数原型
void Servo_Init(Servo_Controller *servo, TIM_HandleTypeDef *htim, uint32_t channel, uint16_t init_angle);
void Servo_Move(Servo_Controller *servo, uint16_t target_angle, uint16_t hold_time, uint8_t smooth_step);

#endif
