#include "servo.h"

// 角度到CCR值的转换（0-180度 → 500-2500）
static uint16_t angle_to_ccr(uint16_t angle) {
    if(angle > 180) angle = 180;
    return 500 + (angle * 2000) / 180;  // 计算CCR值
}

// 初始化舵机控制器
void Servo_Init(Servo_Controller *servo, TIM_HandleTypeDef *htim, uint32_t channel, uint16_t init_angle) {
    servo->htim = htim;
    servo->channel = channel;
    servo->current_angle = init_angle;
    
    // 启动PWM并设置初始位置
    HAL_TIM_PWM_Start(servo->htim, servo->channel);
    __HAL_TIM_SET_COMPARE(servo->htim, servo->channel, angle_to_ccr(init_angle));
}

// 控制舵机运动（阻塞式）
void Servo_Move(Servo_Controller *servo, uint16_t target_angle, uint16_t hold_time, uint8_t smooth_step) {
    uint16_t start_ccr = angle_to_ccr(servo->current_angle);
    uint16_t target_ccr = angle_to_ccr(target_angle);
    
    // 平滑移动（smooth_step=0为立即跳变）
    if(smooth_step > 0) {
        uint16_t current_ccr = start_ccr;
        while(current_ccr != target_ccr) {
            if(current_ccr < target_ccr) {
                current_ccr = (current_ccr + smooth_step > target_ccr) ? target_ccr : current_ccr + smooth_step;
            } else {
                current_ccr = (current_ccr - smooth_step < target_ccr) ? target_ccr : current_ccr - smooth_step;
            }
            __HAL_TIM_SET_COMPARE(servo->htim, servo->channel, current_ccr);
            HAL_Delay(10);  // 每步10ms
        }
    } else {
        __HAL_TIM_SET_COMPARE(servo->htim, servo->channel, target_ccr);
    }
    
    // 更新当前角度并保持时间
    servo->current_angle = target_angle;
    HAL_Delay(hold_time);
}

//用法：
// Servo_Controller my_servo;//申明实例
//Servo_Init(&my_servo, &htim1, TIM_CHANNEL_2, 0);初始化（定时器，通道，初始角度）
// Servo_Move(&my_servo, 85, 1000, 30);  // 平滑移动到130度
// Servo_Move(&my_servo, 0, 0, 0);        // 立即跳回0度
