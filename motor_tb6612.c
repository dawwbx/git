#include "motor_tb6612.h"
#include "main.h"


void motor_init(void)
{
    // 启动电机1的PWM输出
    HAL_TIM_PWM_Start(&MOTOR1_TIM,MOTOR1);
    // 启动电机2的PWM输出
    HAL_TIM_PWM_Start(&MOTOR2_TIM,MOTOR2);
	   // 启动电机3的PWM输出
    HAL_TIM_PWM_Start(&MOTOR3_TIM,MOTOR3);
    // 启动电机4的PWM输出
    HAL_TIM_PWM_Start(&MOTOR4_TIM,MOTOR4);
    // 设置电机1的速度为0
    motor_set(1,0);
    // 设置电机2的速度为0
    motor_set(2,0);
		// 设置电机3的速度为0
    motor_set(3,0);
    // 设置电机4的速度为0
    motor_set(4,0);
}

void motor_set(uint8_t motor, int16_t speed) {
    // 初始化电机转动方向，0表示停止，1表示正转，-1表示反转
    int8_t direction = 0;
    // 判断速度为正，电机正转
    if (speed > 0) {
        direction = 1;
        // 确保速度在允许的最大值范围内
        if (speed > MOTOR_MAX) speed = MOTOR_MAX;
        // 确保速度在允许的最小值范围内
        else if (speed < MOTOR_MIN) speed = MOTOR_MIN;
    // 判断速度为负，电机反转
    } else if (speed < 0) {
        direction = -1;
        // 将速度转换为正数，方便后续PWM占空比设置
        speed = -speed;
        // 确保速度在允许的最大值范围内
        if (speed > MOTOR_MAX) speed = MOTOR_MAX;
        // 确保速度在允许的最小值范围内
        else if (speed < MOTOR_MIN) speed = MOTOR_MIN;
    }

    // 根据电机编号选择对应的电机控制逻辑
    switch (motor) {
        case 1:
            // 电机1正转
            if (direction == 1) {
                MOTOR1_R(); 
            // 电机1反转
            } else if (direction == -1) {
                MOTOR1_L(); 
            }
            // 设置电机1的PWM占空比
            __HAL_TIM_SetCompare(&MOTOR1_TIM, MOTOR1, speed);
            break;
        case 2:
            // 电机2正转
            if (direction == -1) {
                MOTOR2_R(); 
            // 电机2反转
            } else if (direction == 1) {
                MOTOR2_L(); 
            }
            // 设置电机2的PWM占空比
            __HAL_TIM_SetCompare(&MOTOR2_TIM, MOTOR2, speed);
            break;
				case 3:
            // 电机3正转
            if (direction == -1) {
                MOTOR3_R(); 
            // 电机3反转
            } else if (direction == 1) {
                MOTOR3_L(); 
            }
            // 设置电机3的PWM占空比
            __HAL_TIM_SetCompare(&MOTOR3_TIM, MOTOR3, speed);
            break;
				case 4:
            // 电机4正转
            if (direction == 1) {
                MOTOR4_R(); 
            // 电机4反转
            } else if (direction == -1) {
                MOTOR4_L(); 
            }
            // 设置电机4的PWM占空比
            __HAL_TIM_SetCompare(&MOTOR4_TIM, MOTOR4, speed);
            break;
        default:
            break;
    }
}


//使用方法
//motor_init(void) //初始化 （TIM和CHANNEL在motor.h中定义）
//motor_set(1,50) //设置电机1的速度为50

