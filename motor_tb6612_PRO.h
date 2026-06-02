#ifndef MOTOR_TB6612_PRO_H
#define MOTOR_TB6612_PRO_H

#include "main.h"
#include "tim.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 使用说明：
 * 1. 按实际项目修改下方 MOTOR_PRO_COUNT 与 MOTOR_PRO_CONFIG_TABLE
 * 2. 支持 2/3/4 路 TB6612 电机，无需修改 .c 源文件
 * 3. speed > 0 为正转，speed < 0 为反转，speed = 0 为停止
 */

#define MOTOR_PRO_MAX_COUNT 4U
#define MOTOR_PRO_MAX_SPEED 7200U
#define MOTOR_PRO_MIN_SPEED 0U

typedef enum
{
    MOTOR_PRO_STOP_MODE_COAST = 0,
    MOTOR_PRO_STOP_MODE_BRAKE = 1
} motor_pro_stop_mode_t;

typedef struct
{
    TIM_HandleTypeDef *htim;      /* PWM 定时器 */
    uint32_t channel;             /* PWM 通道 */
    GPIO_TypeDef *in1_port;       /* TB6612 IN1 端口 */
    uint16_t in1_pin;             /* TB6612 IN1 引脚 */
    GPIO_TypeDef *in2_port;       /* TB6612 IN2 端口 */
    uint16_t in2_pin;             /* TB6612 IN2 引脚 */
    uint8_t invert;               /* 0: 正常方向, 1: 反向安装时翻转方向 */
} motor_pro_cfg_t;

/* 电机数量：按需改成 2 / 3 / 4 */
#define MOTOR_PRO_COUNT 4U

#if (MOTOR_PRO_COUNT < 2U) || (MOTOR_PRO_COUNT > MOTOR_PRO_MAX_COUNT)
#error "MOTOR_PRO_COUNT must be 2, 3 or 4"
#endif

/*
 * 电机配置表：
 * 格式：{ &htimx, TIM_CHANNEL_x, GPIOx, GPIO_PIN_x, GPIOx, GPIO_PIN_x, invert }
 */
#define MOTOR_PRO_CONFIG_TABLE { \
    { &htim1, TIM_CHANNEL_1, GPIOA, GPIO_PIN_4, GPIOA, GPIO_PIN_5, 0 }, \
    { &htim1, TIM_CHANNEL_2, GPIOB, GPIO_PIN_4, GPIOB, GPIO_PIN_5, 0 }, \
    { &htim1, TIM_CHANNEL_3, GPIOB, GPIO_PIN_6, GPIOB, GPIO_PIN_7, 0 }, \
    { &htim1, TIM_CHANNEL_4, GPIOC, GPIO_PIN_8, GPIOC, GPIO_PIN_9, 0 }  \
}

void motor_pro_init(void);
void motor_pro_set(uint8_t motor, int16_t speed);
void motor_pro_set_all(const int16_t *speeds, uint8_t count);
void motor_pro_set_stop_mode(motor_pro_stop_mode_t mode);
void motor_pro_stop(uint8_t motor);
void motor_pro_stop_all(void);
uint8_t motor_pro_count(void);

#ifdef __cplusplus
}
#endif

#endif
