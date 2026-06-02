#ifndef MOTOR_H
#define MOTOR_H

#include "main.h"
#include "tim.h"

#define MOTOR1_TIM htim1
#define MOTOR2_TIM htim1
#define MOTOR3_TIM htim1
#define MOTOR4_TIM htim1

#define PIN_PORT_1    GPIOA
#define mt1_1     GPIO_PIN_4
#define mt1_2     GPIO_PIN_5
#define PIN_PORT_23    GPIOB
#define mt2_1     GPIO_PIN_4
#define mt2_2     GPIO_PIN_5
#define mt3_1     GPIO_PIN_6
#define mt3_2     GPIO_PIN_7
#define PIN_PORT_4    GPIOC
#define mt4_1     GPIO_PIN_8
#define mt4_2     GPIO_PIN_9

#define MOTOR1 TIM_CHANNEL_1
#define MOTOR2 TIM_CHANNEL_2
#define MOTOR3 TIM_CHANNEL_3
#define MOTOR4 TIM_CHANNEL_4

#define MOTOR_MAX 20000
#define MOTOR_MIN 0

#define MOTOR1_R() do { \
    HAL_GPIO_WritePin(PIN_PORT_1, mt1_1, GPIO_PIN_SET); \
    HAL_GPIO_WritePin(PIN_PORT_1, mt1_2, GPIO_PIN_RESET); \
} while(0)

#define MOTOR1_L() do { \
    HAL_GPIO_WritePin(PIN_PORT_1, mt1_2, GPIO_PIN_SET); \
    HAL_GPIO_WritePin(PIN_PORT_1, mt1_1, GPIO_PIN_RESET); \
} while(0)

#define MOTOR2_R() do { \
    HAL_GPIO_WritePin(PIN_PORT_23, mt2_1, GPIO_PIN_SET); \
    HAL_GPIO_WritePin(PIN_PORT_23, mt2_2, GPIO_PIN_RESET); \
} while(0)

#define MOTOR2_L() do { \
    HAL_GPIO_WritePin(PIN_PORT_23, mt2_2, GPIO_PIN_SET); \
    HAL_GPIO_WritePin(PIN_PORT_23, mt2_1, GPIO_PIN_RESET); \
} while(0)

#define MOTOR3_R() do { \
    HAL_GPIO_WritePin(PIN_PORT_23, mt3_1, GPIO_PIN_SET); \
    HAL_GPIO_WritePin(PIN_PORT_23, mt3_2, GPIO_PIN_RESET); \
} while(0)

#define MOTOR3_L() do { \
    HAL_GPIO_WritePin(PIN_PORT_23, mt3_2, GPIO_PIN_SET); \
    HAL_GPIO_WritePin(PIN_PORT_23, mt3_1, GPIO_PIN_RESET); \
} while(0)

#define MOTOR4_R() do { \
    HAL_GPIO_WritePin(PIN_PORT_4, mt4_1, GPIO_PIN_SET); \
    HAL_GPIO_WritePin(PIN_PORT_4, mt4_2, GPIO_PIN_RESET); \
} while(0)

#define MOTOR4_L() do { \
    HAL_GPIO_WritePin(PIN_PORT_4, mt4_2, GPIO_PIN_SET); \
    HAL_GPIO_WritePin(PIN_PORT_4, mt4_1, GPIO_PIN_RESET); \
} while(0)

void motor_set(uint8_t motor, int16_t speed);
void motor_init(void);
#endif
