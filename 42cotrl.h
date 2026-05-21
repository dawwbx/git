#ifndef _SIMULTANEOUS_MOTOR_CONTROL_H_
#define _SIMULTANEOUS_MOTOR_CONTROL_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"  // 根据实际MCU系列修改
#include <stdint.h>

#include "main.h"  // 根据实际MCU系列修改
typedef struct {
    GPIO_TypeDef *step_port; uint16_t step_pin;
    GPIO_TypeDef *dir_port;  uint16_t dir_pin;
    GPIO_TypeDef *en_port;   uint16_t en_pin;
    GPIO_TypeDef *ms1_port;  uint16_t ms1_pin;
    GPIO_TypeDef *ms2_port;  uint16_t ms2_pin;
} MotorDef;
// 电机位置范围
#define MOTOR_MIN_POSITION   0
#define MOTOR_MAX_POSITION   390000

void MoveAllMotorsParallel(int32_t target_group13, int32_t target_motor2);

/**
 * @brief 设置电机子步模式
 * @param motor      电机序号（1~3）
 * @param microstep  子步类型，可选值：2, 8, 32, 64
 */
void SubdivisionSet(uint8_t motor, uint8_t microstep);

/**
 * @brief 将电机1和电机3绑定为一组，同步移动到指定绝对位置
 * @param target 目标绝对步数（0~390000）
 */
void MoveGroup13To(int32_t target);
void DisableMotor(const MotorDef *def);
/**
 * @brief 控制电机2移动到指定绝对位置
 * @param target 目标绝对步数（0~390000）
 */
void MoveMotor2To(int32_t target);

/**
 * @brief 获取当前电机1&3组的位置
 * @return 当前绝对步数（0~390000）
 */
int32_t GetGroup13Position(void);

/**
 * @brief 获取当前电机2的位置
 * @return 当前绝对步数（0~390000）
 */
int32_t GetMotor2Position(void);
void SendInterleavedGroup13(const MotorDef *m1, const MotorDef *m3);
#ifdef __cplusplus
}
#endif

#endif // _SIMULTANEOUS_MOTOR_CONTROL_H_
