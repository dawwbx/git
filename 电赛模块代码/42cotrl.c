#include "cotrl.h"
#include <math.h>
#include <stdlib.h>  // 添加到文件顶部

// 定义电机 IO 映射结构体


// 量程限制
#define MOTOR_MIN_POSITION 0
#define MOTOR_MAX_POSITION 400000
// 脉冲间隔延时（可根据硬件调整）
#define PULSE_DELAY_NOP 10

// 全局位置（绝对步数）
static int32_t group13_position = 0;    // 电机1&3组当前绝对位置
static int32_t motor2_position   = 0;   // 电机2当前绝对位置

// 电机端口定义（3个物理电机：仅用电机1、2、3）
static const MotorDef motor_defs[3] = {
    // 电机1
    { STEP_1_GPIO_Port, STEP_1_Pin, DIR_1_GPIO_Port, DIR_1_Pin,
      EN_1_GPIO_Port,   EN_1_Pin,   MS1_1_GPIO_Port, MS1_1_Pin,
      MS1_2_GPIO_Port, MS1_2_Pin },
    // 电机2
    { STEP_2_GPIO_Port, STEP_2_Pin, DIR_2_GPIO_Port, DIR_2_Pin,
      EN_2_GPIO_Port,   EN_2_Pin,   MS2_1_GPIO_Port, MS2_1_Pin,
      MS2_2_GPIO_Port, MS2_2_Pin },
    // 电机3
    { STEP_3_GPIO_Port, STEP_3_Pin, DIR_3_GPIO_Port, DIR_3_Pin,
      EN_3_GPIO_Port,   EN_3_Pin,   MS3_1_GPIO_Port, MS3_1_Pin,
      MS3_2_GPIO_Port, MS3_2_Pin }
};

/**
 * @brief 延时空操作
 * @param count  延时循环次数
 */
void DelayNop(uint16_t count)
{
    while (count--) {
        for (volatile uint32_t j = 0; j < 50; j++) {;}
    }
}

/**
 * @brief 同步发送电机1和3的步进脉冲
 */
static inline void SendStepPulseGroup13(const MotorDef *m1, const MotorDef *m3)
{
    // 同时拉低
    HAL_GPIO_WritePin(m1->step_port, m1->step_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(m3->step_port, m3->step_pin, GPIO_PIN_RESET);
    DelayNop(1);
    // 同时拉高
    HAL_GPIO_WritePin(m1->step_port, m1->step_pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(m3->step_port, m3->step_pin, GPIO_PIN_SET);
    DelayNop(PULSE_DELAY_NOP);
}

/**
 * @brief 单电机发送步进脉冲
 */
static inline void SendStepPulse(const MotorDef *def)
{
    HAL_GPIO_WritePin(def->step_port, def->step_pin, GPIO_PIN_RESET);
    DelayNop(1);
    HAL_GPIO_WritePin(def->step_port, def->step_pin, GPIO_PIN_SET);
    DelayNop(PULSE_DELAY_NOP);
}

/**
 * @brief 使能并设置方向
 */
static void EnableAndSetDir(const MotorDef *def, uint8_t dir_flag)
{
    HAL_GPIO_WritePin(def->en_port, def->en_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(def->dir_port, def->dir_pin,
                     dir_flag ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/**
 * @brief 禁用电机
 */
void DisableMotor(const MotorDef *def)
{
    HAL_GPIO_WritePin(def->en_port, def->en_pin, GPIO_PIN_SET);
}

/**
 * @brief 设置子步尺寸（通用）
 */
void SubdivisionSet(uint8_t motor, uint8_t microstep)
{
    if (motor < 1 || motor > 3) return;
    const MotorDef *m = &motor_defs[motor - 1];
    if (microstep == 8) {
        HAL_GPIO_WritePin(m->ms2_port, m->ms2_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(m->ms1_port, m->ms1_pin, GPIO_PIN_RESET);
    } else if (microstep == 32) {
        HAL_GPIO_WritePin(m->ms2_port, m->ms2_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(m->ms1_port, m->ms1_pin, GPIO_PIN_SET);
    } else if (microstep == 64) {
        HAL_GPIO_WritePin(m->ms2_port, m->ms2_pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(m->ms1_port, m->ms1_pin, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(m->ms2_port, m->ms2_pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(m->ms1_port, m->ms1_pin, GPIO_PIN_SET);
    }
}

/**
 * @brief 控制电机1&3组移动到指定绝对位置
 * @param target 目标绝对步数（0~390000）
 */
void MoveGroup13To(int32_t target)
{
    // 限幅保护
    if (target > MOTOR_MAX_POSITION) target = MOTOR_MAX_POSITION;
    if (target < MOTOR_MIN_POSITION) target = MOTOR_MIN_POSITION;

    int32_t delta = target - group13_position;
    if (delta == 0) return;  // 无需移动

    uint8_t dir_flag = (delta < 0) ? 1 : 0;
    uint32_t steps = abs(delta);

    const MotorDef *m1 = &motor_defs[0];
    const MotorDef *m3 = &motor_defs[2];

    // 使能并设置方向
    EnableAndSetDir(m1, dir_flag);
    EnableAndSetDir(m3, dir_flag);

    // 同步步进脉冲
    for (uint32_t i = 0; i < steps; i++) {
        SendStepPulseGroup13(m1, m3);
        group13_position += dir_flag ? -1 : 1;
    }

    HAL_Delay(10);
    DisableMotor(m1);
    DisableMotor(m3);
}

/**
 * @brief 控制电机2移动到指定绝对位置
 * @param target 目标绝对步数（0~390000）
 */
void MoveMotor2To(int32_t target)
{
    // 限幅保护
    if (target > MOTOR_MAX_POSITION) target = MOTOR_MAX_POSITION;
    if (target < MOTOR_MIN_POSITION) target = MOTOR_MIN_POSITION;

    int32_t delta = target - motor2_position;
    if (delta == 0) return;  // 无需移动

    uint8_t dir_flag = (delta < 0) ? 1 : 0;
    uint32_t steps = abs(delta);

    const MotorDef *m2 = &motor_defs[1];

    EnableAndSetDir(m2, dir_flag);

    for (uint32_t i = 0; i < steps; i++) {
        SendStepPulse(m2);
        motor2_position += dir_flag ? -1 : 1;
    }

    HAL_Delay(10);
    DisableMotor(m2);
}
/**
 * @brief 控制所有电机并行移动到指定位置
 * @param target_group13 电机1&3组目标位置
 * @param target_motor2 电机2目标位置
 */
void MoveAllMotorsParallel(int32_t target_group13, int32_t target_motor2)
{
    // 限幅保护
    if (target_group13 > MOTOR_MAX_POSITION) target_group13 = MOTOR_MAX_POSITION;
    if (target_group13 < MOTOR_MIN_POSITION) target_group13 = MOTOR_MIN_POSITION;
    if (target_motor2 > MOTOR_MAX_POSITION) target_motor2 = MOTOR_MAX_POSITION;
    if (target_motor2 < MOTOR_MIN_POSITION) target_motor2 = MOTOR_MIN_POSITION;

    int32_t delta_group13 = target_group13 - group13_position;
    int32_t delta_motor2 = target_motor2 - motor2_position;

    const MotorDef *m1 = &motor_defs[0];
    const MotorDef *m2 = &motor_defs[1];
    const MotorDef *m3 = &motor_defs[2];

    // 使能并设置方向
    if(delta_group13 != 0) {
        uint8_t dir_flag_group13 = (delta_group13 < 0) ? 1 : 0;
        EnableAndSetDir(m1, dir_flag_group13);
        EnableAndSetDir(m3, dir_flag_group13);
    }
    
    if(delta_motor2 != 0) {
        uint8_t dir_flag_motor2 = (delta_motor2 < 0) ? 1 : 0;
        EnableAndSetDir(m2, dir_flag_motor2);
    }

    uint32_t steps_group13 = abs(delta_group13);
    uint32_t steps_motor2 = abs(delta_motor2);
    uint32_t max_steps = steps_group13 > steps_motor2 ? steps_group13 : steps_motor2;

    for (uint32_t i = 0; i < max_steps; i++) {
        if(i < steps_group13) {
            SendStepPulseGroup13(m1, m3);
            group13_position += (delta_group13 < 0) ? -1 : 1;
        }
        if(i < steps_motor2) {
            SendStepPulse(m2);
            motor2_position += (delta_motor2 < 0) ? -1 : 1;
        }
    }

    HAL_Delay(10);
    DisableMotor(m1);
    DisableMotor(m2);
    DisableMotor(m3);
}