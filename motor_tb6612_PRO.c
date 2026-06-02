#include "motor_tb6612_PRO.h"

static const motor_pro_cfg_t g_motor_cfg[MOTOR_PRO_COUNT] = MOTOR_PRO_CONFIG_TABLE;
static motor_pro_stop_mode_t g_stop_mode = MOTOR_PRO_STOP_MODE_BRAKE;

static uint16_t motor_pro_limit_speed(int16_t speed)
{
    uint32_t pwm;

    if (speed >= 0)
    {
        pwm = (uint32_t)speed;
    }
    else
    {
        pwm = (uint32_t)(-speed);
    }

    if (pwm > MOTOR_PRO_MAX_SPEED)
    {
        pwm = MOTOR_PRO_MAX_SPEED;
    }

    return (uint16_t)pwm;
}

static void motor_pro_write_direction(const motor_pro_cfg_t *cfg, int8_t direction)
{
    if ((cfg == 0) || (direction == 0))
    {
        if (cfg != 0)
        {
            if (g_stop_mode == MOTOR_PRO_STOP_MODE_BRAKE)
            {
                HAL_GPIO_WritePin(cfg->in1_port, cfg->in1_pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(cfg->in2_port, cfg->in2_pin, GPIO_PIN_SET);
            }
            else
            {
                HAL_GPIO_WritePin(cfg->in1_port, cfg->in1_pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(cfg->in2_port, cfg->in2_pin, GPIO_PIN_RESET);
            }
        }
        return;
    }

    if (cfg->invert != 0U)
    {
        direction = (int8_t)(-direction);
    }

    if (direction > 0)
    {
        HAL_GPIO_WritePin(cfg->in1_port, cfg->in1_pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(cfg->in2_port, cfg->in2_pin, GPIO_PIN_RESET);
    }
    else
    {
        HAL_GPIO_WritePin(cfg->in1_port, cfg->in1_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(cfg->in2_port, cfg->in2_pin, GPIO_PIN_SET);
    }
}

uint8_t motor_pro_count(void)
{
    return (uint8_t)MOTOR_PRO_COUNT;
}

void motor_pro_init(void)
{
    uint8_t i;

    for (i = 0; i < MOTOR_PRO_COUNT; i++)
    {
        HAL_TIM_PWM_Start(g_motor_cfg[i].htim, g_motor_cfg[i].channel);
        motor_pro_set((uint8_t)(i + 1U), 0);
    }
}

void motor_pro_set(uint8_t motor, int16_t speed)
{
    const motor_pro_cfg_t *cfg;
    uint16_t pwm;
    int8_t direction = 0;

    if ((motor == 0U) || (motor > MOTOR_PRO_COUNT))
    {
        return;
    }

    cfg = &g_motor_cfg[motor - 1U];
    pwm = motor_pro_limit_speed(speed);

    if (speed > 0)
    {
        direction = 1;
    }
    else if (speed < 0)
    {
        direction = -1;
    }

    motor_pro_write_direction(cfg, direction);
    __HAL_TIM_SetCompare(cfg->htim, cfg->channel, pwm);
}

void motor_pro_set_all(const int16_t *speeds, uint8_t count)
{
    uint8_t i;
    uint8_t limit;

    if (speeds == 0)
    {
        return;
    }

    limit = (count < MOTOR_PRO_COUNT) ? count : (uint8_t)MOTOR_PRO_COUNT;
    for (i = 0; i < limit; i++)
    {
        motor_pro_set((uint8_t)(i + 1U), speeds[i]);
    }
}

void motor_pro_set_stop_mode(motor_pro_stop_mode_t mode)
{
    g_stop_mode = mode;
}

void motor_pro_stop(uint8_t motor)
{
    motor_pro_set(motor, 0);
}

void motor_pro_stop_all(void)
{
    uint8_t i;

    for (i = 0; i < MOTOR_PRO_COUNT; i++)
    {
        motor_pro_set((uint8_t)(i + 1U), 0);
    }
}
