#ifndef __MPU6050_H__
#define __MPU6050_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <math.h>
#include <stdint.h>

/* I2C address (HAL expects 8-bit address: 7-bit<<1) */
#define MPU6050_I2C_ADDR         (0x68 << 1)

/* MPU-6050 registers */
#define MPU6050_REG_SMPLRT_DIV   0x19
#define MPU6050_REG_CONFIG       0x1A
#define MPU6050_REG_GYRO_CONFIG  0x1B
#define MPU6050_REG_ACCEL_CONFIG 0x1C
#define MPU6050_REG_PWR_MGMT_1   0x6B
#define MPU6050_REG_WHO_AM_I     0x75
#define MPU6050_REG_ACCEL_XOUT_H 0x3B

/* Sensitivity (default FS_SEL = 0 => ±250°/s; AFS_SEL = 0 => ±2g) */
#define MPU6050_ACCEL_SENSITIVITY 16384.0f   /* LSB/g */
#define MPU6050_GYRO_SENSITIVITY  131.0f     /* LSB/(°/s) */

#define MPU6050_RAD_TO_DEG (180.0f / 3.14159265358979323846f)

typedef struct {
    I2C_HandleTypeDef *hi2c; /* pointer to HAL I2C handle (e.g. &hi2c1) */

    /* raw + converted */
    int16_t rawAccelX;
    int16_t rawAccelY;
    int16_t rawAccelZ;
    int16_t rawTemp;
    int16_t rawGyroX;
    int16_t rawGyroY;
    int16_t rawGyroZ;

    float AccX_g;
    float AccY_g;
    float AccZ_g;
    float Temp_C;
    float GyroX_dps;
    float GyroY_dps;
    float GyroZ_dps;

    /* fused angles (degrees) */
    float Pitch;
    float Roll;
    float Yaw;

    int32_t gyroBiasRawX;
    int32_t gyroBiasRawY;
    int32_t gyroBiasRawZ;

} MPU6050_HandleTypeDef;

/* 初始化：将 dev->hi2c 指向 hi2c, 并做基本配置，返回 HAL status */
HAL_StatusTypeDef MPU6050_Init(MPU6050_HandleTypeDef *dev, I2C_HandleTypeDef *hi2c);

/* 读取原始 14 字节并转换到物理量（g, °/s, ℃），非阻塞之外仍是 HAL 的 blocking I2C 调用 */
HAL_StatusTypeDef MPU6050_ReadAll(MPU6050_HandleTypeDef *dev);

/* 用互补滤波（alpha: 0~1，通常 0.96~0.99）融合加速度与陀螺，dt 秒为时间间隔 */
void MPU6050_UpdateAngles(MPU6050_HandleTypeDef *dev, float dt, float alpha);

/* 低层读写帮助函数（可选直接使用） */
HAL_StatusTypeDef MPU6050_WriteReg(MPU6050_HandleTypeDef *dev, uint8_t reg, uint8_t data);
HAL_StatusTypeDef MPU6050_ReadRegs(MPU6050_HandleTypeDef *dev, uint8_t reg, uint8_t *buf, uint16_t len);


HAL_StatusTypeDef MPU6050_CalibrateGyro(MPU6050_HandleTypeDef *dev, uint16_t samples, uint16_t delay_ms);
#ifdef __cplusplus
}
#endif

#endif /* __MPU6050_H__ */
