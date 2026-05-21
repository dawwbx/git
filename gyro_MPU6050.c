#include "gyro_MPU6050.h"

/* ----------------- 低层 I2C 读写 ----------------- */
HAL_StatusTypeDef MPU6050_WriteReg(MPU6050_HandleTypeDef *dev, uint8_t reg, uint8_t data)
{
    if (!dev || !dev->hi2c) return HAL_ERROR;
    return HAL_I2C_Mem_Write(dev->hi2c, MPU6050_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
}

HAL_StatusTypeDef MPU6050_ReadRegs(MPU6050_HandleTypeDef *dev, uint8_t reg, uint8_t *buf, uint16_t len)
{
    if (!dev || !dev->hi2c) return HAL_ERROR;
    return HAL_I2C_Mem_Read(dev->hi2c, MPU6050_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, buf, len, 100);
}

/* ----------------- 初始化 ----------------- */
HAL_StatusTypeDef MPU6050_Init(MPU6050_HandleTypeDef *dev, I2C_HandleTypeDef *hi2c)
{
    HAL_StatusTypeDef ret;
    uint8_t who = 0;
    dev->gyroBiasRawX = dev->gyroBiasRawY = dev->gyroBiasRawZ = 0;
    if (!dev || !hi2c) return HAL_ERROR;
    dev->hi2c = hi2c;

    /* 检查 WHO_AM_I */
    ret = MPU6050_ReadRegs(dev, MPU6050_REG_WHO_AM_I, &who, 1);
    if (ret != HAL_OK) return ret;
    if (who != 0x68) {
        /* 设备地址不对或未连接 */
        return HAL_ERROR;
    }

    /* 取消睡眠，选择时钟源（使用内部或 PLL） */
    ret = MPU6050_WriteReg(dev, MPU6050_REG_PWR_MGMT_1, 0x01); /* 设置CLK为PLL X轴，退出睡眠 */
    if (ret != HAL_OK) return ret;
    HAL_Delay(10);

    /* 采样率分频： sample_rate = Gyro_output_rate / (1 + SMPLRT_DIV)
       默认Gyro_output_rate = 1kHz when DLPF enabled, 8kHz when disabled.
       这里设置 SMPLRT_DIV=7 -> 1000/(1+7)=125 Hz */
    ret = MPU6050_WriteReg(dev, MPU6050_REG_SMPLRT_DIV, 0x07);
    if (ret != HAL_OK) return ret;

    /* DLPF 配置（CONFIG）: 选择合适的低通滤波。0x06 -> 5 Hz（可根据需要调整） */
    ret = MPU6050_WriteReg(dev, MPU6050_REG_CONFIG, 0x06);
    if (ret != HAL_OK) return ret;

    /* 陀螺量程（GYRO_CONFIG）: 0x00 => ±250 °/s */
    ret = MPU6050_WriteReg(dev, MPU6050_REG_GYRO_CONFIG, 0x00);
    if (ret != HAL_OK) return ret;

    /* 加速度量程（ACCEL_CONFIG）: 0x00 => ±2 g */
    ret = MPU6050_WriteReg(dev, MPU6050_REG_ACCEL_CONFIG, 0x00);
    if (ret != HAL_OK) return ret;

    /* 初始化角度为 0 */
    dev->Pitch = 0.0f;
    dev->Roll  = 0.0f;
    dev->Yaw   = 0.0f;

    return HAL_OK;
}

/* ----------------- 校准函数 ----------------- */
HAL_StatusTypeDef MPU6050_CalibrateGyro(MPU6050_HandleTypeDef *dev, uint16_t samples, uint16_t delay_ms)
{
    if (!dev || !dev->hi2c) return HAL_ERROR;
    int64_t sumX = 0, sumY = 0, sumZ = 0;
    for (uint16_t i = 0; i < samples; ++i) {
        if (MPU6050_ReadAll(dev) != HAL_OK) return HAL_ERROR;
        sumX += dev->rawGyroX;
        sumY += dev->rawGyroY;
        sumZ += dev->rawGyroZ;
        HAL_Delay(delay_ms);
    }
    dev->gyroBiasRawX = (int32_t)(sumX / samples);
    dev->gyroBiasRawY = (int32_t)(sumY / samples);
    dev->gyroBiasRawZ = (int32_t)(sumZ / samples);
    return HAL_OK;
}
/* ----------------- 读取 14 字节并换算 ----------------- */
HAL_StatusTypeDef MPU6050_ReadAll(MPU6050_HandleTypeDef *dev)
{
    uint8_t buf[14];
    HAL_StatusTypeDef ret;

    if (!dev || !dev->hi2c) return HAL_ERROR;

    ret = MPU6050_ReadRegs(dev, MPU6050_REG_ACCEL_XOUT_H, buf, 14);
    if (ret != HAL_OK) return ret;

    dev->rawAccelX = (int16_t)((buf[0] << 8) | buf[1]);
    dev->rawAccelY = (int16_t)((buf[2] << 8) | buf[3]);
    dev->rawAccelZ = (int16_t)((buf[4] << 8) | buf[5]);
    dev->rawTemp   = (int16_t)((buf[6] << 8) | buf[7]);
    dev->rawGyroX  = (int16_t)((buf[8] << 8) | buf[9]);
    dev->rawGyroY  = (int16_t)((buf[10] << 8) | buf[11]);
    dev->rawGyroZ  = (int16_t)((buf[12] << 8) | buf[13]);

    /* 转换 */
    dev->AccX_g = ((float)dev->rawAccelX) / MPU6050_ACCEL_SENSITIVITY;
    dev->AccY_g = ((float)dev->rawAccelY) / MPU6050_ACCEL_SENSITIVITY;
    dev->AccZ_g = ((float)dev->rawAccelZ) / MPU6050_ACCEL_SENSITIVITY;


    
    /* 扣偏置（raw 单位），再换算为 dps */
    int32_t corrX = (int32_t)dev->rawGyroX - dev->gyroBiasRawX;
    int32_t corrY = (int32_t)dev->rawGyroY - dev->gyroBiasRawY;
    int32_t corrZ = (int32_t)dev->rawGyroZ - dev->gyroBiasRawZ;

    dev->GyroX_dps = ((float)corrX) / MPU6050_GYRO_SENSITIVITY;
    dev->GyroY_dps = ((float)corrY) / MPU6050_GYRO_SENSITIVITY;
    dev->GyroZ_dps = ((float)corrZ) / MPU6050_GYRO_SENSITIVITY;

    dev->Temp_C = ((float)dev->rawTemp) / 340.0f + 36.53f;

    return HAL_OK;
}

/* ----------------- 互补滤波更新角度 -----------------
   dt: 秒，alpha: 0~1 (陀螺权重)，例如 0.98
   说明: Pitch/Roll 采用互补滤波；Yaw 仅基于陀螺积分（会漂移）
-------------------------------------------------- */
void MPU6050_UpdateAngles(MPU6050_HandleTypeDef *dev, float dt, float alpha)
{
    if (!dev) return;

    /* 先用最新读到的物理量（请确保已调用 MPU6050_ReadAll） */
    /* 计算由加速度得到的角度（弧度->度） */
    float pitchAcc = atan2f(dev->AccY_g, sqrtf(dev->AccX_g * dev->AccX_g + dev->AccZ_g * dev->AccZ_g)) * MPU6050_RAD_TO_DEG;
    float rollAcc  = atan2f(-dev->AccX_g, dev->AccZ_g) * MPU6050_RAD_TO_DEG;

    /* 用陀螺角速度积分（度/s * s = 度） */
    /* 注意：MPU 陀螺轴方向与坐标系定义要对应，如果发现正负反了可在外部取反 */
    dev->Pitch += dev->GyroX_dps * dt;
    dev->Roll  += dev->GyroY_dps * dt;
    dev->Yaw   += dev->GyroZ_dps * dt;

    /* 互补滤波融合 */
    dev->Pitch = alpha * dev->Pitch + (1.0f - alpha) * pitchAcc;
    dev->Roll  = alpha * dev->Roll  + (1.0f - alpha) * rollAcc;
    /* Yaw 无加速度参考，保持陀螺积分值（会随时间漂移） */
}

/* ------------- 可按需添加重置或校准函数 ------------- */



//使用说明：
// MPU6050_HandleTypeDef mpu;
// uint32_t last_ms = HAL_GetTick();
// while (1) {
//     if (MPU6050_ReadAll(&mpu) == HAL_OK) {
//         uint32_t now = HAL_GetTick();
//         float dt = (now - last_ms) / 1000.0f;
//         if (dt <= 0) dt = 0.001f;
//         last_ms = now;

//         /* 更新角度（互补滤波 alpha 示例） */
//         MPU6050_UpdateAngles(&mpu, dt, 0.98f);

//         /* 打印用于调试 */
//         printf("P:%.2f R:%.2f Y:%.2f rawGz=%d gz=%.3f\n",
//                mpu.Pitch, mpu.Roll, mpu.Yaw, mpu.rawGyroZ, mpu.GyroZ_dps);
//     }
//     HAL_Delay(5);
// }