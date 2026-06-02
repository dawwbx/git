#include "control_car.h"
#include "encoder.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
Car_Control_t Car;

/**
 * @brief 辅助函数：角度归一化处理 (-180 ~ 180)
 * 用于处理跨越 180度/-180度 的情况
 */
static float Optimize_Angle_Error(float target, float current) {
    float err = target - current;
    if (err > 180.0f) err -= 360.0f;
    else if (err < -180.0f) err += 360.0f;
    return err;
}

/**
 * @brief 初始化
 */
void Car_Control_Init(void) {
    // 初始化距离/速度 PID (KP, KI, KD, MaxOut, MaxI)
    // 参数需要根据实际小车重量和电机响应进行微调
    pid_init(&Car.speed_pid, 100.0f, 0.1f, 5.0f, MAX_SPEED_PWM, 200.0f);
    
    // 初始化航向保持 PID (用于走直线时修正偏航)
    // KP=5.0 表示每偏离1度，输出5的旋转力度修正
    pid_init(&Car.heading_pid, 50.0f, 0.0f, 0.5f, 7200.0f, 0.0f);
    
    motor_init(); // 调用底层的电机初始化
}

/**
 * @brief 麦轮逆运动学解算
 * 将期望的 Vx, Vy, Vw 转换为四个轮子的转速
 * * 麦轮布局 (O型长方形布局):
 * 1(FL) /////  \\\\\ 2(FR)
 * |      |
 * 3(RL) \\\\\  ///// 4(RR)
 */
void Car_Set_Velocity(float vx, float vy, float vw) {
    float v1, v2, v3, v4;

    // 经典麦轮运动学公式
    // 注意：这里的正负号取决于你的电机安装方向和tb6612驱动逻辑
    // 假设：motor_set(id, positive) 使轮子向前转
    
    v1 = vy + vx + vw; // FL: 前+ 右+ 逆时针+
    v2 = vy - vx - vw; // FR: 前+ 右- 逆时针-
    v3 = vy - vx + vw; // RL: 前+ 右- 逆时针+
    v4 = vy + vx - vw; // RR: 前+ 右+ 逆时针-

    // 寻找最大值，进行归一化，防止PWM饱和导致比例失调
    float max_v = 0.0f;
    if(fabs(v1) > max_v) max_v = fabs(v1);
    if(fabs(v2) > max_v) max_v = fabs(v2);
    if(fabs(v3) > max_v) max_v = fabs(v3);
    if(fabs(v4) > max_v) max_v = fabs(v4);

    if (max_v > MOTOR_MAX) {
        float scale = MOTOR_MAX / max_v;
        v1 *= scale;
        v2 *= scale;
        v3 *= scale;
        v4 *= scale;
    }

    // 设置电机速度 (假设 motor_set 支持 1-4)
    // 如果 motor_tb6612 只支持 1-2，请务必自行扩充底层驱动
    motor_set(1, (int16_t)v1);
    motor_set(2, (int16_t)v2);
    motor_set(3, (int16_t)v3);
    motor_set(4, (int16_t)v4);
}

void Car_Stop(void) {
    Car_Set_Velocity(0, 0, 0);
}

/**
 * @brief 获取当前Y轴移动距离 (cm)
 * 原理：四个轮子同向转动分量的平均值
 */
static float Get_Distance_Y_cm(int32_t start_enc_1, int32_t start_enc_2, 
                               int32_t start_enc_3, int32_t start_enc_4) {
    int32_t d1 = motor_encoder_1 - start_enc_1;
    int32_t d2 = motor_encoder_2 - start_enc_2;
    int32_t d3 = motor_encoder_3 - start_enc_3;
    int32_t d4 = motor_encoder_4 - start_enc_4;
    
    // Y轴运动：所有轮子同号
    float avg_pulse = (d1 + d2 + d3 + d4) / 4.0f;
    return avg_pulse / PULSE_PER_CM;
}

/**
 * @brief 获取当前X轴移动距离 (cm)
 * 原理：对角轮差分计算
 */
static float Get_Distance_X_cm(int32_t start_enc_1, int32_t start_enc_2, 
                               int32_t start_enc_3, int32_t start_enc_4) {
    int32_t d1 = motor_encoder_1 - start_enc_1;
    int32_t d2 = motor_encoder_2 - start_enc_2;
    int32_t d3 = motor_encoder_3 - start_enc_3;
    int32_t d4 = motor_encoder_4 - start_enc_4;
    
    // X轴运动 (向右)：FL+, FR-, RL-, RR+
    // 公式：(FL - FR - RL + RR) / 4
    float avg_pulse = (d1 - d2 - d3 + d4) / 4.0f;
    return avg_pulse / PULSE_PER_CM;
}

// =================================================================
// 1. 小车左右平移函数
// =================================================================
void Car_Move_Strafe(float distance_cm) {
    // 记录初始状态
    int32_t start_enc[4] = {motor_encoder_1, motor_encoder_2, motor_encoder_3, motor_encoder_4};
    float start_angle = angle; // 锁定当前朝向，平移过程中保持车头不变
    float current_moved = 0.0f;
    
    // 重置PID
    pid_init(&Car.speed_pid, 20.0f, 0.1f, 10.0f, MAX_SPEED_PWM, 200.0f); // 针对平移调整PID

    while (1) {
        // 1. 获取当前已经平移的距离
        current_moved = Get_Distance_X_cm(start_enc[0], start_enc[1], start_enc[2], start_enc[3]);
        
        // 2. 计算距离误差
        float dist_error = distance_cm - current_moved;
        
        // 3. 判断是否到达
        if (fabs(dist_error) < DIST_DEADZONE) {
            Car_Stop();
            break; 
        }

        // 4. 计算速度输出 (位置环)
        // 注意：这里把 pid_calc 的 set 设为 distance_cm，fdb 设为 current_moved
        // 也可以直接传 error 进去，视 pid.c 实现而定，这里按标准调用
        float output_vx = pid_calc(&Car.speed_pid, distance_cm, current_moved);

        // 5. 计算角度修正 (航向环)
        // 如果平移时车身歪了，通过旋转修正
        float angle_error = Optimize_Angle_Error(start_angle, angle);
        float output_vw = angle_error * 5.0f; // 简单P控制修正航向

        // 6. 执行输出 (Vy = 0)
        Car_Set_Velocity(output_vx, 0, output_vw);

        osDelay(10); // 控制周期 10ms
    }
}

// =================================================================
// 2. 小车直行前进固定距离函数
// =================================================================
void Car_Move_Straight(float distance_cm, float tolerance_cm) {
		Car_Stop();
    osDelay(100);
    int32_t start_enc[4] = {motor_encoder_1, motor_encoder_2, motor_encoder_3, motor_encoder_4};
    float start_angle = angle; // 锁定航向
    float current_moved = 0.0f;
		uint32_t timeout_counter = 0;
    // 重置PID
    pid_init(&Car.speed_pid, 300.0f, 1.0f, 20.0f, MAX_SPEED_PWM, 2000.0f);
		pid_init(&Car.heading_pid, 250.0f, 1.0f, 0.8f, MAX_SPEED_PWM, 0.0f);
    while (1) {
        // 1. 获取纵向距离
        current_moved = Get_Distance_Y_cm(start_enc[0], start_enc[1], start_enc[2], start_enc[3]);
        
        // 2. 检查误差
        float dist_error = distance_cm - current_moved;
        
        if (fabs(dist_error) < tolerance_cm) {
            Car_Stop();
            break;
        }

        // 3. 距离PID计算 Vy
        float output_vy = pid_calc(&Car.speed_pid, distance_cm, current_moved);

        // 4. 角度PID计算 Vw (走直线)
        // 目标是 start_angle, 反馈是当前 angle
        // 我们需要手动计算误差传入，或者如果pid_calc处理了环形误差则直接传
        // 既然用的是普通PID，我们最好把角度误差作为输入传给PID的error项，或者手动P控制
        float angle_err = Optimize_Angle_Error(start_angle, angle);
        float output_vw = pid_calc(&Car.heading_pid, 0, -angle_err); // 让误差趋向于0

        // 5. 输出 (Vx = 0)
        Car_Set_Velocity(0, output_vy, output_vw);
				if(++timeout_counter > 1000) {
            Car_Stop();
            break;
        }
        osDelay(10);
    }
}

// =================================================================
// 3. 小车原地转向函数
// =================================================================
void Car_Turn_Spot(float target_angle) {
    // 简单的 P 控制或者 PD 控制即可满足转向
    // 这里的 target_angle 是绝对角度 (-180 ~ 180)
    
    float err = 0.0f;
    uint32_t time_out = 0;

    while (1) {
        // 1. 计算最小角度误差
        err = Optimize_Angle_Error(target_angle, angle);

        // 2. 判断到达
        if (fabs(err) < ANGLE_DEADZONE) {
            Car_Stop();
            break;
        }
        
        // 超时退出 (防止一直转圈)
        if (++time_out > 500) { // 5秒
            Car_Stop(); 
            break; 
        }

        // 3. 简单的 P 控制计算 Vw
        float kp = 6.0f; 
        float output_vw = err * kp;

        // 限制最小启动速度，防止只有嗡嗡声不转
        if (output_vw > 0 && output_vw < 100) output_vw = 100;
        if (output_vw < 0 && output_vw > -100) output_vw = -100;

        // 4. 执行 (Vx=0, Vy=0)
        Car_Set_Velocity(0, 0, output_vw);

        osDelay(10);
    }
}

// =================================================================
// 4. 小车直角转弯函数 (圆弧插补)
// =================================================================
void Car_Turn_Arc(int8_t direction, float radius_cm, float target_angle_diff) {
    // 逻辑：V = Omega * R
    // 我们设定一个恒定的线速度 Vy，计算需要的角速度 Vw
    // Vw = Vy / Radius
    
    // 目标角度
    float target_angle = angle + (direction == 1 ? -target_angle_diff : target_angle_diff); 
    // 注意：这里假设 右转角度减小，左转角度增加。需根据IMU实际方向调整
    if (target_angle > 180) target_angle -= 360;
    if (target_angle < -180) target_angle += 360;

    float linear_speed = 300.0f; // 设定恒定线速度 PWM
    
    // 计算需要的旋转分量
    // 这里的单位需要匹配。假设 radius 是 cm。
    // 我们的 linear_speed 是 PWM值，这不对应物理速度。
    // 若要精确圆弧，必须知道 PWM与cm/s 的关系。
    // 这里采用 近似算法：利用PID控制角度，同时给定前进速度。
    
    // 更简单的实现直角转弯逻辑：
    // 同时给 Vy 和 Vw。
    // Vy = const, Vw 动态调整以贴合角度。
    
    while (1) {
        float err = Optimize_Angle_Error(target_angle, angle);
        
        if (fabs(err) < ANGLE_DEADZONE) {
            Car_Stop();
            break;
        }

        // 基础前进速度
        float out_vy = linear_speed;
        
        // 转向速度 Vw
        // 如果是定半径圆弧，Vw 应该是一个固定值 (linear / radius * K)
        // 但为了准确停在90度，我们叠加一个 P 控制
        float base_vw = (linear_speed / radius_cm) * 15.0f; // 15.0 是经验系数，用于单位转换
        
        // 根据方向决定旋转方向
        float out_vw = 0;
        if (direction == 1) { // 向右
             out_vw = -base_vw; // 假设顺时针为负
        } else { // 向左
             out_vw = base_vw;
        }
        
        // 靠近目标时减速 (可选)
        if (fabs(err) < 15.0f) {
            out_vy *= 0.5f;
            out_vw *= 0.5f;
        }

        Car_Set_Velocity(0, out_vy, out_vw);
        osDelay(10);
    }
}
