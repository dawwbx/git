#include "control_car.h"
#include <math.h>
#include <stdlib.h>

/* * 引用外部全局变量
 * 请确保在你的 main.c 或 stm32f4xx_it.c 中定义了这些变量并实时更新
 */
extern volatile float angle; // 当前角度 (-180.0 ~ 180.0)
extern volatile int32_t motor_encoder_1; // 左前
extern volatile int32_t motor_encoder_2; // 左后
extern volatile int32_t motor_encoder_3; // 右后
extern volatile int32_t motor_encoder_4; // 右前

/* 定义PID控制器实例 */
PID pid_distance; // 距离环PID
PID pid_angle;    // 角度环PID

/* 内部辅助函数声明 */
static float Get_Current_Distance_Avg(int32_t start_enc_l, int32_t start_enc_r);
static float Normalize_Angle_Error(float target, float current);
static void Car_Stop(void);

/**
  * @brief  小车控制初始化，初始化PID参数
  * @note   参数需要根据实际车辆进行整定
  */
void Car_Control_Init(void)
{
    motor_init();

    // 初始化距离PID (目标: cm, 反馈: cm, 输出: PWM速度)
    // KP, KI, KD, MaxOutput, MaxIntegral
    pid_init(&pid_distance, 150.0f, 0.5f, 20.0f, 5000.0f, 1000.0f);

    // 初始化角度PID (目标: 度, 反馈: 度, 输出: PWM差值/转向速度)
    // 用于直行时的修正: KP应较小; 用于原地转向: KP应较大
    pid_init(&pid_angle, 60.0f, 0.1f, 10.0f, 4000.0f, 1000.0f);
}

/**
  * @brief  1. 小车直行前进固定距离
  * @param  distance: 目标距离 (cm)
  * @param  target_heading: 前进时的锁定朝向 (-180~180)
  * @param  tolerance: 允许的距离误差 (cm)
  */
void Car_Go_Straight(float distance, float target_heading, float tolerance)
{
    // 记录初始编码器值
    int32_t start_l = (motor_encoder_1 + motor_encoder_2) / 2;
    int32_t start_r = (motor_encoder_3 + motor_encoder_4) / 2;

    float current_dist = 0.0f;
    float dist_error = distance;

    // 循环直到误差在允许范围内
    while (fabs(dist_error) > tolerance)
    {
        // 1. 获取当前行驶距离
        current_dist = Get_Current_Distance_Avg(start_l, start_r);

        // 2. 计算距离PID输出 (基础速度)
        float base_speed = pid_calc(&pid_distance, distance, current_dist);

        // 3. 计算角度PID输出 (修正转向)
        // 目标是保持 target_heading，如果偏离则修正
        float angle_err = Normalize_Angle_Error(target_heading, angle);

        // 临时调整角度PID参数用于直行修正（通常比原地转向柔和）
        // 如果使用同一套PID，可以忽略此步或使用两套PID结构体
        float turn_adjust = angle_err * 30.0f; // 简单比例控制作为演示，建议使用pid_angle计算

        // 4. 融合输出
        // 直轮车差速模型：左轮=基速-修正，右轮=基速+修正
        float speed_l = base_speed - turn_adjust;
        float speed_r = base_speed + turn_adjust;

        // 5. 执行电机控制
        motor_set(1, (int16_t)speed_l); // 左前
        motor_set(2, (int16_t)speed_l); // 左后
        motor_set(3, (int16_t)speed_r); // 右后
        motor_set(4, (int16_t)speed_r); // 右前

        // 更新误差条件
        dist_error = distance - current_dist;

        HAL_Delay(10); // 控制周期 10ms
    }

    Car_Stop();
}

void Car(float distance, float target_heading, float tolerance)
{
// 记录初始编码器值
    int32_t start_l = (motor_encoder_1 + motor_encoder_2) / 2;
    int32_t start_r = (motor_encoder_3 + motor_encoder_4) / 2;

    float current_dist = 0.0f;
    pid_init(&pid_distance, 150.0f, 0.5f, 20.0f, 5000.0f, 1000.0f);
    while (1)
    { 
         // 1. 获取当前行驶距离
        current_dist = Get_Current_Distance_Avg(start_l, start_r);

        if (current_dist >= distance)
        {
            Car_Stop();
            break;
        }
        
        float base_speed = pid_calc(&pid_distance, distance, current_dist);//距离PID输出
        // 2. 计算角度PID输出 (修正转向)
         float turn_adjust = pid_calc(&pid_angle, target_heading, angle);//角度修正PID输出

        // 3. 融合输出
        float speed_l = base_speed - turn_adjust;
        float speed_r = base_speed + turn_adjust;

        // 4. 执行电机控制
        motor_set(1, (int16_t)speed_l); // 左前
        motor_set(2, (int16_t)speed_l); // 左后
        motor_set(3, (int16_t)speed_r); // 右后
        motor_set(4, (int16_t)speed_r); // 右前
    }
}

void Car_Turn(float target_abs_angle, float error)
{
    if(target_abs_angle > 0 )int8_t dir = 1; //左转
    else int8_t dir = -1; //右转

    float xianzai_angle =  angle;//记录当前角度
    pid_init(&pid_trun, 150.0f, 0.5f, 20.0f, 5000.0f, 1000.0f);
    while (1))
    {
        if(dir)//右转的退出条件
        if (angle >= target_abs_angle)
        {
            break;
        }
        if(dir == -1)//左转的退出条件
        if (angle <= target_abs_angle)
        {
            break;
        }

         float b = pid_calc(&pid_angle, 90, 0);//角度PID输出'
         a = b * dir;
        motor_set(1, (int16_t)a);
        motor_set(2, (int16_t)a);
        motor_set(3, (int16_t)-a);
        motor_set(4, (int16_t)-a);

    }
    
}








/**
  * @brief  2. 小车原地转向
  * @param  dir: 1 为向左转(逆时针), -1 为向右转(顺时针)
  * @param  target_abs_angle: 转向的目标绝对角度 (-180~180)
  */
void Car_Turn_In_Place(int8_t dir, float target_abs_angle)
{
    float angle_error = Normalize_Angle_Error(target_abs_angle, angle);
    float angle_tolerance = 2.0f; // 允许2度误差

    while (fabs(angle_error) > angle_tolerance)
    {
        // 计算角度PID，输出为旋转力矩
        float turn_speed = pid_calc(&pid_angle, target_abs_angle, angle);

        // 如果PID计算未处理环形角度突变（180跳变到-180），需手动处理 error
        // 这里假设 pid_calc 内部直接用 target - input。
        // 为了更好的效果，我们直接对 error 进行 PID 计算 (修改 pid_calc 使用传入的 error)
        // 或者简单地：
        turn_speed = pid_angle.kp * angle_error; // 简化演示，实际请用完整PID

        // 限幅
        if(turn_speed > 3000) turn_speed = 3000;
        if(turn_speed < -3000) turn_speed = -3000;

        // 保证最小启动电压
        if(turn_speed > 0 && turn_speed < 800) turn_speed = 800;
        if(turn_speed < 0 && turn_speed > -800) turn_speed = -800;

        // 原地转向：左轮和右轮速度相反
        // 向左转：左轮后退，右轮前进
        // 向右转：左轮前进，右轮后退
        // 实际上 PID 输出的正负已经包含了方向信息，如果 target > current，error > 0，需要左转

        motor_set(1, (int16_t)(-turn_speed));
        motor_set(2, (int16_t)(-turn_speed));
        motor_set(3, (int16_t)(turn_speed));
        motor_set(4, (int16_t)(turn_speed));

        angle_error = Normalize_Angle_Error(target_abs_angle, angle);
        HAL_Delay(10);
    }
    Car_Stop();
}

/**
  * @brief  3. 智能直行与弯道融合函数
  * @param  distance: 行驶距离 (如果转弯，则此为转弯半径 R)
  * @param  target_angle: 目标朝向
  * @param  dist_allow_err: 距离允许误差
  * @param  angle_allow_err: 角度允许误差
  */
void Car_Smart_Move(float distance, float target_angle, float dist_allow_err, float angle_allow_err)
{
    float current_ang_err = Normalize_Angle_Error(target_angle, angle);

    // 逻辑判断：如果当前角度与目标角度基本一致 (误差 < 5度)，则认为是直行
    if (fabs(current_ang_err) < 5.0f)
    {
        // 执行直行逻辑
        Car_Go_Straight(distance, target_angle, dist_allow_err);
    }
    else
    {
        // 执行差速弯道逻辑
        // 输入距离 distance 变为 转弯半径 Radius
        // 目标是转到 target_angle

        float radius = distance;
        float wheel_track = TRACK_WIDTH; // 轮距，单位 cm

        // 循环直到角度到达目标
        while (fabs(current_ang_err) > angle_allow_err)
        {
            // 1. 利用角度PID产生角速度 (Omega) 对应的基础线速度
            // 我们希望当角度误差减小时，车速也减慢，平滑停在目标角度
            // 这里把 PID 的输出作为 "中心线速度 V_center"

            float v_center = pid_calc(&pid_angle, target_angle, angle);

            // 修正 PID 逻辑：在弯道中，我们希望以恒定速度行驶，直到接近目标角度？
            // 或者是根据误差调整旋转速度？
            // 更好的方式：PID 输出为角速度 Omega
            // V_center = Omega * Radius

            // 重新计算 error 供 PID 使用
            // 注意：这里简单使用比例控制模拟 PID 输出 Omega
            float omega_cmd = current_ang_err * 0.1f; // 系数需整定

            // 限制最大角速度防止失控
            if (omega_cmd > 2.0f) omega_cmd = 2.0f;
            if (omega_cmd < -2.0f) omega_cmd = -2.0f;

            // 基础速度 V = Omega * R
            // 为了保证能动，设置一个基础 V_center，方向由 Omega 决定正负逻辑比较复杂
            // 让我们换一种思路：
            // V_center 设为固定值 (如 1500 PWM 对应的速度)，通过差速实现转弯半径 R

            float base_pwm = 2000.0f; // 假设的基础前进速度
            // 如果误差非常小，减速
            if(fabs(current_ang_err) < 15.0f) base_pwm = 1000.0f;

            // 差速公式:
            // V_L = V_center * (R - W/2) / R
            // V_R = V_center * (R + W/2) / R
            // 注意：这只适用于圆心在左侧或右侧的情况。
            // 需要判断是左转还是右转

            float vl, vr;

            // 判断转弯方向
            if (current_ang_err > 0) // 目标 > 当前，需要增加角度，左转
            {
                // 左转：圆心在左侧。左轮慢，右轮快。
                // 左轮半径 = R - W/2, 右轮半径 = R + W/2
                vl = base_pwm * (radius - wheel_track/2.0f) / radius;
                vr = base_pwm * (radius + wheel_track/2.0f) / radius;
            }
            else // 右转
            {
                // 右转：圆心在右侧。左轮快，右轮慢。
                // 左轮半径 = R + W/2, 右轮半径 = R - W/2
                vl = base_pwm * (radius + wheel_track/2.0f) / radius;
                vr = base_pwm * (radius - wheel_track/2.0f) / radius;
            }

            motor_set(1, (int16_t)vl);
            motor_set(2, (int16_t)vl);
            motor_set(3, (int16_t)vr);
            motor_set(4, (int16_t)vr);

            current_ang_err = Normalize_Angle_Error(target_angle, angle);
            HAL_Delay(10);
        }
        Car_Stop();
    }
}

/* ================= 辅助函数 ================= */

// 计算当前行驶的平均距离 (cm)
// 假设 motor_encoder 是累加值
static float Get_Current_Distance_Avg(int32_t start_enc_l, int32_t start_enc_r)
{
    // 获取当前编码器值（注意：需确保读取原子性或在中断不频繁时读取）
    int32_t curr_l = (motor_encoder_1 + motor_encoder_2) / 2;
    int32_t curr_r = (motor_encoder_3 + motor_encoder_4) / 2;

    int32_t delta_l = abs(curr_l - start_l);
    int32_t delta_r = abs(curr_r - start_r);

    int32_t avg_ticks = (delta_l + delta_r) / 2;

    // 将脉冲数转换为厘米
    // 距离 = (脉冲数 / 一圈总脉冲) * 轮子周长
    float distance = ((float)avg_ticks / ENCODER_TICKS_PER_REV) * (WHEEL_DIAMETER * 3.14159f);

    return distance;
}

// 角度误差标准化 (-180 ~ 180)
// 例如：当前 170，目标 -170。差值 -340 -> 标准化为 +20 (左转更近)
static float Normalize_Angle_Error(float target, float current)
{
    float error = target - current;
    while (error > 180.0f) error -= 360.0f;
    while (error < -180.0f) error += 360.0f;
    return error;
}

static void Car_Stop(void)
{
    motor_set(1, 0);
    motor_set(2, 0);
    motor_set(3, 0);
    motor_set(4, 0);
}
