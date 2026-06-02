#include "ZDT_42cotrl.h"

extern osSemaphoreId_t RxSemHandle;
float pos = 0.0f,
Motor_Cur_Pos = 0.0f,
Motor_speed = 0.0f;
extern volatile uint8_t rxCmd[25];
 uint8_t cmd[20] = {0};
void Emm_V5_Getposition(uint8_t addr)
{
//	uint8_t cmd[3] = {0};
	cmd[0] =  addr;
	cmd[1] =  0x36;                       // 位置功能码
  cmd[2] =  0x6B; 											// 校验字节
	 // 发送命令
	HAL_UART_Transmit_DMA(&huart, cmd, 3);
}

void Emm_V5_GetSpeed(uint8_t addr)
{
//	uint8_t cmd[3] = {0};
	cmd[0] =  addr;
	cmd[1] =  0x35;                       // 速度功能码
  cmd[2] =  0x6B; 											// 校验字节
	 // 发送命令
	HAL_UART_Transmit_DMA(&huart, cmd,3);
}


void Emm_V5_Query_State(uint8_t addr, uint8_t func_code)
{
    static uint8_t tx_cmd[3]; // 发送缓冲区

    // 1. 准备指令
    tx_cmd[0] = addr;
    tx_cmd[1] = func_code;
    tx_cmd[2] = 0x6B;

    // 2. 在发送前，先清除一下信号量（防止有旧的误触发）
    osSemaphoreAcquire(RxSemHandle, 0); 
    
    // 3. 发送查询指令
    if (HAL_UART_Transmit_DMA(&huart1, tx_cmd, 3) != HAL_OK) return;

    // 4. 等待接收完成信号 (核心！)
    // osWaitForever 表示死等，建议设置 10ms 超时，防止电机掉线导致死机
    if (osSemaphoreAcquire(RxSemHandle, 10) == osOK)
    {
        // 5. 收到数据了，开始校验和解析
        // 校验：地址对不对？功能码是不是我发的那个？
        if (rxCmd[0] == addr && rxCmd[1] == func_code)
        {
            if (func_code == 0x36) // 解析位置
            {
                uint32_t raw_pos = ((uint32_t)rxCmd[3] << 24) |
                                   ((uint32_t)rxCmd[4] << 16) |
                                   ((uint32_t)rxCmd[5] << 8)  |
                                   rxCmd[6];
                
                float temp_pos = (float)raw_pos * 360.0f / 65536.0f;
                // 处理符号位
                if (rxCmd[2]) temp_pos = -temp_pos;
                
                Motor_Cur_Pos = temp_pos; // 更新全局变量
            }
            else if (func_code == 0x35) // 解析速度
            {
                uint16_t raw_speed = ((uint16_t)rxCmd[3] << 8) | rxCmd[4];
                int16_t temp_speed = (int16_t)raw_speed;
                
                // 处理符号位
                if (rxCmd[2]) temp_speed = -temp_speed;
                
                Motor_speed = temp_speed; // 更新全局变量
            }
        }
    }
    else
    {
        // 超时了（超过10ms没收到回复）
        // 这里可以做一些错误处理，比如记录丢包率
    }
}
/**
  * @brief    将当前位置清零
  * @param    addr  ：电机地址
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void Emm_V5_Reset_CurPos_To_Zero(uint8_t addr)
{
  uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0x0A;                       // 功能码
  cmd[2] =  0x6D;                       // 辅助码
  cmd[3] =  0x6B;                       // 校验字节
  
  // 发送命令
	HAL_UART_Transmit_DMA(&huart, (uint8_t *)cmd, 4);
}

/**
  * @brief    解除堵转保护
  * @param    addr  ：电机地址
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void Emm_V5_Reset_Clog_Pro(uint8_t addr)
{
  uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0x0E;                       // 功能码
  cmd[2] =  0x52;                       // 辅助码
  cmd[3] =  0x6B;                       // 校验字节
  
  // 发送命令
  HAL_UART_Transmit_DMA(&huart, (uint8_t *)cmd, 4);
}

/**
  * @brief    读取系统参数
  * @param    addr  ：电机地址
  * @param    s     ：系统参数类型
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void Emm_V5_Read_Sys_Params(uint8_t addr, SysParams_t s)
{
  uint8_t i = 0;
  uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[i] = addr; ++i;                   // 地址

  switch(s)                             // 功能码
  {
    case S_VER  : cmd[i] = 0x1F; ++i; break;
    case S_RL   : cmd[i] = 0x20; ++i; break;
    case S_PID  : cmd[i] = 0x21; ++i; break;
    case S_VBUS : cmd[i] = 0x24; ++i; break;
    case S_CPHA : cmd[i] = 0x27; ++i; break;
    case S_ENCL : cmd[i] = 0x31; ++i; break;
    case S_TPOS : cmd[i] = 0x33; ++i; break;
    case S_VEL  : cmd[i] = 0x35; ++i; break;
    case S_CPOS : cmd[i] = 0x36; ++i; break;
    case S_PERR : cmd[i] = 0x37; ++i; break;
    case S_FLAG : cmd[i] = 0x3A; ++i; break;
    case S_ORG  : cmd[i] = 0x3B; ++i; break;
    case S_Conf : cmd[i] = 0x42; ++i; cmd[i] = 0x6C; ++i; break;
    case S_State: cmd[i] = 0x43; ++i; cmd[i] = 0x7A; ++i; break;
    default: break;
  }

  cmd[i] = 0x6B; ++i;                   // 校验字节
  
  // 发送命令
  HAL_UART_Transmit_DMA(&huart, (uint8_t *)cmd, i);
}

/**
  * @brief    修改开环/闭环控制模式
  * @param    addr     ：电机地址
  * @param    svF      ：是否存储标志，false为不存储，true为存储
  * @param    ctrl_mode：控制模式（对应屏幕上的P_Pul菜单），0是关闭脉冲输入引脚，1是开环模式，2是闭环模式，3是让En端口复用为多圈限位开关输入引脚，Dir端口复用为到位输出高电平功能
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void Emm_V5_Modify_Ctrl_Mode(uint8_t addr, bool svF, uint8_t ctrl_mode)
{
  uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0x46;                       // 功能码
  cmd[2] =  0x69;                       // 辅助码
  cmd[3] =  svF;                        // 是否存储标志，false为不存储，true为存储
  cmd[4] =  ctrl_mode;                  // 控制模式（对应屏幕上的P_Pul菜单），0是关闭脉冲输入引脚，1是开环模式，2是闭环模式，3是让En端口复用为多圈限位开关输入引脚，Dir端口复用为到位输出高电平功能
  cmd[5] =  0x6B;                       // 校验字节
  
  // 发送命令
  HAL_UART_Transmit_DMA(&huart, (uint8_t *)cmd, 6);
}

/**
  * @brief    使能信号控制
  * @param    addr  ：电机地址
  * @param    state ：使能状态     ，true为使能电机，false为关闭电机
  * @param    snF   ：多机同步标志 ，false为不启用，true为启用
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void Emm_V5_En_Control(uint8_t addr, bool state, bool snF)
{
  uint8_t cmd[6] = {0};
  
  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0xF3;                       // 功能码
  cmd[2] =  0xAB;                       // 辅助码
  cmd[3] =  (uint8_t)state;             // 使能状态
  cmd[4] =  snF;                        // 多机同步运动标志
  cmd[5] =  0x6B;                       // 校验字节
  
  // 发送命令
  HAL_UART_Transmit_DMA(&huart, (uint8_t *)cmd, 6);
}

/**
  * @brief    速度模式
  * @param    addr：电机地址
  * @param    dir ：方向       ，0为CW，其余值为CCW
  * @param    vel ：速度       ，范围0 - 5000RPM
  * @param    acc ：加速度     ，范围0 - 255，注意：0是直接启动
  * @param    snF ：多机同步标志，false为不启用，true为启用
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void Emm_V5_Vel_Control(uint8_t addr, int16_t vel, uint8_t acc, bool snF)
{
		uint8_t  cal_dir;
    uint16_t abs_vel;

    if (vel < 0)
    {
        cal_dir = 1;                // 负数：方向置为 1 (反转)
        abs_vel = (uint16_t)(-vel); // 取绝对值用于发送
    }
    else
    {
        cal_dir = 0;                // 正数或0：方向置为 0 (正转)
        abs_vel = (uint16_t)vel;    // 直接强转
    }
    // 使用 static 变量记录上一次发送的参数
    // 初始化为不可能的值，确保第一次一定会发送
    static uint8_t  last_dir = 0xFF;
    static uint16_t last_vel = 0xFFFF;
    static uint8_t  last_acc = 0xFF;
    static bool     last_snF = 0xFF;
    // 1. 检查数据是否有变化
    // 如果所有参数都和上次一样，直接退出，不占用总线，也不浪费CPU
    if (cal_dir == last_dir && abs_vel == last_vel && acc == last_acc && snF == last_snF)
    {
        return; 
    }

     if (huart1.gState != HAL_UART_STATE_READY) 
    {
        return; // 发送通道忙，退出等待下一次
    }
    // 3. 只有串口空闲且数据有变化时，才执行发送逻辑
    cmd[0] =  addr;                       
    cmd[1] =  0xF6;                       
    cmd[2] =  cal_dir;                        
    cmd[3] =  (uint8_t)(abs_vel >> 8);        
    cmd[4] =  (uint8_t)(abs_vel >> 0);        
    cmd[5] =  acc;                        
    cmd[6] =  snF;                        
    cmd[7] =  0x6B;                       
 
    HAL_UART_Transmit_DMA(&huart, cmd, 8);

    // 4. 发送成功后，更新“上一次”的记录
    last_dir = cal_dir;
    last_vel = abs_vel;
    last_acc = acc;
    last_snF = snF;
}

/**
  * @brief    位置模式
  * @param    addr：电机地址
  * @param    dir ：方向        ，0为CW，其余值为CCW
  * @param    vel ：速度(RPM)   ，范围0 - 5000RPM
  * @param    acc ：加速度      ，范围0 - 255，注意：0是直接启动
  * @param    clk ：脉冲数      ，范围0- (2^32 - 1)个
  * @param    raF ：相位/绝对标志，false为相对运动，true为绝对值运动
  * @param    snF ：多机同步标志 ，false为不启用，true为启用
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void Emm_V5_Pos_Control(uint8_t addr, uint16_t vel, uint8_t acc, int32_t clk, bool raF, bool snF)
{
    // ============================================================
    // 1. 数据预处理：将带符号脉冲数 clk 解析为 方向(dir) 和 绝对值(abs_clk)
    // ============================================================
    uint8_t  cal_dir;
    uint32_t abs_clk;

    if (clk < 0)
    {
        cal_dir = 1;                // 负数：方向置为 1 (反转)
        abs_clk = (uint32_t)(-clk); // 取绝对值用于发送
    }
    else
    {
        cal_dir = 0;                // 正数或0：方向置为 0 (正转)
        abs_clk = (uint32_t)clk;    // 直接强转
    }

    // ============================================================
    // 2. 静态变量记录上一次发送的参数 (实现去重过滤)
    // ============================================================
    static uint16_t last_vel     = 0xFFFF;
    static uint8_t  last_acc     = 0xFF;
    static uint8_t  last_dir     = 0xFF;
    static uint32_t last_abs_clk = 0xFFFFFFFF;
    static bool     last_raF     = 0xFF;
    static bool     last_snF     = 0xFF;

    // 检查数据是否有变化
    // 注意：如果是“相对运动(raF=0)”，去重过滤可能会导致想连续走两段相同距离时无效
    // 如果你的场景主要是“绝对位置控制(raF=1)”，这个过滤非常完美
    if (vel == last_vel && acc == last_acc && cal_dir == last_dir && 
        abs_clk == last_abs_clk && raF == last_raF && snF == last_snF)
    {
        return; 
    }

    // ============================================================
    // 3. 硬件状态检查 (只检查发送状态)
    // ============================================================
    if (huart1.gState != HAL_UART_STATE_READY) 
    {
        return; // 串口忙，退出
    }

    // ============================================================
    // 4. 装载命令
    // ============================================================
    cmd[0]  =  addr;                      // 地址
    cmd[1]  =  0xFD;                      // 功能码
    cmd[2]  =  cal_dir;                   // 使用计算出的方向
    cmd[3]  =  (uint8_t)(vel >> 8);       // 速度高8位
    cmd[4]  =  (uint8_t)(vel >> 0);       // 速度低8位 
    cmd[5]  =  acc;                       // 加速度
    cmd[6]  =  (uint8_t)(abs_clk >> 24);  // 脉冲数使用绝对值
    cmd[7]  =  (uint8_t)(abs_clk >> 16);      
    cmd[8]  =  (uint8_t)(abs_clk >> 8);       
    cmd[9]  =  (uint8_t)(abs_clk >> 0);       
    cmd[10] =  raF;                       // 相位/绝对标志
    cmd[11] =  snF;                       // 多机同步标志
    cmd[12] =  0x6B;                      // 校验字节
  
    // ============================================================
    // 5. 发送与更新
    // ============================================================
    HAL_UART_Transmit_DMA(&huart1, cmd, 13);

    // 更新记录
    last_vel     = vel;
    last_acc     = acc;
    last_dir     = cal_dir;
    last_abs_clk = abs_clk;
    last_raF     = raF;
    last_snF     = snF;
}
/**
  * @brief    立即停止（所有控制模式都通用）
  * @param    addr  ：电机地址
  * @param    snF   ：多机同步标志，false为不启用，true为启用
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void Emm_V5_Stop_Now(uint8_t addr, bool snF)
{
  uint8_t cmd[5] = {0};
  
  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0xFE;                       // 功能码
  cmd[2] =  0x98;                       // 辅助码
  cmd[3] =  snF;                        // 多机同步运动标志
  cmd[4] =  0x6B;                       // 校验字节
  
  // 发送命令
  HAL_UART_Transmit_DMA(&huart, (uint8_t *)cmd, 5);
}

/**
  * @brief    多机同步运动
  * @param    addr  ：电机地址
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void Emm_V5_Synchronous_motion(uint8_t addr)
{
  uint8_t cmd[4] = {0};
  
  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0xFF;                       // 功能码
  cmd[2] =  0x66;                       // 辅助码
  cmd[3] =  0x6B;                       // 校验字节
  
  // 发送命令
  HAL_UART_Transmit_DMA(&huart, (uint8_t *)cmd, 4);
}

/**
  * @brief    设置单圈回零的零点位置
  * @param    addr  ：电机地址
  * @param    svF   ：是否存储标志，false为不存储，true为存储
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void Emm_V5_Origin_Set_O(uint8_t addr, bool svF)
{
   uint8_t cmd[5] = {0};
  
  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0x93;                       // 功能码
  cmd[2] =  0x88;                       // 辅助码
  cmd[3] =  svF;                        // 是否存储标志，false为不存储，true为存储
  cmd[4] =  0x6B;                       // 校验字节
  
  // 发送命令
  HAL_UART_Transmit_DMA(&huart, (uint8_t *)cmd, 5);
}

/**
  * @brief    修改回零参数
  * @param    addr  ：电机地址
  * @param    svF   ：是否存储标志，false为不存储，true为存储
  * @param    o_mode ：回零模式，0为单圈就近回零，1为单圈方向回零，2为多圈无限位碰撞回零，3为多圈有限位开关回零
  * @param    o_dir  ：回零方向，0为CW，其余值为CCW
  * @param    o_vel  ：回零速度，单位：RPM（转/分钟）
  * @param    o_tm   ：回零超时时间，单位：毫秒
  * @param    sl_vel ：无限位碰撞回零检测转速，单位：RPM（转/分钟）
  * @param    sl_ma  ：无限位碰撞回零检测电流，单位：Ma（毫安）
  * @param    sl_ms  ：无限位碰撞回零检测时间，单位：Ms（毫秒）
  * @param    potF   ：上电自动触发回零，false为不使能，true为使能
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void Emm_V5_Origin_Modify_Params(uint8_t addr, bool svF, uint8_t o_mode, uint8_t o_dir, uint16_t o_vel, uint32_t o_tm, uint16_t sl_vel, uint16_t sl_ma, uint16_t sl_ms, bool potF)
{
  uint8_t cmd[32] = {0};
  
  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0x4C;                       // 功能码
  cmd[2] =  0xAE;                       // 辅助码
  cmd[3] =  svF;                        // 是否存储标志，false为不存储，true为存储
  cmd[4] =  o_mode;                     // 回零模式，0为单圈就近回零，1为单圈方向回零，2为多圈无限位碰撞回零，3为多圈有限位开关回零
  cmd[5] =  o_dir;                      // 回零方向
  cmd[6]  =  (uint8_t)(o_vel >> 8);     // 回零速度(RPM)高8位字节
  cmd[7]  =  (uint8_t)(o_vel >> 0);     // 回零速度(RPM)低8位字节 
  cmd[8]  =  (uint8_t)(o_tm >> 24);     // 回零超时时间(bit24 - bit31)
  cmd[9]  =  (uint8_t)(o_tm >> 16);     // 回零超时时间(bit16 - bit23)
  cmd[10] =  (uint8_t)(o_tm >> 8);      // 回零超时时间(bit8  - bit15)
  cmd[11] =  (uint8_t)(o_tm >> 0);      // 回零超时时间(bit0  - bit7 )
  cmd[12] =  (uint8_t)(sl_vel >> 8);    // 无限位碰撞回零检测转速(RPM)高8位字节
  cmd[13] =  (uint8_t)(sl_vel >> 0);    // 无限位碰撞回零检测转速(RPM)低8位字节 
  cmd[14] =  (uint8_t)(sl_ma >> 8);     // 无限位碰撞回零检测电流(Ma)高8位字节
  cmd[15] =  (uint8_t)(sl_ma >> 0);     // 无限位碰撞回零检测电流(Ma)低8位字节 
  cmd[16] =  (uint8_t)(sl_ms >> 8);     // 无限位碰撞回零检测时间(Ms)高8位字节
  cmd[17] =  (uint8_t)(sl_ms >> 0);     // 无限位碰撞回零检测时间(Ms)低8位字节
  cmd[18] =  potF;                      // 上电自动触发回零，false为不使能，true为使能
  cmd[19] =  0x6B;                      // 校验字节
  
  // 发送命令
  HAL_UART_Transmit_DMA(&huart, (uint8_t *)cmd, 20);
}

/**
  * @brief    触发回零
  * @param    addr   ：电机地址
  * @param    o_mode ：回零模式，0为单圈就近回零，1为单圈方向回零，2为多圈无限位碰撞回零，3为多圈有限位开关回零
  * @param    snF   ：多机同步标志，false为不启用，true为启用
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void Emm_V5_Origin_Trigger_Return(uint8_t addr, uint8_t o_mode, bool snF)
{
  uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0x9A;                       // 功能码
  cmd[2] =  o_mode;                     // 回零模式，0为单圈就近回零，1为单圈方向回零，2为多圈无限位碰撞回零，3为多圈有限位开关回零
  cmd[3] =  snF;                        // 多机同步运动标志，false为不启用，true为启用
  cmd[4] =  0x6B;                       // 校验字节
  
  // 发送命令
  HAL_UART_Transmit_DMA(&huart, (uint8_t *)cmd, 5);
}

/**
  * @brief    强制中断并退出回零
  * @param    addr  ：电机地址
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void Emm_V5_Origin_Interrupt(uint8_t addr)
{
  uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0x9C;                       // 功能码
  cmd[2] =  0x48;                       // 辅助码
  cmd[3] =  0x6B;                       // 校验字节
  
  // 发送命令
  HAL_UART_Transmit_DMA(&huart, (uint8_t *)cmd, 5);
}

