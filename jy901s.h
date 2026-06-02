#ifndef __JY901S_H
#define __JY901S_H

#include "main.h"

#define JY901S_RX_BUF_SIZE 64  // 接收缓冲区大小，大于单次发送的数据量即可

// 定义JY901s数据结构体
typedef struct {
    float roll;   // 横滚角 (X轴)
    float pitch;  // 俯仰角 (Y轴)
    float yaw;    // 航向角 (Z轴)
} JY901s_Data_t;

// 外部声明结构体变量
extern JY901s_Data_t g_jy901s_data;

// 函数声明
void JY901s_Init(UART_HandleTypeDef *huart);
void JY901s_Receive_Callback(UART_HandleTypeDef *huart, uint16_t Size);

#endif /* __JY901S_H */
