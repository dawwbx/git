#include "jy901s.h"
#include <string.h>

// 全局变量定义
JY901s_Data_t g_jy901s_data = {0};
static UART_HandleTypeDef *p_jy901s_uart = NULL;
static uint8_t rx_buffer[JY901S_RX_BUF_SIZE];

/**
 * @brief  初始化JY901s的DMA接收
 * @param  huart: 绑定的串口句柄指针 (例如 &huart1)
 * @retval 无
 */
void JY901s_Init(UART_HandleTypeDef *huart)
{
    p_jy901s_uart = huart;
    
    // 清空缓冲区
    memset(rx_buffer, 0, JY901S_RX_BUF_SIZE);
    
    // 开启串口空闲中断的DMA接收
    // HAL_UARTEx_ReceiveToIdle_DMA 是HAL库中处理不定长DMA接收的利器
    HAL_UARTEx_ReceiveToIdle_DMA(p_jy901s_uart, rx_buffer, JY901S_RX_BUF_SIZE);
}

/**
 * @brief  数据解析核心函数
 * @param  buf: 接收缓冲区指针
 * @param  len: 当前接收到的有效字节数
 * @retval 无
 */
static void JY901s_Parse(uint8_t *buf, uint16_t len)
{
    // 如果长度不足一帧数据（11字节），直接退出
    if (len < 11) return;

    // 遍历整个缓冲区寻找数据帧头 0x55 0x53
    for (uint16_t i = 0; i <= (len - 11); i++)
    {
        if (buf[i] == 0x55 && buf[i + 1] == 0x53)
        {
            // 校验和验证：前10个字节之和的低8位应等于第11个字节
            uint8_t checksum = 0;
            for (uint8_t j = 0; j < 10; j++)
            {
                checksum += buf[i + j];
            }

            if (checksum == buf[i + 10])
            {
                // 提取原始16位整型数据 (小端模式：低字节在前，高字节在后)
                int16_t roll_raw  = (int16_t)((buf[i + 3] << 8)  | buf[i + 2]);
                int16_t pitch_raw = (int16_t)((buf[i + 5] << 8)  | buf[i + 4]);
                int16_t yaw_raw   = (int16_t)((buf[i + 7] << 8)  | buf[i + 6]);

                // 根据维特智能协议公式转换成实际角度值
                g_jy901s_data.roll  = (float)roll_raw / 32768.0f * 180.0f;
                g_jy901s_data.pitch = (float)pitch_raw / 32768.0f * 180.0f;
                g_jy901s_data.yaw   = (float)yaw_raw / 32768.0f * 180.0f;
            }
            
            // 跳过已处理的一帧长度
            i += 10; 
        }
    }
}

/**
 * @brief  串口空闲中断/DMA满中断回调函数
 * 请在 main.c 的 HAL_UARTEx_RxEventCallback 中调用此函数
 * @param  huart: 触发中断的串口句柄
 * @param  Size: 本次接收到的总字节数
 * @retval 无
 */
void JY901s_Receive_Callback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == p_jy901s_uart->Instance)
    {
        // 调用解析函数
        JY901s_Parse(rx_buffer, Size);

        // 重新开启下一次DMA接收中断（这一步非常关键！）
        HAL_UARTEx_ReceiveToIdle_DMA(p_jy901s_uart, rx_buffer, JY901S_RX_BUF_SIZE);
    }
}
