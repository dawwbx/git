//维特智能陀螺仪已经集成模块化，只需读取串口数据再做运算即可
ext
// #define RX_BUF_SIZE 90
// uint8_t rx_buffer[RX_BUF_SIZE];

// volatile float roll_deg = 0.0f;
// volatile float pitch_deg = 0.0f;
// volatile float yaw_deg = 0.0f;
// uint8_t idx = 0;

// void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
// {
//     if (huart->Instance == USART2) {
//             // check headers at positions 22 and 23
//             if (rx_buffer[22] == 0x55 && rx_buffer[23] == 0x53) {
//                 uint8_t *d = &rx_buffer[24];
//                 uint16_t roll  = (d[1] << 8) | d[0];
//                 uint16_t pitch = (d[3] << 8) | d[2];
//                 uint16_t yaw   = (d[5] << 8) | d[4];
//                 // convert to degrees
//                 roll_deg  = ((int16_t)roll)  / 32768.0f * 180.0f;
//                 pitch_deg = ((int16_t)pitch) / 32768.0f * 180.0f;
//                 yaw_deg   = ((int16_t)yaw)   / 32768.0f * 180.0f;
//             }
//             // reset index regardless
//             idx = 0;
//         }
//         // restart UART Rx for next byte
// 	HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rx_buffer, RX_BUF_SIZE);
// }

//使用方法：
// 1.添加变量
// 2.将void HAL_UARTEx_RxEventCallback的内容做微修改(串口通道等)再添加
// 3.最终的可用数据为roll_deg、pitch_deg、yaw_deg,它们表示各个方向的偏移量