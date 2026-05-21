#include "time.h"
#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <string.h>

//更新信号
inline void UpKey(RM_Key *RMkey,bool key)
{
	RMkey->lastKey = RMkey->NowKey;//更新上一次信号
	RMkey->NowKey = key;//获取当前信号
	RMkey->RisingEdge = RMkey->FallingEdge = false;//情况状态
	if(RMkey->NowKey - RMkey->lastKey == 1)RMkey->FallingEdge = true;//设置上升沿信号
	if(RMkey->lastKey - RMkey->NowKey == 1)RMkey->RisingEdge = true;//设置下降沿信号
}
//获取上升沿信号
inline bool GetRisingKey(RM_Key *RMkey)
{
	return RMkey->RisingEdge;//上升沿信号
}
//获取下降沿信号
inline bool GetFallingKey(RM_Key *RMkey)
{
	return RMkey->FallingEdge;//下降沿信号
}

//更新上一时刻
void UpLastTime(RM_StaticTime *starttime)
{
	starttime->lastTime = HAL_GetTick();
}

//判断单次信号
inline bool ISOne(RM_StaticTime *starttime,uint32_t targetTime)
{
	UpKey(&starttime->key,HAL_GetTick() % targetTime);//输入更新状态
	if(GetRisingKey(&starttime->key))return true;
	return false;
}

//判断连续信号
inline bool ISGL(RM_StaticTime *starttime,uint32_t targetTime)
{
	UpKey(&starttime->key,(HAL_GetTick() % targetTime / (float)targetTime) * 100 > 100 - 50/*百分比占比*/);//输入更新状态
	return starttime->key.NowKey;
}

//定时器死亡
inline bool ISDir(RM_StaticTime *starttime,uint32_t dirTime)
{
if(HAL_GetTick() - starttime->lastTime >= dirTime)return true;
return false;
}

//自定义判断单次信号
inline bool ISFromOne(RM_StaticTime *starttime,uint64_t nowTime, uint64_t targetTime)
{
	UpKey(&starttime->key,nowTime % targetTime);//输入更新状态
	if (GetRisingKey(&starttime->key))return true;
	return false;
}

//自定义判断连续信号
inline bool ISFromGL(RM_StaticTime *starttime,uint64_t nowTime, uint64_t targetTime)
{
	UpKey(&starttime->key,(nowTime % targetTime / (float)targetTime) * 100 > 100 - 50);//输入更新状态
	return starttime->key.NowKey;
}

//使用方法
//申明初始化结构体
//RM_StaticTime myTimer = {0}; // 初始化定时器结构体

//无堵塞每1000ms触发一次
//if (ISOne(&myTimer, 1000)) {  // 每1000ms触发一次
    // 执行代码（如LED闪烁）
//}

//无堵塞每2000ms周期中，后1000ms返回true,前1000ms返回false
// if (ISGL(&myTimer, 2000)) {  // 2000ms周期中，后1000ms返回true
//     // 执行代码（如电机开启）
// }else{}// 执行代码（如电机关闭）

//超时报警
//  if(HAL_GPIO_ReadPin(BTN_GPIO_Port, BTN_Pin)) {
//         UpLastTime(&timer);  // 按键按下时记录时间
//     }
//     if(ISDir(&timer, 3000)) {  // 3秒内无操作
//         System_Sleep();  // 进入休眠
//     }








