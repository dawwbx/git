#ifndef __time_H
#define __time_H
#include "time.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
typedef struct 
{
    //当前状态，上一刻状态，上升沿状态，下降沿状态
    bool NowKey,lastKey,RisingEdge,FallingEdge;

}RM_Key;


typedef struct 
{
	uint32_t lastTime;//上一时刻
	RM_Key key;//信号类

}RM_StaticTime;


void UpKey(RM_Key *RMkey,bool key);//更新信号
bool GetRisingKey(RM_Key *RMkey);//获取上升沿信号
bool GetFallingKey(RM_Key *RMkey);//获取下降沿信号
void UpLastTime(RM_StaticTime *starttime);//更新上一时刻
bool ISOne(RM_StaticTime *starttime,uint32_t targetTime);//判断单次信号
bool ISGL(RM_StaticTime *starttime,uint32_t targetTime);//判断连续信号
bool ISDir(RM_StaticTime *starttime,uint32_t dirTime);//定时器死亡
bool ISFromOne(RM_StaticTime *starttime,uint64_t nowTime, uint64_t targetTime);//自定义判断单次信号
bool ISFromGL(RM_StaticTime *starttime,uint64_t nowTime, uint64_t targetTime);//自定义判断连续信号






#endif






