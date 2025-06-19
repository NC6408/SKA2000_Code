/**
  ******************************************************************************
  * @file       : SKA_Timer.c
  * @brief      : 计时器
  * @author     : ZhangYi
  * @version    : None
  * @date       : 2024/10/31
  ******************************************************************************
  */
//

#define _POSIX_C_SOURCE 200809L // 启用CLOCK_MONOTONIC
#include <stddef.h> 
#include <stdio.h>
#include <stdlib.h>

#include "SKA_Timer.h"
#include "SKA_Logger.h"

int8_t SKA_Timer_Start(SKA_Timer *pTimer) {
    /******************** 函数参数合法性检验 ********************/
    if (pTimer == NULL) {
        zlog_error(pCategory, "Invalid parameters of SKA_Timer_Start().");
        return -1;
    }

    /******************** 计时器初始化 ********************/
    // CLOCK_MONOTONIC时钟用于获取系统的单调时间, 从某个固定点（通常是系统启动时间）开始的时间，不受系统时间调整的影响
    if (clock_gettime(CLOCK_MONOTONIC, &pTimer->stStartTime) != 0)
    {
        zlog_error(pCategory, "Failed to get current time");
        exit(EXIT_FAILURE);
    }

    return 0;
}

int8_t SKA_Timer_GetTime(SKA_Timer *pTimer, double *pCurTime) {
    /******************** 函数参数合法性检验 ********************/
    if (pTimer == NULL || pCurTime == NULL) {
        zlog_error(pCategory, "Invalid parameters of SKA_Timer_GetTime().");
        return -1;
    }

    /******************** 获取当前时刻 ********************/
    struct timespec stCurTime;
    if (clock_gettime(CLOCK_MONOTONIC, &stCurTime) != 0)
    {
        zlog_error(pCategory, "Failed to get current time");
        exit(EXIT_FAILURE);
    }
    // 计算时间差
    *pCurTime = (stCurTime.tv_sec - pTimer->stStartTime.tv_sec) + (stCurTime.tv_nsec - pTimer->stStartTime.tv_nsec) / 1e9;


    return 0;
}