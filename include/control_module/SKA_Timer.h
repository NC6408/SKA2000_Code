/**
  ******************************************************************************
  * @file       : SKA_Timer.h
  * @brief      : 计时器
  * @author     : ZhangYi
  * @version    : None
  * @date       : 2024/10/31
  ******************************************************************************
  */
//

#ifndef SKA2000_OHBC_CONTROLLER_SKA_TIMER_H
#define SKA2000_OHBC_CONTROLLER_SKA_TIMER_H

#include <stdint.h>
#include <time.h>

typedef struct {
    struct timespec stStartTime;
} SKA_Timer;

/**
 * @brief 从0开始计时
 * @param pTimer 计时器指针
 * @return 错误码 {0: 正常}
 */
int8_t SKA_Timer_Start(SKA_Timer *pTimer);

/*
 * @brief 获取当前时刻
 * @param pTimer 计时器指针
 * @param pCurTime 存储当前时刻(s)的地址
 * @return 错误码 {0: 正常}
 */
int8_t SKA_Timer_GetTime(SKA_Timer *pTimer, double *pCurTime);

#endif //SKA2000_OHBC_CONTROLLER_SKA_TIMER_H