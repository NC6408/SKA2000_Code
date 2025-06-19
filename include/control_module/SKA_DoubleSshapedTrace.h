/**
  ******************************************************************************
  * @file       : SKA_DoubleSshapedTrace.h
  * @brief      : 双S形速度曲线
  * @author     : ZhangYi
  * @version    : None
  * @date       : 2024/10/24
  ******************************************************************************
  */
//

#ifndef SKA2000_OHBC_CONTROLLER_SKA_DOUBLESSHAPEDTRACE_H
#define SKA2000_OHBC_CONTROLLER_SKA_DOUBLESSHAPEDTRACE_H

#include <stdint.h>

#include "SKA_TraceSegment.h"

/**
  * @brief  获得从零速度(加速度、加加速度也为0)到零速度(加速度、加加速度也为0)的轨迹
  * @param  dDist 非负运送距离(m)
  * @param  dMaxVelocity 最大速度(m/s)
  * @param  dMaxAcceleration 最大加速度(m/s^2)
  * @param  dMaxJerk 最大加加速度(m/s^3)
  * @param  ppHead 轨迹头结点指针的地址
  * @return 错误码 {0: 正常}
  */
int8_t SKA_DST_GetZero2ZeroTrace(double dDist, double dMaxVelocity, double dMaxAcceleration, double dMaxJerk,
                               SKA_TraceSegment** ppHead);

/**
  * @brief  生成急停轨迹
  * @param  dStartV 急停时的速度(m/s, 非负)
  * @param  dStartA 急停时的加速度(m/s^2)
  * @param  dMaxAcceleration 最大加速度(m/s^2)
  * @param  dMaxJerk 最大加加速度(m/s^3)
  * @param  ppHead 轨迹头节点指针的地址
  * @return  错误码 {0: 正常}
  */
int8_t SKA_DST_GetUrgencyStopTrace(double dStartV, double dStartA, double dMaxAcceleration, double dMaxJerk,
                                   SKA_TraceSegment** ppHead);

/**
  * @brief  生成从非零速度（加加速度也为0）到零速度(加速度、加加速度也为0)的轨迹
  * @param  dStartV 开始速度(m/s)
  * @param  dStartA 开始加速度(m/s^2)
  * @param  dTargetPos 目标位置(m)
  * @param  dMaxVelocity 最大速度(m/s)
  * @param  dMaxAcceleration 最大加速度(m/s^2)
  * @param  dMaxJerk 最大加加速度(m/s^3)
  * @param  dBufferTime 缓冲时间(s)
  * @param  ppHead 轨迹头节点的地址
  * @return  错误码 {0: 正常}
  */
int8_t SKA_DST_GetNonzero2ZeroTrace(double dStartV, double dStartA, double dTargetPos, double dMaxVelocity, 
                                    double dMaxAcceleration, double dMaxJerk, double dBufferTime, SKA_TraceSegment** ppHead);

#endif //SKA2000_OHBC_CONTROLLER_SKA_DOUBLESSHAPEDTRACE_H
