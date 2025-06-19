/**
  ******************************************************************************
  * @file       : SKA_TravelUnitTrackingController.h
  * @brief      : 运行机构跟踪控制器
  * @author     : ZhangYi
  * @version    : None
  * @date       : 2024/10/24
  ******************************************************************************
  */
//

#ifndef SKA2000_OHBC_CONTROLLER_SKA_TRAVELUNITTRACKINGCONTROLLER_H
#define SKA2000_OHBC_CONTROLLER_SKA_TRAVELUNITTRACKINGCONTROLLER_H

#include <stdbool.h>
#include <stdint.h>

#include "SKA_Saturation.h"
#include "SKA_RateLimiter.h"
#include "SKA_InertialFilter.h"

// 跟踪控制器
typedef struct {
    bool bActivePosCtrl;                    // 激活闭环定位控制
    double dTs;                             // 采样周期(s)

    SKA_InertialFilter stDelayer;           // 时间延迟补偿器
    double dMaxTrackErr;                    // 最大跟踪误差(m)
    SKA_RateLimiter stTrackErrRL;           // 跟踪误差的速率限制器
    SKA_InertialFilter stTrackErrIF;        // 跟踪误差的惯性滤波器

    double dKp;                             // 比例反馈系数
    double dKi;                             // 积分反馈系数
    double dMaxVi;                          // 积分输出饱和的阈值(m/s)
    double dPosDeadZone;                    // 定位控制的死区半径(m)
    bool bOpenIntegration;                  // 打开积分环节的标志
    double dTrackErrInteg;                  // 跟踪偏差积分

    double dMaxVelocity;                    // 最大速度(m/s)
    SKA_RateLimiter stVelocityRL;           // 输出速度的速率限制器

    double dMaxFreq;                        // 最大输出频率(Hz)
    double dMinFreq;                        // 最小能动频率(Hz)
    double dFreqCompenSoft;                 // 频率死区补偿的软化因子
    
    double dOriTrackError;                  // 原始跟踪偏差
    double dFilTrackError;                  // 滤波后的跟踪偏差
    double dVp;                             // 比例环节输出
    double dVi;                             // 积分环节输出
} SKA_TravelUnitTrackingController;

/**
 * @brief 初始化跟踪控制器
 * @param pTUTC 跟踪控制器地址
 * @param bActivePosCtrl 激活闭环定位控制
 * @param dTs 采样周期(s)
 * @param dDelayTime 系统延迟时间(s)
 * @param dStartPos 开始位置(m)
 * @param dMaxTrackErr 最大跟踪误差(m)
 * @param dMaxTrackErrRate 最大跟踪误差变化率(m/s)
 * @param dTrackErrFilTc 跟踪误差的滤波时间常数(s)
 * @param dKp 比例反馈系数
 * @param dKi 积分反馈系数
 * @param dMaxVi 积分输出饱和的阈值(m/s)
 * @param dPosDeadZone 定位控制的死区半径(m)
 * @param dMaxVelocity 最大速度(m/s)
 * @param dMaxAcceleration 最大加速度(m/s^2)
 * @param dMaxFreq 最大输出频率(Hz)
 * @param dMinFreq 最小能动频率(Hz)
 * @param dFreqCompenSoft 频率死区补偿的软化因子([0,1])
 * @return 错误码 {0: 正常}
 */
int8_t SKA_TUTC_Init(SKA_TravelUnitTrackingController *pTUTC, bool bActivePosCtrl, double dTs, double dDelayTime, double dStartPos,
                     double dMaxTrackErr, double dMaxTrackErrRate, double dTrackErrFilTc,
                     double dKp, double dKi, double dMaxVi, double dPosDeadZone,
                     double dMaxVelocity, double dMaxAcceleration, double dMaxFreq, double dMinFreq, double dFreqCompenSoft);

/**
  * @brief  执行运行机构跟踪控制器
  * @param pTUTC 跟踪控制器地址
  * @param dRefPos 参考位置(m)
  * @param dRefVel 参考速度(m/s)
  * @param dRefAcc 参考加速度(m/s^2)
  * @param dCurPos 当前位置(m)
  * @param pOutputVel 输出速度(m/s)
  * @param pOutputFreq 输出频率(Hz)
  * @param pDirection 输出方向
  * @return 错误码 {0: 正常}
  */
int8_t SKA_TUTC_Run(SKA_TravelUnitTrackingController *pTUTC, 
                    double dRefPos, double dRefVel, double dRefAcc, double dCurPos,
                    double *pOutputVel, double *pOutputFreq, int8_t *pDirection);

/**
 * @brief 获取跟踪控制器的位置跟踪误差
 * @param pTUTC 跟踪控制器地址
 * @param pPosTrackError 位置跟踪误差
 * @return 错误码 {0: 正常}
 */
int8_t SKA_TUTC_GetPosTrackError(SKA_TravelUnitTrackingController *pTUTC, double *pPosTrackError);

#endif //SKA2000_OHBC_CONTROLLER_SKA_TRAVELUNITTRACKINGCONTROLLER_H
