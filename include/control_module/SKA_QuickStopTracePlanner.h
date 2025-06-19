/**
  ******************************************************************************
  * @file       : SKA_QuickStopTracePlanner.h
  * @brief      : 无防摇急停轨迹规划器
  * @author     : ZhangYi
  * @version    : None
  * @date       : 2024/11/05
  ******************************************************************************
  */
//

#ifndef SKA2000_OHBC_CONTROLLER_SKA_QUICKSTOPTRACEPLANNER_H
#define SKA2000_OHBC_CONTROLLER_SKA_QUICKSTOPTRACEPLANNER_H

#include <stdint.h>

#include "SKA_TraceSegment.h"

// 无防摇急停轨迹规划器
typedef struct {
    double dStartTime;               // 开始时刻(s)
    double dStartPos;                // 开始位置(m)
    double dStartVel;                // 开始速度(m/s)
    double dStartAcc;                // 开始加速度(m/s^2)
    double dMaxAcceleration;         // 最大加速度(m/s^2)
    double dMaxJerk;                 // 最大加加速度(m/s^3)
    SKA_TraceSegment *pTraceHead;    // 轨迹头结点指针
} SKA_QuickStopTracePlanner;

/**
 * @brief 创建无防摇急停轨迹规划器
 * @param pQSTP 无防摇急停轨迹规划器的地址
 * @param dStartTime 开始时刻(s)
 * @param dStartPos 开始位置(m)
 * @param dStartVel 开始速度(m/s)
 * @param dMaxAccn 最大加速度(m/s^2)
 * @return 错误码 {0: 正常}
 */
int8_t SKA_QSTP_Create(SKA_QuickStopTracePlanner *pQSTP, double dStartTime, double dStartPos, double dStartVel, double dMaxAcc);

/**
 * @brief 运行无防摇急停轨迹规划器
 * @param pQSTP 无防摇急停轨迹规划器的地址
 * @param dCurTime 当前时间(s)
 * @param pRefPos 存储参考位置(m)的地址
 * @param pRefVel 存储参考速度(m/s)的地址
 * @param pRefAcc 存储参考加速度(m/s^2)的地址
 * @return 错误码 {0: 正常}
 */
int8_t SKA_QSTP_Run(SKA_QuickStopTracePlanner *pQSTP, double dCurTime, double *pRefPos, double *pRefVel, double *pRefAcc);

/**
 * @brief 获取无防摇急停轨迹规划器总时间
 * @param pQSTP 无防摇急停轨迹规划器的地址
 * @param pTotalTime 存储总时间(s)的地址
 * @return 错误码 {0: 正常}
 */
int8_t SKA_QSTP_GetTotalTime(const SKA_QuickStopTracePlanner *pQSTP, double *pTotalTime);

/**
 * @brief 获取无防摇急停轨迹规划器目标位置
 * @param pQSTP 无防摇急停轨迹规划器的地址
 * @param pTargetPos 存储目标位置(m)的地址
 * @return 锝错误码 {0: 正常}
 */
int8_t SKA_QSTP_GetTargetPos(const SKA_QuickStopTracePlanner *pQSTP, double *pTargetPos);

/**
 * @brief 销毁无防摇急停轨迹规划器
 * @param pQSTP 无防摇急停轨迹规划器的地址
 * @return 错误码 {0: 正常}
 */
int8_t SKA_QSTP_Destroy(SKA_QuickStopTracePlanner *pQSTP);

#endif //SKA2000_OHBC_CONTROLLER_SKA_QUICKSTOPTRACEPLANNER_H