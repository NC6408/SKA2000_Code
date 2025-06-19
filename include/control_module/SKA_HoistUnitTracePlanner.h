/**
  ******************************************************************************
  * @file       : SKA_HoistUnitTracePlanner.h
  * @brief      : 起升机构的轨迹规划器
  * @author     : ZhangYi
  * @version    : None
  * @date       : 2024/10/24
  ******************************************************************************
  */
//

#ifndef SKA2000_OHBC_CONTROLLER_SKA_HOISTUNITTRACEPLANNER_H
#define SKA2000_OHBC_CONTROLLER_SKA_HOISTUNITTRACEPLANNER_H

#include <stdint.h>
#include <stdbool.h>

#include "SKA_UnshapedTracePlanner.h"

// 起升机构轨迹规划器
typedef struct {
    SKA_UnshapedTracePlanner stUnsPlanner;      // 未整形轨迹规划器
} SKA_HoistUnitTracePlanner;

/**
 * @brief 创建起升机构轨迹规划器
 * @param pHUTP 起升机构轨迹规划器的地址
 * @param dStartPos 开始位置(m)
 * @param dTargetPos 目标位置(m)
 * @param dMaxVelocity 最大速度(m/s)
 * @param dMaxAcceleration 最大加速度(m/s^2)
 * @param dMaxJerk 最大加加速度(m/s^3)
 * @return 错误码 {0: 正常}
 */
int8_t SKA_HUTP_Create(SKA_HoistUnitTracePlanner *pHUTP, double dStartPos, double dTargetPos, 
                       double dMaxVelocity, double dMaxAcceleration, double dMaxJerk);

/**
 * @brief 执行起升机构轨迹规划器
 * @param pTUTP 起升机构轨迹规划器的地址
 * @param dCurTime 当前时间(s)
 * @param pRefPos 存储参考位置(m)的地址
 * @param pRefVel 存储参考速度(m/s)的地址
 * @param pRefAcc 存储参考加速度(m/s^2)的地址
 * @return 错误码 {0: 正常}
 */
int8_t SKA_HUTP_Run(SKA_HoistUnitTracePlanner *pHUTP, double dCurTime,
                    double *pRefPos, double *pRefVel, double *pRefAcc);

/**
 * @brief 获取轨迹总时间(s)
 * @param pHUTP 起升机构轨迹规划器的地址
 * @param pTotalTime 存储总时间的地址
 * @return 错误码 {0: 正常}
 */
int8_t SKA_HUTP_GetTotalTime(SKA_HoistUnitTracePlanner *pHUTP, double *pTotalTime);

/**
  * @brief  修改目标位置
  * @param  pHUTP 轨迹规划器的地址
  * @param  dCurTime 当前时间(s)
  * @param  dNewTargetPos 新的目标位置(m)
  * @return 错误码 {0: 正常}
  */
int8_t SKA_HUTP_ModifyTargetPos(SKA_HoistUnitTracePlanner *pHUTP, double dCurTime, double dNewTargetPos);

/**
  * @brief  急停
  * @param  pTUTP 轨迹规划器的地址
  * @param  dCurTime 当前时间(s)
  * @return 错误码 {0: 正常}
  */
int8_t SKA_HUTP_QuickStop(SKA_HoistUnitTracePlanner *pHUTP, double dCurTime);

/**
 * @brief 预估制动距离
 * @param pHUTP 起升机构轨迹规划器的地址
 * @param dCurTime 当前时间(s)
 * @param pAscBrakeDistance 存储制动距离(m)的地址
 * @return 错误码 {0: 正常}
 */
int8_t SKA_HUTP_GetAscBrakeDistance(SKA_HoistUnitTracePlanner *pHUTP, double dCurTime, double *pAscBrakeDistance);

/**
 * @brief 获取参考轨迹的实际目标位置
 * @param pTUTP 起升机构轨迹规划器的地址
 * @param pRealTargetPos 存储实际目标位置(m)的地址
 * @return 错误码 {0: 正常}
 */
int8_t SKA_HUTP_GetRealTargetPos(SKA_HoistUnitTracePlanner *pHUTP, double *pRealTargetPos);

/**
 * @brief 修改最大速度
 * @param pTUTP 起升机构轨迹规划器的地址
 * @param dCurTime 当前时间(s)
 * @param dNewMaxVelocity 新的最大速度(m/s)
 * @return 错误码 {0: 正常}
 */
int8_t SKA_HUTP_ModifyMaxVelocity(SKA_HoistUnitTracePlanner *pTUTP, double dCurTime, double dNewMaxVelocity);

/**
 * @brief 销毁起升机构轨迹规划器
 * @param pHUTP 起升机构轨迹规划器的地址
 * @return 错误码 {0: 正常}
 */
int8_t SKA_HUTP_Destroy(SKA_HoistUnitTracePlanner *pHUTP);

#endif //SKA2000_OHBC_CONTROLLER_SKA_HOISTUNITTRACEPLANNER_H
