/**
  ******************************************************************************
  * @file       : SKA_TravelUnitTracePlanner.h
  * @brief      : 运行机构轨迹规划器
  * @author     : ZhangYi
  * @version    : None
  * @date       : 2024/10/24
  ******************************************************************************
  */
//

#ifndef SKA2000_OHBC_CONTROLLER_SKA_TRAVELUNITTRACEPLANNER_H
#define SKA2000_OHBC_CONTROLLER_SKA_TRAVELUNITTRACEPLANNER_H

#include <stdint.h>

#include "SKA_UnshapedTracePlanner.h"
#include "SKA_InputShaper.h"
#include "SKA_QuickStopTracePlanner.h"

// 运行机构轨迹规划器
typedef struct
{
  bool bActiveSwayCtrl;                  // 激活防摇控制
  bool bIsQuickStopping;                 // 正在无防摇急停标志
  SKA_InputShaper stShaper;              // 输入整形器
  SKA_UnshapedTracePlanner stUnsPlanner; // 未整形轨迹规划器
  double dQuickStopMaxAcc;               // 无防摇急停最大加速度(m/s^2)
  SKA_QuickStopTracePlanner stQSTP;      // 无防摇急停轨迹规划器

  bool bIsUseWaypoints;
  uint16_t nObstacleNum;
  uint16_t nWaypointsNum;
  bool bActiveHorizontalMode;
} SKA_TravelUnitTracePlanner;

/**
 * @brief 创建运行机构轨迹规划器
 * @param pTUTP 运行机构轨迹规划器的地址
 * @param bActiveSwayCtrl 激活防摇控制
 * @param eType 输入整形器类型
 * @param dNaturalFreq 自然频率(rad/s)
 * @param dDampingRatio 阻尼比
 * @param dStartPos 开始位置(m)
 * @param dTargetPos 目标位置(m)
 * @param dMaxVelocity 最大速度(m/s)
 * @param dMaxAcceleration 最大加速度(m/s^2)
 * @param dMaxJerk 最大加加速度(m/s^3)
 * @param dQuickStopMaxAcc 无防摇急停最大加速度(m/s^2)
 * @return 错误码 {0: 正常}
 */
int8_t SKA_TUTP_Create(SKA_TravelUnitTracePlanner *pTUTP, bool bActiveSwayCtrl,
                       SKA_InputShaperType eType, double dNaturalFreq, double dDampingRatio,
                       double dStartPos, double dTargetPos, double dMaxVelocity, double dMaxAcceleration, double dMaxJerk, 
                       double dQuickStopMaxAcc);

/**
 * @brief 执行运行机构轨迹规划器
 * @param pTUTP 运行机构轨迹规划器的地址
 * @param dCurTime 当前时间(s)
 * @param pRefPos 存储参考位置(m)的地址
 * @param pRefVel 存储参考速度(m/s)的地址
 * @param pRefAcc 存储参考加速度(m/s^2)的地址
 * @param pUnsRefPos 存储未整形参考位置(m)的地址
 * @param pUnsRefVel 存储未整形参考速度(m/s)的地址
 * @param pUnsRefAcc 存储未整形参考加速度(m/s^2)的地址
 * @return 错误码 {0: 正常}
 */
int8_t SKA_TUTP_Run(SKA_TravelUnitTracePlanner *pTUTP, double dCurTime,
                    double *pRefPos, double *pRefVel, double *pRefAcc,
                    double *pUnsRefPos, double *pUnsRefVel, double *pUnsRefAcc);
                    
int8_t SKA_TUTP_SingleOGetShapedTraceAndTotalTime(SKA_TravelUnitTracePlanner *pTUTP, double dCurTime, double dStartPos, double dTargetPos, bool bActiveSwayCtrl,
                                                   SKA_InputShaperType eType, double dNaturalFreq, double dDampingRatio, double dMaxVel, double dMaxAcc, double dMaxJerk,
                                                   double dQuickStopMaxAcc, double *pPos, double *pVel, double *pAcc, double *pTotaltime);
/**
 * @brief 获取轨迹总时间(s)
 * @param pTUTP 运行机构轨迹规划器的地址
 * @param pTotalTime 存储总时间的地址
 * @return 错误码 {0: 正常}
 */
int8_t SKA_TUTP_GetTotalTime(SKA_TravelUnitTracePlanner *pTUTP, double *pTotalTime);

/**
  * @brief  修改目标位置
  * @param  pTUTP 轨迹规划器的地址
  * @param  dCurTime 当前时间(s)
  * @param  dNewTargetPos 新的目标位置(m)
  * @return 错误码 {0: 正常}
  */
int8_t SKA_TUTP_ModifyTargetPos(SKA_TravelUnitTracePlanner *pTUTP, double dCurTime, double dNewTargetPos);

/**
  * @brief  带防摇缓停
  * @param  pTUTP 轨迹规划器的地址
  * @param  dCurTime 当前时间(s)
  * @return 错误码 {0: 正常}
  * @note   对未整形轨迹进行急停
  */
int8_t SKA_TUTP_AntiswayStop(SKA_TravelUnitTracePlanner *pTUTP, double dCurTime);

/**
 * @brief 预估制动距离
 * @param pTUTP 运行机构轨迹规划器的地址
 * @param dCurTime 当前时间(s)
 * @param pAscBrakeDistance 存储制动距离(m)的地址
 * @return 错误码 {0: 正常}
 */
int8_t SKA_TUTP_GetAscBrakeDistance(SKA_TravelUnitTracePlanner *pTUTP, double dCurTime, double *pAscBrakeDistance);

/**
 * @brief 获取参考轨迹的实际目标位置
 * @param pTUTP 运行机构轨迹规划器的地址
 * @param pRealTargetPos 存储实际目标位置(m)的地址
 * @return 错误码 {0: 正常}
 */
int8_t SKA_TUTP_GetRealTargetPos(SKA_TravelUnitTracePlanner *pTUTP, double *pRealTargetPos);

/**
  * @brief  无防摇急停
  * @param  pTUTP 轨迹规划器的地址
  * @param  dCurTime 当前时间(s)
  * @return 错误码 {0: 正常}
  */
int8_t SKA_TUTP_QuickStop(SKA_TravelUnitTracePlanner *pTUTP, double dCurTime);

/**
 * @brief 修改最大速度
 * @param pTUTP 运行机构轨迹规划器的地址
 * @param dCurTime 当前时间(s)
 * @param dNewMaxVelocity 新的最大速度(m/s)
 * @return 错误码 {0: 正常}
 */
int8_t SKA_TUTP_ModifyMaxVelocity(SKA_TravelUnitTracePlanner *pTUTP, double dCurTime, double dNewMaxVelocity);

/**
 * @brief 销毁运行机构轨迹规划器
 * @param pTUTP 运行机构轨迹规划器的地址
 * @return 错误码 {0: 正常}
 */
int8_t SKA_TUTP_Destroy(SKA_TravelUnitTracePlanner *pTUTP);

#endif //SKA2000_OHBC_CONTROLLER_SKA_TRAVELUNITTRACEPLANNER_H
