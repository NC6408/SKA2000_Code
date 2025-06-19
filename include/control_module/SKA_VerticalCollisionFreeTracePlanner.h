/**
 ******************************************************************************
 * @file       : SKA_VerticalCollisionFreeTracePlanner.h
 * @brief      : 纵向避障轨迹规划器
 * @author     : ShiRong
 * @version    : None
 * @date       : 2024/12/20
 ******************************************************************************
 */
//
#include <stddef.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>

#include "SKA_TravelUnitTracePlanner.h"
#include "SKA_HoistUnitTracePlanner.h"
#include "SKA_InputShaper.h"

#ifndef SKA2000_OHBC_CONTROLLER_SKA_VERTICALCOLLISIONFREETRACEPLANNER_H
#define SKA2000_OHBC_CONTROLLER_SKA_VERTICALCOLLISIONFREETRACEPLANNER_H

typedef struct
{
  double dMaxVelocity;     // 最大速度
  double dMaxAcceleration; // 最大加速度
  double dMaxJerk;         // 最大加加速度
} InputParam;

typedef struct
{
  double dDistance; // 距离(m)
  double dHeight;   // 高度(m)
} SKA_VCFTPTerrainPoint;

typedef struct
{
  /** 配置参数 **/
  InputParam inputParamX;                   // 大车输入参数
  InputParam inputParamY;                   // 小车输入参数
  bool bBridgeIsQuickStopping;              // 大车正在无防摇急停标志
  bool bBridgeActiveSwayCtrl;               // 激活大车防摇控制
  double dBridgeQuickStopMaxAcc;            // 大车无防摇急停最大加速度(m/s^2)
  SKA_InputShaperType BridgeeType;          // 输入整形器类型
  double dBridgeNaturalFreq;                // 自然频率(rad/s)
  double dBridgeDampingRatio;               // 阻尼比
  double dBridgeStartPos;                   // 大车开始位置(m)
  double dBridgeTargetPos;                  // 大车目标位置(m)
  double dBridgeRealMaxVelocity;            // 大车实际最大速度(m/s)
  double dBridgeRealMaxAcceleration;        // 大车实际最大加速度(m/s^2)
  double dBridgeRealMaxJerk;                // 大车实际最大加加速度(m/s^3)
  bool bTrolleyIsQuickStopping;             // 小车正在无防摇急停标志
  bool bTrolleyActiveSwayCtrl;              // 激活防摇控制
  double dTrolleyQuickStopMaxAcc;           // 小车无防摇急停最大加速度(m/s^2)
  SKA_InputShaperType TrolleyeType;         // 小车输入整形类型
  double dTrolleyNaturalFreq;               // 自然频率(rad/s)
  double dTrolleyDampingRatio;              // 阻尼比
  double dTrolleyStartPos;                  // 小车开始位置(m)
  double dTrolleyTargetPos;                 // 小车目标位置(m)
  double dTrolleyRealMaxVelocity;           // 小车实际最大速度(m/s)
  double dTrolleyRealMaxAcceleration;       // 小车实际最大加速度(m/s^2)
  double dTrolleyRealMaxJerk;               // 小车实际最大加加速度(m/s^3)
  double dHoistStartRopeLength;             // 起升机构开始绳长
  double dHoistMinRopeLength;               // 起升机构最小绳长
  double dHoistTargetRopeLength;            // 起升机构目标绳长
  double dHoistMaxVel;                      // 起升机构最大速度(m/s)
  double dHoistMaxAcc;                      // 起升机构最大加速度(m/s^2)
  double dHoistMaxJerk;                     // 起升机构最大加加速度(m/s^3)
  double dTotalHeight;                      // 总高度(m)
  SKA_VCFTPTerrainPoint arrLiftTerrain[20]; // 升吊地形数组
  uint16_t nLiftArraySize;                  // 升吊地形数组的有效元素数量
  SKA_VCFTPTerrainPoint arrDropTerrain[20]; // 落吊地形数组
  uint16_t nDropArraySize;                  // 落吊地形数组的有效元素数量
  double arrObstacleBridgeUpPos[20];        // 大车升吊地形信息位置数组
  double arrObstacleBridgeDownPos[20];      // 大车落吊地形信息位置数组
  double arrObstacleTrolleyUpPos[20];       // 小车升吊地形信息位置数组
  double arrObstacleTrolleyDownPos[20];     // 小车落吊地形信息位置数组
  double dTravelUnitUpMotionTime;           // 运行机构起吊的运行时间
  double dHoistUnitUpMotionTime;            // 起升机构起吊的运行时间
  double dTravelUnitDownMotionTime;         // 运行机构落吊的运行时间
  double dHoistUnitDownMotionTime;          // 起升机构落吊的运行时间
  double dTravelUnitDelayTime;              // 运行机构运行的延迟时间
  double dHoistUnitDelayTime;               // 起升机构运行的延迟时间
  double dHoistUnitStartDownTime;           // 起升机构开始下降的延迟时间
  double dHoistUpTraceTotalTime;            // 起升机构起吊轨迹的总时间
  // 中间变量
  double dDeltaX;                                        // 大车距离长度
  double dDeltaY;                                        // 小车距离长度
  double dlo;                                            // 搜索区间的下界
  double dhi;                                            // 搜索区间的上界
  double dmid;                                           // 搜索区间的中间值
  double dX;                                             // 整形后位置轨迹
  double dDx;                                            // 整形后速度轨迹
  double dDDx;                                           // 整形后加速度轨迹
  double dUlo;                                           // 搜索区间的下界
  double dUhi;                                           // 搜索区间的上界
  double dUmid;                                          // 搜索区间的中间值
  double dUx;                                            // 起升机构位置轨迹
  double dUdx;                                           // 起升机构速度轨迹
  double dUddx;                                          // 起升机构加速度轨迹
  SKA_InputShaper stBridgeShaper;                        // 大车输入整形器
  SKA_InputShaper stTrolleyShaper;                       // 小车输入整形器
  SKA_TravelUnitTracePlanner stTravelUnitTracePlanner;   // 运行机构轨迹规划器
  SKA_TravelUnitTracePlanner stBridgeTracePlanner;       // 大车整形轨迹规划器
  SKA_TravelUnitTracePlanner stTrolleyTracePlanner;      // 小车整形轨迹规划器
  SKA_HoistUnitTracePlanner stHoistUnitTracePlanner;     // 起升机构轨迹规划器
  SKA_HoistUnitTracePlanner stHoistUnitUpTracePlanner;   // 起升机构升吊轨迹规划器
  SKA_HoistUnitTracePlanner stHoistUnitDownTracePlanner; // 起升机构落吊轨迹规划器
} SKA_VerticalCollisionFreeTracePlanner;

/**
 * @brief 创建运行机构轨迹规划器
 * @param pVCFTP 纵向避障轨迹规划器的地址
 * @param bBridgeActiveSwayCtrl 激活大车防摇控制
 * @param BridgeeType 大车输入整形器类型
 * @param dBridgeNaturalFreq 大车自然频率(rad/s)
 * @param dBridgeDampingRatio 大车阻尼比
 * @param dBridgeStartPos 大车开始位置(m)
 * @param dBridgeTargetPos 大车目标位置(m)
 * @param dBridgeMaxVelocity 大车最大速度(m/s)
 * @param dBridgeMaxAcceleration 大车最大加速度(m/s^2)
 * @param dBridgeQuickStopMaxAcc 大车无防摇急停最大加速度(m/s^2)
 * @param bTrolleyActiveSwayCtrl 激活小车防摇控制
 * @param TrolleyType 小车输入整形器类型
 * @param dTrolleyNaturalFreq 小车自然频率(rad/s)
 * @param dTrolleyDampingRatio 小车阻尼比
 * @param dTrolleyStartPos 小车开始位置(m)
 * @param dTrolleyTargetPos 小车目标位置(m)
 * @param dTrolleyMaxVelocity 小车最大速度(m/s)
 * @param dTrolleyMaxAcceleration 小车最大加速度(m/s^2)
 * @param dTrolleyQuickStopMaxAcc 小车无防摇急停最大加速度(m/s^2)
 * @param dHoistStartRopeLength 起升机构开始绳长(m)
 * @param dHoistMinRopeLength 起升机构最小绳长(m)
 * @param dHoistTargetRopeLength 起升机构目标绳长(m)
 * @param dHoistMaxVel 起升机构最大速度(m/s)
 * @param dHoistMaxAcc 起升机构最大加速度(m/s^2)
 * @param dHoistMaxJerk 起升机构最大加加速度(m/s^3)
 * @return 错误码 {0: 正常}
 */
int8_t SKA_VCFTP_Create(SKA_VerticalCollisionFreeTracePlanner *pVCFTP, bool bBridgeActiveSwayCtrl, SKA_InputShaperType BridgeeType,
                        double dBridgeNaturalFreq, double dBridgeDampingRatio, double dBridgeStartPos, double dBridgeTargetPos,
                        double dBridgeMaxVelocity, double dBridgeMaxAcceleration, double dBridgeQuickStopMaxAcc, bool bTrolleyActiveSwayCtrl,
                        SKA_InputShaperType TrolleyeType, double dTrolleyNaturalFreq, double dTrolleyDampingRatio, double dTrolleyStartPos,
                        double dTrolleyTargetPos, double dTrolleyMaxVelocity, double dTrolleyMaxAcceleration, double dTrolleyQuickStopMaxAcc,
                        double dHoistStartRopeLength, double dHoistMinRopeLength, double dHoistTargetRopeLength, double dHoistMaxVel,
                        double dHoistMaxAcc, double dHoistMaxJerk);

/**
 * @brief 执行运行机构轨迹规划器
 * @param pVCFTP 纵向避障轨迹规划器的地址
 * @param dCurTime 当前时间(s)
 * @param pTravelUnitMaxDelayTime 运行机构开始运行的最大延迟时间(s)
 * @param pHoistUnitMaxStartDownTime 起升机构最大开始下降时间 (s)
 * @param pBridgeRefPos 存储大车参考位置(m)的地址
 * @param pBridgeRefVel 存储大车参考速度(m/s)的地址
 * @param pBridgeRefAcc 存储大车参考加速度(m/s^2)的地址
 * @param pBridgeUnsRefPos 存储大车未整形参考位置(m)的地址
 * @param pBridgeUnsRefVel 存储大车未整形参考速度(m/s)的地址
 * @param pBridgeUnsRefAcc 存储大车未整形参考加速度(m/s^2)的地址
 * @param pTrolleyRefPos 存储小车参考位置(m)的地址
 * @param pTrolleyRefVel 存储小车参考速度(m/s)的地址
 * @param pTrolleyRefAcc 存储小车参考加速度(m/s^2)的地址
 * @param pTrolleyUnsRefPos 存储小车未整形参考位置(m)的地址
 * @param pTrolleyUnsRefVel 存储小车未整形参考速度(m/s)的地址
 * @param pTrolleyUnsRefAcc 存储小车未整形参考加速度(m/s^2)的地址
 * @param pHoistRefPos 存储起升机构参考位置(m)的地址
 * @param pHoistRefVel 存储起升机构参考速度(m/s)的地址
 * @param pHoistRefAcc 存储起升机构参考加速度(m/s^2)的地址
 * @return 错误码 {0: 正常}
 */
int8_t SKA_VCFTP_Run(SKA_VerticalCollisionFreeTracePlanner *pVCFTP, double dCurTime, double dTravelUnitMaxDelayTime, double dHoistUnitMaxStartDownTime,
                     double *pBridgeRefPos, double *pBridgeRefVel, double *pBridgeRefAcc, double *pBridgeUnsRefPos, double *pBridgeUnsRefVel, 
                     double *pBridgeUnsRefAcc, double *pTrolleyRefPos, double *pTrolleyRefVel, double *pTrolleyRefAcc, double *pTrolleyUnsRefPos, 
                     double *pTrolleyUnsRefVel, double *pTrolleyUnsRefAcc, double *pHoistRefPos, double *pHoistRefVel, double *pHoistRefAcc);

/**
 * @brief  获取运行机构开始运行的最大延迟时间和起升机构最大开始下降时间
 * @param pVCFTP 纵向避障轨迹规划器的地址
 * @param pTravelUnitMaxDelayTime 存储运行机构开始运行的最大延迟时间(s)的地址
 * @param pHoistUnitMaxStartDownTime 存储起升机构最大开始下降时间 (s)的地址
 * @return 错误码 {0: 正常}
 */
int8_t SKA_VCFTP_TravelMaxDelayTimeAndHoistMaxStartDownTime(SKA_VerticalCollisionFreeTracePlanner *pVCFTP,
                                                            double *pTravelUnitMaxDelayTime, double *pHoistUnitMaxStartDownTime);

/**
 * @brief 销毁运行机构轨迹规划器
 * @param pVCFTP 纵向避障轨迹规划器的地址
 * @return 错误码 {0: 正常}
 */
int8_t SKA_VCFTP_Destroy(SKA_VerticalCollisionFreeTracePlanner *pVCFTP);

#endif // SKA2000_OHBC_CONTROLLER_SKA_VERTICALCOLLISIONFREETRACEPLANNER_H
