/**
 ******************************************************************************
 * @file       : SKA_UnshapedTracePlanner.h
 * @brief      : 未整形轨迹规划器
 * @author     : ZhangYi
 * @version    : None
 * @date       : 2024/10/29
 ******************************************************************************
 */
//

#ifndef SKA2000_OHBC_CONTROLLER_SKA_UNSHAPEDTRACEPLANNER_H
#define SKA2000_OHBC_CONTROLLER_SKA_UNSHAPEDTRACEPLANNER_H

#include <stdint.h>
#include <stdbool.h>

#include "SKA_DoubleSshapedTrace.h"
#include "SKA_InputShaper.h"

typedef struct
{
  double dStartTime; // 开始时间(s)
  double dStartPos;  // 开始位置(mm)
  double dTargetPos; // 目标位置(mm)
} SKA_UTPSingleOSubTask;

typedef struct
{
  float dStartTime;      // 开始时间(s)
  float dStartPos;       // 开始位置(mm)
  float dTargetPos;      // 目标位置(mm)
  float dStartVelocity;  // 开始速度(mm/s)
  float dTargetVelocity; // 目标速度(mm/s)
  float dMinTime;        // 最小时间(s)
} SKA_UTPMultipleOSubTask;

// 未整形轨迹规划器
typedef struct
{
  double dStartPos;        // 开始位置(m)
  double dTargetPos;       // 目标位置(m)
  double dMaxVelocity;     // 最大速度(m/s)
  double dMaxAcceleration; // 最大加速度(m/s^2)
  double dMaxJerk;         // 最大加加速度(m/s^3)
  double dBufferTime;      // 速度变向时的缓冲时间(s)

  bool bIsUrgencyStopping;      // 急停标志
  SKA_TraceSegment *pTraceHead; // 轨迹头结点指针
  uint16_t pTraceNum;

  bool bIsUseWaypoints;
  uint16_t nObstacleNum;
  uint16_t nWaypointsNum;
  bool bActiveSwayCtrl;
  SKA_InputShaperType eType;
  double dNaturalFreq;
  double dDampingRatio;
  SKA_UTPSingleOSubTask SOSubTasks[2]; // 子任务信息
  SKA_UTPMultipleOSubTask MOSubTasks[21];
  double dtotalTime;
  uint8_t nSubTasksNum; // 子任务数量
  uint8_t ntasklndex;   // 任务索引
  SKA_InputShaper InputShaper;
  bool bisUniform;
  uint8_t bActiveHorizontalMode;
} SKA_UnshapedTracePlanner;

/**
 * @brief  创建未整形轨迹规划器
 * @param  pUTP 未整形轨迹规划器的地址
 * @param  dStartPos 开始位置(m)
 * @param  dTargetPos 目标位置(m)
 * @param  dMaxVelocity 最大速度(m/s)
 * @param  dMaxAcceleration 最大加速度(m/s^2)
 * @param  dMaxJerk 最大加加速度(m/s^3)
 * @param  dBufferTime 速度变向时的缓冲时间(s)
 * @return 错误码 {0: 正常}
 */
int8_t SKA_UTP_Create(SKA_UnshapedTracePlanner *pUTP, double dStartPos, double dTargetPos,
                      double dMaxVelocity, double dMaxAcceleration, double dMaxJerk, double dBufferTime);

/**
 * @brief  获取当前时刻的未整形参考轨迹
 * @param  pUTP 未整形参考轨迹规划器的地址
 * @param  dCurTime 当前时间(s)
 * @param  pUnsRefPos 存储未整形参考轨迹位置(m)的地址
 * @param  pUnsRefVelocity 存储未整形参考轨迹速度(m/s)的地址
 * @param  pUnsRefAcceleration 存储未整形参考轨迹加速度(m/s^2)的地址
 * @return 错误码 {0: 正常}
 */
int8_t SKA_UTP_Run(const SKA_UnshapedTracePlanner *pUTP, double dCurTime,
                   double *pUnsRefPos, double *pUnsRefVelocity, double *pUnsRefAcceleration);

/**
 * @brief  获取未整形参考轨迹的持续总时长
 * @param  pUTP 未整形参考轨迹规划器的地址
 * @param  pTotalTime 存储总时间(s)的地址
 * @return 错误码 {0: 正常}
 */
int8_t SKA_UTP_GetTotalTime(const SKA_UnshapedTracePlanner *pUTP, double *pTotalTime);


int8_t SKA_UTP_ObstacleUnshapedTrace(SKA_UnshapedTracePlanner *pUTP, double dCurTime,
                                      double *pUnsRefPos, double *pUnsRefVelocity, double *pUnsRefAcceleration);
/**
 * @brief  修改目标位置
 * @param  pUTP 未整形参考轨迹规划器的地址
 * @param  dCurTime 当前时间(s)
 * @param  dNewTargetPos 新的目标位置(m)
 * @return 错误码 {0: 正常}
 */
int8_t SKA_UTP_ModifyTargetPos(SKA_UnshapedTracePlanner *pUTP, double dCurTime, double dNewTargetPos);

/**
 * @brief  紧急停止
 * @param  pUTP 未整形参考轨迹规划器的地址
 * @param  dCurTime 当前时间(s)
 * @return 错误码 {0: 正常}
 */
int8_t SKA_UTP_UrgencyStop(SKA_UnshapedTracePlanner *pUTP, double dCurTime);

/**
 * @brief  获取紧急停止的目标位置(m)
 * @param  pUTP 未整形参考轨迹规划器的地址
 * @param  dCurTime 当前时间(s)
 * @param  pTargetPos 存储目标位置的地址
 * @return 错误码 {0: 正常}
 */
int8_t SKA_UTP_GetUrgencyStopPos(SKA_UnshapedTracePlanner *pUTP, double dCurTime, double *pTargetPos);

/**
 * @brief  获取实际目标位置
 * @param  pUTP 未整形参考轨迹规划器的地址
 * @param  pRealTargetPos 存储实际目标位置(m)的地址
 * @return 错误码 {0: 正常}
 */
int8_t SKA_UTP_GetRealTargetPos(const SKA_UnshapedTracePlanner *pUTP, double *pRealTargetPos);

/**
 * @brief 获取多障碍物避让情况下未整形轨迹和时间
 * @param dCurTime 当前时间(s)
 * @param dStartPos 开始位置(m)
 * @param dTargetPos 目标位置(m)
 * @param dMaxVelocity 最大速度(m/s)
 * @param dMaxAcceleration 最大加速度(m/s^2)
 * @param dMaxJerk 最大加加速度(m/s^3)
 * @param pPos 存储位置(m)的地址
 * @param pVelocity 存储速度(m/s)的地址
 * @param pAcceleration 存储加速度(m/s^2)的地址
 * @param pTotalTime 存储总时间(s)的地址
 * @return 错误码 {0: 正常}
 */
int8_t SKA_UTP_GetMultipleOUnshapedTrace(SKA_UnshapedTracePlanner *pUTP, double dCurTime, double dStartPos, double dTargetPos, double dStartVelocity,
                                           double dTargetVelocity, double dMinTime, bool bActiveSwayCtrl, SKA_InputShaperType eType,
                                           double dNaturalFreq, double dDampingRatio, double dMaxVelocity, double dMaxAcceleration, double dMaxJerk,
                                           SKA_InputShaper InputShaper, double *pPos, double *pVelocity,
                                           double *pAcceleration, double *pTotalTime, bool *bIsUniform);

/**
 * @brief 获取多障碍物避让情况下未整形轨迹
 * @param dCurTime 当前时间(s)
 * @param dTargetPos 目标位置(m)
 * @param dStartVelocity 开始速度(m/s)
 * @param dTargetVelocity 目标速度(m/s)
 * @param dMaxVelocity 最大速度(m/s)
 * @param dMaxAcceleration 最大加速度(m/s^2)
 * @param dMaxJerk 最大加加速度(m/s^3)
 * @param InputShaper 整形器
 * @param pPos 存储位置(m)的地址
 * @param pVelocity 存储速度(m/s)的地址
 * @param pAcceleration 存储加速度(m/s^2)的地址
 * @param pTotalTime 存储总时间(s)的地址
 * @param IsUniform 存储开始速度和目标速度是否都为0标志的地址
 * @return 错误码 {0: 正常}
 */
int8_t SKA_UTP_MultipleOGetNormalUnshapedTrace(SKA_UnshapedTracePlanner *pUTP, double dCurTime, double dTargetPos, double dStartVelocity, double dTargetVelocity,
                                                 bool bActiveSwayCtrl, SKA_InputShaperType eType, double dNaturalFreq, double dDampingRatio, double dMaxVelocity,
                                                 double dMaxAcceleration, double dMaxJerk, SKA_InputShaper InputShaper, double *pPos,
                                                 double *pVelocity, double *pAcceleration, double *pTotalTime, bool *bIsUniform);

int8_t SKA_UTP_MultipleOGetNormalUnshapedTraceOverMinTime(SKA_UnshapedTracePlanner *pUTP, double dCurTime, double dTargetPos, 
                        double dStartVelocity, double dMinTime, double dMaxVelocity, double dMaxAcceleration, double dMaxJerk, 
                        SKA_InputShaper InputShaper, double* pPos, double* pVelocity, double* pAcceleration, double* pTotalTime, bool *bIsUniform);
/**
 * @brief 获取多障碍物避让情况下未整形轨迹的最短距离和最短时间
 * @param dStartVelocity 开始速度(m/s)
 * @param dTargetVelocity 目标速度(m/s)
 * @param dMaxAcceleration 最大加速度(m/s^2)
 * @param dMaxJerk 最大加加速度(m/s^3)
 * @param pMinDistance 存储最短距离的地址
 * @param pMinTime 存储最短时间的地址
 * @return 错误码 {0: 正常}
 */
int8_t SKA_UTP_MultipleOGetUnshapedMinDistAndTime(SKA_UnshapedTracePlanner *pUTP, double dStartVelocity, double dTargetVelocity, double dMaxAcceleration,
                                                    double dMaxJerk, double *pMinDistance, double *pMinTime);

/**
 * @brief 获取多障碍物避让情况下正常未整形轨迹段
 * @param dCurTime 当前时间(s)
 * @param dStartVelocity 开始速度(m/s)
 * @param dTargetVelocity 目标速度(m/s)
 * @param dMaxAcceleration 最大加速度(m/s^2)
 * @param dMaxJerk 最大加加速度(m/s^3)
 * @param pPos 存储位置(m)的地址
 * @param pVelocity 存储速度(m/s)的地址
 * @param pAcceleration 存储加速度(m/s^2)的地址
 * @return 错误码 {0: 正常}
 */
int8_t SKA_UTP_MultipleOGetNormalUnshapedTraceSegment(SKA_UnshapedTracePlanner *pUTP, double dCurTime, double dStartVelocity, double dTargetVelocity, double dMaxAcceleration,
                                                        double dMaxJerk, double *pPos, double *pVelocity, double *pAcceleration);

int8_t SKA_UTP_GetSingleOUnshapedTrace(SKA_UnshapedTracePlanner *pUTP, double dCurTime, double dStartPos, double dTargetPos, double dMaxVelocity, double dMaxAcceleration,
                                         double dMaxJerk, double *pPos, double *pVelocity, double *pAcceleration, double *pTotalTime);
/**
 * @brief  销毁未整形轨迹规划器
 * @param  pUTP 未整形轨迹规划器的地址
 * @return 错误码 {0: 正常}
 */
int8_t SKA_UTP_Destroy(SKA_UnshapedTracePlanner *pUTP);

/**
 * @brief  打印未整形轨迹规划器的轨迹信息
 * @param  pUTP 未整形轨迹规划器的地址
 * @return 错误码 {0: 正常}
 */
int8_t SKA_UTP_Display(const SKA_UnshapedTracePlanner *pUTP);

#endif // SKA2000_OHBC_CONTROLLER_SKA_UNSHAPEDTRACEPLANNER_H