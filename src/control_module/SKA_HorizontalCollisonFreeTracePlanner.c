/**
 ******************************************************************************
 * @file       : SKA_HorizontalCollisonFreeTracePlanner.h
 * @brief      : 横向避障轨迹规划器
 * @author     : ShiRong
 * @version    : None
 * @date       : 2025/02/20
 ******************************************************************************
 */
//

#include <stddef.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "SKA_HorizontalCollisonFreeTracePlanner.h"
#include "SKA_InputShaper.h"
#include "SKA_UnshapedTracePlanner.h"

/**
 * @brief 判断开始位置和目标位置所确定的矩形区域与障碍物区域是否相交
 * @param dLocationX1 位置1的X坐标(m)
 * @param dLocationY1 位置1的Y坐标(m)
 * @param dLocationW1 位置1的宽度
 * @param dLocationH1 位置1的高度
 * @param dLocationX2 位置2的X坐标
 * @param dLocationY2 位置2的Y坐标
 * @return 错误码 {0: 正常}
 */
static int8_t SKA_HCFTP_IsOpenRectanglesIntersect(double dLocationX1, double dLocationY1, double dLocationW1, double dLocationH1,
                                                  double dLocationX2, double dLocationY2, double dLocationW2, double dLocationH2, bool *pflag);

/**
 * @brief 获取单障碍物右侧避障轨迹
 * @param pSOGNRT 获取单障碍物右侧避障轨迹的地址
 * @param dBridgeTargetPos 大车位置(m)
 * @param dBridgeMaxVelocity 最大速度(m/s)
 * @param dBridgeMaxAcceleration 最大加速度(m/s^2)
 * @param dBridgeMaxJerk 最大加加速度(m/s^3)
 * @param BridgeInputShaper 大车整形器类型
 * @param dTrolleyTargetPos 小车位置(m)
 * @param dTrolleyMaxVelocity 最大速度(m/s)
 * @param dTrolleyMaxAcceleration 最大加速度(m/s^2)
 * @param dTrolleyMaxJerk 最大加加速度(m/s^3)
 * @param TrolleyInputShaper 小车整形器类型
 * @param Obstacle 障碍物信息
 * @param dBridgeMaxVelocity 最大速度(m/s)
 * @param dBridgeMaxAcceleration 最大加速度(m/s^2)
 * @param bBridgeActiveSwayCtrl 激活防摇控制
 * @param BridgeeType 输入整形器类型
 * @param dBridgeNaturalFreq 自然频率(rad/s)
 * @param dBridgeDampingRatio 阻尼比
 * @param dBridgeQuickStopMaxAcc 无防摇急停最大加速度
 * @param bTrolleyActiveSwayCtrl 激活防摇控制
 * @param TrolleyType 输入整形器类型
 * @param dTrolleyNaturalFreq 自然频率(rad/s)
 * @param dTrolleyDampingRatio 阻尼比
 * @param dTrolleyQuickStopMaxAcc 无防摇急停最大加速度
 * @param pBridgeSubTasksNum 存储大车子任务的数量地址
 * @param pBridgeTotalTime 存储大车轨迹总时间的地址(s)
 * @param pTrolleySubTasksNum 存储小车任务的数量地址
 * @param pTrolleyTotalTime 存储小车轨迹总时间的地址(s)
 * @param pTotalTime 存储总轨迹总时间的地址(s)
 * @return 错误码 {0: 正常}
 */
static int8_t SKA_HCFTP_SingleOGetNormalRightTrace(SKA_HorizontalCollisonFreeTracePlanner *pHCFTP, double dBridgeTargetPos, double dBridgeMaxVelocity,
                                                   double dBridgeMaxAcceleration, double dBridgeMaxJerk, SKA_InputShaper BridgeInputShaper, double dTrolleyTargetPos,
                                                   double dTrolleyMaxVelocity, double dTrolleyMaxAcceleration, double dTrolleyMaxJerk, SKA_InputShaper TrolleyInputShaper,
                                                   SKA_HCFTPObstacleCoord Obstacle, bool bBridgeActiveSwayCtrl, SKA_InputShaperType BridgeeType,
                                                   double dBridgeNaturalFreq, double dBridgeDampingRatio, double dBridgeQuickStopMaxAcc,
                                                   bool bTrolleyActiveSwayCtrl, SKA_InputShaperType TrolleyeType, double dTrolleyNaturalFreq,
                                                   double dTrolleyDampingRatio, double dTrolleyQuickStopMaxAcc, uint8_t *pBridgeSubTasksNum,
                                                   double *pBridgeTotalTime, uint8_t *pTrolleySubTasksNum, double *pTrolleyTotalTime, double *pTotalTime);
/**
 * @brief 获取单障碍物避障运动时间
 * @param pSOGTAT 获取单障碍物避障运动时间的地址
 * @param dFindPos 位置
 * @param dStartPos 开始位置
 * @param dTargetPos 目标位置
 * @param bActiveSwayCtrl 激活防摇控制
 * @param eType 输入整形器类型
 * @param dNaturalFreq 自然频率(rad/s)
 * @param dDampingRatio 阻尼比
 * @param dMaxVelocity 最大速度(m/s)
 * @param dMaxAcceleration 最大加速度(m/s^2)
 * @param dMaxJerk 最大加加速度(m/s^3)
 * @param dQuickStopMaxAcc 无防摇急停最大加速度(m/s^2)
 * @param pMotionTime 存储运行时间的地址(s)
 * @return 错误码 {0: 正常}
 */
static int8_t SKA_HCFTP_SingleOGetMotionTime(SKA_HorizontalCollisonFreeTracePlanner *pHCFTP, double dFindPos, double dStartPos, double dTargetPos,
                                             bool bActiveSwayCtrl, SKA_InputShaperType eType, double dNaturalFreq, double dDampingRatio,
                                             double dMaxVelocity, double dMaxAcceleration, double dMaxJerk, double dQuickStopMaxAcc, double *pMotionTime);
/**
 * @brief 获取多障碍物情况下的途径点
 * @param MOGWP 获取多障碍物情况下的途径点的地址
 * @param dBridgeStartPos 大车开始位置(m)
 * @param dBridgeTargetPos 大车目标位置(m)
 * @param dBridgeMaxPos 大车允许的最大位置(m)
 * @param dBridgeMinPos 大车允许的最小位置(m)
 * @param dTrolleyStartPos 小车开始位置(m)
 * @param dTrolleyTargetPos 小车目标位置(m)
 * @param dTrolleyMaxPos  小车允许的最大位置(m)
 * @param dTrolleyMinPos   小车允许的最小位置(m)
 * @param nObstacleNum 障碍物个数
 * @param WayPointsNum 存储途径点的数量的地址
 * @return 错误码 {0: 正常}
 */
static int8_t SKA_HCFTP_MultipleOGetWayPoints(SKA_HorizontalCollisonFreeTracePlanner *pHCFTP, double dBridgeStartPos, double dBridgeTargetPos, double dBridgeMaxPos,
                                              double dBridgeMinPos, double dTrolleyStartPos, double dTrolleyTargetPos, double dTrolleyMaxPos,
                                              double dTrolleyMinPos, uint8_t nObstacleNum, uint8_t *WayPointsNum);

/**
 * @brief 多障碍物情况下判断障碍物顶点是否在矩形内
 * @param dPosX 顶点X坐标
 * @param dPosY 顶点Y坐标
 * @param dPosRX 大车允许的位置(m)
 * @param dPosRY 小车允许的位置(m)
 * @param dPosRW 大车允许的距离(m)
 * @param dPosRH 小车允许的距离(m)
 * @param pflag 存储障碍物顶点是否在矩形内的地址
 * @return 错误码 {0: 正常}
 */
static int8_t SKA_HCFTP_MultipleOIsPointInCloseRectangle(double dPosX, double dPosY, double dPosRX,
                                                         double dPosRY, double dPosRW, double dPosRH, bool *pflag);
/**
 * @brief 获取多障碍物避让情况下途径点的速度
 * @param pHCFTP 获取多障碍物避让情况下途径点的速度的地址
 * @param nWayPointsNum 途径点的数量
 * @param bBridgeActiveSwayCtrl 激活防摇控制
 * @param BridgeeType 输入整形器类型
 * @param dBridgeNaturalFreq 自然频率(rad/s)
 * @param dBridgeDampingRatio 阻尼比
 * @param dBridgeQuickStopMaxAcc 无防摇急停最大加速度
 * @param dBridgeMaxVelocity 最大速度(m/s)
 * @param dBridgeMaxAcceleration 最大加速度(m/s^2)
 * @param dBridgeMaxJerk 最大加加速度(m/s^3)
 * @param stBridgeShaper 大车整形器类型
 * @param bTrolleyActiveSwayCtrl 激活防摇控制
 * @param TrolleyeType 输入整形器类型
 * @param dTrolleyNaturalFreq 自然频率(rad/s)
 * @param dTrolleyDampingRatio 阻尼比
 * @param dTrolleyQuickStopMaxAcc 无防摇急停最大加速度
 * @param dTrolleyMaxVelocity 最大速度(m/s)
 * @param dTrolleyMaxAcceleration 最大加速度(m/s^2)
 * @param dTrolleyMaxJerk 最大加加速度(m/s^3)
 * @param stTrolleyShaper 小车整形器类型
 * @return 错误码 {0: 正常}
 */
static int8_t SKA_HCFTP_MultipleOGetWayPointsVelocity(SKA_HorizontalCollisonFreeTracePlanner *pHCFTP, uint8_t nWayPointsNum, bool bBridgeActiveSwayCtrl,
                                                      SKA_InputShaperType BridgeeType, double dBridgeNaturalFreq, double dBridgeDampingRatio,
                                                      double dBridgeQuickStopMaxAcc, double dBridgeMaxVelocity, double dBridgeMaxAcceleration,
                                                      double dBridgeMaxJerk, SKA_InputShaper stBridgeShaper, bool bTrolleyActiveSwayCtrl,
                                                      SKA_InputShaperType TrolleyeType, double dTrolleyNaturalFreq, double dTrolleyDampingRatio,
                                                      double dTrolleyQuickStopMaxAcc, double dTrolleyMaxVelocity, double dTrolleyMaxAcceleration,
                                                      double dTrolleyMaxJerk, SKA_InputShaper stTrolleyShaper);

/**
 * @brief 获取多障碍物避让情况下最大开始速度
 * @param dDistance 距离(m）
 * @param dMaxVelocity 最大速度(m/s)
 * @param dMaxAcceleration 最大加速度(m/s^2)
 * @param dMaxJerk 最大加加速度(m/s^3)
 * @param InputShaper 整形器
 * @param pMaxStartVelocity 存储最大开始速度的地址
 * @return 错误码 {0: 正常}
 */
static int8_t SKA_HCFTP_MultipleOGetMaxStartVelocity(SKA_HorizontalCollisonFreeTracePlanner *pHCFTP, double dDistance, double dMaxVelocity, double dMaxAcceleration, double dMaxJerk,
                                                     SKA_InputShaper InputShaper, double *pMaxStartVelocity);

/**
 * @brief 获取多障碍物避让情况下整形轨迹的最短距离和最短时间
 * @param dStartVelocity 开始速度(m/s)
 * @param dTargetVelocity 目标速度(m/s)
 * @param dMaxAcceleration 最大加速度(m/s^2)
 * @param dMaxJerk 最大加加速度(m/s^3)
 * @param InputShaper 整形器
 * @param pMinDistance 存储最短距离的地址
 * @param pMinTime 存储最短时间的地址
 * @return 错误码 {0: 正常}
 */
static int8_t SKA_HCFTP_MultipleOGetShapedMinDistAndTime(SKA_HorizontalCollisonFreeTracePlanner *pHCFTP, double dStartVelocity, double dTargetVelocity, double dMaxAcceleration,
                                                         double dMaxJerk, SKA_InputShaper InputShaper, double *pMinDistance, double *pMinTime);

/**
 * @brief 获取多障碍物避让情况下整形轨迹
 * @param dCurTime 当前时间(s)
 * @param dStartPos 开始位置(m)
 * @param dTargetPos 目标位置(m)
 * @param dStartVelocity 开始速度(m/s)
 * @param dTargetVelocity 目标速度(m/s)
 * @param dMinTime 最短时间(s)
 * @param dMaxVelocity 最大速度(m/s)
 * @param dMaxAcceleration 最大加速度(m/s^2)
 * @param dMaxJerk 最大加加速度(m/s^3)
 * @param InputShaper 整形器
 * @param pPos 存储位置(m)的地址
 * @param pVelocity 存储速度(m/s)的地址
 * @param pAcceleration 存储加速度(m/s^2)的地址
 * @param pTotalTime 存储总时间(s)的地址
 * @return 错误码 {0: 正常}
 */
  static int8_t SKA_HCFTP_MultipleOGetShapedTrace(SKA_HorizontalCollisonFreeTracePlanner *pHCFTP, double dCurTime, double dStartPos, double dTargetPos,
                                                  double dStartVelocity, double dTargetVelocity, double dMinTime, bool bActiveSwayCtrl, SKA_InputShaperType eType,
                                                  double dNaturalFreq, double dDampingRatio, double dMaxVelocity, double dMaxAcceleration, double dMaxJerk,  
                                                  SKA_InputShaper InputShaper, double *pPos, double *pVelocity, double *pAcceleration, double *pTotalTime);


/**
 * @brief 多障碍物避让情况下获取最短时间的最大开始速度
 * @param dDistance 距离(m)
 * @param dMaxAcceleration 最大加速度(m/s^2)
 * @param dMaxJerk 最大加加速度(m/s^3)
 * @param InputShaper 整形器
 * @param dMinTime 最短时间(s)
 * @param dMaxTargetVelocity 最大的目标速度(m/s)的
 * @param pMaxStartVelocity 存储最大开始速度(m/s)的地址
 * @return 错误码 {0: 正常}
 */
static int8_t SKA_HCFTP_MultipleOGetMaxStartVelocityOverMinTime(SKA_HorizontalCollisonFreeTracePlanner *pHCFTP, double dDistance, double dMaxAcceleration, double dMaxJerk, SKA_InputShaper InputShaper,
                                                                double dMinTime, double dMaxTargetVelocity, double *pMaxStartVelocity);

int8_t SKA_HCFTP_Create(SKA_HorizontalCollisonFreeTracePlanner *pHCFTP, bool bBridgeActiveSwayCtrl, SKA_InputShaperType BridgeeType,
                        double dBridgeNaturalFreq, double dBridgeDampingRatio, double dBridgeStartPos, double dBridgeTargetPos,
                        double dBridgeMaxVelocity, double dBridgeMaxAcceleration, double dBridgeQuickStopMaxAcc, double dBridgeMaxPos,
                        double dBridgeMinPos, bool bTrolleyActiveSwayCtrl, SKA_InputShaperType TrolleyeType, double dTrolleyNaturalFreq, 
                        double dTrolleyDampingRatio, double dTrolleyStartPos, double dTrolleyTargetPos, double dTrolleyMaxVelocity,  
                        double dTrolleyMaxAcceleration, double dTrolleyQuickStopMaxAcc, double dTrolleyMaxPos, double dTrolleyMinPos)
{
  /******************** 函数参数合法性检验 ********************/
  if (pHCFTP == NULL || dBridgeNaturalFreq <= 0 || dBridgeDampingRatio < 0 || dBridgeDampingRatio >= 1 ||
      dBridgeMaxVelocity <= 0 || dBridgeMaxAcceleration <= 0 || dBridgeQuickStopMaxAcc <= 0 ||
      dTrolleyNaturalFreq <= 0 || dTrolleyDampingRatio < 0 || dTrolleyDampingRatio >= 1 ||
      dTrolleyMaxVelocity <= 0 || dTrolleyMaxAcceleration <= 0 || dTrolleyQuickStopMaxAcc <= 0)
  {
    printf("Invalid parameters of SKA_HCFTP_Create().\n");
    return -1;
  }

  // 创建大车轨迹规划器
  pHCFTP->dBridgeQuickStopMaxAcc = dBridgeQuickStopMaxAcc;
  pHCFTP->bBridgeActiveSwayCtrl = bBridgeActiveSwayCtrl;
  pHCFTP->BridgeeType = BridgeeType;
  pHCFTP->dBridgeNaturalFreq = dBridgeNaturalFreq;
  pHCFTP->dBridgeDampingRatio = dBridgeDampingRatio;
  pHCFTP->dBridgeStartPos = dBridgeStartPos;
  pHCFTP->dBridgeTargetPos = dBridgeTargetPos;
  pHCFTP->dBridgeMaxVelocity = dBridgeMaxVelocity;
  pHCFTP->dBridgeMaxAcceleration = dBridgeMaxAcceleration;
  pHCFTP->dBridgeMaxJerk = 5.0 * pow(dBridgeMaxAcceleration, 2) / dBridgeMaxVelocity;
  double dBridgeMaxJerk;
  dBridgeMaxJerk = pHCFTP->dBridgeMaxJerk;
  pHCFTP->dBridgeMaxPos = dBridgeMaxPos;
  pHCFTP->dBridgeMinPos = dBridgeMinPos;

  // 创建小车轨迹规划器
  pHCFTP->dTrolleyQuickStopMaxAcc = dTrolleyQuickStopMaxAcc;
  pHCFTP->bTrolleyActiveSwayCtrl = bTrolleyActiveSwayCtrl;
  pHCFTP->TrolleyeType = TrolleyeType;
  pHCFTP->dTrolleyNaturalFreq = dTrolleyNaturalFreq;
  pHCFTP->dTrolleyDampingRatio = dTrolleyDampingRatio;
  pHCFTP->dTrolleyStartPos = dTrolleyStartPos;
  pHCFTP->dTrolleyTargetPos = dTrolleyTargetPos;
  pHCFTP->dTrolleyMaxVelocity = dTrolleyMaxVelocity;
  pHCFTP->dTrolleyMaxAcceleration = dTrolleyMaxAcceleration;
  pHCFTP->dTrolleyMaxJerk = 5.0 * pow(dTrolleyMaxAcceleration, 2) / dTrolleyMaxVelocity;
  double dTrolleyMaxJerk;
  dTrolleyMaxJerk = pHCFTP->dTrolleyMaxJerk;
  pHCFTP->dTrolleyMaxPos = dTrolleyMaxPos;
  pHCFTP->dTrolleyMinPos = dTrolleyMinPos;

  if ((pHCFTP->bIsUseWaypoints == 1 && pHCFTP->nWaypointsNum == 0) ||
      (pHCFTP->bIsUseWaypoints == 0 && pHCFTP->nObstacleNum == 0))
  {
    /******************** 创建无障碍物运行机构轨迹规划器 ********************/

    SKA_TUTP_Create(&(pHCFTP->stBridgeTracePlanner), pHCFTP->bBridgeActiveSwayCtrl, pHCFTP->BridgeeType,
                    pHCFTP->dBridgeNaturalFreq, pHCFTP->dBridgeDampingRatio, pHCFTP->dBridgeStartPos, pHCFTP->dBridgeTargetPos,
                    pHCFTP->dBridgeMaxVelocity, pHCFTP->dBridgeMaxAcceleration, pHCFTP->dBridgeMaxJerk, pHCFTP->dBridgeQuickStopMaxAcc);
    SKA_TUTP_Create(&(pHCFTP->stTrolleyTracePlanner), pHCFTP->bTrolleyActiveSwayCtrl, pHCFTP->TrolleyeType,
                    pHCFTP->dTrolleyNaturalFreq, pHCFTP->dTrolleyDampingRatio, dTrolleyStartPos, dTrolleyTargetPos,
                    pHCFTP->dTrolleyMaxVelocity, pHCFTP->dTrolleyMaxAcceleration, pHCFTP->dTrolleyMaxJerk, pHCFTP->dTrolleyQuickStopMaxAcc);
  }
  else if (pHCFTP->bIsUseWaypoints == 0 && pHCFTP->nObstacleNum == 1)
  {
 /******************** 创建单个障碍物运行机构轨迹规划器 ********************/
    
    /** 1.判断开始位置和目标位置是否在障碍物区域内 **/
    bool bflag, bflag1, bflag2;

    bflag1 = (dBridgeStartPos > pHCFTP->Obstacle.dObstacleCoordX) && (dBridgeStartPos < pHCFTP->Obstacle.dObstacleCoordX + pHCFTP->Obstacle.dObstacleCoordW) &&
             (dTrolleyStartPos > pHCFTP->Obstacle.dObstacleCoordY) && (dTrolleyStartPos < pHCFTP->Obstacle.dObstacleCoordY + pHCFTP->Obstacle.dObstacleCoordH);
    bflag2 = (dBridgeTargetPos > pHCFTP->Obstacle.dObstacleCoordX) && (dBridgeTargetPos < pHCFTP->Obstacle.dObstacleCoordX + pHCFTP->Obstacle.dObstacleCoordW) && 
            (dTrolleyTargetPos > pHCFTP->Obstacle.dObstacleCoordY) && (dTrolleyTargetPos < pHCFTP->Obstacle.dObstacleCoordY + pHCFTP->Obstacle.dObstacleCoordH);

    if (bflag1 == 1 || bflag2 == 1)
    {
      // 无法运动
      pHCFTP->pBridgeSubTasksNum = 0;
      pHCFTP->pBridgeTotalTime = -1.0;
      pHCFTP->pTrolleySubTasksNum = 0;
      pHCFTP->pTrolleyTotalTime = -1.0;
      pHCFTP->pTotalTime = -1.0;

      return 0;
    }
    // 判断开始位置和目标位置所确定的矩形区域与障碍物区域是否相交
    SKA_HCFTP_IsOpenRectanglesIntersect(fmin(dBridgeStartPos, dBridgeTargetPos), fmin(dTrolleyStartPos, dTrolleyTargetPos),
                                        fabs(dBridgeTargetPos - dBridgeStartPos), fabs(dTrolleyTargetPos - dTrolleyStartPos),
                                         pHCFTP->Obstacle.dObstacleCoordX, pHCFTP->Obstacle.dObstacleCoordY,
                                         pHCFTP->Obstacle.dObstacleCoordW, pHCFTP->Obstacle.dObstacleCoordH, &(bflag));
    double dfttx, dftty;
    double dfx, dfdx, dfddx;
    if (bflag == 0)
    {

      // 无需避障
      pHCFTP->stSOBridgeSubTask[0].dStartTime = 0.0;
      pHCFTP->stSOBridgeSubTask[0].dStartPos = dBridgeStartPos;
      pHCFTP->stSOBridgeSubTask[0].dTargetPos = dBridgeTargetPos;
      pHCFTP->pBridgeSubTasksNum = 1;

      pHCFTP->stSOTrolleySubTask[0].dStartTime = 0.0;
      pHCFTP->stSOTrolleySubTask[0].dStartPos = dTrolleyStartPos;
      pHCFTP->stSOTrolleySubTask[0].dTargetPos = dTrolleyTargetPos;
      pHCFTP->pTrolleySubTasksNum = 1;

      SKA_TUTP_SingleOGetShapedTraceAndTotalTime(&(pHCFTP->stTracePlanner), 0.0, dBridgeStartPos, dBridgeTargetPos, bBridgeActiveSwayCtrl, BridgeeType, dBridgeNaturalFreq,
                                                  dBridgeDampingRatio, dBridgeMaxVelocity, dBridgeMaxAcceleration, dBridgeMaxJerk, dBridgeQuickStopMaxAcc,
                                                  &(dfx), &(dfdx), &(dfddx), &(dfttx));
                                                  
      SKA_TUTP_SingleOGetShapedTraceAndTotalTime(&(pHCFTP->stTracePlanner), 0.0, dTrolleyStartPos, dTrolleyTargetPos, bTrolleyActiveSwayCtrl, TrolleyeType, dTrolleyNaturalFreq,
                                                  dTrolleyDampingRatio, dTrolleyMaxVelocity, dTrolleyMaxAcceleration, dTrolleyMaxJerk, dTrolleyQuickStopMaxAcc,
                                                  &(dfx), &(dfdx), &(dfddx), &(dftty));
      pHCFTP->pTotalTime = fmax(dfttx, dftty);
      
      return 0;
    }

    if (bBridgeActiveSwayCtrl && bTrolleyActiveSwayCtrl)
    {
      SKA_InpSha_Init(&(pHCFTP->stBridgeShaper), BridgeeType, dBridgeNaturalFreq, dBridgeDampingRatio);
      SKA_InpSha_Init(&(pHCFTP->stTrolleyShaper), TrolleyeType, dTrolleyNaturalFreq, dTrolleyDampingRatio);
    }
    printf("2\n");

    // 分别计算从左右两侧绕过的避障轨迹
    if ((dBridgeTargetPos >= dBridgeStartPos) && (dTrolleyTargetPos >= dTrolleyStartPos))
    {
      /*右侧轨迹*/
      if (pHCFTP->Obstacle.dObstacleCoordY < dTrolleyMinPos || pHCFTP->Obstacle.dObstacleCoordX + pHCFTP->Obstacle.dObstacleCoordW > dBridgeMaxPos)
      {
        // 无法从右侧绕过
        pHCFTP->dTotalTime_R = -1.0;
      }
      else
      {
        // 平移

        pHCFTP->tmpObstacle.dObstacleCoordX = pHCFTP->Obstacle.dObstacleCoordX - dBridgeStartPos;
        pHCFTP->tmpObstacle.dObstacleCoordY = pHCFTP->Obstacle.dObstacleCoordY - dTrolleyStartPos;
        pHCFTP->tmpObstacle.dObstacleCoordW = pHCFTP->Obstacle.dObstacleCoordW;
        pHCFTP->tmpObstacle.dObstacleCoordH = pHCFTP->Obstacle.dObstacleCoordH;

        SKA_HCFTP_SingleOGetNormalRightTrace(pHCFTP, (dBridgeTargetPos - dBridgeStartPos),
                                             dBridgeMaxVelocity, dBridgeMaxAcceleration, dBridgeMaxJerk, pHCFTP->stBridgeShaper,
                                             (dTrolleyTargetPos - dTrolleyStartPos), dTrolleyMaxVelocity, dTrolleyMaxAcceleration,
                                             dTrolleyMaxJerk, pHCFTP->stTrolleyShaper, pHCFTP->tmpObstacle,
                                             bBridgeActiveSwayCtrl, BridgeeType, dBridgeNaturalFreq, dBridgeDampingRatio, dBridgeQuickStopMaxAcc,
                                             bTrolleyActiveSwayCtrl, TrolleyeType, dTrolleyNaturalFreq, dTrolleyDampingRatio, dTrolleyQuickStopMaxAcc,
                                             &(pHCFTP->nSubTasksNumX_R), &(pHCFTP->dTotalTimeX_R), &(pHCFTP->nSubTasksNumY_R),
                                             &(pHCFTP->dTotalTimeY_R), &(pHCFTP->dTotalTime_R));
        printf("3\n");
        memcpy(pHCFTP->SubTasksX_R, pHCFTP->stSOBridgeSubTask, sizeof(pHCFTP->stSOBridgeSubTask));
        memcpy(pHCFTP->SubTasksY_R, pHCFTP->stSOTrolleySubTask, sizeof(pHCFTP->stSOTrolleySubTask));

        for (int i = 0; i < pHCFTP->nSubTasksNumX_R; i++)
        {
          pHCFTP->SubTasksX_R[i].dStartPos = pHCFTP->SubTasksX_R[i].dStartPos + dBridgeStartPos;
          pHCFTP->SubTasksX_R[i].dTargetPos = pHCFTP->SubTasksX_R[i].dTargetPos + dBridgeStartPos;
        }
        for (int i = 0; i < pHCFTP->nSubTasksNumY_R; i++)
        {
          pHCFTP->SubTasksY_R[i].dStartPos = pHCFTP->SubTasksY_R[i].dStartPos + dTrolleyStartPos;
          pHCFTP->SubTasksY_R[i].dTargetPos = pHCFTP->SubTasksY_R[i].dTargetPos + dTrolleyStartPos;
        }
      }
      /*左侧轨迹*/
      if (pHCFTP->Obstacle.dObstacleCoordX < dBridgeMinPos || pHCFTP->Obstacle.dObstacleCoordY + pHCFTP->Obstacle.dObstacleCoordH > dTrolleyMaxPos)
      {
        // 无法从左侧绕过
        pHCFTP->dTotalTime_L = -1.0;
      }
      else
      {
        // 平移+关于直线y=x翻转
        pHCFTP->tmpObstacle.dObstacleCoordX = pHCFTP->Obstacle.dObstacleCoordY - dTrolleyStartPos;
        pHCFTP->tmpObstacle.dObstacleCoordY = pHCFTP->Obstacle.dObstacleCoordX - dBridgeStartPos;
        pHCFTP->tmpObstacle.dObstacleCoordW = pHCFTP->Obstacle.dObstacleCoordH;
        pHCFTP->tmpObstacle.dObstacleCoordH = pHCFTP->Obstacle.dObstacleCoordW;
        SKA_HCFTP_SingleOGetNormalRightTrace(pHCFTP, dTrolleyTargetPos - dTrolleyStartPos,
                                             dTrolleyMaxVelocity, dTrolleyMaxAcceleration, dTrolleyMaxJerk, pHCFTP->stBridgeShaper,
                                             dBridgeTargetPos - dBridgeStartPos, dBridgeMaxVelocity, dBridgeMaxAcceleration,
                                             dBridgeMaxJerk, pHCFTP->stTrolleyShaper, pHCFTP->tmpObstacle,
                                             bBridgeActiveSwayCtrl, BridgeeType, dBridgeNaturalFreq, dBridgeDampingRatio, dBridgeQuickStopMaxAcc,
                                             bTrolleyActiveSwayCtrl, TrolleyeType, dTrolleyNaturalFreq, dTrolleyDampingRatio, dTrolleyQuickStopMaxAcc,
                                             &(pHCFTP->nSubTasksNumX_L), &(pHCFTP->dTotalTimeX_L),
                                             &(pHCFTP->nSubTasksNumY_L), &(pHCFTP->dTotalTimeY_L), &(pHCFTP->dTotalTime_L));
        printf("4\n");
        memcpy(pHCFTP->SubTasksX_L, pHCFTP->stSOBridgeSubTask, sizeof(pHCFTP->stSOBridgeSubTask));
        memcpy(pHCFTP->SubTasksY_L, pHCFTP->stSOTrolleySubTask, sizeof(pHCFTP->stSOTrolleySubTask));

        memcpy(pHCFTP->tmpSubTasks, pHCFTP->SubTasksX_L, sizeof(pHCFTP->SubTasksX_L));
        memcpy(pHCFTP->SubTasksX_L, pHCFTP->SubTasksY_L, sizeof(pHCFTP->SubTasksY_L));
        memcpy(pHCFTP->SubTasksY_L, pHCFTP->tmpSubTasks, sizeof(pHCFTP->tmpSubTasks));

        pHCFTP->ntmpNum = pHCFTP->nSubTasksNumX_L;
        pHCFTP->nSubTasksNumX_L = pHCFTP->nSubTasksNumY_L;
        pHCFTP->nSubTasksNumY_L = pHCFTP->ntmpNum;
        pHCFTP->ftmpTotalTime = pHCFTP->dTotalTimeX_L;
        pHCFTP->dTotalTimeX_L = pHCFTP->dTotalTimeY_L;
        pHCFTP->dTotalTimeY_L = pHCFTP->ftmpTotalTime;

        for (int i = 0; i < pHCFTP->nSubTasksNumX_L; i++)
        {
          pHCFTP->SubTasksX_L[i].dStartPos = pHCFTP->SubTasksX_L[i].dStartPos + dBridgeStartPos;
          pHCFTP->SubTasksX_L[i].dTargetPos = pHCFTP->SubTasksX_L[i].dTargetPos + dBridgeStartPos;
        }
        for (int i = 0; i < pHCFTP->nSubTasksNumY_L; i++)
        {
          pHCFTP->SubTasksY_L[i].dStartPos = pHCFTP->SubTasksY_L[i].dStartPos + dTrolleyStartPos;
          pHCFTP->SubTasksY_L[i].dTargetPos = pHCFTP->SubTasksY_L[i].dTargetPos + dTrolleyStartPos;
        }
        

      }
    }
    else if ((dBridgeTargetPos < dBridgeStartPos) && (dTrolleyTargetPos >= dTrolleyStartPos))
    {

      /*右侧轨迹*/
      if ((pHCFTP->Obstacle.dObstacleCoordX + pHCFTP->Obstacle.dObstacleCoordW > dBridgeMaxPos) || 
          (pHCFTP->Obstacle.dObstacleCoordY + pHCFTP->Obstacle.dObstacleCoordH) > dTrolleyMaxPos)
      {
        // 无法从右侧绕过
        pHCFTP->dTotalTime_R = -1.0;
      }
      else
      {
        // 平移+顺时针旋转90度
        pHCFTP->tmpObstacle.dObstacleCoordX = pHCFTP->Obstacle.dObstacleCoordY - dTrolleyStartPos;
        pHCFTP->tmpObstacle.dObstacleCoordY = -(pHCFTP->Obstacle.dObstacleCoordX - dBridgeStartPos + pHCFTP->Obstacle.dObstacleCoordW);
        pHCFTP->tmpObstacle.dObstacleCoordW = pHCFTP->Obstacle.dObstacleCoordH;
        pHCFTP->tmpObstacle.dObstacleCoordH = pHCFTP->Obstacle.dObstacleCoordW;

        SKA_HCFTP_SingleOGetNormalRightTrace(pHCFTP, dTrolleyTargetPos - dTrolleyStartPos,
                                             dTrolleyMaxVelocity, dTrolleyMaxAcceleration, dTrolleyMaxJerk, pHCFTP->stTrolleyShaper,
                                             -(dBridgeTargetPos - dBridgeStartPos), dBridgeMaxVelocity, dBridgeMaxAcceleration,
                                             dBridgeMaxJerk, pHCFTP->stBridgeShaper, pHCFTP->tmpObstacle,
                                             bBridgeActiveSwayCtrl, BridgeeType, dBridgeNaturalFreq, dBridgeDampingRatio, dBridgeQuickStopMaxAcc,
                                             bTrolleyActiveSwayCtrl, TrolleyeType, dTrolleyNaturalFreq, dTrolleyDampingRatio, dTrolleyQuickStopMaxAcc,
                                             &(pHCFTP->nSubTasksNumX_R), &(pHCFTP->dTotalTimeX_R),
                                             &(pHCFTP->nSubTasksNumY_R), &(pHCFTP->dTotalTimeY_R), &(pHCFTP->dTotalTime_R));
        printf("5\n");
        memcpy(pHCFTP->SubTasksX_R, pHCFTP->stSOBridgeSubTask, sizeof(pHCFTP->stSOBridgeSubTask));
        memcpy(pHCFTP->SubTasksY_R, pHCFTP->stSOTrolleySubTask, sizeof(pHCFTP->stSOTrolleySubTask));

        memcpy(pHCFTP->tmpSubTasks, pHCFTP->SubTasksX_R, sizeof(pHCFTP->SubTasksX_R));
        memcpy(pHCFTP->SubTasksX_R, pHCFTP->SubTasksY_R, sizeof(pHCFTP->SubTasksY_R));
        memcpy(pHCFTP->SubTasksY_R, pHCFTP->tmpSubTasks, sizeof(pHCFTP->tmpSubTasks));

        pHCFTP->ntmpNum = pHCFTP->nSubTasksNumX_R;
        pHCFTP->nSubTasksNumX_R = pHCFTP->nSubTasksNumY_R;
        pHCFTP->nSubTasksNumY_R = pHCFTP->ntmpNum;
        pHCFTP->ftmpTotalTime = pHCFTP->dTotalTimeX_R;
        pHCFTP->dTotalTimeX_R = pHCFTP->dTotalTimeY_R;
        pHCFTP->dTotalTimeY_R = pHCFTP->ftmpTotalTime;

        for (int i = 0; i < pHCFTP->nSubTasksNumX_R; i++)
        {
          pHCFTP->SubTasksX_R[i].dStartPos = -pHCFTP->SubTasksX_R[i].dStartPos + dBridgeStartPos;
          pHCFTP->SubTasksX_R[i].dTargetPos = -pHCFTP->SubTasksX_R[i].dTargetPos + dBridgeStartPos;
        }
        for (int i = 0; i < pHCFTP->nSubTasksNumY_R; i++)
        {
          pHCFTP->SubTasksY_R[i].dStartPos = pHCFTP->SubTasksY_R[i].dStartPos + dTrolleyStartPos;
          pHCFTP->SubTasksY_R[i].dTargetPos = pHCFTP->SubTasksY_R[i].dTargetPos + dTrolleyStartPos;
        }
      }

      /*左侧轨迹*/
      if (pHCFTP->Obstacle.dObstacleCoordX < dBridgeMinPos || pHCFTP->Obstacle.dObstacleCoordY < dTrolleyMinPos)
      {

        // 无法从左侧绕过
        pHCFTP->dTotalTime_L = -1.0;
      }
      else
      {
        // 平移+关于直线Y轴翻转
        pHCFTP->tmpObstacle.dObstacleCoordX = -(pHCFTP->Obstacle.dObstacleCoordX - dBridgeStartPos + pHCFTP->Obstacle.dObstacleCoordW);
        pHCFTP->tmpObstacle.dObstacleCoordY = pHCFTP->Obstacle.dObstacleCoordY - dTrolleyStartPos;
        pHCFTP->tmpObstacle.dObstacleCoordW = pHCFTP->Obstacle.dObstacleCoordW;
        pHCFTP->tmpObstacle.dObstacleCoordH = pHCFTP->Obstacle.dObstacleCoordH;
        SKA_HCFTP_SingleOGetNormalRightTrace(pHCFTP, -(dBridgeTargetPos - dBridgeStartPos),
                                             dBridgeMaxVelocity, dBridgeMaxAcceleration, dBridgeMaxJerk, pHCFTP->stBridgeShaper,
                                             dTrolleyTargetPos - dTrolleyStartPos, dTrolleyMaxVelocity, dTrolleyMaxAcceleration,
                                             dTrolleyMaxJerk, pHCFTP->stTrolleyShaper, pHCFTP->tmpObstacle,
                                             bBridgeActiveSwayCtrl, BridgeeType, dBridgeNaturalFreq, dBridgeDampingRatio, dBridgeQuickStopMaxAcc,
                                             bTrolleyActiveSwayCtrl, TrolleyeType, dTrolleyNaturalFreq, dTrolleyDampingRatio, dTrolleyQuickStopMaxAcc,
                                             &(pHCFTP->nSubTasksNumX_L), &(pHCFTP->dTotalTimeX_L),
                                             &(pHCFTP->nSubTasksNumY_L), &(pHCFTP->dTotalTimeY_L), &(pHCFTP->dTotalTime_L));
        printf("6\n");
        memcpy(pHCFTP->SubTasksX_L, pHCFTP->stSOBridgeSubTask, sizeof(pHCFTP->stSOBridgeSubTask));
        memcpy(pHCFTP->SubTasksY_L, pHCFTP->stSOTrolleySubTask, sizeof(pHCFTP->stSOTrolleySubTask));

        for (int i = 0; i < pHCFTP->nSubTasksNumX_L; i++)
        {
          pHCFTP->SubTasksX_L[i].dStartPos = -pHCFTP->SubTasksX_L[i].dStartPos + dBridgeStartPos;
          pHCFTP->SubTasksX_L[i].dTargetPos = -pHCFTP->SubTasksX_L[i].dTargetPos + dBridgeStartPos;
        }
        for (int i = 0; i < pHCFTP->nSubTasksNumY_L; i++)
        {
          pHCFTP->SubTasksY_L[i].dStartPos = pHCFTP->SubTasksY_L[i].dStartPos + dTrolleyStartPos;
          pHCFTP->SubTasksY_L[i].dTargetPos = pHCFTP->SubTasksY_L[i].dTargetPos + dTrolleyStartPos;
        }
      }
    }
    else if ((dBridgeTargetPos < dBridgeStartPos) && (dTrolleyTargetPos < dTrolleyStartPos))
    {

      /*右侧轨迹*/ //---
      if ((pHCFTP->Obstacle.dObstacleCoordX < dBridgeMinPos) || (pHCFTP->Obstacle.dObstacleCoordY + pHCFTP->Obstacle.dObstacleCoordH > dTrolleyMaxPos))
      {
        // 无法从右侧绕过
        pHCFTP->dTotalTime_R = -1.0;
      }
      else
      {
        // 平移+关于原点对称
        pHCFTP->tmpObstacle.dObstacleCoordX = -(pHCFTP->Obstacle.dObstacleCoordX - dBridgeStartPos + pHCFTP->Obstacle.dObstacleCoordW);
        pHCFTP->tmpObstacle.dObstacleCoordY = -(pHCFTP->Obstacle.dObstacleCoordY - dTrolleyStartPos + pHCFTP->Obstacle.dObstacleCoordH);
        pHCFTP->tmpObstacle.dObstacleCoordW = pHCFTP->Obstacle.dObstacleCoordW;
        pHCFTP->tmpObstacle.dObstacleCoordH = pHCFTP->Obstacle.dObstacleCoordH;
        SKA_HCFTP_SingleOGetNormalRightTrace(pHCFTP, -(dBridgeTargetPos - dBridgeStartPos),
                                             dBridgeMaxVelocity, dBridgeMaxAcceleration, dBridgeMaxJerk, pHCFTP->stBridgeShaper,
                                             -(dTrolleyTargetPos - dTrolleyStartPos), dTrolleyMaxVelocity, dTrolleyMaxAcceleration,
                                             dTrolleyMaxJerk, pHCFTP->stTrolleyShaper, pHCFTP->tmpObstacle,
                                             bBridgeActiveSwayCtrl, BridgeeType, dBridgeNaturalFreq, dBridgeDampingRatio, dBridgeQuickStopMaxAcc,
                                             bTrolleyActiveSwayCtrl, TrolleyeType, dTrolleyNaturalFreq, dTrolleyDampingRatio, dTrolleyQuickStopMaxAcc,
                                             &(pHCFTP->nSubTasksNumX_R), &(pHCFTP->dTotalTimeX_R),
                                             &(pHCFTP->nSubTasksNumY_R), &(pHCFTP->dTotalTimeY_R), &(pHCFTP->dTotalTime_R));
        printf("7\n");
        memcpy(pHCFTP->SubTasksX_R, pHCFTP->stSOBridgeSubTask, sizeof(pHCFTP->stSOBridgeSubTask));
        memcpy(pHCFTP->SubTasksY_R, pHCFTP->stSOTrolleySubTask, sizeof(pHCFTP->stSOTrolleySubTask));

        for (int i = 0; i < pHCFTP->nSubTasksNumX_R; i++)
        {
          pHCFTP->SubTasksX_R[i].dStartPos = -pHCFTP->SubTasksX_R[i].dStartPos + dBridgeStartPos;
          pHCFTP->SubTasksX_R[i].dTargetPos = -pHCFTP->SubTasksX_R[i].dTargetPos + dBridgeStartPos;
        }

        for (int i = 0; i < pHCFTP->nSubTasksNumY_R; i++)
        {
          pHCFTP->SubTasksY_R[i].dStartPos = -pHCFTP->SubTasksY_R[i].dStartPos + dTrolleyStartPos;
          pHCFTP->SubTasksY_R[i].dTargetPos = -pHCFTP->SubTasksY_R[i].dTargetPos + dTrolleyStartPos;
        }
      }

      /*左侧轨迹*/
      if (pHCFTP->Obstacle.dObstacleCoordX + pHCFTP->Obstacle.dObstacleCoordW > dBridgeMaxPos || pHCFTP->Obstacle.dObstacleCoordY < dTrolleyMinPos)
      {
        // 无法从左侧绕过
        pHCFTP->dTotalTime_L = -1.0;
      }
      else
      {
        // 平移+关于y=-x对称
        pHCFTP->tmpObstacle.dObstacleCoordX = -(pHCFTP->Obstacle.dObstacleCoordY - dTrolleyStartPos + pHCFTP->Obstacle.dObstacleCoordH);
        pHCFTP->tmpObstacle.dObstacleCoordY = -(pHCFTP->Obstacle.dObstacleCoordX - dBridgeStartPos + pHCFTP->Obstacle.dObstacleCoordW);
        pHCFTP->tmpObstacle.dObstacleCoordW = pHCFTP->Obstacle.dObstacleCoordH;
        pHCFTP->tmpObstacle.dObstacleCoordH = pHCFTP->Obstacle.dObstacleCoordW;
        SKA_HCFTP_SingleOGetNormalRightTrace(pHCFTP, -(dTrolleyTargetPos - dTrolleyStartPos),
                                             dTrolleyMaxVelocity, dTrolleyMaxAcceleration, dTrolleyMaxJerk, pHCFTP->stTrolleyShaper,
                                             -(dBridgeTargetPos - dBridgeStartPos), dBridgeMaxVelocity, dBridgeMaxAcceleration,
                                             dBridgeMaxJerk, pHCFTP->stBridgeShaper, pHCFTP->tmpObstacle,
                                             bBridgeActiveSwayCtrl, BridgeeType, dBridgeNaturalFreq, dBridgeDampingRatio, dBridgeQuickStopMaxAcc,
                                             bTrolleyActiveSwayCtrl, TrolleyeType, dTrolleyNaturalFreq, dTrolleyDampingRatio, dTrolleyQuickStopMaxAcc,
                                             &(pHCFTP->nSubTasksNumX_L), &(pHCFTP->dTotalTimeX_L),
                                             &(pHCFTP->nSubTasksNumY_L), &(pHCFTP->dTotalTimeY_L), &(pHCFTP->dTotalTime_L));
        printf("8\n");
        memcpy(pHCFTP->SubTasksX_L, pHCFTP->stSOBridgeSubTask, sizeof(pHCFTP->stSOBridgeSubTask));
        memcpy(pHCFTP->SubTasksY_L, pHCFTP->stSOTrolleySubTask, sizeof(pHCFTP->stSOTrolleySubTask));

        memcpy(pHCFTP->tmpSubTasks, pHCFTP->SubTasksX_L, sizeof(pHCFTP->SubTasksX_L));
        memcpy(pHCFTP->SubTasksX_L, pHCFTP->SubTasksY_L, sizeof(pHCFTP->SubTasksY_L));
        memcpy(pHCFTP->SubTasksY_L, pHCFTP->tmpSubTasks, sizeof(pHCFTP->tmpSubTasks));

        pHCFTP->ntmpNum = pHCFTP->nSubTasksNumX_L;
        pHCFTP->nSubTasksNumX_L = pHCFTP->nSubTasksNumY_L;
        pHCFTP->nSubTasksNumY_L = pHCFTP->ntmpNum;
        pHCFTP->ftmpTotalTime = pHCFTP->dTotalTimeX_L;
        pHCFTP->dTotalTimeX_L = pHCFTP->dTotalTimeY_L;
        pHCFTP->dTotalTimeY_L = pHCFTP->ftmpTotalTime;

        for (int i = 0; i < pHCFTP->nSubTasksNumX_L; i++)
        {
          pHCFTP->SubTasksX_L[i].dStartPos = -pHCFTP->SubTasksX_L[i].dStartPos + dBridgeStartPos;
          pHCFTP->SubTasksX_L[i].dTargetPos = -pHCFTP->SubTasksX_L[i].dTargetPos + dBridgeStartPos;
        }

        for (int i = 0; i < pHCFTP->nSubTasksNumY_L; i++)
        {
          pHCFTP->SubTasksY_L[i].dStartPos = -pHCFTP->SubTasksY_L[i].dStartPos + dTrolleyStartPos;
          pHCFTP->SubTasksY_L[i].dTargetPos = -pHCFTP->SubTasksY_L[i].dTargetPos + dTrolleyStartPos;
        }
      }
    }
    else
    {
      /*右侧轨迹*/
      if ((pHCFTP->Obstacle.dObstacleCoordX < dBridgeMinPos) || (pHCFTP->Obstacle.dObstacleCoordY < dTrolleyMinPos))
      {

        // 无法从右侧绕过
        pHCFTP->dTotalTime_R = -1.0;
      }
      else
      {

        // 平移+逆时针旋转90度
        pHCFTP->tmpObstacle.dObstacleCoordX = -(pHCFTP->Obstacle.dObstacleCoordY - dTrolleyStartPos + pHCFTP->Obstacle.dObstacleCoordH);
        pHCFTP->tmpObstacle.dObstacleCoordY = pHCFTP->Obstacle.dObstacleCoordX - dBridgeStartPos;
        pHCFTP->tmpObstacle.dObstacleCoordW = pHCFTP->Obstacle.dObstacleCoordH;
        pHCFTP->tmpObstacle.dObstacleCoordH = pHCFTP->Obstacle.dObstacleCoordW;
        SKA_HCFTP_SingleOGetNormalRightTrace(pHCFTP, -(dTrolleyTargetPos - dTrolleyStartPos),
                                             dTrolleyMaxVelocity, dTrolleyMaxAcceleration, dTrolleyMaxJerk, pHCFTP->stTrolleyShaper,
                                             dBridgeTargetPos - dBridgeStartPos, dBridgeMaxVelocity, dBridgeMaxAcceleration,
                                             dBridgeMaxJerk, pHCFTP->stBridgeShaper, pHCFTP->tmpObstacle,
                                             bBridgeActiveSwayCtrl, BridgeeType, dBridgeNaturalFreq, dBridgeDampingRatio, dBridgeQuickStopMaxAcc,
                                             bTrolleyActiveSwayCtrl, TrolleyeType, dTrolleyNaturalFreq, dTrolleyDampingRatio, dTrolleyQuickStopMaxAcc,
                                             &(pHCFTP->nSubTasksNumX_R), &(pHCFTP->dTotalTimeX_R),
                                             &(pHCFTP->nSubTasksNumY_R), &(pHCFTP->dTotalTimeY_R), &(pHCFTP->dTotalTime_R));
        printf("9\n");
        memcpy(pHCFTP->SubTasksX_R, pHCFTP->stSOBridgeSubTask, sizeof(pHCFTP->stSOBridgeSubTask));
        memcpy(pHCFTP->SubTasksY_R, pHCFTP->stSOTrolleySubTask, sizeof(pHCFTP->stSOTrolleySubTask));

        memcpy(pHCFTP->tmpSubTasks, pHCFTP->SubTasksX_R, sizeof(pHCFTP->SubTasksX_R));
        memcpy(pHCFTP->SubTasksX_R, pHCFTP->SubTasksY_R, sizeof(pHCFTP->SubTasksY_R));
        memcpy(pHCFTP->SubTasksY_R, pHCFTP->tmpSubTasks, sizeof(pHCFTP->tmpSubTasks));

        pHCFTP->ntmpNum = pHCFTP->nSubTasksNumX_R;
        pHCFTP->nSubTasksNumX_R = pHCFTP->nSubTasksNumY_R;
        pHCFTP->nSubTasksNumY_R = pHCFTP->ntmpNum;
        pHCFTP->ftmpTotalTime = pHCFTP->dTotalTimeX_R;
        pHCFTP->dTotalTimeX_R = pHCFTP->dTotalTimeY_R;
        pHCFTP->dTotalTimeY_R = pHCFTP->ftmpTotalTime;

        for (int i = 0; i < pHCFTP->nSubTasksNumX_R; i++)
        {
          pHCFTP->SubTasksX_R[i].dStartPos = pHCFTP->SubTasksX_R[i].dStartPos + dBridgeStartPos;
          pHCFTP->SubTasksX_R[i].dTargetPos = pHCFTP->SubTasksX_R[i].dTargetPos + dBridgeStartPos;
        }
        for (int i = 0; i < pHCFTP->nSubTasksNumY_R; i++)
        {
          pHCFTP->SubTasksY_R[i].dStartPos = -pHCFTP->SubTasksY_R[i].dStartPos + dTrolleyStartPos;
          pHCFTP->SubTasksY_R[i].dTargetPos = -pHCFTP->SubTasksY_R[i].dTargetPos + dTrolleyStartPos;
        }
      }

      /*左侧轨迹*/
      if (pHCFTP->Obstacle.dObstacleCoordX + pHCFTP->Obstacle.dObstacleCoordW > dBridgeMaxPos || 
          pHCFTP->Obstacle.dObstacleCoordY + pHCFTP->Obstacle.dObstacleCoordH > dTrolleyMaxPos)
      {
        // 无法从左侧绕过
        pHCFTP->dTotalTime_L = -1.0;
      }
      else
      {

        // 平移+关于直线Y轴翻转
        pHCFTP->tmpObstacle.dObstacleCoordX = pHCFTP->Obstacle.dObstacleCoordX - dBridgeStartPos;
        pHCFTP->tmpObstacle.dObstacleCoordY = -(pHCFTP->Obstacle.dObstacleCoordY - dTrolleyStartPos + pHCFTP->Obstacle.dObstacleCoordH);
        pHCFTP->tmpObstacle.dObstacleCoordW = pHCFTP->Obstacle.dObstacleCoordW;
        pHCFTP->tmpObstacle.dObstacleCoordH = pHCFTP->Obstacle.dObstacleCoordH;

        SKA_HCFTP_SingleOGetNormalRightTrace(pHCFTP, (dBridgeTargetPos - dBridgeStartPos),
                                             dBridgeMaxVelocity, dBridgeMaxAcceleration, dBridgeMaxJerk, pHCFTP->stBridgeShaper,
                                             -(dTrolleyTargetPos - dTrolleyStartPos), dTrolleyMaxVelocity, dTrolleyMaxAcceleration,
                                             dTrolleyMaxJerk, pHCFTP->stTrolleyShaper, pHCFTP->tmpObstacle,
                                             bBridgeActiveSwayCtrl, BridgeeType, dBridgeNaturalFreq, dBridgeDampingRatio, dBridgeQuickStopMaxAcc,
                                             bTrolleyActiveSwayCtrl, TrolleyeType, dTrolleyNaturalFreq, dTrolleyDampingRatio, dTrolleyQuickStopMaxAcc,
                                             &(pHCFTP->nSubTasksNumX_L), &(pHCFTP->dTotalTimeX_L),
                                             &(pHCFTP->nSubTasksNumY_L), &(pHCFTP->dTotalTimeY_L), &(pHCFTP->dTotalTime_L));
        printf("10\n");
        memcpy(pHCFTP->SubTasksX_L, pHCFTP->stSOBridgeSubTask, sizeof(pHCFTP->stSOBridgeSubTask));
        memcpy(pHCFTP->SubTasksY_L, pHCFTP->stSOTrolleySubTask, sizeof(pHCFTP->stSOTrolleySubTask));

        for (int i = 0; i < pHCFTP->nSubTasksNumX_L; i++)
        {
          pHCFTP->SubTasksX_L[i].dStartPos = pHCFTP->SubTasksX_L[i].dStartPos + dBridgeStartPos;
          pHCFTP->SubTasksX_L[i].dTargetPos = pHCFTP->SubTasksX_L[i].dTargetPos + dBridgeStartPos;
        }
        for (int i = 0; i < pHCFTP->nSubTasksNumY_L; i++)
        {
          pHCFTP->SubTasksY_L[i].dStartPos = -pHCFTP->SubTasksY_L[i].dStartPos + dTrolleyStartPos;
          pHCFTP->SubTasksY_L[i].dTargetPos = -pHCFTP->SubTasksY_L[i].dTargetPos + dTrolleyStartPos;
        }
      }
    }
    // 选择最短时间的轨迹
    if (pHCFTP->dTotalTime_R < 0 && pHCFTP->dTotalTime_L < 0)
    {
      // 无合法轨迹

      pHCFTP->pBridgeSubTasksNum = 0;
      pHCFTP->pBridgeTotalTime = -1;
      pHCFTP->pTrolleySubTasksNum = 0;
      pHCFTP->pTrolleyTotalTime = -1;
      pHCFTP->pTotalTime = -1;
    }
    else if (pHCFTP->dTotalTime_R < 0)
    {
    

      memcpy(pHCFTP->stSOBridgeSubTask, pHCFTP->SubTasksX_L, sizeof(pHCFTP->SubTasksX_L));
      pHCFTP->pBridgeSubTasksNum = pHCFTP->nSubTasksNumX_L;
      pHCFTP->pBridgeTotalTime = pHCFTP->dTotalTimeX_L;
      memcpy(pHCFTP->stSOTrolleySubTask, pHCFTP->SubTasksY_L, sizeof(pHCFTP->SubTasksY_L));
      pHCFTP->pTrolleySubTasksNum = pHCFTP->nSubTasksNumY_L;
      pHCFTP->pTrolleyTotalTime = pHCFTP->dTotalTimeY_L;
      pHCFTP->pTotalTime = pHCFTP->dTotalTime_L;
    }
    else if (pHCFTP->dTotalTime_L < 0)
    {

      memcpy(pHCFTP->stSOBridgeSubTask, pHCFTP->SubTasksX_R, sizeof(pHCFTP->SubTasksX_R));
      pHCFTP->pBridgeSubTasksNum = pHCFTP->nSubTasksNumX_R;
      pHCFTP->pBridgeTotalTime = pHCFTP->dTotalTimeX_R;
      memcpy(pHCFTP->stSOTrolleySubTask, pHCFTP->SubTasksY_R, sizeof(pHCFTP->SubTasksY_R));
      pHCFTP->pTrolleySubTasksNum = pHCFTP->nSubTasksNumY_R;
      pHCFTP->pTrolleyTotalTime = pHCFTP->dTotalTimeY_R;
      pHCFTP->pTotalTime = pHCFTP->dTotalTime_R;
    }
    else
    {

      if (pHCFTP->dTotalTime_R <= pHCFTP->dTotalTime_L)
      {
        memcpy(pHCFTP->stSOBridgeSubTask, pHCFTP->SubTasksX_R, sizeof(pHCFTP->SubTasksX_R));
        pHCFTP->pBridgeSubTasksNum = pHCFTP->nSubTasksNumX_R;
        pHCFTP->pBridgeTotalTime = pHCFTP->dTotalTimeX_R;
        memcpy(pHCFTP->stSOTrolleySubTask, pHCFTP->SubTasksY_R, sizeof(pHCFTP->SubTasksY_R));
        pHCFTP->pTrolleySubTasksNum = pHCFTP->nSubTasksNumY_R;
        pHCFTP->pTrolleyTotalTime = pHCFTP->dTotalTimeY_R;
        pHCFTP->pTotalTime = pHCFTP->dTotalTime_R;
      }
      else
      {
        memcpy(pHCFTP->stSOBridgeSubTask, pHCFTP->SubTasksX_L, sizeof(pHCFTP->SubTasksX_L));
        pHCFTP->pBridgeSubTasksNum = pHCFTP->nSubTasksNumX_L;
        pHCFTP->pBridgeTotalTime = pHCFTP->dTotalTimeX_L;
        memcpy(pHCFTP->stSOTrolleySubTask, pHCFTP->SubTasksY_L, sizeof(pHCFTP->SubTasksY_L));
        pHCFTP->pTrolleySubTasksNum = pHCFTP->nSubTasksNumY_L;
        pHCFTP->pTrolleyTotalTime = pHCFTP->dTotalTimeY_L;
        pHCFTP->pTotalTime = pHCFTP->dTotalTime_L;
      }
    }
    printf("pHCFTP->pBridgeSubTasksNum: %d\n",pHCFTP->pBridgeSubTasksNum);
    printf("pHCFTP->pBridgeTotalTime: %lf\n",pHCFTP->pBridgeTotalTime);
    printf("pHCFTP->stSOBridgeSubTask[0].dStartTime: %lf\n",pHCFTP->stSOBridgeSubTask[0].dStartTime);
    printf("pHCFTP->stSOBridgeSubTask[0].dStartPos: %lf\n",pHCFTP->stSOBridgeSubTask[0].dStartPos);
    printf("pHCFTP->stSOBridgeSubTask[0].dTargetPos: %lf\n",pHCFTP->stSOBridgeSubTask[0].dTargetPos);
    
    printf("pHCFTP->pTrolleySubTasksNum: %d\n",pHCFTP->pTrolleySubTasksNum);
    printf("pHCFTP->pTrolleyTotalTime: %lf\n",pHCFTP->pTrolleyTotalTime);
    printf("pHCFTP->stSOTrolleySubTask[0].dStartTime: %lf\n",pHCFTP->stSOTrolleySubTask[0].dStartTime);
    printf("pHCFTP->stSOTrolleySubTask[0].dStartPos: %lf\n",pHCFTP->stSOTrolleySubTask[0].dStartPos);
    printf("pHCFTP->stSOTrolleySubTask[0].dTargetPos: %lf\n",pHCFTP->stSOTrolleySubTask[0].dTargetPos);

  } 
  else{
      // 生成路径点
    if (pHCFTP->bIsUseWaypoints == 0)
    {
      // 使用障碍物生成路径点
      memcpy(pHCFTP->ArrObstacles, pHCFTP->MObstacles, sizeof(pHCFTP->MObstacles));

      SKA_HCFTP_MultipleOGetWayPoints(pHCFTP, dBridgeStartPos, dBridgeTargetPos,
                                      dBridgeMinPos, dBridgeMaxPos, dTrolleyStartPos, dTrolleyTargetPos,
                                      dTrolleyMinPos, dTrolleyMaxPos, pHCFTP->nObstacleNum, &(pHCFTP->waypointsNum));
      memcpy(pHCFTP->waypoints, pHCFTP->ArrWayPoints, sizeof(pHCFTP->ArrWayPoints));
    }
    else
    {
      // 使用中途路径点生成路径点
      pHCFTP->waypointsNum = pHCFTP->nWaypointsNum + 2;

      pHCFTP->waypoints[0].dWaypointsCoordX = dBridgeStartPos;
      pHCFTP->waypoints[0].dWaypointsCoordY = dTrolleyStartPos;
      pHCFTP->waypoints[pHCFTP->waypointsNum - 1].dWaypointsCoordX = dBridgeTargetPos;
      pHCFTP->waypoints[pHCFTP->waypointsNum - 1].dWaypointsCoordY = dTrolleyTargetPos;

      if (pHCFTP->nWaypointsNum > 0)
      {
        for (int i = 1; i <= pHCFTP->waypointsNum - 2; i++)
        {
          pHCFTP->waypoints[i].dWaypointsCoordX = pHCFTP->MidWaypoints[i - 1].dWaypointsCoordX;
          pHCFTP->waypoints[i].dWaypointsCoordY = pHCFTP->MidWaypoints[i - 1].dWaypointsCoordY;
        }
      }
    }

    // 无合法路径
    //  处理路径点数量为0的情况
    if (pHCFTP->waypointsNum == 0)
    {
      pHCFTP->pBridgeSubTasksNum = 0;
      pHCFTP->pBridgeTotalTime = -1;
      pHCFTP->pTrolleySubTasksNum = 0;
      pHCFTP->pTrolleyTotalTime = -1;
      pHCFTP->pTotalTime = -1;
      return 0;
    }

    /*计算路径点的速度方向和运动模式*/
    // 确定路径点速度方向（0静止 1正向 -1反向）
    // 起点和终点
    pHCFTP->motionDirect[0].nx = 0;
    pHCFTP->motionDirect[0].ny = 0;
    pHCFTP->motionDirect[pHCFTP->waypointsNum - 1].nx = 0;
    pHCFTP->motionDirect[pHCFTP->waypointsNum - 1].ny = 0;

    if (pHCFTP->waypointsNum > 2)
    {
      for (int i = 1; i <= pHCFTP->waypointsNum - 2; i++)
      {
        // 上一个路径点的限制
        //  X方向
        if (fabs(pHCFTP->waypoints[i].dWaypointsCoordX - pHCFTP->waypoints[i - 1].dWaypointsCoordX) < SKA_FlOAT_ERROR)
        {
          pHCFTP->motionDirect[i].nx = 0;
        }
        else if (pHCFTP->waypoints[i].dWaypointsCoordX > pHCFTP->waypoints[i - 1].dWaypointsCoordX)
        {
          pHCFTP->motionDirect[i].nx = 1;
        }
        else
        {
          pHCFTP->motionDirect[i].nx = -1;
        }

        // Y方向
        if (fabs(pHCFTP->waypoints[i].dWaypointsCoordY - pHCFTP->waypoints[i - 1].dWaypointsCoordY) < SKA_FlOAT_ERROR)
        {
          pHCFTP->motionDirect[i].ny = 0;
        }
        else if (pHCFTP->waypoints[i].dWaypointsCoordY > pHCFTP->waypoints[i - 1].dWaypointsCoordY)
        {
          pHCFTP->motionDirect[i].ny = 1;
        }
        else
        {
          pHCFTP->motionDirect[i].ny = -1;
        }

        // 下一个路径点的限制
        //  X方向
        if (fabs(pHCFTP->waypoints[i + 1].dWaypointsCoordX - pHCFTP->waypoints[i].dWaypointsCoordX) < SKA_FlOAT_ERROR)
        {
          pHCFTP->motionDirect[i].nx = 0;
        }
        else if (pHCFTP->waypoints[i + 1].dWaypointsCoordX > pHCFTP->waypoints[i].dWaypointsCoordX)
        {

          if (pHCFTP->motionDirect[i].nx == -1)
          {
            pHCFTP->motionDirect[i].nx = 0;
          }
        }
        else
        {

          if (pHCFTP->motionDirect[i].nx == 1)
          {
            pHCFTP->motionDirect[i].nx = 0;
          }
        }

        // Y方向
        if (fabs(pHCFTP->waypoints[i + 1].dWaypointsCoordY - pHCFTP->waypoints[i].dWaypointsCoordY) < SKA_FlOAT_ERROR)
        {
          pHCFTP->motionDirect[i].ny = 0;
        }
        else if (pHCFTP->waypoints[i + 1].dWaypointsCoordY > pHCFTP->waypoints[i].dWaypointsCoordY)
        {

          if (pHCFTP->motionDirect[i].ny == -1)
          {
            pHCFTP->motionDirect[i].ny = 0;
          }
        }
        else
        {

          if (pHCFTP->motionDirect[i].ny == 1)
          {
            pHCFTP->motionDirect[i].ny = 0;
          }
        }

        // 限制路径点在X方向和Y方向不能同时运动
        if (pHCFTP->motionDirect[i].nx != 0 && pHCFTP->motionDirect[i].ny != 0)
        {
          pHCFTP->motionDirect[i].ny = 0;
        }
      }
    }

    /*确定子任务运动模式*/
    // 0静止->静止  1静止->运动 2运动->静止  3运动->运动
    for (int i = 0; i <= pHCFTP->waypointsNum - 2; i++)
    {
      // X方向
      if (pHCFTP->motionDirect[i].nx == 0 && pHCFTP->motionDirect[i + 1].nx == 0)
      {
        pHCFTP->motionMode[i].nx = 0;
      }
      else if (pHCFTP->motionDirect[i].nx == 0 && pHCFTP->motionDirect[i + 1].nx != 0)
      {
        pHCFTP->motionMode[i].nx = 1;
      }
      else if (pHCFTP->motionDirect[i].nx != 0 && pHCFTP->motionDirect[i + 1].nx == 0)
      {
        pHCFTP->motionMode[i].nx = 2;
      }
      else
      {
        pHCFTP->motionMode[i].nx = 3;
      }

      // Y方向
      if (pHCFTP->motionDirect[i].ny == 0 && pHCFTP->motionDirect[i + 1].ny == 0)
      {
        pHCFTP->motionMode[i].ny = 0;
      }
      else if (pHCFTP->motionDirect[i].ny == 0 && pHCFTP->motionDirect[i + 1].ny != 0)
      {
        pHCFTP->motionMode[i].ny = 1;
      }
      else if (pHCFTP->motionDirect[i].ny != 0 && pHCFTP->motionDirect[i + 1].ny == 0)
      {
        pHCFTP->motionMode[i].ny = 2;
      }
      else
      {
        pHCFTP->motionMode[i].ny = 3;
      }
    }

    if (bBridgeActiveSwayCtrl && bTrolleyActiveSwayCtrl)
    {
      SKA_InpSha_Init(&(pHCFTP->stBridgeShaper), BridgeeType, dBridgeNaturalFreq, dBridgeDampingRatio);
      SKA_InpSha_Init(&(pHCFTP->stTrolleyShaper), TrolleyeType, dTrolleyNaturalFreq, dTrolleyDampingRatio);
    }
    // 计算路径点的速度大小
    memcpy(pHCFTP->arrWayPoints, pHCFTP->waypoints, sizeof(pHCFTP->waypoints));
    memcpy(pHCFTP->MotionMode, pHCFTP->motionMode, sizeof(pHCFTP->motionMode));

    SKA_HCFTP_MultipleOGetWayPointsVelocity(pHCFTP, pHCFTP->waypointsNum, bBridgeActiveSwayCtrl, BridgeeType,
                                            dBridgeNaturalFreq, dBridgeDampingRatio, dBridgeQuickStopMaxAcc, dBridgeMaxVelocity, dBridgeMaxAcceleration,
                                            dBridgeMaxJerk, pHCFTP->stBridgeShaper, bTrolleyActiveSwayCtrl, TrolleyeType, dTrolleyNaturalFreq,
                                            dTrolleyDampingRatio, dTrolleyQuickStopMaxAcc, dTrolleyMaxVelocity, dTrolleyMaxAcceleration, dTrolleyMaxJerk, pHCFTP->stTrolleyShaper);

    memcpy(pHCFTP->waypointsVelocity, pHCFTP->OutputWayPointsVelocity, sizeof(pHCFTP->OutputWayPointsVelocity));

    // 生成子任务计算
    pHCFTP->pBridgeSubTasksNum = pHCFTP->waypointsNum - 1;
    pHCFTP->pTrolleySubTasksNum = pHCFTP->waypointsNum - 1;
    pHCFTP->pTotalTime = 0.0;

    // 生成子任务
    double dtmp, dtmp1, dtmp2, dttx, dtty;
    for (int i = 0; i <= pHCFTP->waypointsNum - 2; i++)
    {

      // X方向任务

      pHCFTP->stBridgeSubTask[i].dStartPos = pHCFTP->waypoints[i].dWaypointsCoordX;
      pHCFTP->stBridgeSubTask[i].dTargetPos = pHCFTP->waypoints[i + 1].dWaypointsCoordX;
      pHCFTP->stBridgeSubTask[i].dStartVelocity = pHCFTP->waypointsVelocity[i].dWaypointsCoordX;
      pHCFTP->stBridgeSubTask[i].dTargetVelocity = pHCFTP->waypointsVelocity[i + 1].dWaypointsCoordX;
      if (pHCFTP->motionMode[i].nx == 3)
      {

        SKA_TUTP_SingleOGetShapedTraceAndTotalTime(&(pHCFTP->stTracePlanner), 0.0, pHCFTP->waypoints[i].dWaypointsCoordY, pHCFTP->waypoints[i + 1].dWaypointsCoordY,
                                                    bTrolleyActiveSwayCtrl, TrolleyeType, dTrolleyNaturalFreq, dTrolleyDampingRatio, dTrolleyMaxVelocity,
                                                    dTrolleyMaxAcceleration, dTrolleyMaxJerk, dTrolleyQuickStopMaxAcc, &(dtmp), &(dtmp1),
                                                    &(dtmp2), &(pHCFTP->stBridgeSubTask[i].dMinTime));

        SKA_HCFTP_MultipleOGetShapedTrace(pHCFTP, 0.0, pHCFTP->waypoints[i].dWaypointsCoordX, pHCFTP->waypoints[i + 1].dWaypointsCoordX, pHCFTP->waypointsVelocity[i].dWaypointsCoordX,
                                          pHCFTP->waypointsVelocity[i + 1].dWaypointsCoordX, pHCFTP->stBridgeSubTask[i].dMinTime, 
                                          bBridgeActiveSwayCtrl, BridgeeType, dBridgeNaturalFreq, dBridgeDampingRatio, dBridgeMaxVelocity, dBridgeMaxAcceleration,
                                          dBridgeMaxJerk, pHCFTP->stBridgeShaper, &(dtmp), &(dtmp1), &(dtmp2), &(dttx));
      }
      else
      {
        pHCFTP->stBridgeSubTask[i].dMinTime = -1;
        SKA_HCFTP_MultipleOGetShapedTrace(pHCFTP, 0.0, pHCFTP->waypoints[i].dWaypointsCoordX, pHCFTP->waypoints[i + 1].dWaypointsCoordX,
                                          pHCFTP->waypointsVelocity[i].dWaypointsCoordX, pHCFTP->waypointsVelocity[i + 1].dWaypointsCoordX, pHCFTP->stBridgeSubTask[i].dMinTime,
                                          bBridgeActiveSwayCtrl, BridgeeType, dBridgeNaturalFreq, dBridgeDampingRatio, dBridgeMaxVelocity, dBridgeMaxAcceleration, dBridgeMaxJerk, pHCFTP->stBridgeShaper,
                                          &(dtmp), &(dtmp1), &(dtmp2), &(dttx));
      }

      // Y方向任务
      pHCFTP->stTrolleySubTask[i].dStartPos = pHCFTP->waypoints[i].dWaypointsCoordY;
      pHCFTP->stTrolleySubTask[i].dTargetPos = pHCFTP->waypoints[i + 1].dWaypointsCoordY;
      pHCFTP->stTrolleySubTask[i].dStartVelocity = pHCFTP->waypointsVelocity[i].dWaypointsCoordY;
      pHCFTP->stTrolleySubTask[i].dTargetVelocity = pHCFTP->waypointsVelocity[i + 1].dWaypointsCoordY;
      if (pHCFTP->motionMode[i].ny == 3)
      {

        SKA_TUTP_SingleOGetShapedTraceAndTotalTime(&(pHCFTP->stTracePlanner), 0.0, pHCFTP->waypoints[i].dWaypointsCoordX, pHCFTP->waypoints[i + 1].dWaypointsCoordX,
                                                    bBridgeActiveSwayCtrl, BridgeeType, dBridgeNaturalFreq, dBridgeDampingRatio, dBridgeMaxVelocity, dBridgeMaxAcceleration, dBridgeMaxJerk,
                                                    dBridgeQuickStopMaxAcc, &(dtmp), &(dtmp1), &(dtmp2), &(pHCFTP->stTrolleySubTask[i].dMinTime));
        SKA_HCFTP_MultipleOGetShapedTrace(pHCFTP, 0.0, pHCFTP->waypoints[i].dWaypointsCoordY, pHCFTP->waypoints[i + 1].dWaypointsCoordY,
                                          pHCFTP->waypointsVelocity[i].dWaypointsCoordY, pHCFTP->waypointsVelocity[i + 1].dWaypointsCoordY, pHCFTP->stTrolleySubTask[i].dMinTime,
                                          bTrolleyActiveSwayCtrl, TrolleyeType, dTrolleyNaturalFreq, dTrolleyDampingRatio, dTrolleyMaxVelocity, dTrolleyMaxAcceleration, dTrolleyMaxJerk, pHCFTP->stTrolleyShaper,
                                          &(dtmp), &(dtmp1), &(dtmp2), &(dtty));
      }
      else
      {
        pHCFTP->stTrolleySubTask[i].dMinTime = -1;
        SKA_HCFTP_MultipleOGetShapedTrace(pHCFTP, 0.0, pHCFTP->waypoints[i].dWaypointsCoordY, pHCFTP->waypoints[i + 1].dWaypointsCoordY,
                                          pHCFTP->waypointsVelocity[i].dWaypointsCoordY, pHCFTP->waypointsVelocity[i + 1].dWaypointsCoordY, pHCFTP->stTrolleySubTask[i].dMinTime,
                                          bTrolleyActiveSwayCtrl, TrolleyeType, dTrolleyNaturalFreq, dTrolleyDampingRatio, dTrolleyMaxVelocity, dTrolleyMaxAcceleration, dTrolleyMaxJerk, pHCFTP->stTrolleyShaper,
                                          &(dtmp), &(dtmp1), &(dtmp2), &(dtty));
      }

      // 协调开始时间

      pHCFTP->stBridgeSubTask[i].dStartTime = pHCFTP->pTotalTime;
      pHCFTP->stTrolleySubTask[i].dStartTime = pHCFTP->pTotalTime;

      if ((pHCFTP->motionMode[i].nx == 0 || pHCFTP->motionMode[i].nx == 2) && (pHCFTP->motionMode[i].ny == 1 || pHCFTP->motionMode[i].ny == 3))
      {
        if (dttx > dtty)
        {
          pHCFTP->stTrolleySubTask[i].dStartTime = pHCFTP->pTotalTime + dttx - dtty;
        }
      }
      if ((pHCFTP->motionMode[i].nx == 1 || pHCFTP->motionMode[i].nx == 3) && (pHCFTP->motionMode[i].ny == 0 || pHCFTP->motionMode[i].ny == 2))
      {
        if (dttx < dtty)
        {
          pHCFTP->stBridgeSubTask[i].dStartTime = pHCFTP->pTotalTime + dtty - dttx;
        }
      }
      if (i == pHCFTP->waypointsNum - 2)
      {
        pHCFTP->pBridgeTotalTime = pHCFTP->stBridgeSubTask[i].dStartTime + dttx;
        pHCFTP->pTrolleyTotalTime = pHCFTP->stTrolleySubTask[i].dStartTime + dtty;
      }
      pHCFTP->pTotalTime = pHCFTP->pTotalTime + fmax(dttx, dtty);
    }

  }
   return 0;
}

  int8_t SKA_HCFTP_Run(SKA_HorizontalCollisonFreeTracePlanner * pHCFTP, double dCurTime, double *pBridgeRefX, double *pBridgeRefDx, double *pBridgeRefDdx, 
                       double *pBridgeUnsRefx, double *pBridgeUnsRefDx, double *pBridgeUnsRefDdx, double *pTrolleyRefX, double *pTrolleyRefDx, 
                       double *pTrolleyRefDdx, double *pTrolleyUnsRefx, double *pTrolleyUnsRefDx, double *pTrolleyUnsRefDdx)
                       
  {

  /******************** 函数参数合法性检验 ********************/
    if (pHCFTP == NULL || pBridgeRefX == NULL || pBridgeRefDx == NULL || pBridgeRefDdx == NULL ||
        pBridgeUnsRefx == NULL || pBridgeUnsRefDx == NULL || pBridgeUnsRefDdx == NULL ||
        pTrolleyRefX == NULL || pTrolleyRefDx == NULL || pTrolleyRefDdx == NULL ||
        pTrolleyUnsRefx == NULL || pTrolleyUnsRefDx == NULL || pTrolleyUnsRefDdx == NULL)
    {
        printf("Invalid parameters of SKA_HCFTP_Run().\n");
        return -1;
    }

    /******************** 运行机构轨迹规划器 ********************/

    if ((pHCFTP->bIsUseWaypoints == 1 && pHCFTP->nWaypointsNum == 0) ||
        (pHCFTP->bIsUseWaypoints == 0 && pHCFTP->nObstacleNum == 0))
    {
      // 无障碍物
      SKA_TUTP_Run(&(pHCFTP->stBridgeTracePlanner), dCurTime, pBridgeRefX, pBridgeRefDx, 
                   pBridgeRefDdx, pBridgeUnsRefx, pBridgeUnsRefDx, pBridgeUnsRefDdx);
      SKA_TUTP_Run(&(pHCFTP->stTrolleyTracePlanner), dCurTime, pTrolleyRefX, pTrolleyRefDx, 
                   pTrolleyRefDdx, pTrolleyUnsRefx, pTrolleyUnsRefDx, pTrolleyUnsRefDdx);
    }
    else if (pHCFTP->bIsUseWaypoints == 0 && pHCFTP->nObstacleNum == 1)
    {
      // 单个障碍物
      pHCFTP->stBridgeTracePlanner.stUnsPlanner.bActiveHorizontalMode = pHCFTP->bActiveHorizontalMode;
      printf("stUnsPlanner bActiveHorizontalMode: %d\n", pHCFTP->stBridgeTracePlanner.stUnsPlanner.bActiveHorizontalMode);
      pHCFTP->stBridgeTracePlanner.stUnsPlanner.bActiveSwayCtrl = pHCFTP->bBridgeActiveSwayCtrl;
      pHCFTP->stBridgeTracePlanner.stUnsPlanner.eType = pHCFTP->BridgeeType;
      pHCFTP->stBridgeTracePlanner.stUnsPlanner.dNaturalFreq = pHCFTP->dBridgeNaturalFreq;
      pHCFTP->stBridgeTracePlanner.stUnsPlanner.dDampingRatio = pHCFTP->dBridgeDampingRatio;
      pHCFTP->stBridgeTracePlanner.stUnsPlanner.bActiveHorizontalMode = pHCFTP->bActiveHorizontalMode;
      pHCFTP->stTrolleyTracePlanner.stUnsPlanner.bActiveSwayCtrl = pHCFTP->bTrolleyActiveSwayCtrl;
      pHCFTP->stTrolleyTracePlanner.stUnsPlanner.eType = pHCFTP->TrolleyeType;
      pHCFTP->stTrolleyTracePlanner.stUnsPlanner.dNaturalFreq = pHCFTP->dTrolleyNaturalFreq;
      pHCFTP->stTrolleyTracePlanner.stUnsPlanner.dDampingRatio = pHCFTP->dTrolleyDampingRatio;
      memcpy(pHCFTP->stBridgeTracePlanner.stUnsPlanner.SOSubTasks, pHCFTP->stSOBridgeSubTask, sizeof(pHCFTP->stSOBridgeSubTask));
      memcpy(pHCFTP->stTrolleyTracePlanner.stUnsPlanner.SOSubTasks, pHCFTP->stSOTrolleySubTask, sizeof(pHCFTP->stSOTrolleySubTask));
      pHCFTP->stBridgeTracePlanner.stUnsPlanner.nSubTasksNum = pHCFTP->pBridgeSubTasksNum; 
      pHCFTP->stTrolleyTracePlanner.stUnsPlanner.nSubTasksNum = pHCFTP->pTrolleySubTasksNum;
      
      printf("0311\n");
      SKA_TUTP_Run(&(pHCFTP->stBridgeTracePlanner), dCurTime, pBridgeRefX, pBridgeRefDx,
                   pBridgeRefDdx, pBridgeUnsRefx, pBridgeUnsRefDx, pBridgeUnsRefDdx);
      SKA_TUTP_Run(&(pHCFTP->stTrolleyTracePlanner), dCurTime, pTrolleyRefX, pTrolleyRefDx,
                   pTrolleyRefDdx, pTrolleyUnsRefx, pTrolleyUnsRefDx, pTrolleyUnsRefDdx);
    }
    else
    {
     // 多个障碍物
      pHCFTP->stBridgeTracePlanner.stUnsPlanner.bActiveSwayCtrl = pHCFTP->bBridgeActiveSwayCtrl;
      pHCFTP->stBridgeTracePlanner.stUnsPlanner.eType = pHCFTP->BridgeeType;
      pHCFTP->stBridgeTracePlanner.stUnsPlanner.dNaturalFreq = pHCFTP->dBridgeNaturalFreq;
      pHCFTP->stBridgeTracePlanner.stUnsPlanner.dDampingRatio = pHCFTP->dBridgeDampingRatio;
      pHCFTP->stTrolleyTracePlanner.stUnsPlanner.bActiveSwayCtrl = pHCFTP->bTrolleyActiveSwayCtrl;
      pHCFTP->stTrolleyTracePlanner.stUnsPlanner.eType = pHCFTP->TrolleyeType;
      pHCFTP->stTrolleyTracePlanner.stUnsPlanner.dNaturalFreq = pHCFTP->dTrolleyNaturalFreq;
      pHCFTP->stTrolleyTracePlanner.stUnsPlanner.dDampingRatio = pHCFTP->dTrolleyDampingRatio;
      pHCFTP->stBridgeTracePlanner.stUnsPlanner.InputShaper = pHCFTP->stBridgeShaper;
      pHCFTP->stTrolleyTracePlanner.stUnsPlanner.InputShaper = pHCFTP->stTrolleyShaper;
      memcpy(pHCFTP->stBridgeTracePlanner.stUnsPlanner.MOSubTasks, pHCFTP->stMOBridgeSubTask, sizeof(pHCFTP->stMOBridgeSubTask));
      memcpy(pHCFTP->stTrolleyTracePlanner.stUnsPlanner.MOSubTasks, pHCFTP->stMOTrolleySubTask, sizeof(pHCFTP->stMOTrolleySubTask));
      pHCFTP->stBridgeTracePlanner.stUnsPlanner.nSubTasksNum = pHCFTP->pBridgeSubTasksNum; 
      pHCFTP->stTrolleyTracePlanner.stUnsPlanner.nSubTasksNum = pHCFTP->pTrolleySubTasksNum; 
     
      SKA_TUTP_Run(&(pHCFTP->stBridgeTracePlanner), dCurTime, pBridgeRefX, pBridgeRefDx, 
                   pBridgeRefDdx, pBridgeUnsRefx, pBridgeUnsRefDx, pBridgeUnsRefDdx);
      SKA_TUTP_Run(&(pHCFTP->stTrolleyTracePlanner), dCurTime, pTrolleyRefX, pTrolleyRefDx, 
                   pTrolleyRefDdx, pTrolleyUnsRefx, pTrolleyUnsRefDx, pTrolleyUnsRefDdx);
                   
    }
    return 0;
  }

  int8_t SKA_HCFTP_Destroy(SKA_HorizontalCollisonFreeTracePlanner * pHCFTP)
{
    /******************** 函数参数合法性检验 ********************/
    if (pHCFTP == NULL)
    {
        printf("Invalid parameters of SKA_HCFTP_Destroy().\n");
        return -1;
    }

    /******************** 销毁运行机构轨迹规划器 ********************/
    SKA_TUTP_Destroy(&(pHCFTP->stBridgeTracePlanner));
    SKA_TUTP_Destroy(&(pHCFTP->stTrolleyTracePlanner));

    return 0;
}



  static int8_t SKA_HCFTP_IsOpenRectanglesIntersect(double dLocationX1, double dLocationY1, double dLocationW1,
                                                    double dLocationH1, double dLocationX2, double dLocationY2,
                                                    double dLocationW2, double dLocationH2, bool *pflag)
  {
    /******************** 函数参数合法性检验 ********************/
    if (pflag == NULL)
    {
      {
        printf("Invalid parameters of SKA_HCFTP_IsOpenRectanglesIntersect().\n");
        return -1;
      }
    }
    /******************** 判断开始位置和目标位置所确定的矩形区域与障碍物区域是否相交 ********************/
    if (dLocationX1 >= dLocationX2 + dLocationW2 || dLocationX1 + dLocationW1 <= dLocationX2 ||
        dLocationY1 >= dLocationY2 + dLocationH2 || dLocationY1 + dLocationH1 <= dLocationY2)
    {
      *pflag = 0;
    }
    else
    {
      *pflag = 1;
    }

    return 0;
  }
  static int8_t SKA_HCFTP_SingleOGetNormalRightTrace(SKA_HorizontalCollisonFreeTracePlanner * pHCFTP, double dBridgeTargetPos, double dBridgeMaxVelocity, double dBridgeMaxAcceleration,
                                                     double dBridgeMaxJerk, SKA_InputShaper BridgeInputShaper, double dTrolleyTargetPos, double dTrolleyMaxVelocity, double dTrolleyMaxAcceleration,
                                                     double dTrolleyMaxJerk, SKA_InputShaper TrolleyInputShaper, SKA_HCFTPObstacleCoord Obstacle,
                                                     bool bBridgeActiveSwayCtrl, SKA_InputShaperType BridgeeType, double dBridgeNaturalFreq, double dBridgeDampingRatio, double dBridgeQuickStopMaxAcc,
                                                     bool bTrolleyActiveSwayCtrl, SKA_InputShaperType TrolleyeType, double dTrolleyNaturalFreq, double dTrolleyDampingRatio, double dTrolleyQuickStopMaxAcc,
                                                     uint8_t *pBridgeSubTasksNum, double *pBridgeTotalTime, uint8_t *pTrolleySubTasksNum, double *pTrolleyTotalTime, double *pTotalTime)
  {
    double dtx, dtx2, dty, dty2;
    double dfx, dfdx, dfddx;
    double dttx, dtty;

    if (Obstacle.dObstacleCoordY >= 0.0 && Obstacle.dObstacleCoordX + Obstacle.dObstacleCoordW <= dBridgeTargetPos)
    {
      SKA_HCFTP_SingleOGetMotionTime(pHCFTP, (Obstacle.dObstacleCoordX + Obstacle.dObstacleCoordW), 0.0, dBridgeTargetPos,
                                     bBridgeActiveSwayCtrl, BridgeeType, dBridgeNaturalFreq, dBridgeDampingRatio, dBridgeMaxVelocity,
                                     dBridgeMaxAcceleration, dBridgeMaxJerk, dBridgeQuickStopMaxAcc, &(dtx));

      SKA_HCFTP_SingleOGetMotionTime(pHCFTP, Obstacle.dObstacleCoordY, 0.0, dTrolleyTargetPos,
                                     bTrolleyActiveSwayCtrl, TrolleyeType, dTrolleyNaturalFreq, dTrolleyDampingRatio,
                                     dTrolleyMaxVelocity, dTrolleyMaxAcceleration, dTrolleyMaxJerk, dTrolleyQuickStopMaxAcc, &(dty));

      SKA_TUTP_SingleOGetShapedTraceAndTotalTime(&(pHCFTP->stTracePlanner), 0.0, 0.0, dBridgeTargetPos, bBridgeActiveSwayCtrl, BridgeeType, dBridgeNaturalFreq,
                                                  dBridgeDampingRatio, dBridgeMaxVelocity, dBridgeMaxAcceleration, dBridgeMaxJerk, dBridgeQuickStopMaxAcc,
                                                  &(dfx), &(dfdx), &(dfddx), &(dttx));


      SKA_TUTP_SingleOGetShapedTraceAndTotalTime(&(pHCFTP->stTracePlanner), 0.0, 0.0, dTrolleyTargetPos, bTrolleyActiveSwayCtrl, TrolleyeType, dTrolleyNaturalFreq,
                                                  dTrolleyDampingRatio, dTrolleyMaxVelocity, dTrolleyMaxAcceleration, dTrolleyMaxJerk, dTrolleyQuickStopMaxAcc,
                                                  &(dfx), &(dfdx), &(dfddx), &(dtty));

      *pBridgeSubTasksNum = 1;
      *pTrolleySubTasksNum = 1;

      pHCFTP->stSOBridgeSubTask[0].dStartTime = 0.0;
      pHCFTP->stSOBridgeSubTask[0].dStartPos = 0.0;
      pHCFTP->stSOBridgeSubTask[0].dTargetPos = dBridgeTargetPos;

      if (dtx <= dty)
      {
        pHCFTP->stSOTrolleySubTask[0].dStartTime = 0.0;
        pHCFTP->stSOTrolleySubTask[0].dStartPos = 0.0;
        pHCFTP->stSOTrolleySubTask[0].dTargetPos = dTrolleyTargetPos;

        *pBridgeTotalTime = dttx;
        *pTrolleyTotalTime = dtty;
      }
      else
      {
        pHCFTP->stSOTrolleySubTask[0].dStartTime = dtx - dty;
        pHCFTP->stSOTrolleySubTask[0].dStartPos = 0.0;
        pHCFTP->stSOTrolleySubTask[0].dTargetPos = dTrolleyTargetPos;

        *pBridgeTotalTime = dttx;
        *pTrolleyTotalTime = dtx - dty + dtty;
      }
    }
    else if (Obstacle.dObstacleCoordY <= 0.0 && (Obstacle.dObstacleCoordX + Obstacle.dObstacleCoordW) <= dBridgeTargetPos)
    {

      SKA_HCFTP_SingleOGetMotionTime(pHCFTP, Obstacle.dObstacleCoordX, 0.0, dBridgeTargetPos,
                                     bBridgeActiveSwayCtrl, BridgeeType, dBridgeNaturalFreq, dBridgeDampingRatio,
                                     dBridgeMaxVelocity, dBridgeMaxAcceleration, dBridgeMaxJerk, dBridgeQuickStopMaxAcc, &(dtx));

      SKA_TUTP_SingleOGetShapedTraceAndTotalTime(&(pHCFTP->stTracePlanner), 0.0, 0.0, Obstacle.dObstacleCoordY, bTrolleyActiveSwayCtrl, TrolleyeType, dTrolleyNaturalFreq,
                                                  dTrolleyDampingRatio, dTrolleyMaxVelocity, dTrolleyMaxAcceleration, dTrolleyMaxJerk, dTrolleyQuickStopMaxAcc,
                                                  &(dfx), &(dfdx), &(dfddx), &(dtty));

      *pBridgeSubTasksNum = 1;
      *pTrolleySubTasksNum = 2;

      if (dtx <= dty)
      {
        pHCFTP->stSOBridgeSubTask[0].dStartTime = dty - dtx;
        pHCFTP->stSOBridgeSubTask[0].dStartPos = 0.0;
        pHCFTP->stSOBridgeSubTask[0].dTargetPos = dBridgeTargetPos;
      }
      else
      {
        pHCFTP->stSOBridgeSubTask[0].dStartTime = 0.0;
        pHCFTP->stSOBridgeSubTask[0].dStartPos = 0.0;
        pHCFTP->stSOBridgeSubTask[0].dTargetPos = dBridgeTargetPos;
      }

      pHCFTP->stSOTrolleySubTask[0].dStartTime = 0.0;
      pHCFTP->stSOTrolleySubTask[0].dStartPos = 0.0;
      pHCFTP->stSOTrolleySubTask[0].dTargetPos = Obstacle.dObstacleCoordY;
      SKA_HCFTP_SingleOGetMotionTime(pHCFTP, (Obstacle.dObstacleCoordX + Obstacle.dObstacleCoordW), 0.0, dBridgeTargetPos,
                                     bBridgeActiveSwayCtrl, BridgeeType, dBridgeNaturalFreq, dBridgeDampingRatio, dBridgeMaxVelocity,
                                     dBridgeMaxAcceleration, dBridgeMaxJerk, dBridgeQuickStopMaxAcc, &(dtx2));

      pHCFTP->stSOTrolleySubTask[1].dStartTime = pHCFTP->stSOBridgeSubTask[0].dStartTime + dtx2;
      pHCFTP->stSOTrolleySubTask[1].dStartPos = Obstacle.dObstacleCoordY;
      pHCFTP->stSOTrolleySubTask[1].dTargetPos = dTrolleyTargetPos;
      SKA_TUTP_SingleOGetShapedTraceAndTotalTime(&(pHCFTP->stTracePlanner), 0.0, 0.0, dBridgeTargetPos, bBridgeActiveSwayCtrl, BridgeeType, dBridgeNaturalFreq,
                                                  dBridgeDampingRatio, dBridgeMaxVelocity, dBridgeMaxAcceleration, dBridgeMaxJerk, dBridgeQuickStopMaxAcc, &(dfx), &(dfdx), &(dfddx), &(dtty));
      SKA_TUTP_SingleOGetShapedTraceAndTotalTime(&(pHCFTP->stTracePlanner), 0.0, Obstacle.dObstacleCoordY, dTrolleyTargetPos, bTrolleyActiveSwayCtrl, TrolleyeType, dTrolleyNaturalFreq,
                                                  dTrolleyDampingRatio, dTrolleyMaxVelocity, dTrolleyMaxAcceleration, dTrolleyMaxJerk, dTrolleyQuickStopMaxAcc,
                                                  &(dfx), &(dfdx), &(dfddx), &(dtty));

      dttx = pHCFTP->stSOBridgeSubTask[*pBridgeSubTasksNum - 1].dStartTime + dttx;
      dtty = pHCFTP->stSOTrolleySubTask[*pTrolleySubTasksNum - 1].dStartTime + dtty;
      *pBridgeTotalTime = dttx;
      *pTrolleyTotalTime = dtty;
    }
    else if (Obstacle.dObstacleCoordY >= 0.0 && Obstacle.dObstacleCoordX + Obstacle.dObstacleCoordW >= dBridgeTargetPos)
    {

      SKA_TUTP_SingleOGetShapedTraceAndTotalTime(&(pHCFTP->stTracePlanner), 0.0, 0.0, (Obstacle.dObstacleCoordX + Obstacle.dObstacleCoordW),
                                                  bBridgeActiveSwayCtrl, BridgeeType, dBridgeNaturalFreq, dBridgeDampingRatio,
                                                  dBridgeMaxVelocity, dBridgeMaxAcceleration, dBridgeMaxJerk, dBridgeQuickStopMaxAcc,
                                                  &(dfx), &(dfdx), &(dfddx), &(dtx));
      SKA_HCFTP_SingleOGetMotionTime(pHCFTP, Obstacle.dObstacleCoordY, 0.0, dTrolleyTargetPos,
                                     bTrolleyActiveSwayCtrl, TrolleyeType, dTrolleyNaturalFreq, dTrolleyDampingRatio,
                                     dTrolleyMaxVelocity, dTrolleyMaxAcceleration, dTrolleyMaxJerk, dTrolleyQuickStopMaxAcc, &(dty));

      *pBridgeSubTasksNum = 2;
      *pTrolleySubTasksNum = 1;
      pHCFTP->stSOBridgeSubTask[0].dStartTime = 0.0;
      pHCFTP->stSOBridgeSubTask[0].dStartPos = 0.0;
      pHCFTP->stSOBridgeSubTask[0].dTargetPos = Obstacle.dObstacleCoordX + Obstacle.dObstacleCoordW;

      if (dtx <= dty)
      {
        pHCFTP->stSOTrolleySubTask[0].dStartTime = 0.0;
        pHCFTP->stSOTrolleySubTask[0].dStartPos = 0.0;
        pHCFTP->stSOTrolleySubTask[0].dTargetPos = dTrolleyTargetPos;
      }
      else
      {
        pHCFTP->stSOTrolleySubTask[0].dStartTime = dtx - dty;
        pHCFTP->stSOTrolleySubTask[0].dStartPos = 0.0;
        pHCFTP->stSOTrolleySubTask[0].dTargetPos = dTrolleyTargetPos;
      }
      SKA_HCFTP_SingleOGetMotionTime(pHCFTP, (Obstacle.dObstacleCoordY + Obstacle.dObstacleCoordH), 0.0, dTrolleyTargetPos,
                                     bTrolleyActiveSwayCtrl, TrolleyeType, dTrolleyNaturalFreq, dTrolleyDampingRatio,
                                     dTrolleyMaxVelocity, dTrolleyMaxAcceleration, dTrolleyMaxJerk, dTrolleyQuickStopMaxAcc, &(dty2));

      pHCFTP->stSOBridgeSubTask[1].dStartTime = pHCFTP->stSOTrolleySubTask[0].dStartTime + dty2;
      pHCFTP->stSOBridgeSubTask[1].dStartPos = Obstacle.dObstacleCoordX + Obstacle.dObstacleCoordW;
      pHCFTP->stSOBridgeSubTask[1].dTargetPos = dBridgeTargetPos;

      SKA_TUTP_SingleOGetShapedTraceAndTotalTime(&(pHCFTP->stTracePlanner), 0.0, (Obstacle.dObstacleCoordX + Obstacle.dObstacleCoordW),
                                                  dBridgeTargetPos, bBridgeActiveSwayCtrl, BridgeeType, dBridgeNaturalFreq, dBridgeDampingRatio, dBridgeMaxVelocity,
                                                  dBridgeMaxAcceleration, dBridgeMaxJerk, dBridgeQuickStopMaxAcc, &(dfx), &(dfdx), &(dfddx), &(dttx));
      SKA_TUTP_SingleOGetShapedTraceAndTotalTime(&(pHCFTP->stTracePlanner), 0.0, 0.0, dTrolleyTargetPos, bTrolleyActiveSwayCtrl, TrolleyeType, dTrolleyNaturalFreq,
                                                  dTrolleyDampingRatio, dTrolleyMaxVelocity, dTrolleyMaxAcceleration, dTrolleyMaxJerk, dTrolleyQuickStopMaxAcc,
                                                  &(dfx), &(dfdx), &(dfddx), &(dtty));

      dttx = pHCFTP->stSOBridgeSubTask[*pBridgeSubTasksNum - 1].dStartTime + dttx;
      dtty = pHCFTP->stSOTrolleySubTask[*pTrolleySubTasksNum - 1].dStartTime + dtty;
      *pBridgeTotalTime = dttx;
      *pTrolleyTotalTime = dtty;
    }
    else
    {
      SKA_HCFTP_SingleOGetMotionTime(pHCFTP, Obstacle.dObstacleCoordX, 0.0, (Obstacle.dObstacleCoordX + Obstacle.dObstacleCoordW),
                                     bBridgeActiveSwayCtrl, BridgeeType, dBridgeNaturalFreq, dBridgeDampingRatio, dBridgeMaxVelocity,
                                     dBridgeMaxAcceleration, dBridgeMaxJerk, dBridgeQuickStopMaxAcc, &(dtx));
      SKA_TUTP_SingleOGetShapedTraceAndTotalTime(&(pHCFTP->stTracePlanner), 0.0, 0.0, Obstacle.dObstacleCoordY,
                                                  bTrolleyActiveSwayCtrl, TrolleyeType, dTrolleyNaturalFreq, dTrolleyDampingRatio,
                                                  dTrolleyMaxVelocity, dTrolleyMaxAcceleration, dTrolleyMaxJerk, dTrolleyQuickStopMaxAcc,
                                                  &(dfx), &(dfdx), &(dfddx), &(dty));

      *pBridgeSubTasksNum = 2;
      *pTrolleySubTasksNum = 2;

      if (dtx <= dty)
      {
        pHCFTP->stSOBridgeSubTask[0].dStartTime = dty - dtx;
        pHCFTP->stSOBridgeSubTask[0].dStartPos = 0.0;
        pHCFTP->stSOBridgeSubTask[0].dTargetPos = Obstacle.dObstacleCoordX + Obstacle.dObstacleCoordW;
      }
      else
      {
        pHCFTP->stSOBridgeSubTask[0].dStartTime = 0.0;
        pHCFTP->stSOBridgeSubTask[0].dStartPos = 0.0;
        pHCFTP->stSOBridgeSubTask[0].dTargetPos = Obstacle.dObstacleCoordX + Obstacle.dObstacleCoordW;
      }

      pHCFTP->stSOTrolleySubTask[0].dStartTime = 0.0;
      pHCFTP->stSOTrolleySubTask[0].dStartPos = 0.0;
      pHCFTP->stSOTrolleySubTask[0].dTargetPos = Obstacle.dObstacleCoordY;
      SKA_TUTP_SingleOGetShapedTraceAndTotalTime(&(pHCFTP->stTracePlanner), 0.0, 0.0, (Obstacle.dObstacleCoordX + Obstacle.dObstacleCoordW),
                                                  bBridgeActiveSwayCtrl, BridgeeType, dBridgeNaturalFreq, dBridgeDampingRatio, dBridgeMaxVelocity,
                                                  dBridgeMaxAcceleration, dBridgeMaxJerk, dBridgeQuickStopMaxAcc, &(dfx), &(dfdx), &(dfddx), &(dttx));

      dttx = pHCFTP->stSOBridgeSubTask[0].dStartTime + dttx;
      pHCFTP->stSOTrolleySubTask[1].dStartTime = dttx;
      pHCFTP->stSOTrolleySubTask[1].dStartPos = Obstacle.dObstacleCoordY;
      pHCFTP->stSOTrolleySubTask[1].dTargetPos = dTrolleyTargetPos;
      SKA_HCFTP_SingleOGetMotionTime(pHCFTP, (Obstacle.dObstacleCoordY + Obstacle.dObstacleCoordH), Obstacle.dObstacleCoordY,
                                     dTrolleyTargetPos, bTrolleyActiveSwayCtrl, TrolleyeType, dTrolleyNaturalFreq, dTrolleyDampingRatio,
                                     dTrolleyMaxVelocity, dTrolleyMaxAcceleration, dTrolleyMaxJerk, dTrolleyQuickStopMaxAcc, &(dty2));
      dty2 = dttx + dty2;

      pHCFTP->stSOBridgeSubTask[1].dStartTime = dty2;
      pHCFTP->stSOBridgeSubTask[1].dStartPos = Obstacle.dObstacleCoordX + Obstacle.dObstacleCoordW;
      pHCFTP->stSOBridgeSubTask[1].dTargetPos = dBridgeTargetPos;
      SKA_TUTP_SingleOGetShapedTraceAndTotalTime(&(pHCFTP->stTracePlanner), 0.0, (Obstacle.dObstacleCoordX + Obstacle.dObstacleCoordW), dBridgeTargetPos,
                                                  bBridgeActiveSwayCtrl, BridgeeType, dBridgeNaturalFreq, dBridgeDampingRatio, dBridgeMaxVelocity,
                                                  dBridgeMaxAcceleration, dBridgeMaxJerk, dBridgeQuickStopMaxAcc, &(dfx), &(dfdx), &(dfddx), &(dttx));
      SKA_TUTP_SingleOGetShapedTraceAndTotalTime(&(pHCFTP->stTracePlanner), 0.0, Obstacle.dObstacleCoordY, dTrolleyTargetPos, bTrolleyActiveSwayCtrl, TrolleyeType, dTrolleyNaturalFreq,
                                                  dTrolleyDampingRatio, dTrolleyMaxVelocity, dTrolleyMaxAcceleration, dTrolleyMaxJerk, dTrolleyQuickStopMaxAcc,
                                                  &(dfx), &(dfdx), &(dfddx), &(dtty));

      dttx = pHCFTP->stSOBridgeSubTask[*pBridgeSubTasksNum - 1].dStartTime + dttx;
      dtty = pHCFTP->stSOTrolleySubTask[*pTrolleySubTasksNum - 1].dStartTime + dtty;
      *pBridgeTotalTime = dttx;
      *pTrolleyTotalTime = dtty;
    }

    *pTotalTime = fmax(*pBridgeTotalTime, *pTrolleyTotalTime);
    printf("SingleOGetNormalRightTrace stSOBridgeSubTask[0].dStartTime: %lf\n",pHCFTP->stSOBridgeSubTask[0].dStartTime);
    printf("SingleOGetNormalRightTrace stSOBridgeSubTask[0].dStartPos: %lf\n",pHCFTP->stSOBridgeSubTask[0].dStartPos);
    printf("SingleOGetNormalRightTrace stSOBridgeSubTask[0].dTargetPos: %lf\n",pHCFTP->stSOBridgeSubTask[0].dTargetPos);
    
    printf("SingleOGetNormalRightTrace stSOTrolleySubTask[0].dStartTime: %lf\n",pHCFTP->stSOTrolleySubTask[0].dStartTime);
    printf("SingleOGetNormalRightTrace stSOTrolleySubTask[0].dStartPos: %lf\n",pHCFTP->stSOTrolleySubTask[0].dStartPos);
    printf("SingleOGetNormalRightTrace stSOTrolleySubTask[0].dTargetPos: %lf\n",pHCFTP->stSOTrolleySubTask[0].dTargetPos);
    return 0;
  }

  static int8_t SKA_HCFTP_SingleOGetMotionTime(SKA_HorizontalCollisonFreeTracePlanner *pHCFTP, double dFindPos, double dStartPos, double dTargetPos,
                                               bool bActiveSwayCtrl, SKA_InputShaperType eType, double dNaturalFreq, double dDampingRatio,
                                               double dMaxVelocity, double dMaxAcceleration, double dMaxJerk, double dQuickStopMaxAcc, double *pMotionTime)
  {
    if (dFindPos < dStartPos && dFindPos < dTargetPos || dFindPos > dStartPos && dFindPos > dTargetPos)
    {
      *pMotionTime = -1;
      return 0;
    }
    double dlo, dhi, dmid;
    double dfx, dfdx, dfddx;
    double dtt;
    dlo = 0.0;
    SKA_TUTP_SingleOGetShapedTraceAndTotalTime(&(pHCFTP->stTracePlanner), 0.0, dStartPos, dTargetPos,
                                                bActiveSwayCtrl, eType, dNaturalFreq, dDampingRatio,
                                                dMaxVelocity, dMaxAcceleration, dMaxJerk, dQuickStopMaxAcc,
                                                &(dfx), &(dfdx), &(dfddx), &(dhi));
    while (dhi - dlo > SKA_FlOAT_ERROR)
    {
      dmid = (dlo + dhi) / 2.0;
      SKA_TUTP_SingleOGetShapedTraceAndTotalTime(&(pHCFTP->stTracePlanner), dmid, dStartPos, dTargetPos,
                                                  bActiveSwayCtrl, eType, dNaturalFreq, dDampingRatio,
                                                  dMaxVelocity, dMaxAcceleration, dMaxJerk, dQuickStopMaxAcc,
                                                  &(dfx), &(dfdx), &(dfddx), &(dtt));
      if ((dTargetPos - dStartPos) * (dFindPos - dfx) >= 0)
      {
        dlo = dmid;
      }
      else
      {
        dhi = dmid;
      }
    }
    *pMotionTime = (dlo + dhi) / 2.0;

    return 0;
  }

  static int8_t SKA_HCFTP_MultipleOGetWayPoints(SKA_HorizontalCollisonFreeTracePlanner *pHCFTP, double dBridgeStartPos, double dBridgeTargetPos, double dBridgeMaxPos, double dBridgeMinPos,
                                                double dTrolleyStartPos, double dTrolleyTargetPos, double dTrolleyMaxPos, double dTrolleyMinPos, uint8_t nObstacleNum, uint8_t *WayPointsNum)
  {
    // 障碍物顶点
    pHCFTP->vertexNum = 2;
    bool bflag;
    // 起点和终点
    pHCFTP->vertex[0].dCoordX = dBridgeStartPos;
    pHCFTP->vertex[0].dCoordY = dTrolleyStartPos;
    pHCFTP->vertex[1].dCoordX = dBridgeTargetPos;
    pHCFTP->vertex[1].dCoordY = dTrolleyTargetPos;

    if (nObstacleNum > 0)
    {
      for (int i = 0; i < nObstacleNum; i++)
      {
        // 左下角
        SKA_HCFTP_MultipleOIsPointInCloseRectangle(pHCFTP->ArrObstacles[i].dObstacleCoordX, pHCFTP->ArrObstacles[i].dObstacleCoordY,
                                                   dBridgeMinPos, dTrolleyMinPos, dBridgeMaxPos - dBridgeMinPos, dTrolleyMaxPos - dTrolleyMinPos, &bflag);
        if (bflag == 1)
        {
          pHCFTP->vertex[pHCFTP->vertexNum].dCoordX = pHCFTP->ArrObstacles[i].dObstacleCoordX;
          pHCFTP->vertex[pHCFTP->vertexNum].dCoordY = pHCFTP->ArrObstacles[i].dObstacleCoordY;
          pHCFTP->vertexNum++;
        }

        // 右下角
        SKA_HCFTP_MultipleOIsPointInCloseRectangle(pHCFTP->ArrObstacles[i].dObstacleCoordX + pHCFTP->ArrObstacles[i].dObstacleCoordW,
                                                   pHCFTP->ArrObstacles[i].dObstacleCoordY, dBridgeMinPos, dTrolleyMinPos,
                                                   dBridgeMaxPos - dBridgeMinPos, dTrolleyMaxPos - dTrolleyMinPos, &bflag);
        if (bflag == 1)
        {
          pHCFTP->vertex[pHCFTP->vertexNum].dCoordX = pHCFTP->ArrObstacles[i].dObstacleCoordX + pHCFTP->ArrObstacles[i].dObstacleCoordW;
          pHCFTP->vertex[pHCFTP->vertexNum].dCoordY = pHCFTP->ArrObstacles[i].dObstacleCoordY;
          pHCFTP->vertexNum++;
        }

        // 右上角
        SKA_HCFTP_MultipleOIsPointInCloseRectangle(pHCFTP->ArrObstacles[i].dObstacleCoordX + pHCFTP->ArrObstacles[i].dObstacleCoordW,
                                                   pHCFTP->ArrObstacles[i].dObstacleCoordY + pHCFTP->ArrObstacles[i].dObstacleCoordH,
                                                   dBridgeMinPos, dTrolleyMinPos, dBridgeMaxPos - dBridgeMinPos,
                                                   dTrolleyMaxPos - dTrolleyMinPos, &bflag);
        if (bflag == 1)
        {
          pHCFTP->vertex[pHCFTP->vertexNum].dCoordX = pHCFTP->ArrObstacles[i].dObstacleCoordX + pHCFTP->ArrObstacles[i].dObstacleCoordW;
          pHCFTP->vertex[pHCFTP->vertexNum].dCoordY = pHCFTP->ArrObstacles[i].dObstacleCoordY + pHCFTP->ArrObstacles[i].dObstacleCoordH;
          pHCFTP->vertexNum++;
        }

        // 左上角
        SKA_HCFTP_MultipleOIsPointInCloseRectangle(pHCFTP->ArrObstacles[i].dObstacleCoordX,
                                                   pHCFTP->ArrObstacles[i].dObstacleCoordY + pHCFTP->ArrObstacles[i].dObstacleCoordH, dBridgeMinPos,
                                                   dTrolleyMinPos, dBridgeMaxPos - dBridgeMinPos, dTrolleyMaxPos - dTrolleyMinPos, &bflag);
        if (bflag == 1)
        {
          pHCFTP->vertex[pHCFTP->vertexNum].dCoordX = pHCFTP->ArrObstacles[i].dObstacleCoordX;
          pHCFTP->vertex[pHCFTP->vertexNum].dCoordY = pHCFTP->ArrObstacles[i].dObstacleCoordY + pHCFTP->ArrObstacles[i].dObstacleCoordH;
          pHCFTP->vertexNum++;
        }
      }
    }

    // 计算边的权重
    for (int i = 0; i < pHCFTP->vertexNum; i++)
    {
      for (int j = 0; j < pHCFTP->vertexNum; j++)
      {
        if (i == j)
        {
          // 禁止任意顶点存在指向自己的边
          pHCFTP->edge[i * pHCFTP->vertexNum + j] = -1; // 自环
        }
        else
        {
          // 欧式距离
          pHCFTP->edge[i * pHCFTP->vertexNum + j] = sqrt(pow(pHCFTP->vertex[i].dCoordX - pHCFTP->vertex[j].dCoordX, 2) +
                                                       pow(pHCFTP->vertex[i].dCoordY - pHCFTP->vertex[j].dCoordY, 2));
          // 排除不可行路径
          if (nObstacleNum > 0)
          {
            for (int k = 0; k < nObstacleNum; k++)
            {
              SKA_HCFTP_IsOpenRectanglesIntersect(fmin(pHCFTP->vertex[i].dCoordX, pHCFTP->vertex[j].dCoordX), fmin(pHCFTP->vertex[i].dCoordY, pHCFTP->vertex[j].dCoordY),
                                                  fabs(pHCFTP->vertex[i].dCoordX - pHCFTP->vertex[j].dCoordX), fabs(pHCFTP->vertex[i].dCoordY - pHCFTP->vertex[j].dCoordY),
                                                  pHCFTP->ArrObstacles[k].dObstacleCoordX, pHCFTP->ArrObstacles[k].dObstacleCoordY, pHCFTP->ArrObstacles[k].dObstacleCoordW,
                                                  pHCFTP->ArrObstacles[k].dObstacleCoordH, &(bflag));
              if (bflag == 1)
              {
                pHCFTP->edge[i * pHCFTP->vertexNum + j] = -1; // 不可行路径
                break;
              }
            }
          }
        }
      }
    }

    // 初始化单源路径
    for (int i = 0; i < pHCFTP->vertexNum; i++)
    {
      if (i == 0)
      {
        pHCFTP->vertex[i].dSingleSourceSPath = 0;
      }
      else
      {
        pHCFTP->vertex[i].dSingleSourceSPath = -1;
      }
      pHCFTP->vertex[i].nPrecursorNode = -1;
    }

    // Dijkstra算法
    pHCFTP->sizeOfQ = pHCFTP->vertexNum;
    for (uint16_t i = 0; i < pHCFTP->vertexNum; i++)
    {
      pHCFTP->isInQ[i] = 1;
    }
    double dtmp;
    while (pHCFTP->sizeOfQ > 0)
    {
      // 找到最小单源路径的结点
      double minD = -1;
      int minIndex = -1;
      for (uint16_t i = 0; i < pHCFTP->vertexNum; i++)
      {
        if (pHCFTP->isInQ[i] == 1)
        {
          if (minIndex < -0.5)
          {
            minD = pHCFTP->vertex[i].dSingleSourceSPath;
            minIndex = (int16_t)i;
          }
          else
          {
            if (pHCFTP->vertex[i].dSingleSourceSPath >= -0.5)
            {
              if (minD < -0.5 || (minD > -0.5 && pHCFTP->vertex[i].dSingleSourceSPath < minD))
              {
                minD = pHCFTP->vertex[i].dSingleSourceSPath;
                minIndex = (int16_t)i;
              }
            }
          }
        }
      }

      pHCFTP->isInQ[minIndex] = 0;
      pHCFTP->sizeOfQ--;

      // 对其他结点进行松弛
      for (uint16_t i = 0; i < pHCFTP->vertexNum; i++)
      {
        dtmp = pHCFTP->edge[(uint16_t)minIndex * pHCFTP->vertexNum + i];
        if (pHCFTP->isInQ[i] == 1 && dtmp > -0.5)
        {
          if (pHCFTP->vertex[i].dSingleSourceSPath < -0.5)
          {
            pHCFTP->vertex[i].dSingleSourceSPath = minD + dtmp;
            pHCFTP->vertex[i].nPrecursorNode = (int8_t)minIndex;
          }
          else
          {
            if (pHCFTP->vertex[i].dSingleSourceSPath > minD + dtmp)
            {
              pHCFTP->vertex[i].dSingleSourceSPath = minD + dtmp;
              pHCFTP->vertex[i].nPrecursorNode = (int8_t)minIndex;
            }
          }
        }
      }
    }

    /*确定从起点到终点的最短路径*/
    uint16_t i = 1;
    *WayPointsNum = 1;
    pHCFTP->ArrWayPoints[0].dWaypointsCoordX = dBridgeTargetPos;
    pHCFTP->ArrWayPoints[0].dWaypointsCoordY = dTrolleyTargetPos;

    while (pHCFTP->vertex[i].nPrecursorNode >= 0)
    {
      i = (uint8_t)pHCFTP->vertex[i].nPrecursorNode;
      pHCFTP->ArrWayPoints[*WayPointsNum].dWaypointsCoordX = pHCFTP->vertex[i].dCoordX;
      pHCFTP->ArrWayPoints[*WayPointsNum].dWaypointsCoordY = pHCFTP->vertex[i].dCoordY;
      *WayPointsNum = *WayPointsNum + 1;
    }

    if (i != 0)
    {
      // 无法找到一条从起点到终点的最短路径
      *WayPointsNum = 0;
      return 0;
    }

    // 反转路径
    for (uint16_t i = 0; i < (*WayPointsNum); i++)
    {
      uint16_t j = *WayPointsNum - 1 - i;
      if (i < j)
      {
        dtmp = pHCFTP->ArrWayPoints[i].dWaypointsCoordX;
        pHCFTP->ArrWayPoints[i].dWaypointsCoordX = pHCFTP->ArrWayPoints[j].dWaypointsCoordX;
        pHCFTP->ArrWayPoints[j].dWaypointsCoordX = dtmp;

        dtmp = pHCFTP->ArrWayPoints[i].dWaypointsCoordY;
        pHCFTP->ArrWayPoints[i].dWaypointsCoordY = pHCFTP->ArrWayPoints[j].dWaypointsCoordY;
        pHCFTP->ArrWayPoints[j].dWaypointsCoordY = dtmp;
      }
      else
      {
        break;
      }
    }
    return 0;
  }

  static int8_t SKA_HCFTP_MultipleOIsPointInCloseRectangle(double dPosX, double dPosY, double dPosRX,
                                                           double dPosRY, double dPosRW, double dPosRH, bool *pflag)
  {
    *pflag = (dPosX >= dPosRX) && (dPosX <= dPosRX + dPosRW) && (dPosY >= dPosRY) && (dPosY <= dPosRY + dPosRH);

    return 0;
  }

  static int8_t SKA_HCFTP_MultipleOGetWayPointsVelocity(SKA_HorizontalCollisonFreeTracePlanner *pHCFTP, uint8_t nWayPointsNum, bool bBridgeActiveSwayCtrl,
                                                        SKA_InputShaperType BridgeeType, double dBridgeNaturalFreq, double dBridgeDampingRatio,
                                                        double dBridgeQuickStopMaxAcc, double dBridgeMaxVelocity, double dBridgeMaxAcceleration,
                                                        double dBridgeMaxJerk, SKA_InputShaper stBridgeShaper, bool bTrolleyActiveSwayCtrl,
                                                        SKA_InputShaperType TrolleyeType, double dTrolleyNaturalFreq, double dTrolleyDampingRatio,
                                                        double dTrolleyQuickStopMaxAcc, double dTrolleyMaxVelocity, double dTrolleyMaxAcceleration,
                                                        double dTrolleyMaxJerk, SKA_InputShaper stTrolleyShaper)
  {

    double dminTime, dtmp, dtmp1, dtmp2;

    // 倒序确定路径点的最大速度
    pHCFTP->maxWaypointsVelocity[nWayPointsNum - 1].dWaypointsCoordX = 0.0;
    pHCFTP->maxWaypointsVelocity[nWayPointsNum - 1].dWaypointsCoordY = 0.0;

    if (nWayPointsNum > 2)
    {
      for (int i = nWayPointsNum - 2; i >= 1; i--)
      {
        // X方向
        if (pHCFTP->MotionMode[i].nx == 0 || pHCFTP->MotionMode[i].nx == 1)
        {
          pHCFTP->maxWaypointsVelocity[i].dWaypointsCoordX = 0.0;
        }
        else if (pHCFTP->MotionMode[i].nx == 2)
        {
          SKA_HCFTP_MultipleOGetMaxStartVelocity(pHCFTP, fabs(pHCFTP->arrWayPoints[i + 1].dWaypointsCoordX - pHCFTP->arrWayPoints[i].dWaypointsCoordX),
                                                 dBridgeMaxVelocity, dBridgeMaxAcceleration, dBridgeMaxJerk, stBridgeShaper, &(pHCFTP->maxWaypointsVelocity[i].dWaypointsCoordX));
        }
        else
        {

          SKA_TUTP_SingleOGetShapedTraceAndTotalTime(&(pHCFTP->stTracePlanner), 0.0, pHCFTP->arrWayPoints[i].dWaypointsCoordY, pHCFTP->arrWayPoints[i + 1].dWaypointsCoordY,
                                                      bTrolleyActiveSwayCtrl, TrolleyeType, dTrolleyNaturalFreq, dTrolleyDampingRatio, dTrolleyMaxVelocity, dTrolleyMaxAcceleration,
                                                      dTrolleyMaxJerk, dTrolleyQuickStopMaxAcc, &(dtmp), &(dtmp1), &(dtmp2), &(dminTime));

          SKA_HCFTP_MultipleOGetMaxStartVelocityOverMinTime(pHCFTP,fabs(pHCFTP->arrWayPoints[i + 1].dWaypointsCoordX - pHCFTP->arrWayPoints[i].dWaypointsCoordX),
                                                            dBridgeMaxAcceleration, dBridgeMaxJerk, stBridgeShaper, dminTime, (pHCFTP->maxWaypointsVelocity[i + 1].dWaypointsCoordX),
                                                            &(pHCFTP->maxWaypointsVelocity[i].dWaypointsCoordX));
        }
        // Y方向
        if (pHCFTP->MotionMode[i].ny == 0 || pHCFTP->MotionMode[i].ny == 1)
        {
          pHCFTP->maxWaypointsVelocity[i].dWaypointsCoordY = 0.0;
        }
        else if (pHCFTP->MotionMode[i].ny == 2)
        {
          SKA_HCFTP_MultipleOGetMaxStartVelocity(pHCFTP,fabs(pHCFTP->arrWayPoints[i + 1].dWaypointsCoordY - pHCFTP->arrWayPoints[i].dWaypointsCoordY),
                                                 dTrolleyMaxVelocity, dTrolleyMaxAcceleration, dTrolleyMaxJerk, stTrolleyShaper, &(pHCFTP->maxWaypointsVelocity[i].dWaypointsCoordY));
        }
        else
        {
          SKA_TUTP_SingleOGetShapedTraceAndTotalTime(&(pHCFTP->stTracePlanner), 0.0, pHCFTP->arrWayPoints[i].dWaypointsCoordX, pHCFTP->arrWayPoints[i + 1].dWaypointsCoordX,
                                                      bBridgeActiveSwayCtrl, BridgeeType, dBridgeNaturalFreq, dBridgeDampingRatio, dBridgeMaxVelocity, dBridgeMaxAcceleration,
                                                      dBridgeMaxJerk, dBridgeQuickStopMaxAcc, &(dtmp), &(dtmp1), &(dtmp2), &(dminTime));

          SKA_HCFTP_MultipleOGetMaxStartVelocityOverMinTime(pHCFTP,fabs(pHCFTP->arrWayPoints[i + 1].dWaypointsCoordY - pHCFTP->arrWayPoints[i].dWaypointsCoordY), dTrolleyMaxAcceleration, dTrolleyMaxJerk, stTrolleyShaper,
                                                            dminTime, (pHCFTP->maxWaypointsVelocity[i + 1].dWaypointsCoordY), &(pHCFTP->maxWaypointsVelocity[i].dWaypointsCoordY));
        }
      }
    }
    // 正序确定路径点的最大速度
    // 起点和终点
    pHCFTP->OutputWayPointsVelocity[0].dWaypointsCoordX = 0.0;
    pHCFTP->OutputWayPointsVelocity[0].dWaypointsCoordY = 0.0;
    pHCFTP->OutputWayPointsVelocity[nWayPointsNum - 1].dWaypointsCoordX = 0.0;
    pHCFTP->OutputWayPointsVelocity[nWayPointsNum - 1].dWaypointsCoordY = 0.0;

    if (nWayPointsNum > 2)
    {
      for (int i = 1; i < nWayPointsNum - 1; i++)
      {
        // X方向
        if (pHCFTP->MotionMode[i - 1].nx == 0 || pHCFTP->MotionMode[i - 1].nx == 2)
        {
          pHCFTP->OutputWayPointsVelocity[i].dWaypointsCoordX = 0.0;
        }
        else if (pHCFTP->MotionMode[i - 1].nx == 1)
        {
          SKA_HCFTP_MultipleOGetMaxStartVelocity(pHCFTP, fabs(pHCFTP->arrWayPoints[i].dWaypointsCoordX - pHCFTP->arrWayPoints[i - 1].dWaypointsCoordX),
                                                 dBridgeMaxVelocity, dBridgeMaxAcceleration, dBridgeMaxJerk, stBridgeShaper, &(pHCFTP->OutputWayPointsVelocity[i].dWaypointsCoordX));
          pHCFTP->OutputWayPointsVelocity[i].dWaypointsCoordX = fmin(pHCFTP->OutputWayPointsVelocity[i].dWaypointsCoordX, pHCFTP->maxWaypointsVelocity[i].dWaypointsCoordX);
        }
        else
        {
          pHCFTP->OutputWayPointsVelocity[i].dWaypointsCoordX = pHCFTP->OutputWayPointsVelocity[i - 1].dWaypointsCoordX;
        }

        // Y方向
        if (pHCFTP->MotionMode[i - 1].ny == 0 || pHCFTP->MotionMode[i - 1].ny == 2)
        {
          pHCFTP->OutputWayPointsVelocity[i].dWaypointsCoordY = 0.0;
        }
        else if (pHCFTP->MotionMode[i - 1].ny == 1)
        {
          SKA_HCFTP_MultipleOGetMaxStartVelocity(pHCFTP,fabs(pHCFTP->arrWayPoints[i].dWaypointsCoordY - pHCFTP->arrWayPoints[i - 1].dWaypointsCoordY),
                                                 dTrolleyMaxVelocity, dTrolleyMaxAcceleration, dTrolleyMaxJerk, stTrolleyShaper, &(pHCFTP->OutputWayPointsVelocity[i].dWaypointsCoordY));
          pHCFTP->OutputWayPointsVelocity[i].dWaypointsCoordY = fmin(pHCFTP->OutputWayPointsVelocity[i].dWaypointsCoordY, pHCFTP->maxWaypointsVelocity[i].dWaypointsCoordY);
        }
        else
        {
          pHCFTP->OutputWayPointsVelocity[i].dWaypointsCoordY = pHCFTP->OutputWayPointsVelocity[i - 1].dWaypointsCoordY;
        }
      }
    }

    return 0;
  }

static int8_t SKA_HCFTP_MultipleOGetMaxStartVelocity(SKA_HorizontalCollisonFreeTracePlanner *pHCFTP, double dDistance, double dMaxVelocity, double dMaxAcceleration, double dMaxJerk,
                                                       SKA_InputShaper InputShaper, double *pMaxStartVelocity)
  {
    double dminDist, dminTime, dlo, dhi, dmid;

    SKA_HCFTP_MultipleOGetShapedMinDistAndTime(pHCFTP, dMaxVelocity, 0.0, dMaxAcceleration, dMaxJerk, InputShaper, &(dminDist), &(dminTime));
    if (dminDist <= dDistance)
    {
      *pMaxStartVelocity = dMaxVelocity;
    }
    else
    {
      dlo = 0.0;
      dhi = dMaxVelocity;
      while (dhi - dlo > SKA_FlOAT_ERROR)
      {

        dmid = (dlo + dhi) / 2.0;
        SKA_HCFTP_MultipleOGetShapedMinDistAndTime(pHCFTP, dmid, 0.0, dMaxAcceleration, dMaxJerk, InputShaper, &(dminDist), &(dminTime));
        if (dminDist <= dDistance)
        {
          dlo = dmid;
        }
        else
        {
          dhi = dmid;
        }
      }
      *pMaxStartVelocity = (dlo + dhi) / 2.0;
    }
    return 0;
  }

  static int8_t SKA_HCFTP_MultipleOGetShapedMinDistAndTime(SKA_HorizontalCollisonFreeTracePlanner *pHCFTP, double dStartVelocity, double dTargetVelocity, double dMaxAcceleration,
                                                           double dMaxJerk, SKA_InputShaper InputShaper, double *pMinDistance, double *pMinTime)
  {

    double dtmp, dtN;
    // 如果起始速度小于0.00001，则无需运动
    if (dStartVelocity < SKA_FlOAT_ERROR)
    {
      // 无需运动
      *pMinDistance = 0.0;
      *pMinTime = 0.0;
    }
    else
    {

      // 调用M_GetUnshapedMinDistAndTime_Run函数计算无形状的最小距离和时间
      SKA_UTP_MultipleOGetUnshapedMinDistAndTime(&(pHCFTP->stUnsPlanner), dStartVelocity, dTargetVelocity, dMaxAcceleration, dMaxJerk, pMinDistance, pMinTime);
      dtmp = 0.0;
      // 计算输入形状的脉冲值和脉冲时间的和
      for (int i = 0; i < InputShaper.nImpulseNum; i++)
      {
        dtmp = dtmp + InputShaper.arrImpulseValue[i] * InputShaper.arrImpulseTime[i];
      }

      // 获取输入形状的脉冲时间
      dtN = InputShaper.arrImpulseTime[InputShaper.nImpulseNum - 1];
      // 计算最小距离
      *pMinDistance = *pMinDistance + dStartVelocity * dtmp + dTargetVelocity * (dtN - dtmp);
      // 计算最小时间
      *pMinTime += dtN;
    }
    return 0;
  }

  static int8_t SKA_HCFTP_MultipleOGetShapedTrace(SKA_HorizontalCollisonFreeTracePlanner *pHCFTP, double dCurTime, double dStartPos, double dTargetPos,
                                                  double dStartVelocity, double dTargetVelocity, double dMinTime, bool bActiveSwayCtrl, SKA_InputShaperType eType,
                                                  double dNaturalFreq, double dDampingRatio, double dMaxVelocity, double dMaxAcceleration, double dMaxJerk,  
                                                  SKA_InputShaper InputShaper, double *pPos, double *pVelocity, double *pAcceleration, double *pTotalTime)

  {

    double dfx, dfdx, dfddx;
    bool bisUniform;
    // 无需运动
    if (fabs(dStartPos - dTargetPos) < SKA_FlOAT_ERROR)
    {
      *pPos = dTargetPos;
      *pVelocity = 0.0;
      *pAcceleration = 0.0;
      *pTotalTime = 0.0;
    }
    else
    {
      *pPos = 0.0;
      *pVelocity = 0.0;
      *pAcceleration = 0.0;

      for (int i = 0; i < InputShaper.nImpulseNum; i++)
      {
        SKA_UTP_GetMultipleOUnshapedTrace(&(pHCFTP->stUnsPlanner), dCurTime - InputShaper.arrImpulseTime[i], dStartPos, dTargetPos, dStartVelocity,
                                            dTargetVelocity, dMinTime, bActiveSwayCtrl, eType,
                                            dNaturalFreq, dDampingRatio, dMaxVelocity, dMaxAcceleration, dMaxJerk, InputShaper,
                                            &(dfx), &(dfdx), &(dfddx), pTotalTime, &(bisUniform));

        *pPos += InputShaper.arrImpulseValue[i] * dfx;
        *pVelocity += InputShaper.arrImpulseValue[i] * dfdx;
        *pAcceleration += InputShaper.arrImpulseValue[i] * dfddx;
      }
      if (bisUniform == 0)
      {
        *pTotalTime += InputShaper.arrImpulseTime[InputShaper.nImpulseNum - 1];
      }
    }
    return 0;
  }
  

  static int8_t SKA_HCFTP_MultipleOGetMaxStartVelocityOverMinTime(SKA_HorizontalCollisonFreeTracePlanner *pHCFTP, double dDistance, double dMaxAcceleration, double dMaxJerk, SKA_InputShaper InputShaper,
                                                                  double dMinTime, double dMaxTargetVelocity, double *pMaxStartVelocity)
  {

    double dtN, dt0, dt1, dt2;
    double dv0, ddist0, dlo, dhi;
    double dmid, dminDist, dtmp;
    if (dMinTime < SKA_FlOAT_ERROR)
    {
      // 无最短时间限制
      *pMaxStartVelocity = dMaxTargetVelocity;
      return 0;
    }

    if (dMinTime * dMaxTargetVelocity <= dDistance)
    {
      // 最大目标速度可通过匀速运动达到
      *pMaxStartVelocity = dMaxTargetVelocity;
    }
    else
    {
      // 优先减少运行时间, 再增大开始速度
      dtN = InputShaper.arrImpulseTime[InputShaper.nImpulseNum - 1];
      if (dtN >= dMinTime)
      {
        // 非匀速运动的用时超过MinTime，采用匀速运动
        *pMaxStartVelocity = dDistance / dMinTime;
      }
      else
      {
        // 在保持运行时间为MinTime时，使用非匀速运动来增大开始速度

        // 临界情况
        dt0 = fabs(dMinTime - dtN);

        if (dt0 / 2.0 <= 2.0 * dMaxAcceleration / dMaxJerk)
        {
          dt1 = dt0 / 4.0;
          dt2 = 0.0;
        }
        else
        {
          dt1 = dMaxAcceleration / dMaxJerk;
          dt2 = fabs(dt0 / 2.0 - 2.0 * dt1);
        }
        dv0 = dMaxJerk * dt1 * (dt1 + dt2);
        ddist0 = dMaxJerk * dt1 * (dt1 + dt2) * (2.0 * dt1 + dt2) + dv0 * dtN;

        if (dDistance >= ddist0)
        {
          // 未减速到0
          *pMaxStartVelocity = dv0 + (dDistance - ddist0) / dMinTime;
        }
        else
        {
          // 减速到0
          dlo = 0.0;
          dhi = dv0;
          while (dhi - dlo > SKA_FlOAT_ERROR)
          {
            dmid = (dlo + dhi) / 2.0;

            SKA_UTP_MultipleOGetUnshapedMinDistAndTime(&(pHCFTP->stUnsPlanner),dmid, 0.0, dMaxAcceleration, dMaxJerk, &(dminDist), &(dtmp));

            dminDist = 2.0 * dminDist + dmid * dtN;
            if (dminDist < dDistance)
            {
              dlo = dmid;
            }
            else
            {
              dhi = dmid;
            }
          }
          *pMaxStartVelocity = (dlo + dhi) / 2.0;
        }
        *pMaxStartVelocity = fmin(*pMaxStartVelocity, dMaxTargetVelocity);
      }
    }
    return 0;
  }

  

