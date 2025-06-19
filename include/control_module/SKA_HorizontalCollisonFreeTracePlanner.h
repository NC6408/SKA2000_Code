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

#ifndef SKA2000_OHBC_CONTROLLER_SKA_HORIZONTALCOLLISONFREETRACEPLANNER_H
#define SKA2000_OHBC_CONTROLLER_SKA_HORIZONTALCOLLISONFREETRACEPLANNER_H

#include <stddef.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>

#include "SKA_TravelUnitTracePlanner.h"
#include "SKA_InputShaper.h"

typedef struct
{
  double dObstacleCoordX; // x坐标(mm)
  double dObstacleCoordY; // y坐标(mm)
  double dObstacleCoordW; // w坐标(mm)
  double dObstacleCoordH; // h坐标(mm)
} SKA_HCFTPObstacleCoord; // 静态障碍物坐标信息

typedef struct
{
  double dWaypointsCoordX; // x坐标(mm)
  double dWaypointsCoordY; // y坐标(mm)
} SKA_HCFTPWaypointsCoord; // 途经点径点坐标信息

typedef struct
{
  double dStartTime; // 开始时间(s)
  double dStartPos;  // 开始位置(mm)
  double dTargetPos; // 目标位置(mm)
} SKA_HCFTPSingleOSubTask;

typedef struct
{
  double dStartTime;      // 开始时间(s)
  double dStartPos;       // 开始位置(mm)
  double dTargetPos;      // 目标位置(mm)
  double dStartVelocity;  // 开始速度(mm/s)
  double dTargetVelocity; // 目标速度(mm/s)
  double dMinTime;        // 最小时间(s)
} SKA_HCFTPMultipleOSubTask;

typedef struct
{
  int8_t nx; // x轴方向
  int8_t ny; // y轴方向
} SKA_HCFTPMultipleOMotion;


typedef struct
{
  double dCoordX;            // X坐标
  double dCoordY;            // Y坐标
  double dSingleSourceSPath; // 单源最短路径
  int8_t nPrecursorNode;     // 前驱结点
} SKA_HCFTPVertex;


typedef struct
{

  /** 配置参数 **/
  bool bBridgeIsQuickStopping;                      // 大车正在无防摇急停标志
  bool bBridgeActiveSwayCtrl;                       // 激活大车防摇控制
  double dBridgeQuickStopMaxAcc;                    // 大车无防摇急停最大加速度(m/s^2)
  SKA_InputShaperType BridgeeType;                  // 输入整形器类型
  double dBridgeNaturalFreq;                        // 自然频率(rad/s)
  double dBridgeDampingRatio;                       // 阻尼比
  double dBridgeStartPos;                           // 大车开始位置(m)
  double dBridgeTargetPos;                          // 大车目标位置(m)
  double dBridgeMaxVelocity;                        // 大车最大速度(m/s)
  double dBridgeMaxAcceleration;                    // 大车最大加速度(m/s^2)
  double dBridgeMaxJerk;                            // 大车最大加加速度(m/s^3)
  bool bTrolleyIsQuickStopping;                     // 小车正在无防摇急停标志
  bool bTrolleyActiveSwayCtrl;                      // 激活防摇控制
  double dTrolleyQuickStopMaxAcc;                   // 小车无防摇急停最大加速度(m/s^2)
  SKA_InputShaperType TrolleyeType;                 // 小车输入整形类型
  double dTrolleyNaturalFreq;                       // 自然频率(rad/s)
  double dTrolleyDampingRatio;                      // 阻尼比
  double dTrolleyStartPos;                          // 小车开始位置(m)
  double dTrolleyTargetPos;                         // 小车目标位置(m)
  double dTrolleyMaxVelocity;                       // 小车最大速度(m/s)
  double dTrolleyMaxAcceleration;                   // 小车最大加速度(m/s^2)
  double dTrolleyMaxJerk;                           // 小车最大加加速度(m/s^3)
  SKA_InputShaper stBridgeShaper;                   // 大车输入整形器
  SKA_InputShaper stTrolleyShaper;                  // 小车输入整形器
  SKA_TravelUnitTracePlanner stTracePlanner;  // 整形轨迹规划器
  SKA_TravelUnitTracePlanner stBridgeTracePlanner;  // 大车整形轨迹规划器
  SKA_TravelUnitTracePlanner stTrolleyTracePlanner; // 小车整形轨迹规划器
  // 障碍物
  bool bIsUseWaypoints;
  uint16_t nObstacleNum;
  double dObstacleInflation;
  uint16_t nWaypointsNum;
  double dBridgeMaxPos;
  double dBridgeMinPos;
  double dTrolleyMaxPos;
  double dTrolleyMinPos;
  uint8_t bActiveHorizontalMode;
  // 单障碍物
  SKA_HCFTPObstacleCoord Obstacle;
  SKA_HCFTPObstacleCoord tmpObstacle;                   // 障碍物坐标
  SKA_HCFTPSingleOSubTask tmpSubTasks[2];                        // 子任务
  uint8_t ntmpNum;                                      // 子任务数量
  double ftmpTotalTime;                                 // 子任务总时间
  SKA_HCFTPSingleOSubTask SubTasksX_R[2];                        // 右侧轨迹的大车子任务
  uint8_t nSubTasksNumX_R;                              // 右侧轨迹的大车子任务数量
  double dTotalTimeX_R;                                 // 右侧轨迹的大车子任务总时间
  SKA_HCFTPSingleOSubTask SubTasksY_R[2];                        // 右侧轨迹的小车子任务
  uint8_t nSubTasksNumY_R;                              // 右侧轨迹的小车子任务数量
  double dTotalTimeY_R;                                 // 右侧轨迹的小车子任务总时间
  double dTotalTime_R;                                  // 右侧轨迹的总时间
  SKA_HCFTPSingleOSubTask SubTasksX_L[2];                        // 左侧轨迹的大车子任务
  uint8_t nSubTasksNumX_L;                              // 左侧轨迹的大车子任务数量
  double dTotalTimeX_L;                                 // 左侧轨迹的大车子任务总时间
  SKA_HCFTPSingleOSubTask SubTasksY_L[2];                        // 左侧轨迹的小车子任务
  uint8_t nSubTasksNumY_L;                              // 左侧轨迹的小车子任务数量
  double dTotalTimeY_L;                                 // 左侧轨迹的小车子任务总时间
  double dTotalTime_L;                                  // 左侧轨迹的总时间

  SKA_HCFTPSingleOSubTask stSOBridgeSubTask[2];                  // 单障碍物避障大车子任务
  SKA_HCFTPSingleOSubTask stSOTrolleySubTask[2];                 // 单障碍物避障小车子任务
  // 多障碍物
  SKA_HCFTPWaypointsCoord MidWaypoints[20];                           // 中途路径点信息
  SKA_HCFTPObstacleCoord MObstacles[5];                                // 障碍物信息
  SKA_HCFTPMultipleOSubTask stBridgeSubTask[21];                               // 大车任务信息
  SKA_HCFTPMultipleOSubTask stTrolleySubTask[21];                              // 小车任务信息
  uint8_t waypointsNum;                                               // 路径点数量
  SKA_HCFTPWaypointsCoord waypoints[22];                              // 路径点信息
  SKA_HCFTPMultipleOMotion motionDirect[22];                                   // 运动方向
  SKA_HCFTPMultipleOMotion motionMode[22];                                     // 运动模式
  SKA_HCFTPWaypointsCoord waypointsVelocity[22];                      // 路径点速度信息

  // SKA_SingleOGetShapedTraceAndTotalTime stGetShapedTraceAndTotalTime; // 获取整形轨迹和总时间
  SKA_HCFTPMultipleOSubTask stMOBridgeSubTask[21];                             // 多障碍物避障大车任务信息
  SKA_HCFTPMultipleOSubTask stMOTrolleySubTask[21];                            // 多障碍物避障小车任务信息

//SKA_HCFTP_MultipleOGetWayPointsVelocity
  SKA_HCFTPMultipleOMotion MotionMode[22];                                     // 运行模式数组
  SKA_HCFTPWaypointsCoord arrWayPoints[22];                              // 路径点信息
  SKA_HCFTPWaypointsCoord OutputWayPointsVelocity[22];                // 输出路径点速度
  SKA_HCFTPWaypointsCoord maxWaypointsVelocity[22];                   // 最大途径点速度

//SKA_HCFTP_MultipleOGetWayPoints
  uint8_t vertexNum;                     // 顶点个数
  SKA_HCFTPVertex vertex[22];                     // 顶点信息数组
  double edge[484];                      // 边权重数组
  bool isInQ[22];                        // 队列标志数组
  uint8_t sizeOfQ;                       // 队列大小
  SKA_HCFTPWaypointsCoord ArrWayPoints[22]; // 路径点信息
  SKA_HCFTPObstacleCoord ArrObstacles[5];   // 障碍物信息
  
  //SKA_HCFTP_SingleOGetNormalRightTrace
  SKA_UnshapedTracePlanner stUnsPlanner; // 未整形轨迹规划器
  uint8_t pBridgeSubTasksNum;
  double pBridgeTotalTime;
  uint8_t pTrolleySubTasksNum;
  double pTrolleyTotalTime;
  double pTotalTime;
} SKA_HorizontalCollisonFreeTracePlanner;

/**
 * @brief 创建无障碍物运行机构轨迹规划器
 * @param pNOTP 无障碍物轨迹规划器的地址
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
 * @return 错误码 {0: 正常}
 */

int8_t SKA_HCFTP_Create(SKA_HorizontalCollisonFreeTracePlanner *pHCFTP, bool bBridgeActiveSwayCtrl, SKA_InputShaperType BridgeeType,
                        double dBridgeNaturalFreq, double dBridgeDampingRatio, double dBridgeStartPos, double dBridgeTargetPos,
                        double dBridgeMaxVelocity, double dBridgeMaxAcceleration, double dBridgeQuickStopMaxAcc, double dBridgeMaxPos,
                        double dBridgeMinPos, bool bTrolleyActiveSwayCtrl, SKA_InputShaperType TrolleyeType, double dTrolleyNaturalFreq,
                        double dTrolleyDampingRatio, double dTrolleyStartPos, double dTrolleyTargetPos, double dTrolleyMaxVelocity,
                        double dTrolleyMaxAcceleration, double dTrolleyQuickStopMaxAcc, double dTrolleyMaxPos, double dTrolleyMinPos);

/**
 * @brief 创建单个障碍物运行机构轨迹规划器
 * @param pHCFTP 单个障碍物轨迹规划器的地址
 * @param bBridgeActiveSwayCtrl 激活大车防摇控制
 * @param BridgeeType 大车输入整形器类型
 * @param dBridgeNaturalFreq 大车自然频率(rad/s)
 * @param dBridgeDampingRatio 大车阻尼比
 * @param dBridgeStartPos 大车开始位置(m)
 * @param dBridgeTargetPos 大车目标位置(m)
 * @param dBridgeMaxVel 大车最大速度(m/s)
 * @param dBridgeMaxAcc 大车最大加速度(m/s^2)
 * @param dBridgeMaxJerk 大车最大加加速度(m/s^3)
 * @param dBridgeMaxPos 允许大车运行的最大位置(m)
 * @param dBridgeMinPos 允许大车运行的最小位置(m)
 * @param bTrolleyActiveSwayCtrl 激活小车防摇控制
 * @param TrolleyType 小车输入整形器类型
 * @param dTrolleyNaturalFreq 小车自然频率(rad/s)
 * @param dTrolleyDampingRatio 小车阻尼比
 * @param dTrolleyStartPos 小车开始位置(m)
 * @param dTrolleyTargetPos 小车目标位置(m)
 * @param dTrolleyMaxVelocity 小车最大速度(m/s)
 * @param dTrolleyMaxAcc 小车最大加速度(m/s^2)
 * @param dTrolleyMaxJerk 小车最大加加速度(m/s^3)
 * @param dTrolleyMaxPos 允许小车运行的最大位置(m)
 * @param dTrolleyMinPos 允许小车运行的最小位置(m)
 * @param ObstacleCoord 单个障碍物坐标信息
 * @param pBridgeSubTasksNum 存储大车子任务数量的地址
 * @param pBridgeTotalTime 存储大车总运行时间的地址
 * @param pTrolleySubTasksNum 存储小车子任务数量的地址
 * @param pTrolleyTotalTime 存储小车总运行时间的地址
 * @param pTotalTime 存储总运行时间的地址
 * @return 错误码 {0: 正常}
 */

int8_t SKA_HCFTP_Run(SKA_HorizontalCollisonFreeTracePlanner *pHCFTP, double dCurTime,
                     double *pBridgeRefX, double *pBridgeRefDx, double *pBridgeRefDdx, double *pBridgeUnsRefx, double *pBridgeUnsRefDx, double *pBridgeUnsRefDdx,
                     double *pTrolleyRefX, double *pTrolleyRefDx, double *pTrolleyRefDdx, double *pTrolleyUnsRefx, double *pTrolleyUnsRefDx, double *pTrolleyUnsRefDdx);

int8_t SKA_HCFTP_Destroy(SKA_HorizontalCollisonFreeTracePlanner * pHCFTP);
/**
 * @brief 获取单障碍物横向避障整形轨迹和运行总时间
 * @param pSOGTAT 获取单障碍物横向避障整形轨迹和运行总时间的地址
 * @param dCurTime 当前时间
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
 * @param pPos 存储整形位置(m)的地址
 * @param pVel 存储整形速度(m/s)的地址
 * @param pAcc 存储整形加速度(m/s^2)的地址
 * @param pTotaltime 存储整形轨迹总时间的地址(s)
 * @return 错误码 {0: 正常}
 */

#endif // SKA2000_OHBC_CONTROLLER_SKA_HORIZONTALCOLLISONFREETRACEPLANNER_H
