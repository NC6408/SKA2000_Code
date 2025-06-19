/**
  ******************************************************************************
  * @file       : SKA_AntiswayController.h
  * @brief      : 防摇控制器
  * @author     : ZhangYi
  * @version    : None
  * @date       : 2024/10/24
  ******************************************************************************
  */
//

#ifndef SKA2000_OHBC_CONTROLLER_SKA_ANTISWAYCONTROLLER_H
#define SKA2000_OHBC_CONTROLLER_SKA_ANTISWAYCONTROLLER_H

#include <stdint.h>

#include "SKA_CommunicationInterface.h"
#include "SKA_RiseEdgeDetector.h"
#include "SKA_TravelUnitTracePlanner.h"
#include "SKA_TravelUnitTrackingController.h"
#include "SKA_Timer.h"
#include "SKA_ZeroSpeedDetector.h"
#include "SKA_AssistAntiswayController.h"
#include "SKA_ZeroManualDetector.h"
#include "SKA_CannotDriveErrorDetector.h"
#include "SKA_HoistUnitTracePlanner.h"
#include "SKA_HoistUnitTrackingController.h"
#include "SKA_VerticalCollisionFreeTracePlanner.h"
#include "SKA_HorizontalCollisonFreeTracePlanner.h"

// 通用控制位
typedef union
{
    struct
    {
        uint8_t bActiveVerticalMode : 1;      // 激活纵向避障模式(高电平触发)
        uint8_t bStartVerticalTask : 1;       // 启动纵向避障任务(上升沿触发)
        uint8_t bIsMaintainMinRopeLength : 1; // 是否保持最短绳长(0:关, 1:开)
        uint8_t bActiveHorizontalMode : 1;    // 激活横向避障模式(高电平触发)
        uint8_t bStartHorizontalTask : 1;     // 启动横向避障任务(上升沿触发)
        uint8_t bIsUseWaypoints : 1;          // 横向避障(0:关, 1:开)
        uint8_t bReset : 1;                   // 复位(高电平触发)
        uint8_t bReboot : 1;                  // 主动重启(0:关, 1:开)
    } stBits;
    uint8_t nData;
} SKA_AscGenCtrlBits;

// 通用输入
typedef struct {
    SKA_AscGenCtrlBits stCtrlBits;          // 控制位
    double dCurRopeLength;                  // 当前绳长(m)
} SKA_AscGeneralInput;

// 运行机构控制位
typedef SKA_ComTravelCtrlBits SKA_AscTravelCtrlBits;

// 运行机构输入
typedef struct {
    double dEffRopeLength;                  // 有效绳长(m)
    SKA_AscTravelCtrlBits stCtrlBits;       // 控制位
    double dCurPos;                         // 当前位置(m)
    double dTargetPos;                      // 目标位置(m)
    double dSpeedLimit;                     // 速度限制(m/s)
    double dManualFreq;                     // 手动输入频率(Hz)
} SKA_AscTravelInput;

// 起升控制位
typedef SKA_ComHoistCtrlBits SKA_AscHoistCtrlBits;

// 起升输入
typedef struct {
    SKA_AscHoistCtrlBits stCtrlBits;        // 控制位
    double dTargetRopeLength;               // 目标绳长(m)
    double dSpeedLimit;                     // 速度限制(m/s)
    double dManualFreq;                     // 手动输入频率(Hz)
} SKA_AscHoistInput;

// 相机状态位
typedef SKA_ComCameraStateBits SKA_AscCameraStateBits;

// 相机错误位
typedef SKA_ComCameraErrorBits SKA_AscCameraErrorBits;

// 相机警告位
typedef SKA_ComCameraWarnBits SKA_AscCameraWarnBits;

// 相机输入
typedef struct {
    SKA_AscCameraStateBits stStateBits;     // 状态位
    SKA_AscCameraErrorBits stErrorBits;     // 错误位
    SKA_AscCameraWarnBits stWarnBits;       // 警告位
    double dSwayPositionX;                  // X方向的摆动位置(m)
    double dSwayVelocityX;                  // X方向的摆动速度(m/s)
    double dSwayPositionY;                  // Y方向的摆动位置(m)
    double dSwayVelocityY;                  // Y方向的摆动速度(m/s)
    double dRotationAngle;                  // 旋转角度(°)
    double dRotationVelocity;               // 旋转角速度(°/s)
} SKA_AscCameraInput;

typedef struct {
    double dDistance;                       // 距离(m)
    double dHeight;                         // 高度(m)
} SKA_AscTerrainPoint;

// 纵向避障输入
typedef struct {
    double dTotalHeight;                        // 总高度(m)
    SKA_AscTerrainPoint arrLiftTerrain[10];     // 升吊地形数组
    uint16_t nLiftArraySize;                    // 升吊地形数组的有效元素数量
    SKA_AscTerrainPoint arrDropTerrain[10];     // 落吊地形数组
    uint16_t nDropArraySize;                    // 落吊地形数组的有效元素数量
} SKA_AscVerticalAvoidanceInput;

typedef struct
{
    double dObstacleCoordX; // x坐标(mm)
    double dObstacleCoordY; // y坐标(mm)
    double dObstacleCoordW; // w坐标(mm)
    double dObstacleCoordH; // h坐标(mm)
} SKA_AscObstacleCoord;     // 静态障碍物坐标信息

typedef struct
{
    double dWaypointsCoordX; // x坐标(mm)
    double dWaypointsCoordY; // y坐标(mm)
} SKA_AscWaypointsCoord;     // 途径点坐标信息

// 横向避障输入
typedef struct
{
    uint16_t nObstacleNum;                  // 静态障碍物数量
    uint16_t nWaypointsNum;                 // 中途经过的路径点数量,取值范围[0,20]
    uint16_t nObstacleInflation;              // 障碍物膨胀因子
    SKA_AscWaypointsCoord arrWaypoints[20]; // 途径点坐标数组
    SKA_AscObstacleCoord arrObstacle[5];    // 静态障碍物坐标数组
} SKA_AscHorizontalAvoidanceInput;

// 防摇控制器输入
typedef struct
{
    SKA_AscGeneralInput stGeneralInput;                // 通用
    SKA_AscTravelInput stBridgeInput;                  // 大车
    SKA_AscTravelInput stTrolleyInput;                 // 小车
    SKA_AscHoistInput stHoistInput;                    // 起升
    SKA_AscCameraInput stCameraInput;                  // 相机
    SKA_AscVerticalAvoidanceInput stVerticalInput;     // 纵向避障
    SKA_AscHorizontalAvoidanceInput stHorizontalInput; // 横向避障
} SKA_AntiswayControllerInput;

// 通用状态位
typedef union {
    struct {
        uint16_t nParamSetIndex : 3;           // 参数集索引;
    } stBits;
    uint8_t nData;
} SKA_AscGenStateBits;

// 通用错误位
typedef union {
    struct {
        uint8_t bIsAscParameterInvalid  : 1;   // 参数集无效
        uint8_t bIsCommunicateInvalid : 1;   // 通讯错误
        uint8_t bIsCurRopeLenInvalid : 1;   // 当前绳长无效
        uint8_t bIsTerrainInfoInvalid   : 1;   // 地形信息无效

    } stBits;
    uint8_t nData;
} SKA_AscGenErrorBits;

// 通用警告位
typedef union {
    struct {
       uint16_t bResetInExecution : 1;    // 任务执行中复位
    } stBits;
    uint8_t nData;
} SKA_AscGenWarnBits;

// 通用输出
typedef struct {
    SKA_AscGenStateBits stStateBits;    // 通用状态位
    SKA_AscGenErrorBits stErrorBits;    // 通用错误位
    SKA_AscGenWarnBits stWarnBits;      // 通用警告位
} SKA_AscGeneralOutput;

// 运行机构状态位
typedef SKA_ComTravelStateBits SKA_AscTravelStateBits;

// 运行机构错误位
typedef SKA_ComTravelErrorBits SKA_AscTravelErrorBits;

// 运行机构警告位
typedef SKA_ComTravelWarnBits SKA_AscTravelWarnBits;

// 运行机构输出
typedef struct {
    SKA_AscTravelStateBits stStateBits;     // 状态位
    SKA_AscTravelErrorBits stErrorBits;     // 错误位
    SKA_AscTravelWarnBits stWarnBits;       // 警告位
    double dOutputFreq;                     // 输出频率(Hz)
    double dOutputSpeed;                    // 输出速度(m/s)
    double dBrakeDist;                    // 带防摇缓停的制动距离(m)
    double dEffRopeLen;                     // 有效绳长(m)
    double dTargetPos;                      // 目标位置(m)
    double dOriTrackErr;                    // 原始跟踪误差(m)
    double dFilTrackErr;                    // 滤波后跟踪误差(m)
} SKA_AscTravelOutput;

// 起升状态位
typedef SKA_ComHoistStateBits SKA_AscHoistStateBits;

// 起升错误位
typedef SKA_ComHoistErrorBits SKA_AscHoistErrorBits;

// 起升警告位
typedef SKA_ComHoistWarnBits SKA_AscHoistWarnBits;

// 起升输出
typedef struct {
    SKA_AscHoistStateBits stStateBits;      // 状态位
    SKA_AscHoistErrorBits stErrorBits;      // 错误位
    SKA_AscHoistWarnBits stWarnBits;        // 警告位
    double dOutputFreq;                     // 输出频率(Hz)
    double dOutputSpeed;                    // 输出速度(m/s)
    double dTargetRopeLength;               // 目标绳长(m)
    double dOriTrackErr;                    // 原始跟踪误差(m)
    double dFilTrackErr;                    // 滤波后跟踪误差(m)
} SKA_AscHoistOutput;

// 防摇控制器输出
typedef struct {
    SKA_AscGeneralOutput stGeneralOutput;       // 通用
    SKA_AscTravelOutput stBridgeOutput;         // 大车
    SKA_AscTravelOutput stTrolleyOutput;        // 小车
    SKA_AscHoistOutput stHoistOutput;           // 起升
} SKA_AntiswayControllerOutput;

// 通用参数
typedef struct {
    double dMinEffRopeLength;           // 最小有效绳长(m)
    double dMaxEffRopeLength;           // 最大有效绳长(m)
    double dTerrainInflation;           // 地形高度膨胀因子(m)
    bool bAdaptiveHeightFlag;           // 是否自适应高度
} SKA_AscGeneralParam;

// 运行机构参数
typedef struct {
    double dMinPos;                     // 最小位置(m)
    double dMaxPos;                     // 最大位置(m)
    double dMaxPosErr;                  // 最大定位误差(m)
    double dStaticSpeed;                // 静止检测的速度阈值(m/s)
    uint8_t nAsLevel;                   // 防摇等级
    double dMaxOutFreq;                 // 最大输出频率(Hz)
    double dMinMoveFreq;                // 最小运动频率(Hz)
    double dMaxOutSpeed;                // 最大输出速度(m/s)
    double dMaxRefSpeed;                // 最大参考速度(m/s)
    double dMaxOutAcc;                  // 最大输出加速度(m/s^2)
    double dMaxRefAcc;                  // 最大参考加速度(m/s^2)
    double dMinAssistSpeed;             // 辅助防摇启动阈值(m/s)
    double dPrelimitSpeed;              // 预限位速度(m/s)
    double dDelayTime;                  // 系统延迟时间(s)
    double dDriveOnDelay;               // 驱动器启动延迟(s)
    double dMaxTrackErr;                // 最大跟踪误差(m)
    double dTrackErrLim;                // 跟踪偏差的限幅滤波(m)
    double dTrackErrRateLim;            // 跟踪偏差的限速滤波(m/s)
    double dTrackErrFilTc;              // 跟踪偏差的惯性滤波(s)
    double dPosTrackKp;                 // 位置跟踪控制器的比例系数Kp(1)
    double dPosTrackKi;                 // 位置跟踪控制器的比例系数Ki(1)
    double dMaxResSway;                 // 最大残余摆动(m)
    double dAntiswayKa;                 // 闭环防摇控制器的增益系数(1)
} SKA_AscTravelParam;

// 起升参数
typedef struct {
    double dMinRopeLen;                 // 最小绳长(m)
    double dMaxRopeLen;                 // 最大绳长(m)
    double dMaxPosErr;                  // 最大定位误差(m)
    double dStaticSpeed;                // 静止检测的速度阈值(m/s)
    double dMaxOutFreq;                 // 最大输出频率(Hz)
    double dMinMoveFreq;                // 最小运动频率(Hz)
    double dMaxOutSpeed;                // 最大输出速度(m/s)
    double dMaxRefSpeed;                // 最大参考速度(m/s)
    double dMaxOutAcc;                  // 最大输出加速度(m/s^2)
    double dMaxRefAcc;                  // 最大参考加速度(m/s^2)
    double dPrelimitSpeed;              // 预限位速度(1%)
    double dDelayTime;                  // 系统延迟时间(s)
    double dMaxTrackErr;                // 最大跟踪误差(m)
    double dTrackErrLim;                // 跟踪偏差的限幅滤波(m)
    double dTrackErrRateLim;            // 跟踪偏差的限速滤波(m/s)
    double dTrackErrFilTc;              // 跟踪偏差的惯性滤波(s)
    double dPosTrackKp;                 // 位置跟踪控制器的比例系数Kp(1)
    double dPosTrackKi;                 // 位置跟踪控制器的比例系数Ki(1)
} SKA_AscHoistParam;

// 防摇控制器参数
typedef struct {
    SKA_AscGeneralParam stGeneralParam;         // 通用
    SKA_AscTravelParam stBridgeParam;           // 大车
    SKA_AscTravelParam stTrolleyParam;          // 小车
    SKA_AscHoistParam stHoistParam;             // 起升
} SKA_AntiswayControllerParam;

// 通用输入指令解析器
typedef struct {
    bool bStartVerticalTask;                        // 启动纵向避障任务
    SKA_RiseEdgeDetector stStartVerticalTaskRED;    // 纵向避障任务启动信号上升沿检测器
    bool bStartHorizontalTask;                     // 启动横向避SKA_AscWaypointsCoordSKA_AscWaypointsCoord障任务
    SKA_RiseEdgeDetector stStartHorizontalTaskRED; // 横向避障任务启动信号上升沿检测器
    bool bQuickStop;                     // 无防摇急停指令
    bool bAscStop;                       // 带防摇缓停指令
    SKA_RiseEdgeDetector stQuickStopRED; // 无防摇急停信号上升沿检测器
    SKA_RiseEdgeDetector stAscStopRED;   // 带防摇缓停信号上升沿检测器
} SKA_AscGeneralInputCmdParser;

// 运行机构输入指令解析器
typedef struct {
    bool bStart;                                    // 启动指令
    bool bQuickStop;                                // 无防摇急停指令
    bool bAscStop;                                  // 带防摇缓停指令
    SKA_RiseEdgeDetector stStartRED;                // 启动信号上升沿检测器
    SKA_RiseEdgeDetector stQuickStopRED;            // 无防摇急停信号上升沿检测器
    SKA_RiseEdgeDetector stAscStopRED;              // 带防摇缓停信号上升沿检测器
} SKA_AscTravelInputCmdParser;

// 起升机构输入指令解析器
typedef struct {
    bool bStart;                                    // 启动指令
    bool bQuickStop;                                // 急停指令
    SKA_RiseEdgeDetector stStartRED;                // 启动信号上升沿检测器
    SKA_RiseEdgeDetector stQuickStopRED;            // 急停信号上升沿检测器
} SKA_AscHoistInputCmdParser;

// 任务模式
typedef enum {
    // 单轴运动模式
    SKA_ASC_SINGLE_MOVE_TASK_MODE,
    // 横向避障模式
    SKA_ASC_HORIZONTAL_AVOIDANCE_TASK_MODE,
    // 纵向避障模式
    SKA_ASC_VERTICAL_AVOIDANCE_TASK_MODE
} SKA_TaskMode;

// 任务状态
typedef enum {
    // 空闲
    SKA_ASC_FREE_TASK_STATE,
    // 准备
    SKA_ASC_PLAN_TASK_STATE,
    // 执行
    SKA_ASC_EXECUTING_TASK_STATE,
    // 完成
    SKA_ASC_FINISH_TASK_STATE
} SKA_TaskState;

// 运行机构单轴运动任务
typedef struct {
    char* pTaskName;                                    // 任务名称
    bool bHasInputErrors;                               // 输入错误标志
    SKA_AscTravelParam stParam;                         // 参数
    uint8_t nOperationMode;                             // 工作模式(0: 手动模式, 1: 自动模式)
    SKA_TaskState eTaskState;                           // 任务状态
    bool bHasBeenBrakeOpened;                           // 抱闸已经开启过的标志
    bool bIsQuickStopping;                              // 正在无防摇急停标志
    bool bIsAscStopping;                                // 正在带防摇缓停标志
    double dTaskStartPos;                               // 当前任务的开始位置(m)
    double dTaskTargetPos;                              // 当前任务的目标位置(m)
    SKA_TravelUnitTracePlanner stTracePlanner;          // 轨迹规划器
    SKA_TravelUnitTrackingController stTrackCtrler;     // 跟踪控制器
    SKA_Timer stTimer;                                  // 计时器
    SKA_ZeroSpeedDetector stZSD;                        // 零速度检测器
    SKA_ZeroManualDetector stZMD;                       // 零手动输入检测器
    SKA_AssistAntiswayController stAAC;                 // 辅助防摇控制器
    bool bIsPreLimiting;                                // 预限位标志
    SKA_CannotDriveErrorDetector stCDED;                // 无法驱动错误检测器
} SKA_TravelSingleMoveTask;

// 起升机构单轴运动任务
typedef struct {
    char *pTaskName;                                    // 任务名称
    bool bHasInputErrors;                               // 输入错误标志
    SKA_AscHoistParam stParam;                          // 参数
    uint8_t nOperationMode;                             // 工作模式(0: 手动模式, 1: 自动模式)
    SKA_TaskState eTaskState;                           // 任务状态
    bool bHasBeenBrakeOpened;                           // 抱闸已经开启过的标志
    bool bIsQuickStopping;                              // 正在无防摇急停标志
    double dTaskStartPos;                               // 当前任务的开始位置(m)
    double dTaskTargetPos;                              // 当前任务的目标位置(m)
    SKA_HoistUnitTracePlanner stTracePlanner;           // 轨迹规划器
    SKA_HoistUnitTrackingController stTrackCtrler;      // 跟踪控制器
    SKA_Timer stTimer;                                  // 计时器
    SKA_ZeroSpeedDetector stZSD;                        // 零速度检测器
    SKA_ZeroManualDetector stZMD;                       // 零手动输入检测器
    bool bIsPreLimiting;                                // 预限位标志
    SKA_CannotDriveErrorDetector stCDED;                // 无法驱动错误检测器
} SKA_HoistSingleMoveTask;

// 纵向避障任务
typedef struct {
    char *pTaskName;                                            // 任务名称
    bool bHasInputErrors;                               // 输入错误标志
    SKA_TaskState eTaskState;                                   // 任务状态
    SKA_AscGeneralParam stGeneralParam;                         // 通用
    SKA_AscTravelParam stBridgeParam;                           // 大车
    SKA_AscTravelParam stTrolleyParam;                          // 小车
    SKA_AscHoistParam stHoistParam;                             // 起升
    uint8_t nOperationMode;                                     // 工作模式(0: 手动模式, 1: 自动模式)
    bool bHasBeenBrakeOpened;                                   // 大小车抱闸已经开启过的标志
    double dBridgeTaskStartPos;                                 // 大车当前任务的开始位置(m)
    double dBridgeTaskTargetPos;                                // 大车当前任务的目标位置(m)
    double dTrolleyTaskStartPos;                                // 小车当前任务的开始位置(m)
    double dTrolleyTaskTargetPos;                               // 小车当前任务的目标位置(m)
    double dHoistTaskStartRopeLength;                           // 起升当前任务的开始位置(m)
    double dHoistTaskTargetRopeLength;                          // 起升当前任务的目标位置(m)
    double dHoistMinRopeLength;                                 // 起升机构起吊的最短绳长（即起升机构的最高高度）
    double dTravelUnitMaxDelayTime;                             // 运行机构开始运行的最大延迟时间(s)
    double dHoistUnitMaxStartDownTime;                          // 起升机构最大开始下降时间(s)
    SKA_Timer stTimer;                                         // 计时器
    SKA_ZeroSpeedDetector stBridgeZSD;                         // 大车零速度检测器
    SKA_ZeroSpeedDetector stTrolleyZSD;                        // 小车零速度检测器
    SKA_ZeroSpeedDetector stHoistZSD;                          // 起升机构零速度检测器
    SKA_CannotDriveErrorDetector stBridgeCDED;                 // 大车无法驱动错误检测器
    SKA_CannotDriveErrorDetector stTrolleyCDED;                // 小车无法驱动错误检测器
    SKA_CannotDriveErrorDetector stHoistCDED;                  // 起升机构无法驱动错误检测器
    SKA_VerticalCollisionFreeTracePlanner stVCFTracePlanner;     // 纵向避障轨迹规划器
    SKA_TravelUnitTrackingController stBridgeTrackCtrler;      // 大车跟踪控制器
    SKA_TravelUnitTrackingController stTrolleyTrackCtrler;     // 小车跟踪控制器
    SKA_HoistUnitTrackingController stHoistTrackCtrler;        // 起升跟踪控制器
} SKA_VerticalAvoidanceTask;

// 横向避障任务
typedef struct
{
    char *pTaskName;                                          // 任务名称
    bool bHasInputErrors;                                    // 输入错误标志
    SKA_TaskState eTaskState;                                 // 任务状态
    SKA_AscGeneralParam stGeneralParam;                       // 通用
    SKA_AscTravelParam stBridgeParam;                         // 大车
    SKA_AscTravelParam stTrolleyParam;                        // 小车
    SKA_AscHoistParam stHoistParam;                           // 起升
    uint8_t nOperationMode;                                   // 工作模式(0: 手动模式, 1: 自动模式)
    bool bHasBeenBrakeOpened;                                 // 大小车抱闸已经开启过的标志
    double dBridgeTaskStartPos;                               // 大车当前任务的开始位置(m)
    double dBridgeTaskTargetPos;                              // 大车当前任务的目标位置(m)
    double dTrolleyTaskStartPos;                              // 小车当前任务的开始位置(m)
    double dTrolleyTaskTargetPos;                             // 小车当前任务的目标位置(m)
    // SKA_AscObstacleCoord arrObstacle[5];                      // 静态障碍物坐标数组
    // SKA_AscWaypointsCoord arrWaypoints[20];                   // 途径点坐标数组
    SKA_AscObstacleCoord arrInflatedObstacle[5];              // 静态膨胀后障碍物坐标数组
    SKA_Timer stTimer;                                        // 计时器
    SKA_ZeroSpeedDetector stBridgeZSD;                        // 大车零速度检测器
    SKA_ZeroSpeedDetector stTrolleyZSD;                       // 小车零速度检测器
    SKA_CannotDriveErrorDetector stBridgeCDED;                // 大车无法驱动错误检测器
    SKA_CannotDriveErrorDetector stTrolleyCDED;               // 小车无法驱动错误检测器
    SKA_HorizontalCollisonFreeTracePlanner stHCFTracePlanner; // 横向避障轨迹规划器
    double dSingleOTotalTimeX;                                // 单个障碍物时大车运行任务总时间
    double dSingleOTotalTimeY;                                // 单个障碍物时小车运行任务总时间
    double dSingleOTotalTime;                                 // 单个障碍物时运行任务总时间
    uint8_t nSingleOSubTasksNumX;                             // 单个障碍物时大车子任务数量
    uint8_t nSingleOSubTasksNumY;                             // 单个障碍物时小车任务数量
    double dMultipleOTotalTimeX;                              // 多个障碍物时大车运行任务总时间
    double dMultipleOTotalTimeY;                              // 多个障碍物时小车运行任务总时间
    double dMultipleOTotalTime;                               // 多个障碍物时运行任务总时间
    uint8_t nMultipleOSubTasksNumX;                           // 多个障碍物时大车子任务数量
    uint8_t nMultipleOSubTasksNumY;                           // 多个障碍物时小车任务数量
    SKA_TravelUnitTrackingController stBridgeTrackCtrler;     // 大车跟踪控制器
    SKA_TravelUnitTrackingController stTrolleyTrackCtrler;    // 小车跟踪控制器
} SKA_HorizontalAvoidanceTask;

// 防摇控制器
typedef struct {
    double dCtrlPeriod;                                 // 控制周期(s)
    SKA_AntiswayControllerInput stInput;                // 输入
    SKA_AntiswayControllerOutput stOutput;              // 输出
    SKA_AntiswayControllerParam stParam;                // 参数
    SKA_AntiswayControllerParam stUsingParam;           // 使用中的参数
    SKA_AscGeneralInputCmdParser stGeneralICP;          // 通用输入指令解析器
    SKA_AscTravelInputCmdParser stBridgeICP;            // 大车输入指令解析器
    SKA_AscTravelInputCmdParser stTrolleyICP;           // 小车输入指令解析器
    SKA_AscHoistInputCmdParser stHoistICP;              // 起升输入指令解析器
    SKA_TaskMode eTaskMode;                             // 任务模式
    SKA_TravelSingleMoveTask stBridgeTask;              // 大车单轴运动任务
    SKA_TravelSingleMoveTask stTrolleyTask;             // 小车单轴运动任务
    SKA_HoistSingleMoveTask stHoistTask;                // 起升单轴运动任务
    SKA_HorizontalAvoidanceTask stHorizontalTask;       // 横向避障任务
    SKA_VerticalAvoidanceTask stVerticalTask;           // 纵向避障任务
} SKA_AntiswayController;

/**
 * @brief 初始化防摇控制器
 * @param pASC 防摇控制器地址
 * @param dCtrlPeriod 控制周期
 * @return 错误码 {0: 正常}
 */
int8_t SKA_ASC_Init(SKA_AntiswayController *pASC, double dCtrlPeriod);

/**
 * @brief 设置防摇控制器输入
 * @param pAscInput 防摇控制器输入的地址
 * @param pComInput 通信输入的地址
 * @return 错误码 {0: 正常}
 */
int8_t SKA_ASC_SetInput(SKA_AntiswayControllerInput *pAscInput, SKA_ComAntiswayControllerInput *pComInput);

/**
 * @brief 设置防摇控制器参数
 * @param pAscParam 防摇控制器参数的地址
 * @param pComParam 通信参数的地址
 * @return 错误码 {0: 正常}
 */
int8_t SKA_ASC_SetParam(SKA_AntiswayControllerParam *pAscParam, SKA_ComAntiswayControllerParam *pComParam);

/**
 * @brief 获取防摇控制器输出
 * @param pAscOutput 防摇控制器输出的地址
 * @param pComOutput 通信输出的地址
 * @return 错误码 {0: 正常}
 */
int8_t SKA_ASC_GetOutput(SKA_AntiswayControllerOutput *pAscOutput, SKA_ComAntiswayControllerOutput *pComOutput);

/**
 * @brief 运行防摇控制器, 约定该函数按照控制周期被循环调用
 * @param pASC 防摇控制器地址
 * @return 错误码 {0: 正常}
 */
int8_t SKA_ASC_Run(SKA_AntiswayController *pASC);

/**
 * @brief 纵向避障任务状态切换
 * @param dCtrlPeriod 控制周期
 * @param pTask 纵向避障运动任务指针
 * @param stVerticalInput 纵向避障输入指针
 * @param pBridgeInput 大车输入指针
 * @param pTrolleyInput 小车输入指针
 * @param pHoistInput 起升机构输入指针
 * @param stGeneralInput 通用输入指针
 * @param pBridgeParam 大车参数指针
 * @param pTrolleyParam 小车参数指针
 * @param pHoistParam 起升机构参数指针
 * @param pGeneralOutput 通用输出指针
 * @param pBridgeOutput 大车输出指针
 * @param pTrolleyOutput 小车输出指针
 * @param pHoistOutput 起升机构输出指针
 * @param pCmdParser 输入指令解析器指针
 * @return 错误码 {0: 正常}
 */
int8_t SKA_ASC_VerticalAvoidanceTaskStateTransition(double dCtrlPeriod,
                                                    SKA_VerticalAvoidanceTask *pTask,
                                                    SKA_AscVerticalAvoidanceInput *pstVerticalInput,
                                                    SKA_AscTravelInput *pBridgeInput,
                                                    SKA_AscTravelInput *pTrolleyInput,
                                                    SKA_AscHoistInput  *pHoistInput,
                                                    SKA_AscGeneralInput *pstGeneralInput,
                                                    SKA_AscTravelParam *pBridgeParam,  
                                                    SKA_AscTravelParam *pTrolleyParam,
                                                    SKA_AscHoistParam *pHoistParam, 
                                                    SKA_AscGeneralOutput *pGeneralOutput,
                                                    SKA_AscTravelOutput *pBridgeOutput,
                                                    SKA_AscTravelOutput *pTrolleyOutput,
                                                    SKA_AscHoistOutput *pHoistOutput,
                                                    SKA_AscGeneralInputCmdParser *pCmdParser) ;
/**
 * @brief 横向避障任务状态切换
 * @param dCtrlPeriod 控制周期
 * @param pTask 横向避障运动任务指针
 * @param pstHorizontalInput 横向避障输入指针
 * @param pBridgeInput 大车输入指针
 * @param pTrolleyInput 小车输入指针
 * @param stGeneralInput 通用输入指针
 * @param pBridgeParam 大车参数指针
 * @param pTrolleyParam 小车参数指针
 * @param pGeneralOutput 通用输出指针
 * @param pBridgeOutput 大车输出指针
 * @param pTrolleyOutput 小车输出指针
 * @param pCmdParser 输入指令解析器指针
 * @return 错误码 {0: 正常}
 */
int8_t SKA_ASC_HorizontalAvoidanceTaskStateTransition(double dCtrlPeriod,
                                                      SKA_HorizontalAvoidanceTask *pTask,
                                                      SKA_AscHorizontalAvoidanceInput *pstHorizontalInput,
                                                      SKA_AscTravelInput *pBridgeInput,
                                                      SKA_AscTravelInput *pTrolleyInput,
                                                      SKA_AscGeneralInput *pstGeneralInput,
                                                      SKA_AscTravelParam *pBridgeParam,
                                                      SKA_AscTravelParam *pTrolleyParam,
                                                      SKA_AscGeneralOutput *pGeneralOutput,
                                                      SKA_AscTravelOutput *pBridgeOutput,
                                                      SKA_AscTravelOutput *pTrolleyOutput,
                                                      SKA_AscGeneralInputCmdParser *pCmdParser);
#endif //SKA2000_OHBC_CONTROLLER_SKA_ANTISWAYCONTROLLER_H
