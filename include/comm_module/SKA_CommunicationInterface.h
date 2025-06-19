/**
  ******************************************************************************
  * @file       : SKA_CommunicationInterface.h
  * @brief      : 通信接口
  * @author     : ZhangYi
  * @version    : None
  * @date       : 2024/10/30
  ******************************************************************************
  */
//

#ifndef SKA2000_OHBC_CONTROLLER_SKA_COMMUNICATIONINTERFACE_H
#define SKA2000_OHBC_CONTROLLER_SKA_COMMUNICATIONINTERFACE_H

#include <stdint.h>
#include <stdbool.h>
#include <cJSON.h>

#pragma pack(push, 1) // 确保结构体按照1字节对齐，避免填充字节

// 通用控制位
typedef union
{
    struct
    {
        uint16_t nParamSetIndex : 3;           // 参数集索引
        uint16_t bActiveVerticalMode : 1;      // 激活纵向避障模式(高电平触发)
        uint16_t bStartVerticalTask : 1;       // 启动纵向避障任务(上升沿触发)
        uint16_t bIsMaintainMinRopeLength : 1; // 是否保持最短绳长(0:关, 1:开)
        uint16_t bActiveHorizontalMode : 1;    // 激活横向避障模式(高电平触发)
        uint16_t bStartHorizontalTask : 1;     // 启动横向避障任务(上升沿触发)
        uint16_t bIsUseWaypoints  : 1;         // 是否途经点(0:关, 1:开)
        uint16_t bReset : 1;                   // 复位(高电平触发)
        uint16_t bReboot : 1;                  // 主动重启(0:关, 1:开)
    } stBits;
    uint16_t nData;
} SKA_ComGenCtrlBits;

// 定义报文头结构体
typedef struct {
    uint16_t serverIpSuffix;   // 服务端IP地址的后两位
    uint16_t dataLength;       // 报文数据的长度
} SKA_ComPacketHeader;
 

// 通用输入
typedef struct {
    uint16_t nPlcHeartbeat;             // PL心跳
    SKA_ComGenCtrlBits stCtrlBits;      // 通用控制位
    int32_t nCurRopeLength;             // 当前绳长(mm)
} SKA_ComGeneralInput;

// 运行机构控制位
typedef union {
    struct {
        uint16_t bEnable : 1;           // 控制器使能(高电平触发)
        uint16_t bStart : 1;            // 启动新任务(上升沿触发)
        uint16_t bReset : 1;            // 复位(高电平触发)
        uint16_t bQuickStop : 1;        // 无防摇急停(上升沿触发)
        uint16_t bAntiswayStop : 1;     // 带防摇缓停(上升沿触发)
        uint16_t bPrelimit : 1;         // 预限位减速(高电平触发)
        uint16_t nOperationMode : 2;    // 工作模式(0:手动模式, 1:自动模式)
        uint16_t bIsBrakeOpened : 1;    // 抱闸状态反馈(0:关, 1:开)
        uint16_t bActiveSwayCtrl : 1;   // 激活防摇控制(0:关, 1:开)
        uint16_t bActivePosCtrl : 1;    // 激活定位控制(0:关, 1:开)
        uint16_t bActiveCamera : 1;     // 激活闭环防摇(0:关, 1:开)
    } stBits;
    uint16_t nData;
} SKA_ComTravelCtrlBits;

// 运行机构输入
typedef struct {
    SKA_ComTravelCtrlBits stCtrlBits;       // 控制位
    int16_t nRopeOffset;                    // 绳长补偿(mm)
    int32_t nCurPos;                        // 当前位置(mm)
    int32_t nTargetPos;                     // 目标位置(mm)
    uint16_t nSpeedLimit;                   // 速度限制(mm/s)
    int16_t nManualFreq;                    // 手动输入频率(0.01Hz)
} SKA_ComTravelInput;

// 起升控制位
typedef union {
    struct {
        uint16_t bEnable : 1;           // 控制器使能(高电平触发)
        uint16_t bStart : 1;            // 启动新任务(上升沿触发)
        uint16_t bReset : 1;            // 复位(高电平触发)
        uint16_t bQuickStop : 1;        // 急停(上升沿触发)
        uint16_t bPrelimit : 1;         // 预限位减速(高电平触发)
        uint16_t nOperationMode : 1;    // 工作模式(0:手动模式, 1:自动模式)
        uint16_t bIsBrakeOpened : 1;    // 抱闸状态反馈(0:关, 1:开)
        uint16_t bActivePosCtrl : 1;    // 激活定位控制(0:关, 1:开)
    } stBits;
    uint16_t nData;
} SKA_ComHoistCtrlBits;

// 起升输入
typedef struct {
    SKA_ComHoistCtrlBits stCtrlBits;        // 控制位
    int32_t nTargetRopeLength;              // 目标绳长(mm)
    uint16_t nSpeedLimit;                   // 速度限制(mm/s)
    int16_t nManualFreq;                    // 手动输入频率(0.01Hz)
} SKA_ComHoistInput;

// 相机状态位
typedef union {
    struct {
        uint16_t nSpare : 16;
    } stBits;
    uint16_t nData;
} SKA_ComCameraStateBits;

// 相机错误位
typedef union {
    struct {
        uint16_t nSpare : 16;
    } stBits;
    uint16_t nData;
} SKA_ComCameraErrorBits;

// 相机警告位
typedef union {
    struct {
        uint16_t nSpare : 16;
    } stBits;
    uint16_t nData;
} SKA_ComCameraWarnBits;

// 相机输入
typedef struct {
    SKA_ComCameraStateBits stStateBits;     // 状态位
    SKA_ComCameraErrorBits stErrorBits;     // 错误位
    SKA_ComCameraWarnBits stWarnBits;       // 警告位
    int16_t nSwayPositionX;                 // X方向的摆动位置(mm)
    int16_t nSwayVelocityX;                 // X方向的摆动速度(mm/s)
    int16_t nSwayPositionY;                 // Y方向的摆动位置(mm)
    int16_t nSwayVelocityY;                 // Y方向的摆动速度(mm/s)
    int16_t nRotationAngle;                 // 旋转角度(0.01°)
    int16_t nRotationVelocity;              // 旋转角速度(0.01°/s)
} SKA_ComCameraInput;

typedef struct {
    uint16_t nDistance;                     // 距离(cm)
    int16_t nHeight;                        // 高度(cm)
} SKA_ComTerrainPoint;

// 纵向避障输入
typedef struct {
    int32_t nTotalHeight;                           // 总高度(mm)
    SKA_ComTerrainPoint arrLiftTerrain[10];         // 升吊地形数组
    uint16_t nLiftArraySize;                        // 升吊地形数组的有效元素数量
    SKA_ComTerrainPoint arrDropTerrain[10];         // 落吊地形数组
    uint16_t nDropArraySize;                        // 落吊地形数组的有效元素数量
} SKA_ComVerticalAvoidanceInput;

typedef struct
{
    int32_t nObstacleCoordX; // x坐标(mm)
    int16_t nObstacleCoordY; // y坐标(mm)
    int32_t nObstacleCoordW; // w坐标(mm)
    int16_t nObstacleCoordH; // h坐标(mm)
} SKA_ComObstacleCoordinate; // 静态障碍物坐标信息

typedef struct
{
    int32_t nWaypointsCoordX; // x坐标(mm)
    int16_t nWaypointsCoordY; // y坐标(mm)
} SKA_ComWaypointsCoordinate; // 途经点径点坐标信息

// 横向避障输入
typedef struct
{
    uint16_t nObstacleNum;                       // 静态障碍物数量
    uint16_t nWaypointsNum;                      // 中途经过的路径点数量,取值范围[0,20]
    uint16_t nObstacleInflation;                 // 障碍物膨胀因子
    SKA_ComWaypointsCoordinate arrWaypoints[20]; // 途径点坐标数组
    SKA_ComObstacleCoordinate arrObstacle[5];    // 静态障碍物坐标数组

} SKA_ComHorizontalAvoidanceInput;

// 防摇控制器输入
typedef volatile struct
{
    SKA_ComPacketHeader stReceivePacketHeader;         // 接收数据包头
    SKA_ComGeneralInput stGeneralInput;                // 通用
    SKA_ComTravelInput stBridgeInput;                  // 大车
    SKA_ComTravelInput stTrolleyInput;                 // 小车
    SKA_ComHoistInput stHoistInput;                    // 起升
    SKA_ComCameraInput stCameraInput;                  // 相机
    SKA_ComVerticalAvoidanceInput stVerticalInput;     // 纵向避障
    SKA_ComHorizontalAvoidanceInput stHorizontalInput; // 横向避障
} SKA_ComAntiswayControllerInput;

// 通用状态位
typedef union {
    struct {
        uint16_t nSelectedParamSetIndex : 3;    // 被选择的参数集索引
    } stBits;
    uint16_t nData;
} SKA_ComGenStateBits;

// 通用错误位
typedef union {
    struct {
        uint16_t bIsParamSetInvalid : 1;        // 参数集无效
        uint16_t bIsCommunicationError : 1;     // 通信错误
        uint16_t bIsCurRopeLenInvalid : 1;      // 当前绳长无效
        uint16_t bIsTerrainInfoInvalid   : 1;   // 地形信息无效
    } stBits;
    uint16_t nData;
} SKA_ComGenErrorBits;

// 通用警告位
typedef union {
    struct {
        uint16_t bIsCommCycleUnstable : 1;          // 通信周期不稳定
        uint16_t bResetInExecution : 1;             // 任务执行中复位
    } stBits;
    uint16_t nData;
} SKA_ComGenWarnBits;

// 通用输出
typedef struct {
    uint16_t nAscHeartbeat;             // 防摇控制器心跳
    SKA_ComGenStateBits stStateBits;    // 通用状态位
    SKA_ComGenErrorBits stErrorBits;    // 通用错误位
    SKA_ComGenWarnBits stWarnBits;      // 通用警告位
} SKA_ComGeneralOutput;

// 运行机构状态位
typedef union {
    struct {
        uint16_t bIsEnabled : 1;            // 控制器使能反馈
        uint16_t bIsPositive : 1;           // 正向运动
        uint16_t bIsNegative : 1;           // 反向运动
        uint16_t bIsBusy : 1;               // 忙碌
        uint16_t bOpenBrake : 1;            // 打开抱闸
        uint16_t bIsZeroManual : 1;         // 零输入频率检测反馈
        uint16_t bIsPosErrOk : 1;           // 位置误差满足完成要求
        uint16_t bIsZeroSpeed : 1;          // 零运行速度检测反馈
        uint16_t bIsSwayAngleOk : 1;        // 负载摆动满足完成要求
        uint16_t nOperationMode : 2;        // 工作模式反馈
    } stBits;
    uint16_t nData;
} SKA_ComTravelStateBits;

// 运行机构错误位
typedef union {
    struct {
        uint16_t bIsWorkModeInvalid : 1;        // 工作模式无效
        uint16_t bIsCurPosInvalid : 1;          // 当前位置无效
        uint16_t bIsTargetPosInvalid : 1;       // 目标位置无效
        uint16_t bIsTrackErrTooLarge : 1;       // 跟踪误差过大
        uint16_t bIsEffRopeLenInvalid : 1;      // 有效绳长无效
        uint16_t bIsBrakeOpenTimeTooLong : 1;   // 开闸时间太长
        uint16_t bCannotDriveByLowFreq : 1;     // 无法驱动
    } stBits;
    uint16_t nData;
} SKA_ComTravelErrorBits;

// 运行机构警告位
typedef union {
    struct {
        uint16_t bIsSwayCtrlDeactivated : 1;        // 未激活防摇控制
        uint16_t bIsPosCtrlDeactivated : 1;         // 未激活定位控制
        uint16_t bResetInExecution : 1;             // 任务执行中复位
        uint16_t bQuickStopInExecution : 1;         // 任务执行中无防摇急停
        uint16_t bAntiswayStopInExecution : 1;      // 任务执行中带防摇缓停
        uint16_t bPrelimitlInExecution : 1;         // 任务执行中预限位减速
        uint16_t bChangeTargetPosInExecution : 1;   // 任务执行中更新目标位置
    } stBits;
    uint16_t nData;
} SKA_ComTravelWarnBits;

// 运行机构输出
typedef struct {
    SKA_ComTravelStateBits stStateBits;     // 状态位
    SKA_ComTravelErrorBits stErrorBits;     // 错误位
    SKA_ComTravelWarnBits stWarnBits;       // 警告位
    int16_t nOutputFreq;                    // 输出频率(0.01Hz)
    int16_t nOutputSpeed;                   // 输出速度(mm/s)
    uint16_t nBrakeDist;                  // 带防摇缓停的制动距离(mm)
    int32_t nEffRopeLen;                    // 有效绳长(mm)
    int32_t nTargetPos;                     // 目标位置(mm)
    int32_t nOriTrackErr;                   // 原始跟踪误差(mm)
    int32_t nFilTrackErr;                   // 滤波后跟踪误差(mm)
} SKA_ComTravelOutput;

// 起升状态位
typedef union {
    struct {
        uint16_t bIsEnabled : 1;                // 控制器使能反馈
        uint16_t bIsPositive : 1;               // 正向运动
        uint16_t bIsNegative : 1;               // 反向运动
        uint16_t bIsBusy : 1;                   // 忙碌
        uint16_t bIsZeroManual : 1;             // 零输入频率检测反馈
        uint16_t bIsPosErrOk : 1;               // 位置误差满足完成要求
        uint16_t bIsZeroSpeed : 1;              // 零运行速度检测反馈
        uint16_t nOperationMode : 1;            // 工作模式反馈
    } stBits;
    uint16_t nData;
} SKA_ComHoistStateBits;

// 起升错误位
typedef union {
    struct {
        uint16_t bIsWorkModeInvalid : 1;            // 工作模式无效
        uint16_t bIsTargetRopeLenInvalid : 1;       // 目标绳长无效
        uint16_t bIsTrackErrTooLarge : 1;           // 跟踪误差过大
        uint16_t bIsBrakeOpenTimeTooLong : 1;       // 开闸时间太长
        uint16_t bCannotDriveByLowFreq : 1;         // 无法驱动
    } stBits;
    uint16_t nData;
} SKA_ComHoistErrorBits;

// 起升警告位
typedef union {
    struct {
        uint16_t bIsPosCtrlDeactivated : 1;             // 未激活定位控制
        uint16_t bResetInExecution : 1;                 // 任务执行中复位
        uint16_t bQuickStopInExecution : 1;             // 任务执行中急停
        uint16_t bPrelimitlInExecution : 1;             // 任务执行中预限位减速
        uint16_t bChangeTargetRopeLenInExecution : 1;   // 任务执行中更新目标绳长
    } stBits;
    uint16_t nData;
} SKA_ComHoistWarnBits;

// 起升输出
typedef struct {
    SKA_ComHoistStateBits stStateBits;      // 状态位
    SKA_ComHoistErrorBits stErrorBits;      // 错误位
    SKA_ComHoistWarnBits stWarnBits;        // 警告位
    int16_t nOutputFreq;                    // 输出频率(0.01Hz)
    int16_t nOutputSpeed;                   // 输出速度(mm/s)
    int32_t nTargetRopeLength;              // 目标绳长(mm)
    int32_t nOriTrackErr;                   // 原始跟踪误差(mm)
    int32_t nFilTrackErr;                   // 滤波后跟踪误差(mm)
} SKA_ComHoistOutput;

// 防摇控制器输出
typedef volatile struct {
    SKA_ComPacketHeader stSendPacketHeader;     // 发送数据包头
    SKA_ComGeneralOutput stGeneralOutput;       // 通用
    SKA_ComTravelOutput stBridgeOutput;         // 大车
    SKA_ComTravelOutput stTrolleyOutput;        // 小车
    SKA_ComHoistOutput stHoistOutput;           // 起升
} SKA_ComAntiswayControllerOutput;

// 通用参数
typedef struct {
    uint16_t nSamplePeriod;             // 采样周期(ms)
    int32_t nMinEffRopeLength;          // 最小有效绳长(mm)
    int32_t nMaxEffRopeLength;          // 最大有效绳长(mm)
    int16_t nTerrainInflation;      // 地形高度膨胀因子(mm)
} SKA_ComGeneralParam;

// 运行机构参数
typedef struct {
    int32_t nMinPos;                    // 最小位置(mm)
    int32_t nMaxPos;                    // 最大位置(mm)
    uint16_t nMaxPosErr;                // 最大定位误差(mm)
    uint16_t nStaticSpeed;              // 静止检测的速度阈值(mm/s)
    uint16_t nAsLevel;                  // 防摇等级
    uint16_t nMaxOutFreq;               // 最大输出频率(0.01Hz)
    uint16_t nMinMoveFreq;              // 最小运动频率(0.01Hz)
    uint16_t nMaxOutSpeed;              // 最大输出速度(mm/s)
    uint16_t nMaxRefSpeed;              // 最大参考速度(mm/s)
    uint16_t nMaxOutAcc;                // 最大输出加速度(mm/s^2)
    uint16_t nMaxRefAcc;                // 最大参考加速度(mm/s^2)
    uint16_t nMinAssistSpeed;           // 辅助防摇启动阈值(mm/s)
    uint16_t nPrelimitSpeed;            // 预限位速度(1%)
    uint16_t nDelayTime;                // 系统延迟时间(ms)
    uint16_t nDriveOnDelay;             // 驱动器启动延迟(ms)
    uint16_t nMaxTrackErr;              // 最大跟踪误差(mm)
    uint16_t nTrackErrLim;              // 跟踪偏差的限幅滤波(mm)
    uint16_t nTrackErrRateLim;          // 跟踪偏差的限速滤波(mm/s)
    uint16_t nTrackErrFilTc;            // 跟踪偏差的惯性滤波(ms)
    uint16_t nPosTrackKp;               // 位置跟踪控制器的比例系数Kp(0.001)
    uint16_t nPosTrackKi;               // 位置跟踪控制器的比例系数Ki(0.001)
    uint16_t nMaxResSway;               // 最大残余摆动(mm)
    uint16_t nAntiswayKa;               // 闭环防摇控制器的增益系数(0.001)
} SKA_ComTravelParam;

// 起升参数
typedef struct {
    int32_t nMinRopeLen;                // 最小绳长(mm)
    int32_t nMaxRopeLen;                // 最大绳长(mm)
    uint16_t nMaxPosErr;                // 最大定位误差(mm)
    uint16_t nStaticSpeed;              // 静止检测的速度阈值(mm/s)
    uint16_t nMaxOutFreq;               // 最大输出频率(0.01Hz)
    uint16_t nMinMoveFreq;              // 最小运动频率(0.01Hz)
    uint16_t nMaxOutSpeed;              // 最大输出速度(mm/s)
    uint16_t nMaxRefSpeed;              // 最大参考速度(mm/s)
    uint16_t nMaxOutAcc;                // 最大输出加速度(mm/s^2)
    uint16_t nMaxRefAcc;                // 最大参考加速度(mm/s^2)
    uint16_t nPrelimitSpeed;            // 预限位速度(100%)
    uint16_t nDelayTime;                // 系统延迟时间(ms)
    uint16_t nMaxTrackErr;              // 最大跟踪误差(mm)
    uint16_t nTrackErrLim;              // 跟踪偏差的限幅滤波(mm)
    uint16_t nTrackErrRateLim;          // 跟踪偏差的限速滤波(mm/s)
    uint16_t nTrackErrFilTc;            // 跟踪偏差的惯性滤波(ms)
    uint16_t nPosTrackKp;               // 位置跟踪控制器的比例系数Kp(0.001)
    uint16_t nPosTrackKi;               // 位置跟踪控制器的比例系数Ki(0.001)
} SKA_ComHoistParam;

// 防摇控制器参数
typedef volatile struct {
    SKA_ComGeneralParam stGeneralParam;     // 通用
    SKA_ComTravelParam stBridgeParam;       // 大车
    SKA_ComTravelParam stTrolleyParam;      // 小车
    SKA_ComHoistParam stHoistParam;         // 起升
} SKA_ComAntiswayControllerParam;

#pragma pack(pop) // 恢复默认对齐方式

// 从PLC接收的数据
extern SKA_ComAntiswayControllerInput stComAscInput;

// 向PLC发送的数据
extern SKA_ComAntiswayControllerOutput stComAscOutput;

// 防摇参数配置
extern SKA_ComAntiswayControllerParam stComAscParam;

// 接收数据故障
extern bool bIsRecvDataFault;

// 发送数据故障
extern bool bIsSendDataFault;

/**
 * @brief 获取参数最小值
 * @param pComParam 存放结果的地址
 * @return 错误码 {0: 正常}
 */
int8_t SKA_ComInt_getMinValue(SKA_ComAntiswayControllerParam *pComParam);

/**
 * @brief 获取参数最大值
 * @param pComParam 存放结果的地址
 * @return 错误码 {0: 正常}
 */
int8_t SKA_ComInt_getMaxValue(SKA_ComAntiswayControllerParam *pComParam);

/**
 * @brief 获取参数默认值
 * @param pComParam 存放结果的地址
 * @return 错误码 {0: 正常}
 */
int8_t SKA_ComInt_getDefaultValue(SKA_ComAntiswayControllerParam *pComParam);

/**
 * @brief 初始化输入
 * @param pComInput 输入的地址
 * @return 错误码 {0: 正常}
 */
int8_t SKA_ComInt_InitInput(SKA_ComAntiswayControllerInput *pComInput);

/**
 * @brief 初始化参数
 * @param pComParam 参数的地址
 * @return 错误码 {0: 正常}
 */
int8_t SKA_ComInt_InitParam(SKA_ComAntiswayControllerParam *pComParam);

/**
 * @brief 初始化输出
 * @param pComOutput 输出的地址
 * @return 错误码 {0: 正常}
 */
int8_t SKA_ComInt_InitOutput(SKA_ComAntiswayControllerOutput *pComOutput);

/**
 * @brief 显示输入
 * @param pComAscInput 输入的地址
 * @return 错误码 {0: 正常}
 */
int8_t SKA_ComInt_ShowInput(const SKA_ComAntiswayControllerInput *pComAscInput);

/**
 * @brief 显示输出
 * @param pComAscOutput 输出的地址
 * @return 错误码 {0: 正常}
 */
int8_t SKA_ComInt_ShowOutput(const SKA_ComAntiswayControllerOutput *pComAscOutput);

/**
 * @brief 将输入数据的字节顺序由网络字节顺序转换为主机字节顺序
 * @param pComAscInputN 网络字节顺序的输入数据的地址
 * @param pComAscInputH 主机字节顺序的输入数据的地址
 * @return 错误码 {0: 正常}
 */
int8_t SKA_ComInt_InputN2H(const SKA_ComAntiswayControllerInput *pComAscInputN, SKA_ComAntiswayControllerInput *pComAscInputH);


/**
 * @brief 将输出数据的字节顺序由主机字节顺序转换为网络字节顺序
 * @param pComAscOutputH 主机字节顺序的输出数据的地址
 * @param pComAscOutputN 网络字节顺序的输出数据的地址
 * @return 错误码 {0: 正常}
 */
int8_t SKA_ComInt_OutputH2N(const SKA_ComAntiswayControllerOutput *pComAscOutputH, SKA_ComAntiswayControllerOutput *pComAscOutputN);

/**
 * @brief 将寄存器缓冲区反序列化为输入数据
 * @param pRegisterBuffer 寄存器缓冲区的地址
 * @param nRegNum 寄存器数量
 * @param pComAscInputH 主机字节顺序的输入数据的地址
 * @return 错误码 {0: 正常}
 */
int8_t SKA_ComInt_InputDeserialize(const uint16_t *pRegisterBuffer, int32_t nRegNum, SKA_ComAntiswayControllerInput *pComAscInputH);

/**
 * @brief 将输出数据序列化为寄存器缓冲区
 * @param pComOutputH 主机字节顺序的输出数据的地址
 * @param pRegisterBuffer 寄存器缓冲区的地址
 * @param nRegNum 寄存器数量
 * @return 错误码 {0: 正常}
 */
int8_t SKA_ComInt_OutputSerialize(const SKA_ComAntiswayControllerOutput *pComOutputH, uint16_t *pRegisterBuffer, int32_t nRegNum);

/**
 * @brief 创建参数数据的JSON对象
 * @param pComParam 参数的地址
 * @param ppJson JSON对象指针的地址
 * @return 错误码 {0: 正常}
 */
int8_t SKA_ComInt_CreateParamJson(const SKA_ComAntiswayControllerParam *pComParam, cJSON **ppJson);

/**
 * @brief 从JSON对象解析参数数据
 * @param pJson JSON对象
 * @param pComParam 参数的地址
 * @return 错误码 {0: 正常}
 */
int8_t SKA_ComInt_ParseParamFromJson(const cJSON *pJson, SKA_ComAntiswayControllerParam *pComParam);

#endif //SKA2000_OHBC_CONTROLLER_SKA_COMMUNICATIONINTERFACE_H