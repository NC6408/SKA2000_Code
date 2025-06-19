/**
  ******************************************************************************
  * @file       : SKA_AntiswayController.c
  * @brief      : 防摇控制器
  * @author     : ZhangYi
  * @version    : None
  * @date       : 2024/10/24
  ******************************************************************************
  */
//

#include <stddef.h> 
#include <math.h>
#include <stdio.h>

#include "SKA_AntiswayController.h"
#include "SKA_Utils.h"

/**
 * @brief 初始化运行机构单轴运动任务
 * @param pTSMT 运行机构单轴运动任务的地址
 * @param pTaskName 任务名称
 * @return 错误码 {0: 正常}
 */
static int8_t SKA_ASC_InitTravelSingleMoveTask(SKA_TravelSingleMoveTask *pTSMT, char* pTaskName);

/**
 * @brief 初始化起升机构单轴运动任务
 * @param pHSMT 起升机构单轴运动任务的地址
 * @param pTaskName 任务名称
 * @return 错误码 {0: 正常}
 */
static int8_t SKA_ASC_InitHoistSingleMoveTask(SKA_HoistSingleMoveTask *pHSMT, char* pTaskName);

/**
 * @brief 初始化横向避障任务
 * @param pHTT 横向避障任务的地址
 * @param pTaskName 任务名称
 * @return 错误码 {0: 正常}
 */
static int8_t SKA_ASC_InitHorizontalTask(SKA_HorizontalAvoidanceTask *pHTT, char* pTaskName);

/**
 * @brief 初始化纵向避障任务
 * @param pVTT 纵向避障任务的地址
 * @param pTaskName 任务名称
 * @return 错误码 {0: 正常}
 */
static int8_t SKA_ASC_InitVerticalTask(SKA_VerticalAvoidanceTask *pVTT, char* pTaskName);

/**
 * @brief 判断防摇控制器输入的合法性
 * @param pASC 防摇控制器地址
 * @return 错误码 {0: 正常}
 */
static int8_t SKA_ASC_IsInputValid(SKA_AntiswayController *pASC);

/**
 * @brief 防摇控制器的输入指令解析
 * @param pASC 防摇控制器
 * @return 错误码 {0: 正常}
 */
static int8_t SKA_ASC_InputCmdParse(SKA_AntiswayController *pASC);

/**
 * @brief 防摇控制器的使用中参数更新
 * @param pASC 防摇控制器
 * @return 错误码 {0: 正常}
 */
static int8_t SKA_ASC_UpdateUsingParam(SKA_AntiswayController *pASC);

/**
 * @brief 防摇控制器的任务模式切换
 * @param pASC 防摇控制器
 * @return 错误码 {0: 正常}
 */
static int8_t SKA_ASC_TaskModeTransition(SKA_AntiswayController *pASC);

/**
 * @brief 防摇控制器的任务状态切换
 * @param pASC 防摇控制器
 * @return 错误码 {0: 正常}
 */
static int8_t SKA_ASC_TaskStateTransition(SKA_AntiswayController *pASC);

/**
 * @brief 运行机构单轴运动任务状态切换
 * @param dCtrlPeriod 控制周期
 * @param pTask 运行机构单轴运动任务指针
 * @param pInput 运行机构输入指针
 * @param pParam 运行机构参数指针
 * @param pGeneralOutput 通用输出指针
 * @param pOutput 运行机构输出指针
 * @param pCmdParser 输入指令解析器指针
 * @return 错误码 {0: 正常}
 */
static int8_t SKA_ASC_TravelSingleMoveTaskStateTransition(double dCtrlPeriod,
                                                          SKA_TravelSingleMoveTask *pTask, 
                                                          SKA_AscTravelInput *pInput,
                                                          SKA_AscTravelParam *pParam, 
                                                          SKA_AscGeneralOutput *pGeneralOutput,
                                                          SKA_AscTravelOutput *pOutput,
                                                          SKA_AscTravelInputCmdParser *pCmdParser);

/**
 * @brief 起升机构单轴运动任务状态切换
 * @param dCtrlPeriod 控制周期
 * @param pTask 起升机构单轴运动任务指针
 * @param pGeneralInput 通用输入指针
 * @param pInput 起升机构输入指针
 * @param pParam 起升机构参数指针
 * @param pGeneralOutput 通用输出指针
 * @param pOutput 起升机构输出指针
 * @param pCmdParser 输入指令解析器指针
 * @return 错误码 {0: 正常}
 */
static int8_t SKA_ASC_HoistSingleMoveTaskStateTransition(double dCtrlPeriod,
                                                         SKA_HoistSingleMoveTask *pTask, 
                                                         SKA_AscGeneralInput *pGeneralInput,
                                                         SKA_AscHoistInput  *pInput,
                                                         SKA_AscHoistParam *pParam, 
                                                         SKA_AscGeneralOutput *pGeneralOutput,
                                                         SKA_AscHoistOutput *pOutput,
                                                         SKA_AscHoistInputCmdParser *pCmdParser);

int8_t SKA_ASC_Init(SKA_AntiswayController *pASC, double dCtrlPeriod) {
    /******************** 函数参数合法性检验 ********************/
    if (pASC == NULL || dCtrlPeriod <= 0) {
        printf("Invalid paramerters of SKA_ASC_Init().\n");
        return -1;
    }

    /******************** 初始化 ********************/

    // 控制周期
    pASC->dCtrlPeriod = dCtrlPeriod;

    // 通用输入指令解析器
    pASC->stGeneralICP.bStartVerticalTask = 0;
    SKA_RED_Init(&(pASC->stGeneralICP.stStartVerticalTaskRED));
    pASC->stGeneralICP.bStartHorizontalTask = 0;
    SKA_RED_Init(&(pASC->stGeneralICP.stStartHorizontalTaskRED));

    // 大车输入指令解析器
    pASC->stBridgeICP.bStart = 0;
    SKA_RED_Init(&(pASC->stBridgeICP.stStartRED));
    pASC->stBridgeICP.bQuickStop = 0;
    SKA_RED_Init(&(pASC->stBridgeICP.stQuickStopRED));
    pASC->stBridgeICP.bAscStop = 0;
    SKA_RED_Init(&(pASC->stBridgeICP.stAscStopRED));

    // 小车输入指令解析器
    pASC->stTrolleyICP.bStart = 0;
    SKA_RED_Init(&(pASC->stTrolleyICP.stStartRED));
    pASC->stTrolleyICP.bQuickStop = 0;
    SKA_RED_Init(&(pASC->stTrolleyICP.stQuickStopRED));
    pASC->stTrolleyICP.bAscStop = 0;
    SKA_RED_Init(&(pASC->stTrolleyICP.stAscStopRED));

    // 起升机构输入指令解析器
    pASC->stHoistICP.bStart = 0;
    SKA_RED_Init(&(pASC->stHoistICP.stStartRED));
    pASC->stHoistICP.bQuickStop = 0;
    SKA_RED_Init(&(pASC->stHoistICP.stQuickStopRED));

    // 任务模式
    pASC->eTaskMode = SKA_ASC_SINGLE_MOVE_TASK_MODE;

    // 大车单轴运动任务
    SKA_ASC_InitTravelSingleMoveTask(&(pASC->stBridgeTask), "BridgeSingleTask");

    // 小车单轴运动任务
    SKA_ASC_InitTravelSingleMoveTask(&(pASC->stTrolleyTask), "TrolleySingleTask");

    // 起升单轴运动任务
    SKA_ASC_InitHoistSingleMoveTask(&(pASC->stHoistTask), "HoistSingleTask");

    // 横向避障任务
    SKA_ASC_InitHorizontalTask(&(pASC->stHorizontalTask), "HorizontalTask");

    // 纵向避障任务
    SKA_ASC_InitVerticalTask(&(pASC->stVerticalTask), "VerticalTask");

    return 0;
}

int8_t SKA_ASC_SetInput(SKA_AntiswayControllerInput *pAscInput, SKA_ComAntiswayControllerInput *pComInput) {
    /******************** 函数参数合法性检验 ********************/
    if (pAscInput == NULL || pComInput == NULL) {
        printf("Invalid paramerters of SKA_ASC_SetInput().\n");
        return -1;
    }

    /******************** 设置输入 ********************/

    /* 通用 */
    pAscInput->stGeneralInput.stCtrlBits.stBits.bActiveVerticalMode = pComInput->stGeneralInput.stCtrlBits.stBits.bActiveVerticalMode;
    pAscInput->stGeneralInput.stCtrlBits.stBits.bStartVerticalTask = pComInput->stGeneralInput.stCtrlBits.stBits.bStartVerticalTask;
    pAscInput->stGeneralInput.stCtrlBits.stBits.bIsMaintainMinRopeLength = pComInput->stGeneralInput.stCtrlBits.stBits.bIsMaintainMinRopeLength;
    pAscInput->stGeneralInput.stCtrlBits.stBits.bActiveHorizontalMode = pComInput->stGeneralInput.stCtrlBits.stBits.bActiveHorizontalMode;
    pAscInput->stGeneralInput.stCtrlBits.stBits.bStartHorizontalTask = pComInput->stGeneralInput.stCtrlBits.stBits.bStartHorizontalTask;
    pAscInput->stGeneralInput.stCtrlBits.stBits.bIsUseWaypoints = pComInput->stGeneralInput.stCtrlBits.stBits.bIsUseWaypoints;
    pAscInput->stGeneralInput.stCtrlBits.stBits.bReset = pComInput->stGeneralInput.stCtrlBits.stBits.bReset;
    pAscInput->stGeneralInput.stCtrlBits.stBits.bReboot = pComInput->stGeneralInput.stCtrlBits.stBits.bReboot;

    /* 当前绳长 */
    pAscInput->stGeneralInput.dCurRopeLength = pComInput->stGeneralInput.nCurRopeLength / 1000.0;
    
   /* 大车 */

    int32_t nCurRopeLength = pComInput->stGeneralInput.nCurRopeLength;
    int32_t nRopeOffset = pComInput->stBridgeInput.nRopeOffset;
    pAscInput->stBridgeInput.dEffRopeLength = (nCurRopeLength + nRopeOffset) / 1000.0;
    pAscInput->stBridgeInput.stCtrlBits.nData = pComInput->stBridgeInput.stCtrlBits.nData;
    pAscInput->stBridgeInput.dCurPos = pComInput->stBridgeInput.nCurPos / 1000.0;
    pAscInput->stBridgeInput.dTargetPos = pComInput->stBridgeInput.nTargetPos / 1000.0;
    pAscInput->stBridgeInput.dSpeedLimit = pComInput->stBridgeInput.nSpeedLimit / 1000.0;
    pAscInput->stBridgeInput.dManualFreq = pComInput->stBridgeInput.nManualFreq / 100.0;

    /* 小车 */

    nCurRopeLength = pComInput->stGeneralInput.nCurRopeLength;
    nRopeOffset = pComInput->stTrolleyInput.nRopeOffset;
    pAscInput->stTrolleyInput.dEffRopeLength = (nCurRopeLength + nRopeOffset) / 1000.0;
    pAscInput->stTrolleyInput.stCtrlBits.nData = pComInput->stTrolleyInput.stCtrlBits.nData;
    pAscInput->stTrolleyInput.dCurPos = pComInput->stTrolleyInput.nCurPos / 1000.0;
    pAscInput->stTrolleyInput.dTargetPos = pComInput->stTrolleyInput.nTargetPos / 1000.0;
    pAscInput->stTrolleyInput.dSpeedLimit = pComInput->stTrolleyInput.nSpeedLimit / 1000.0;
    pAscInput->stTrolleyInput.dManualFreq = pComInput->stTrolleyInput.nManualFreq / 100.0;

    /* 起升 */

    pAscInput->stHoistInput.stCtrlBits.nData = pComInput->stHoistInput.stCtrlBits.nData;
    pAscInput->stHoistInput.dTargetRopeLength = pComInput->stHoistInput.nTargetRopeLength / 1000.0;
    pAscInput->stHoistInput.dSpeedLimit = pComInput->stHoistInput.nSpeedLimit / 1000.0;
    pAscInput->stHoistInput.dManualFreq = pComInput->stHoistInput.nManualFreq / 100.0;

    /* 相机 */

    pAscInput->stCameraInput.stStateBits.nData = pComInput->stCameraInput.stStateBits.nData;
    pAscInput->stCameraInput.stErrorBits.nData = pComInput->stCameraInput.stErrorBits.nData;
    pAscInput->stCameraInput.stWarnBits.nData = pComInput->stCameraInput.stWarnBits.nData;
    pAscInput->stCameraInput.dSwayPositionX = pComInput->stCameraInput.nSwayPositionX / 1000.0;
    pAscInput->stCameraInput.dSwayVelocityX = pComInput->stCameraInput.nSwayVelocityX / 1000.0;
    pAscInput->stCameraInput.dSwayPositionY = pComInput->stCameraInput.nSwayPositionY / 1000.0;
    pAscInput->stCameraInput.dSwayVelocityY = pComInput->stCameraInput.nSwayVelocityY / 1000.0;
    pAscInput->stCameraInput.dRotationAngle = pComInput->stCameraInput.nRotationAngle / 100.0;
    pAscInput->stCameraInput.dRotationVelocity = pComInput->stCameraInput.nRotationVelocity / 100.0;

    /* 纵向避障 */

    pAscInput->stVerticalInput.dTotalHeight = pComInput->stVerticalInput.nTotalHeight / 1000.0;
    for (int i = 0; i < 10; i++) {
        pAscInput->stVerticalInput.arrLiftTerrain[i].dDistance = pComInput->stVerticalInput.arrLiftTerrain[i].nDistance / 100.0;
        pAscInput->stVerticalInput.arrLiftTerrain[i].dHeight = pComInput->stVerticalInput.arrLiftTerrain[i].nHeight / 100.0;
        pAscInput->stVerticalInput.arrDropTerrain[i].dDistance = pComInput->stVerticalInput.arrDropTerrain[i].nDistance / 100.0;
        pAscInput->stVerticalInput.arrDropTerrain[i].dHeight = pComInput->stVerticalInput.arrDropTerrain[i].nHeight / 100.0;
    }
    pAscInput->stVerticalInput.nLiftArraySize = pComInput->stVerticalInput.nLiftArraySize;
    pAscInput->stVerticalInput.nDropArraySize = pComInput->stVerticalInput.nDropArraySize;

     /* 横向避障 */ 

    pAscInput->stHorizontalInput.nObstacleNum = pComInput->stHorizontalInput.nObstacleNum;
    pAscInput->stHorizontalInput.nWaypointsNum = pComInput->stHorizontalInput.nWaypointsNum;
    pAscInput->stHorizontalInput.nObstacleInflation = pComInput->stHorizontalInput.nObstacleInflation / 1000.0;
    for (int i = 0; i < 20; i++)
    {
        pAscInput->stHorizontalInput.arrWaypoints[i].dWaypointsCoordX = pComInput->stHorizontalInput.arrWaypoints[i].nWaypointsCoordX / 1000.0;
        pAscInput->stHorizontalInput.arrWaypoints[i].dWaypointsCoordY = pComInput->stHorizontalInput.arrWaypoints[i].nWaypointsCoordY / 1000.0;
    }
    for (int i = 0; i < 5; i++)
    {
        pAscInput->stHorizontalInput.arrObstacle[i].dObstacleCoordX = pComInput->stHorizontalInput.arrObstacle[i].nObstacleCoordX / 1000.0;
        pAscInput->stHorizontalInput.arrObstacle[i].dObstacleCoordY = pComInput->stHorizontalInput.arrObstacle[i].nObstacleCoordY / 1000.0;
        pAscInput->stHorizontalInput.arrObstacle[i].dObstacleCoordW = pComInput->stHorizontalInput.arrObstacle[i].nObstacleCoordW / 1000.0;
        pAscInput->stHorizontalInput.arrObstacle[i].dObstacleCoordH = pComInput->stHorizontalInput.arrObstacle[i].nObstacleCoordH / 1000.0;
    }

    return 0;
}

int8_t SKA_ASC_SetParam(SKA_AntiswayControllerParam *pAscParam, SKA_ComAntiswayControllerParam *pComParam) {
    /******************** 函数参数合法性检验 ********************/
    if (pAscParam == NULL || pComParam == NULL) {
        printf("Invalid paramerters of SKA_ASC_SetParam().\n");
        return -1;
    }

    /******************** 设置参数 ********************/

    /* 通用 */

    pAscParam->stGeneralParam.dMinEffRopeLength = pComParam->stGeneralParam.nMinEffRopeLength / 1000.0;
    pAscParam->stGeneralParam.dMaxEffRopeLength = pComParam->stGeneralParam.nMaxEffRopeLength / 1000.0;
    pAscParam->stGeneralParam.dTerrainInflation = pComParam->stGeneralParam.nTerrainInflation / 1000.0;

    /* 大车 */

    pAscParam->stBridgeParam.dMinPos = pComParam->stBridgeParam.nMinPos / 1000.0;
    pAscParam->stBridgeParam.dMaxPos = pComParam->stBridgeParam.nMaxPos / 1000.0;
    pAscParam->stBridgeParam.dMaxPosErr = pComParam->stBridgeParam.nMaxPosErr / 1000.0;
    pAscParam->stBridgeParam.dStaticSpeed = pComParam->stBridgeParam.nStaticSpeed / 1000.0;
    pAscParam->stBridgeParam.nAsLevel = pComParam->stBridgeParam.nAsLevel;
    pAscParam->stBridgeParam.dMaxOutFreq = pComParam->stBridgeParam.nMaxOutFreq / 100.0;
    pAscParam->stBridgeParam.dMinMoveFreq = pComParam->stBridgeParam.nMinMoveFreq / 100.0;
    pAscParam->stBridgeParam.dMaxOutSpeed = pComParam->stBridgeParam.nMaxOutSpeed / 1000.0;
    pAscParam->stBridgeParam.dMaxRefSpeed = pComParam->stBridgeParam.nMaxRefSpeed / 1000.0;
    pAscParam->stBridgeParam.dMaxOutAcc = pComParam->stBridgeParam.nMaxOutAcc / 1000.0;
    pAscParam->stBridgeParam.dMaxRefAcc = pComParam->stBridgeParam.nMaxRefAcc / 1000.0;
    pAscParam->stBridgeParam.dMinAssistSpeed = pComParam->stBridgeParam.nMinAssistSpeed / 1000.0;
    pAscParam->stBridgeParam.dPrelimitSpeed = (pComParam->stBridgeParam.nPrelimitSpeed / 100.0) * pAscParam->stBridgeParam.dMaxOutSpeed;
    pAscParam->stBridgeParam.dDelayTime = pComParam->stBridgeParam.nDelayTime / 1000.0;
    pAscParam->stBridgeParam.dDriveOnDelay = pComParam->stBridgeParam.nDriveOnDelay / 1000.0;
    pAscParam->stBridgeParam.dMaxTrackErr = pComParam->stBridgeParam.nMaxTrackErr / 1000.0;
    pAscParam->stBridgeParam.dTrackErrLim = pComParam->stBridgeParam.nTrackErrLim / 1000.0;
    pAscParam->stBridgeParam.dTrackErrRateLim = pComParam->stBridgeParam.nTrackErrRateLim / 1000.0;
    pAscParam->stBridgeParam.dTrackErrFilTc = pComParam->stBridgeParam.nTrackErrFilTc / 1000.0;
    pAscParam->stBridgeParam.dPosTrackKp = pComParam->stBridgeParam.nPosTrackKp / 1000.0;
    pAscParam->stBridgeParam.dPosTrackKi = pComParam->stBridgeParam.nPosTrackKi / 1000.0;
    pAscParam->stBridgeParam.dMaxResSway = pComParam->stBridgeParam.nMaxResSway / 1000.0;
    pAscParam->stBridgeParam.dAntiswayKa = pComParam->stBridgeParam.nAntiswayKa / 1000.0;

    /* 小车 */

    pAscParam->stTrolleyParam.dMinPos = pComParam->stTrolleyParam.nMinPos / 1000.0;
    pAscParam->stTrolleyParam.dMaxPos = pComParam->stTrolleyParam.nMaxPos / 1000.0;
    pAscParam->stTrolleyParam.dMaxPosErr = pComParam->stTrolleyParam.nMaxPosErr / 1000.0;
    pAscParam->stTrolleyParam.dStaticSpeed = pComParam->stTrolleyParam.nStaticSpeed / 1000.0;
    pAscParam->stTrolleyParam.nAsLevel = pComParam->stTrolleyParam.nAsLevel;
    pAscParam->stTrolleyParam.dMaxOutFreq = pComParam->stTrolleyParam.nMaxOutFreq / 100.0;
    pAscParam->stTrolleyParam.dMinMoveFreq = pComParam->stTrolleyParam.nMinMoveFreq / 100.0;
    pAscParam->stTrolleyParam.dMaxOutSpeed = pComParam->stTrolleyParam.nMaxOutSpeed / 1000.0;
    pAscParam->stTrolleyParam.dMaxRefSpeed = pComParam->stTrolleyParam.nMaxRefSpeed / 1000.0;
    pAscParam->stTrolleyParam.dMaxOutAcc = pComParam->stTrolleyParam.nMaxOutAcc / 1000.0;
    pAscParam->stTrolleyParam.dMaxRefAcc = pComParam->stTrolleyParam.nMaxRefAcc / 1000.0;
    pAscParam->stTrolleyParam.dMinAssistSpeed = pComParam->stTrolleyParam.nMinAssistSpeed / 1000.0;
    pAscParam->stTrolleyParam.dPrelimitSpeed = (pComParam->stTrolleyParam.nPrelimitSpeed / 100.0) * pAscParam->stTrolleyParam.dMaxOutSpeed;
    pAscParam->stTrolleyParam.dDelayTime = pComParam->stTrolleyParam.nDelayTime / 1000.0;
    pAscParam->stTrolleyParam.dDriveOnDelay = pComParam->stTrolleyParam.nDriveOnDelay / 1000.0;
    pAscParam->stTrolleyParam.dMaxTrackErr = pComParam->stTrolleyParam.nMaxTrackErr / 1000.0;
    pAscParam->stTrolleyParam.dTrackErrLim = pComParam->stTrolleyParam.nTrackErrLim / 1000.0;
    pAscParam->stTrolleyParam.dTrackErrRateLim = pComParam->stTrolleyParam.nTrackErrRateLim / 1000.0;
    pAscParam->stTrolleyParam.dTrackErrFilTc = pComParam->stTrolleyParam.nTrackErrFilTc / 1000.0;
    pAscParam->stTrolleyParam.dPosTrackKp = pComParam->stTrolleyParam.nPosTrackKp / 1000.0;
    pAscParam->stTrolleyParam.dPosTrackKi = pComParam->stTrolleyParam.nPosTrackKi / 1000.0;
    pAscParam->stTrolleyParam.dMaxResSway = pComParam->stTrolleyParam.nMaxResSway / 1000.0;
    pAscParam->stTrolleyParam.dAntiswayKa = pComParam->stTrolleyParam.nAntiswayKa / 1000.0;

    /* 起升 */

    pAscParam->stHoistParam.dMinRopeLen = pComParam->stHoistParam.nMinRopeLen / 1000.0;
    pAscParam->stHoistParam.dMaxRopeLen = pComParam->stHoistParam.nMaxRopeLen / 1000.0;
    pAscParam->stHoistParam.dMaxPosErr = pComParam->stHoistParam.nMaxPosErr / 1000.0;
    pAscParam->stHoistParam.dStaticSpeed = pComParam->stHoistParam.nStaticSpeed / 1000.0;
    pAscParam->stHoistParam.dMaxOutFreq = pComParam->stHoistParam.nMaxOutFreq / 100.0;
    pAscParam->stHoistParam.dMinMoveFreq = pComParam->stHoistParam.nMinMoveFreq / 100.0;
    pAscParam->stHoistParam.dMaxOutSpeed = pComParam->stHoistParam.nMaxOutSpeed / 1000.0;
    pAscParam->stHoistParam.dMaxRefSpeed = pComParam->stHoistParam.nMaxRefSpeed / 1000.0;
    pAscParam->stHoistParam.dMaxOutAcc = pComParam->stHoistParam.nMaxOutAcc / 1000.0;
    pAscParam->stHoistParam.dMaxRefAcc = pComParam->stHoistParam.nMaxRefAcc / 1000.0;
    pAscParam->stHoistParam.dPrelimitSpeed = (pComParam->stHoistParam.nPrelimitSpeed / 100.0) * pAscParam->stHoistParam.dMaxOutSpeed;
    pAscParam->stHoistParam.dDelayTime = pComParam->stHoistParam.nDelayTime / 1000.0;
    pAscParam->stHoistParam.dMaxTrackErr = pComParam->stHoistParam.nMaxTrackErr / 1000.0;
    pAscParam->stHoistParam.dTrackErrLim = pComParam->stHoistParam.nTrackErrLim / 1000.0;
    pAscParam->stHoistParam.dTrackErrRateLim = pComParam->stHoistParam.nTrackErrRateLim / 1000.0;
    pAscParam->stHoistParam.dTrackErrFilTc = pComParam->stHoistParam.nTrackErrFilTc / 1000.0;
    pAscParam->stHoistParam.dPosTrackKp = pComParam->stHoistParam.nPosTrackKp / 1000.0;
    pAscParam->stHoistParam.dPosTrackKi = pComParam->stHoistParam.nPosTrackKi / 1000.0;

    return 0;
}

int8_t SKA_ASC_GetOutput(SKA_AntiswayControllerOutput *pAscOutput, SKA_ComAntiswayControllerOutput *pComOutput) {
    /******************** 函数参数合法性检验 ********************/
    if (pAscOutput == NULL || pComOutput == NULL) {
        printf("Invalid paramerters of SKA_ASC_GetOutput().\n");
        return -1;
    }

    /******************** 获取输出 ********************/

    /* 通用 */
    pComOutput->stGeneralOutput.stErrorBits.stBits.bIsCurRopeLenInvalid = pAscOutput->stGeneralOutput.stErrorBits.stBits.bIsCurRopeLenInvalid;
    pComOutput->stGeneralOutput.stErrorBits.stBits.bIsTerrainInfoInvalid = pAscOutput->stGeneralOutput.stErrorBits.stBits.bIsTerrainInfoInvalid;
    pComOutput->stGeneralOutput.stWarnBits.stBits.bResetInExecution = pAscOutput->stGeneralOutput.stWarnBits.stBits.bResetInExecution;

    /* 大车 */

    pComOutput->stBridgeOutput.stStateBits.nData = pAscOutput->stBridgeOutput.stStateBits.nData;
    pComOutput->stBridgeOutput.stErrorBits.nData = pAscOutput->stBridgeOutput.stErrorBits.nData;
    pComOutput->stBridgeOutput.stWarnBits.nData = pAscOutput->stBridgeOutput.stWarnBits.nData;
    pComOutput->stBridgeOutput.nOutputFreq = (int16_t)(pAscOutput->stBridgeOutput.dOutputFreq * 100);
    pComOutput->stBridgeOutput.nOutputSpeed = (int16_t)(pAscOutput->stBridgeOutput.dOutputSpeed * 1000);
    pComOutput->stBridgeOutput.nBrakeDist = (uint16_t)(pAscOutput->stBridgeOutput.dBrakeDist * 1000);
    pComOutput->stBridgeOutput.nEffRopeLen = (int32_t)(pAscOutput->stBridgeOutput.dEffRopeLen * 1000);
    pComOutput->stBridgeOutput.nTargetPos = (int32_t)(pAscOutput->stBridgeOutput.dTargetPos * 1000);
    pComOutput->stBridgeOutput.nOriTrackErr = (int32_t)(pAscOutput->stBridgeOutput.dOriTrackErr * 1000);
    pComOutput->stBridgeOutput.nFilTrackErr = (int32_t)(pAscOutput->stBridgeOutput.dFilTrackErr * 1000);

    /* 小车 */

    pComOutput->stTrolleyOutput.stStateBits.nData = pAscOutput->stTrolleyOutput.stStateBits.nData;
    pComOutput->stTrolleyOutput.stErrorBits.nData = pAscOutput->stTrolleyOutput.stErrorBits.nData;
    pComOutput->stTrolleyOutput.stWarnBits.nData = pAscOutput->stTrolleyOutput.stWarnBits.nData;
    pComOutput->stTrolleyOutput.nOutputFreq = (int16_t)(pAscOutput->stTrolleyOutput.dOutputFreq * 100);
    pComOutput->stTrolleyOutput.nOutputSpeed = (int16_t)(pAscOutput->stTrolleyOutput.dOutputSpeed * 1000);
    pComOutput->stTrolleyOutput.nBrakeDist = (uint16_t)(pAscOutput->stTrolleyOutput.dBrakeDist * 1000);
    pComOutput->stTrolleyOutput.nEffRopeLen = (int32_t)(pAscOutput->stTrolleyOutput.dEffRopeLen * 1000);
    pComOutput->stTrolleyOutput.nTargetPos = (int32_t)(pAscOutput->stTrolleyOutput.dTargetPos * 1000);
    pComOutput->stTrolleyOutput.nOriTrackErr = (int32_t)(pAscOutput->stTrolleyOutput.dOriTrackErr * 1000);
    pComOutput->stTrolleyOutput.nFilTrackErr = (int32_t)(pAscOutput->stTrolleyOutput.dFilTrackErr * 1000);

    /* 起升 */

    pComOutput->stHoistOutput.stStateBits.nData = pAscOutput->stHoistOutput.stStateBits.nData;
    pComOutput->stHoistOutput.stErrorBits.nData = pAscOutput->stHoistOutput.stErrorBits.nData;
    pComOutput->stHoistOutput.stWarnBits.nData = pAscOutput->stHoistOutput.stWarnBits.nData;
    pComOutput->stHoistOutput.nOutputFreq = (int16_t)(pAscOutput->stHoistOutput.dOutputFreq * 100);
    pComOutput->stHoistOutput.nOutputSpeed = (int16_t)(pAscOutput->stHoistOutput.dOutputSpeed * 1000);
    pComOutput->stHoistOutput.nTargetRopeLength = (int32_t)(pAscOutput->stHoistOutput.dTargetRopeLength * 1000);
    pComOutput->stHoistOutput.nOriTrackErr = (int32_t)(pAscOutput->stHoistOutput.dOriTrackErr * 1000);
    pComOutput->stHoistOutput.nFilTrackErr = (int32_t)(pAscOutput->stHoistOutput.dFilTrackErr * 1000);

    return 0;
}

int8_t SKA_ASC_Run(SKA_AntiswayController *pASC) {
    /******************** 函数参数合法性检验 ********************/
    if (pASC == NULL) {
        printf("Invalid paramerters of SKA_ASC_Run().\n");
        return -1;
    }

    /******************** 输入合法性检验 ********************/
    SKA_ASC_IsInputValid(pASC);

    /******************** 输入指令解析 ********************/
    SKA_ASC_InputCmdParse(pASC);

    /******************** 更新参数 ********************/
    SKA_ASC_UpdateUsingParam(pASC);

    /******************** 任务模式切换 ********************/
    SKA_ASC_TaskModeTransition(pASC);

    /******************** 任务状态切换 ********************/
    SKA_ASC_TaskStateTransition(pASC);

    return 0;
}

static int8_t SKA_ASC_InitTravelSingleMoveTask(SKA_TravelSingleMoveTask *pTSMT, char* pTaskName) {
    /******************** 函数参数合法性检验 ********************/
    if (pTSMT == NULL) {
        printf("Invalid paramerters of SKA_ASC_InitTravelSingleMoveTask().\n");
        return -1;
    }

    /******************** 初始化 ********************/

    // 名称
    pTSMT->pTaskName = pTaskName;
    // 工作模式
    pTSMT->nOperationMode = 0;
    // 任务状态
    pTSMT->eTaskState = SKA_ASC_FREE_TASK_STATE;
    // 正在无防摇急停标志
    pTSMT->bIsQuickStopping = 0;
    // 正在带防摇缓停标志
    pTSMT->bIsAscStopping = 0;
    // 抱闸已经开启过的标志
    pTSMT->bHasBeenBrakeOpened = 0;

    return 0;
}

static int8_t SKA_ASC_InitHoistSingleMoveTask(SKA_HoistSingleMoveTask *pHSMT, char* pTaskName) {
    /******************** 函数参数合法性检验 ********************/
    if (pHSMT == NULL) {
        printf("Invalid paramerters of SKA_ASC_InitHoistSingleMoveTask().\n");
        return -1;
    }

    /******************** 初始化 ********************/

    // 名称
    pHSMT->pTaskName = pTaskName;
    // 工作模式
    pHSMT->nOperationMode = 0;
    // 任务状态
    pHSMT->eTaskState = SKA_ASC_FREE_TASK_STATE;
    // 正在急停标志
    pHSMT->bIsQuickStopping = 0;

    return 0;
}

static int8_t SKA_ASC_InitHorizontalTask(SKA_HorizontalAvoidanceTask *pHTT, char* pTaskName) {
    /******************** 函数参数合法性检验 ********************/
    if (pHTT == NULL) {
        printf("Invalid paramerters of SKA_ASC_InitHorizontalTask().\n");
        return -1;
    }

    /******************** 初始化 ********************/
    // 任务名称
    pHTT->pTaskName = pTaskName;
    // 工作模式
    pHTT->nOperationMode = 0;
    // 任务状态
    pHTT->eTaskState = SKA_ASC_FREE_TASK_STATE;
    // 抱闸已经开启过的标志
    pHTT->bHasBeenBrakeOpened = 0;

    return 0;
}

static int8_t SKA_ASC_InitVerticalTask(SKA_VerticalAvoidanceTask *pVTT, char* pTaskName) {
    /******************** 函数参数合法性检验 ********************/
    if (pVTT == NULL) {
        printf("Invalid paramerters of SKA_ASC_InitVerticalTask().\n");
        return -1;
    }

    /******************** 初始化 ********************/
     // 任务名称
    pVTT->pTaskName = pTaskName;
    // 工作模式
    pVTT->nOperationMode = 0;
    // 任务状态
    pVTT->eTaskState = SKA_ASC_FREE_TASK_STATE;
    // 抱闸已经开启过的标志
    pVTT->bHasBeenBrakeOpened = 0;

    return 0;
}

static int8_t SKA_ASC_IsInputValid(SKA_AntiswayController *pASC) {
    /******************** 函数参数合法性检验 ********************/
    if (pASC == NULL) {
        printf("Invalid paramerters of SKA_ASC_IsInputValid().\n");
        return -1;
    }

    /******************** 判断输入的合法性 ********************/
    
    // 复位输入错误标志
    pASC->stBridgeTask.bHasInputErrors = 0;
    pASC->stTrolleyTask.bHasInputErrors = 0;
    pASC->stHoistTask.bHasInputErrors = 0;
    pASC->stVerticalTask.bHasInputErrors = 0;
    pASC->stHorizontalTask.bHasInputErrors = 0;

    /* 通用 */

    // 当前绳长
    double dCurRopeLen = pASC->stInput.stGeneralInput.dCurRopeLength;
    double dMinRopeLen = pASC->stParam.stHoistParam.dMinRopeLen;
    double dMaxRopeLen = pASC->stParam.stHoistParam.dMaxRopeLen;
    if (dCurRopeLen < dMinRopeLen || dCurRopeLen > dMaxRopeLen) {
        // 当前绳长无效
        pASC->stBridgeTask.bHasInputErrors = 1;
        pASC->stTrolleyTask.bHasInputErrors = 1;
        pASC->stHoistTask.bHasInputErrors = 1;
        pASC->stVerticalTask.bHasInputErrors = 1;
        pASC->stHorizontalTask.bHasInputErrors = 1;
        pASC->stOutput.stGeneralOutput.stErrorBits.stBits.bIsCurRopeLenInvalid = 1;
     //   printf("Bridge Input CurRopeLen Invalid.\n");
    } else {
        pASC->stOutput.stGeneralOutput.stErrorBits.stBits.bIsCurRopeLenInvalid = 0;
    }

    /* 大车 */
    if (pASC->stInput.stBridgeInput.stCtrlBits.stBits.bEnable == 1) {
        // 有效绳长
        double dEffRopeLen = pASC->stInput.stBridgeInput.dEffRopeLength;
        double dMaxEffRopeLen = pASC->stParam.stGeneralParam.dMaxEffRopeLength;
        double dMinEffRopeLen = pASC->stParam.stGeneralParam.dMinEffRopeLength;
        if (dEffRopeLen < dMinEffRopeLen || dEffRopeLen > dMaxEffRopeLen) {
            // 有效绳长无效
            pASC->stBridgeTask.bHasInputErrors = 1;
            pASC->stVerticalTask.bHasInputErrors = 1;
            pASC->stHorizontalTask.bHasInputErrors = 1;

            pASC->stOutput.stBridgeOutput.stErrorBits.stBits.bIsEffRopeLenInvalid = 1;
            printf("Bridge Input EffRopeLen Invalid.\n");
        } else {
            pASC->stOutput.stBridgeOutput.stErrorBits.stBits.bIsEffRopeLenInvalid = 0;
        }

        // 工作模式
        uint8_t nWorkMode = pASC->stInput.stBridgeInput.stCtrlBits.stBits.nOperationMode;
        if (nWorkMode >= 2) {
            // 工作模式无效
            pASC->stBridgeTask.bHasInputErrors = 1;
            pASC->stOutput.stBridgeOutput.stErrorBits.stBits.bIsWorkModeInvalid = 1;
            printf("Bridge Input WorkMode Invalid.\n");
        } else {
            pASC->stOutput.stBridgeOutput.stErrorBits.stBits.bIsWorkModeInvalid = 0;
        }

        // 当前位置
        double dCurPos = pASC->stInput.stBridgeInput.dCurPos;
        double dMinPos = pASC->stParam.stBridgeParam.dMinPos;
        double dMaxPos = pASC->stParam.stBridgeParam.dMaxPos;
        if (dCurPos < dMinPos || dCurPos > dMaxPos) {
            // 当前位置无效
            pASC->stBridgeTask.bHasInputErrors = 1;
            pASC->stVerticalTask.bHasInputErrors = 1;
            pASC->stHorizontalTask.bHasInputErrors = 1;
            pASC->stOutput.stBridgeOutput.stErrorBits.stBits.bIsCurPosInvalid = 1;
            printf("Bridge Input CurPos Invalid.\n");
        } else {
            pASC->stOutput.stBridgeOutput.stErrorBits.stBits.bIsCurPosInvalid = 0;
        }

        // 目标位置
        double dTargetPos = pASC->stInput.stBridgeInput.dTargetPos;
        double dMinTargetPos = pASC->stParam.stBridgeParam.dMinPos;
        double dMaxTargetPos = pASC->stParam.stBridgeParam.dMaxPos;
        if (dTargetPos < dMinTargetPos || dTargetPos > dMaxTargetPos) {
            // 目标位置无效
            pASC->stBridgeTask.bHasInputErrors = 1;
            pASC->stVerticalTask.bHasInputErrors = 1;
            pASC->stHorizontalTask.bHasInputErrors = 1;
            pASC->stOutput.stBridgeOutput.stErrorBits.stBits.bIsTargetPosInvalid = 1;
            printf("Bridge Input TargetPos Invalid.\n");
        } else {
            pASC->stOutput.stBridgeOutput.stErrorBits.stBits.bIsTargetPosInvalid = 0;
        }
    }

    
    /* 小车 */
    if (pASC->stInput.stTrolleyInput.stCtrlBits.stBits.bEnable == 1) {
        // 有效绳长
        double dEffRopeLen = pASC->stInput.stTrolleyInput.dEffRopeLength;
        double dMaxEffRopeLen = pASC->stParam.stGeneralParam.dMaxEffRopeLength;
        double dMinEffRopeLen = pASC->stParam.stGeneralParam.dMinEffRopeLength;
        if (dEffRopeLen < dMinEffRopeLen || dEffRopeLen > dMaxEffRopeLen) {
            // 有效绳长无效
            pASC->stTrolleyTask.bHasInputErrors = 1;
            pASC->stVerticalTask.bHasInputErrors = 1;
            pASC->stHorizontalTask.bHasInputErrors = 1;
            pASC->stOutput.stTrolleyOutput.stErrorBits.stBits.bIsEffRopeLenInvalid = 1;
            printf("Trolley Input EffRopeLen Invalid.\n");
        } else {
            pASC->stOutput.stTrolleyOutput.stErrorBits.stBits.bIsEffRopeLenInvalid = 0;
        }

        // 工作模式
        uint8_t nWorkMode = pASC->stInput.stTrolleyInput.stCtrlBits.stBits.nOperationMode;
        if (nWorkMode >= 2) {
            // 工作模式无效
            pASC->stTrolleyTask.bHasInputErrors = 1;
            pASC->stOutput.stTrolleyOutput.stErrorBits.stBits.bIsWorkModeInvalid = 1;
            printf("Trolley Input WorkMode Invalid.\n");
        } else {
            pASC->stOutput.stTrolleyOutput.stErrorBits.stBits.bIsWorkModeInvalid = 0;
        }

        // 当前位置
        double dCurPos = pASC->stInput.stTrolleyInput.dCurPos;
        double dMinPos = pASC->stParam.stTrolleyParam.dMinPos;
        double dMaxPos = pASC->stParam.stTrolleyParam.dMaxPos;
        if (dCurPos < dMinPos || dCurPos > dMaxPos) {
            // 当前位置无效
            pASC->stTrolleyTask.bHasInputErrors = 1;
            pASC->stVerticalTask.bHasInputErrors = 1;
            pASC->stHorizontalTask.bHasInputErrors = 1;
            pASC->stOutput.stTrolleyOutput.stErrorBits.stBits.bIsCurPosInvalid = 1;
            printf("Trolley Input CurPos Invalid.\n");
        } else {
            pASC->stOutput.stTrolleyOutput.stErrorBits.stBits.bIsCurPosInvalid = 0;
        }

        // 目标位置
        double dTargetPos = pASC->stInput.stTrolleyInput.dTargetPos;
        double dMinTargetPos = pASC->stParam.stTrolleyParam.dMinPos;
        double dMaxTargetPos = pASC->stParam.stTrolleyParam.dMaxPos;
        if (dTargetPos < dMinTargetPos || dTargetPos > dMaxTargetPos) {
            // 目标位置无效
            pASC->stTrolleyTask.bHasInputErrors = 1;
            pASC->stVerticalTask.bHasInputErrors = 1;
            pASC->stHorizontalTask.bHasInputErrors = 1;
            pASC->stOutput.stTrolleyOutput.stErrorBits.stBits.bIsTargetPosInvalid = 1;
            printf("Trolley Input TargetPos Invalid.\n");
        } else {
            pASC->stOutput.stTrolleyOutput.stErrorBits.stBits.bIsTargetPosInvalid = 0;
        }
    }

    /* 起升 */
    if (pASC->stInput.stHoistInput.stCtrlBits.stBits.bEnable == 1) {
        // 目标绳长
        double dTargetRopeLength = pASC->stInput.stHoistInput.dTargetRopeLength;
        double dMinRopeLength = pASC->stParam.stHoistParam.dMinRopeLen;
        double dMaxRopeLength = pASC->stParam.stHoistParam.dMaxRopeLen;
        if (dTargetRopeLength < dMinRopeLength || dTargetRopeLength > dMaxRopeLength) {
            // 目标绳长无效
            pASC->stHoistTask.bHasInputErrors = 1;
            pASC->stVerticalTask.bHasInputErrors = 1;
            pASC->stOutput.stHoistOutput.stErrorBits.stBits.bIsTargetRopeLenInvalid = 1;
            printf("Hoist Input TargetRopeLen Invalid.\n");
        } else {
            pASC->stOutput.stHoistOutput.stErrorBits.stBits.bIsTargetRopeLenInvalid = 0;
        }
    }
        /* 纵向避让 */
    if (pASC->stInput.stGeneralInput.stCtrlBits.stBits.bActiveVerticalMode == 1 &&
        pASC->stInput.stGeneralInput.stCtrlBits.stBits.bActiveHorizontalMode == 0) {
        
        uint16_t nLiftArraySize = pASC->stInput.stVerticalInput.nLiftArraySize;
        uint16_t nDropArraySize = pASC->stInput.stVerticalInput.nDropArraySize;

        if (nLiftArraySize == 0 || nLiftArraySize > 10 || nDropArraySize == 0 || nDropArraySize > 10) {
            // 地形信息无效
            pASC->stVerticalTask.bHasInputErrors = 1;
            pASC->stOutput.stGeneralOutput.stErrorBits.stBits.bIsTerrainInfoInvalid = 1;
            printf("VerticalTask Input TerrainInfo Invalid.\n");
        } else {
            pASC->stOutput.stGeneralOutput.stErrorBits.stBits.bIsTerrainInfoInvalid = 0;
        }
    }

    /* 横向避障 */
    if (pASC->stInput.stGeneralInput.stCtrlBits.stBits.bActiveHorizontalMode == 1 && 
        pASC->stInput.stGeneralInput.stCtrlBits.stBits.bActiveVerticalMode == 0)
    {
        
        uint16_t nObstacleNum = pASC->stInput.stHorizontalInput.nObstacleNum;
        uint16_t nWaypointsNum = pASC->stInput.stHorizontalInput.nWaypointsNum;
        uint16_t nObstacleInflation = pASC->stInput.stHorizontalInput.nObstacleInflation;
        double dBridgeMinPos = pASC->stHorizontalTask.stBridgeParam.dMinPos;
        double dBridgeMaxPos = pASC->stHorizontalTask.stBridgeParam.dMaxPos;
        double dTrolleyMinPos = pASC->stHorizontalTask.stTrolleyParam.dMinPos;
        double dTrolleyMaxPos = pASC->stHorizontalTask.stTrolleyParam.dMaxPos;
        // 途径点数量判断
        if (nWaypointsNum > 20 || nObstacleNum > 5 || nObstacleInflation < 0.0)
        {
            pASC->stHoistTask.bHasInputErrors = 1;
            printf("Error: Invalid Waypoints or Obstacles\n");
        }
        // 途径点位置判断
        if (nWaypointsNum > 0)
        {
            for (int i = 0; i < nWaypointsNum; i++)
            {
                if (pASC->stInput.stHorizontalInput.arrWaypoints[i].dWaypointsCoordX < dBridgeMinPos ||
                    pASC->stInput.stHorizontalInput.arrWaypoints[i].dWaypointsCoordX > dBridgeMaxPos ||
                    pASC->stInput.stHorizontalInput.arrWaypoints[i].dWaypointsCoordY < dTrolleyMinPos ||
                    pASC->stInput.stHorizontalInput.arrWaypoints[i].dWaypointsCoordY > dTrolleyMaxPos)
                {
                    pASC->stHoistTask.bHasInputErrors = 1;
                    printf("Error: Invalid Waypoint Position\n");
                }
            }
        }
        // 检查宽高合法性
        if (nObstacleNum > 0)
        {
            for (int i = 0; i < nObstacleNum; i++)
            {
                if (pASC->stInput.stHorizontalInput.arrObstacle[i].dObstacleCoordW < 0.0 ||
                    pASC->stInput.stHorizontalInput.arrObstacle[i].dObstacleCoordH < 0.0)
                {
                    pASC->stHoistTask.bHasInputErrors = 1;
                    printf("Error: Invalid Obstacle Dimensions\n");
                }
            }
        }
    }

    if (pASC->stBridgeTask.bHasInputErrors == 1 || 
        pASC->stTrolleyTask.bHasInputErrors == 1 || 
        pASC->stHoistTask.bHasInputErrors == 1 ||
        pASC->stVerticalTask.bHasInputErrors == 1 ||
        pASC->stHorizontalTask.bHasInputErrors == 1) {
  
      // printf("Input errors!\n");
    }

    return 0;
}

static int8_t SKA_ASC_InputCmdParse(SKA_AntiswayController *pASC) {
    /******************** 函数参数合法性检验 ********************/
    if (pASC == NULL) {
        printf("Invalid paramerters of SKA_ASC_InputCmdParse().\n");
        return -1;
    }

    /******************** 输入指令解析 ********************/

    /* 通用 */
    // 纵向避障任务启动指令
    SKA_RED_Run(&(pASC->stGeneralICP.stStartVerticalTaskRED),
                pASC->stInput.stGeneralInput.stCtrlBits.stBits.bStartVerticalTask,
                &(pASC->stGeneralICP.bStartVerticalTask));
    // 横向避障任务启动指令
    SKA_RED_Run(&(pASC->stGeneralICP.stStartHorizontalTaskRED),
                pASC->stInput.stGeneralInput.stCtrlBits.stBits.bStartHorizontalTask,
                &(pASC->stGeneralICP.bStartHorizontalTask));

    /* 大车 */
    // 启动指令
    SKA_RED_Run(&(pASC->stBridgeICP.stStartRED), 
                pASC->stInput.stBridgeInput.stCtrlBits.stBits.bStart,
                &(pASC->stBridgeICP.bStart));
    // 无防摇急停指令
    SKA_RED_Run(&(pASC->stBridgeICP.stQuickStopRED), 
                pASC->stInput.stBridgeInput.stCtrlBits.stBits.bQuickStop,
                &(pASC->stBridgeICP.bQuickStop));
    // 带防摇缓停指令
    SKA_RED_Run(&(pASC->stBridgeICP.stAscStopRED), 
                pASC->stInput.stBridgeInput.stCtrlBits.stBits.bAntiswayStop,
                &(pASC->stBridgeICP.bAscStop));
    
    /* 小车 */
    // 启动指令
    SKA_RED_Run(&(pASC->stTrolleyICP.stStartRED), 
                pASC->stInput.stTrolleyInput.stCtrlBits.stBits.bStart,
                &(pASC->stTrolleyICP.bStart));
    // 无防摇急停指令
    SKA_RED_Run(&(pASC->stTrolleyICP.stQuickStopRED), 
                pASC->stInput.stTrolleyInput.stCtrlBits.stBits.bQuickStop,
                &(pASC->stTrolleyICP.bQuickStop));
    // 带防摇缓停指令
    SKA_RED_Run(&(pASC->stTrolleyICP.stAscStopRED), 
                pASC->stInput.stTrolleyInput.stCtrlBits.stBits.bAntiswayStop,
                &(pASC->stTrolleyICP.bAscStop));

    /* 起升 */
    SKA_RED_Run(&(pASC->stHoistICP.stStartRED), 
                pASC->stInput.stHoistInput.stCtrlBits.stBits.bStart,
                &(pASC->stHoistICP.bStart));
    // 无防摇急停指令
    SKA_RED_Run(&(pASC->stHoistICP.stQuickStopRED), 
                pASC->stInput.stHoistInput.stCtrlBits.stBits.bQuickStop,
                &(pASC->stHoistICP.bQuickStop));

    return 0;
}

static int8_t SKA_ASC_UpdateUsingParam(SKA_AntiswayController *pASC) {
    /******************** 函数参数合法性检验 ********************/
    if (pASC == NULL) {
        printf("Invalid paramerters of SKA_ASC_UpdateUsingParam().\n");
        return -1;
    }

    /******************** 参数更新 ********************/
    // 当且仅当大车、小车和起升机构无运送任务时，可以更新参数
    bool bCanUpdate = 0;
    if (pASC->eTaskMode == SKA_ASC_SINGLE_MOVE_TASK_MODE) {
        if (pASC->stBridgeTask.eTaskState == SKA_ASC_FREE_TASK_STATE && 
            pASC->stTrolleyTask.eTaskState == SKA_ASC_FREE_TASK_STATE &&
            pASC->stHoistTask.eTaskState == SKA_ASC_FREE_TASK_STATE) {
            bCanUpdate = 1;
        }
    } else if (pASC->eTaskMode == SKA_ASC_HORIZONTAL_AVOIDANCE_TASK_MODE) {
        if (pASC->stHorizontalTask.eTaskState == SKA_ASC_FREE_TASK_STATE && 
            pASC->stHoistTask.eTaskState == SKA_ASC_FREE_TASK_STATE) {
            bCanUpdate = 1;
        }
    } else if(pASC->eTaskMode == SKA_ASC_VERTICAL_AVOIDANCE_TASK_MODE) {
        if (pASC->stVerticalTask.eTaskState == SKA_ASC_FREE_TASK_STATE) {
            bCanUpdate = 1;
        }
    } else {
        return -1;
    }
    if (bCanUpdate) {
        pASC->stUsingParam = pASC->stParam;
    }

    return 0;
}

static int8_t SKA_ASC_TaskModeTransition(SKA_AntiswayController *pASC) {
    /******************** 函数参数合法性检验 ********************/
     if (pASC == NULL) {
        printf("Invalid paramerters of SKA_ASC_TaskModeTransition().\n");
        return -1;
    }
    
    /******************** 任务模式切换 ********************/
    if (pASC->eTaskMode == SKA_ASC_SINGLE_MOVE_TASK_MODE) {

        // 单轴模式->纵向避让模式
        if ((pASC->stBridgeTask.eTaskState == SKA_ASC_FREE_TASK_STATE &&
             pASC->stTrolleyTask.eTaskState == SKA_ASC_FREE_TASK_STATE &&
             pASC->stHoistTask.eTaskState == SKA_ASC_FREE_TASK_STATE) &&
             pASC->stInput.stGeneralInput.stCtrlBits.stBits.bActiveVerticalMode == 1 && 
             pASC->stInput.stGeneralInput.stCtrlBits.stBits.bActiveHorizontalMode == 0)
        {
            pASC->eTaskMode = SKA_ASC_VERTICAL_AVOIDANCE_TASK_MODE;
        }

        // 单轴模式->横向避让模式
        if ((pASC->stBridgeTask.eTaskState == SKA_ASC_FREE_TASK_STATE &&
            pASC->stTrolleyTask.eTaskState == SKA_ASC_FREE_TASK_STATE &&
            pASC->stHoistTask.eTaskState == SKA_ASC_FREE_TASK_STATE) &&
            pASC->stInput.stGeneralInput.stCtrlBits.stBits.bActiveHorizontalMode == 1 && 
             pASC->stInput.stGeneralInput.stCtrlBits.stBits.bActiveVerticalMode == 0)
       {
           pASC->eTaskMode = SKA_ASC_HORIZONTAL_AVOIDANCE_TASK_MODE;
       }  

    } else if (pASC->eTaskMode == SKA_ASC_HORIZONTAL_AVOIDANCE_TASK_MODE) {
        // 横向避让模式->单轴模式
        if ((pASC->stHorizontalTask.eTaskState == SKA_ASC_FREE_TASK_STATE) &&
            (pASC->stInput.stGeneralInput.stCtrlBits.stBits.bActiveHorizontalMode == 0 &&
             pASC->stInput.stGeneralInput.stCtrlBits.stBits.bActiveVerticalMode == 0))
        {
            pASC->eTaskMode = SKA_ASC_SINGLE_MOVE_TASK_MODE;
        }

    } else if(pASC->eTaskMode == SKA_ASC_VERTICAL_AVOIDANCE_TASK_MODE) {
        // 纵向避让模式->单轴模式
        if ((pASC->stVerticalTask.eTaskState == SKA_ASC_FREE_TASK_STATE) &&
            (pASC->stInput.stGeneralInput.stCtrlBits.stBits.bActiveVerticalMode == 0 && 
             pASC->stInput.stGeneralInput.stCtrlBits.stBits.bActiveHorizontalMode == 0))
        {
            pASC->eTaskMode = SKA_ASC_SINGLE_MOVE_TASK_MODE;
        }
        
    } else {
        return -1;
    }

    return 0;
}


static int8_t SKA_ASC_TaskStateTransition(SKA_AntiswayController *pASC) {
    /******************** 函数参数合法性检验 ********************/
    if (pASC == NULL) {
        printf("Invalid paramerters of SKA_ASC_TaskStateTransition().\n");
        return -1;
    }

    /******************** 任务状态切换 ********************/
    if (pASC->eTaskMode == SKA_ASC_SINGLE_MOVE_TASK_MODE) {
        // 大车单轴运动任务
        SKA_ASC_TravelSingleMoveTaskStateTransition(pASC->dCtrlPeriod,
                                                    &(pASC->stBridgeTask), 
                                                    &(pASC->stInput.stBridgeInput),
                                                    &(pASC->stUsingParam.stBridgeParam), 
                                                    &(pASC->stOutput.stGeneralOutput),
                                                    &(pASC->stOutput.stBridgeOutput),
                                                    &(pASC->stBridgeICP));

        // 小车单轴运动任务
        SKA_ASC_TravelSingleMoveTaskStateTransition(pASC->dCtrlPeriod,
                                                    &(pASC->stTrolleyTask), 
                                                    &(pASC->stInput.stTrolleyInput),
                                                    &(pASC->stUsingParam.stTrolleyParam), 
                                                    &(pASC->stOutput.stGeneralOutput),
                                                    &(pASC->stOutput.stTrolleyOutput),
                                                    &(pASC->stTrolleyICP));

        // 起升单轴运动任务
        SKA_ASC_HoistSingleMoveTaskStateTransition(pASC->dCtrlPeriod,
                                                   &(pASC->stHoistTask),
                                                   &(pASC->stInput.stGeneralInput),
                                                   &(pASC->stInput.stHoistInput),
                                                   &(pASC->stUsingParam.stHoistParam),
                                                   &(pASC->stOutput.stGeneralOutput),
                                                   &(pASC->stOutput.stHoistOutput),
                                                   &(pASC->stHoistICP));
    } else if (pASC->eTaskMode == SKA_ASC_HORIZONTAL_AVOIDANCE_TASK_MODE) {
        // 横向避障任务       
        SKA_ASC_HorizontalAvoidanceTaskStateTransition(pASC->dCtrlPeriod,
                                                     &(pASC->stHorizontalTask),
                                                     &(pASC->stInput.stHorizontalInput),
                                                     &(pASC->stInput.stBridgeInput),
                                                     &(pASC->stInput.stTrolleyInput),
                                                     &(pASC->stInput.stGeneralInput),
                                                     &(pASC->stUsingParam.stBridgeParam),
                                                     &(pASC->stUsingParam.stTrolleyParam),
                                                     &(pASC->stOutput.stGeneralOutput),
                                                     &(pASC->stOutput.stBridgeOutput),
                                                     &(pASC->stOutput.stTrolleyOutput),
                                                     &(pASC->stGeneralICP));
    } else if(pASC->eTaskMode == SKA_ASC_VERTICAL_AVOIDANCE_TASK_MODE) {
        // 纵向避障任务   
        
          SKA_ASC_VerticalAvoidanceTaskStateTransition(pASC->dCtrlPeriod,
                                                    &(pASC->stVerticalTask),
                                                    &(pASC->stInput.stVerticalInput),
                                                    &(pASC->stInput.stBridgeInput),
                                                    &(pASC->stInput.stTrolleyInput),
                                                    &(pASC->stInput.stHoistInput),
                                                    &(pASC->stInput.stGeneralInput),
                                                    &(pASC->stUsingParam.stBridgeParam), 
                                                    &(pASC->stUsingParam.stTrolleyParam), 
                                                    &(pASC->stUsingParam.stHoistParam),
                                                    &(pASC->stOutput.stGeneralOutput),
                                                    &(pASC->stOutput.stBridgeOutput),
                                                    &(pASC->stOutput.stTrolleyOutput),
                                                    &(pASC->stOutput.stHoistOutput),
                                                    &(pASC->stGeneralICP));
    } else {
        return -1;
    }

    return 0;
}

static int8_t SKA_ASC_TravelSingleMoveTaskStateTransition(double dCtrlPeriod,
                                                          SKA_TravelSingleMoveTask *pTask, 
                                                          SKA_AscTravelInput *pInput,
                                                          SKA_AscTravelParam *pParam, 
                                                          SKA_AscGeneralOutput *pGeneralOutput,
                                                          SKA_AscTravelOutput *pOutput,
                                                          SKA_AscTravelInputCmdParser *pCmdParser) {
    /******************** 函数参数合法性检验 ********************/
    if (pTask == NULL || pInput == NULL || pParam == NULL || pOutput == NULL || dCtrlPeriod <= 0 || pCmdParser == NULL) {
        printf("Invalid paramerters of SKA_ASC_TravelSingleMoveTaskStateTransition().\n");
        return -1;
    }

    /******************** 状态转换 ********************/
    
    // 使能指令
    bool bEnable = pInput->stCtrlBits.stBits.bEnable;

    // 启动指令
    bool bHasNewTask = pCmdParser->bStart;

    // 复位指令
    bool bReset = pInput->stCtrlBits.stBits.bReset;

    // 无防摇急停指令
    bool bQuickStop = pCmdParser->bQuickStop;

    // 带防摇缓停指令
    bool bAscStop = pCmdParser->bAscStop;

    // 预限位减速指令
    bool bPrelimit = pInput->stCtrlBits.stBits.bPrelimit;

    if (bEnable) {
        // 控制器使能

        // 使能反馈
        pOutput->stStateBits.stBits.bIsEnabled = 1;

        // 有效绳长反馈
        double dEffRopeLen = pInput->dEffRopeLength;
        pOutput->dEffRopeLen = dEffRopeLen;

        // 工作模式切换
        if (!pOutput->stErrorBits.stBits.bIsWorkModeInvalid) {
            if (pTask->nOperationMode == 0) {
                if (pInput->stCtrlBits.stBits.nOperationMode == 0) {
                    // 无变化
                    ;
                } else if (pInput->stCtrlBits.stBits.nOperationMode == 1) {
                    // 手动->自动
                    // 仅允许在空闲状态下切换
                    if (pTask->eTaskState == SKA_ASC_FREE_TASK_STATE) {
                        pTask->nOperationMode = 1;
                    }
                } else {
                    ;
                }
            } else if (pTask->nOperationMode == 1) {
                if (pInput->stCtrlBits.stBits.nOperationMode == 0) {
                    // 自动->手动
                    // 仅允许在空闲状态下切换
                    if (pTask->eTaskState == SKA_ASC_FREE_TASK_STATE) {
                        pTask->nOperationMode = 0;
                    }
                } else if (pInput->stCtrlBits.stBits.nOperationMode == 1) {
                    // 无变化
                    ;
                } else {
                    ;
                }
            } else {
                ;
            }
        }
        // 工作模式反馈
        pOutput->stStateBits.stBits.nOperationMode = pTask->nOperationMode;
        
        switch (pTask->eTaskState) {
        case SKA_ASC_FREE_TASK_STATE: {
            // 空闲状态
            // 设置输出
            pOutput->stStateBits.stBits.bIsPositive = 0;
            pOutput->stStateBits.stBits.bIsNegative = 0;
            pOutput->stStateBits.stBits.bIsBusy = 0;
            pOutput->stStateBits.stBits.bOpenBrake = 0;
            pOutput->stStateBits.stBits.bIsZeroManual = 0;
            pOutput->stStateBits.stBits.bIsPosErrOk = 0;
            pOutput->stStateBits.stBits.bIsZeroSpeed = 0;
            pOutput->stStateBits.stBits.bIsSwayAngleOk = 0;
            pOutput->dOutputFreq = 0.0;
            pOutput->dOutputSpeed = 0.0;
            pOutput->dBrakeDist = 0.0;
            pOutput->dOriTrackErr = 0.0;
            pOutput->dFilTrackErr = 0.0;

            if (bReset) {
                // 复位
                break;
            }

            if (pTask->bHasInputErrors) {
                // 存在输入错误
                break;
            }

            bool bStartNewTask = 0;
            if (pTask->nOperationMode == 0) {
                // 手动模式
                if (fabs(pInput->dManualFreq) > SKA_FlOAT_ERROR) {
                    // 启动新任务
                    bStartNewTask = 1;
                    printf("[%s]Start A New Manual Task.", pTask->pTaskName);
                }
            } else if (pTask->nOperationMode == 1) {
                // 自动模式
                if (bHasNewTask) {
                    // 启动新任务
                    bStartNewTask = 1;
                    // 记录任务的开始位置和目标位置
                    pTask->dTaskStartPos = pInput->dCurPos;
                    pTask->dTaskTargetPos = pInput->dTargetPos;
                    // 目标位置反馈
                    pOutput->dTargetPos = pTask->dTaskTargetPos;
                    printf("[%s]Start A New Auto Task. StartPos = %lfm, TargetPos = %lfm.\n", 
                           pTask->pTaskName, pTask->dTaskStartPos, pTask->dTaskTargetPos);
                }
            } else {
                return -1;
            }
            if (bStartNewTask) {
                // 拷贝参数
                pTask->stParam = *pParam;
                // 状态转换
                pTask->eTaskState = SKA_ASC_PLAN_TASK_STATE;
            }

            break;
        }
        case SKA_ASC_PLAN_TASK_STATE: {
            // 准备状态

            // 消除<跟踪误差过大>错误标志
            pOutput->stErrorBits.stBits.bIsTrackErrTooLarge = 0;
            
            // 消除<开闸时间过长>错误标志
            pOutput->stErrorBits.stBits.bIsBrakeOpenTimeTooLong = 0;

            // 消除<低频不能驱动>错误标志
            pOutput->stErrorBits.stBits.bCannotDriveByLowFreq = 0;

            // 消除<任务执行中复位>警告标志
            pOutput->stWarnBits.stBits.bResetInExecution = 0;

            // 消除<任务执行中无防摇急停>警告标志
            pOutput->stWarnBits.stBits.bQuickStopInExecution = 0;

            // 消除<任务执行中带防摇缓停>警告标志
            pOutput->stWarnBits.stBits.bAntiswayStopInExecution = 0;

            // 消除<任务执行中预限位减速>警告标志
            pOutput->stWarnBits.stBits.bPrelimitlInExecution = 0;

            // 设置<未激活防摇控制>警告标志
            if (pInput->stCtrlBits.stBits.bActiveSwayCtrl == 0) {
                pOutput->stWarnBits.stBits.bIsSwayCtrlDeactivated = 1;
            } else {
                pOutput->stWarnBits.stBits.bIsSwayCtrlDeactivated = 0;
            }

            // 设置<未激活定位控制>警告标志
            if (pTask->nOperationMode == 1) {
                // 仅在自动模式下生效
                if (pInput->stCtrlBits.stBits.bActivePosCtrl == 0) {
                    pOutput->stWarnBits.stBits.bIsPosCtrlDeactivated = 1;
                } else {
                    pOutput->stWarnBits.stBits.bIsPosCtrlDeactivated = 0;
                }
            }

            // 消除<任务执行中更新目标位置>警告标志
            pOutput->stWarnBits.stBits.bChangeTargetPosInExecution = 0;

            // 控制器忙碌
            pOutput->stStateBits.stBits.bIsBusy = 1;

            // 复位
            if (bReset) {

                printf("[%s]Reset.", pTask->pTaskName);

                // 状态转换
                pTask->eTaskState = SKA_ASC_FREE_TASK_STATE;

                // 设置<任务执行中复位>警告标志
                pOutput->stWarnBits.stBits.bResetInExecution = 1;

                break;
            }

            // 存在输入错误
            if (pTask->bHasInputErrors) {
                // 状态转换
                pTask->eTaskState = SKA_ASC_FREE_TASK_STATE;

                break;
            }
            
            if (pTask->nOperationMode == 1) {
                // 无防摇急停和带防摇缓停只在自动模式下生效
                if (bQuickStop) {
                    // 无防摇急停

                    // 状态转换
                    pTask->eTaskState = SKA_ASC_FREE_TASK_STATE;

                    // 设置<任务执行中无防摇急停>警告标志
                    pOutput->stWarnBits.stBits.bQuickStopInExecution = 1;

                    break;
                }
                if (bAscStop) {
                    // 带防摇缓停

                    // 状态转换
                    pTask->eTaskState = SKA_ASC_FREE_TASK_STATE;

                    // 设置<任务执行中带防摇缓停>警告标志
                    pOutput->stWarnBits.stBits.bAntiswayStopInExecution = 1;

                    break;
                }
            }         
                
            // 初始化

            /* 抱闸已开启过标志 */
            pTask->bHasBeenBrakeOpened = 0;

            /* 预限位标志 */

            pTask->bIsPreLimiting = 0;

            // 设置<任务执行中预限位减速>警告标志
            if (bPrelimit) {
                pOutput->stWarnBits.stBits.bPrelimitlInExecution = 1;
            }

            // 急停标志、缓停标志、轨迹规划器只在自动模式下生效
            if (pTask->nOperationMode == 1) {
                /* 正在无防摇急停标志 */
                pTask->bIsQuickStopping = 0;

                /* 正在带防摇缓停标志 */
                pTask->bIsAscStopping = 0;

                /* 轨迹规划器 */
                bool bActiveSwayCtrl = pInput->stCtrlBits.stBits.bActiveSwayCtrl;
                // 输入整形器
                SKA_InputShaperType eIsType;
                switch (pTask->stParam.nAsLevel)
                {
                case 0:
                    eIsType = SKA_IS_ZV;
                    break;
                case 1:
                    eIsType = SKA_IS_ZVD;
                    break;
                case 2:
                    eIsType = SKA_IS_ZVDD;
                    break;
                default:
                    return -1;
                }
                double dWn = sqrt(SKA_G / dEffRopeLen);
                double dZeta = 0;
                // 未整形规划器
                double dStartPos = pTask->dTaskStartPos;
                double dTargetPos = pTask->dTaskTargetPos;
                double dMaxVel;
                SKA_Min(pTask->stParam.dMaxRefSpeed, pInput->dSpeedLimit, &dMaxVel);
                if (pTask->bIsPreLimiting != bPrelimit) {
                    pTask->bIsPreLimiting = bPrelimit;
                    if (bPrelimit) {
                        // 预限位减速
                        SKA_Min(pTask->stParam.dPrelimitSpeed, dMaxVel, &dMaxVel);
                    } else {
                        // 取消预限位减速
                        ;
                    }
                }
                double dMaxAcc = pTask->stParam.dMaxRefAcc;
                // 为使可以达到最大加速度, 须jm >= am ^ 2 / vm
                double dMaxJerk = 5.0 * pow(dMaxAcc, 2) / dMaxVel;
                double dQuickStopMaxAcc = pTask->stParam.dMaxOutAcc;
                SKA_TUTP_Create(&(pTask->stTracePlanner), bActiveSwayCtrl,
                                eIsType, dWn, dZeta, 
                                dStartPos, dTargetPos, dMaxVel, dMaxAcc, dMaxJerk,
                                dQuickStopMaxAcc);
            }

            // 零手动输入检测器、辅助防摇控制器仅在手动模式下生效
            if (pTask->nOperationMode == 0) {
                /* 零手动输入检测器 */

                SKA_ZMD_Create(&(pTask->stZMD), dCtrlPeriod, 0.5);

                /* 辅助防摇控制器 */
                bool bActiveSwayCtrl = pInput->stCtrlBits.stBits.bActiveSwayCtrl;
                double dTs = dCtrlPeriod;
                double dMaxVel;
                SKA_Min(pTask->stParam.dMaxRefSpeed, pInput->dSpeedLimit, &dMaxVel);
                if (pTask->bIsPreLimiting != bPrelimit) {
                    pTask->bIsPreLimiting = bPrelimit;
                    if (bPrelimit) {
                        // 预限位减速
                        SKA_Min(pTask->stParam.dPrelimitSpeed, dMaxVel, &dMaxVel);
                    } else {
                        // 取消预限位减速
                        ;
                    }
                }
                double dMaxAcc = pTask->stParam.dMaxRefAcc;
                double dIneFilTc = 0.1;
                double dInitVel = 0;
                SKA_AAC_Create(&(pTask->stAAC), bActiveSwayCtrl, dTs, dMaxVel, dMaxAcc, dIneFilTc, dInitVel);
            }
            
            /* 跟踪控制器 */
            bool bActiveClosedPosCtrl;
            if (pTask->nOperationMode == 0) {
                // 手动模式下禁用跟踪控制器的闭环定位控制
                bActiveClosedPosCtrl = 0;
            } else {
                bActiveClosedPosCtrl = pInput->stCtrlBits.stBits.bActivePosCtrl;
            }
            double dTs = dCtrlPeriod;
            double dDelayTime = pTask->stParam.dDelayTime;
            double dStartPos = pTask->dTaskStartPos;
            double dMaxTrackErr = pTask->stParam.dTrackErrLim;
            double dMaxTrackErrRate = pTask->stParam.dTrackErrRateLim;
            double dTrackErrFilTc = pTask->stParam.dTrackErrFilTc;
            double dKp = pTask->stParam.dPosTrackKp;
            double dKi = pTask->stParam.dPosTrackKi;
            double dMaxVi = 0.1;
            double dPosDeadZone = pTask->stParam.dMaxPosErr / 2;
            double dMaxVel = pTask->stParam.dMaxOutSpeed;
            double dMaxAcc = pTask->stParam.dMaxOutAcc;
            double dMaxFreq = pTask->stParam.dMaxOutFreq;
            double dMinFreq = pTask->stParam.dMinMoveFreq;
            double dFreqCompenSoft = 0.5;
            SKA_TUTC_Init(&(pTask->stTrackCtrler), bActiveClosedPosCtrl, dTs, dDelayTime,
                            dStartPos, dMaxTrackErr, dMaxTrackErrRate, dTrackErrFilTc,
                            dKp, dKi, dMaxVi, dPosDeadZone, dMaxVel, dMaxAcc,
                            dMaxFreq, dMinFreq, dFreqCompenSoft);
            
            /* 计时器 */
            SKA_Timer_Start(&(pTask->stTimer));

            /* 零速度检测器 */
            // 检测时域长度(s)
            double dZeroSpeedTimeLen;
            if (pTask->nOperationMode == 0) {
                // 手动模式略短
                dZeroSpeedTimeLen = 0.5;
            } else if (pTask->nOperationMode == 1) {
                // 自动模式略长
                dZeroSpeedTimeLen = 1.5;
            } else {
                return -1;
            }
            SKA_ZSD_Create(&(pTask->stZSD), dCtrlPeriod, dZeroSpeedTimeLen,
                            pTask->stParam.dStaticSpeed);

            /* 无法驱动错误检测器 */
            double dCannotDriveTimeLen = 2.0;
            double dPosAccuracy = 0.01;
            double dFreqThreshold = 1.0;
            SKA_CDED_Create(&(pTask->stCDED), dCtrlPeriod, dCannotDriveTimeLen, 
                            dPosAccuracy, dFreqThreshold);

            // 状态转换
            pTask->eTaskState = SKA_ASC_EXECUTING_TASK_STATE;

            break;
        }
        case SKA_ASC_EXECUTING_TASK_STATE: {
            // 执行状态

            // 复位
            if (bReset) {

                printf("[%s]Reset.", pTask->pTaskName);

                // 状态转换
                // 需要进入完成状态执行清理工作
                pTask->eTaskState = SKA_ASC_FINISH_TASK_STATE;

                // 设置<任务执行中复位>警告标志
                pOutput->stWarnBits.stBits.bResetInExecution = 1;

                break;
            }

            // 打开抱闸
            pOutput->stStateBits.stBits.bOpenBrake = 1;

            // 在收到开闸反馈前输出运动方向
            if (!pTask->bHasBeenBrakeOpened) {
                if (pTask->dTaskTargetPos >= pTask->dTaskStartPos) {
                    pOutput->stStateBits.stBits.bIsPositive = 1;
                    pOutput->stStateBits.stBits.bIsNegative = 0;
                } else {
                    pOutput->stStateBits.stBits.bIsPositive = 0;
                    pOutput->stStateBits.stBits.bIsNegative = 1;
                }
            }

            // 判断抱闸是否开启
            if (!pTask->bHasBeenBrakeOpened &&
                pInput->stCtrlBits.stBits.bIsBrakeOpened) {
                // 抱闸已打开
                pTask->bHasBeenBrakeOpened = 1;
                // 重新计时
                SKA_Timer_Start(&(pTask->stTimer));
            }

            // 获取当前运行时刻
            // 在抱闸未开启时, 当前时刻为抱闸启动过程持续时长
            // 在抱闸已开启时, 当前时刻为控制作用运行时长
            double dCurTime;
            SKA_Timer_GetTime(&(pTask->stTimer), &dCurTime);

            // 存在输入错误
            if (pTask->bHasInputErrors) {
                bool bIsCurRopeLenInvalid = pGeneralOutput->stErrorBits.stBits.bIsCurRopeLenInvalid;
                bool bIsWorkModeInvalid = pOutput->stErrorBits.stBits.bIsWorkModeInvalid;
                bool bIsCurPosInvalid = pOutput->stErrorBits.stBits.bIsCurPosInvalid;
                bool bIsTargetPosInvalid = pOutput->stErrorBits.stBits.bIsTargetPosInvalid;
                bool bIsEffRopeLenInvalid = pOutput->stErrorBits.stBits.bIsEffRopeLenInvalid;

                if (bIsCurRopeLenInvalid || bIsWorkModeInvalid || bIsCurPosInvalid || bIsEffRopeLenInvalid) {
                    // 无防摇急停
                    if (!pTask->bIsQuickStopping) {
                        printf("[%s]Quick Stop.", pTask->pTaskName);

                        // 避免重复
                        pTask->bIsQuickStopping = 1;

                        // 参考轨迹更新时刻
                        double dTriggerTime;
                        if (!pTask->bHasBeenBrakeOpened) {
                            // 抱闸未开启, 视为在0时刻更新参考轨迹
                            dTriggerTime = 0.0;
                        } else {
                            dTriggerTime = dCurTime;
                        }

                        SKA_TUTP_QuickStop(&(pTask->stTracePlanner), dTriggerTime);
                    }
                } else if (bIsTargetPosInvalid) {
                    // 带防摇缓停
                    if (!pTask->bIsQuickStopping && !pTask->bIsAscStopping) {
                        printf("[%s]Antisway Stop.", pTask->pTaskName);

                        // 避免重复
                        pTask->bIsAscStopping = 1;

                        // 参考轨迹更新时刻
                        double dTriggerTime;
                        if (!pTask->bHasBeenBrakeOpened) {
                            // 抱闸未开启, 视为在0时刻更新参考轨迹
                            dTriggerTime = 0.0;
                        } else {
                            dTriggerTime = dCurTime;
                        }

                        SKA_TUTP_AntiswayStop(&(pTask->stTracePlanner), dTriggerTime);
                    }
                } else {
                    // None
                }

                break;
            }

            // 无防摇急停、带防摇缓停缓停、目标位置切换尽在自动模式下生效
            if (pTask->nOperationMode == 1) {
                // 无防摇急停
                if (!pTask->bIsQuickStopping && bQuickStop) {
                    printf("[%s]Quick Stop.", pTask->pTaskName);

                    // 避免重复
                    pTask->bIsQuickStopping = 1;

                    // 参考轨迹更新时刻
                    double dTriggerTime;
                    if (!pTask->bHasBeenBrakeOpened) {
                        // 抱闸未开启, 视为在0时刻更新参考轨迹
                        dTriggerTime = 0.0;
                    } else {
                        dTriggerTime = dCurTime;
                    }

                    SKA_TUTP_QuickStop(&(pTask->stTracePlanner), dTriggerTime);

                    // 设置<任务执行中无防摇急停>警告标志
                    pOutput->stWarnBits.stBits.bQuickStopInExecution = 1;
                }
                
                // 带防摇缓停
                if (!pTask->bIsQuickStopping && !pTask->bIsAscStopping && bAscStop) {
                    printf("[%s]Antisway Stop.", pTask->pTaskName);

                    // 避免重复
                    pTask->bIsAscStopping = 1;

                    // 参考轨迹更新时刻
                    double dTriggerTime;
                    if (!pTask->bHasBeenBrakeOpened) {
                        // 抱闸未开启, 视为在0时刻更新参考轨迹
                        dTriggerTime = 0.0;
                    } else {
                        dTriggerTime = dCurTime;
                    }

                    SKA_TUTP_AntiswayStop(&(pTask->stTracePlanner), dTriggerTime);

                    // 设置<任务执行中带防摇缓停>警告标志
                    pOutput->stWarnBits.stBits.bAntiswayStopInExecution = 1;
                }
                
                // 目标位置切换
                if (!pTask->bIsQuickStopping && !pTask->bIsAscStopping &&
                    fabs(pInput->dTargetPos - pTask->dTaskTargetPos) > SKA_FlOAT_ERROR) {

                    printf("[%s]Changing New Target Position: %.3fm.\n", pTask->pTaskName, pInput->dTargetPos);

                    // 参考轨迹更新时刻
                    double dTriggerTime;
                    if (!pTask->bHasBeenBrakeOpened) {
                        // 抱闸未开启, 视为在0时刻更新参考轨迹
                        dTriggerTime = 0.0;
                    } else {
                        dTriggerTime = dCurTime;
                    }

                    SKA_TUTP_ModifyTargetPos(&(pTask->stTracePlanner), dTriggerTime, pInput->dTargetPos);

                    printf("[%s]Changed New Target Position: %.3fm.\n", pTask->pTaskName, pInput->dTargetPos);

                    pTask->dTaskTargetPos = pInput->dTargetPos;

                    pOutput->dTargetPos = pTask->dTaskTargetPos;
                    
                    // 设置<任务执行中更新目标位置>警告标志
                    pOutput->stWarnBits.stBits.bChangeTargetPosInExecution = 1;
                }

                if (!pTask->bIsQuickStopping && !pTask->bIsAscStopping) {
                    // 预限位减速功能在急停或缓停时无效
                    if (pTask->bIsPreLimiting != bPrelimit) {
                        pTask->bIsPreLimiting = bPrelimit;
                        double dMaxVel;
                        SKA_Min(pTask->stParam.dMaxRefSpeed, pInput->dSpeedLimit, &dMaxVel);
                        if (bPrelimit) {
                            // 预限位减速
                            SKA_Min(pTask->stParam.dPrelimitSpeed, dMaxVel, &dMaxVel);
                        } else {
                            // 取消预限位减速
                            ;
                        }
                        // 重新规划轨迹
                        // 参考轨迹更新时刻
                        double dTriggerTime;
                        if (!pTask->bHasBeenBrakeOpened) {
                            // 抱闸未开启, 视为在0时刻更新参考轨迹
                            dTriggerTime = 0.0;
                        } else {
                            dTriggerTime = dCurTime;
                        }
                        SKA_TUTP_ModifyMaxVelocity(&(pTask->stTracePlanner), dTriggerTime, dMaxVel);
                    }
                }
            }

            // 设置<任务执行中预限位减速>警告标志
            if (bPrelimit) {
                pOutput->stWarnBits.stBits.bPrelimitlInExecution = 1;
            }

            // 检测抱闸是否开启
            if (!pTask->bHasBeenBrakeOpened) {
                // 等待抱闸打开

                // 设置<开闸时间过长>错误标志
                double dMaxOpenBrakeTime = 5.0;
                bool bIsBrakeOpenTimeTooLong = pOutput->stErrorBits.stBits.bIsBrakeOpenTimeTooLong;
                if (!bIsBrakeOpenTimeTooLong && dCurTime > dMaxOpenBrakeTime) {
                    pOutput->stErrorBits.stBits.bIsBrakeOpenTimeTooLong = 1;
                    // 错误处理逻辑
                    // 转换为完成状态

                    printf("[%s]ERROR: Open Brake Time Too Long.\n", pTask->pTaskName);

                    // 设置输出
                    pOutput->stStateBits.stBits.bOpenBrake = 0;

                    // 状态转换
                    pTask->eTaskState = SKA_ASC_FINISH_TASK_STATE;
                }

            } else {
                // 抱闸已打开, 执行轨迹跟踪控制算法
                
                double dRefX, dRefDx, dRefDdx;

                if (pTask->nOperationMode == 0) {
                    // 辅助防摇仅在手动模式下生效

                    /* 辅助防摇 */

                    if (pTask->bIsPreLimiting != bPrelimit) {
                        pTask->bIsPreLimiting = bPrelimit;
                        double dMaxVel;
                        SKA_Min(pTask->stParam.dMaxRefSpeed, pInput->dSpeedLimit, &dMaxVel);
                        if (bPrelimit) {
                            // 预限位减速
                            SKA_Min(pTask->stParam.dPrelimitSpeed, dMaxVel, &dMaxVel);
                        } else {
                            // 取消预限位减速
                            ;
                        }
                        // 重新设置最大速度
                        SKA_AAC_SetMaxVelocity(&(pTask->stAAC), dMaxVel);
                    }
                    double dManualVelocity = pInput->dManualFreq * 
                                                pTask->stParam.dMaxOutSpeed / pTask->stParam.dMaxOutFreq;
                    SKA_InputShaperType eIsType;
                    switch (pTask->stParam.nAsLevel)
                    {
                    case 0:
                        eIsType = SKA_IS_ZV;
                        break;
                    case 1:
                        eIsType = SKA_IS_ZVD;
                        break;
                    case 2:
                        eIsType = SKA_IS_ZVDD;
                        break;
                    default:
                        return -1;
                    }
                    double dWn = sqrt(SKA_G / dEffRopeLen);
                    double dZeta = 0;
                    SKA_AAC_Run(&(pTask->stAAC), dManualVelocity, eIsType, dWn, dZeta, &dRefDx);
                }

                if (pTask->nOperationMode == 1) {
                    // 轨迹规划器、制动距离估计仅在自动模式下生效

                    /* 轨迹规划 */

                    double dUnsRefx, dUnsRefDx, dUnsRefDdx;
                    SKA_TUTP_Run(&(pTask->stTracePlanner), dCurTime,
                                &dRefX, &dRefDx, &dRefDdx, &dUnsRefx, &dUnsRefDx, &dUnsRefDdx);

                    // 估计制动距离
                    double dBrakeDistance;
                    SKA_TUTP_GetAscBrakeDistance(&(pTask->stTracePlanner), dCurTime, &dBrakeDistance);
                    pOutput->dBrakeDist = dBrakeDistance;
                }

                /* 跟踪控制 */

                if (pTask->nOperationMode == 0) {
                    // 手动模式下的跟踪控制

                    double dOutputFreq, dOutputVel;
                    int8_t nDirection;
                    SKA_TUTC_Run(&(pTask->stTrackCtrler),
                                0.0, dRefDx, 0.0, 0.0,
                                &dOutputVel, &dOutputFreq, &nDirection);

                    if (nDirection == 1) {
                        pOutput->stStateBits.stBits.bIsPositive = 1;
                        pOutput->stStateBits.stBits.bIsNegative = 0;
                    } else if (nDirection == -1) {
                        pOutput->stStateBits.stBits.bIsPositive = 0;
                        pOutput->stStateBits.stBits.bIsNegative = 1;
                    } else {
                        pOutput->stStateBits.stBits.bIsPositive = 0;
                        pOutput->stStateBits.stBits.bIsNegative = 0;
                    }
                    pOutput->dOutputFreq = dOutputFreq;
                    pOutput->dOutputSpeed = dOutputVel;

                } else if (pTask->nOperationMode == 1) {
                    // 自动模式下的跟踪控制

                    // 缓停或急停时停用闭环定位控制
                    if (pTask->bIsQuickStopping || pTask->bIsAscStopping) {
                        pTask->stTrackCtrler.bActivePosCtrl = 0;
                    }

                    // 积分分离
                    double dOpenIntegThreshold = 1.0;
                    double dTaskTargetPos = pTask->dTaskTargetPos;
                    double dCurPos = pInput->dCurPos;
                    if(fabs(dTaskTargetPos - dCurPos) < dOpenIntegThreshold){
                        pTask->stTrackCtrler.bOpenIntegration = 1;
                    }else{
                        pTask->stTrackCtrler.bOpenIntegration = 0;
                    }

                    double dOutputFreq, dOutputVel;
                    int8_t nDirection;
                    SKA_TUTC_Run(&(pTask->stTrackCtrler),
                                dRefX, dRefDx, dRefDdx, dCurPos,
                                &dOutputVel, &dOutputFreq, &nDirection);
                    
                    // 设置<跟踪误差过大>错误标志
                    // <跟踪误差过大>错误标志只在自动模式下激活定位控制时有效
                    if (pTask->stTrackCtrler.bActivePosCtrl == 1) {
                        bool bIsTrackErrTooLarge = pOutput->stErrorBits.stBits.bIsTrackErrTooLarge;
                        double dPosTrackErr;
                        SKA_TUTC_GetPosTrackError(&(pTask->stTrackCtrler), &dPosTrackErr);
                        if (!bIsTrackErrTooLarge && fabs(dPosTrackErr) > pTask->stParam.dMaxTrackErr) {
                            printf("[%s]ERROR: Track Error Too Large.\n", pTask->pTaskName);
                            pOutput->stErrorBits.stBits.bIsTrackErrTooLarge = 1;
                            // 错误处理逻辑
                            // 带防摇缓停
                            if (!pTask->bIsQuickStopping && 
                                !pTask->bIsAscStopping) {
                                // 避免重复
                                pTask->bIsAscStopping = 1;
                                SKA_TUTP_AntiswayStop(&(pTask->stTracePlanner), dCurTime);
                            }
                        }
                    }

                    if (nDirection == 1) {
                        pOutput->stStateBits.stBits.bIsPositive = 1;
                        pOutput->stStateBits.stBits.bIsNegative = 0;
                    } else if (nDirection == -1) {
                        pOutput->stStateBits.stBits.bIsPositive = 0;
                        pOutput->stStateBits.stBits.bIsNegative = 1;
                    } else {
                        pOutput->stStateBits.stBits.bIsPositive = 0;
                        pOutput->stStateBits.stBits.bIsNegative = 0;
                    }
                    pOutput->dOutputFreq = dOutputFreq;
                    pOutput->dOutputSpeed = dOutputVel;
                    pOutput->dOriTrackErr = pTask->stTrackCtrler.dOriTrackError;
                    pOutput->dFilTrackErr = pTask->stTrackCtrler.dFilTrackError;

                } else {
                    return -1;
                }

                // 设置<无法驱动>错误标志
                if (!pOutput->stErrorBits.stBits.bCannotDriveByLowFreq) {
                    bool bCannotDrive;
                    SKA_CDED_Run(&(pTask->stCDED), pInput->dCurPos, 
                                    pOutput->dOutputFreq, &bCannotDrive);
                    if (bCannotDrive) {
                        printf("[%s]ERROR: Cannot Drive.\n", pTask->pTaskName);
                        pOutput->stErrorBits.stBits.bCannotDriveByLowFreq = 1;
                    }
                }
            }
            
            /* 任务完成条件 */

            // 零手动输入条件
            bool bIsZeroManual;
            if (pTask->nOperationMode == 0) {
                SKA_ZMD_Run(&(pTask->stZMD), pInput->dManualFreq, &bIsZeroManual);
            } else if (pTask->nOperationMode == 1) {
                // 零手动输入条件仅在手动模式下生效，自动模式下视为满足条件
                bIsZeroManual = 1;
            } else {
                return -1;
            }
            pOutput->stStateBits.stBits.bIsZeroManual = bIsZeroManual;

            // 定位误差条件
            bool bIsPosErrOk;
            if (pTask->nOperationMode == 0) {
                // 手动模式下定位功能无效, 视为定位误差满足条件
                bIsPosErrOk = 1;
            } else if (pTask->nOperationMode == 1) {
                double dTaskTargetPos = pTask->dTaskTargetPos;
                double dCurPos = pInput->dCurPos;
                double dMaxPosErr = pTask->stParam.dMaxPosErr;
                // 缓停或急停状态下目标位置无效，视为满足定位误差条件
                if (fabs(dTaskTargetPos - dCurPos) < dMaxPosErr ||
                    pTask->bIsQuickStopping || pTask->bIsAscStopping) {
                    bIsPosErrOk = 1;
                } else {
                    bIsPosErrOk = 0;
                }
            } else {
                return -1;
            }
            pOutput->stStateBits.stBits.bIsPosErrOk = bIsPosErrOk;

            // 零速度条件
            bool bIsZeroSpeed;
            double dCurPos = pInput->dCurPos;
            SKA_ZSD_Run(&(pTask->stZSD), dCurPos, &bIsZeroSpeed);
            pOutput->stStateBits.stBits.bIsZeroSpeed = bIsZeroSpeed;

            // 负载摆角条件
            bool bIsSwayAngleOk;
            // 标准防摇版本的负载角度根据理论模型计算, 视为满足负载摆角条件
            if (pTask->nOperationMode == 0) {
                bIsSwayAngleOk = 1;
            } else if (pTask->nOperationMode == 1) {
                bIsSwayAngleOk = 1;
                // double dRefTraceTotalTime;
                // SKA_TUTP_GetTotalTime(&(pTask->stTracePlanner), &dRefTraceTotalTime);
                // if (dRefTraceTotalTime < SKA_FlOAT_ERROR) {
                //     // 参考轨迹持续时长为0, 视为无须移车
                //     bIsSwayAngleOk = 1;
                // } else {
                //     if (pTask->bHasBeenBrakeOpened && dCurTime > dRefTraceTotalTime) {
                //         bIsSwayAngleOk = 1;
                //     } else {
                //         bIsSwayAngleOk = 0;
                //     }
                // }
            } else {
                return -1;
            }
            pOutput->stStateBits.stBits.bIsSwayAngleOk = bIsSwayAngleOk;

            // 任务完成条件判断
            if (bIsZeroManual && bIsPosErrOk && bIsZeroSpeed && bIsSwayAngleOk) {
                // 任务完成

                // 设置输出
                pOutput->stStateBits.stBits.bIsPositive = 0;
                pOutput->stStateBits.stBits.bIsNegative = 0;
                pOutput->stStateBits.stBits.bOpenBrake = 0;
                pOutput->dOutputFreq = 0.0;
                pOutput->dOutputSpeed = 0.0;
                
                // 状态转换
                pTask->eTaskState = SKA_ASC_FINISH_TASK_STATE;
            }
            
            break;
        }
        case SKA_ASC_FINISH_TASK_STATE: {
            // 完成状态

            printf("[%s]Task Finished.\n", pTask->pTaskName);

            /* 设置输出 */
            pOutput->stStateBits.stBits.bIsPositive = 0;
            pOutput->stStateBits.stBits.bIsNegative = 0;
            pOutput->stStateBits.stBits.bIsBusy = 0;
            pOutput->stStateBits.stBits.bOpenBrake = 0;
            pOutput->stStateBits.stBits.bIsZeroManual = 0;
            pOutput->stStateBits.stBits.bIsPosErrOk = 0;
            pOutput->stStateBits.stBits.bIsZeroSpeed = 0;
            pOutput->stStateBits.stBits.bIsSwayAngleOk = 0;
            pOutput->dOutputFreq = 0.0;
            pOutput->dOutputSpeed = 0.0;
            pOutput->dBrakeDist = 0.0;
            pOutput->dOriTrackErr = 0.0;
            pOutput->dFilTrackErr = 0.0;

            /* 清理操作 */

            if (pTask->nOperationMode == 0) {
                // 手动模式独有的模块

                // 零手动输入检测器
                SKA_ZMD_Destroy(&(pTask->stZMD));

                // 辅助防摇控制器
                SKA_AAC_Destroy(&(pTask->stAAC));

            } else if (pTask->nOperationMode == 1) {
                // 自动模式独有的模块

                // 轨迹规划器
                SKA_TUTP_Destroy(&(pTask->stTracePlanner));
            } else {
                return -1;
            }

            // 零速度检测器
            SKA_ZSD_Destroy(&(pTask->stZSD));

            /* 无法驱动错误检测器 */
            SKA_CDED_Destroy(&(pTask->stCDED));

            /* 状态转换 */
            pTask->eTaskState = SKA_ASC_FREE_TASK_STATE;

            break;
        }
        default: {
            return -1;
        }
        }
    } else {
        // 控制器未使能

        // 设置输出
        pOutput->stStateBits.nData = 0;
        pOutput->stErrorBits.nData = 0;
        pOutput->stWarnBits.nData = 0;
        pOutput->dOutputFreq = 0.0;
        pOutput->dOutputSpeed = 0.0;
        pOutput->dBrakeDist = 0.0;
        pOutput->dEffRopeLen = 0.0;
        pOutput->dTargetPos = 0.0;
        pOutput->dOriTrackErr = 0.0;
        pOutput->dFilTrackErr = 0.0;

        // 控制器复位
        pTask->eTaskState = SKA_ASC_FREE_TASK_STATE;
    }

    return 0;
}

static int8_t SKA_ASC_HoistSingleMoveTaskStateTransition(double dCtrlPeriod,
                                                         SKA_HoistSingleMoveTask *pTask, 
                                                         SKA_AscGeneralInput *pGeneralInput,
                                                         SKA_AscHoistInput  *pInput,
                                                         SKA_AscHoistParam *pParam, 
                                                         SKA_AscGeneralOutput *pGeneralOutput,
                                                         SKA_AscHoistOutput *pOutput,
                                                         SKA_AscHoistInputCmdParser *pCmdParser) {
    /******************** 函数参数合法性检验 ********************/
    if (pTask == NULL || pInput == NULL || pParam == NULL || pOutput == NULL || dCtrlPeriod <= 0 || pCmdParser == NULL) {
        printf("Invalid parameters of SKA_ASC_HoistSingleMoveTaskStateTransition().\n");
        return -1;
    }

    /******************** 状态转换 ********************/
    
    // 使能指令
    bool bEnable = pInput->stCtrlBits.stBits.bEnable;

    // 启动指令
    bool bHasNewTask = pCmdParser->bStart;

    // 复位指令
    bool bReset = pInput->stCtrlBits.stBits.bReset;

    // 急停指令
    bool bQuickStop = pCmdParser->bQuickStop;

    // 预限位减速指令
    bool bPrelimit = pInput->stCtrlBits.stBits.bPrelimit;

    if (bEnable) {
        // 控制器使能

        // 使能反馈
        pOutput->stStateBits.stBits.bIsEnabled = bEnable;

        // 工作模式切换
        if (!pOutput->stErrorBits.stBits.bIsWorkModeInvalid) {
            if (pTask->nOperationMode == 0) {
                if (pInput->stCtrlBits.stBits.nOperationMode == 0) {
                    // 无变化
                    ;
                } else if (pInput->stCtrlBits.stBits.nOperationMode == 1) {
                    // 手动->自动
                    // 仅允许在空闲状态下切换
                    if (pTask->eTaskState == SKA_ASC_FREE_TASK_STATE) {
                        pTask->nOperationMode = 1;
                    }
                } else {
                    ;
                }
            } else if (pTask->nOperationMode == 1) {
                if (pInput->stCtrlBits.stBits.nOperationMode == 0) {
                    // 自动->手动
                    // 仅允许在空闲状态下切换
                    if (pTask->eTaskState == SKA_ASC_FREE_TASK_STATE) {
                        pTask->nOperationMode = 0;
                    }
                } else if (pInput->stCtrlBits.stBits.nOperationMode == 1) {
                    // 无变化
                    ;
                } else {
                    ;
                }
            } else {
                ;
            }
        }
        // 工作模式反馈
        pOutput->stStateBits.stBits.nOperationMode = pTask->nOperationMode;
        
        switch (pTask->eTaskState) {
        case SKA_ASC_FREE_TASK_STATE: {
            // 空闲状态

            // 设置输出
            pOutput->stStateBits.stBits.bIsPositive = 0;
            pOutput->stStateBits.stBits.bIsNegative = 0;
            pOutput->stStateBits.stBits.bIsBusy = 0;
            pOutput->stStateBits.stBits.bIsZeroManual = 0;
            pOutput->stStateBits.stBits.bIsPosErrOk = 0;
            pOutput->stStateBits.stBits.bIsZeroSpeed = 0;
            pOutput->dOutputFreq = 0.0;
            pOutput->dOutputSpeed = 0.0;
            pOutput->dOriTrackErr = 0.0;
            pOutput->dFilTrackErr = 0.0;

            if (bReset) {
                // 复位
                break;
            }

            if (pTask->bHasInputErrors) {
                // 存在输入错误
                break;
            }

            bool bStartNewTask = 0;
            if (pTask->nOperationMode == 0) {
                // 手动模式
                if (fabs(pInput->dManualFreq) > SKA_FlOAT_ERROR) {
                    // 启动新任务
                    bStartNewTask = 1;
                    printf("[%s]Start A New Manual Task.", pTask->pTaskName);
                }
            } else if (pTask->nOperationMode == 1) {
                // 自动模式
                if (bHasNewTask) {
                    // 启动新任务
                    bStartNewTask = 1;
                    // 记录任务的开始位置和目标位置
                    pTask->dTaskStartPos = pGeneralInput->dCurRopeLength;
                    pTask->dTaskTargetPos = pInput->dTargetRopeLength;
                    // 目标位置反馈
                    pOutput->dTargetRopeLength = pTask->dTaskTargetPos;
                    printf("[%s]Start A New Auto Task. StartPos = %lfm, TargetPos = %lfm.\n", 
                           pTask->pTaskName, pTask->dTaskStartPos, pTask->dTaskTargetPos);
                }
            } else {
                return -1;
            }
            if (bStartNewTask) {
                // 拷贝参数
                pTask->stParam = *pParam;
                // 状态转换
                pTask->eTaskState = SKA_ASC_PLAN_TASK_STATE;
            }

            break;
        }
        case SKA_ASC_PLAN_TASK_STATE: {
            // 准备状态

            // 消除<跟踪误差过大>错误标志
            pOutput->stErrorBits.stBits.bIsTrackErrTooLarge = 0;
            
            // 消除<开闸时间过长>错误标志
            pOutput->stErrorBits.stBits.bIsBrakeOpenTimeTooLong = 0;

            // 消除<低频不能驱动>错误标志
            pOutput->stErrorBits.stBits.bCannotDriveByLowFreq = 0;

            // 消除<任务执行中复位>警告标志
            pOutput->stWarnBits.stBits.bResetInExecution = 0;

            // 消除<任务执行中急停>警告标志
            pOutput->stWarnBits.stBits.bQuickStopInExecution = 0;

            // 消除<任务执行中预限位减速>警告标志
            pOutput->stWarnBits.stBits.bPrelimitlInExecution = 0;

            // 设置<未激活定位控制>警告标志
            // 设置<未激活定位控制>警告标志
            if (pTask->nOperationMode == 1) {
                // 仅在自动模式下生效
                if (pInput->stCtrlBits.stBits.bActivePosCtrl == 0) {
                    pOutput->stWarnBits.stBits.bIsPosCtrlDeactivated = 1;
                } else {
                    pOutput->stWarnBits.stBits.bIsPosCtrlDeactivated = 0;
                }
            }

            // 消除<任务执行中更新目标位置>警告标志
            pOutput->stWarnBits.stBits.bChangeTargetRopeLenInExecution = 0;

            // 控制器忙碌
            pOutput->stStateBits.stBits.bIsBusy = 1;

            // 复位
            if (bReset) {

                printf("[%s]Reset.", pTask->pTaskName);

                // 状态转换
                pTask->eTaskState = SKA_ASC_FREE_TASK_STATE;

                // 设置<任务执行中复位>警告标志
                pOutput->stWarnBits.stBits.bResetInExecution = 1;

                break;
            }

            // 存在输入错误
            if (pTask->bHasInputErrors) {
                // 状态转换
                pTask->eTaskState = SKA_ASC_FREE_TASK_STATE;

                break;
            }
            
            if (pTask->nOperationMode == 1) {
                // 急停只在自动模式下生效
                if (bQuickStop) {
                    // 无防摇急停

                    // 状态转换
                    pTask->eTaskState = SKA_ASC_FREE_TASK_STATE;

                    // 设置<任务执行中急停>警告标志
                    pOutput->stWarnBits.stBits.bQuickStopInExecution = 1;

                    break;
                }
            }         
                
            // 初始化

            /* 抱闸已开启过标志 */
            pTask->bHasBeenBrakeOpened = 0;

            /* 预限位标志 */

            pTask->bIsPreLimiting = 0;

            // 设置<任务执行中预限位减速>警告标志
            if (bPrelimit) {
                pOutput->stWarnBits.stBits.bPrelimitlInExecution = 1;
            }

            // 急停标志、轨迹规划器只在自动模式下生效
            if (pTask->nOperationMode == 1) {
                /* 正在无防摇急停标志 */
                pTask->bIsQuickStopping = 0;

                /* 轨迹规划器 */
                double dStartPos = pTask->dTaskStartPos;
                double dTargetPos = pTask->dTaskTargetPos;
                double dMaxVel;
                SKA_Min(pTask->stParam.dMaxRefSpeed, pInput->dSpeedLimit, &dMaxVel);
                if (pTask->bIsPreLimiting != bPrelimit) {
                    pTask->bIsPreLimiting = bPrelimit;
                    if (bPrelimit) {
                        // 预限位减速
                        SKA_Min(pTask->stParam.dPrelimitSpeed, dMaxVel, &dMaxVel);
                    } else {
                        // 取消预限位减速
                        ;
                    }
                }
                double dMaxAcc = pTask->stParam.dMaxRefAcc;
                // 为使可以达到最大加速度, 须jm >= am ^ 2 / vm
                double dMaxJerk = 5.0 * pow(dMaxAcc, 2) / dMaxVel;
                SKA_HUTP_Create(&(pTask->stTracePlanner), dStartPos, dTargetPos, dMaxVel, dMaxAcc, dMaxJerk);
            }

            // 零手动输入检测器仅在手动模式下生效
            if (pTask->nOperationMode == 0) {
                /* 零手动输入检测器 */
                SKA_ZMD_Create(&(pTask->stZMD), dCtrlPeriod, 0.5);
            }
            
            /* 跟踪控制器 */
            bool bActiveClosedPosCtrl;
            if (pTask->nOperationMode == 0) {
                // 手动模式下禁用跟踪控制器的闭环定位控制
                bActiveClosedPosCtrl = 0;
            } else {
                bActiveClosedPosCtrl = pInput->stCtrlBits.stBits.bActivePosCtrl;
            }
            double dTs = dCtrlPeriod;
            double dDelayTime = pTask->stParam.dDelayTime;
            double dStartPos = pTask->dTaskStartPos;
            double dMaxTrackErr = pTask->stParam.dTrackErrLim;
            double dMaxTrackErrRate = pTask->stParam.dTrackErrRateLim;
            double dTrackErrFilTc = pTask->stParam.dTrackErrFilTc;
            double dKp = pTask->stParam.dPosTrackKp;
            double dKi = pTask->stParam.dPosTrackKi;
            double dMaxVi = 0.1;
            double dPosDeadZone = pTask->stParam.dMaxPosErr / 2;
            double dMaxVel = pTask->stParam.dMaxOutSpeed;
            double dMaxAcc = pTask->stParam.dMaxOutAcc;
            double dMaxFreq = pTask->stParam.dMaxOutFreq;
            double dMinFreq = pTask->stParam.dMinMoveFreq;
            double dFreqCompenSoft = 0.5;
            SKA_HUTC_Init(&(pTask->stTrackCtrler), bActiveClosedPosCtrl, dTs, dDelayTime,
                            dStartPos, dMaxTrackErr, dMaxTrackErrRate, dTrackErrFilTc,
                            dKp, dKi, dMaxVi, dPosDeadZone, dMaxVel, dMaxAcc,
                            dMaxFreq, dMinFreq, dFreqCompenSoft);
            
            /* 计时器 */
            SKA_Timer_Start(&(pTask->stTimer));

            /* 零速度检测器 */
            // 检测时域长度(s)
            double dZeroSpeedTimeLen;
            if (pTask->nOperationMode == 0) {
                // 手动模式略短
                dZeroSpeedTimeLen = 0.5;
            } else if (pTask->nOperationMode == 1) {
                // 自动模式略长
                dZeroSpeedTimeLen = 1.5;
            } else {
                return -1;
            }
            SKA_ZSD_Create(&(pTask->stZSD), dCtrlPeriod, dZeroSpeedTimeLen,
                            pTask->stParam.dStaticSpeed);

            /* 无法驱动错误检测器 */
            double dCannotDriveTimeLen = 3.0;
            double dPosAccuracy = 0.01;
            double dFreqThreshold = 1.0;
            SKA_CDED_Create(&(pTask->stCDED), dCtrlPeriod, dCannotDriveTimeLen, 
                            dPosAccuracy, dFreqThreshold);

            // 状态转换
            pTask->eTaskState = SKA_ASC_EXECUTING_TASK_STATE;

            break;
        }
        case SKA_ASC_EXECUTING_TASK_STATE: {
            // 执行状态

            // 复位
            if (bReset) {

                printf("[%s]Reset.", pTask->pTaskName);
                
                // 状态转换
                // 需要进入完成状态执行清理工作
                pTask->eTaskState = SKA_ASC_FINISH_TASK_STATE;

                // 设置<任务执行中复位>警告标志
                pOutput->stWarnBits.stBits.bResetInExecution = 1;

                break;
            }

            // 获取当前运行时刻
            double dCurTime;
            SKA_Timer_GetTime(&(pTask->stTimer), &dCurTime);

            // 判断抱闸是否开启
            if (!pTask->bHasBeenBrakeOpened &&
                pInput->stCtrlBits.stBits.bIsBrakeOpened) {
                // 抱闸已打开
                pTask->bHasBeenBrakeOpened = 1;
            }
            if (!pTask->bHasBeenBrakeOpened) {
                // 设置<开闸时间过长>错误标志
                double dMaxOpenBrakeTime = 5.0;
                bool bIsBrakeOpenTimeTooLong = pOutput->stErrorBits.stBits.bIsBrakeOpenTimeTooLong;
                if (!bIsBrakeOpenTimeTooLong && dCurTime > dMaxOpenBrakeTime) {
                    pOutput->stErrorBits.stBits.bIsBrakeOpenTimeTooLong = 1;
                    // 错误处理逻辑
                    // 转换为完成状态

                    printf("[%s]ERROR: Open Brake Time Too Long.\n", pTask->pTaskName);

                    // 状态转换
                    pTask->eTaskState = SKA_ASC_FINISH_TASK_STATE;
                }
            }

            // 存在输入错误
            if (pTask->bHasInputErrors) {
                // 急停
                if (!pTask->bIsQuickStopping) {
                    printf("[%s]Quick Stop.", pTask->pTaskName);

                    // 避免重复
                    pTask->bIsQuickStopping = 1;

                    SKA_HUTP_QuickStop(&(pTask->stTracePlanner), dCurTime);
                }

                break;
            }

            // 急停、目标位置切换尽在自动模式下生效
            if (pTask->nOperationMode == 1) {
                // 急停
                if (!pTask->bIsQuickStopping && bQuickStop) {
                    printf("[%s]Quick Stop.", pTask->pTaskName);

                    // 避免重复
                    pTask->bIsQuickStopping = 1;

                    SKA_HUTP_QuickStop(&(pTask->stTracePlanner), dCurTime);

                    // 设置<任务执行中无防摇急停>警告标志
                    pOutput->stWarnBits.stBits.bQuickStopInExecution = 1;
                }
                
                // 目标位置切换
                if (!pTask->bIsQuickStopping &&
                    fabs(pInput->dTargetRopeLength - pTask->dTaskTargetPos) > SKA_FlOAT_ERROR) {
                    
                    printf("[%s]Changing New Target Rope Length: %.3fm.\n", pTask->pTaskName, pInput->dTargetRopeLength);

                    SKA_HUTP_ModifyTargetPos(&(pTask->stTracePlanner), dCurTime, pInput->dTargetRopeLength);

                    printf("[%s]Changed New Target Rope Length: %.3fm.\n", pTask->pTaskName, pInput->dTargetRopeLength);

                    pTask->dTaskTargetPos = pInput->dTargetRopeLength;

                    pOutput->dTargetRopeLength = pTask->dTaskTargetPos;
                    
                    // 设置<任务执行中更新目标位置>警告标志
                    pOutput->stWarnBits.stBits.bChangeTargetRopeLenInExecution = 1;
                }

                if (!pTask->bIsQuickStopping) {
                    // 预限位减速功能在急停时无效
                    if (pTask->bIsPreLimiting != bPrelimit) {
                        pTask->bIsPreLimiting = bPrelimit;
                        double dMaxVel;
                        SKA_Min(pTask->stParam.dMaxRefSpeed, pInput->dSpeedLimit, &dMaxVel);
                        if (bPrelimit) {
                            // 预限位减速
                            SKA_Min(pTask->stParam.dPrelimitSpeed, dMaxVel, &dMaxVel);
                        } else {
                            // 取消预限位减速
                            ;
                        }
                        // 重新规划轨迹
                        SKA_HUTP_ModifyMaxVelocity(&(pTask->stTracePlanner), dCurTime, dMaxVel);
                    }
                }
            }

            // 设置<任务执行中预限位减速>警告标志
            if (bPrelimit) {
                pOutput->stWarnBits.stBits.bPrelimitlInExecution = 1;
            }

            /* 执行轨迹跟踪控制算法 */ 
                
            double dRefX, dRefDx, dRefDdx;

            if (pTask->nOperationMode == 0) {
                // 手动模式
                dRefDx = pInput->dManualFreq * 
                         pTask->stParam.dMaxOutSpeed / pTask->stParam.dMaxOutFreq;
            }

            if (pTask->nOperationMode == 1) {
                // 轨迹规划器仅在自动模式下生效

                /* 轨迹规划 */
                SKA_HUTP_Run(&(pTask->stTracePlanner), dCurTime, &dRefX, &dRefDx, &dRefDdx);
            }

            /* 跟踪控制 */

            if (pTask->nOperationMode == 0) {
                // 手动模式下的跟踪控制

                double dOutputFreq, dOutputVel;
                int8_t nDirection;
                SKA_HUTC_Run(&(pTask->stTrackCtrler),
                            0.0, dRefDx, 0.0, 0.0,
                            &dOutputVel, &dOutputFreq, &nDirection);

                if (nDirection == 1) {
                    pOutput->stStateBits.stBits.bIsPositive = 1;
                    pOutput->stStateBits.stBits.bIsNegative = 0;
                } else if (nDirection == -1) {
                    pOutput->stStateBits.stBits.bIsPositive = 0;
                    pOutput->stStateBits.stBits.bIsNegative = 1;
                } else {
                    pOutput->stStateBits.stBits.bIsPositive = 0;
                    pOutput->stStateBits.stBits.bIsNegative = 0;
                }
                pOutput->dOutputFreq = dOutputFreq;
                pOutput->dOutputSpeed = dOutputVel;

            } else if (pTask->nOperationMode == 1) {
                // 自动模式下的跟踪控制

                // 急停时停用闭环定位控制
                if (pTask->bIsQuickStopping) {
                    pTask->stTrackCtrler.bActivePosCtrl = 0;
                }

                // 积分分离
                double dOpenIntegThreshold = 1.0;
                double dTaskTargetPos = pTask->dTaskTargetPos;
                double dCurPos = pGeneralInput->dCurRopeLength;
                if(fabs(dTaskTargetPos - dCurPos) < dOpenIntegThreshold){
                    pTask->stTrackCtrler.bOpenIntegration = 1;
                }else{
                    pTask->stTrackCtrler.bOpenIntegration = 0;
                }

                double dOutputFreq, dOutputVel;
                int8_t nDirection;
                SKA_HUTC_Run(&(pTask->stTrackCtrler),
                            dRefX, dRefDx, dRefDdx, dCurPos,
                            &dOutputVel, &dOutputFreq, &nDirection);
                
                // 设置<跟踪误差过大>错误标志
                // <跟踪误差过大>错误标志只在自动模式下激活定位控制时有效
                if (pTask->stTrackCtrler.bActivePosCtrl == 1) {
                    bool bIsTrackErrTooLarge = pOutput->stErrorBits.stBits.bIsTrackErrTooLarge;
                    double dPosTrackErr;
                    SKA_HUTC_GetPosTrackError(&(pTask->stTrackCtrler), &dPosTrackErr);
                    if (!bIsTrackErrTooLarge && fabs(dPosTrackErr) > pTask->stParam.dMaxTrackErr) {
                        printf("[%s]ERROR: Track Error Too Large.\n", pTask->pTaskName);
                        pOutput->stErrorBits.stBits.bIsTrackErrTooLarge = 1;
                        // 错误处理逻辑
                        // 急停
                        if (!pTask->bIsQuickStopping) {
                            // 避免重复
                            pTask->bIsQuickStopping = 1;
                            SKA_HUTP_QuickStop(&(pTask->stTracePlanner), dCurTime);
                        }
                    }
                }

                if (nDirection == 1) {
                    pOutput->stStateBits.stBits.bIsPositive = 1;
                    pOutput->stStateBits.stBits.bIsNegative = 0;
                } else if (nDirection == -1) {
                    pOutput->stStateBits.stBits.bIsPositive = 0;
                    pOutput->stStateBits.stBits.bIsNegative = 1;
                } else {
                    pOutput->stStateBits.stBits.bIsPositive = 0;
                    pOutput->stStateBits.stBits.bIsNegative = 0;
                }
                pOutput->dOutputFreq = dOutputFreq;
                pOutput->dOutputSpeed = dOutputVel;
                pOutput->dOriTrackErr = pTask->stTrackCtrler.dOriTrackError;
                pOutput->dFilTrackErr = pTask->stTrackCtrler.dFilTrackError;

            } else {
                return -1;
            }

            // 设置<无法驱动>错误标志
            if (pTask->bHasBeenBrakeOpened && !pOutput->stErrorBits.stBits.bCannotDriveByLowFreq) {
                bool bCannotDrive;
                SKA_CDED_Run(&(pTask->stCDED), pGeneralInput->dCurRopeLength, 
                                pOutput->dOutputFreq, &bCannotDrive);
                if (bCannotDrive) {
                    printf("[%s]ERROR: Cannot Drive.\n", pTask->pTaskName);
                    pOutput->stErrorBits.stBits.bCannotDriveByLowFreq = 1;
                }
            }
            
            /* 任务完成条件 */

            // 零手动输入条件
            bool bIsZeroManual;
            if (pTask->nOperationMode == 0) {
                SKA_ZMD_Run(&(pTask->stZMD), pInput->dManualFreq, &bIsZeroManual);
            } else if (pTask->nOperationMode == 1) {
                // 零手动输入条件仅在手动模式下生效，自动模式下视为满足条件
                bIsZeroManual = 1;
            } else {
                return -1;
            }
            pOutput->stStateBits.stBits.bIsZeroManual = bIsZeroManual;

            // 定位误差条件
            bool bIsPosErrOk;
            if (pTask->nOperationMode == 0) {
                // 手动模式下定位功能无效, 视为定位误差满足条件
                bIsPosErrOk = 1;
            } else if (pTask->nOperationMode == 1) {
                double dTaskTargetPos = pTask->dTaskTargetPos;
                double dCurPos = pGeneralInput->dCurRopeLength;
                double dMaxPosErr = pTask->stParam.dMaxPosErr;
                // 急停状态下目标位置无效，视为满足定位误差条件
                if (fabs(dTaskTargetPos - dCurPos) < dMaxPosErr || pTask->bIsQuickStopping) {
                    bIsPosErrOk = 1;
                } else {
                    bIsPosErrOk = 0;
                }
            } else {
                return -1;
            }
            pOutput->stStateBits.stBits.bIsPosErrOk = bIsPosErrOk;

            // 零速度条件
            bool bIsZeroSpeed;
            double dCurPos = pGeneralInput->dCurRopeLength;
            SKA_ZSD_Run(&(pTask->stZSD), dCurPos, &bIsZeroSpeed);
            pOutput->stStateBits.stBits.bIsZeroSpeed = bIsZeroSpeed;

            // 任务完成条件判断
            if (bIsZeroManual && bIsPosErrOk && bIsZeroSpeed) {
                // 任务完成

                // 设置输出
                pOutput->stStateBits.stBits.bIsPositive = 0;
                pOutput->stStateBits.stBits.bIsNegative = 0;
                pOutput->dOutputFreq = 0.0;
                pOutput->dOutputSpeed = 0.0;
                
                // 状态转换
                pTask->eTaskState = SKA_ASC_FINISH_TASK_STATE;
            }
            
            break;
        }
        case SKA_ASC_FINISH_TASK_STATE: {
            // 完成状态

            printf("[%s]Task Finished.\n", pTask->pTaskName);

            /* 设置输出 */
            pOutput->stStateBits.stBits.bIsPositive = 0;
            pOutput->stStateBits.stBits.bIsNegative = 0;
            pOutput->stStateBits.stBits.bIsBusy = 0;
            pOutput->stStateBits.stBits.bIsPosErrOk = 0;
            pOutput->dOutputFreq = 0.0;
            pOutput->dOutputSpeed = 0.0;
            pOutput->dOriTrackErr = 0.0;
            pOutput->dFilTrackErr = 0.0;

            /* 清理操作 */

            if (pTask->nOperationMode == 0) {
                // 手动模式独有的模块

                // 零手动输入检测器
                SKA_ZMD_Destroy(&(pTask->stZMD));

            } else if (pTask->nOperationMode == 1) {
                // 自动模式独有的模块

                // 轨迹规划器
                SKA_HUTP_Destroy(&(pTask->stTracePlanner));
            } else {
                return -1;
            }

            // 零速度检测器
            SKA_ZSD_Destroy(&(pTask->stZSD));

            /* 无法驱动错误检测器 */
            SKA_CDED_Destroy(&(pTask->stCDED));

            /* 状态转换 */
            pTask->eTaskState = SKA_ASC_FREE_TASK_STATE;

            break;
        }
        default: {
            return -1;
        }
        }
    } else {
        // 控制器未使能

        // 设置输出
        pOutput->stStateBits.nData = 0;
        pOutput->stErrorBits.nData = 0;
        pOutput->stWarnBits.nData = 0;
        pOutput->dOutputFreq = 0.0;
        pOutput->dOutputSpeed = 0.0;
        pOutput->dTargetRopeLength = 0.0;
        pOutput->dOriTrackErr = 0.0;
        pOutput->dFilTrackErr = 0.0;

        // 控制器复位
        pTask->eTaskState = SKA_ASC_FREE_TASK_STATE;
    }

    return 0;
}