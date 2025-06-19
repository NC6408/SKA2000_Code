/**
 ******************************************************************************
 * @file       : SKA_VerticalAvoidanceTask.c
 * @brief      : 纵向避障任务
 * @author     : ShiRong
 * @version    : None
 * @date       : 2024/12/10
 ******************************************************************************
 */
//

#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include "SKA_AntiswayController.h"

int8_t SKA_ASC_VerticalAvoidanceTaskStateTransition(double dCtrlPeriod,
                                                    SKA_VerticalAvoidanceTask *pTask,
                                                    SKA_AscVerticalAvoidanceInput *pstVerticalInput,
                                                    SKA_AscTravelInput *pBridgeInput,
                                                    SKA_AscTravelInput *pTrolleyInput,
                                                    SKA_AscHoistInput *pHoistInput,
                                                    SKA_AscGeneralInput *pstGeneralInput,
                                                    SKA_AscTravelParam *pBridgeParam,
                                                    SKA_AscTravelParam *pTrolleyParam,
                                                    SKA_AscHoistParam *pHoistParam,
                                                    SKA_AscGeneralOutput *pGeneralOutput,
                                                    SKA_AscTravelOutput *pBridgeOutput,
                                                    SKA_AscTravelOutput *pTrolleyOutput,
                                                    SKA_AscHoistOutput *pHoistOutput,
                                                    SKA_AscGeneralInputCmdParser *pCmdParser)

{
    /******************** 函数参数合法性检验 ********************/
    if (pTask == NULL || dCtrlPeriod <= 0)
    {
        printf("Invalid parameter of SKA_ASC_VerticalAvoidanceTaskStateTransition().\n");
        return -1;
    }

    /******************** 状态转换 ********************/

    // 使能指令 (激活纵向避障模式)
    bool bEnable = pstGeneralInput->stCtrlBits.stBits.bActiveVerticalMode;

    // 启动指令
    bool bHasNewTask = pCmdParser->bStartVerticalTask;

    // 复位指令
    bool bReset = pstGeneralInput->stCtrlBits.stBits.bReset;

    bool bIsMaintainMinRopeLength = pstGeneralInput->stCtrlBits.stBits.bIsMaintainMinRopeLength;

    if (bEnable)
    {
        // 激活纵向避障模式

        // 大小车有效绳长反馈
        double dBridgeEffRopeLen = pBridgeInput->dEffRopeLength;
        pBridgeOutput->dEffRopeLen = dBridgeEffRopeLen;
        double dTrolleyEffRopeLen = pTrolleyInput->dEffRopeLength;
        pTrolleyOutput->dEffRopeLen = dTrolleyEffRopeLen;

        switch (pTask->eTaskState)
        {
        case SKA_ASC_FREE_TASK_STATE:
        {
            // 空闲状态
            printf("VerticalTask: SKA_ASC_FREE_TASK_STATE\n");
            // 设置大车输出
            pBridgeOutput->stStateBits.stBits.bIsPositive = 0;
            pBridgeOutput->stStateBits.stBits.bIsNegative = 0;
            pBridgeOutput->stStateBits.stBits.bIsBusy = 0;
            pBridgeOutput->stStateBits.stBits.bOpenBrake = 0;
            pBridgeOutput->stStateBits.stBits.bIsZeroManual = 0;
            pBridgeOutput->stStateBits.stBits.bIsPosErrOk = 0;
            pBridgeOutput->stStateBits.stBits.bIsZeroSpeed = 0;
            pBridgeOutput->stStateBits.stBits.bIsSwayAngleOk = 0;
            pBridgeOutput->dOutputFreq = 0.0;
            pBridgeOutput->dOutputSpeed = 0.0;
            pBridgeOutput->dBrakeDist = 0.0;
            pBridgeOutput->dOriTrackErr = 0.0;
            pBridgeOutput->dFilTrackErr = 0.0;
            // 设置小车输出
            pTrolleyOutput->stStateBits.stBits.bIsPositive = 0;
            pTrolleyOutput->stStateBits.stBits.bIsNegative = 0;
            pTrolleyOutput->stStateBits.stBits.bIsBusy = 0;
            pTrolleyOutput->stStateBits.stBits.bOpenBrake = 0;
            pTrolleyOutput->stStateBits.stBits.bIsZeroManual = 0;
            pTrolleyOutput->stStateBits.stBits.bIsPosErrOk = 0;
            pTrolleyOutput->stStateBits.stBits.bIsZeroSpeed = 0;
            pTrolleyOutput->stStateBits.stBits.bIsSwayAngleOk = 0;
            pTrolleyOutput->dOutputFreq = 0.0;
            pTrolleyOutput->dOutputSpeed = 0.0;
            pTrolleyOutput->dBrakeDist = 0.0;
            pTrolleyOutput->dOriTrackErr = 0.0;
            pTrolleyOutput->dFilTrackErr = 0.0;
            // 设置起升输出
            pHoistOutput->stStateBits.stBits.bIsPositive = 0;
            pHoistOutput->stStateBits.stBits.bIsNegative = 0;
            pHoistOutput->stStateBits.stBits.bIsBusy = 0;
            pHoistOutput->stStateBits.stBits.bIsZeroManual = 0;
            pHoistOutput->stStateBits.stBits.bIsPosErrOk = 0;
            pHoistOutput->stStateBits.stBits.bIsZeroSpeed = 0;
            pHoistOutput->dOutputFreq = 0.0;
            pHoistOutput->dOutputSpeed = 0.0;
            pHoistOutput->dOriTrackErr = 0.0;
            pHoistOutput->dFilTrackErr = 0.0;

            if (bReset) {
                // 复位          
                break;               
            }            

            bool bStartNewTask = 0;
            // 自动模式
            if (bHasNewTask)
            {
                // 启动新任务
                bStartNewTask = 1;

                // 记录大小车、起升任务的开始位置和目标位置
                pTask->dBridgeTaskStartPos = pBridgeInput->dCurPos;
                pTask->dBridgeTaskTargetPos = pBridgeInput->dTargetPos;
                pTask->dTrolleyTaskStartPos = pTrolleyInput->dCurPos;
                pTask->dTrolleyTaskTargetPos = pTrolleyInput->dTargetPos;
                pTask->dHoistTaskStartRopeLength = pstGeneralInput->dCurRopeLength;
                pTask->dHoistTaskTargetRopeLength = pHoistInput->dTargetRopeLength;

                // 目标位置反馈
                pBridgeOutput->dTargetPos = pTask->dBridgeTaskTargetPos;
                pTrolleyOutput->dTargetPos = pTask->dTrolleyTaskTargetPos;
                pHoistOutput->dTargetRopeLength = pTask->dHoistTaskTargetRopeLength;
                printf("[%s]Vertical Start A New Auto BridgeTask. BridgeStartPos = %lfm, BridgeTargetPos = %lfm.\n",
                       pTask->pTaskName, pTask->dBridgeTaskStartPos, pTask->dBridgeTaskTargetPos);
                printf("[%s]Vertical Start A New Auto TrolleyTask. TrolleyStartPos = %lfm, TrolleyTargetPos = %lfm.\n",
                       pTask->pTaskName, pTask->dTrolleyTaskStartPos, pTask->dTrolleyTaskTargetPos);
                printf("[%s]Vertical Start A New Auto HoistTask. HoistStartPos = %lfm, HoistTargetPos = %lfm.\n",
                       pTask->pTaskName, pTask->dHoistTaskStartRopeLength, pTask->dHoistTaskTargetRopeLength);
            }

            if (bStartNewTask)
            {
                // 拷贝参数
                pTask->stBridgeParam = *pBridgeParam;
                pTask->stTrolleyParam = *pTrolleyParam;
                pTask->stHoistParam = *pHoistParam;
                // 状态转换
                pTask->eTaskState = SKA_ASC_PLAN_TASK_STATE;
            }
            break;
        }
        case SKA_ASC_PLAN_TASK_STATE:
        {
            // 准备状态
            
            // 消除<跟踪误差过大>错误标志
            pBridgeOutput->stErrorBits.stBits.bIsTrackErrTooLarge = 0;
            pTrolleyOutput->stErrorBits.stBits.bIsTrackErrTooLarge = 0;
            pHoistOutput->stErrorBits.stBits.bIsTrackErrTooLarge = 0;
            
            // 消除<开闸时间过长>错误标志
            pBridgeOutput->stErrorBits.stBits.bIsBrakeOpenTimeTooLong = 0;
            pTrolleyOutput->stErrorBits.stBits.bIsBrakeOpenTimeTooLong = 0;
            pHoistOutput->stErrorBits.stBits.bIsBrakeOpenTimeTooLong = 0;

            // 消除<低频不能驱动>错误标志
            pBridgeOutput->stErrorBits.stBits.bCannotDriveByLowFreq = 0;
            pTrolleyOutput->stErrorBits.stBits.bCannotDriveByLowFreq = 0;
            pHoistOutput->stErrorBits.stBits.bCannotDriveByLowFreq = 0;

            // 消除<任务执行中复位>警告标志
            pGeneralOutput->stWarnBits.stBits.bResetInExecution = 0;

            // 设置<未激活防摇控制>警告标志
            if (pBridgeInput->stCtrlBits.stBits.bActiveSwayCtrl == 0) {
                pBridgeOutput->stWarnBits.stBits.bIsSwayCtrlDeactivated = 1;
            } else {
                pBridgeOutput->stWarnBits.stBits.bIsSwayCtrlDeactivated = 0;
            }
            if (pTrolleyInput->stCtrlBits.stBits.bActiveSwayCtrl == 0) {
                pTrolleyOutput->stWarnBits.stBits.bIsSwayCtrlDeactivated = 1;
            } else {
                pTrolleyOutput->stWarnBits.stBits.bIsSwayCtrlDeactivated = 0;
            }
            
            // 设置<未激活定位控制>警告标志
                // 仅在自动模式下生效
                if (pBridgeInput->stCtrlBits.stBits.bActivePosCtrl == 0) {
                    pBridgeOutput->stWarnBits.stBits.bIsPosCtrlDeactivated = 1;
                } else {
                    pBridgeOutput->stWarnBits.stBits.bIsPosCtrlDeactivated = 0;
                }
                if (pTrolleyInput->stCtrlBits.stBits.bActivePosCtrl == 0) {
                    pTrolleyOutput->stWarnBits.stBits.bIsPosCtrlDeactivated = 1;
                } else {
                    pTrolleyOutput->stWarnBits.stBits.bIsPosCtrlDeactivated = 0;
                }
                if (pHoistInput->stCtrlBits.stBits.bActivePosCtrl == 0) {
                    pHoistOutput->stWarnBits.stBits.bIsPosCtrlDeactivated = 1;
                } else {
                    pHoistOutput->stWarnBits.stBits.bIsPosCtrlDeactivated = 0;
                }

            // 复位
            if (bReset) {

                printf("[%s]Reset.", pTask->pTaskName);

                // 状态转换
                pTask->eTaskState = SKA_ASC_FREE_TASK_STATE;

                // 设置<任务执行中复位>警告标志
                pGeneralOutput->stWarnBits.stBits.bResetInExecution = 1;
                printf("stWarnBits bResetInExecution: %d\n", pGeneralOutput->stWarnBits.stBits.bResetInExecution);

                break;
            }

            // 存在输入错误
            if (pTask->bHasInputErrors) {
                // 状态转换
                pTask->eTaskState = SKA_ASC_FREE_TASK_STATE;

                break;
            }

            // 控制器忙碌
            pBridgeOutput->stStateBits.stBits.bIsBusy = 1;
            pTrolleyOutput->stStateBits.stBits.bIsBusy = 1;
            pHoistOutput->stStateBits.stBits.bIsBusy = 1;

            // 初始化

            /* 抱闸已开启过标志 */
            pTask->bHasBeenBrakeOpened = 0;

            // 大车输入整形器
            SKA_InputShaperType BridgeeIsType;
            switch (pTask->stBridgeParam.nAsLevel)
            {
            case 0:
                BridgeeIsType = SKA_IS_ZV;
                break;
            case 1:
                BridgeeIsType = SKA_IS_ZVD;
                break;
            case 2:
                BridgeeIsType = SKA_IS_ZVDD;
                break;
            default:
                return -1;
            }
            // 小车输入整形器
            SKA_InputShaperType TrolleyeIsType;
            switch (pTask->stTrolleyParam.nAsLevel)
            {
            case 0:
                TrolleyeIsType = SKA_IS_ZV;
                break;
            case 1:
                TrolleyeIsType = SKA_IS_ZVD;
                break;
            case 2:
                TrolleyeIsType = SKA_IS_ZVDD;
                break;
            default:
                return -1;
            }
            /* 大车轨迹规划器参数 */
            bool bBridgeActiveSwayCtrl = pBridgeInput->stCtrlBits.stBits.bActiveSwayCtrl;
            double dBridgeWn = sqrt(SKA_G / dBridgeEffRopeLen);
            double dBridgeZeta = 0;
            double dBridgeStartPos = pTask->dBridgeTaskStartPos;
            double dBridgeTargetPos = pTask->dBridgeTaskTargetPos;
            double dBridgeQuickStopMaxAcc = pTask->stBridgeParam.dMaxOutAcc;
            /* 小车轨迹规划器参数 */
            bool bTrolleyActiveSwayCtrl = pTrolleyInput->stCtrlBits.stBits.bActiveSwayCtrl;
            double dTrolleyWn = sqrt(SKA_G / dTrolleyEffRopeLen);
            double dTrolleyZeta = 0;
            double dTrolleyStartPos = pTask->dTrolleyTaskStartPos;
            double dTrolleyTargetPos = pTask->dTrolleyTaskTargetPos;
            double dTrolleyQuickStopMaxAcc = pTask->stTrolleyParam.dMaxOutAcc;
            /* 起升机构轨迹规划器参数 */
            double dHoistStartRopeLength = pTask->dHoistTaskStartRopeLength;
            double dHoistTargetRopeLength = pTask->dHoistTaskTargetRopeLength;
            double dHoistMaxAcc = pTask->stHoistParam.dMaxRefAcc;
            double dHoistMaxVel = pTask->stHoistParam.dMaxRefSpeed;
            double dHoistMaxJerk = 5.0 * pow(dHoistMaxAcc, 2) / dHoistMaxVel;
           
            // 最短绳长，即起升机构的最大高度
            pTask->dHoistMinRopeLength = pstVerticalInput->dTotalHeight - pstVerticalInput->arrLiftTerrain[pstVerticalInput->nLiftArraySize - 1].dHeight;
            // 判断最短绳长
            if ((dHoistStartRopeLength > pTask->dHoistMinRopeLength) && (dHoistTargetRopeLength < pTask->dHoistMinRopeLength))
            {
                // 如果初始高度小于障碍物最高点，且目标高度高于障碍物最高点(为绳长关系)
                pTask->dHoistMinRopeLength = dHoistTargetRopeLength;
            }
            else if (dHoistStartRopeLength < pTask->dHoistMinRopeLength && dHoistTargetRopeLength < pTask->dHoistMinRopeLength)
            {
                // 如果初始高度高于障碍物最高点，且目标高度高于障碍物最高点
                pTask->dHoistMinRopeLength = dHoistTargetRopeLength;
            }
            else if (dHoistStartRopeLength < pTask->dHoistMinRopeLength && dHoistTargetRopeLength > pTask->dHoistMinRopeLength)
            {
                // 如果初始高度大于障碍物最高点，且目标高度小于障碍物最高点
                if (bIsMaintainMinRopeLength)
                {
                    pTask->dHoistMinRopeLength = dHoistStartRopeLength;
                    dHoistTargetRopeLength = pTask->dHoistMinRopeLength;
                }
                else
                {
                    pTask->dHoistMinRopeLength = dHoistStartRopeLength;
                }
            }
            else
            {
                // 一般情况，翻越障碍物
                if (bIsMaintainMinRopeLength)
                {
                    dHoistTargetRopeLength = pTask->dHoistMinRopeLength;
                }
                else
                {
                    ;
                }
            }

            /***** 创建运行机构和起升机构轨迹规划器 ******/
            SKA_VCFTP_Create(&(pTask->stVCFTracePlanner), bBridgeActiveSwayCtrl, BridgeeIsType, dBridgeWn, dBridgeZeta,
                             dBridgeStartPos, dBridgeTargetPos, pTask->stBridgeParam.dMaxRefSpeed, pTask->stBridgeParam.dMaxRefAcc, dBridgeQuickStopMaxAcc,
                             bTrolleyActiveSwayCtrl, TrolleyeIsType, dTrolleyWn, dTrolleyZeta, dTrolleyStartPos, dTrolleyTargetPos, pTask->stTrolleyParam.dMaxRefSpeed,
                             pTask->stTrolleyParam.dMaxRefAcc, dTrolleyQuickStopMaxAcc, dHoistStartRopeLength, pTask->dHoistMinRopeLength, dHoistTargetRopeLength,
                             dHoistMaxVel, dHoistMaxAcc, dHoistMaxJerk);

            pTask->stVCFTracePlanner.dTotalHeight = pstVerticalInput->dTotalHeight;
            pTask->stVCFTracePlanner.nLiftArraySize = pstVerticalInput->nLiftArraySize;
            pTask->stVCFTracePlanner.nDropArraySize = pstVerticalInput->nDropArraySize;
            memcpy(pTask->stVCFTracePlanner.arrLiftTerrain, pstVerticalInput->arrLiftTerrain, sizeof(pstVerticalInput->arrLiftTerrain));
            memcpy(pTask->stVCFTracePlanner.arrDropTerrain, pstVerticalInput->arrDropTerrain, sizeof(pstVerticalInput->arrDropTerrain));

            // 计算起升机构最大延迟时间和起升机构最大开始下降时间
            SKA_VCFTP_TravelMaxDelayTimeAndHoistMaxStartDownTime(&(pTask->stVCFTracePlanner), &(pTask->dTravelUnitMaxDelayTime), &(pTask->dHoistUnitMaxStartDownTime));

            /* 大车跟踪控制器 */
            bool bBridgeActiveClosedPosCtrl;
            bBridgeActiveClosedPosCtrl = pBridgeInput->stCtrlBits.stBits.bActivePosCtrl;
            double dBridgeTs = dCtrlPeriod;
            double dBridgeDelayTime = pTask->stBridgeParam.dDelayTime;
            double dBridgeMaxTrackErr = pTask->stBridgeParam.dTrackErrLim;
            double dBridgeMaxTrackErrRate = pTask->stBridgeParam.dTrackErrRateLim;
            double dBridgeTrackErrFilTc = pTask->stBridgeParam.dTrackErrFilTc;
            double dBridgeKp = pTask->stBridgeParam.dPosTrackKp;
            double dBridgeKi = pTask->stBridgeParam.dPosTrackKi;
            double dBridgeMaxVi = 0.1;
            double dBridgePosDeadZone = pTask->stBridgeParam.dMaxPosErr / 2;
            double dBridgeMaxVel = pTask->stBridgeParam.dMaxOutSpeed;
            double dBridgeMaxAcc = pTask->stBridgeParam.dMaxOutAcc;
            double dBridgeMaxFreq = pTask->stBridgeParam.dMaxOutFreq;
            double dBridgeMinFreq = pTask->stBridgeParam.dMinMoveFreq;
            double dBridgeFreqCompenSoft = 0.5;
            SKA_TUTC_Init(&(pTask->stBridgeTrackCtrler), bBridgeActiveClosedPosCtrl, dBridgeTs, dBridgeDelayTime,
                          dBridgeStartPos, dBridgeMaxTrackErr, dBridgeMaxTrackErrRate, dBridgeTrackErrFilTc,
                          dBridgeKp, dBridgeKi, dBridgeMaxVi, dBridgePosDeadZone, dBridgeMaxVel, dBridgeMaxAcc,
                          dBridgeMaxFreq, dBridgeMinFreq, dBridgeFreqCompenSoft);

            /* 小车跟踪控制器 */
            bool bTrolleyActiveClosedPosCtrl;
            bTrolleyActiveClosedPosCtrl = pTrolleyInput->stCtrlBits.stBits.bActivePosCtrl;
            double dTrolleyTs = dCtrlPeriod;
            double dTrolleyDelayTime = pTask->stTrolleyParam.dDelayTime;
            double dTrolleyMaxTrackErr = pTask->stTrolleyParam.dTrackErrLim;
            double dTrolleyMaxTrackErrRate = pTask->stTrolleyParam.dTrackErrRateLim;
            double dTrolleyTrackErrFilTc = pTask->stTrolleyParam.dTrackErrFilTc;
            double dTrolleyKp = pTask->stTrolleyParam.dPosTrackKp;
            double dTrolleyKi = pTask->stTrolleyParam.dPosTrackKi;
            double dTrolleyMaxVi = 0.1;
            double dTrolleyPosDeadZone = pTask->stTrolleyParam.dMaxPosErr / 2;
            double dTrolleyMaxVel = pTask->stTrolleyParam.dMaxOutSpeed;
            double dTrolleyMaxAcc = pTask->stTrolleyParam.dMaxOutAcc;
            double dTrolleyMaxFreq = pTask->stTrolleyParam.dMaxOutFreq;
            double dTrolleyMinFreq = pTask->stTrolleyParam.dMinMoveFreq;
            double dTrolleyFreqCompenSoft = 0.5;
            SKA_TUTC_Init(&(pTask->stTrolleyTrackCtrler), bTrolleyActiveClosedPosCtrl, dTrolleyTs, dTrolleyDelayTime,
                          dTrolleyStartPos, dTrolleyMaxTrackErr, dTrolleyMaxTrackErrRate, dTrolleyTrackErrFilTc,
                          dTrolleyKp, dTrolleyKi, dTrolleyMaxVi, dTrolleyPosDeadZone, dTrolleyMaxVel, dTrolleyMaxAcc,
                          dTrolleyMaxFreq, dTrolleyMinFreq, dTrolleyFreqCompenSoft);

            /* 起升机构跟踪控制器 */
            bool bHoistActiveClosedPosCtrl;
            bHoistActiveClosedPosCtrl = pHoistInput->stCtrlBits.stBits.bActivePosCtrl;
            double dHoistTs = dCtrlPeriod;
            double dHoistDelayTime = pTask->stHoistParam.dDelayTime;
            double dHoistStartPos = pTask->dHoistTaskStartRopeLength;
            double dHoistMaxTrackErr = pTask->stHoistParam.dTrackErrLim;
            double dHoistMaxTrackErrRate = pTask->stHoistParam.dTrackErrRateLim;
            double dHoistTrackErrFilTc = pTask->stHoistParam.dTrackErrFilTc;
            double dHoistKp = pTask->stHoistParam.dPosTrackKp;
            double dHoistKi = pTask->stHoistParam.dPosTrackKi;
            double dHoistMaxVi = 0.1;
            double dHoistPosDeadZone = pTask->stHoistParam.dMaxPosErr / 2;
            double dHoistOutMaxVel = pTask->stHoistParam.dMaxOutSpeed;
            double dHoistOutMaxAcc = pTask->stHoistParam.dMaxOutAcc;
            double dHoistMaxFreq = pTask->stHoistParam.dMaxOutFreq;
            double dHoistMinFreq = pTask->stHoistParam.dMinMoveFreq;
            double dHoistFreqCompenSoft = 0.5;
            SKA_HUTC_Init(&(pTask->stHoistTrackCtrler), bHoistActiveClosedPosCtrl, dHoistTs, dHoistDelayTime,
                          dHoistStartPos, dHoistMaxTrackErr, dHoistMaxTrackErrRate, dHoistTrackErrFilTc,
                          dHoistKp, dHoistKi, dHoistMaxVi, dHoistPosDeadZone, dHoistOutMaxVel, dHoistOutMaxAcc,
                          dHoistMaxFreq, dHoistMinFreq, dHoistFreqCompenSoft);
            /* 计时器 */
            SKA_Timer_Start(&(pTask->stTimer));

            /* 零速度检测器 */
            // 检测时域长度(s)
            double dZeroSpeedTimeLen;
            // 自动模式略长
            dZeroSpeedTimeLen = 1.5;

            SKA_ZSD_Create(&(pTask->stBridgeZSD), dCtrlPeriod, dZeroSpeedTimeLen,
                           pTask->stBridgeParam.dStaticSpeed);
            SKA_ZSD_Create(&(pTask->stTrolleyZSD), dCtrlPeriod, dZeroSpeedTimeLen,
                           pTask->stTrolleyParam.dStaticSpeed);
            /* 无法驱动错误检测器 */
            double dBridgeCannotDriveTimeLen = 2.0;
            double dBridgePosAccuracy = 0.01;
            double dBridgeFreqThreshold = 1.0;
            SKA_CDED_Create(&(pTask->stBridgeCDED), dCtrlPeriod, dBridgeCannotDriveTimeLen, 
                            dBridgePosAccuracy, dBridgeFreqThreshold);
            double dTrolleyCannotDriveTimeLen = 2.0;
            double dTrolleyPosAccuracy = 0.01;
            double dTrolleyFreqThreshold = 1.0;
            SKA_CDED_Create(&(pTask->stTrolleyCDED), dCtrlPeriod, dTrolleyCannotDriveTimeLen, 
                            dTrolleyPosAccuracy, dTrolleyFreqThreshold);
            double dHoistCannotDriveTimeLen = 2.0;
            double dHoistPosAccuracy = 0.01;
            double dHoistFreqThreshold = 1.0;
            SKA_CDED_Create(&(pTask->stHoistCDED), dCtrlPeriod, dHoistCannotDriveTimeLen, 
                            dHoistPosAccuracy, dHoistFreqThreshold);

            // 状态转换
            pTask->eTaskState = SKA_ASC_EXECUTING_TASK_STATE;

            break;
        }
        case SKA_ASC_EXECUTING_TASK_STATE:
        {
            // 执行状态
            // 复位
            if (bReset) {

                printf("[%s]Reset.", pTask->pTaskName);
                
                // 状态转换
                // 需要进入完成状态执行清理工作
                pTask->eTaskState = SKA_ASC_FINISH_TASK_STATE;

                // 设置<任务执行中复位>警告标志
                pGeneralOutput->stWarnBits.stBits.bResetInExecution = 1;

                break;
            }
            
            // 打开抱闸
            pBridgeOutput->stStateBits.stBits.bOpenBrake = 1;
            pTrolleyOutput->stStateBits.stBits.bOpenBrake = 1;

            // 在收到开闸反馈前输出运动方向
            if (!pTask->bHasBeenBrakeOpened)
            {
                if (pTask->dBridgeTaskTargetPos >= pTask->dBridgeTaskStartPos)
                {
                    pBridgeOutput->stStateBits.stBits.bIsPositive = 1;
                    pBridgeOutput->stStateBits.stBits.bIsNegative = 0;
                }
                else
                {
                    pBridgeOutput->stStateBits.stBits.bIsPositive = 0;
                    pBridgeOutput->stStateBits.stBits.bIsNegative = 1;
                }
                if (pTask->dTrolleyTaskTargetPos >= pTask->dTrolleyTaskStartPos)
                {
                    pTrolleyOutput->stStateBits.stBits.bIsPositive = 1;
                    pTrolleyOutput->stStateBits.stBits.bIsNegative = 0;
                }
                else
                {
                    pTrolleyOutput->stStateBits.stBits.bIsPositive = 0;
                    pTrolleyOutput->stStateBits.stBits.bIsNegative = 1;
                }
            }
            // 判断抱闸是否开启
            if (!pTask->bHasBeenBrakeOpened &&
                pBridgeInput->stCtrlBits.stBits.bIsBrakeOpened &&
                pTrolleyInput->stCtrlBits.stBits.bIsBrakeOpened)
            {
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

            // 检测抱闸是否开启
            if (!pTask->bHasBeenBrakeOpened) {
                // 等待抱闸打开

                // 设置<开闸时间过长>错误标志
                double dMaxOpenBrakeTime = 5.0;
                bool bBridgeIsBrakeOpenTimeTooLong = pBridgeOutput->stErrorBits.stBits.bIsBrakeOpenTimeTooLong;
                bool bTrolleyIsBrakeOpenTimeTooLong = pTrolleyOutput->stErrorBits.stBits.bIsBrakeOpenTimeTooLong;
                bool bHoistIsBrakeOpenTimeTooLong = pHoistOutput->stErrorBits.stBits.bIsBrakeOpenTimeTooLong;
                if (!bBridgeIsBrakeOpenTimeTooLong && dCurTime > dMaxOpenBrakeTime) {
                    pBridgeOutput->stErrorBits.stBits.bIsBrakeOpenTimeTooLong = 1;
                    // 错误处理逻辑
                    // 转换为完成状态

                    printf("[%s]ERROR: Bridge Open Brake Time Too Long.\n", pTask->pTaskName);

                    // 设置输出
                    pBridgeOutput->stStateBits.stBits.bOpenBrake = 0;

                    // 状态转换
                    pTask->eTaskState = SKA_ASC_FINISH_TASK_STATE;
                }
                if (!bTrolleyIsBrakeOpenTimeTooLong && dCurTime > dMaxOpenBrakeTime) {
                    pTrolleyOutput->stErrorBits.stBits.bIsBrakeOpenTimeTooLong = 1;
                    // 错误处理逻辑
                    // 转换为完成状态

                    printf("[%s]ERROR: Trolley Open Brake Time Too Long.\n", pTask->pTaskName);

                    // 设置输出
                    pTrolleyOutput->stStateBits.stBits.bOpenBrake = 0;

                    // 状态转换
                    pTask->eTaskState = SKA_ASC_FINISH_TASK_STATE;
                }
                if (!bHoistIsBrakeOpenTimeTooLong && dCurTime > dMaxOpenBrakeTime) {
                    pHoistOutput->stErrorBits.stBits.bIsBrakeOpenTimeTooLong = 1;
                    // 错误处理逻辑
                    // 转换为完成状态

                    printf("[%s]ERROR: Hoist Unit Open Brake Time Too Long.\n", pTask->pTaskName);

                    // 状态转换
                    pTask->eTaskState = SKA_ASC_FINISH_TASK_STATE;
                }

            } else {
                // 抱闸已打开, 执行轨迹跟踪控制算法
                double dBridgeRefX, dBridgeRefDx, dBridgeRefDdx;
                double dTrolleyRefX, dTrolleyRefDx, dTrolleyRefDdx;
                double dHoistRefX, dHoistRefDx, dHoistRefDdx;
                double dBridgeUnsRefx, dBridgeUnsRefDx, dBridgeUnsRefDdx;
                double dTrolleyUnsRefx, dTrolleyUnsRefDx, dTrolleyUnsRefDdx;

                // 轨迹规划器
                SKA_VCFTP_Run(&(pTask->stVCFTracePlanner), dCurTime, pTask->dTravelUnitMaxDelayTime, pTask->dHoistUnitMaxStartDownTime, &dBridgeRefX,
                              &dBridgeRefDx, &dBridgeRefDdx, &dBridgeUnsRefx, &dBridgeUnsRefDx, &dBridgeUnsRefDdx, &dTrolleyRefX, &dTrolleyRefDx,
                              &dTrolleyRefDdx, &dTrolleyUnsRefx, &dTrolleyUnsRefDx, &dTrolleyUnsRefDdx, &dHoistRefX, &dHoistRefDx, &dHoistRefDdx);
                
                /* 跟踪控制 */
                // 大车跟踪控制
                //  积分分离
                double dBridgeOpenIntegThreshold = 1.0;
                double dBridgeTaskTargetPos = pTask->dBridgeTaskTargetPos;
                double dBridgeCurPos = pBridgeInput->dCurPos;
                if (fabs(dBridgeTaskTargetPos - dBridgeCurPos) < dBridgeOpenIntegThreshold)
                {
                    pTask->stBridgeTrackCtrler.bOpenIntegration = 1;
                }
                else
                {
                    pTask->stBridgeTrackCtrler.bOpenIntegration = 0;
                }
                double dBridgeOutputFreq, dBridgeOutputVel;
                int8_t nBridgeDirection;
                SKA_TUTC_Run(&(pTask->stBridgeTrackCtrler),
                             dBridgeRefX, dBridgeRefDx, dBridgeRefDdx, dBridgeCurPos,
                             &dBridgeOutputVel, &dBridgeOutputFreq, &nBridgeDirection);               
                
                if (nBridgeDirection == 1)
                {
                    pBridgeOutput->stStateBits.stBits.bIsPositive = 1;
                    pBridgeOutput->stStateBits.stBits.bIsNegative = 0;
                }
                else if (nBridgeDirection == -1)
                {
                    pBridgeOutput->stStateBits.stBits.bIsPositive = 0;
                    pBridgeOutput->stStateBits.stBits.bIsNegative = 1;
                }
                else
                {
                    pBridgeOutput->stStateBits.stBits.bIsPositive = 0;
                    pBridgeOutput->stStateBits.stBits.bIsNegative = 0;
                }
                pBridgeOutput->dOutputFreq = dBridgeOutputFreq;
                pBridgeOutput->dOutputSpeed = dBridgeOutputVel;
                pBridgeOutput->dOriTrackErr = pTask->stBridgeTrackCtrler.dOriTrackError;
                pBridgeOutput->dFilTrackErr = pTask->stBridgeTrackCtrler.dFilTrackError;
                
                // 设置<无法驱动>错误标志
                if (!pBridgeOutput->stErrorBits.stBits.bCannotDriveByLowFreq) {
                    bool bBridgeCannotDrive;
                    SKA_CDED_Run(&(pTask->stBridgeCDED), pBridgeInput->dCurPos, 
                                    pBridgeOutput->dOutputFreq, &bBridgeCannotDrive);
                    if (bBridgeCannotDrive) {
                        printf("[%s]ERROR: Bridge Cannot Drive.\n", pTask->pTaskName);
                        pBridgeOutput->stErrorBits.stBits.bCannotDriveByLowFreq = 1;
                    }
                }

                // 小车跟踪控制
                //  积分分离
                double dTrolleyOpenIntegThreshold = 1.0;
                double dTrolleyTaskTargetPos = pTask->dTrolleyTaskTargetPos;
                double dTrolleyCurPos = pTrolleyInput->dCurPos;
                if (fabs(dTrolleyTaskTargetPos - dTrolleyCurPos) < dTrolleyOpenIntegThreshold)
                {
                    pTask->stTrolleyTrackCtrler.bOpenIntegration = 1;
                }
                else
                {
                    pTask->stTrolleyTrackCtrler.bOpenIntegration = 0;
                }

                double dTrolleyOutputFreq, dTrolleyOutputVel;
                int8_t nTrolleyDirection;
                SKA_TUTC_Run(&(pTask->stTrolleyTrackCtrler),
                             dTrolleyRefX, dTrolleyRefDx, dTrolleyRefDdx, dTrolleyCurPos,
                             &dTrolleyOutputVel, &dTrolleyOutputFreq, &nTrolleyDirection);

                if (nTrolleyDirection == 1)
                {
                    pTrolleyOutput->stStateBits.stBits.bIsPositive = 1;
                    pTrolleyOutput->stStateBits.stBits.bIsNegative = 0;
                }
                else if (nTrolleyDirection == -1)
                {
                    pTrolleyOutput->stStateBits.stBits.bIsPositive = 0;
                    pTrolleyOutput->stStateBits.stBits.bIsNegative = 1;
                }
                else
                {
                    pTrolleyOutput->stStateBits.stBits.bIsPositive = 0;
                    pTrolleyOutput->stStateBits.stBits.bIsNegative = 0;
                }
                pTrolleyOutput->dOutputFreq = dTrolleyOutputFreq;
                pTrolleyOutput->dOutputSpeed = dTrolleyOutputVel;
                pTrolleyOutput->dOriTrackErr = pTask->stTrolleyTrackCtrler.dOriTrackError;
                pTrolleyOutput->dFilTrackErr = pTask->stTrolleyTrackCtrler.dFilTrackError;

                // 设置<无法驱动>错误标志
                if (!pTrolleyOutput->stErrorBits.stBits.bCannotDriveByLowFreq) {
                    bool bTrolleyCannotDrive;
                    SKA_CDED_Run(&(pTask->stTrolleyCDED), pTrolleyInput->dCurPos, 
                                    pTrolleyOutput->dOutputFreq, &bTrolleyCannotDrive);
                    if (bTrolleyCannotDrive) {
                        printf("[%s]ERROR: Trolley Cannot Drive.\n", pTask->pTaskName);
                        pTrolleyOutput->stErrorBits.stBits.bCannotDriveByLowFreq = 1;
                    }
                }

                // 起升机构跟踪控制
                //  积分分离
                double dHoistOpenIntegThreshold = 1.0;
                double dHoistTaskTargetRopeLength = pTask->dHoistTaskTargetRopeLength;
                double dHoistCurPos = pstGeneralInput->dCurRopeLength;
                if (fabs(dHoistTaskTargetRopeLength - dHoistCurPos) < dHoistOpenIntegThreshold)
                {
                    pTask->stHoistTrackCtrler.bOpenIntegration = 1;
                }
                else
                {
                    pTask->stHoistTrackCtrler.bOpenIntegration = 0;
                }

                double dHoistOutputFreq, dHoistOutputVel;
                int8_t nHoistDirection;
                SKA_HUTC_Run(&(pTask->stHoistTrackCtrler),
                             dHoistRefX, dHoistRefDx, dHoistRefDdx, dHoistCurPos,
                             &dHoistOutputVel, &dHoistOutputFreq, &nHoistDirection);

                if (nHoistDirection == 1)
                {
                    pHoistOutput->stStateBits.stBits.bIsPositive = 1;
                    pHoistOutput->stStateBits.stBits.bIsNegative = 0;
                }
                else if (nHoistDirection == -1)
                {
                    pHoistOutput->stStateBits.stBits.bIsPositive = 0;
                    pHoistOutput->stStateBits.stBits.bIsNegative = 1;
                }
                else
                {
                    pHoistOutput->stStateBits.stBits.bIsPositive = 0;
                    pHoistOutput->stStateBits.stBits.bIsNegative = 0;
                }
                pHoistOutput->dOutputFreq = dHoistOutputFreq;
                pHoistOutput->dOutputSpeed = dHoistOutputVel;
                pHoistOutput->dOriTrackErr = pTask->stHoistTrackCtrler.dOriTrackError;
                pHoistOutput->dFilTrackErr = pTask->stHoistTrackCtrler.dFilTrackError;

                // 设置<无法驱动>错误标志
                if (!pHoistOutput->stErrorBits.stBits.bCannotDriveByLowFreq) {
                    bool bHoistCannotDrive;
                    SKA_CDED_Run(&(pTask->stHoistCDED), pstGeneralInput->dCurRopeLength, 
                                    pHoistOutput->dOutputFreq, &bHoistCannotDrive);
                    if (bHoistCannotDrive) {
                        printf("[%s]ERROR: Hoist Cannot Drive.\n", pTask->pTaskName);
                        pHoistOutput->stErrorBits.stBits.bCannotDriveByLowFreq = 1;
                    }
                }
            }

            /* 任务完成条件 */
            // 定位误差条件
            bool bIsBridgePosErrOk, bIsTrolleyPosErrOk, bIsHoistPosErrOk;
            double dBridgeTaskTargetPos = pTask->dBridgeTaskTargetPos;
            double dBridgeCurPos = pBridgeInput->dCurPos;
            double dBridgeMaxPosErr = pTask->stBridgeParam.dMaxPosErr;
            double dTrolleyTaskTargetPos = pTask->dTrolleyTaskTargetPos;
            double dTrolleyCurPos = pTrolleyInput->dCurPos;
            double dTrolleyMaxPosErr = pTask->stTrolleyParam.dMaxPosErr;
            double dHoistTaskTargetRopeLength;
            double dHoistCurPos = pstGeneralInput->dCurRopeLength;
            double dHoistMaxPosErr = pTask->stHoistParam.dMaxPosErr;
            if (bIsMaintainMinRopeLength){
                dHoistTaskTargetRopeLength = pTask->dHoistMinRopeLength;
            } else{
                dHoistTaskTargetRopeLength = pTask->dHoistTaskTargetRopeLength;
            }
            
            if (fabs(dBridgeTaskTargetPos - dBridgeCurPos) < dBridgeMaxPosErr)
            {
                bIsBridgePosErrOk = 1;
            }
            else
            {
                bIsBridgePosErrOk = 0;
            }
            if (fabs(dTrolleyTaskTargetPos - dTrolleyCurPos) < dTrolleyMaxPosErr)
            {
                bIsTrolleyPosErrOk = 1;
            }
            else
            {
                bIsTrolleyPosErrOk = 0;
            }            
            if (fabs(dHoistTaskTargetRopeLength - dHoistCurPos) < dHoistMaxPosErr)
            {
                bIsHoistPosErrOk = 1;               
            }
            else
            {
                bIsHoistPosErrOk = 0;
            }
            
            
            // 零速度条件
            bool bIsBridgeZeroSpeed;
            SKA_ZSD_Run(&(pTask->stBridgeZSD), dBridgeCurPos, &bIsBridgeZeroSpeed);
            pBridgeOutput->stStateBits.stBits.bIsZeroSpeed = bIsBridgeZeroSpeed;

            bool bIsTrolleyZeroSpeed;
            SKA_ZSD_Run(&(pTask->stTrolleyZSD), dTrolleyCurPos, &bIsTrolleyZeroSpeed);
            pTrolleyOutput->stStateBits.stBits.bIsZeroSpeed = bIsTrolleyZeroSpeed;

            // 任务完成条件判断
            if (bIsBridgePosErrOk == 1 && bIsBridgeZeroSpeed &&
                bIsTrolleyPosErrOk == 1 && bIsTrolleyZeroSpeed)
            {
                // 大车输出
                pBridgeOutput->stStateBits.stBits.bIsPositive = 0;
                pBridgeOutput->stStateBits.stBits.bIsNegative = 0;
                pBridgeOutput->stStateBits.stBits.bOpenBrake = 0;
                pBridgeOutput->dOutputFreq = 0.0;
                pBridgeOutput->dOutputSpeed = 0.0;
                // 小车输出
                pTrolleyOutput->stStateBits.stBits.bIsPositive = 0;
                pTrolleyOutput->stStateBits.stBits.bIsNegative = 0;
                pTrolleyOutput->stStateBits.stBits.bOpenBrake = 0;
                pTrolleyOutput->dOutputFreq = 0.0;
                pTrolleyOutput->dOutputSpeed = 0.0;
            }
            
            // 任务完成条件判断
            if (bIsBridgePosErrOk == 1 && bIsBridgeZeroSpeed && 
                bIsTrolleyPosErrOk == 1 && bIsTrolleyZeroSpeed && 
                bIsHoistPosErrOk == 1) {
                    // 起升输出
                pHoistOutput->stStateBits.stBits.bIsPositive = 0;
                pHoistOutput->stStateBits.stBits.bIsNegative = 0;
                pHoistOutput->dOutputFreq = 0.0;
                pHoistOutput->dOutputSpeed = 0.0;
                // 任务完成
                // 状态转换
                pTask->eTaskState = SKA_ASC_FINISH_TASK_STATE;
            }

            break;
        }
        case SKA_ASC_FINISH_TASK_STATE:
        {
            // 完成状态

            /* 设置输出 */
            pBridgeOutput->stStateBits.stBits.bIsPositive = 0;
            pBridgeOutput->stStateBits.stBits.bIsNegative = 0;
            pBridgeOutput->stStateBits.stBits.bIsBusy = 0;
            pBridgeOutput->stStateBits.stBits.bOpenBrake = 0;
            pBridgeOutput->stStateBits.stBits.bIsZeroManual = 0;
            pBridgeOutput->stStateBits.stBits.bIsPosErrOk = 0;
            pBridgeOutput->stStateBits.stBits.bIsZeroSpeed = 0;
            pBridgeOutput->stStateBits.stBits.bIsSwayAngleOk = 0;
            pBridgeOutput->dOutputFreq = 0.0;
            pBridgeOutput->dOutputSpeed = 0.0;
            pBridgeOutput->dBrakeDist = 0.0;
            pBridgeOutput->dOriTrackErr = 0.0;
            pBridgeOutput->dFilTrackErr = 0.0;

            pTrolleyOutput->stStateBits.stBits.bIsPositive = 0;
            pTrolleyOutput->stStateBits.stBits.bIsNegative = 0;
            pTrolleyOutput->stStateBits.stBits.bIsBusy = 0;
            pTrolleyOutput->stStateBits.stBits.bOpenBrake = 0;
            pTrolleyOutput->stStateBits.stBits.bIsZeroManual = 0;
            pTrolleyOutput->stStateBits.stBits.bIsPosErrOk = 0;
            pTrolleyOutput->stStateBits.stBits.bIsZeroSpeed = 0;
            pTrolleyOutput->stStateBits.stBits.bIsSwayAngleOk = 0;
            pTrolleyOutput->dOutputFreq = 0.0;
            pTrolleyOutput->dOutputSpeed = 0.0;
            pTrolleyOutput->dBrakeDist = 0.0;
            pTrolleyOutput->dOriTrackErr = 0.0;
            pTrolleyOutput->dFilTrackErr = 0.0;

            pHoistOutput->stStateBits.stBits.bIsPositive = 0;
            pHoistOutput->stStateBits.stBits.bIsNegative = 0;
            pHoistOutput->stStateBits.stBits.bIsBusy = 0;
            pHoistOutput->stStateBits.stBits.bIsPosErrOk = 0;
            pHoistOutput->dOutputFreq = 0.0;
            pHoistOutput->dOutputSpeed = 0.0;
            pHoistOutput->dOriTrackErr = 0.0;
            pHoistOutput->dFilTrackErr = 0.0;
         
            /* 清理操作 */
            // 自动模式独有的模块
            // 轨迹规划器
            SKA_VCFTP_Destroy(&(pTask->stVCFTracePlanner));

            // 零速度检测器
            SKA_ZSD_Destroy(&(pTask->stBridgeZSD));
            SKA_ZSD_Destroy(&(pTask->stTrolleyZSD));

            /* 无法驱动错误检测器 */
            SKA_CDED_Destroy(&(pTask->stBridgeCDED));
            SKA_CDED_Destroy(&(pTask->stTrolleyCDED));
            SKA_CDED_Destroy(&(pTask->stHoistCDED));

            /* 状态转换 */
            pTask->eTaskState = SKA_ASC_FREE_TASK_STATE;

            break;
        }
        default:
        {
            return -1;
        }
        }
    }
    else
    {
        // 控制器未使能

        // 设置输出
        pBridgeOutput->stStateBits.nData = 0;
        pBridgeOutput->stErrorBits.nData = 0;
        pBridgeOutput->stWarnBits.nData = 0;
        pBridgeOutput->dOutputFreq = 0.0;
        pBridgeOutput->dOutputSpeed = 0.0;
        pBridgeOutput->dBrakeDist = 0.0;
        pBridgeOutput->dEffRopeLen = 0.0;
        pBridgeOutput->dTargetPos = 0.0;
        pBridgeOutput->dOriTrackErr = 0.0;
        pBridgeOutput->dFilTrackErr = 0.0;

        pTrolleyOutput->stStateBits.nData = 0;
        pTrolleyOutput->stErrorBits.nData = 0;
        pTrolleyOutput->stWarnBits.nData = 0;
        pTrolleyOutput->dOutputFreq = 0.0;
        pTrolleyOutput->dOutputSpeed = 0.0;
        pTrolleyOutput->dBrakeDist = 0.0;
        pTrolleyOutput->dEffRopeLen = 0.0;
        pTrolleyOutput->dTargetPos = 0.0;
        pTrolleyOutput->dOriTrackErr = 0.0;
        pTrolleyOutput->dFilTrackErr = 0.0;

        pHoistOutput->stStateBits.nData = 0;
        pHoistOutput->stErrorBits.nData = 0;
        pHoistOutput->stWarnBits.nData = 0;
        pHoistOutput->dOutputFreq = 0.0;
        pHoistOutput->dOutputSpeed = 0.0;
        pHoistOutput->dTargetRopeLength = 0.0;
        pHoistOutput->dOriTrackErr = 0.0;
        pHoistOutput->dFilTrackErr = 0.0;

        // 控制器复位
        pTask->eTaskState = SKA_ASC_FREE_TASK_STATE;
    }

    return 0;
}