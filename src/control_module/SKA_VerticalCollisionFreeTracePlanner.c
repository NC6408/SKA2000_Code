/**
 ******************************************************************************
 * @file       : SKA_VerticalCollisionFreeTracePlanner.c
 * @brief      : 纵向避障轨迹规划器
 * @author     : ZhangYi
 * @version    : None
 * @date       : 2024/10/24
 ******************************************************************************
 */
//

#include <stddef.h>

#include "SKA_VerticalCollisionFreeTracePlanner.h"

/**
 * @brief  协调大小车运动参数器(协调大小车最大速度、最大加速度、最大加加速度)
 * @param  pVCFTP                纵向避障轨迹规划器的地址
 * @param  dMaxVelocityX         大车最大速度(m/s)
 * @param  dMaxAccelerationX     大车最大加速度(m/s^2)
 * @param  dMaxVelocityY         小车最大速度(m/s)
 * @param  dMaxAccelerationY     小车最大加速度(m/s^2)
 * @param  dStartPosX            大车任务开始位置(m)
 * @param  dTargetPosX           大车任务目标位置(m)
 * @param  dStartPosY            小车任务开始位置(m)
 * @param  dTargetPosY           小车任务目标位置(m)
 * @param  pRealMaxVelocityX 存储协调运动后大车的实际最大速度(m/s)的地址
 * @param  pRealMaxAccelerationX 存储协调运动后大车的实际最大加速度(m/s^2)的地址
 * @param  pRealMaxJerkX     存储协调运动后大车的实际最大加加速度(m/s^3)的地址
 * @param  pRealMaxVelocityY 存储协调运动后小车的实际最大速度(m/s)的地址
 * @param  pRealMaxAccelerationY 存储协调运动后小车的实际最大加速度(m/s^2)的地址
 * @param  pRealMaxJerkY     存储协调运动后小车的实际最大加加速度(m/s^3)的地址
 * @return 0:success, -1:fail
 * @note   None
 */
static int8_t SKA_VCFTP_CoordinateXYMotion(SKA_VerticalCollisionFreeTracePlanner *pVCFTP, double dMaxVelocityX,
                                           double dMaxAccelerationX, double dMaxVelocityY, double dMaxAccelerationY,
                                           double dStartPosX, double dTargetPosX, double dStartPosY, double dTargetPosY,
                                           double *pRealMaxVelocityX, double *pRealMaxAccelerationX, double *pRealMaxJerkX,
                                           double *pRealMaxVelocityY, double *pRealMaxAccelerationY, double *pRealMaxJerkY);

/**
 * @brief  获取地形信息
 * @return 0:success, -1:fail
 * @note   None
 */
static int8_t SKA_VCFTP_GetTerrainInformation(SKA_VerticalCollisionFreeTracePlanner *pVCFTP);

/**
 * @brief  计算运行机构运动时间和起升机构运动时间
 * @param  pVCFTP 运行机构轨迹规划器的地址
 * @param  bActiveSwayCtrl 激活防摇控制
 * @param  eType 输入整形器类型
 * @param  dNaturalFreq 自然频率(rad/s)
 * @param  dDampingRatio 阻尼比
 * @param  dTravelUnitStartPos 运行机构开始位置(m)
 * @param  dTravelUnitTargetPos 运行机构目标位置(m)
 * @param  dTravelUnitFindPos   障碍物水平位置(m)
 * @param  dTravelUnitMaxVelocity 运行机构最大速度(m/s)
 * @param  dTravelUnitMaxAcceleration 运行机构最大加速度(m/s^2)
 * @param  dTravelUnitMaxJerk     运行机构最大加加速度(m/s^3)
 * @param  dHoistUnitStartPos 起升机构开始位置(m)
 * @param  dHoistUnitTargetPos 起升机构目标位置(m)
 * @param  dHoistUnitFindPos   障碍物高度位置(m)
 * @param  dHoistUnitMaxVelocity 起升机构最大速度(m/s)
 * @param  dHoistUnitMaxAcceleration 起升机构最大加速度(m/s^2)
 * @param  dHoistUnitMaxJerk     起升机构最大加加速度(m/s^3)
 * @param  pTravelUnitMotionTime 存储运行机构整形后运动时间的地址
 * @param  pHoistUnitMotionTime  存储起升运动运动时间的地址
 * @return 0:success, -1:fail
 * @note   None
 */
static int8_t SKA_VCFTP_ComputeTravelUnitAndHoistUnitMotionTime(SKA_VerticalCollisionFreeTracePlanner *pVCFTP, bool bActiveSwayCtrl, SKA_InputShaperType eType,
                                                                double dNaturalFreq, double dDampingRatio, double dTravelUnitStartPos, double dTravelUnitTargetPos,
                                                                double dTravelUnitFindPos, double dTravelUnitMaxVelocity, double dTravelUnitMaxAcceleration,
                                                                double dTravelUnitMaxJerk, double dQuickStopMaxAcc, double dHoistUnitStartPos,
                                                                double dHoistUnitTargetPos, double dHoistUnitFindPos, double dHoistUnitMaxVelocity,
                                                                double dHoistUnitMaxAcceleration, double dHoistUnitMaxJerk,
                                                                double *pTravelUnitMotionTime, double *pHoistUnitMotionTime);

int8_t SKA_VCFTP_Create(SKA_VerticalCollisionFreeTracePlanner *pVCFTP, bool bBridgeActiveSwayCtrl, SKA_InputShaperType BridgeeType,
                        double dBridgeNaturalFreq, double dBridgeDampingRatio, double dBridgeStartPos, double dBridgeTargetPos,
                        double dBridgeMaxVelocity, double dBridgeMaxAcceleration, double dBridgeQuickStopMaxAcc, bool bTrolleyActiveSwayCtrl,
                        SKA_InputShaperType TrolleyeType, double dTrolleyNaturalFreq, double dTrolleyDampingRatio, double dTrolleyStartPos,
                        double dTrolleyTargetPos, double dTrolleyMaxVelocity, double dTrolleyMaxAcceleration, double dTrolleyQuickStopMaxAcc,
                        double dHoistStartRopeLength, double dHoistMinRopeLength, double dHoistTargetRopeLength, double dHoistMaxVel,
                        double dHoistMaxAcc, double dHoistMaxJerk)
{
    /******************** 函数参数合法性检验 ********************/
    if (pVCFTP == NULL || dBridgeNaturalFreq <= 0 || dBridgeDampingRatio < 0 || dBridgeDampingRatio >= 1 ||
        dBridgeMaxVelocity <= 0 || dBridgeMaxAcceleration <= 0 || dBridgeQuickStopMaxAcc <= 0 ||
        dTrolleyNaturalFreq <= 0 || dTrolleyDampingRatio < 0 || dTrolleyDampingRatio >= 1 ||
        dTrolleyMaxVelocity <= 0 || dTrolleyMaxAcceleration <= 0 || dTrolleyQuickStopMaxAcc <= 0 ||
        dHoistMaxVel <= 0 || dHoistMaxAcc <= 0 || dHoistMaxJerk <= 0)
    {
        printf("Invalid parameters of SKA_VCFTP_Create().\n");
        return -1;
    }

    /******************** 创建运行机构轨迹规划器 ********************/

    // 协调大小车运动参数器(协调大小车最大速度、最大加速度、最大加加速度)
    SKA_VCFTP_CoordinateXYMotion(pVCFTP, dBridgeMaxVelocity, dBridgeMaxAcceleration, dTrolleyMaxVelocity, dTrolleyMaxAcceleration,
                                 dBridgeStartPos, dBridgeTargetPos, dTrolleyStartPos, dTrolleyTargetPos, &(pVCFTP->dBridgeRealMaxVelocity),
                                 &(pVCFTP->dBridgeRealMaxAcceleration), &(pVCFTP->dBridgeRealMaxJerk), &(pVCFTP->dTrolleyRealMaxVelocity),
                                 &(pVCFTP->dTrolleyRealMaxAcceleration), &(pVCFTP->dTrolleyRealMaxJerk));

    // 创建大车轨迹规划器
    pVCFTP->bBridgeIsQuickStopping = 0;
    pVCFTP->dBridgeQuickStopMaxAcc = dBridgeQuickStopMaxAcc;
    pVCFTP->bBridgeActiveSwayCtrl = bBridgeActiveSwayCtrl;
    pVCFTP->BridgeeType = BridgeeType;
    pVCFTP->dBridgeNaturalFreq = dBridgeNaturalFreq;
    pVCFTP->dBridgeDampingRatio = dBridgeDampingRatio;
    pVCFTP->dBridgeStartPos = dBridgeStartPos;
    pVCFTP->dBridgeTargetPos = dBridgeTargetPos;
    SKA_TUTP_Create(&(pVCFTP->stBridgeTracePlanner), pVCFTP->bBridgeActiveSwayCtrl, pVCFTP->BridgeeType,
                    pVCFTP->dBridgeNaturalFreq, pVCFTP->dBridgeDampingRatio, dBridgeStartPos, dBridgeTargetPos,
                    pVCFTP->dBridgeRealMaxVelocity, pVCFTP->dBridgeRealMaxAcceleration, pVCFTP->dBridgeRealMaxJerk, pVCFTP->dBridgeQuickStopMaxAcc);
    // 创建小车轨迹规划器
    pVCFTP->bTrolleyIsQuickStopping = 0;
    pVCFTP->dTrolleyQuickStopMaxAcc = dTrolleyQuickStopMaxAcc;
    pVCFTP->bTrolleyActiveSwayCtrl = bTrolleyActiveSwayCtrl;
    pVCFTP->TrolleyeType = TrolleyeType;
    pVCFTP->dTrolleyNaturalFreq = dTrolleyNaturalFreq;
    pVCFTP->dTrolleyDampingRatio = dTrolleyDampingRatio;
    pVCFTP->dTrolleyStartPos = dTrolleyStartPos;
    pVCFTP->dTrolleyTargetPos = dTrolleyTargetPos;
    SKA_TUTP_Create(&(pVCFTP->stTrolleyTracePlanner), pVCFTP->bTrolleyActiveSwayCtrl, pVCFTP->TrolleyeType,
                    pVCFTP->dTrolleyNaturalFreq, pVCFTP->dTrolleyDampingRatio, dTrolleyStartPos, dTrolleyTargetPos,
                    pVCFTP->dTrolleyRealMaxVelocity, pVCFTP->dTrolleyRealMaxAcceleration, pVCFTP->dTrolleyRealMaxJerk, pVCFTP->dTrolleyQuickStopMaxAcc);
    // 创建起升机构的升吊和落吊的轨迹规划器
    pVCFTP->dHoistStartRopeLength = dHoistStartRopeLength;
    pVCFTP->dHoistMinRopeLength = dHoistMinRopeLength;
    pVCFTP->dHoistTargetRopeLength = dHoistTargetRopeLength;
    pVCFTP->dHoistMaxVel = dHoistMaxVel;
    pVCFTP->dHoistMaxAcc = dHoistMaxAcc;
    pVCFTP->dHoistMaxJerk = dHoistMaxJerk;
    SKA_HUTP_Create(&(pVCFTP->stHoistUnitUpTracePlanner), dHoistStartRopeLength, dHoistMinRopeLength, dHoistMaxVel, dHoistMaxAcc, dHoistMaxJerk);
    SKA_HUTP_Create(&(pVCFTP->stHoistUnitDownTracePlanner), dHoistMinRopeLength, dHoistTargetRopeLength, dHoistMaxVel, dHoistMaxAcc, dHoistMaxJerk);
    SKA_HUTP_GetTotalTime(&(pVCFTP->stHoistUnitUpTracePlanner), &(pVCFTP->dHoistUpTraceTotalTime));
    return 0;
}

int8_t SKA_VCFTP_Run(SKA_VerticalCollisionFreeTracePlanner *pVCFTP, double dCurTime, double dTravelUnitMaxDelayTime, double dHoistUnitMaxStartDownTime,
                     double *pBridgeRefPos, double *pBridgeRefVel, double *pBridgeRefAcc, double *pBridgeUnsRefPos, double *pBridgeUnsRefVel, 
                     double *pBridgeUnsRefAcc, double *pTrolleyRefPos, double *pTrolleyRefVel, double *pTrolleyRefAcc, double *pTrolleyUnsRefPos, 
                     double *pTrolleyUnsRefVel, double *pTrolleyUnsRefAcc, double *pHoistRefPos, double *pHoistRefVel, double *pHoistRefAcc)
{
    /******************** 函数参数合法性检验 ********************/
    if (pVCFTP == NULL || pBridgeRefPos == NULL || pBridgeRefVel == NULL || pBridgeRefAcc == NULL ||
        pBridgeUnsRefPos == NULL || pBridgeUnsRefVel == NULL || pBridgeUnsRefAcc == NULL ||
        pTrolleyRefPos == NULL || pTrolleyRefVel == NULL || pTrolleyRefAcc == NULL ||
        pTrolleyUnsRefPos == NULL || pTrolleyUnsRefVel == NULL || pTrolleyUnsRefAcc == NULL ||
        pHoistRefPos == NULL || pHoistRefVel == NULL || pHoistRefAcc == NULL)
    {
        printf("Invalid parameters of SKA_VCFTP_Run().\n");
        return -1;
    }

    /******************** 运行机构轨迹规划器 ********************/
    
    if (fabs(pVCFTP->dBridgeStartPos - pVCFTP->dBridgeTargetPos) > SKA_FlOAT_ERROR &&
        fabs(pVCFTP->dTrolleyStartPos - pVCFTP->dTrolleyTargetPos) > SKA_FlOAT_ERROR)
    {
        /* 大车轨迹规划 */
        if (dCurTime < dTravelUnitMaxDelayTime)
        {
            *pBridgeRefPos = pVCFTP->dBridgeStartPos;
            *pBridgeRefVel = 0.0;
            *pBridgeRefAcc = 0.0;
        }
        else
        {
            SKA_TUTP_Run(&(pVCFTP->stBridgeTracePlanner), dCurTime - dTravelUnitMaxDelayTime,
                         pBridgeRefPos, pBridgeRefVel, pBridgeRefAcc, pBridgeUnsRefPos, pBridgeUnsRefVel, pBridgeUnsRefAcc);
        }

        /* 小车轨迹规划 */
        if (dCurTime < dTravelUnitMaxDelayTime)
        {
            *pTrolleyRefPos = pVCFTP->dTrolleyStartPos;
            *pTrolleyRefVel = 0.0;
            *pTrolleyRefAcc = 0.0;
        }
        else
        {
            SKA_TUTP_Run(&(pVCFTP->stTrolleyTracePlanner), dCurTime - dTravelUnitMaxDelayTime,
                         pTrolleyRefPos, pTrolleyRefVel, pTrolleyRefAcc, pTrolleyUnsRefPos, pTrolleyUnsRefVel, pTrolleyUnsRefAcc);
        }
    }

    if ((fabs(pVCFTP->dBridgeStartPos - pVCFTP->dBridgeTargetPos) < SKA_FlOAT_ERROR) &&
        (fabs(pVCFTP->dTrolleyStartPos - pVCFTP->dTrolleyTargetPos) > SKA_FlOAT_ERROR))
    {
        /* 大车保持不动 */
        *pBridgeRefPos = pVCFTP->dBridgeStartPos;
        *pBridgeRefVel = 0.0;
        *pBridgeRefAcc = 0.0;

        /* 小车轨迹规划 */
        if (dCurTime < dTravelUnitMaxDelayTime)
        {
            *pTrolleyRefPos = pVCFTP->dTrolleyStartPos;
            *pTrolleyRefVel = 0.0;
            *pTrolleyRefAcc = 0.0;
        }
        else
        {
            SKA_TUTP_Run(&(pVCFTP->stTrolleyTracePlanner), dCurTime - dTravelUnitMaxDelayTime,
                         pTrolleyRefPos, pTrolleyRefVel, pTrolleyRefAcc, pTrolleyUnsRefPos, pTrolleyUnsRefVel, pTrolleyUnsRefAcc);
        }
    }
    if (fabs(pVCFTP->dBridgeStartPos - pVCFTP->dBridgeTargetPos) > SKA_FlOAT_ERROR &&
        fabs(pVCFTP->dTrolleyStartPos - pVCFTP->dTrolleyTargetPos) < SKA_FlOAT_ERROR)
    {
        /* 大车轨迹规划 */
        if (dCurTime < dTravelUnitMaxDelayTime)
        {
            *pBridgeRefPos = pVCFTP->dBridgeStartPos;
            *pBridgeRefVel = 0.0;
            *pBridgeRefAcc = 0.0;
        }
        else
        {
            SKA_TUTP_Run(&(pVCFTP->stBridgeTracePlanner), dCurTime - dTravelUnitMaxDelayTime,
                         pBridgeRefPos, pBridgeRefVel, pBridgeRefAcc, pBridgeUnsRefPos, pBridgeUnsRefVel, pBridgeUnsRefAcc);
        }
        /* 小车保持不动 */
        *pTrolleyRefPos = pVCFTP->dTrolleyStartPos;
        *pTrolleyRefVel = 0.0;
        *pTrolleyRefAcc = 0.0;
    }

    if (fabs(pVCFTP->dBridgeStartPos - pVCFTP->dBridgeTargetPos) < SKA_FlOAT_ERROR &&
        fabs(pVCFTP->dTrolleyStartPos - pVCFTP->dTrolleyTargetPos) < SKA_FlOAT_ERROR)
    {
        /* 大车保持不动 */
        *pBridgeRefPos = pVCFTP->dBridgeStartPos;
        *pBridgeRefVel = 0.0;
        *pBridgeRefAcc = 0.0;
        /* 小车保持不动 */
        *pTrolleyRefPos = pVCFTP->dTrolleyStartPos;
        *pTrolleyRefVel = 0.0;
        *pTrolleyRefAcc = 0.0;
    }

    /* 起升机构轨迹规划 */
    if (pVCFTP->dHoistUpTraceTotalTime <= dHoistUnitMaxStartDownTime)
    {
        // 可以在最高处停留
        if (dCurTime <= pVCFTP->dHoistUpTraceTotalTime)
        {
            SKA_HUTP_Run(&(pVCFTP->stHoistUnitUpTracePlanner), dCurTime, pHoistRefPos, pHoistRefVel, pHoistRefAcc);
        }
        else if (dCurTime <= dHoistUnitMaxStartDownTime)
        {
            *pHoistRefPos = pVCFTP->dHoistMinRopeLength;
            *pHoistRefVel = 0.0;
            *pHoistRefAcc = 0.0;
        }
        else
        {
            SKA_HUTP_Run(&(pVCFTP->stHoistUnitDownTracePlanner), dCurTime - dHoistUnitMaxStartDownTime,
                         pHoistRefPos, pHoistRefVel, pHoistRefAcc);
        }
    }
    else
    {
        // 不在最高处停留
        if (dCurTime <= pVCFTP->dHoistUpTraceTotalTime)
        {
            SKA_HUTP_Run(&(pVCFTP->stHoistUnitUpTracePlanner), dCurTime, pHoistRefPos, pHoistRefVel, pHoistRefAcc);
        }
        else
        {
            SKA_HUTP_Run(&(pVCFTP->stHoistUnitDownTracePlanner), dCurTime - pVCFTP->dHoistUpTraceTotalTime,
                         pHoistRefPos, pHoistRefVel, pHoistRefAcc);
        }
    }
    return 0;
}

int8_t SKA_VCFTP_TravelMaxDelayTimeAndHoistMaxStartDownTime(SKA_VerticalCollisionFreeTracePlanner *pVCFTP,
                                                            double *pTravelUnitMaxDelayTime, double *pHoistUnitMaxStartDownTime)
{
    /******************** 函数参数合法性检验 ********************/
    if (pVCFTP == NULL || pTravelUnitMaxDelayTime == NULL || pHoistUnitMaxStartDownTime == NULL)
    {
        printf("Invalid parameters of SKA_VCFTP_TravelMaxDelayTimeAndHoistMaxStartDownTime().\n");
        return -1;
    }

    /******************** 获取运行机构开始运行的最大延迟时间和起升机构最大开始下降时间 ********************/
    // 获取地形信息
    SKA_VCFTP_GetTerrainInformation(pVCFTP);
    
    // 初始化运行机构最大延迟时间
    *pTravelUnitMaxDelayTime = 0;
    for (int i = 0; i < pVCFTP->nLiftArraySize; i++)
    {
        // 获取运行机构和起升机构起吊轨迹时间
        if (fabs(pVCFTP->dBridgeTargetPos - pVCFTP->dBridgeStartPos) > SKA_FlOAT_ERROR)
        {
            SKA_VCFTP_ComputeTravelUnitAndHoistUnitMotionTime(pVCFTP, pVCFTP->bBridgeActiveSwayCtrl, pVCFTP->BridgeeType,
                                                              pVCFTP->dBridgeNaturalFreq, pVCFTP->dBridgeDampingRatio,
                                                              pVCFTP->dBridgeStartPos, pVCFTP->dBridgeTargetPos, pVCFTP->arrObstacleBridgeUpPos[i],
                                                              pVCFTP->dBridgeRealMaxVelocity, pVCFTP->dBridgeRealMaxAcceleration, pVCFTP->dBridgeRealMaxJerk,
                                                              pVCFTP->dBridgeQuickStopMaxAcc, pVCFTP->dHoistStartRopeLength, pVCFTP->dHoistMinRopeLength,
                                                              pVCFTP->dTotalHeight - pVCFTP->arrLiftTerrain[i].dHeight,
                                                              pVCFTP->dHoistMaxVel, pVCFTP->dHoistMaxAcc, pVCFTP->dHoistMaxJerk,
                                                              &(pVCFTP->dTravelUnitUpMotionTime), &(pVCFTP->dHoistUnitUpMotionTime));
        }
        else
        {
            SKA_VCFTP_ComputeTravelUnitAndHoistUnitMotionTime(pVCFTP, pVCFTP->bTrolleyActiveSwayCtrl, pVCFTP->TrolleyeType,
                                                              pVCFTP->dTrolleyNaturalFreq, pVCFTP->dTrolleyDampingRatio,
                                                              pVCFTP->dTrolleyStartPos, pVCFTP->dTrolleyTargetPos, pVCFTP->arrObstacleTrolleyUpPos[i],
                                                              pVCFTP->dTrolleyRealMaxVelocity, pVCFTP->dTrolleyRealMaxAcceleration, pVCFTP->dTrolleyRealMaxJerk,
                                                              pVCFTP->dTrolleyQuickStopMaxAcc, pVCFTP->dHoistStartRopeLength, pVCFTP->dHoistMinRopeLength,
                                                              pVCFTP->dTotalHeight - pVCFTP->arrLiftTerrain[i].dHeight,
                                                              pVCFTP->dHoistMaxVel, pVCFTP->dHoistMaxAcc, pVCFTP->dHoistMaxJerk,
                                                              &(pVCFTP->dTravelUnitUpMotionTime), &(pVCFTP->dHoistUnitUpMotionTime));
        }

        if (pVCFTP->dHoistUnitUpMotionTime > pVCFTP->dTravelUnitUpMotionTime)
        {
            pVCFTP->dTravelUnitDelayTime = pVCFTP->dHoistUnitUpMotionTime - pVCFTP->dTravelUnitUpMotionTime;
        }
        else
        {
            pVCFTP->dTravelUnitDelayTime = 0;
        }
        // 获取运行机构最大延迟时间
        if (pVCFTP->dTravelUnitDelayTime >= *pTravelUnitMaxDelayTime)
        {
            *pTravelUnitMaxDelayTime = pVCFTP->dTravelUnitDelayTime;
        }
    }

    // 初始化起升机构的最大开始下降时间
    *pHoistUnitMaxStartDownTime = 0;
    /* 获取落吊运行时间 */
    for (int i = 0; i < pVCFTP->nLiftArraySize + pVCFTP->nDropArraySize; i++)
    {
        if (i < pVCFTP->nDropArraySize)
        {
            // 考虑“起吊障碍物”的落吊运行时间
            // 获取运行机构和起升机构落吊轨迹时间
            if (fabs(pVCFTP->dBridgeTargetPos - pVCFTP->dBridgeStartPos) > SKA_FlOAT_ERROR)
            {
                // 大车运行距离不为0时
                SKA_VCFTP_ComputeTravelUnitAndHoistUnitMotionTime(pVCFTP, pVCFTP->bBridgeActiveSwayCtrl, pVCFTP->BridgeeType,
                                                                  pVCFTP->dBridgeNaturalFreq, pVCFTP->dBridgeDampingRatio, pVCFTP->dBridgeStartPos,
                                                                  pVCFTP->dBridgeTargetPos, pVCFTP->arrObstacleBridgeDownPos[i],
                                                                  pVCFTP->dBridgeRealMaxVelocity, pVCFTP->dBridgeRealMaxAcceleration, pVCFTP->dBridgeRealMaxJerk,
                                                                  pVCFTP->dBridgeQuickStopMaxAcc, pVCFTP->dHoistMinRopeLength, pVCFTP->dHoistTargetRopeLength,
                                                                  pVCFTP->dTotalHeight - pVCFTP->arrDropTerrain[i].dHeight,
                                                                  pVCFTP->dHoistMaxVel, pVCFTP->dHoistMaxAcc, pVCFTP->dHoistMaxJerk,
                                                                  &(pVCFTP->dTravelUnitDownMotionTime), &(pVCFTP->dHoistUnitDownMotionTime));

            }
            else
            {
                // 大车运行距离为0时，即只运行小车情况
                SKA_VCFTP_ComputeTravelUnitAndHoistUnitMotionTime(pVCFTP, pVCFTP->bTrolleyActiveSwayCtrl, pVCFTP->TrolleyeType,
                                                                  pVCFTP->dTrolleyNaturalFreq, pVCFTP->dTrolleyDampingRatio, pVCFTP->dTrolleyStartPos,
                                                                  pVCFTP->dTrolleyTargetPos, pVCFTP->arrObstacleTrolleyDownPos[i],
                                                                  pVCFTP->dTrolleyRealMaxVelocity, pVCFTP->dTrolleyRealMaxAcceleration, pVCFTP->dTrolleyRealMaxJerk,
                                                                  pVCFTP->dTrolleyQuickStopMaxAcc, pVCFTP->dHoistMinRopeLength, pVCFTP->dHoistTargetRopeLength,
                                                                  pVCFTP->dTotalHeight - pVCFTP->arrDropTerrain[i].dHeight,
                                                                  pVCFTP->dHoistMaxVel, pVCFTP->dHoistMaxAcc, pVCFTP->dHoistMaxJerk,
                                                                  &(pVCFTP->dTravelUnitDownMotionTime), &(pVCFTP->dHoistUnitDownMotionTime));
            }
        }
        else
        {
            // 考虑“落吊障碍物”的落吊运行时间
            // 获取运行机构落吊轨迹时间、起升机构落吊轨迹时间
            if (fabs(pVCFTP->dBridgeTargetPos - pVCFTP->dBridgeStartPos) > SKA_FlOAT_ERROR)
            {
                // 大车运行距离不为0时
                SKA_VCFTP_ComputeTravelUnitAndHoistUnitMotionTime(pVCFTP, pVCFTP->bBridgeActiveSwayCtrl, pVCFTP->BridgeeType,
                                                                  pVCFTP->dBridgeNaturalFreq, pVCFTP->dBridgeDampingRatio, pVCFTP->dBridgeStartPos,
                                                                  pVCFTP->dBridgeTargetPos, pVCFTP->arrObstacleBridgeUpPos[i - pVCFTP->nDropArraySize],
                                                                  pVCFTP->dBridgeRealMaxVelocity, pVCFTP->dBridgeRealMaxAcceleration, pVCFTP->dBridgeRealMaxJerk,
                                                                  pVCFTP->dBridgeQuickStopMaxAcc, pVCFTP->dHoistMinRopeLength, pVCFTP->dHoistTargetRopeLength,
                                                                  pVCFTP->dTotalHeight - pVCFTP->arrLiftTerrain[i - pVCFTP->nDropArraySize].dHeight,
                                                                  pVCFTP->dHoistMaxVel, pVCFTP->dHoistMaxAcc, pVCFTP->dHoistMaxJerk,
                                                                  &(pVCFTP->dTravelUnitDownMotionTime), &(pVCFTP->dHoistUnitDownMotionTime));
            }
            else
            {
                // 大车运行距离为0时，即只运行小车情况
                SKA_VCFTP_ComputeTravelUnitAndHoistUnitMotionTime(pVCFTP, pVCFTP->bTrolleyActiveSwayCtrl, pVCFTP->TrolleyeType,
                                                                  pVCFTP->dTrolleyNaturalFreq, pVCFTP->dTrolleyDampingRatio, pVCFTP->dTrolleyStartPos,
                                                                  pVCFTP->dTrolleyTargetPos, pVCFTP->arrObstacleTrolleyUpPos[i - pVCFTP->nDropArraySize],
                                                                  pVCFTP->dTrolleyRealMaxVelocity, pVCFTP->dTrolleyRealMaxAcceleration, pVCFTP->dTrolleyRealMaxJerk,
                                                                  pVCFTP->dTrolleyQuickStopMaxAcc, pVCFTP->dHoistMinRopeLength, pVCFTP->dHoistTargetRopeLength,
                                                                  pVCFTP->dTotalHeight - pVCFTP->arrLiftTerrain[i - pVCFTP->nDropArraySize].dHeight,
                                                                  pVCFTP->dHoistMaxVel, pVCFTP->dHoistMaxAcc, pVCFTP->dHoistMaxJerk,
                                                                  &(pVCFTP->dTravelUnitDownMotionTime), &(pVCFTP->dHoistUnitDownMotionTime));
            }
        }


        pVCFTP->dHoistUnitDelayTime = pVCFTP->dTravelUnitDownMotionTime - pVCFTP->dHoistUnitDownMotionTime;

        if (pVCFTP->dHoistUnitDownMotionTime == -1)
        {
            pVCFTP->dHoistUnitDelayTime = 0;
        }
        pVCFTP->dHoistUnitStartDownTime = *pTravelUnitMaxDelayTime + pVCFTP->dHoistUnitDelayTime;
        // 启动时间有效性判断
        if (pVCFTP->dHoistUnitStartDownTime < 0)
        {
            pVCFTP->dHoistUnitStartDownTime = 0;
        }
        // 获取起升机构的最大开始降落时间
        if (pVCFTP->dHoistUnitStartDownTime >= *pHoistUnitMaxStartDownTime)
        {
            *pHoistUnitMaxStartDownTime = pVCFTP->dHoistUnitStartDownTime;
        }
    }
    return 0;
}

int8_t SKA_VCFTP_Destroy(SKA_VerticalCollisionFreeTracePlanner *pVCFTP)
{
    /******************** 函数参数合法性检验 ********************/
    if (pVCFTP == NULL)
    {
        printf("Invalid parameters of SKA_VCFTP_Destroy().\n");
        return -1;
    }

    /******************** 销毁运行机构轨迹规划器 ********************/
    SKA_TUTP_Destroy(&(pVCFTP->stBridgeTracePlanner));
    SKA_TUTP_Destroy(&(pVCFTP->stTrolleyTracePlanner));
    SKA_HUTP_Destroy(&(pVCFTP->stHoistUnitUpTracePlanner));
    SKA_HUTP_Destroy(&(pVCFTP->stHoistUnitDownTracePlanner));

    return 0;
}

static int8_t SKA_VCFTP_CoordinateXYMotion(SKA_VerticalCollisionFreeTracePlanner *pVCFTP, double dMaxVelocityX,
                                           double dMaxAccelerationX, double dMaxVelocityY, double dMaxAccelerationY,
                                           double dStartPosX, double dTargetPosX, double dStartPosY, double dTargetPosY,
                                           double *pRealMaxVelocityX, double *pRealMaxAccelerationX, double *pRealMaxJerkX,
                                           double *pRealMaxVelocityY, double *pRealMaxAccelerationY, double *pRealMaxJerkY)
{
    /******************** 函数参数合法性检验 ********************/
    if (pVCFTP == NULL || dMaxVelocityX <= 0 || dMaxAccelerationX <= 0 ||
        dMaxVelocityY <= 0 || dMaxAccelerationY <= 0 || pRealMaxVelocityX == NULL ||
        pRealMaxAccelerationX == NULL || pRealMaxJerkX == NULL ||
        pRealMaxVelocityY == NULL || pRealMaxAccelerationY == NULL || pRealMaxJerkY == NULL)
    {
        printf("Invalid parameters of SKA_VCFTP_CoordinateXYMotion().\n");
        return -1;
    }

    /******************** 初始化参数器 ********************/
    pVCFTP->inputParamX.dMaxVelocity = dMaxVelocityX;
    pVCFTP->inputParamX.dMaxAcceleration = dMaxAccelerationX;
    pVCFTP->inputParamX.dMaxJerk = 5 * pow(pVCFTP->inputParamX.dMaxAcceleration, 2) / pVCFTP->inputParamX.dMaxVelocity;
    pVCFTP->inputParamY.dMaxVelocity = dMaxVelocityY;
    pVCFTP->inputParamY.dMaxAcceleration = dMaxAccelerationY;
    pVCFTP->inputParamY.dMaxJerk = 5 * pow(pVCFTP->inputParamY.dMaxAcceleration, 2) / pVCFTP->inputParamY.dMaxVelocity;
    *pRealMaxVelocityX = pVCFTP->inputParamX.dMaxVelocity;
    *pRealMaxAccelerationX = pVCFTP->inputParamX.dMaxAcceleration;
    *pRealMaxJerkX = pVCFTP->inputParamX.dMaxJerk;
    *pRealMaxVelocityY = pVCFTP->inputParamY.dMaxVelocity;
    *pRealMaxAccelerationY = pVCFTP->inputParamY.dMaxAcceleration;
    *pRealMaxJerkY = pVCFTP->inputParamY.dMaxJerk;
    
    /******************** 执行协调大小车运动参数器 ********************/
    pVCFTP->dDeltaX = dTargetPosX - dStartPosX;
    pVCFTP->dDeltaY = dTargetPosY - dStartPosY;
    double fRatioY_X, fRatioX_Y;
    if (pVCFTP->dDeltaX == 0.0 && pVCFTP->dDeltaY == 0.0)
    {
        *pRealMaxVelocityX = 0.0;
        *pRealMaxAccelerationX = 0.0;
        *pRealMaxJerkX = 0.0;
        *pRealMaxVelocityY = 0.0;
        *pRealMaxAccelerationY = 0.0;
        *pRealMaxJerkY = 0.0;
    }
    else if (pVCFTP->dDeltaX == 0 && pVCFTP->dDeltaY != 0)
    {
        *pRealMaxVelocityX = 0.0;
        *pRealMaxAccelerationX = 0.0;
        *pRealMaxJerkX = 0.0;
        *pRealMaxVelocityY = pVCFTP->inputParamY.dMaxVelocity;
        *pRealMaxAccelerationY = pVCFTP->inputParamY.dMaxAcceleration;
        *pRealMaxJerkY = pVCFTP->inputParamY.dMaxJerk;
    }
    else if (pVCFTP->dDeltaY == 0 && pVCFTP->dDeltaX != 0)
    {
        *pRealMaxVelocityY = 0.0;
        *pRealMaxAccelerationY = 0.0;
        *pRealMaxJerkY = 0.0;
        *pRealMaxVelocityX = pVCFTP->inputParamX.dMaxVelocity;
        *pRealMaxAccelerationX = pVCFTP->inputParamX.dMaxAcceleration;
        *pRealMaxJerkX = pVCFTP->inputParamX.dMaxJerk;
    }
    else
    {
        fRatioY_X = fabs(pVCFTP->dDeltaY) / fabs(pVCFTP->dDeltaX);
        fRatioX_Y = fabs(pVCFTP->dDeltaX) / fabs(pVCFTP->dDeltaY);
        // XY最大速度协调
        *pRealMaxVelocityY = fRatioY_X * pVCFTP->inputParamX.dMaxVelocity;
        if (*pRealMaxVelocityY > pVCFTP->inputParamY.dMaxVelocity)
        {
            *pRealMaxVelocityY = pVCFTP->inputParamY.dMaxVelocity;
            *pRealMaxVelocityX = fRatioX_Y * pVCFTP->inputParamY.dMaxVelocity;
        }
        else
        {
            *pRealMaxVelocityX = pVCFTP->inputParamX.dMaxVelocity;
        }
        // XY最大加速度协调
        *pRealMaxAccelerationY = fRatioY_X * pVCFTP->inputParamX.dMaxAcceleration;
        if (*pRealMaxAccelerationY > pVCFTP->inputParamY.dMaxAcceleration)
        {
            *pRealMaxAccelerationY = pVCFTP->inputParamY.dMaxAcceleration;
            *pRealMaxAccelerationX = fRatioX_Y * pVCFTP->inputParamY.dMaxAcceleration;
        }
        else
        {
            *pRealMaxAccelerationX = pVCFTP->inputParamX.dMaxAcceleration;
        }
        // XY最大加加速度协调
        *pRealMaxJerkY = fRatioY_X * pVCFTP->inputParamX.dMaxJerk;
        if (*pRealMaxJerkY > pVCFTP->inputParamY.dMaxJerk)
        {
            *pRealMaxJerkY = pVCFTP->inputParamY.dMaxJerk;
            *pRealMaxJerkX = fRatioX_Y * pVCFTP->inputParamY.dMaxJerk;
        }
        else
        {
            *pRealMaxJerkX = pVCFTP->inputParamX.dMaxJerk;
        }
    }

    return 0;
}

static int8_t SKA_VCFTP_GetTerrainInformation(SKA_VerticalCollisionFreeTracePlanner *pVCFTP)
{
    /******************** 函数参数合法性检验 ********************/
    if (pVCFTP == NULL)
    {
        printf("Invalid parameters of SKA_VCFTP_GetTerrainInformation().\n");
        return -1;
    }

    /******************** 地形信息分解x坐标、y坐标 ********************/
    double dTotalDistance = sqrt(pow((pVCFTP->dBridgeTargetPos - pVCFTP->dBridgeStartPos), 2) +
                                 pow((pVCFTP->dTrolleyTargetPos - pVCFTP->dTrolleyStartPos), 2));

    if (fabs(pVCFTP->dBridgeTargetPos - pVCFTP->dBridgeStartPos) > SKA_FlOAT_ERROR)
    {
        for (int i = 0; i < pVCFTP->nLiftArraySize; i++)
        {
            pVCFTP->arrObstacleBridgeUpPos[i] = (pVCFTP->arrLiftTerrain[i].dDistance / dTotalDistance) *
                                                    (pVCFTP->dBridgeTargetPos - pVCFTP->dBridgeStartPos) + pVCFTP->dBridgeStartPos;
        }

        for (int i = 0; i < pVCFTP->nDropArraySize; i++)
        {
            pVCFTP->arrObstacleBridgeDownPos[i] = (pVCFTP->arrDropTerrain[i].dDistance / dTotalDistance) *
                                                      (pVCFTP->dBridgeTargetPos - pVCFTP->dBridgeStartPos) + pVCFTP->dBridgeStartPos;
        }
    }
    else
    {
        for (int i = 0; i < pVCFTP->nLiftArraySize; i++)
        {
            pVCFTP->arrObstacleTrolleyUpPos[i] = (pVCFTP->arrLiftTerrain[i].dDistance / dTotalDistance) *
                                                     (pVCFTP->dTrolleyTargetPos - pVCFTP->dTrolleyStartPos) + pVCFTP->dTrolleyStartPos;
        }

        for (int i = 0; i < pVCFTP->nDropArraySize; i++)
        {
            pVCFTP->arrObstacleTrolleyDownPos[i] = (pVCFTP->arrDropTerrain[i].dDistance / dTotalDistance) *
                                                       (pVCFTP->dTrolleyTargetPos - pVCFTP->dTrolleyStartPos) + pVCFTP->dTrolleyStartPos;
        }
    }

    return 0;
}

static int8_t SKA_VCFTP_ComputeTravelUnitAndHoistUnitMotionTime(SKA_VerticalCollisionFreeTracePlanner *pVCFTP, bool bActiveSwayCtrl, SKA_InputShaperType eType,
                                                                double dNaturalFreq, double dDampingRatio, double dTravelUnitStartPos, double dTravelUnitTargetPos,
                                                                double dTravelUnitFindPos, double dTravelUnitMaxVelocity, double dTravelUnitMaxAcceleration,
                                                                double dTravelUnitMaxJerk, double dQuickStopMaxAcc, double dHoistUnitStartPos, double dHoistUnitTargetPos, 
                                                                double dHoistUnitFindPos, double dHoistUnitMaxVelocity, double dHoistUnitMaxAcceleration, double dHoistUnitMaxJerk,
                                                                double *pTravelUnitMotionTime, double *pHoistUnitMotionTime)
{
    /******************** 函数参数合法性检验 ********************/
    if (pVCFTP == NULL || dNaturalFreq <= 0 || dDampingRatio < 0 || dDampingRatio >= 1 ||
        dTravelUnitMaxVelocity <= 0 || dTravelUnitMaxAcceleration <= 0 || dTravelUnitMaxJerk <= 0 ||
        dTravelUnitFindPos < dTravelUnitStartPos && dTravelUnitFindPos < dTravelUnitTargetPos ||
        dTravelUnitFindPos > dTravelUnitStartPos && dTravelUnitFindPos > dTravelUnitTargetPos || 
        pTravelUnitMotionTime == NULL || dHoistUnitMaxVelocity <= 0 || dHoistUnitMaxAcceleration <= 0 || 
        dHoistUnitMaxJerk <= 0 || pHoistUnitMotionTime == NULL)
    {
        printf("Invalid parameters of SKA_VCFTP_ComputeTravelUnitAndHoistUnitMotionTime().\n");
        return -1;
    }

    /******************** 计算运行机构轨迹时间 ********************/
    pVCFTP->dlo = 0.0;
    if (fabsf(dTravelUnitStartPos - dTravelUnitTargetPos) < SKA_FlOAT_ERROR)
    {
        pVCFTP->dX = dTravelUnitTargetPos;
        pVCFTP->dDx = 0.0;
        pVCFTP->dDDx = 0.0;
        pVCFTP->dhi = 0.0;
    }
    else
    {
        pVCFTP->dX = 0.0;
        pVCFTP->dDx = 0.0;
        pVCFTP->dDDx = 0.0;
        double dTmpUnsRefPos, dTmpUnsRefVel, dTmpUnsRefAcc;
        SKA_TUTP_Create(&(pVCFTP->stTravelUnitTracePlanner), bActiveSwayCtrl, eType, dNaturalFreq, dDampingRatio, dTravelUnitStartPos,
                        dTravelUnitTargetPos, dTravelUnitMaxVelocity, dTravelUnitMaxAcceleration, dTravelUnitMaxJerk, dQuickStopMaxAcc);
        
        SKA_TUTP_Run(&(pVCFTP->stTravelUnitTracePlanner), 0.0, &(pVCFTP->dX), &(pVCFTP->dDx), &(pVCFTP->dDDx), &(dTmpUnsRefPos), &(dTmpUnsRefVel), &(dTmpUnsRefAcc));
        SKA_TUTP_GetTotalTime(&(pVCFTP->stTravelUnitTracePlanner), &(pVCFTP->dhi));        
    }

    while (pVCFTP->dhi - pVCFTP->dlo > SKA_FlOAT_ERROR)
    {
        pVCFTP->dmid = (pVCFTP->dlo + pVCFTP->dhi) / 2.0;
        pVCFTP->dX = 0.0;
        pVCFTP->dDx = 0.0;
        pVCFTP->dDDx = 0.0;
        double dTmpUnsRefPos, dTmpUnsRefVel, dTmpUnsRefAcc;
        SKA_TUTP_Run(&(pVCFTP->stTravelUnitTracePlanner), pVCFTP->dmid, &(pVCFTP->dX), &(pVCFTP->dDx), &(pVCFTP->dDDx), &(dTmpUnsRefPos), &(dTmpUnsRefVel), &(dTmpUnsRefAcc));
        if ((dTravelUnitTargetPos - dTravelUnitStartPos) * (dTravelUnitFindPos - pVCFTP->dX) >= 0.0)
        {
            pVCFTP->dlo = pVCFTP->dmid;
        }
        else
        {
            pVCFTP->dhi = pVCFTP->dmid;
        } 
    }
    *pTravelUnitMotionTime = (pVCFTP->dlo + pVCFTP->dhi) / 2.0;
    SKA_TUTP_Destroy(&(pVCFTP->stTravelUnitTracePlanner));

    /******************** 计算起升机构轨迹时间 ********************/
    if (dHoistUnitFindPos < dHoistUnitStartPos && dHoistUnitFindPos < dHoistUnitTargetPos ||
        dHoistUnitFindPos > dHoistUnitStartPos && dHoistUnitFindPos > dHoistUnitTargetPos)
    {
        *pHoistUnitMotionTime = -1.0;
        return 0;
    }
    pVCFTP->dUlo = 0.0;
    SKA_HUTP_Create(&(pVCFTP->stHoistUnitTracePlanner), dHoistUnitStartPos, dHoistUnitTargetPos, dHoistUnitMaxVelocity, dHoistUnitMaxAcceleration, dHoistUnitMaxJerk);
 
    SKA_HUTP_Run(&(pVCFTP->stHoistUnitTracePlanner), 0.0, &(pVCFTP->dUx), &(pVCFTP->dUdx), &(pVCFTP->dUddx));
    SKA_HUTP_GetTotalTime(&(pVCFTP->stHoistUnitTracePlanner), &(pVCFTP->dUhi));

    while (pVCFTP->dUhi - pVCFTP->dUlo > SKA_FlOAT_ERROR)
    {
        pVCFTP->dUmid = (pVCFTP->dUlo + pVCFTP->dUhi) / 2.0;
        SKA_HUTP_Run(&(pVCFTP->stHoistUnitTracePlanner), pVCFTP->dUmid, &(pVCFTP->dUx), &(pVCFTP->dUdx), &(pVCFTP->dUddx));
        if ((dHoistUnitTargetPos - dHoistUnitStartPos) * (dHoistUnitFindPos - pVCFTP->dUx) >= 0.0)
        {
            pVCFTP->dUlo = pVCFTP->dUmid;
        }
        else
        {
            pVCFTP->dUhi = pVCFTP->dUmid;
        }
    }
    *pHoistUnitMotionTime = (pVCFTP->dUlo + pVCFTP->dUhi) / 2.0;
    SKA_HUTP_Destroy(&(pVCFTP->stHoistUnitTracePlanner));
    return 0;
}