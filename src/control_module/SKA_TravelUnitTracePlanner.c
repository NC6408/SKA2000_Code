/**
 ******************************************************************************
 * @file       : SKA_TravelUnitTracePlanner.c
 * @brief      : 运行机构轨迹规划器
 * @author     : ZhangYi
 * @version    : None
 * @date       : 2024/10/24
 ******************************************************************************
 */

#include <stddef.h>
#include <math.h>
#include <stdio.h>

#include "SKA_TravelUnitTracePlanner.h"

int8_t SKA_TUTP_Create(SKA_TravelUnitTracePlanner *pTUTP, bool bActiveSwayCtrl,
                       SKA_InputShaperType eType, double dNaturalFreq, double dDampingRatio,
                       double dStartPos, double dTargetPos, double dMaxVelocity, double dMaxAcceleration, double dMaxJerk,
                       double dQuickStopMaxAcc)
{
    /******************** 函数参数合法性检验 ********************/
    if (pTUTP == NULL || dNaturalFreq <= 0 || dDampingRatio < 0 || dDampingRatio >= 1 ||
        dMaxVelocity <= 0 || dMaxAcceleration <= 0 || dMaxJerk <= 0 || dQuickStopMaxAcc <= 0)
    {
        printf("Invalid parameters of SKA_TUTP_Create().\n");
        return -1;
    }

    /******************** 创建运行机构轨迹规划器 ********************/
    pTUTP->bIsQuickStopping = 0;
    pTUTP->dQuickStopMaxAcc = dQuickStopMaxAcc;
    pTUTP->bActiveSwayCtrl = bActiveSwayCtrl;
    if (bActiveSwayCtrl)
    {
        SKA_InpSha_Init(&pTUTP->stShaper, eType, dNaturalFreq, dDampingRatio);
    }
    SKA_UTP_Create(&pTUTP->stUnsPlanner, dStartPos, dTargetPos, dMaxVelocity, dMaxAcceleration, dMaxJerk,
                   pTUTP->stShaper.arrImpulseTime[pTUTP->stShaper.nImpulseNum - 1]);

    return 0;
}

int8_t SKA_TUTP_Run(SKA_TravelUnitTracePlanner *pTUTP, double dCurTime,
                    double *pRefPos, double *pRefVel, double *pRefAcc,
                    double *pUnsRefPos, double *pUnsRefVel, double *pUnsRefAcc)
{
    /******************** 函数参数合法性检验 ********************/
    if (pTUTP == NULL || pRefPos == NULL || pRefVel == NULL || pRefAcc == NULL ||
        pUnsRefPos == NULL || pUnsRefVel == NULL || pUnsRefAcc == NULL)
    {
        printf("Invalid parameters of SKA_TUTP_Run().\n");
        return -1;
    }

    /******************** 运行运行机构轨迹规划器 ********************/
    printf("bActiveHorizontalMode1111111111: %d\n", pTUTP->bActiveHorizontalMode);
    if (pTUTP->bActiveHorizontalMode == 0)
    {
    printf("no1\n");
        if (pTUTP->bIsQuickStopping)
        {
            // 无防摇急停
    printf("no2\n");
            SKA_QSTP_Run(&(pTUTP->stQSTP), dCurTime, pRefPos, pRefVel, pRefAcc);

            // 未整形轨迹无意义
            *pUnsRefPos = 0;
            *pUnsRefVel = 0;
            *pUnsRefAcc = 0;
        }
        else
        {
        printf("no3\n");
            // 未整形轨迹
            SKA_UTP_Run(&(pTUTP->stUnsPlanner), dCurTime, pUnsRefPos, pUnsRefVel, pUnsRefAcc);

            if (pTUTP->bActiveSwayCtrl)
            {
                // 整形轨迹
                double dRefX = 0.0f, dRefDx = 0.0f, dRefDdx = 0.0f;
                double dX, dDx, dDdx;
                for (int i = 0; i < pTUTP->stShaper.nImpulseNum; i++)
                {
                    SKA_UTP_Run(&(pTUTP->stUnsPlanner), dCurTime - pTUTP->stShaper.arrImpulseTime[i], &dX, &dDx, &dDdx);
                    dRefX += pTUTP->stShaper.arrImpulseValue[i] * dX;
                    dRefDx += pTUTP->stShaper.arrImpulseValue[i] * dDx;
                    dRefDdx += pTUTP->stShaper.arrImpulseValue[i] * dDdx;
                }
                *pRefPos = dRefX;
                *pRefVel = dRefDx;
                *pRefAcc = dRefDdx;
            }
            else
            {
                // 未激活防摇
                *pRefPos = *pUnsRefPos;
                *pRefVel = *pUnsRefVel;
                *pRefAcc = *pUnsRefAcc;
            }
        }
    }
    else
    {
        // 横向避让情况
        if ((pTUTP->bIsUseWaypoints == 1 && pTUTP->nWaypointsNum == 0) ||
            (pTUTP->bIsUseWaypoints == 0 && pTUTP->nObstacleNum == 0))
        {
            printf("no11111\n");
            SKA_UTP_Run(&(pTUTP->stUnsPlanner), dCurTime, pUnsRefPos, pUnsRefVel, pUnsRefAcc);
        }
        else if (pTUTP->bIsUseWaypoints == 0 && pTUTP->nObstacleNum == 1)
        {
            // 单个障碍物未整形轨迹
            printf("0322\n");
            SKA_UTP_ObstacleUnshapedTrace(&(pTUTP->stUnsPlanner), dCurTime, pUnsRefPos, pUnsRefVel, pUnsRefAcc);
            printf("0333\n");
        }
        else
        {
            // 多个障碍物未整形轨迹
            SKA_UTP_ObstacleUnshapedTrace(&(pTUTP->stUnsPlanner), dCurTime, pUnsRefPos, pUnsRefVel, pUnsRefAcc);
        }
printf("no333333\n");
        if (pTUTP->bActiveSwayCtrl)
        {
            // 整形轨迹
            double dRefX = 0.0f, dRefDx = 0.0f, dRefDdx = 0.0f;
            double dX, dDx, dDdx;
            for (int i = 0; i < pTUTP->stShaper.nImpulseNum; i++)
            {

                if ((pTUTP->bIsUseWaypoints == 1 && pTUTP->nWaypointsNum == 0) ||
                    (pTUTP->bIsUseWaypoints == 0 && pTUTP->nObstacleNum == 0))
                {
                    SKA_UTP_Run(&(pTUTP->stUnsPlanner), dCurTime - pTUTP->stShaper.arrImpulseTime[i], &dX, &dDx, &dDdx);
                }
                else if (pTUTP->bIsUseWaypoints == 0 && pTUTP->nObstacleNum == 1)
                // 有障碍物未整形轨迹
                {
                    SKA_UTP_ObstacleUnshapedTrace(&(pTUTP->stUnsPlanner), dCurTime - pTUTP->stShaper.arrImpulseTime[i], &dX, &dDx, &dDdx);
                }
                else
                {
                    SKA_UTP_ObstacleUnshapedTrace(&(pTUTP->stUnsPlanner), dCurTime - pTUTP->stShaper.arrImpulseTime[i], &dX, &dDx, &dDdx);
                }

                dRefX += pTUTP->stShaper.arrImpulseValue[i] * dX;
                dRefDx += pTUTP->stShaper.arrImpulseValue[i] * dDx;
                dRefDdx += pTUTP->stShaper.arrImpulseValue[i] * dDdx;
            }
            *pRefPos = dRefX;
            *pRefVel = dRefDx;
            *pRefAcc = dRefDdx;
        }
        else
        {
            // 未激活防摇
            *pRefPos = *pUnsRefPos;
            *pRefVel = *pUnsRefVel;
            *pRefAcc = *pUnsRefAcc;
        }
    }
    return 0;
}

int8_t SKA_TUTP_SingleOGetShapedTraceAndTotalTime(SKA_TravelUnitTracePlanner *pTUTP, double dCurTime, double dStartPos, double dTargetPos, bool bActiveSwayCtrl,
                                                   SKA_InputShaperType eType, double dNaturalFreq, double dDampingRatio, double dMaxVel, double dMaxAcc, double dMaxJerk,
                                                   double dQuickStopMaxAcc, double *pPos, double *pVel, double *pAcc, double *pTotaltime)
{
    if (pTUTP == NULL || dNaturalFreq <= 0 || dDampingRatio < 0 || dDampingRatio >= 1 ||
        dMaxVel <= 0 || dMaxAcc <= 0 || dMaxJerk <= 0 || dQuickStopMaxAcc <= 0 ||
        pPos == NULL || pVel == NULL || pAcc == NULL || pTotaltime == NULL)
    {

        printf("Invalid parameters of SKA_TUTP_SingleOGetShapedTraceAndTotalTime().\n");
        return -1;
    }
    if (fabs(dStartPos - dTargetPos) < SKA_FlOAT_ERROR)
    {
        *pPos = dTargetPos;
        *pVel = 0.0;
        *pAcc = 0.0;
        *pTotaltime = 0.0;
    }
    else
    {
        *pPos = 0.0;
        *pVel = 0.0;
        *pAcc = 0.0;
        double dTmpUnsRefPos, dTmpUnsRefVel, dTmpUnsRefAcc;
        pTUTP->bActiveHorizontalMode = 1;
        SKA_TUTP_Create(pTUTP, bActiveSwayCtrl, eType, dNaturalFreq, dDampingRatio, dStartPos,
                        dTargetPos, dMaxVel, dMaxAcc, dMaxJerk, dQuickStopMaxAcc);
        SKA_TUTP_Run(pTUTP, dCurTime, pPos, pVel, pAcc, &(dTmpUnsRefPos), &(dTmpUnsRefVel), &(dTmpUnsRefAcc));
        SKA_TUTP_GetTotalTime(pTUTP, pTotaltime);
    }
    SKA_TUTP_Destroy(pTUTP);

    return 0;
}
int8_t SKA_TUTP_GetTotalTime(SKA_TravelUnitTracePlanner *pTUTP, double *pTotalTime)
{
    /******************** 函数参数合法性检验 ********************/
    if (pTUTP == NULL || pTotalTime == NULL)
    {
        printf("Invalid parameters of SKA_TUTP_GetTotalTime().\n");
        return -1;
    }

    /******************** 获取运行机构轨迹规划器总时间 ********************/
    if (pTUTP->bIsQuickStopping)
    {
        // 无防摇急停
        SKA_QSTP_GetTotalTime(&(pTUTP->stQSTP), pTotalTime);
    }
    else
    {
        double dUnsTotalTime;
        SKA_UTP_GetTotalTime(&(pTUTP->stUnsPlanner), &dUnsTotalTime);
        if (pTUTP->bActiveSwayCtrl)
        {
            if (dUnsTotalTime < SKA_FlOAT_ERROR)
            {
                *pTotalTime = 0.0;
            }
            else
            {
                *pTotalTime = dUnsTotalTime + pTUTP->stShaper.arrImpulseTime[pTUTP->stShaper.nImpulseNum - 1];
            }
        }
        else
        {
            *pTotalTime = dUnsTotalTime;
        }
    }

    return 0;
}

int8_t SKA_TUTP_ModifyTargetPos(SKA_TravelUnitTracePlanner *pTUTP, double dCurTime, double dNewTargetPos)
{
    /******************** 函数参数合法性检验 ********************/
    if (pTUTP == NULL)
    {
        printf("Invalid parameters of SKA_TUTP_ModifyTargetPos().\n");
        return -1;
    }

    /******************** 目标位置切换 ********************/
    SKA_UTP_ModifyTargetPos(&(pTUTP->stUnsPlanner), dCurTime, dNewTargetPos);

    return 0;
}

int8_t SKA_TUTP_AntiswayStop(SKA_TravelUnitTracePlanner *pTUTP, double dCurTime)
{
    /******************** 函数参数合法性检验 ********************/
    if (pTUTP == NULL)
    {
        printf("Invalid parameters of SKA_TUTP_AntiswayStop().\n");
        return -1;
    }

    /******************** 紧急停止 ********************/
    SKA_UTP_UrgencyStop(&(pTUTP->stUnsPlanner), dCurTime);

    return 0;
}

int8_t SKA_TUTP_GetAscBrakeDistance(SKA_TravelUnitTracePlanner *pTUTP, double dCurTime, double *pAscBrakeDistance)
{
    /******************** 函数参数合法性检验 ********************/
    if (pTUTP == NULL || pAscBrakeDistance == NULL)
    {
        printf("Invalid parameters of SKA_TUTP_GetAscBrakeDistance().\n");
        return -1;
    }

    /******************** 计算制动距离 ********************/

    // 当前参考位置
    double dCurPos, dCurVel, dCurAcc, dCurUnsPos, dCurUnsVel, dCurUnsAcc;
    SKA_TUTP_Run(pTUTP, dCurTime, &dCurPos, &dCurVel, &dCurAcc, &dCurUnsPos, &dCurUnsVel, &dCurUnsAcc);
    // 制动目标位置
    double dBrakeTargetPos;
    if (pTUTP->bIsQuickStopping || pTUTP->stUnsPlanner.bIsUrgencyStopping)
    {
        // 已触发无防摇急停或带防摇缓停
        SKA_TUTP_GetRealTargetPos(pTUTP, &dBrakeTargetPos);
    }
    else
    {
        // 预估无防摇急停的制动距离
        SKA_UTP_GetUrgencyStopPos(&(pTUTP->stUnsPlanner), dCurTime, &dBrakeTargetPos);
    }
    *pAscBrakeDistance = fabs(dBrakeTargetPos - dCurPos);

    return 0;
}

int8_t SKA_TUTP_GetRealTargetPos(SKA_TravelUnitTracePlanner *pTUTP, double *pRealTargetPos)
{
    /******************** 函数参数合法性检验 ********************/
    if (pTUTP == NULL || pRealTargetPos == NULL)
    {
        printf("Invalid parameters of SKA_TUTP_GetRealTargetPos().\n");
        return -1;
    }

    /******************** 获取实际目标位置 ********************/

    if (pTUTP->bIsQuickStopping)
    {
        // 无防摇急停中
        SKA_QSTP_GetTargetPos(&(pTUTP->stQSTP), pRealTargetPos);
    }
    else
    {
        SKA_UTP_GetRealTargetPos(&(pTUTP->stUnsPlanner), pRealTargetPos);
    }

    return 0;
}

int8_t SKA_TUTP_QuickStop(SKA_TravelUnitTracePlanner *pTUTP, double dCurTime)
{
    /******************** 函数参数合法性检验 ********************/
    if (pTUTP == NULL || pTUTP->bIsQuickStopping)
    {
        printf("Invalid parameters of SKA_TUTP_QuickStop().\n");
        return -1;
    }

    /******************** 无防摇急停 ********************/

    double dCurPos, dCurVel, dCurAcc, dCurUnsPos, dCurUnsVel, dCurUnsAcc;
    SKA_TUTP_Run(pTUTP, dCurTime, &dCurPos, &dCurVel, &dCurAcc, &dCurUnsPos, &dCurUnsVel, &dCurUnsAcc);

    pTUTP->bIsQuickStopping = 1;
    SKA_QSTP_Create(&(pTUTP->stQSTP), dCurTime, dCurPos, dCurVel, pTUTP->dQuickStopMaxAcc);

    return 0;
}

int8_t SKA_TUTP_ModifyMaxVelocity(SKA_TravelUnitTracePlanner *pTUTP, double dCurTime, double dNewMaxVelocity)
{
    /******************** 函数参数合法性检验 ********************/
    if (pTUTP == NULL || dNewMaxVelocity <= 0)
    {
        printf("Invalid parameters of SKA_TUTP_ModifyMaxVelocity().\n");
        return -1;
    }

    /******************** 修改最大速度 ********************/
    // TODO

    return 0;
}

int8_t SKA_TUTP_Destroy(SKA_TravelUnitTracePlanner *pTUTP)
{
    /******************** 函数参数合法性检验 ********************/
    if (pTUTP == NULL)
    {
        printf("Invalid parameters of SKA_TUTP_Destroy().\n");
        return -1;
    }

    /******************** 销毁运行机构轨迹规划器 ********************/
    SKA_UTP_Destroy(&pTUTP->stUnsPlanner);
    if (pTUTP->bIsQuickStopping)
    {
        SKA_QSTP_Destroy(&(pTUTP->stQSTP));
    }

    return 0;
}