/**
 ******************************************************************************
 * @file       : SKA_UnshapedTracePlanner.c.c
 * @brief      : 未整形轨迹规划器
 * @author     : ZhangYi
 * @version    : None
 * @date       : 2024/10/29
 ******************************************************************************
 */
//

#include <stddef.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>

#include "SKA_UnshapedTracePlanner.h"

#include "SKA_Constants.h"
#include "SKA_Utils.h"

int8_t SKA_UTP_Create(SKA_UnshapedTracePlanner *pUTP, double dStartPos, double dTargetPos,
                      double dMaxVelocity, double dMaxAcceleration, double dMaxJerk, double dBufferTime)
{
    /******************** 函数参数合法性检验 ********************/
    if (pUTP == NULL || dMaxVelocity <= 0 || dMaxAcceleration <= 0 || dMaxJerk <= 0 || dBufferTime < 0)
    {
        printf("Invalid parameters of SKA_UTP_Create().\n");
        return -1;
    }

    /******************** 创建 ********************/

    pUTP->dStartPos = dStartPos;
    pUTP->dTargetPos = dTargetPos;
    pUTP->dMaxVelocity = dMaxVelocity;
    pUTP->dMaxAcceleration = dMaxAcceleration;
    pUTP->dMaxJerk = dMaxJerk;
    pUTP->dBufferTime = dBufferTime;
    pUTP->bIsUrgencyStopping = 0;
    SKA_DST_GetZero2ZeroTrace(fabs(pUTP->dStartPos - pUTP->dTargetPos), pUTP->dMaxVelocity, pUTP->dMaxAcceleration,
                              pUTP->dMaxJerk, &(pUTP->pTraceHead));

    // 校正方向
    double dDirection;
    if (pUTP->dTargetPos >= pUTP->dStartPos)
    {
        dDirection = 1.0;
    }
    else
    {
        dDirection = -1.0;
    }
    SKA_TraceSegment *p = pUTP->pTraceHead->pNext;
    while (p != pUTP->pTraceHead)
    {
        p->dX0 = pUTP->dStartPos + dDirection * p->dX0;
        p->dV0 = dDirection * p->dV0;
        p->dA0 = dDirection * p->dA0;
        p->dJerk = dDirection * p->dJerk;

        p = p->pNext;
    }
    return 0;
}

int8_t SKA_UTP_Run(const SKA_UnshapedTracePlanner *pUTP, double dCurTime,
                   double *pUnsRefPos, double *pUnsRefVelocity, double *pUnsRefAcceleration)
{
    /******************** 函数参数合法性检验 ********************/
    if (pUTP == NULL || pUnsRefPos == NULL || pUnsRefVelocity == NULL || pUnsRefAcceleration == NULL)
    {
        printf("Invalid parameters of 1SKA_UTP_Run().\n");
        return -1;
    }
    if (pUTP->pTraceHead == NULL || pUTP->pTraceHead->pNext == pUTP->pTraceHead)
    {
        printf("Invalid parameters of 2SKA_UTP_Run().\n");
        return -1;
    }

    /******************** 计算轨迹点 ********************/

    // 小于0的时刻默认为静止在开始位置
    if (dCurTime <= 0.0)
    {
        *pUnsRefPos = pUTP->dStartPos;
        *pUnsRefVelocity = 0.0;
        *pUnsRefAcceleration = 0.0;
        return 0;
    }

    // 寻找轨迹段
    SKA_TraceSegment *pCurSeg = pUTP->pTraceHead->pNext;
    while (pCurSeg->pNext != pUTP->pTraceHead && dCurTime >= pCurSeg->pNext->dT0)
    {
        pCurSeg = pCurSeg->pNext;
    }

    // 计算轨迹点
    SKA_Trace_GetTracePoint(dCurTime, pCurSeg, pUnsRefPos, pUnsRefVelocity, pUnsRefAcceleration);

    return 0;
}

int8_t SKA_UTP_GetTotalTime(const SKA_UnshapedTracePlanner *pUTP, double *pTotalTime)
{
    /******************** 函数参数合法性检验 ********************/
    if (pUTP == NULL || pTotalTime == NULL)
    {
        printf("Invalid parameters of SKA_UTP_GetTotalTime().\n");
        return -1;
    }
    if (pUTP->pTraceHead == NULL || pUTP->pTraceHead->pNext == pUTP->pTraceHead)
    {
        printf("Invalid parameters of SKA_UTP_GetTotalTime().\n");
        return -1;
    }

    /******************** 计算总时间 ********************/
    SKA_Trace_GetTotalTime(pUTP->pTraceHead, pTotalTime);

    return 0;
}

int8_t SKA_UTP_ObstacleUnshapedTrace(SKA_UnshapedTracePlanner *pUTP, double dCurTime,
                                     double *pUnsRefPos, double *pUnsRefVelocity, double *pUnsRefAcceleration)
{
    /******************** 函数参数合法性检验 ********************/
    if (pUTP == NULL || pUnsRefPos == NULL || pUnsRefVelocity == NULL || pUnsRefAcceleration == NULL)
    {
        printf("Invalid parameters of SKA_UTP_ObstacleUnshapedTrace().\n");
        return -1;
    }
    double dStartTime;
    if ((pUTP->bIsUseWaypoints == 0 && pUTP->nObstacleNum == 1))
    {
        dStartTime = pUTP->SOSubTasks[pUTP->ntasklndex + 1].dStartTime;
    }
    if ((pUTP->bIsUseWaypoints == 1 && pUTP->nWaypointsNum > 0) || (pUTP->bIsUseWaypoints == 0 && pUTP->nObstacleNum > 1))
    {
        dStartTime = pUTP->MOSubTasks[pUTP->ntasklndex + 1].dStartTime;
    }

    pUTP->ntasklndex = 0;
    while (pUTP->ntasklndex < pUTP->nSubTasksNum - 1)
    {
        if (dCurTime >= dStartTime)
        {
            pUTP->ntasklndex = pUTP->ntasklndex + 1;
        }
        else
        {
            break;
        }
    }
    if (pUTP->bIsUseWaypoints == 0 && pUTP->nObstacleNum == 1)
    {
        SKA_UTP_GetSingleOUnshapedTrace(pUTP, (dCurTime - pUTP->SOSubTasks[pUTP->ntasklndex].dStartTime),
                                          pUTP->SOSubTasks[pUTP->ntasklndex].dStartPos, pUTP->SOSubTasks[pUTP->ntasklndex].dTargetPos,
                                          pUTP->dMaxVelocity, pUTP->dMaxAcceleration, pUTP->dMaxJerk, pUnsRefPos, pUnsRefVelocity,
                                          pUnsRefAcceleration, &(pUTP->dtotalTime));
    }
    if ((pUTP->bIsUseWaypoints == 1 && pUTP->nWaypointsNum > 0) || (pUTP->bIsUseWaypoints == 0 && pUTP->nObstacleNum > 1))
    {

        SKA_UTP_GetMultipleOUnshapedTrace(pUTP, (dCurTime - pUTP->MOSubTasks[pUTP->ntasklndex].dStartTime),
                                            pUTP->MOSubTasks[pUTP->ntasklndex].dStartPos, pUTP->MOSubTasks[pUTP->ntasklndex].dTargetPos,
                                            pUTP->MOSubTasks[pUTP->ntasklndex].dStartVelocity, pUTP->MOSubTasks[pUTP->ntasklndex].dTargetVelocity,
                                            pUTP->bActiveSwayCtrl, pUTP->eType, pUTP->dNaturalFreq, pUTP->dDampingRatio,
                                            pUTP->MOSubTasks[pUTP->ntasklndex].dMinTime, pUTP->dMaxVelocity, pUTP->dMaxAcceleration, pUTP->dMaxJerk,
                                            pUTP->InputShaper, pUnsRefPos, pUnsRefVelocity, pUnsRefAcceleration, &(pUTP->dtotalTime), &(pUTP->bisUniform));
    }

    return 0;
}

int8_t SKA_UTP_ModifyTargetPos(SKA_UnshapedTracePlanner *pUTP, double dCurTime, double dNewTargetPos)
{
    /******************** 函数参数合法性检验 ********************/
    if (pUTP == NULL || pUTP->bIsUrgencyStopping == 1 || pUTP->pTraceHead == NULL || pUTP->pTraceHead->pNext == pUTP->pTraceHead)
    {
        printf("Invalid parameters of SKA_UTP_ModifyTargetPos().\n");
        return -1;
    }

    /******************** 修改目标位置 ********************/

    // 更新目标位置
    pUTP->dTargetPos = dNewTargetPos;

    // 寻找轨迹段
    SKA_TraceSegment *pCurSeg = pUTP->pTraceHead->pNext;
    while (pCurSeg->pNext != pUTP->pTraceHead && dCurTime >= pCurSeg->pNext->dT0)
    {
        pCurSeg = pCurSeg->pNext;
    }

    // 选择切换时刻
    // 只允许在加速度取值为0、am、-am处切换
    double dStartT, dStartX, dStartV, dStartA, dDirection;
    if (dCurTime <= 0.0)
    {
        dStartT = 0.0;
        dStartX = pUTP->dStartPos;
        dStartV = 0.0;
        dStartA = 0.0;
    }
    else
    {
        if (pCurSeg->dJerk == 0.0)
        {
            dStartT = dCurTime;
            SKA_Trace_GetTracePoint(dStartT, pCurSeg, &dStartX, &dStartV, &dStartA);
        }
        else
        {
            while (pCurSeg->pNext != pUTP->pTraceHead)
            {
                SKA_Trace_GetTracePoint(pCurSeg->pNext->dT0, pCurSeg, &dStartX, &dStartV, &dStartA);
                if (fabs(dStartA) < SKA_FlOAT_ERROR || fabs(fabs(dStartA) - pUTP->dMaxAcceleration) < SKA_FlOAT_ERROR)
                {
                    dStartT = pCurSeg->pNext->dT0;
                    break;
                }
                else
                {
                    pCurSeg = pCurSeg->pNext;
                }
            }
        }
    }
    if (dStartV >= 0.0)
    {
        dDirection = 1.0;
    }
    else
    {
        dDirection = -1.0;
    }

    // 生成目标位置切换轨迹
    SKA_TraceSegment *pHeadChange;
    SKA_DST_GetNonzero2ZeroTrace(dDirection * dStartV, dDirection * dStartA, dDirection * (dNewTargetPos - dStartX),
                                 pUTP->dMaxVelocity, pUTP->dMaxAcceleration, pUTP->dMaxJerk, pUTP->dBufferTime, &pHeadChange);

    // 加入缓冲过程、校正方向
    uint16_t nChangeTraceNum;
    SKA_Trace_GetTraceSegmentNum(pHeadChange, &nChangeTraceNum);

    if (nChangeTraceNum >= 2 && fabs(pHeadChange->pNext->dJerk) < SKA_FlOAT_ERROR && fabs(pHeadChange->pNext->dA0) < SKA_FlOAT_ERROR && fabs(pHeadChange->pNext->dV0) < SKA_FlOAT_ERROR && fabs(pHeadChange->pNext->pNext->dT0 - pUTP->dBufferTime) < SKA_FlOAT_ERROR)
    {
        if (fabs(pCurSeg->dJerk) < SKA_FlOAT_ERROR && fabs(pCurSeg->dA0) < SKA_FlOAT_ERROR && fabs(pCurSeg->dV0) < SKA_FlOAT_ERROR)
        {
            if (pCurSeg->pPrev == pUTP->pTraceHead || dDirection * pCurSeg->pPrev->dJerk * pHeadChange->pNext->pNext->dJerk > 0.0)
            {
                // 速度符号不变，无需加入缓冲阶段
                dStartT -= pUTP->dBufferTime;

                SKA_TraceSegment *tmp = pHeadChange->pNext;
                pHeadChange->pNext = pHeadChange->pNext->pNext;
                pHeadChange->pNext->pPrev = pHeadChange;
                free(tmp);
            }
            else
            {
                // 速度符号改变，修改原轨迹的缓冲阶段，无需加入缓冲阶段
                SKA_Max(dStartT, pCurSeg->dT0 + pUTP->dBufferTime, &dStartT);
                dStartT -= pUTP->dBufferTime;

                SKA_TraceSegment *tmp = pHeadChange->pNext;
                pHeadChange->pNext = pHeadChange->pNext->pNext;
                pHeadChange->pNext->pPrev = pHeadChange;
                free(tmp);
            }
        }
        else
        {
            if (dDirection * pCurSeg->dJerk * pHeadChange->pNext->pNext->dJerk > 0.0)
            {
                // 速度符号不变，无需加入缓冲阶段
                dStartT -= pUTP->dBufferTime;

                SKA_TraceSegment *tmp = pHeadChange->pNext;
                pHeadChange->pNext = pHeadChange->pNext->pNext;
                pHeadChange->pNext->pPrev = pHeadChange;
                free(tmp);
            }
            else
            {
                // 速度符号改变，需要加入缓冲阶段
            }
        }
    }
    else
    {
        // 目标切换轨迹无缓冲阶段
    }
    SKA_TraceSegment *p = pHeadChange->pNext;
    while (p != pHeadChange)
    {
        p->dT0 = dStartT + p->dT0;
        p->dX0 = dStartX + dDirection * p->dX0;
        p->dV0 = dDirection * p->dV0;
        p->dA0 = dDirection * p->dA0;
        p->dJerk = dDirection * p->dJerk;

        p = p->pNext;
    }

    // 删除旧轨迹
    pUTP->pTraceHead->pPrev->pNext = NULL;
    SKA_Trace_FreeSinglyLinkedListTrace(pCurSeg->pNext);

    // 合并新轨迹
    if (pHeadChange->pNext != pHeadChange)
    {
        pCurSeg->pNext = pHeadChange->pNext;
        pHeadChange->pNext->pPrev = pCurSeg;
        pHeadChange->pPrev->pNext = pUTP->pTraceHead;
        pUTP->pTraceHead->pPrev = pHeadChange->pPrev;
    }
    free(pHeadChange);

    return 0;
}

int8_t SKA_UTP_UrgencyStop(SKA_UnshapedTracePlanner *pUTP, double dCurTime)
{
    /******************** 函数参数合法性检验 ********************/
    if (pUTP == NULL || pUTP->bIsUrgencyStopping == 1 || pUTP->pTraceHead == NULL || pUTP->pTraceHead->pNext == pUTP->pTraceHead)
    {
        printf("Invalid parameters of SKA_UTP_UrgencyStop().\n");
        return -1;
    }

    /******************** 急停 ********************/

    // 设置急停标志
    pUTP->bIsUrgencyStopping = 1;

    // 寻找轨迹段
    SKA_TraceSegment *pCurSeg = pUTP->pTraceHead->pNext;
    while (pCurSeg->pNext != pUTP->pTraceHead && dCurTime >= pCurSeg->pNext->dT0)
    {
        pCurSeg = pCurSeg->pNext;
    }

    // 选择急停时刻
    // 只允许在加速度取值为0、am、-am处急停
    double dStartT, dStartX, dStartV, dStartA, dDirection;
    if (dCurTime <= 0.0)
    {
        dStartT = 0.0;
        dStartX = pUTP->dStartPos;
        dStartV = 0.0;
        dStartA = 0.0;
    }
    else
    {
        if (pCurSeg->dJerk == 0.0)
        {
            dStartT = dCurTime;
            SKA_Trace_GetTracePoint(dStartT, pCurSeg, &dStartX, &dStartV, &dStartA);
        }
        else
        {
            while (pCurSeg->pNext != pUTP->pTraceHead)
            {
                SKA_Trace_GetTracePoint(pCurSeg->pNext->dT0, pCurSeg, &dStartX, &dStartV, &dStartA);
                if (fabs(dStartA) < SKA_FlOAT_ERROR || fabs(fabs(dStartA) - pUTP->dMaxAcceleration) < SKA_FlOAT_ERROR)
                {
                    dStartT = pCurSeg->pNext->dT0;
                    break;
                }
                else
                {
                    pCurSeg = pCurSeg->pNext;
                }
            }
        }
    }
    if (dStartV >= 0.0)
    {
        dDirection = 1.0;
    }
    else
    {
        dDirection = -1.0;
    }

    // 生成急停轨迹
    SKA_TraceSegment *pHeadStop;
    SKA_DST_GetUrgencyStopTrace(dDirection * dStartV, dDirection * dStartA, pUTP->dMaxAcceleration, pUTP->dMaxJerk, &pHeadStop);
    //    printf("Urgency Stop Trace\n");
    //    SKA_UTP_DisplayTraceLinkedList(pHeadStop);

    // 校正方向
    SKA_TraceSegment *p = pHeadStop->pNext;
    while (p != pHeadStop)
    {
        p->dT0 = dStartT + p->dT0;
        p->dX0 = dStartX + dDirection * p->dX0;
        p->dV0 = dDirection * p->dV0;
        p->dA0 = dDirection * p->dA0;
        p->dJerk = dDirection * p->dJerk;

        p = p->pNext;
    }

    // 删除旧轨迹
    pUTP->pTraceHead->pPrev->pNext = NULL;
    SKA_Trace_FreeSinglyLinkedListTrace(pCurSeg->pNext);
    // 合并新轨迹
    pCurSeg->pNext = pHeadStop->pNext;
    pHeadStop->pNext->pPrev = pCurSeg;
    pHeadStop->pPrev->pNext = pUTP->pTraceHead;
    pUTP->pTraceHead->pPrev = pHeadStop->pPrev;
    free(pHeadStop);

    return 0;
}

int8_t SKA_UTP_GetUrgencyStopPos(SKA_UnshapedTracePlanner *pUTP, double dCurTime, double *pTargetPos)
{
    /******************** 函数参数合法性检验 ********************/
    if (pUTP == NULL || pUTP->bIsUrgencyStopping == 1 || pUTP->pTraceHead == NULL || pUTP->pTraceHead->pNext == pUTP->pTraceHead)
    {
        printf("Invalid parameters of SKA_UTP_GetUrgencyStopPos().\n");
        return -1;
    }

    /******************** 获取急停停止位置 ********************/

    // 寻找轨迹段
    SKA_TraceSegment *pCurSeg = pUTP->pTraceHead->pNext;
    while (pCurSeg->pNext != pUTP->pTraceHead && dCurTime >= pCurSeg->pNext->dT0)
    {
        pCurSeg = pCurSeg->pNext;
    }

    // 选择急停时刻
    // 只允许在加速度取值为0、am、-am处急停
    double dStartT, dStartX, dStartV, dStartA, dDirection;
    if (dCurTime <= 0.0)
    {
        dStartT = 0.0;
        dStartX = pUTP->dStartPos;
        dStartV = 0.0;
        dStartA = 0.0;
    }
    else
    {
        if (pCurSeg->dJerk == 0.0)
        {
            dStartT = dCurTime;
            SKA_Trace_GetTracePoint(dStartT, pCurSeg, &dStartX, &dStartV, &dStartA);
        }
        else
        {
            while (pCurSeg->pNext != pUTP->pTraceHead)
            {
                SKA_Trace_GetTracePoint(pCurSeg->pNext->dT0, pCurSeg, &dStartX, &dStartV, &dStartA);
                if (fabs(dStartA) < SKA_FlOAT_ERROR || fabs(fabs(dStartA) - pUTP->dMaxAcceleration) < SKA_FlOAT_ERROR)
                {
                    dStartT = pCurSeg->pNext->dT0;
                    break;
                }
                else
                {
                    pCurSeg = pCurSeg->pNext;
                }
            }
        }
    }
    if (dStartV >= 0.0)
    {
        dDirection = 1.0;
    }
    else
    {
        dDirection = -1.0;
    }

    // 生成急停轨迹
    SKA_TraceSegment *pHeadStop;
    SKA_DST_GetUrgencyStopTrace(dDirection * dStartV, dDirection * dStartA, pUTP->dMaxAcceleration, pUTP->dMaxJerk, &pHeadStop);
    //    printf("Urgency Stop Trace\n");
    //    SKA_UTP_DisplayTraceLinkedList(pHeadStop);

    // 校正方向
    SKA_TraceSegment *p = pHeadStop->pNext;
    while (p != pHeadStop)
    {
        p->dT0 = dStartT + p->dT0;
        p->dX0 = dStartX + dDirection * p->dX0;
        p->dV0 = dDirection * p->dV0;
        p->dA0 = dDirection * p->dA0;
        p->dJerk = dDirection * p->dJerk;

        p = p->pNext;
    }

    // 获取目标位置
    SKA_Trace_GetTargetPos(pHeadStop, pTargetPos);

    // 删除轨迹
    pHeadStop->pPrev->pNext = NULL;
    SKA_Trace_FreeSinglyLinkedListTrace(pHeadStop);

    return 0;
}

int8_t SKA_UTP_GetRealTargetPos(const SKA_UnshapedTracePlanner *pUTP, double *pRealTargetPos)
{
    /******************** 函数参数合法性检验 ********************/
    if (pUTP == NULL || pRealTargetPos == NULL || pUTP->pTraceHead == NULL || pUTP->pTraceHead->pPrev == pUTP->pTraceHead)
    {
        printf("Invalid parameters of SKA_UTP_GetRealTargetPos().\n");
        return -1;
    }

    /******************** 获取实际目标位置 ********************/
    SKA_Trace_GetTargetPos(pUTP->pTraceHead, pRealTargetPos);

    return 0;
}

int8_t SKA_UTP_GetMultipleOUnshapedTrace(SKA_UnshapedTracePlanner *pUTP, double dCurTime, double dStartPos, double dTargetPos, double dStartVelocity,
                                           double dTargetVelocity, double dMinTime, bool bActiveSwayCtrl, SKA_InputShaperType eType,
                                           double dNaturalFreq, double dDampingRatio, double dMaxVelocity, double dMaxAcceleration, double dMaxJerk,
                                           SKA_InputShaper InputShaper, double *pPos, double *pVelocity,
                                           double *pAcceleration, double *pTotalTime, bool *bIsUniform)
{
    double dfx0, dtmp, dtmp1, dtmp2, dtmp3;
    bool btmp4;

    if (dMinTime < 0.0)
    {
        SKA_UTP_MultipleOGetNormalUnshapedTrace(pUTP, dCurTime, fabs(dTargetPos - dStartPos), dStartVelocity, dTargetVelocity,
                                                  bActiveSwayCtrl, eType, dNaturalFreq, dDampingRatio, dMaxVelocity,
                                                  dMaxAcceleration, dMaxJerk, InputShaper, pPos, pVelocity, pAcceleration, pTotalTime, bIsUniform);
        dfx0 = 0.0;
        for (int i = 0; i < InputShaper.nImpulseNum; i++)
        {
            SKA_UTP_MultipleOGetNormalUnshapedTrace(pUTP, 0.0 - InputShaper.arrImpulseTime[i], fabs(dTargetPos - dStartPos), dStartVelocity,
                                                      dTargetVelocity, bActiveSwayCtrl, eType, dNaturalFreq, dDampingRatio, dMaxVelocity, dMaxAcceleration, dMaxJerk, InputShaper,
                                                      &(dtmp), &(dtmp1), &(dtmp2), &(dtmp3), &(btmp4));
            dfx0 += InputShaper.arrImpulseValue[i] * dtmp;
        }
    }
    else
    {
        SKA_UTP_MultipleOGetNormalUnshapedTraceOverMinTime(pUTP, dCurTime, fabs(dTargetPos - dStartPos), dStartVelocity, dMinTime, dMaxVelocity,
                                                             dMaxAcceleration, dMaxJerk, InputShaper, pPos, pVelocity, pAcceleration, pTotalTime, bIsUniform);
        dfx0 = 0.0;
        for (int i = 0; i < InputShaper.nImpulseNum; i++)
        {
            SKA_UTP_MultipleOGetNormalUnshapedTraceOverMinTime(pUTP, 0.0 - InputShaper.arrImpulseTime[i], fabs(dTargetPos - dStartPos),
                                                                 dStartVelocity, dMinTime, dMaxVelocity, dMaxAcceleration, dMaxJerk,
                                                                 InputShaper, &(dtmp), &(dtmp1), &(dtmp2), &(dtmp3), &(btmp4));
            dfx0 += InputShaper.arrImpulseValue[i] * dtmp;
        }
    }

    if (dTargetPos >= dStartPos)
    {
        *pPos = dStartPos + (*pPos - dfx0);
    }
    else
    {
        *pPos = dStartPos - (*pPos - dfx0);
        *pVelocity = -(*pVelocity);
        *pAcceleration = -(*pAcceleration);
    }

    return 0;
}

int8_t SKA_UTP_MultipleOGetNormalUnshapedTrace(SKA_UnshapedTracePlanner *pUTP, double dCurTime, double dTargetPos, double dStartVelocity, double dTargetVelocity,
                                                 bool bActiveSwayCtrl, SKA_InputShaperType eType, double dNaturalFreq, double dDampingRatio, double dMaxVelocity,
                                                 double dMaxAcceleration, double dMaxJerk, SKA_InputShaper InputShaper, double *pPos,
                                                 double *pVelocity, double *pAcceleration, double *pTotalTime, bool *bIsUniform)
{

    double ddist1, ddist2, dminDist, dtmp;
    double dalpha, dtN, dvMid, dtMid;
    double dlo, dhi, dtt1, dtt2, dx0;
    // 未整形轨迹减速到0的最短距离和时间
    if (dStartVelocity < SKA_FlOAT_ERROR && dTargetVelocity < SKA_FlOAT_ERROR)
    {
        // 开始速度和目标速度都为0

        SKA_UTP_GetSingleOUnshapedTrace(pUTP, dCurTime, 0.0, dTargetPos, dMaxVelocity, dMaxAcceleration,
                                          dMaxJerk, pPos, pVelocity, pAcceleration, pTotalTime);
        *bIsUniform = 0;
    }
    else
    {
        // 开始速度不为0或目标速度不为0
        // 临界情况

        SKA_UTP_MultipleOGetUnshapedMinDistAndTime(pUTP, dStartVelocity, dMaxVelocity, dMaxAcceleration, dMaxJerk, &(ddist1), &(dtmp));
        SKA_UTP_MultipleOGetUnshapedMinDistAndTime(pUTP,dMaxVelocity, dTargetVelocity, dMaxAcceleration, dMaxJerk, &(ddist2), &(dtmp));

        dalpha = 0.0;
        for (int i = 0; i < InputShaper.nImpulseNum; i++)
        {
            dalpha += InputShaper.arrImpulseValue[i] * InputShaper.arrImpulseTime[i];
        }
        dtN = InputShaper.arrImpulseTime[InputShaper.nImpulseNum - 1];
        dminDist = ddist1 + ddist2 + dalpha * dStartVelocity + (dtN - dalpha) * dTargetVelocity;

        if (dTargetPos >= dminDist)
        {
            dvMid = dMaxVelocity;
            dtMid = (dTargetPos - dminDist) / dvMid;
        }
        else
        {
            dlo = fmax(dStartVelocity, dTargetVelocity);
            dhi = dMaxVelocity;

            while (dhi - dlo > SKA_FlOAT_ERROR)
            {
                dvMid = (dlo + dhi) / 2.0;
                SKA_UTP_MultipleOGetUnshapedMinDistAndTime(pUTP, dStartVelocity, dvMid, dMaxAcceleration, dMaxJerk, &(ddist1), &(dtmp));
                SKA_UTP_MultipleOGetUnshapedMinDistAndTime(pUTP, dvMid, dTargetVelocity, dMaxAcceleration, dMaxJerk, &(ddist2), &(dtmp));
                dminDist = ddist1 + ddist2 + dalpha * dStartVelocity + (dtN - dalpha) * dTargetVelocity;
                if (dminDist > dTargetPos)
                {
                    dhi = dvMid;
                }
                else
                {
                    dlo = dvMid;
                }
            }
            dvMid = (dlo + dhi) / 2.0;
            dtMid = 0.0;
        }

        SKA_UTP_MultipleOGetUnshapedMinDistAndTime(pUTP, dStartVelocity, dvMid, dMaxAcceleration, dMaxJerk, &(ddist1), &(dtt1));
        SKA_UTP_MultipleOGetUnshapedMinDistAndTime(pUTP, dvMid, dTargetVelocity, dMaxAcceleration, dMaxJerk, &(ddist2), &(dtt2));
        *pTotalTime = dtt1 + dtMid + dtt2;
        *bIsUniform = 0;

        if (dCurTime < dtt1)
        {
            SKA_UTP_MultipleOGetNormalUnshapedTraceSegment(pUTP, dCurTime, dStartVelocity, dvMid, dMaxAcceleration, dMaxJerk, pPos, pVelocity, pAcceleration);
        }
        else if (dCurTime < dtt1 + dtMid)
        {

            SKA_UTP_MultipleOGetNormalUnshapedTraceSegment(pUTP, dtt1, dStartVelocity, dvMid, dMaxAcceleration, dMaxJerk, &(dx0), pVelocity, pAcceleration);
            *pPos = dx0 + dvMid * (dCurTime - dtt1);
            *pVelocity = dvMid;
            *pAcceleration = 0.0;
        }
        else
        {
            SKA_UTP_MultipleOGetNormalUnshapedTraceSegment(pUTP, dtt1, dStartVelocity, dvMid, dMaxAcceleration, dMaxJerk, &(dx0), pVelocity, pAcceleration);
            dx0 += dvMid * dtMid;
            SKA_UTP_MultipleOGetNormalUnshapedTraceSegment(pUTP, dCurTime - dtt1 - dtMid, dvMid, dTargetVelocity,
                                                             dMaxAcceleration, dMaxJerk, pPos, pVelocity, pAcceleration);
            *pPos += dx0;
        }
    }
    return 0;
}



int8_t SKA_UTP_MultipleOGetNormalUnshapedTraceOverMinTime(SKA_UnshapedTracePlanner *pUTP, double dCurTime, double dTargetPos, 
                        double dStartVelocity, double dMinTime, double dMaxVelocity, double dMaxAcceleration, double dMaxJerk, 
                        SKA_InputShaper InputShaper, double* pPos, double* pVelocity, double* pAcceleration, double* pTotalTime, bool *bIsUniform)
                        {
                        
                            
    double dtN, dstep1Dist, dstep1Time, dvMid, dtMid, dlo, dhi, dtmp, dstep1TimeMax;
    double dt1, dt2, dx0;                        
   dtN = InputShaper.arrImpulseTime[InputShaper.nImpulseNum - 1];

    if (dStartVelocity > SKA_FlOAT_ERROR && dTargetPos >= dMinTime * dStartVelocity && dStartVelocity * dtN >= dTargetPos) {
        // 匀速运动
        *pPos = dStartVelocity * dCurTime;
        *pVelocity = dStartVelocity;
        *pAcceleration = 0.0f;
        *pTotalTime = dTargetPos / dStartVelocity;
        *bIsUniform = 1;
    } else {
        // 非匀速运动
        if (dTargetPos >= dMinTime * dStartVelocity) {
            // 加速、匀速、减速
            //计算最快轨迹
            SKA_UTP_MultipleOGetUnshapedMinDistAndTime(pUTP, dStartVelocity, dMaxVelocity, 
                                            dMaxAcceleration, dMaxJerk, &(dstep1Dist), &(dstep1Time));

            if (2 * dstep1Dist + dStartVelocity * dtN <= dTargetPos) {
                // 达到MaxVelocity
                dvMid = dMaxVelocity;
                dtMid = (dTargetPos - 2.0f * dstep1Dist - dStartVelocity * dtN) / dvMid;
                *pTotalTime = 2.0f * dstep1Time + dtMid + dtN;
            } else {
                // 未达到MaxVelocity
                dlo = dStartVelocity;
                dhi = dMaxVelocity;
                while (dhi - dlo > SKA_FlOAT_ERROR) {
                    dvMid = (dlo + dhi) / 2.0f;
                    SKA_UTP_MultipleOGetUnshapedMinDistAndTime(pUTP, dStartVelocity, dvMid, 
                                                dMaxAcceleration, dMaxJerk, &(dstep1Dist), &(dstep1Time));

                    dtmp = 2.0f * dstep1Dist + dStartVelocity * dtN;
                    if (dtmp > dTargetPos) {
                        dhi = dvMid;
                    } else {
                        dlo = dvMid;
                    }
                }
                dvMid = (dlo + dhi) / 2.0f;
                SKA_UTP_MultipleOGetUnshapedMinDistAndTime(pUTP, dStartVelocity, dvMid,
                                                dMaxAcceleration, dMaxJerk, &(dstep1Dist), &(dstep1Time));
                dtMid = 0.0f;
                *pTotalTime = 2.0f * dstep1Time + dtMid + dtN;
            }

            // 确保总时间不小于MinTime
            if (*pTotalTime < dMinTime) {
                dlo = dStartVelocity;
                dhi = dvMid;
                while (dhi - dlo > SKA_FlOAT_ERROR) {
                    dvMid = (dlo + dhi) / 2.0f;
                    SKA_UTP_MultipleOGetUnshapedMinDistAndTime(pUTP, dStartVelocity, dvMid,
                                                    dMaxAcceleration, dMaxJerk, &(dstep1Dist), &(dstep1Time));
                    dtmp = 2.0f * dstep1Dist + dStartVelocity * dtN;
                    dtMid = (dTargetPos - dtmp) / dvMid;
                    dtmp = 2.0f * dstep1Time + dtMid + dtN;
                    if (dtmp > dMinTime) {
                        dlo = dvMid;
                    } else {
                        dhi = dvMid;
                    }
                }
                dvMid = (dlo + dhi) / 2.0f;
                SKA_UTP_MultipleOGetUnshapedMinDistAndTime(pUTP, dStartVelocity, dvMid,
                                                dMaxAcceleration, dMaxJerk, &(dstep1Dist), &(dstep1Time));
                dtmp = 2.0f * dstep1Dist + dStartVelocity * dtN;
                dtMid = (dTargetPos - dtmp) / dvMid;
                *pTotalTime = dMinTime;
            }
        } else {
            // 减速、匀速、加速
            *pTotalTime = dMinTime;
            dstep1TimeMax = (dMinTime - dtN) / 2.0f;
            if (dstep1TimeMax >= 2.0f * dMaxAcceleration / dMaxJerk) {
                dt1 = dMaxAcceleration / dMaxJerk;
                dt2 = dstep1TimeMax - 2.0f * dt1;
            } else {
                dt1 = dstep1TimeMax / 2.0f;
                dt2 = 0.0f;
            }
            dlo = fmax(dStartVelocity - dMaxJerk * dt1 * (dt1 + dt2), 0.0);
            dhi = dStartVelocity;
            while (dhi - dlo > SKA_FlOAT_ERROR) {
                dvMid = (dlo + dhi) / 2.0f;
                SKA_UTP_MultipleOGetUnshapedMinDistAndTime(pUTP, dStartVelocity, dvMid,
                                                dMaxAcceleration, dMaxJerk, &(dstep1Dist), &(dstep1Time));

                dtMid = dMinTime - 2.0f * dstep1Time - dtN;
                dtmp = 2.0f * dstep1Dist + dvMid * dtMid + dStartVelocity * dtN;
                if (dtmp > dTargetPos) {
                    dhi = dvMid;
                } else {
                    dlo = dvMid;
                }
            }
            dvMid = (dlo + dhi) / 2.0f;
            SKA_UTP_MultipleOGetUnshapedMinDistAndTime(pUTP, dStartVelocity, dvMid,
                                            dMaxAcceleration, dMaxJerk, &(dstep1Dist), &(dstep1Time));

            dtMid = dMinTime - 2.0f * dstep1Time - dtN;
        }

        *bIsUniform = 0;
        *pTotalTime = *pTotalTime - dtN;

        // 计算轨迹点
        if (dCurTime < dstep1Time) {
            SKA_UTP_MultipleOGetNormalUnshapedTraceSegment(pUTP, dCurTime, dStartVelocity,
                                                dvMid, dMaxAcceleration, dMaxJerk, pPos, pVelocity, pAcceleration);
        } else if (dCurTime < dstep1Time + dtMid) {
            SKA_UTP_MultipleOGetNormalUnshapedTraceSegment(pUTP, dstep1Time, dStartVelocity,
                                                dvMid, dMaxAcceleration, dMaxJerk, &(dx0), pVelocity, pAcceleration);
            *pPos = dx0 + dvMid * (dCurTime - dstep1Time);
            *pVelocity = dvMid;
            *pAcceleration = 0.0f;
        } else {
           SKA_UTP_MultipleOGetNormalUnshapedTraceSegment(pUTP, dstep1Time, dStartVelocity,
                                               dvMid, dMaxAcceleration, dMaxJerk, &(dx0), pVelocity, pAcceleration);
            dx0 = dx0 + dvMid * dtMid;
           SKA_UTP_MultipleOGetNormalUnshapedTraceSegment(pUTP, dCurTime - dstep1Time - dtMid, dvMid, dStartVelocity,
                                               dMaxAcceleration, dMaxJerk, pPos, pVelocity, pAcceleration);
            *pPos = *pPos + dx0;
        }
    }
                
    return 0;
} 

int8_t SKA_UTP_MultipleOGetUnshapedMinDistAndTime(SKA_UnshapedTracePlanner *pUTP, double dStartVelocity, double dTargetVelocity, double dMaxAcceleration,
                                                    double dMaxJerk, double *pMinDistance, double *pMinTime)
{
    double ddeltaV, dt1, dt2;

    ddeltaV = fabs(dStartVelocity - dTargetVelocity);

    if (ddeltaV < SKA_FlOAT_ERROR)
    {
        // 无需运动
        *pMinDistance = 0.0;
        *pMinTime = 0.0;
    }
    else
    {
        if (ddeltaV < dMaxAcceleration * dMaxAcceleration / dMaxJerk)
        {
            dt1 = sqrt(ddeltaV / dMaxJerk);
            dt2 = 0.0;
        }
        else
        {
            dt1 = dMaxAcceleration / dMaxJerk;
            dt2 = fabs(ddeltaV - dMaxAcceleration * dMaxAcceleration / dMaxJerk) / dMaxAcceleration;
        }

        if (dTargetVelocity >= dStartVelocity)
        {
            *pMinDistance = dStartVelocity * (2.0 * dt1 + dt2) + 0.5 * dMaxJerk * dt1 * (2.0 * dt1 + dt2) * (dt1 + dt2);
        }
        else
        {
            *pMinDistance = dStartVelocity * (2.0 * dt1 + dt2) - 0.5 * dMaxJerk * dt1 * (2.0 * dt1 + dt2) * (dt1 + dt2);
        }
        *pMinTime = 2 * dt1 + dt2;
    }
    return 0;
}

int8_t SKA_UTP_MultipleOGetNormalUnshapedTraceSegment(SKA_UnshapedTracePlanner *pUTP, double dCurTime, double dStartVelocity, double dTargetVelocity, double dMaxAcceleration,
                                                        double dMaxJerk, double *pPos, double *pVelocity, double *pAcceleration)
{

    double ddeltaV, dt1, dt2, djm;
    ddeltaV = fabs(dStartVelocity - dTargetVelocity);

    if (ddeltaV < (dMaxAcceleration * dMaxAcceleration) / dMaxJerk)
    {
        dt1 = sqrt(ddeltaV / dMaxJerk);
        dt2 = 0.0;
    }
    else
    {
        dt1 = dMaxAcceleration / dMaxJerk;
        dt2 = fabs(ddeltaV - dMaxAcceleration * dMaxAcceleration / dMaxJerk) / dMaxAcceleration;
    }

    if (dTargetVelocity >= dStartVelocity)
    {
        djm = dMaxJerk;
    }
    else
    {
        djm = -dMaxJerk;
    }

    if (dCurTime <= 0.0)
    {
        *pPos = dStartVelocity * dCurTime;
        *pVelocity = dStartVelocity;
        *pAcceleration = 0.0;
    }
    else if (dCurTime <= dt1)
    {
        *pPos = (1.0 / 6.0) * djm * pow(dCurTime, 3) + dStartVelocity * dCurTime;
        *pVelocity = 0.5 * djm * pow(dCurTime, 2) + dStartVelocity;
        *pAcceleration = djm * dCurTime;
    }
    else if (dCurTime <= dt1 + dt2)
    {
        *pPos = 0.5 * djm * dt1 * pow(dCurTime - dt1, 2) + (0.5 * djm * pow(dt1, 2) + dStartVelocity) * (dCurTime - dt1) +
                (1.0 / 6.0) * djm * pow(dt1, 3) + dStartVelocity * dt1;
        *pVelocity = djm * dt1 * (dCurTime - dt1) + 0.5 * djm * pow(dt1, 2) + dStartVelocity;
        *pAcceleration = djm * dt1;
    }
    else if (dCurTime <= 2 * dt1 + dt2)
    {
        *pPos = -(1.0 / 6.0) * djm * pow(dCurTime - dt1 - dt2, 3) + 0.5 * djm * dt1 * pow(dCurTime - dt1 - dt2, 2) +
                (djm * dt1 * dt2 + 0.5 * djm * pow(dt1, 2) + dStartVelocity) * (dCurTime - dt1 - dt2) +
                0.5 * djm * dt1 * dt2 * (dt1 + dt2) + (1.0 / 6.0) * djm * pow(dt1, 3) +
                dStartVelocity * (dt1 + dt2);
        *pVelocity = -0.5 * djm * pow((dCurTime - dt1 - dt2), 2) + djm * dt1 * (dCurTime - dt1 - dt2) +
                     djm * dt1 * dt2 + 0.5 * djm * pow(dt1, 2) + dStartVelocity;
        *pAcceleration = -djm * (dCurTime - dt1 - dt2) + djm * dt1;
    }
    else
    {
        *pPos = (djm * dt1 * (dt1 + dt2) + dStartVelocity) * (dCurTime - 2 * dt1 - dt2) +
                0.5 * djm * dt1 * (dt1 + dt2) * (2 * dt1 + dt2) + dStartVelocity * (2 * dt1 + dt2);
        *pVelocity = djm * dt1 * (dt1 + dt2) + dStartVelocity;
        *pAcceleration = 0.0;
    }

    return 0;
}

int8_t SKA_UTP_GetSingleOUnshapedTrace(SKA_UnshapedTracePlanner *pUTP, double dCurTime, double dStartPos, double dTargetPos, double dMaxVelocity, double dMaxAcceleration,
                                         double dMaxJerk, double *pPos, double *pVelocity, double *pAcceleration, double *pTotalTime)
{

    double dt1Max, dt2Max, dS0, dS1;
    double dDist, dt1, dt2, dt3;

    dt1Max = dMaxAcceleration / dMaxJerk;
    dt2Max = fabs(dMaxVelocity / dMaxAcceleration - dt1Max);
    dS0 = dMaxAcceleration * (2.0 * dt1Max + dt2Max) * (dt1Max + dt2Max);
    dS1 = 2.0 * dMaxAcceleration * pow(dt1Max, 2);
    dDist = fabs(dTargetPos - dStartPos);

    if (dDist < dS1)
    {
        dt1 = pow((dDist / (2.0 * dMaxJerk)), 1.0 / 3.0);
        dt2 = 0.0;
        dt3 = 0.0;
    }
    else if (dDist < dS0)
    {
        dt1 = dt1Max;
        dt2 = fabs(-3.0 * dt1 + sqrtf(pow(dt1, 2) + 4.0 * dDist / dMaxAcceleration)) / 2.0;
        dt3 = 0.0;
    }
    else
    {
        dt1 = dt1Max;
        dt2 = dt2Max;
        dt3 = fabs(dDist / (dMaxAcceleration * (dt1 + dt2)) - 2.0 * dt1 - dt2);
    }

    if (dCurTime < 0.0)
    {
        *pPos = 0.0;
        *pVelocity = 0.0;
        *pAcceleration = 0.0;
    }
    else if (dCurTime < dt1)
    {
        *pPos = (1.0 / 6.0) * dMaxJerk * pow(dCurTime, 3);
        *pVelocity = 0.5 * dMaxJerk * pow(dCurTime, 2);
        *pAcceleration = dMaxJerk * dCurTime;
    }
    else if (dCurTime < dt1 + dt2)
    {

        double dt = dCurTime - dt1;
        *pPos = 0.5 * dMaxJerk * dt1 * pow(dt, 2) + 0.5 * dMaxJerk * pow(dt1, 2) * dt + (1.0 / 6.0) * dMaxJerk * pow(dt1, 3);
        *pVelocity = dMaxJerk * dt1 * dt + 0.5 * dMaxJerk * pow(dt1, 2);
        *pAcceleration = dMaxJerk * dt1;
    }
    else if (dCurTime < 2.0 * dt1 + dt2)
    {

        double dt = dCurTime - dt1 - dt2;
        *pPos = -(1.0 / 6.0) * dMaxJerk * pow(dt, 3) + 0.5 * dMaxJerk * dt1 * pow(dt, 2) + (dMaxJerk * dt1 * dt2 + 0.5 * dMaxJerk * pow(dt1, 2)) * dt +
                0.5 * dMaxJerk * dt1 * pow(dt2, 2) + 0.5 * dMaxJerk * pow(dt1, 2) * dt2 + (1.0 / 6.0) * dMaxJerk * pow(dt1, 3);
        *pVelocity = -0.5 * dMaxJerk * pow(dt, 2) + dMaxJerk * dt1 * dt + dMaxJerk * dt1 * dt2 + 0.5 * dMaxJerk * pow(dt1, 2);
        *pAcceleration = -dMaxJerk * dt + dMaxJerk * dt1;
    }
    else if (dCurTime < 2.0 * dt1 + dt2 + dt3)
    {

        double dt = dCurTime - 2.0 * dt1 - dt2;
        *pPos = (dMaxJerk * pow(dt1, 2) + dMaxJerk * dt1 * dt2) * dt + dMaxJerk * pow(dt1, 3) + 1.5 * dMaxJerk * pow(dt1, 2) * dt2 + 0.5 * dMaxJerk * dt1 * pow(dt2, 2);
        *pVelocity = dMaxJerk * pow(dt1, 2) + dMaxJerk * dt1 * dt2;
        *pAcceleration = 0.0;
    }
    else if (dCurTime < 3.0 * dt1 + dt2 + dt3)
    {

        double dt = dCurTime - 2.0 * dt1 - dt2 - dt3;
        *pPos = -(1.0 / 6.0) * dMaxJerk * pow(dt, 3) + (dMaxJerk * pow(dt1, 2) + dMaxJerk * dt1 * dt2) * dt + (dMaxJerk * pow(dt1, 2) + dMaxJerk * dt1 * dt2) * dt3 + dMaxJerk * pow(dt1, 3) + 1.5 * dMaxJerk * pow(dt1, 2) * dt2 + 0.5 * dMaxJerk * dt1 * dt2 * dt2;
        *pVelocity = -0.5 * dMaxJerk * pow(dt, 2) + dMaxJerk * pow(dt1, 2) + dMaxJerk * dt1 * dt2;
        *pAcceleration = -dMaxJerk * dt;
    }
    else if (dCurTime < 3.0 * dt1 + 2.0 * dt2 + dt3)
    {

        double dt = dCurTime - 3.0 * dt1 - dt2 - dt3;
        *pPos = -0.5 * dMaxJerk * dt1 * pow(dt, 2) + (0.5 * dMaxJerk * pow(dt1, 2) + dMaxJerk * dt1 * dt2) * dt + (dMaxJerk * pow(dt1, 2) + dMaxJerk * dt1 * dt2) * dt3 + (11.0 / 6.0) * dMaxJerk * pow(dt1, 3) + 2.5 * dMaxJerk * pow(dt1, 2) * dt2 + 0.5 * dMaxJerk * dt1 * dt2 * dt2;
        *pVelocity = -dMaxJerk * dt1 * dt + 0.5 * dMaxJerk * pow(dt1, 2) + dMaxJerk * dt1 * dt2;
        *pAcceleration = -dMaxJerk * dt1;
    }
    else if (dCurTime < 4.0 * dt1 + 2.0 * dt2 + dt3)
    {

        double dt = dCurTime - 3.0 * dt1 - 2.0 * dt2 - dt3;
        *pPos = (1.0 / 6.0) * dMaxJerk * pow(dt, 3) - 0.5 * dMaxJerk * dt1 * dt * dt + 0.5 * dMaxJerk * pow(dt1, 2) * dt + (11.0 / 6.0) * dMaxJerk * pow(dt1, 3) + 3.0 * dMaxJerk * pow(dt1, 2) * dt2 + dMaxJerk * dt1 * dt2 * dt2 + (dMaxJerk * pow(dt1, 2) + dMaxJerk * dt1 * dt2) * dt3;
        *pVelocity = 0.5 * dMaxJerk * pow(dt, 2) - dMaxJerk * dt1 * dt + 0.5 * dMaxJerk * pow(dt1, 2);
        *pAcceleration = dMaxJerk * dt - dMaxJerk * dt1;
    }
    else
    {
        *pPos = dDist;
        *pVelocity = 0.0;
        *pAcceleration = 0.0;
    }

    if (dTargetPos >= dStartPos)
    {
        *pPos = *pPos + dStartPos;
    }
    else
    {
        *pPos = dStartPos - *pPos;
        *pVelocity = -*pVelocity;
        *pAcceleration = -*pAcceleration;
    }

    *pTotalTime = 4.0 * dt1 + 2.0 * dt2 + dt3;

    return 0;
}

int8_t SKA_UTP_Destroy(SKA_UnshapedTracePlanner *pUTP)
{
    /******************** 函数参数合法性检验 ********************/
    if (pUTP == NULL)
    {
        printf("Invalid parameters of SKA_UTP_Destroy().\n");
        return -1;
    }

    /******************** 销毁 ********************/

    // 释放轨迹链表
    if (pUTP->pTraceHead != NULL)
    {
        pUTP->pTraceHead->pPrev->pNext = NULL;
        SKA_TraceSegment *pCur;
        SKA_TraceSegment *pNext = pUTP->pTraceHead;
        while (pNext != NULL)
        {
            pCur = pNext;
            pNext = pCur->pNext;
            free(pCur);
        }

        pUTP->pTraceHead = NULL;
    }

    return 0;
}

int8_t SKA_UTP_Display(const SKA_UnshapedTracePlanner *pUTP)
{
    /******************** 函数参数合法性检验 ********************/
    if (pUTP == NULL || pUTP->pTraceHead == NULL)
    {
        printf("Invalid parameters of SKA_UTP_Display().\n");
        return -1;
    }

    /******************** 显示 ********************/
    SKA_Trace_Display(pUTP->pTraceHead);

    return 0;
}