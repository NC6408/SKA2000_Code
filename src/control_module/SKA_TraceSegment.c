/**
  ******************************************************************************
  * @file       : SKA_TraceSegment.c
  * @brief      : 轨迹段
  * @author     : ZhangYi
  * @version    : None
  * @date       : 2024/11/05
  ******************************************************************************
  */
//

#include <stddef.h> 
#include <math.h>
#include <stdlib.h>
#include <stdio.h>

#include "SKA_TraceSegment.h"

int8_t SKA_Trace_SetTraceSegment(SKA_TraceSegment* pSeg, double dT0, double dX0, double dV0, double dA0, double dJerk,
                               SKA_TraceSegment* pPrev, SKA_TraceSegment* pNext) {
    /******************** 函数参数合法性检验 ********************/
    if (pSeg == NULL) {
        printf("Invalid parameters of SKA_Trace_SetTraceSegment().\n");
        return -1;
    }

    /******************** 设置轨迹段 ********************/
    pSeg->dT0 = dT0;
    pSeg->dX0 = dX0;
    pSeg->dV0 = dV0;
    pSeg->dA0 = dA0;
    pSeg->dJerk = dJerk;
    pSeg->pPrev = pPrev;
    pSeg->pNext = pNext;

    return 0;
}

int8_t SKA_Trace_GetTraceSegmentNum(const SKA_TraceSegment *pHead, uint16_t *pNum) {
    /******************** 函数参数合法性检验 ********************/
    if (pHead == NULL) {
        printf("Invalid parameters of SKA_Trace_GetTraceSegmentNum().\n");
        return -1;
    }

    /******************** 计算轨迹段数量 ********************/
    uint16_t num = 0;
    const SKA_TraceSegment *p = pHead;
    while (p->pNext != pHead) {
        p = p->pNext;
        num++;
    }
    *pNum = num;

    return 0;
}

int8_t SKA_Trace_GetTracePoint(double dCurTime, const SKA_TraceSegment *pSeg, double *pX, double *pV, double *pA) {
    /******************** 函数参数合法性检验 ********************/
    if (pSeg == NULL || pX == NULL || pV == NULL || pA == NULL) {
        printf("Invalid parameters of SKA_Trace_GetTracePoint().\n");
        return -1;
    }

    /******************** 计算轨迹点位置 ********************/
    // 恒加加速度运动
    double dT = dCurTime - pSeg->dT0;
    *pA = pSeg->dA0 + pSeg->dJerk * dT;
    *pV = pSeg->dV0 + pSeg->dA0 * dT + 0.5 * pSeg->dJerk * pow(dT, 2.0);
    *pX = pSeg->dX0 + pSeg->dV0 * dT + 0.5 * pSeg->dA0 * pow(dT, 2.0) + (1.0 / 6.0) * pSeg->dJerk * pow(dT, 3.0);

    return 0;
}

int8_t SKA_Trace_GetTotalTime(const SKA_TraceSegment *pHead, double *pTotalTime) {
    /******************** 函数参数合法性检验 ********************/
    if (pHead == NULL || pTotalTime == NULL || pHead->pPrev == pHead) {
        printf("Invalid parameters of SKA_Trace_GetTotalTime().\n");
        return -1;
    }

    /******************** 计算轨迹段总时间 ********************/
    *pTotalTime = pHead->pPrev->dT0;

    return 0;
}

int8_t SKA_Trace_GetTargetPos(const SKA_TraceSegment *pHead, double *pTargetPos) {
    /******************** 函数参数合法性检验 ********************/
    if (pHead == NULL || pTargetPos == NULL || pHead->pPrev == pHead) {
        printf("Invalid parameters of SKA_Trace_GetTargetPos().\n");
        return -1;
    }

    /******************** 计算轨迹的目标位置 ********************/
    *pTargetPos = pHead->pPrev->dX0;

    return 0;
}

int8_t SKA_Trace_Display(const SKA_TraceSegment* pHead) {
    /******************** 函数参数合法性检验 ********************/
    if (pHead == NULL) {
        printf("Invalid parameters of SKA_Trace_Display().\n");
        return -1;
    }
    
    /******************** 打印轨迹 ********************/
    const SKA_TraceSegment *p = pHead;
    while (p->pNext != pHead) {
        p = p->pNext;
        printf("[t0: %.2fs, x0: %.3fm, v0: %.3fm/s, a0: %.3fm/s^2, jerk: %.3fm/s^3]\n",
               p->dT0, p->dX0, p->dV0, p->dA0, p->dJerk);
    }

    return 0;
}

int8_t SKA_Trace_FreeSinglyLinkedListTrace(SKA_TraceSegment* p) {
    /******************** 函数参数合法性检验 ********************/
    if (p == NULL) {
        return 0;
    }
    if (p->pNext == p) {
        printf("Invalid parameters of SKA_Trace_FreeSinglyLinkedListTrace().\n");
        return -1;
    }

    /******************** 释放单向链表轨迹 ********************/
    SKA_TraceSegment* pCur;
    SKA_TraceSegment* pNext = p;
    while (pNext != NULL) {
        pCur = pNext;
        pNext = pCur->pNext;
        free(pCur);
    }
    return 0;
}
