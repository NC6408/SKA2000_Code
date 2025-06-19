/**
  ******************************************************************************
  * @file       : SKA_QuickStopTracePlanner.c
  * @brief      : 无防摇急停轨迹规划器
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

#include "SKA_QuickStopTracePlanner.h"
#include "SKA_Constants.h"

int8_t SKA_QSTP_Create(SKA_QuickStopTracePlanner *pQSTP, double dStartTime, double dStartPos, double dStartVel, double dMaxAcc) {
    /******************** 函数参数合法性检验 ********************/
    if (pQSTP == NULL || dMaxAcc <= 0) {
        printf("Invalid parameters of SKA_QSTP_Create().\n");
        return -1;
    }

    /******************** 创建 ********************/

    // 梯形速度曲线

    double dT1 = fabs(dStartVel) / dMaxAcc;

    int8_t nDirection;
    if (dStartVel >= 0) {
        nDirection = 1;
    } else {
        nDirection = -1;
    }

    // 创建轨迹链表
    
    SKA_TraceSegment *pHead = (SKA_TraceSegment *)malloc(sizeof(SKA_TraceSegment));
    pHead->pNext = pHead;
    pHead->pPrev = pHead;
    SKA_TraceSegment *pPrev = pHead;
    double dT0 = dStartTime, dX0 = dStartPos, dV0 = dStartVel, dA0 = -nDirection * dMaxAcc;

    if (dT1 > SKA_FlOAT_ERROR) {
        SKA_TraceSegment *pSeg = (SKA_TraceSegment *)malloc(sizeof(SKA_TraceSegment));
        SKA_Trace_SetTraceSegment(pSeg, dT0, dX0, dV0, dA0, 0.0, pPrev, NULL);
        pPrev->pNext = pSeg;
        pPrev = pSeg;
        
        dT0 += dT1;
        SKA_Trace_GetTracePoint(dT0, pSeg, &dX0, &dV0, &dA0);
    }

    SKA_TraceSegment *pSeg = (SKA_TraceSegment *)malloc(sizeof(SKA_TraceSegment));
    SKA_Trace_SetTraceSegment(pSeg, dT0, dX0, 0.0, 0.0, 0.0, pPrev, NULL);
    pPrev->pNext = pSeg;
    pPrev = pSeg;
    
    pPrev->pNext = pHead;
    pHead->pPrev = pPrev;

    // 初始化急停轨迹头结点
    pQSTP->pTraceHead = pHead;

    return 0;
}

int8_t SKA_QSTP_Run(SKA_QuickStopTracePlanner *pQSTP, double dCurTime, double *pRefPos, double *pRefVel, double *pRefAcc) {
    /******************** 函数参数合法性检验 ********************/
    if (pQSTP == NULL || pRefPos == NULL || pRefVel == NULL || pRefAcc == NULL) {
        printf("Invalid parameters of SKA_QSTP_Run().\n");
        return -1;
    }
    if (pQSTP->pTraceHead == NULL || pQSTP->pTraceHead->pNext == pQSTP->pTraceHead) {
        printf("Invalid parameters of SKA_QSTP_Run().\n");
        return -1;
    }

    /******************** 运行 ********************/

    // 寻找轨迹段
    SKA_TraceSegment *pCurSeg = pQSTP->pTraceHead->pNext;
    while (pCurSeg->pNext != pQSTP->pTraceHead && dCurTime >= pCurSeg->pNext->dT0) {
        pCurSeg = pCurSeg->pNext;
    }
    
    // 计算轨迹点
    SKA_Trace_GetTracePoint(dCurTime, pCurSeg, pRefPos, pRefVel, pRefAcc);

    return 0;
}

int8_t SKA_QSTP_GetTotalTime(const SKA_QuickStopTracePlanner *pQSTP, double *pTotalTime) {
    /******************** 函数参数合法性检验 ********************/
    if (pQSTP == NULL || pTotalTime == NULL) {
        printf("Invalid parameters of SKA_QSTP_GetTotalTime().\n");
        return -1;
    }

    /******************** 获取总时间 ********************/
    SKA_Trace_GetTotalTime(pQSTP->pTraceHead, pTotalTime);

    return 0;
}

int8_t SKA_QSTP_GetTargetPos(const SKA_QuickStopTracePlanner *pQSTP, double *pTargetPos) {
    /******************** 函数参数合法性检验 ********************/
    if (pQSTP == NULL || pTargetPos == NULL || pQSTP->pTraceHead == NULL || pQSTP->pTraceHead->pPrev == pQSTP->pTraceHead) {
        printf("Invalid parameters of SKA_QSTP_GetTargetPos().\n");
        return -1;
    }

    /******************** 获取目标位置 ********************/
    *pTargetPos = pQSTP->pTraceHead->pPrev->dX0;

    return 0;
}

int8_t SKA_QSTP_Destroy(SKA_QuickStopTracePlanner *pQSTP) {
    /******************** 函数参数合法性检验 ********************/
    if (pQSTP == NULL) {
        printf("Invalid parameters of SKA_QSTP_Destroy().\n");
        return -1;
    }

    /******************** 销毁 ********************/
    if (pQSTP->pTraceHead != NULL) {
        pQSTP->pTraceHead->pPrev->pNext = NULL;
        SKA_TraceSegment *pCur;
        SKA_TraceSegment *pNext = pQSTP->pTraceHead;
        while (pNext != NULL) {
            pCur = pNext;
            pNext = pCur->pNext;
            free(pCur);
        }

        pQSTP->pTraceHead = NULL;
    }

    return 0;
}