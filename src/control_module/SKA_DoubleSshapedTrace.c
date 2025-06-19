/**
  ******************************************************************************
  * @file       : SKA_DoubleSshapedTrace.c
  * @brief      : 双S形速度曲线
  * @author     : ZhangYi
  * @version    : None
  * @date       : 2024/10/24
  ******************************************************************************
  */
//

#include <stddef.h> 
#include <math.h>
#include <stdlib.h>
#include <stdio.h>

#include "SKA_Constants.h"
#include "SKA_DoubleSshapedTrace.h"

int8_t SKA_DST_GetZero2ZeroTrace(double dDist, double dMaxVelocity, double dMaxAcceleration, double dMaxJerk,
                               SKA_TraceSegment** ppHead) {
    /******************** 函数参数合法性检验 ********************/
    if (dDist < 0 || dMaxVelocity <= 0 || dMaxAcceleration <= 0 || dMaxJerk <= 0) {
        printf("Invalid parameters of SKA_DST_GetZero2ZeroTrace().\n");
        return -1;
    }

    /******************** 计算初始轨迹 ********************/

    /** 计算双S形速度曲线的参数 **/
                                   
    // 阶段1的最大时长
    double dT1Max = dMaxAcceleration / dMaxJerk;
    // 阶段2的最大时长
    double dT2Max = (dMaxVelocity / dMaxAcceleration) - dT1Max;
    // 阶段3时长恰好为0的运送距离
    double dS0 = dMaxAcceleration * (2.0 * dT1Max + dT2Max) * (dT1Max + dT2Max);
    // 阶段2时长恰好为0的运送距离
    double dS1 = 2.0 * dMaxAcceleration * pow(dT1Max, 2);
    double dT1, dT2, dT3;
    if (dDist < dS1) {
        dT1 = pow(dDist / (2.0 * dMaxJerk), 1.0 / 3.0);
        dT2 = 0.0;
        dT3 = 0.0;

    } else if (dDist < dS0) {
        dT1 = dT1Max;
        dT2 = (-3.0 * dT1 + sqrt(pow(dT1, 2) + 4.0 * dDist / dMaxAcceleration)) / 2.0;
        dT3 = 0.0;

    } else {
        dT1 = dT1Max;
        dT2 = dT2Max;
        dT3 = dDist / (dMaxAcceleration * (dT1 + dT2)) - 2.0 * dT1 - dT2;

    }
    
    /** 创建轨迹链表 **/
    
    SKA_TraceSegment *pHead = (SKA_TraceSegment *)malloc(sizeof(SKA_TraceSegment));
    pHead->pNext = pHead;
    pHead->pPrev = pHead;
    SKA_TraceSegment *pPrev = pHead;
    double dT0 = 0.0, dX0 = 0.0, dV0 = 0.0, dA0 = 0.0;
    
    if (dT1 > SKA_FlOAT_ERROR) {
        SKA_TraceSegment *pSeg = (SKA_TraceSegment *)malloc(sizeof(SKA_TraceSegment));
        SKA_Trace_SetTraceSegment(pSeg, dT0, dX0, dV0, dA0, dMaxJerk, pPrev, NULL);
        pPrev->pNext = pSeg;
        pPrev = pSeg;

        dT0 += dT1;
        SKA_Trace_GetTracePoint(dT0, pSeg, &dX0, &dV0, &dA0);
    }
    
    if (dT2 > SKA_FlOAT_ERROR) {
        SKA_TraceSegment *pSeg = (SKA_TraceSegment *)malloc(sizeof(SKA_TraceSegment));
        SKA_Trace_SetTraceSegment(pSeg, dT0, dX0, dV0, dA0, 0.0, pPrev, NULL);
        pPrev->pNext = pSeg;
        pPrev = pSeg;

        dT0 += dT2;
        SKA_Trace_GetTracePoint(dT0, pSeg, &dX0, &dV0, &dA0);
    }
    
    if (dT1 > SKA_FlOAT_ERROR) {
        SKA_TraceSegment *pSeg = (SKA_TraceSegment *)malloc(sizeof(SKA_TraceSegment));
        SKA_Trace_SetTraceSegment(pSeg, dT0, dX0, dV0, dA0, -dMaxJerk, pPrev, NULL);
        pPrev->pNext = pSeg;
        pPrev = pSeg;

        dT0 += dT1;
        SKA_Trace_GetTracePoint(dT0, pSeg, &dX0, &dV0, &dA0);
    }
    
    if (dT3 > SKA_FlOAT_ERROR) {
        SKA_TraceSegment *pSeg = (SKA_TraceSegment *)malloc(sizeof(SKA_TraceSegment));
        SKA_Trace_SetTraceSegment(pSeg, dT0, dX0, dV0, dA0, 0.0, pPrev, NULL);
        pPrev->pNext = pSeg;
        pPrev = pSeg;

        dT0 += dT3;
        SKA_Trace_GetTracePoint(dT0, pSeg, &dX0, &dV0, &dA0);
    }
    
    if (dT1 > SKA_FlOAT_ERROR) {
        SKA_TraceSegment *pSeg = (SKA_TraceSegment *)malloc(sizeof(SKA_TraceSegment));
        SKA_Trace_SetTraceSegment(pSeg, dT0, dX0, dV0, dA0, -dMaxJerk, pPrev, NULL);
        pPrev->pNext = pSeg;
        pPrev = pSeg;

        dT0 += dT1;
        SKA_Trace_GetTracePoint(dT0, pSeg, &dX0, &dV0, &dA0);
    }
    
    if (dT2 > SKA_FlOAT_ERROR) {
        SKA_TraceSegment *pSeg = (SKA_TraceSegment *)malloc(sizeof(SKA_TraceSegment));
        SKA_Trace_SetTraceSegment(pSeg, dT0, dX0, dV0, dA0, 0.0, pPrev, NULL);
        pPrev->pNext = pSeg;
        pPrev = pSeg;

        dT0 += dT2;
        SKA_Trace_GetTracePoint(dT0, pSeg, &dX0, &dV0, &dA0);
    }
    
    if (dT1 > SKA_FlOAT_ERROR) {
        SKA_TraceSegment *pSeg = (SKA_TraceSegment *)malloc(sizeof(SKA_TraceSegment));
        SKA_Trace_SetTraceSegment(pSeg, dT0, dX0, dV0, dA0, dMaxJerk, pPrev, NULL);
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

    *ppHead = pHead;

    return 0;
}

int8_t SKA_DST_GetUrgencyStopTrace(double dStartV, double dStartA, double dMaxAcceleration, double dMaxJerk, 
                                   SKA_TraceSegment** ppHead) {
    /******************** 函数参数合法性检验 ********************/
    if (dStartV < 0.0 || dMaxAcceleration <= 0.0 || dMaxJerk <= 0.0) {
        printf("Invalid parameters of SKA_DST_GetUrgencyStopTrace().\n");
        return -1;
    }

    /******************** 计算急停轨迹 ********************/
    SKA_TraceSegment *pHead = (SKA_TraceSegment *)malloc(sizeof(SKA_TraceSegment));
    pHead->pNext = pHead;
    pHead->pPrev = pHead;
    SKA_TraceSegment *pPrev = pHead;
    
    double dT0 = 0.0, dT1, dT2;
    double dX0 = 0.0;
    double dV0 = dStartV;
    double dA0 = dStartA;
    
    if (fabs(dStartA) < SKA_FlOAT_ERROR) {
        // 初始加速度为0
        
        if (dStartV > pow(dMaxAcceleration, 2.0) / dMaxJerk) {
            dT1 = dMaxAcceleration / dMaxJerk;
            dT2 = fabs(dStartV - pow(dMaxAcceleration, 2.0) / dMaxJerk) / dMaxAcceleration;
        } else {
            dT1 = sqrt(dStartV / dMaxJerk);
            dT2 = 0.0;
        }
        
        if (dT1 > SKA_FlOAT_ERROR) {
            SKA_TraceSegment *pSeg = (SKA_TraceSegment *)malloc(sizeof(SKA_TraceSegment));
            SKA_Trace_SetTraceSegment(pSeg, dT0, dX0, dV0, dA0, -dMaxJerk, pPrev, NULL);
            pPrev->pNext = pSeg;
            pPrev = pSeg;
            
            dT0 += dT1;
            SKA_Trace_GetTracePoint(dT0, pSeg, &dX0, &dV0, &dA0);
        }
        
        if (dT2 > SKA_FlOAT_ERROR) {
            SKA_TraceSegment *pSeg = (SKA_TraceSegment *)malloc(sizeof(SKA_TraceSegment));
            SKA_Trace_SetTraceSegment(pSeg, dT0, dX0, dV0, dA0, 0.0, pPrev, NULL);
            pPrev->pNext = pSeg;
            pPrev = pSeg;
            
            dT0 += dT2;
            SKA_Trace_GetTracePoint(dT0, pSeg, &dX0, &dV0, &dA0);
        }
        
        if (dT1 > SKA_FlOAT_ERROR) {
            SKA_TraceSegment *pSeg = (SKA_TraceSegment *)malloc(sizeof(SKA_TraceSegment));
            SKA_Trace_SetTraceSegment(pSeg, dT0, dX0, dV0, dA0, dMaxJerk, pPrev, NULL);
            pPrev->pNext = pSeg;
            pPrev = pSeg;
            
            dT0 += dT1;
            SKA_Trace_GetTracePoint(dT0, pSeg, &dX0, &dV0, &dA0);
        }
        
        SKA_TraceSegment *pSeg = (SKA_TraceSegment *)malloc(sizeof(SKA_TraceSegment));
        SKA_Trace_SetTraceSegment(pSeg, dT0, dX0, 0.0, 0.0, 0.0, pPrev, NULL);
        pPrev->pNext = pSeg;
        pPrev = pSeg;
        
    } else if (fabs(dStartA + dMaxAcceleration) < SKA_FlOAT_ERROR) {
        // 初始加速度为-am
        
        dT2 = dMaxAcceleration / dMaxJerk;
        dT1 = fabs(dStartV - 0.5 * dMaxJerk * pow(dT2, 2.0)) / dMaxAcceleration;
        
        if (dT1 > SKA_FlOAT_ERROR) {
            SKA_TraceSegment *pSeg = (SKA_TraceSegment *)malloc(sizeof(SKA_TraceSegment));
            SKA_Trace_SetTraceSegment(pSeg, dT0, dX0, dV0, dA0, 0.0, pPrev, NULL);
            pPrev->pNext = pSeg;
            pPrev = pSeg;
            
            dT0 += dT1;
            SKA_Trace_GetTracePoint(dT0, pSeg, &dX0, &dV0, &dA0);
        }
        
        if (dT2 > SKA_FlOAT_ERROR) {
            SKA_TraceSegment *pSeg = (SKA_TraceSegment *)malloc(sizeof(SKA_TraceSegment));
            SKA_Trace_SetTraceSegment(pSeg, dT0, dX0, dV0, dA0, dMaxJerk, pPrev, NULL);
            pPrev->pNext = pSeg;
            pPrev = pSeg;
            
            dT0 += dT2;
            SKA_Trace_GetTracePoint(dT0, pSeg, &dX0, &dV0, &dA0);
        }
        
        SKA_TraceSegment *pSeg = (SKA_TraceSegment *)malloc(sizeof(SKA_TraceSegment));
        SKA_Trace_SetTraceSegment(pSeg, dT0, dX0, 0.0, 0.0, 0.0, pPrev, NULL);
        pPrev->pNext = pSeg;
        pPrev = pSeg;
        
    } else {
        // 初始加速度为am
        
        dT1 = dMaxAcceleration / dMaxJerk;
        dT2 = fabs(dStartV - pow(dMaxAcceleration, 2.0) / (2.0 * dMaxJerk)) / dMaxAcceleration;
        
        if (dT1 > SKA_FlOAT_ERROR) {
            SKA_TraceSegment *pSeg = (SKA_TraceSegment *)malloc(sizeof(SKA_TraceSegment));
            SKA_Trace_SetTraceSegment(pSeg, dT0, dX0, dV0, dA0, -dMaxJerk, pPrev, NULL);
            pPrev->pNext = pSeg;
            pPrev = pSeg;
            
            dT0 += dT1;
            SKA_Trace_GetTracePoint(dT0, pSeg, &dX0, &dV0, &dA0);
        }
        
        if (dT1 > SKA_FlOAT_ERROR) {
            SKA_TraceSegment *pSeg = (SKA_TraceSegment *)malloc(sizeof(SKA_TraceSegment));
            SKA_Trace_SetTraceSegment(pSeg, dT0, dX0, dV0, dA0, -dMaxJerk, pPrev, NULL);
            pPrev->pNext = pSeg;
            pPrev = pSeg;
            
            dT0 += dT1;
            SKA_Trace_GetTracePoint(dT0, pSeg, &dX0, &dV0, &dA0);
        }
        
        if (dT2 > SKA_FlOAT_ERROR) {
            SKA_TraceSegment *pSeg = (SKA_TraceSegment *)malloc(sizeof(SKA_TraceSegment));
            SKA_Trace_SetTraceSegment(pSeg, dT0, dX0, dV0, dA0, 0.0, pPrev, NULL);
            pPrev->pNext = pSeg;
            pPrev = pSeg;
            
            dT0 += dT2;
            SKA_Trace_GetTracePoint(dT0, pSeg, &dX0, &dV0, &dA0);
        }
        
        if (dT1 > SKA_FlOAT_ERROR) {
            SKA_TraceSegment *pSeg = (SKA_TraceSegment *)malloc(sizeof(SKA_TraceSegment));
            SKA_Trace_SetTraceSegment(pSeg, dT0, dX0, dV0, dA0, dMaxJerk, pPrev, NULL);
            pPrev->pNext = pSeg;
            pPrev = pSeg;
            
            dT0 += dT1;
            SKA_Trace_GetTracePoint(dT0, pSeg, &dX0, &dV0, &dA0);
        }
        
        SKA_TraceSegment *pSeg = (SKA_TraceSegment *)malloc(sizeof(SKA_TraceSegment));
        SKA_Trace_SetTraceSegment(pSeg, dT0, dX0, 0.0, 0.0, 0.0, pPrev, NULL);
        pPrev->pNext = pSeg;
        pPrev = pSeg;
    }
    
    pPrev->pNext = pHead;
    pHead->pPrev = pPrev;
    *ppHead = pHead;

    return 0;
}

int8_t SKA_DST_GetNonzero2ZeroTrace(double dStartV, double dStartA, double dTargetPos, double dMaxVelocity, 
                                    double dMaxAcceleration, double dMaxJerk, double dBufferTime, SKA_TraceSegment** ppHead) {
    /******************** 函数参数合法性检验 ********************/
    if (dStartV <  0.0 || dMaxVelocity <= 0.0 || dStartV > dMaxVelocity + SKA_FlOAT_ERROR || dMaxAcceleration <= 0.0 || dMaxJerk <= 0.0 || dBufferTime < 0) {
        printf("Invalid parameters of SKA_DST_GetNonzero2ZeroTrace().\n");
        return -1;
    }

    /******************** 计算目标位置切换轨迹 ********************/

    SKA_TraceSegment *pHead = (SKA_TraceSegment *)malloc(sizeof(SKA_TraceSegment));
    pHead->pNext = pHead;
    pHead->pPrev = pHead;
    SKA_TraceSegment *pPrev = pHead;
  
    // 计算急停轨迹
    SKA_TraceSegment* pHeadStop;
    SKA_DST_GetUrgencyStopTrace(dStartV, dStartA, dMaxAcceleration, dMaxJerk, &pHeadStop);
    // 急停位置
    double fStopPos = pHeadStop->pPrev->dX0;
    
    if (dTargetPos <= fStopPos) {
        // 急停 + 反向
        
        pPrev->pNext = pHeadStop->pNext;
        pHeadStop->pNext->pPrev = pPrev;
        pPrev = pHeadStop->pPrev;
        free(pHeadStop);
        
        SKA_TraceSegment* pHeadBack;
        SKA_DST_GetZero2ZeroTrace(fStopPos - dTargetPos, dMaxVelocity, dMaxAcceleration, dMaxJerk, &pHeadBack);

        double dT0 = pPrev->dT0 + dBufferTime;
        SKA_TraceSegment* p = pHeadBack->pNext;
        while (p != pHeadBack) {
            p->dT0 = dT0 + p->dT0;
            p->dX0 = fStopPos - p->dX0;
            p->dV0 = - p->dV0;
            p->dA0 = - p->dA0;
            p->dJerk = - p->dJerk;
            
            p = p->pNext;
        }
        pPrev->pNext = pHeadBack->pNext;
        pHeadBack->pNext->pPrev = pPrev;
        pPrev = pHeadBack->pPrev;
        free(pHeadBack);
        
        pPrev->pNext = pHead;
        pHead->pPrev = pPrev;
        *ppHead = pHead;

        return 0;
    } 

    // 无反向运动
    double dT1, dT2, dT3, dT4, dT5, dT6;
    double dT0, dX0, dV0, dA0;
    double lo, hi;

    if (fabs(dStartA) < SKA_FlOAT_ERROR) {
        // 加速、匀速、减速
        
        // 计算加速到最大速度且匀速阶段时长为0的位移阈值
        if (fabs(dMaxVelocity - dStartV) <= pow(dMaxAcceleration, 2.0) / dMaxJerk) {
            dT1 = sqrt(fabs(dMaxVelocity - dStartV) / dMaxJerk);
            dT2 = 0;
        } else {
            dT1 = dMaxAcceleration/ dMaxJerk;
            dT2 = fabs(fabs(dMaxVelocity - dStartV) / dMaxAcceleration- dT1);
        }
        dT3 = 0;
        if (dMaxVelocity <= pow(dMaxAcceleration, 2.0) / dMaxJerk) {
            dT4 = sqrt(dMaxVelocity / dMaxJerk);
            dT5 = 0;
        } else {
            dT4 = dMaxAcceleration/ dMaxJerk ;
            dT5 = fabs(dMaxVelocity / dMaxAcceleration - dT4);
        }
        double s0 = dStartV * dT1 + (dStartV + 0.5 * dMaxJerk * pow(dT1, 2.0)) * dT2 + 0.5 * dMaxJerk * dT1 * pow(dT2, 2.0)
            + (dStartV + dMaxJerk * dT1 * (dT1 + dT2)) * (dT1 + dT3 + dT4 + dT5) - 0.5 * dMaxJerk * pow(dT4, 2.0) * dT5
            - 0.5 * dMaxJerk * dT4 * pow(dT5, 2.0) + (dStartV + dMaxJerk * dT1 * (dT1 + dT2) - dMaxJerk * pow(dT4, 2.0) - dMaxJerk * dT4 * dT5) * dT4;
        
        double deltaX = dTargetPos;
        if (deltaX >= s0) {
            // 加速到MaxVelocity，插入匀速阶段
            dT3 = fabs(deltaX - s0) / dMaxVelocity;
        } else {
            // 加速到maxV<MaxVelocity，匀速阶段时长为0
            
            // 二分法计算maxV
            lo = dStartV;
            hi = dMaxVelocity;
            do {
                double maxV = (lo + hi) / 2;
                if (fabs(maxV - dStartV) <= pow(dMaxAcceleration, 2.0) / dMaxJerk) {
                    dT1 = sqrt(fabs(maxV - dStartV) / dMaxJerk);
                    dT2 = 0;
                } else {
                    dT1 = dMaxAcceleration/ dMaxJerk ;
                    dT2 = fabs(fabs(maxV - dStartV) / dMaxAcceleration- dT1);
                }
                dT3 = 0;
                if (maxV <= pow(dMaxAcceleration, 2.0) / dMaxJerk) {
                    dT4 = sqrt(maxV / dMaxJerk);
                    dT5 = 0;
                } else {
                    dT4 = dMaxAcceleration/ dMaxJerk ;
                    dT5 = fabs(maxV / dMaxAcceleration- dT4);
                }
                s0 = dStartV * dT1 + (dStartV + 0.5 * dMaxJerk * pow(dT1, 2.0)) * dT2 + 0.5 * dMaxJerk * dT1 * pow(dT2, 2.0)
                    + (dStartV + dMaxJerk * dT1 * (dT1 + dT2)) * (dT1 + dT3 + dT4 + dT5) - 0.5 * dMaxJerk * pow(dT4, 2.0) * dT5
                    - 0.5 * dMaxJerk * dT4 * pow(dT5, 2.0) + (dStartV + dMaxJerk * dT1 * (dT1 + dT2) - dMaxJerk * pow(dT4, 2.0) - dMaxJerk * dT4 * dT5) * dT4;
                
                if (deltaX >= s0) {
                    lo = maxV;
                } else {
                    hi = maxV;
                }
            } while (hi - lo >= SKA_FlOAT_ERROR);
        }
        
        dT0 = 0.0;
        dX0 = 0.0;
        dV0 = dStartV;
        dA0 = 0.0;
        
        if (dT1 > SKA_FlOAT_ERROR) {
            SKA_TraceSegment *pSeg = (SKA_TraceSegment *)malloc(sizeof(SKA_TraceSegment));
            SKA_Trace_SetTraceSegment(pSeg, dT0, dX0, dV0, dA0, dMaxJerk, pPrev, NULL);
            pPrev->pNext = pSeg;
            pPrev = pSeg;
            
            dT0 += dT1;
            SKA_Trace_GetTracePoint(dT0, pSeg, &dX0, &dV0, &dA0);
        }
        
        if (dT2 > SKA_FlOAT_ERROR) {
            SKA_TraceSegment *pSeg = (SKA_TraceSegment *)malloc(sizeof(SKA_TraceSegment));
            SKA_Trace_SetTraceSegment(pSeg, dT0, dX0, dV0, dA0, 0.0, pPrev, NULL);
            pPrev->pNext = pSeg;
            pPrev = pSeg;
            
            dT0 += dT2;
            SKA_Trace_GetTracePoint(dT0, pSeg, &dX0, &dV0, &dA0);
        }
        
        if (dT1 > SKA_FlOAT_ERROR) {
            SKA_TraceSegment *pSeg = (SKA_TraceSegment *)malloc(sizeof(SKA_TraceSegment));
            SKA_Trace_SetTraceSegment(pSeg, dT0, dX0, dV0, dA0, -dMaxJerk, pPrev, NULL);
            pPrev->pNext = pSeg;
            pPrev = pSeg;
            
            dT0 += dT1;
            SKA_Trace_GetTracePoint(dT0, pSeg, &dX0, &dV0, &dA0);
        }
        
        if (dT3 > SKA_FlOAT_ERROR) {
            SKA_TraceSegment *pSeg = (SKA_TraceSegment *)malloc(sizeof(SKA_TraceSegment));
            SKA_Trace_SetTraceSegment(pSeg, dT0, dX0, dV0, dA0, 0.0, pPrev, NULL);
            pPrev->pNext = pSeg;
            pPrev = pSeg;
            
            dT0 += dT3;
            SKA_Trace_GetTracePoint(dT0, pSeg, &dX0, &dV0, &dA0);
        }
        
        if (dT4 > SKA_FlOAT_ERROR) {
            SKA_TraceSegment *pSeg = (SKA_TraceSegment *)malloc(sizeof(SKA_TraceSegment));
            SKA_Trace_SetTraceSegment(pSeg, dT0, dX0, dV0, dA0, -dMaxJerk, pPrev, NULL);
            pPrev->pNext = pSeg;
            pPrev = pSeg;
            
            dT0 += dT4;
            SKA_Trace_GetTracePoint(dT0, pSeg, &dX0, &dV0, &dA0);
        }
        
        if (dT5 > SKA_FlOAT_ERROR) {
            SKA_TraceSegment *pSeg = (SKA_TraceSegment *)malloc(sizeof(SKA_TraceSegment));
            SKA_Trace_SetTraceSegment(pSeg, dT0, dX0, dV0, dA0, 0.0, pPrev, NULL);
            pPrev->pNext = pSeg;
            pPrev = pSeg;
            
            dT0 += dT5;
            SKA_Trace_GetTracePoint(dT0, pSeg, &dX0, &dV0, &dA0);
        }
        
        if (dT4 > SKA_FlOAT_ERROR) {
            SKA_TraceSegment *pSeg = (SKA_TraceSegment *)malloc(sizeof(SKA_TraceSegment));
            SKA_Trace_SetTraceSegment(pSeg, dT0, dX0, dV0, dA0, dMaxJerk, pPrev, NULL);
            pPrev->pNext = pSeg;
            pPrev = pSeg;
            
            dT0 += dT4;
            SKA_Trace_GetTracePoint(dT0, pSeg, &dX0, &dV0, &dA0);
        }
        
        SKA_TraceSegment *pSeg = (SKA_TraceSegment *)malloc(sizeof(SKA_TraceSegment));
        SKA_Trace_SetTraceSegment(pSeg, dT0, dX0, 0.0, 0.0, 0.0, pPrev, NULL);
        pPrev->pNext = pSeg;
        pPrev = pSeg;
        
    } else if (fabs(dStartA - dMaxAcceleration) < SKA_FlOAT_ERROR) {
        // 加速、匀速、减速
        
        // 计算加速到MaxVelocity且匀速阶段时长为0的位移阈值
        dT1 = fabs(dMaxVelocity - dStartV - pow(dMaxAcceleration, 2.0) / (2.0 * dMaxJerk)) / dMaxAcceleration;
        dT2 = dMaxAcceleration/ dMaxJerk ;
        dT3 = 0;
        dT4 = dMaxAcceleration/ dMaxJerk ;
        dT5 = fabs(dMaxVelocity / dMaxAcceleration- dT4);
        SKA_TraceSegment tmpSeg;
        dT0 = 0.0;
        dX0 = 0.0;
        dV0 = dStartV;
        dA0 = dMaxAcceleration;
        SKA_Trace_SetTraceSegment(&tmpSeg, dT0, dX0, dV0, dA0, 0.0, NULL, NULL);
        dT0 += dT1;
        SKA_Trace_GetTracePoint(dT0, &tmpSeg, &dX0, &dV0, &dA0);
        SKA_Trace_SetTraceSegment(&tmpSeg, dT0, dX0, dV0, dA0, -dMaxJerk, NULL, NULL);
        dT0 += dT2;
        SKA_Trace_GetTracePoint(dT0, &tmpSeg, &dX0, &dV0, &dA0);
        SKA_Trace_SetTraceSegment(&tmpSeg, dT0, dX0, dV0, dA0, 0.0, NULL, NULL);
        dT0 += dT3;
        SKA_Trace_GetTracePoint(dT0, &tmpSeg, &dX0, &dV0, &dA0);
        SKA_Trace_SetTraceSegment(&tmpSeg, dT0, dX0, dV0, dA0, -dMaxJerk, NULL, NULL);
        dT0 += dT4;
        SKA_Trace_GetTracePoint(dT0, &tmpSeg, &dX0, &dV0, &dA0);
        SKA_Trace_SetTraceSegment(&tmpSeg, dT0, dX0, dV0, dA0, 0.0, NULL, NULL);
        dT0 += dT5;
        SKA_Trace_GetTracePoint(dT0, &tmpSeg, &dX0, &dV0, &dA0);
        SKA_Trace_SetTraceSegment(&tmpSeg, dT0, dX0, dV0, dA0, dMaxJerk, NULL, NULL);
        dT0 += dT4;
        SKA_Trace_GetTracePoint(dT0, &tmpSeg, &dX0, &dV0, &dA0);
        
        double s0 = dX0;
        
        double deltaX = dTargetPos;
        if (deltaX >= s0) {
            // 加速到MaxVelocity，插入匀速阶段
            
            dT3 = fabs(deltaX - s0) / dMaxVelocity;
            
        } else {
            // 加速到maxV<MaxVelocity，匀速阶段时长为0
            
            // 二分法计算maxV
            lo = dStartV;
            hi = dMaxVelocity;
            do {
                double maxV = (lo + hi) / 2.0;
                dT1 = fabs(maxV - dStartV - pow(dMaxAcceleration, 2.0) / (2.0 * dMaxJerk )) / dMaxAcceleration;
                dT2 = dMaxAcceleration/ dMaxJerk ;
                dT3 = 0.0;
                dT4 = dMaxAcceleration/ dMaxJerk ;
                dT5 = fabs(maxV / dMaxAcceleration- dT4);
                SKA_TraceSegment tmpSeg;
                dT0 = 0.0;
                dX0 = 0.0;
                dV0 = dStartV;
                dA0 = dMaxAcceleration;
                SKA_Trace_SetTraceSegment(&tmpSeg, dT0, dX0, dV0, dA0, 0.0, NULL, NULL);
                tmpSeg.dT0 = dT0;
                tmpSeg.dX0 = dX0;
                tmpSeg.dV0 = dV0;
                tmpSeg.dA0 = dA0;
                tmpSeg.dJerk = 0.0;
                dT0 += dT1;
                SKA_Trace_GetTracePoint(dT0, &tmpSeg, &dX0, &dV0, &dA0);
                SKA_Trace_SetTraceSegment(&tmpSeg, dT0, dX0, dV0, dA0, -dMaxJerk, NULL, NULL);
                dT0 += dT2;
                SKA_Trace_GetTracePoint(dT0, &tmpSeg, &dX0, &dV0, &dA0);
                SKA_Trace_SetTraceSegment(&tmpSeg, dT0, dX0, dV0, dA0, 0.0, NULL, NULL);
                dT0 += dT3;
                SKA_Trace_GetTracePoint(dT0, &tmpSeg, &dX0, &dV0, &dA0);
                SKA_Trace_SetTraceSegment(&tmpSeg, dT0, dX0, dV0, dA0, -dMaxJerk, NULL, NULL);
                dT0 += dT4;
                SKA_Trace_GetTracePoint(dT0, &tmpSeg, &dX0, &dV0, &dA0);
                SKA_Trace_SetTraceSegment(&tmpSeg, dT0, dX0, dV0, dA0, 0.0, NULL, NULL);
                dT0 += dT5;
                SKA_Trace_GetTracePoint(dT0, &tmpSeg, &dX0, &dV0, &dA0);
                SKA_Trace_SetTraceSegment(&tmpSeg, dT0, dX0, dV0, dA0, dMaxJerk, NULL, NULL);
                dT0 += dT4;
                SKA_Trace_GetTracePoint(dT0, &tmpSeg, &dX0, &dV0, &dA0);

                s0 = dX0;
                if (deltaX > s0) {
                    lo = maxV;
                } else {
                    hi = maxV;
                }
            } while (hi - lo >= SKA_FlOAT_ERROR);
            
        }
        
        // 生成轨迹
        dT0 = 0.0;
        dX0 = 0.0;
        dV0 = dStartV;
        dA0 = dMaxAcceleration;
        
        if (dT1 > SKA_FlOAT_ERROR) {
            SKA_TraceSegment *pSeg = (SKA_TraceSegment *)malloc(sizeof(SKA_TraceSegment));
            SKA_Trace_SetTraceSegment(pSeg, dT0, dX0, dV0, dA0, 0.0, pPrev, NULL);
            pPrev->pNext = pSeg;
            pPrev = pSeg;
            
            dT0 += dT1;
            SKA_Trace_GetTracePoint(dT0, pSeg, &dX0, &dV0, &dA0);;
        }
        
        if (dT2 > SKA_FlOAT_ERROR) {
            SKA_TraceSegment *pSeg = (SKA_TraceSegment *)malloc(sizeof(SKA_TraceSegment));
            SKA_Trace_SetTraceSegment(pSeg, dT0, dX0, dV0, dA0, -dMaxJerk, pPrev, NULL);
            pPrev->pNext = pSeg;
            pPrev = pSeg;
            
            dT0 += dT2;
            SKA_Trace_GetTracePoint(dT0, pSeg, &dX0, &dV0, &dA0);
        }
        
        if (dT3 > SKA_FlOAT_ERROR) {
            SKA_TraceSegment *pSeg = (SKA_TraceSegment *)malloc(sizeof(SKA_TraceSegment));
            SKA_Trace_SetTraceSegment(pSeg, dT0, dX0, dV0, dA0, 0.0, pPrev, NULL);
            pPrev->pNext = pSeg;
            pPrev = pSeg;
            
            dT0 += dT3;
            SKA_Trace_GetTracePoint(dT0, pSeg, &dX0, &dV0, &dA0);
        }
        
        if (dT4 > SKA_FlOAT_ERROR) {
            SKA_TraceSegment *pSeg = (SKA_TraceSegment *)malloc(sizeof(SKA_TraceSegment));
            SKA_Trace_SetTraceSegment(pSeg, dT0, dX0, dV0, dA0, -dMaxJerk, pPrev, NULL);
            pPrev->pNext = pSeg;
            pPrev = pSeg;
            
            dT0 += dT4;
            SKA_Trace_GetTracePoint(dT0, pSeg, &dX0, &dV0, &dA0);
        }
        
        if (dT5 > SKA_FlOAT_ERROR) {
            SKA_TraceSegment *pSeg = (SKA_TraceSegment *)malloc(sizeof(SKA_TraceSegment));
            SKA_Trace_SetTraceSegment(pSeg, dT0, dX0, dV0, dA0, 0.0, pPrev, NULL);
            pPrev->pNext = pSeg;
            pPrev = pSeg;
            
            dT0 += dT5;
            SKA_Trace_GetTracePoint(dT0, pSeg, &dX0, &dV0, &dA0);
        }
        
        if (dT4 > SKA_FlOAT_ERROR) {
            SKA_TraceSegment *pSeg = (SKA_TraceSegment *)malloc(sizeof(SKA_TraceSegment));
            SKA_Trace_SetTraceSegment(pSeg, dT0, dX0, dV0, dA0, dMaxJerk, pPrev, NULL);
            pPrev->pNext = pSeg;
            pPrev = pSeg;
            
            dT0 += dT4;
            SKA_Trace_GetTracePoint(dT0, pSeg, &dX0, &dV0, &dA0);
        }
        
        SKA_TraceSegment *pSeg = (SKA_TraceSegment *)malloc(sizeof(SKA_TraceSegment));
        SKA_Trace_SetTraceSegment(pSeg, dT0, dX0, 0.0, 0.0, 0.0, pPrev, NULL);
        pPrev->pNext = pSeg;
        pPrev = pSeg;
        
    } else {
        // 减速、加速、匀速、减速
        
        // 计算加速到MaxVelocity且匀速阶段时长为0的位移阈值
        dT1 = dMaxAcceleration/ dMaxJerk ;
        if (fabs(dMaxVelocity - dStartV + 0.5 * pow(dMaxAcceleration, 2.0) / dMaxJerk) < pow(dMaxAcceleration, 2.0) / dMaxJerk) {
            dT2 = sqrt(fabs(dMaxVelocity - dStartV + 0.5 * pow(dMaxAcceleration, 2.0) / dMaxJerk) / dMaxJerk);
            dT3 = 0.0;
        } else {
            dT2 = dMaxAcceleration/ dMaxJerk ;
            dT3 = fabs(fabs(dMaxVelocity - dStartV + 0.5 * pow(dMaxAcceleration, 2.0) / dMaxJerk) / dMaxAcceleration - dT2);
        }
        dT4 = 0.0;
        if (dMaxVelocity < pow(dMaxAcceleration, 2.0) / dMaxJerk) {
            dT5 = sqrt(dMaxVelocity / dMaxJerk);
            dT6 = 0.0;
        } else {
            dT5 = dMaxAcceleration/ dMaxJerk;
            dT6 = fabs(dMaxVelocity / dMaxAcceleration - dT5);
        }
        SKA_TraceSegment tmpSeg;
        dT0 = 0.0;
        dX0 = 0.0;
        dV0 = dStartV;
        dA0 = - dMaxAcceleration;
        SKA_Trace_SetTraceSegment(&tmpSeg, dT0, dX0, dV0, dA0, dMaxJerk, NULL, NULL);
        dT0 += dT1;
        SKA_Trace_GetTracePoint(dT0, &tmpSeg, &dX0, &dV0, &dA0);
        SKA_Trace_SetTraceSegment(&tmpSeg, dT0, dX0, dV0, dA0, dMaxJerk, NULL, NULL);
        dT0 += dT2;
        SKA_Trace_GetTracePoint(dT0, &tmpSeg, &dX0, &dV0, &dA0);
        SKA_Trace_SetTraceSegment(&tmpSeg, dT0, dX0, dV0, dA0, 0.0, NULL, NULL);
        dT0 += dT3;
        SKA_Trace_GetTracePoint(dT0, &tmpSeg, &dX0, &dV0, &dA0);
        SKA_Trace_SetTraceSegment(&tmpSeg, dT0, dX0, dV0, dA0, -dMaxJerk, NULL, NULL);
        dT0 += dT2;
        SKA_Trace_GetTracePoint(dT0, &tmpSeg, &dX0, &dV0, &dA0);
        SKA_Trace_SetTraceSegment(&tmpSeg, dT0, dX0, dV0, dA0, 0.0, NULL, NULL);
        dT0 += dT4;
        SKA_Trace_GetTracePoint(dT0, &tmpSeg, &dX0, &dV0, &dA0);
        SKA_Trace_SetTraceSegment(&tmpSeg, dT0, dX0, dV0, dA0, -dMaxJerk, NULL, NULL);
        dT0 += dT5;
        SKA_Trace_GetTracePoint(dT0, &tmpSeg, &dX0, &dV0, &dA0);
        SKA_Trace_SetTraceSegment(&tmpSeg, dT0, dX0, dV0, dA0, 0.0, NULL, NULL);
        dT0 += dT6;
        SKA_Trace_GetTracePoint(dT0, &tmpSeg, &dX0, &dV0, &dA0);
        SKA_Trace_SetTraceSegment(&tmpSeg, dT0, dX0, dV0, dA0, dMaxJerk, NULL, NULL);
        dT0 += dT5;
        SKA_Trace_GetTracePoint(dT0, &tmpSeg, &dX0, &dV0, &dA0);

        double s0 = dX0;
        
        double deltaX = dTargetPos;
        if (deltaX >= s0) {
            // 加速到MaxVelocity，插入匀速阶段
            
            dT4 = fabs(deltaX - s0) / dMaxVelocity;
            
            // 生成轨迹
            dT0 = 0.0;
            dX0 = 0.0;
            dV0 = dStartV;
            dA0 = - dMaxAcceleration;
            if (dT1 > SKA_FlOAT_ERROR) {
                SKA_TraceSegment *pSeg = (SKA_TraceSegment *)malloc(sizeof(SKA_TraceSegment));
                SKA_Trace_SetTraceSegment(pSeg, dT0, dX0, dV0, dA0, dMaxJerk, pPrev, NULL);
                pPrev->pNext = pSeg;
                pPrev = pSeg;
                
                dT0 += dT1;
                SKA_Trace_GetTracePoint(dT0, pSeg, &dX0, &dV0, &dA0);
            }
            
            if (dT2 > SKA_FlOAT_ERROR) {
                SKA_TraceSegment *pSeg = (SKA_TraceSegment *)malloc(sizeof(SKA_TraceSegment));
                SKA_Trace_SetTraceSegment(pSeg, dT0, dX0, dV0, dA0, dMaxJerk, pPrev, NULL);
                pPrev->pNext = pSeg;
                pPrev = pSeg;
                
                dT0 += dT2;
                SKA_Trace_GetTracePoint(dT0, pSeg, &dX0, &dV0, &dA0);
            }
            
            if (dT3 > SKA_FlOAT_ERROR) {
                SKA_TraceSegment *pSeg = (SKA_TraceSegment *)malloc(sizeof(SKA_TraceSegment));
                SKA_Trace_SetTraceSegment(pSeg, dT0, dX0, dV0, dA0, 0.0, pPrev, NULL);
                pPrev->pNext = pSeg;
                pPrev = pSeg;
                
                dT0 += dT3;
                SKA_Trace_GetTracePoint(dT0, pSeg, &dX0, &dV0, &dA0);
            }
            
            if (dT2 > SKA_FlOAT_ERROR) {
                SKA_TraceSegment *pSeg = (SKA_TraceSegment *)malloc(sizeof(SKA_TraceSegment));
                SKA_Trace_SetTraceSegment(pSeg, dT0, dX0, dV0, dA0, -dMaxJerk, pPrev, NULL);
                pPrev->pNext = pSeg;
                pPrev = pSeg;
                
                dT0 += dT2;
                SKA_Trace_GetTracePoint(dT0, pSeg, &dX0, &dV0, &dA0);
            }
            
            if (dT4 > SKA_FlOAT_ERROR) {
                SKA_TraceSegment *pSeg = (SKA_TraceSegment *)malloc(sizeof(SKA_TraceSegment));
                SKA_Trace_SetTraceSegment(pSeg, dT0, dX0, dV0, dA0, 0.0, pPrev, NULL);
                pPrev->pNext = pSeg;
                pPrev = pSeg;
                
                dT0 += dT4;
                SKA_Trace_GetTracePoint(dT0, pSeg, &dX0, &dV0, &dA0);
            }
            
            if (dT5 > SKA_FlOAT_ERROR) {
                SKA_TraceSegment *pSeg = (SKA_TraceSegment *)malloc(sizeof(SKA_TraceSegment));
                SKA_Trace_SetTraceSegment(pSeg, dT0, dX0, dV0, dA0, -dMaxJerk, pPrev, NULL);
                pPrev->pNext = pSeg;
                pPrev = pSeg;
                
                dT0 += dT5;
                SKA_Trace_GetTracePoint(dT0, pSeg, &dX0, &dV0, &dA0);
            }
            
            if (dT6 > SKA_FlOAT_ERROR) {
                SKA_TraceSegment *pSeg = (SKA_TraceSegment *)malloc(sizeof(SKA_TraceSegment));
                SKA_Trace_SetTraceSegment(pSeg, dT0, dX0, dV0, dA0, 0.0, pPrev, NULL);
                pPrev->pNext = pSeg;
                pPrev = pSeg;
                
                dT0 += dT6;
                SKA_Trace_GetTracePoint(dT0, pSeg, &dX0, &dV0, &dA0);
            }
            
            if (dT5 > SKA_FlOAT_ERROR) {
                SKA_TraceSegment *pSeg = (SKA_TraceSegment *)malloc(sizeof(SKA_TraceSegment));
                SKA_Trace_SetTraceSegment(pSeg, dT0, dX0, dV0, dA0, dMaxJerk, pPrev, NULL);
                pPrev->pNext = pSeg;
                pPrev = pSeg;
                
                dT0 += dT5;
                SKA_Trace_GetTracePoint(dT0, pSeg, &dX0, &dV0, &dA0);
            }
            
            SKA_TraceSegment *pSeg = (SKA_TraceSegment *)malloc(sizeof(SKA_TraceSegment));
            SKA_Trace_SetTraceSegment(pSeg, dT0, dX0, 0.0, 0.0, 0.0, pPrev, NULL);
            pPrev->pNext = pSeg;
            pPrev = pSeg;
            
        } else {
            
            // 计算恰好无加速阶段的位移阈值
            dT1 = dMaxAcceleration / dMaxJerk ;
            if (fabs(dStartV - 0.5 * pow(dMaxAcceleration, 2.0) / dMaxJerk ) < pow(dMaxAcceleration, 2.0) / dMaxJerk) {
                dT2 = sqrt(fabs(dStartV - 0.5 * pow(dMaxAcceleration, 2.0) / dMaxJerk) / dMaxJerk);
                dT3 = 0;
            } else {
                dT2 = dMaxAcceleration / dMaxJerk ;
                dT3 = fabs(fabs(dStartV - 0.5 * pow(dMaxAcceleration, 2.0) / dMaxJerk) / dMaxAcceleration - dT2);
            }
            SKA_TraceSegment tmpSeg;
            dT0 = 0.0;
            dX0 = 0.0;
            dV0 = dStartV;
            dA0 = - dMaxAcceleration;
            SKA_Trace_SetTraceSegment(&tmpSeg, dT0, dX0, dV0, dA0, dMaxJerk, NULL, NULL);
            dT0 += dT1;
            SKA_Trace_GetTracePoint(dT0, &tmpSeg, &dX0, &dV0, &dA0);
            SKA_Trace_SetTraceSegment(&tmpSeg, dT0, dX0, dV0, dA0, -dMaxJerk, NULL, NULL);
            dT0 += dT2;
            SKA_Trace_GetTracePoint(dT0, &tmpSeg, &dX0, &dV0, &dA0);
            SKA_Trace_SetTraceSegment(&tmpSeg, dT0, dX0, dV0, dA0, 0.0, NULL, NULL);
            dT0 += dT3;
            SKA_Trace_GetTracePoint(dT0, &tmpSeg, &dX0, &dV0, &dA0);
            SKA_Trace_SetTraceSegment(&tmpSeg, dT0, dX0, dV0, dA0, dMaxJerk, NULL, NULL);
            dT0 += dT2;
            SKA_Trace_GetTracePoint(dT0, &tmpSeg, &dX0, &dV0, &dA0);

            s0 = dX0;
            
            if (deltaX >= s0) {
                // 加速阶段速度变化量，匀速阶段时长为0
                
                // 二分法计算deltaV
                lo = 0.0;
                hi = fabs(dMaxVelocity - (dStartV - 0.5 * pow(dMaxAcceleration, 2.0) / dMaxJerk));
                do {
                    double deltaV = (lo + hi) / 2.0;
                    dT1 = dMaxAcceleration / dMaxJerk;
                    if (deltaV < pow(dMaxAcceleration, 2.0) / dMaxJerk) {
                        dT2 = sqrt(deltaV / dMaxJerk);
                        dT3 = 0.0;
                    } else {
                        dT2 = dMaxAcceleration / dMaxJerk ;
                        dT3 = fabs(deltaV - pow(dMaxAcceleration, 2.0) / dMaxJerk) / dMaxAcceleration;
                    }
                    dT4 = 0.0;
                    double maxV = fabs(dStartV - 0.5 * dMaxAcceleration * dT1 + deltaV);
                    if (maxV < pow(dMaxAcceleration, 2.0) / dMaxJerk) {
                        dT5 = sqrt(maxV / dMaxJerk);
                        dT6 = 0.0;
                    } else {
                        dT5 = dMaxAcceleration / dMaxJerk ;
                        dT6 = fabs(maxV - pow(dMaxAcceleration, 2.0) / dMaxJerk) / dMaxAcceleration;
                    }
                    SKA_TraceSegment tmpSeg;
                    dT0 = 0.0;
                    dX0 = 0.0;
                    dV0 = dStartV;
                    dA0 = - dMaxAcceleration;
                    SKA_Trace_SetTraceSegment(&tmpSeg, dT0, dX0, dV0, dA0, dMaxJerk, NULL, NULL);
                    dT0 += dT1;
                    SKA_Trace_GetTracePoint(dT0, &tmpSeg, &dX0, &dV0, &dA0);
                    SKA_Trace_SetTraceSegment(&tmpSeg, dT0, dX0, dV0, dA0, dMaxJerk, NULL, NULL);
                    dT0 += dT2;
                    SKA_Trace_GetTracePoint(dT0, &tmpSeg, &dX0, &dV0, &dA0);
                    SKA_Trace_SetTraceSegment(&tmpSeg, dT0, dX0, dV0, dA0, 0.0, NULL, NULL);
                    dT0 += dT3;
                    SKA_Trace_GetTracePoint(dT0, &tmpSeg, &dX0, &dV0, &dA0);
                    SKA_Trace_SetTraceSegment(&tmpSeg, dT0, dX0, dV0, dA0, -dMaxJerk, NULL, NULL);
                    dT0 += dT2;
                    SKA_Trace_GetTracePoint(dT0, &tmpSeg, &dX0, &dV0, &dA0);
                    SKA_Trace_SetTraceSegment(&tmpSeg, dT0, dX0, dV0, dA0, 0.0, NULL, NULL);
                    dT0 += dT4;
                    SKA_Trace_GetTracePoint(dT0, &tmpSeg, &dX0, &dV0, &dA0);
                    SKA_Trace_SetTraceSegment(&tmpSeg, dT0, dX0, dV0, dA0, -dMaxJerk, NULL, NULL);
                    dT0 += dT5;
                    SKA_Trace_GetTracePoint(dT0, &tmpSeg, &dX0, &dV0, &dA0);
                    SKA_Trace_SetTraceSegment(&tmpSeg, dT0, dX0, dV0, dA0, 0.0, NULL, NULL);
                    dT0 += dT6;
                    SKA_Trace_GetTracePoint(dT0, &tmpSeg, &dX0, &dV0, &dA0);
                    SKA_Trace_SetTraceSegment(&tmpSeg, dT0, dX0, dV0, dA0, dMaxJerk, NULL, NULL);
                    dT0 += dT5;
                    SKA_Trace_GetTracePoint(dT0, &tmpSeg, &dX0, &dV0, &dA0);
                    
                    double s0 = dX0;
                    
                    if (deltaX > s0) {
                        lo = deltaV;
                    } else {
                        hi = deltaV;
                    }
                } while (hi - lo >= SKA_FlOAT_ERROR);
                
                // 生成轨迹
                
                dT0 = 0.0;
                dX0 = 0.0;
                dV0 = dStartV;
                dA0 = - dMaxAcceleration;
                if (dT1 > SKA_FlOAT_ERROR) {
                    SKA_TraceSegment *pSeg = (SKA_TraceSegment *)malloc(sizeof(SKA_TraceSegment));
                    SKA_Trace_SetTraceSegment(pSeg, dT0, dX0, dV0, dA0, dMaxJerk, pPrev, NULL);
                    
                    dT0 += dT1;
                    SKA_Trace_GetTracePoint(dT0, pSeg, &dX0, &dV0, &dA0);
                }
                
                if (dT2 > SKA_FlOAT_ERROR) {
                    SKA_TraceSegment *pSeg = (SKA_TraceSegment *)malloc(sizeof(SKA_TraceSegment));
                    SKA_Trace_SetTraceSegment(pSeg, dT0, dX0, dV0, dA0, dMaxJerk, pPrev, NULL);
                    pPrev->pNext = pSeg;
                    pPrev = pSeg;
                    
                    dT0 += dT2;
                    SKA_Trace_GetTracePoint(dT0, pSeg, &dX0, &dV0, &dA0);
                }
                
                if (dT3 > SKA_FlOAT_ERROR) {
                    SKA_TraceSegment *pSeg = (SKA_TraceSegment *)malloc(sizeof(SKA_TraceSegment));
                    SKA_Trace_SetTraceSegment(pSeg, dT0, dX0, dV0, dA0, 0.0, pPrev, NULL);
                    pPrev->pNext = pSeg;
                    pPrev = pSeg;
                    
                    dT0 += dT3;
                    SKA_Trace_GetTracePoint(dT0, pSeg, &dX0, &dV0, &dA0);
                }
                
                if (dT2 > SKA_FlOAT_ERROR) {
                    SKA_TraceSegment *pSeg = (SKA_TraceSegment *)malloc(sizeof(SKA_TraceSegment));
                    SKA_Trace_SetTraceSegment(pSeg, dT0, dX0, dV0, dA0, -dMaxJerk, pPrev, NULL);
                    pPrev->pNext = pSeg;
                    pPrev = pSeg;
                    
                    dT0 += dT2;
                    SKA_Trace_GetTracePoint(dT0, pSeg, &dX0, &dV0, &dA0);
                }
                
                if (dT4 > SKA_FlOAT_ERROR) {
                    SKA_TraceSegment *pSeg = (SKA_TraceSegment *)malloc(sizeof(SKA_TraceSegment));
                    SKA_Trace_SetTraceSegment(pSeg, dT0, dX0, dV0, dA0, 0.0, pPrev, NULL);
                    pPrev->pNext = pSeg;
                    pPrev = pSeg;
                    
                    dT0 += dT4;
                    SKA_Trace_GetTracePoint(dT0, pSeg, &dX0, &dV0, &dA0);
                }
                
                if (dT5 > SKA_FlOAT_ERROR) {
                    SKA_TraceSegment *pSeg = (SKA_TraceSegment *)malloc(sizeof(SKA_TraceSegment));
                    SKA_Trace_SetTraceSegment(pSeg, dT0, dX0, dV0, dA0, -dMaxJerk, pPrev, NULL);
                    pPrev->pNext = pSeg;
                    pPrev = pSeg;
                    
                    dT0 += dT5;
                    SKA_Trace_GetTracePoint(dT0, pSeg, &dX0, &dV0, &dA0);
                }
                
                if (dT6 > SKA_FlOAT_ERROR) {
                    SKA_TraceSegment *pSeg = (SKA_TraceSegment *)malloc(sizeof(SKA_TraceSegment));
                    SKA_Trace_SetTraceSegment(pSeg, dT0, dX0, dV0, dA0, 0.0, pPrev, NULL);
                    pPrev->pNext = pSeg;
                    pPrev = pSeg;
                    
                    dT0 += dT6;
                    SKA_Trace_GetTracePoint(dT0, pSeg, &dX0, &dV0, &dA0);
                }
                
                if (dT5 > SKA_FlOAT_ERROR) {
                    SKA_TraceSegment *pSeg = (SKA_TraceSegment *)malloc(sizeof(SKA_TraceSegment));
                    SKA_Trace_SetTraceSegment(pSeg, dT0, dX0, dV0, dA0, dMaxJerk, pPrev, NULL);
                    pPrev->pNext = pSeg;
                    pPrev = pSeg;
                    
                    dT0 += dT5;
                    SKA_Trace_GetTracePoint(dT0, pSeg, &dX0, &dV0, &dA0);
                }
                
                SKA_TraceSegment *pSeg = (SKA_TraceSegment *)malloc(sizeof(SKA_TraceSegment));
                SKA_Trace_SetTraceSegment(pSeg, dT0, dX0, 0.0, 0.0, 0.0, pPrev, NULL);
                pPrev->pNext = pSeg;
                pPrev = pSeg;
                
            } else {
                // 无加速和匀速阶段
                
                // 二分法计算轨迹参数
                lo = 0.0;
                hi = dMaxAcceleration / dMaxJerk ;
                do {
                    dT1 = (lo + hi) / 2.0;
                    double dT4max = dMaxAcceleration / dMaxJerk ;
                    if ((2.0 * dMaxAcceleration - dMaxJerk * dT1) * dT1 + 0.5 * dMaxAcceleration * dT4max < dStartV) {
                        dT2 = dT1;
                        dT3 = fabs(dStartV - ((2.0 * dMaxAcceleration - dMaxJerk * dT1) * dT1 + 0.5 * dMaxAcceleration * dT4max)) / dMaxAcceleration;
                        dT4 = dT4max;
                    } else {
                        double ka = 1.0;
                        double kb = 2.0 * (dMaxAcceleration / dMaxJerk - dT1);
                        double kc = pow(dMaxAcceleration, 2.0) / (2.0 * pow(dMaxJerk, 2.0)) - dStartV / dMaxJerk;
                        dT2 = (- kb + sqrt(pow(kb, 2.0) - 4 * ka * kc)) / (2.0 * ka);
                        dT3 = 0.0;
                        dT4 = fabs(dMaxAcceleration- dMaxJerk * dT1 + dMaxJerk * dT2) / dMaxJerk;
                    }
                    
                    SKA_TraceSegment tmpSeg;
                    dT0 = 0.0;
                    dX0 = 0.0;
                    dV0 = dStartV;
                    dA0 = - dMaxAcceleration;
                    SKA_Trace_SetTraceSegment(&tmpSeg, dT0, dX0, dV0, dA0, dMaxJerk, NULL, NULL);
                    dT0 += dT1;
                    SKA_Trace_GetTracePoint(dT0, &tmpSeg, &dX0, &dV0, &dA0);
                    SKA_Trace_SetTraceSegment(&tmpSeg, dT0, dX0, dV0, dA0, -dMaxJerk, NULL, NULL);
                    dT0 += dT2;
                    SKA_Trace_GetTracePoint(dT0, &tmpSeg, &dX0, &dV0, &dA0);
                    SKA_Trace_SetTraceSegment(&tmpSeg, dT0, dX0, dV0, dA0, 0.0, NULL, NULL);
                    dT0 += dT3;
                    SKA_Trace_GetTracePoint(dT0, &tmpSeg, &dX0, &dV0, &dA0);
                    SKA_Trace_SetTraceSegment(&tmpSeg, dT0, dX0, dV0, dA0, dMaxJerk, NULL, NULL);
                    dT0 += dT4;
                    SKA_Trace_GetTracePoint(dT0, &tmpSeg, &dX0, &dV0, &dA0);

                    double s0 = dX0;
                    
                    if (deltaX > s0) {
                        lo = dT1;
                    } else {
                        hi = dT1;
                    }
                } while (hi - lo >= SKA_FlOAT_ERROR);
                
                // 生成轨迹
                
                dT0 = 0.0;
                dX0 = 0.0;
                dV0 =  dStartV;
                dA0 = - dMaxAcceleration;
                if (dT1 > SKA_FlOAT_ERROR) {
                    SKA_TraceSegment *pSeg = (SKA_TraceSegment *)malloc(sizeof(SKA_TraceSegment));
                    SKA_Trace_SetTraceSegment(pSeg, dT0, dX0, dV0, dA0, dMaxJerk, pPrev, NULL);
                    pPrev->pNext = pSeg;
                    pPrev = pSeg;
                    
                    dT0 += dT1;
                    SKA_Trace_GetTracePoint(dT0, pSeg, &dX0, &dV0, &dA0);
                }
                
                if (dT2 > SKA_FlOAT_ERROR) {
                    SKA_TraceSegment *pSeg = (SKA_TraceSegment *)malloc(sizeof(SKA_TraceSegment));
                    SKA_Trace_SetTraceSegment(pSeg, dT0, dX0, dV0, dA0, -dMaxJerk, pPrev, NULL);
                    pPrev->pNext = pSeg;
                    pPrev = pSeg;
                    
                    dT0 += dT2;
                    SKA_Trace_GetTracePoint(dT0, pSeg, &dX0, &dV0, &dA0);
                }
                
                if (dT3 > SKA_FlOAT_ERROR) {
                    SKA_TraceSegment *pSeg = (SKA_TraceSegment *)malloc(sizeof(SKA_TraceSegment));
                    SKA_Trace_SetTraceSegment(pSeg, dT0, dX0, dV0, dA0, 0.0, pPrev, NULL);
                    pPrev->pNext = pSeg;
                    pPrev = pSeg;
                    
                    dT0 += dT3;
                    SKA_Trace_GetTracePoint(dT0, pSeg, &dX0, &dV0, &dA0);
                }
                
                if (dT4 > SKA_FlOAT_ERROR) {
                    SKA_TraceSegment *pSeg = (SKA_TraceSegment *)malloc(sizeof(SKA_TraceSegment));
                    SKA_Trace_SetTraceSegment(pSeg, dT0, dX0, dV0, dA0, dMaxJerk, pPrev, NULL);
                    pPrev->pNext = pSeg;
                    pPrev = pSeg;
                    
                    dT0 += dT4;
                    SKA_Trace_GetTracePoint(dT0, pSeg, &dX0, &dV0, &dA0);
                }
                
                SKA_TraceSegment *pSeg = (SKA_TraceSegment *)malloc(sizeof(SKA_TraceSegment));
                SKA_Trace_SetTraceSegment(pSeg, dT0, dX0, 0.0, 0.0, 0.0, pPrev, NULL);
                pPrev->pNext = pSeg;
                pPrev = pSeg;
                
            }
            
        }
        
    }
    
    pPrev->pNext = pHead;
    pHead->pPrev = pPrev;
    *ppHead = pHead;

    return 0;
}
