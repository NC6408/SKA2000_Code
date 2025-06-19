/**
  ******************************************************************************
  * @file       : SKA_TraceSegment.h
  * @brief      : 轨迹段
  * @author     : ZhangYi
  * @version    : None
  * @date       : 2024/11/05
  ******************************************************************************
  */
//

#ifndef SKA2000_OHBC_CONTROLLER_SKA_TRACESEGMENT_H
#define SKA2000_OHBC_CONTROLLER_SKA_TRACESEGMENT_H

#include <stdint.h>

// 一条完整轨迹是由若干个轨迹段组成的双向链表

// 轨迹段
typedef struct SKA_TraceListNode {
    double dT0;                              // 开始时刻(s)
    double dX0;                              // 开始位置(m)
    double dV0;                              // 开始速度(m/s)
    double dA0;                              // 开始加速度(m/s^2)
    double dJerk;                            // 加加速度(m/s^3)
    struct SKA_TraceListNode *pPrev;         // 前一个轨迹段指针
    struct SKA_TraceListNode *pNext;         // 后一个轨迹段指针
} SKA_TraceSegment;

/**
  * @brief  设置轨迹段的属性
  * @param  pSeg 轨迹段的地址
  * @param  dT0 开始时刻(s)
  * @param  dX0 开始位置(m)
  * @param  dV0 开始速度(m/s)
  * @param  dA0 开始加速度(m/s^2)
  * @param  dJerk 加加速度(m/s^3)
  * @param  pPrev 前一个轨迹段指针
  * @param  pNext 后一个轨迹段指针
  * @return 错误码 {0: 正常}
  */
int8_t SKA_Trace_SetTraceSegment(SKA_TraceSegment* pSeg, double dT0, double dX0, double dV0, double dA0, double dJerk,
                               SKA_TraceSegment* pPrev, SKA_TraceSegment* pNext);

/**
  * @brief  获得轨迹链表中轨迹段的数量
  * @param  pSeg 轨迹头结点的地址
  * @param  pNum 保存轨迹段的数量的地址
  * @return 错误码 {0: 正常}
  */
int8_t SKA_Trace_GetTraceSegmentNum(const SKA_TraceSegment* pSeg, uint16_t *pNum);

/**
  * @brief  获得指定轨迹段在指定时刻的状态
  * @param  dCurTime 指定时刻(s)
  * @param  pSeg 轨迹段的地址
  * @param  pX 保存位置(m)的地址
  * @param  pV 保存速度(m/s)的地址
  * @param  pA 保存加速度(m/s^2)的地址
  * @return 错误码 {0: 正常}
  */
int8_t SKA_Trace_GetTracePoint(double dCurTime, const SKA_TraceSegment *pSeg, double *pX, double *pV, double *pA);

/**
 * @brief  获得轨迹链表中轨迹段的总时间(s)
 * @param  pHead 轨迹头结点的地址
 * @param  pTotalTime 保存总时间的地址
 * @return 错误码 {0: 正常}
 */
int8_t SKA_Trace_GetTotalTime(const SKA_TraceSegment *pHead, double *pTotalTime);

/**
 * @brief  获得轨迹的目标位置(m)
 * @param  pHead 轨迹头结点的地址
 * @param  pTargetPos 保存总时间的地址
 * @return 错误码 {0: 正常}
 */
int8_t SKA_Trace_GetTargetPos(const SKA_TraceSegment *pHead, double *pTargetPos);

/**
 * @brief  显示轨迹
 * @param  pHead 轨迹头节点的地址
 * @return 错误码 {0: 正常}
 */
int8_t SKA_Trace_Display(const SKA_TraceSegment* pHead);

/**
  * @brief  释放单向链表轨迹
  * @param  p 链表首元地址
  * @return 错误码 {0: 正常}
  */
int8_t SKA_Trace_FreeSinglyLinkedListTrace(SKA_TraceSegment* p);

#endif //SKA2000_OHBC_CONTROLLER_SKA_TRACESEGMENT_H