/**
  ******************************************************************************
  * @file       : SKA_CannotDriveErrorDetector.c
  * @brief      : 无法驱动错误检测器
  * @author     : ZhangYi
  * @version    : None
  * @date       : 2024/11/06
  ******************************************************************************
  */
//

#include <stddef.h> 
#include <float.h>
#include <math.h>
#include <stdio.h>

#include "SKA_CannotDriveErrorDetector.h"

int8_t SKA_CDED_Create(SKA_CannotDriveErrorDetector *pCDED, double dTs, double dTimeLen, double dPosAccuracy, double dFreqThreshold) {
    /******************** 函数参数合法性检验 ********************/
    if (pCDED == NULL || dTs <= 0 || dTimeLen <= 0 || dPosAccuracy < 0 || dFreqThreshold < 0) {
        printf("Invalid parameters of SKA_CDED_Create().\n");
        return -1;
    }

    /******************** 创建 ********************/
    pCDED->dTs = dTs;
    pCDED->dTimeLen = dTimeLen;
    pCDED->dPosAccuracy = dPosAccuracy;
    pCDED->dFreqThreshold = dFreqThreshold;
    uint16_t nCap = (uint16_t)ceil(dTimeLen / dTs) + 1;
    SKA_CirBuf_Create(&(pCDED->stPosCirBuf), nCap);
    SKA_CirBuf_Create(&(pCDED->stFeqCirBuf), nCap);

    return 0;
}


int8_t SKA_CDED_Run(SKA_CannotDriveErrorDetector *pCDED, double dCurPos, double dCurFreq, bool *pCannotDrive) {
    /******************** 函数参数合法性检验 ********************/
    if (pCDED == NULL || pCannotDrive == NULL) {
        printf("Invalid parameters of SKA_CDED_Run().\n");
        return -1;
    }

    /******************** 运行 ********************/

    // 存储当前位置测量值、速度输出值
    SKA_CirBuf_Add(&pCDED->stPosCirBuf, dCurPos);
    SKA_CirBuf_Add(&pCDED->stFeqCirBuf, dCurFreq);

    if (pCDED->stPosCirBuf.nSize < pCDED->stPosCirBuf.nCapacity) {
        // 数据量不足
        *pCannotDrive = 0;
    } else {
        // 最早的频率输出是否超过阈值
        double dEarliestFreq;
        SKA_CirBuf_Get(&(pCDED->stFeqCirBuf), pCDED->stFeqCirBuf.nCapacity - 1, &dEarliestFreq);
        if (fabs(dEarliestFreq) > pCDED->dFreqThreshold) {
            // 判断位置测量值变化是否超过精度
            double dMinPos = DBL_MAX;
            double dMaxPos = DBL_MIN;
            for (uint16_t i = 0; i < pCDED->stPosCirBuf.nCapacity; i++) {
                double dPos;
                SKA_CirBuf_Get(&(pCDED->stPosCirBuf), i, &dPos);
                if (dMinPos > dPos) {
                    dMinPos = dPos;
                }
                if (dMaxPos < dPos) {
                    dMaxPos = dPos;
                }
            }
            if (dMaxPos - dMinPos <= pCDED->dPosAccuracy) {
                *pCannotDrive = 1;
            } else {
                *pCannotDrive = 0;
            }
        } else {
            *pCannotDrive = 0;
        }
    }

    return 0;
}


int8_t SKA_CDED_Destroy(SKA_CannotDriveErrorDetector *pCDED) {
    /******************** 函数参数合法性检验 ********************/
    if (pCDED == NULL) {
        printf("Invalid parameters of SKA_CDED_Destroy().\n");
        return -1;
    }

    /******************** 销毁 ********************/
    SKA_CirBuf_Destroy(&(pCDED->stPosCirBuf));
    SKA_CirBuf_Destroy(&(pCDED->stFeqCirBuf));

    return 0;
}