/**
  ******************************************************************************
  * @file       : SKA_ZeroSpeedDetector.c
  * @brief      : 零速度检测器
  * @author     : ZhangYi
  * @version    : None
  * @date       : 2024/10/31
  ******************************************************************************
  */
//

#include <stddef.h> 
#include <float.h>
#include <math.h>
#include <stdio.h>

#include "SKA_ZeroSpeedDetector.h"

int8_t SKA_ZSD_Create(SKA_ZeroSpeedDetector *pZSD, double dTs, double dTimeLen, double dMaxSpeed) {
    /******************** 函数参数合法性检验 ********************/
    if (pZSD == NULL || dTs <= 0 || dTimeLen <= 0 || dMaxSpeed < 0) {
        printf("Invalid parameters of SKA_ZSD_Create().\n");
        return -1;
    }

    /******************** 创建零速度检测器 ********************/
    uint16_t nCap = (uint16_t)ceil(dTimeLen / dTs) + 1;
    SKA_CirBuf_Create(&pZSD->stPosCirBuf, nCap);
    pZSD->dTs = dTs;
    pZSD->dTimeLen = dTimeLen;
    pZSD->dMaxSpeed = dMaxSpeed;

    return 0;
}

int8_t SKA_ZSD_Run(SKA_ZeroSpeedDetector *pZSD, double dCurPos, bool *pIsZeroSpeed) {
    /******************** 函数参数合法性检验 ********************/
    if (pZSD == NULL || pIsZeroSpeed == NULL) {
        printf("Invalid parameters of SKA_ZSD_Run().\n");
        return -1;
    }

    /******************** 运行零速度检测器 ********************/

    // 存储当前位置测量值
    SKA_CirBuf_Add(&pZSD->stPosCirBuf, dCurPos);

    // 计算最大位置变化量
    double dMaxDertaPos = pZSD->dMaxSpeed * pZSD->dTimeLen;

    // 判断检测时域内的位置测量值变化量是否符合零速度要求
    if (pZSD->stPosCirBuf.nSize < pZSD->stPosCirBuf.nCapacity) {
        // 数据量不足
        *pIsZeroSpeed = 0;
    } else {
        double dMinPos = DBL_MAX;
        double dMaxPos = DBL_MIN;
        for (uint16_t i = 0; i < pZSD->stPosCirBuf.nCapacity; i++) {
            double dPos;
            SKA_CirBuf_Get(&(pZSD->stPosCirBuf), i, &dPos);
            if (dMinPos > dPos) {
                dMinPos = dPos;
            }
            if (dMaxPos < dPos) {
                dMaxPos = dPos;
            }
        }
        if (dMaxPos - dMinPos <= dMaxDertaPos) {
            *pIsZeroSpeed = 1;
        } else {
            *pIsZeroSpeed = 0;
        }
    }

    return 0;
}

int8_t SKA_ZSD_Destroy(SKA_ZeroSpeedDetector *pZSD) {
    /******************** 函数参数合法性检验 ********************/
    if (pZSD == NULL) {
        printf("Invalid parameters of SKA_ZSD_Destroy().\n");
        return -1;
    }

    /******************** 销毁零速度检测器 ********************/
    SKA_CirBuf_Destroy(&pZSD->stPosCirBuf);

    return 0;
}