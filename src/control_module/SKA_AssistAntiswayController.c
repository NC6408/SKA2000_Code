/**
  ******************************************************************************
  * @file       : SKA_AssistAntiswayController.c
  * @brief      : 辅助防摇控制器
  * @author     : ZhangYi
  * @version    : None
  * @date       : 2024/10/31
  ******************************************************************************
  */
//

#include <stddef.h> 
#include <math.h>
#include <stdio.h>

#include "SKA_AssistAntiswayController.h"

int8_t SKA_AAC_Create(SKA_AssistAntiswayController *pAAC, bool bActiveSwayCtrl, double dTs, 
                      double dMaxVelocity, double dMaxAcceleration, double dIneFilTc, double dInitialVelocity) {
    /******************** 函数参数合法性检验 ********************/
    if (pAAC == NULL || dTs <= 0 || dMaxVelocity < 0 || dIneFilTc < 0) {
        printf("Invalid parameters of SKA_AAC_Create().\n");
        return -1;
    }

    /******************** 创建辅助防摇控制器 ********************/
    pAAC->bActiveSwayCtrl = bActiveSwayCtrl;
    pAAC->dTs = dTs;
    pAAC->dMaxVelocity = dMaxVelocity;
    SKA_RatLim_Init(&(pAAC->stRL), dMaxAcceleration, -dMaxAcceleration, dTs, dInitialVelocity);
    SKA_IneFil_Init(&(pAAC->stIF), dTs, dIneFilTc, dInitialVelocity);
    // 设置一个比所有情况下输入整形器持续时间更长的缓冲区
    double dDuration = 1.5 * 2 * SKA_PI * sqrt(50 / SKA_G);
    uint16_t nCap = (uint16_t)(dDuration / dTs);
    SKA_CirBuf_Create(&(pAAC->stCB), nCap);
    SKA_CirBuf_Init(&(pAAC->stCB), dInitialVelocity);

    return 0;
}

int8_t SKA_AAC_Run(SKA_AssistAntiswayController *pAAC, double dManualVelocity, 
                   SKA_InputShaperType eType, double dNaturalFreq, double dDampingRatio, 
                   double *pOutputVelocity) {
    /******************** 函数参数合法性检验 ********************/
    if (pAAC == NULL || dNaturalFreq <= 0 || dDampingRatio < 0 || dDampingRatio >= 1 || pOutputVelocity == NULL) {
        printf("Invalid parameters of SKA_AAC_Run().\n");
        return -1;
    }

    /******************** 运行辅助防摇控制器 ********************/

    /* 输入速度预处理 */

    // 限幅滤波
    double dSat;
    SKA_Sat_Run(dManualVelocity, -pAAC->dMaxVelocity, pAAC->dMaxVelocity, &dSat);

    // 限速滤波
    double dRL;
    SKA_RatLim_Run(&(pAAC->stRL), dSat, &dRL);

    // 惯性滤波
    double dIF;
    SKA_IneFil_Run(&(pAAC->stIF), dRL, &dIF);

    // 预处理后的输入速度
    double dProInputVel = dIF;

    /* 记录输入速度 */

    SKA_CirBuf_Add(&(pAAC->stCB), dProInputVel);

    /* 输入整形 */

    if (pAAC->bActiveSwayCtrl) {
        // 激活防摇控制

        // 在线计算脉冲序列
        SKA_InpSha_Init(&(pAAC->stIS), eType, dNaturalFreq, dDampingRatio);

        // 卷积运算
        *pOutputVelocity = 0;
        for (int i = 0; i < pAAC->stIS.nImpulseNum; i++) {
            double dT = pAAC->stIS.arrImpulseTime[i];
            uint16_t nOffset = (uint16_t)ceil(dT / pAAC->dTs);
            double dV;
            SKA_CirBuf_Get(&(pAAC->stCB), nOffset, &dV);
            double dA = pAAC->stIS.arrImpulseValue[i];
            *pOutputVelocity += dA * dV;
        }
    } else {
        // 未激活防摇控制

        *pOutputVelocity = dProInputVel;
    }
    
    return 0;
}

int8_t SKA_AAC_SetMaxVelocity(SKA_AssistAntiswayController *pAAC, double dMaxVelocity) {
    /******************** 函数参数合法性检验 ********************/
    if (pAAC == NULL || dMaxVelocity < 0) {
        printf("Invalid parameters of SKA_AAC_SetMaxVelocity().\n");
        return -1;
    }

    /******************** 设置辅助防摇控制器的最大速度 ********************/
    pAAC->dMaxVelocity = dMaxVelocity;

    return 0;
}


int8_t SKA_AAC_Destroy(SKA_AssistAntiswayController *pAAC) {
    /******************** 函数参数合法性检验 ********************/
    if (pAAC == NULL) {
        printf("Invalid parameters of SKA_AAC_Destroy().\n");
        return -1;
    }

    /******************** 销毁辅助防摇控制器 ********************/
    SKA_CirBuf_Destroy(&(pAAC->stCB));

    return 0;
}