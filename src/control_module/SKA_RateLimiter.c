/**
  ******************************************************************************
  * @file       : SKA_RateLimiter.c
  * @brief      : 速率限制器
  * @author     : ZhangYi
  * @version    : None
  * @date       : 2024/10/24
  ******************************************************************************
  */
//

#include <stddef.h> 
#include <stdio.h>

#include "SKA_RateLimiter.h"

int8_t SKA_RatLim_Init(SKA_RateLimiter *pRL, double dRiseRate, double dFallRate, double dTs, double dPrevOutput) {
    /******************** 函数参数合法性检验 ********************/
    if (pRL == NULL || dRiseRate < 0 || dFallRate > 0 || dTs <= 0) {
        printf("Invalid parameters of SKA_RatLim_Init().\n");
        return -1;
    }

    /******************** 初始化 ********************/
    pRL->dRiseRate = dRiseRate;
    pRL->dFallRate = dFallRate;
    pRL->dTs = dTs;
    pRL->dPrevOutput = dPrevOutput;

    return 0;
}

int8_t SKA_RatLim_Run(SKA_RateLimiter *pRL, double dInput, double *pOutput) {
    /******************** 函数参数合法性检验 ********************/
    if (pRL == NULL || pOutput == NULL) {
        printf("Invalid parameters of SKA_RatLim_Run().\n");
        return -1;
    }

    /******************** 速率限制 ********************/
    double rate = (dInput - pRL->dPrevOutput) / pRL->dTs;
    if(rate < pRL->dFallRate){
        *pOutput = pRL->dPrevOutput + pRL->dFallRate * pRL->dTs;
    }else if(rate > pRL->dRiseRate){
        *pOutput = pRL->dPrevOutput + pRL->dRiseRate * pRL->dTs;
    }else{
        *pOutput = dInput;
    }
    pRL->dPrevOutput = *pOutput;

    return 0;
}