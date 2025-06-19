/**
  ******************************************************************************
  * @file       : SKA_InertialFilter.c
  * @brief      : 惯性环节
  * @author     : ZhangYi
  * @version    : None
  * @date       : 2024/10/24
  ******************************************************************************
  */
//

#include <stddef.h> 
#include <stdio.h>

#include "SKA_InertialFilter.h"

int8_t SKA_IneFil_Init(SKA_InertialFilter *pIneFil, double dTs,double dTc, double dInitOutput) {
    /******************** 函数参数合法性检验 ********************/
    if (pIneFil == NULL || dTs <= 0 || dTc < 0) {
        printf("Invalid parameters of SKA_IneFil_Init().\n");
        return -1;
    }

    /******************** 初始化 ********************/
    pIneFil->dTs = dTs;
    pIneFil->dTc = dTc;
    pIneFil->dPrevOutput = dInitOutput;

    return 0;
}

int8_t SKA_IneFil_Run(SKA_InertialFilter *pIneFil, double dInput, double *pOutput) {
    /******************** 函数参数合法性检验 ********************/
    if (pIneFil == NULL || pOutput == NULL) {
        printf("Invalid parameters of SKA_IneFil_Run().\n");
        return -1;
    }

    /******************** 运行 ********************/
    double dAlpha = pIneFil->dTs / (pIneFil->dTc + pIneFil->dTs);
    *pOutput = (1 - dAlpha) * pIneFil->dPrevOutput + dAlpha * dInput;
    pIneFil->dPrevOutput = *pOutput;

    return 0;
}