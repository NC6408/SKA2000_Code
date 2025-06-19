/**
  ******************************************************************************
  * @file       : SKA_InputShaper.c
  * @brief      : 输入整形器
  * @author     : ZhangYi
  * @version    : None
  * @date       : 2024/10/24
  ******************************************************************************
  */
//

#include <stddef.h> 
#include <math.h>
#include <stdio.h>

#include "SKA_InputShaper.h"
#include "SKA_Constants.h"

int8_t SKA_InpSha_Init(SKA_InputShaper *pIS, SKA_InputShaperType eType, double dNaturalFreq, double dDampingRatio) {
    /******************** 函数参数合法性检验 ********************/
    if (pIS == NULL || dNaturalFreq <= 0 || dDampingRatio < 0 || dDampingRatio >= 1) {
        printf("Invalid parameters of SKA_InpSha_Init().\n");
        return -1;
    }

    /******************** 初始化 ********************/

    // 阻尼振荡周期
    double dTd = 2.0 * SKA_PI / (dNaturalFreq * sqrt(1.0 - pow(dDampingRatio, 2)));
    double dK = exp(-dDampingRatio * SKA_PI / sqrt(1.0 - powf(dDampingRatio, 2)));
    switch (eType){
    case SKA_IS_ZV: {
        pIS->nImpulseNum = 2;

        pIS->arrImpulseTime[0] = 0.0;
        pIS->arrImpulseTime[1] = 0.5 * dTd;

        pIS->arrImpulseValue[0] = 1.0 / (1.0 + dK);
        pIS->arrImpulseValue[1] = dK / (1.0 + dK);

        break;
    }
    case SKA_IS_ZVD: {
        pIS->nImpulseNum = 3;

        pIS->arrImpulseTime[0] = 0.0;
        pIS->arrImpulseTime[1] = 0.5 * dTd;
        pIS->arrImpulseTime[2] = 1.0 * dTd;
        
        double dD = 1.0 + 2.0 * dK + pow(dK, 2);
        pIS->arrImpulseValue[0] = 1.0 / dD;
        pIS->arrImpulseValue[1] = 2.0 * dK / dD;
        pIS->arrImpulseValue[2] = pow(dK, 2) / dD;

        break;
    }
    case SKA_IS_ZVDD: {
        pIS->nImpulseNum = 4;

        pIS->arrImpulseTime[0] = 0.0;
        pIS->arrImpulseTime[1] = 0.5 * dTd;
        pIS->arrImpulseTime[2] = 1.0 * dTd;
        pIS->arrImpulseTime[3] = 1.5 * dTd;
        
        double dD = 1.0 + 3.0 * dK + 3.0 * pow(dK, 2) + pow(dK, 3);
        pIS->arrImpulseValue[0] = 1.0 / dD;
        pIS->arrImpulseValue[1] = 3.0 * dK / dD;
        pIS->arrImpulseValue[2] = 3.0 * pow(dK, 2) / dD;
        pIS->arrImpulseValue[3] = pow(dK, 3) / dD;

        break;
    }
    default:
        return -1;
    }

    return 0;
}
