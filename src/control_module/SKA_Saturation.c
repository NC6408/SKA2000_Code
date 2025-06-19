/**
  ******************************************************************************
  * @file       : SKA_Saturation.c
  * @brief      : 饱和元件
  * @author     : ZhangYi
  * @version    : None
  * @date       : 2024/10/24
  ******************************************************************************
  */
//

#include <stddef.h> 
#include <stdio.h>

#include "SKA_Saturation.h"

int8_t SKA_Sat_Run(double dInput, double dMin, double dMax, double *pOutput) {
    /******************** 函数参数合法性检验 ********************/
    if (pOutput == NULL) {
        printf("Invalid parameters of SKA_Sat_Run().\n");
        return -1;
    }

    /******************** 运行 ********************/
    if (dInput < dMin) {
        *pOutput = dMin;
    } else if (dInput > dMax) {
        *pOutput = dMax;
    } else {
        *pOutput = dInput;
    }

    return 0;
}