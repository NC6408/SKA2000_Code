/**
  ******************************************************************************
  * @file       : SKA_Utils.c
  * @brief      : 工具函数
  * @author     : ZhangYi
  * @version    : None
  * @date       : 2024/11/04
  ******************************************************************************
  */
//

#include <stddef.h> 

#include "SKA_Utils.h"

int8_t SKA_Max(double v1, double v2, double* pMax) {
    if (v1 >= v2) {
        *pMax = v1;
    } else {
        *pMax =  v2;
    }
    return 0;
}

int8_t SKA_Min(double v1, double v2, double* pMin) {
    if (v1 >= v2) {
        *pMin = v2;
    } else {
        *pMin =  v1;
    }
    return 0;
}