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

#ifndef SKA2000_OHBC_CONTROLLER_SKA_UTILS_H
#define SKA2000_OHBC_CONTROLLER_SKA_UTILS_H

#include <stdint.h>

/**
  * @brief  求最大值
  * @param  v1 值1
  * @param  v2 值2
  * @param  pMax 最大值的地址
  * @return 错误码 {0: 正常}
  */
int8_t SKA_Max(double v1, double v2, double* pMax);

/**
  * @brief  求最小值
  * @param  v1 值1
  * @param  v2 值2
  * @param  pMin 最小值的地址
  * @return 错误码 {0: 正常}
  */
int8_t SKA_Min(double v1, double v2, double* pMin);

#endif // SKA2000_OHBC_CONTROLLER_SKA_UTILS_H