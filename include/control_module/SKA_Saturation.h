/**
  ******************************************************************************
  * @file       : SKA_Saturation.h
  * @brief      : 饱和元件
  * @author     : ZhangYi
  * @version    : None
  * @date       : 2024/10/24
  ******************************************************************************
  */
//

#ifndef SKA2000_OHBC_CONTROLLER_SKA_SATURATION_H
#define SKA2000_OHBC_CONTROLLER_SKA_SATURATION_H

#include <stdint.h>

/**
 * @brief 运行饱和元件
 * @param dInput 输入
 * @param dMin 最小值
 * @param dMax 最大值
 * @param pOutput 输出值的地址
 * @return 错误码 {0: 正常}
 */
int8_t SKA_Sat_Run(double dInput, double dMin, double dMax, double *pOutput);

#endif //SKA2000_OHBC_CONTROLLER_SKA_SATURATION_H
