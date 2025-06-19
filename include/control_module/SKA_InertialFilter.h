/**
  ******************************************************************************
  * @file       : SKA_InertialFilter.h
  * @brief      : 惯性环节
  * @author     : ZhangYi
  * @version    : None
  * @date       : 2024/10/24
  ******************************************************************************
  */
//

#ifndef SKA2000_OHBC_CONTROLLER_SKA_INERTIALFILTER_H
#define SKA2000_OHBC_CONTROLLER_SKA_INERTIALFILTER_H

#include <stdint.h>

// 惯性滤波器
typedef struct {
    double dTs;                 // 采样周期(s)
    double dTc;                 // 时间常数(s)
    double dPrevOutput;         // 上一次输出
} SKA_InertialFilter;

/**
  * @brief  初始化惯性滤波器
  * @param  pIneFil 惯性滤波器地址
  * @param  dTs 采样周期(s)
  * @param  dTc 时间常数(s)
  * @param  dInitOutput 初始输出
  * @return 错误码 {0: 正常}
  */
int8_t SKA_IneFil_Init(SKA_InertialFilter *pIneFil, double dTs,double dTc, double dInitOutput);

/**
  * @brief  运行惯性滤波器
  * @param  pIneFil 惯性滤波器地址
  * @param  fInput 输入值
  * @param  pOutput 输出值的地址
  * @return 错误码 {0: 正常}
  */
int8_t SKA_IneFil_Run(SKA_InertialFilter *pIneFil, double fInput, double *pOutput);

#endif //SKA2000_OHBC_CONTROLLER_SKA_INERTIALFILTER_H
