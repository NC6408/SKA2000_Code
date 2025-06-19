/**
  ******************************************************************************
  * @file       : SKA_RateLimiter.h
  * @brief      : 速率限制器
  * @author     : ZhangYi
  * @version    : None
  * @date       : 2024/10/24
  ******************************************************************************
  */
//

#ifndef SKA2000_OHBC_CONTROLLER_SKA_RATELIMITER_H
#define SKA2000_OHBC_CONTROLLER_SKA_RATELIMITER_H

#include <stdint.h>

// 速率限制器
typedef struct {
    double dRiseRate;       // 最大速率
    double dFallRate;       // 最小速率
    double dTs;             // 采样周期(s)
    double dPrevOutput;     // 上次输出值
} SKA_RateLimiter;

/**
  * @brief  初始化速率限制器
  * @param  pRL 速率限制器的地址
  * @param  dRiseRate 最大速率
  * @param  dFallRate 最小速率
  * @param  dTs 采样周期(s)
  * @param  dPrevOutput 上次输出值
  * @return 错误码 {0: 正常}
  */
int8_t SKA_RatLim_Init(SKA_RateLimiter *pRL, double dRiseRate, double dFallRate, double dTs, double dPrevOutput);

/**
  * @brief  运行速率限制器
  * @param  pRL 速率限制器的地址
  * @param  dInput 输入值
  * @param  pOutput 输出值的地址
  * @return 错误码 {0: 正常}
  */
int8_t SKA_RatLim_Run(SKA_RateLimiter *pRL, double dInput, double *pOutput);

#endif //SKA2000_OHBC_CONTROLLER_SKA_RATELIMITER_H
