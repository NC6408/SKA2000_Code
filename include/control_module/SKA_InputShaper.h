/**
  ******************************************************************************
  * @file       : SKA_InputShaper.h
  * @brief      : 输入整形器
  * @author     : ZhangYi
  * @version    : None
  * @date       : 2024/10/24
  ******************************************************************************
  */
//

#ifndef SKA2000_OHBC_CONTROLLER_SKA_INPUTSHAPER_H
#define SKA2000_OHBC_CONTROLLER_SKA_INPUTSHAPER_H

#include <stdint.h>

#include "SKA_Constants.h"

// 输入整形器的类型
typedef enum {
    SKA_IS_ZV,          // ZV
    SKA_IS_ZVD,         // ZVD
    SKA_IS_ZVDD         // ZVDD
} SKA_InputShaperType;

// 输入整形器
typedef struct {
    uint8_t nImpulseNum;                // 脉冲数量
    double arrImpulseTime[10];          // 脉冲时间
    double arrImpulseValue[10];         // 脉冲幅值
} SKA_InputShaper;

/**
  * @brief  初始化输入整形器
  * @param  pIS 输入整形器地址
  * @param  eType 输入整形器类型
  * @param  dNaturalFreq 自然频率(rad/s)
  * @param  dDampingRatio 阻尼比
  * @return 错误码 {0: 正常}
  */
int8_t SKA_InpSha_Init(SKA_InputShaper *pIS, SKA_InputShaperType eType, double dNaturalFreq, double dDampingRatio);

#endif //SKA2000_OHBC_CONTROLLER_SKA_INPUTSHAPER_H
