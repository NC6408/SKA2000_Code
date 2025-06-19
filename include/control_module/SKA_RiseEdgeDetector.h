/**
  ******************************************************************************
  * @file       : SKA_RiseEdgeDetector.h
  * @brief      : 上升沿检测器
  * @author     : ZhangYi
  * @version    : None
  * @date       : 2024/10/24
  ******************************************************************************
  */
//

#ifndef SKA2000_OHBC_CONTROLLER_SKA_RISEEDGEDETECTOR_H
#define SKA2000_OHBC_CONTROLLER_SKA_RISEEDGEDETECTOR_H

#include <stdbool.h>
#include <stdint.h>

// 上升沿检测器
typedef struct {
    bool bPrevInput;    // 上次输入
} SKA_RiseEdgeDetector;
 
/**
 * @brief 上升沿检测器初始化
 * @param pRED 上升沿检测器的地址
 * @return 错误码 {0: 正常}
 */
int8_t SKA_RED_Init(SKA_RiseEdgeDetector *pRED);

/**
 * @brief 上升沿检测器运行
 * @param pRED 上升沿检测器的地址
 * @param bInput 输入
 * @param pResult 存储结果的地址
 * @return 错误码 {0: 正常}
 */
int8_t SKA_RED_Run(SKA_RiseEdgeDetector *pRED, bool bInput, bool *pResult);

#endif //SKA2000_OHBC_CONTROLLER_SKA_RISEEDGEDETECTOR_H
