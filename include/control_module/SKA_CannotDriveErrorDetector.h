/**
  ******************************************************************************
  * @file       : SKA_CannotDriveErrorDetector.h
  * @brief      : 无法驱动错误检测器
  * @author     : ZhangYi
  * @version    : None
  * @date       : 2024/11/06
  ******************************************************************************
  */
//

#ifndef SKA2000_OHBC_CONTROLLER_SKA_CANNOTDRIVEERRORDETECTOR_H
#define SKA2000_OHBC_CONTROLLER_SKA_CANNOTDRIVEERRORDETECTOR_H

#include <stdint.h>
#include <stdbool.h>

#include "SKA_CircularBuffer.h"

// 无法驱动错误检测器
typedef struct {
    double dTs;                             // 采样周期(s)
    double dTimeLen;                        // 检测时域长度(s)
    double dPosAccuracy;                    // 位置测量值的精度(m), 静止时的位置测量值变化量不超过该值
    double dFreqThreshold;                  // 启动检测的频率阈值(Hz)
    SKA_CircularBuffer stPosCirBuf;         // 位置测量值的循环缓冲区
    SKA_CircularBuffer stFeqCirBuf;         // 频率输出值的循环缓冲区
} SKA_CannotDriveErrorDetector;

/**
 * @brief 创建无法驱动错误检测器
 * @param pCDED 无法驱动错误检测器的地址
 * @param dTs 采样周期(s)
 * @param dTimeLen 检测时域长度(s)
 * @param dPosAccuracy 位置测量值的精度(m)
 * @param dFreqThreshold 频率阈值(Hz)
 * @return 错误码 {0: 正常}
 */
int8_t SKA_CDED_Create(SKA_CannotDriveErrorDetector *pCDED, double dTs, double dTimeLen, double dPosAccuracy, double dFreqThreshold);

/**
 * @brief 运行无法驱动错误检测器
 * @param pCDED 无法驱动错误检测器的地址
 * @param dCurPos 当前位置(m)
 * @param dCurFreq 当前频率(Hz)
 * @param pCannotDrive 存储检测结果的地址
 * @return 错误码 {0: 正常}
 */
int8_t SKA_CDED_Run(SKA_CannotDriveErrorDetector *pCDED, double dCurPos, double dCurFreq, bool *pCannotDrive);

/**
 * @brief 销毁无法驱动错误检测器
 * @param pCDED 无法驱动错误检测器的地址
 * @return 错误码 {0: 正常}
 */
int8_t SKA_CDED_Destroy(SKA_CannotDriveErrorDetector *pCDED);

#endif //SKA2000_OHBC_CONTROLLER_SKA_CANNOTDRIVEERRORDETECTOR_H
