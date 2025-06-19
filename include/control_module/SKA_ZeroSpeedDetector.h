/**
  ******************************************************************************
  * @file       : SKA_ZeroSpeedDetector.h
  * @brief      : 零速度检测器
  * @author     : ZhangYi
  * @version    : None
  * @date       : 2024/10/31
  ******************************************************************************
  */
//

#ifndef SKA2000_OHBC_CONTROLLER_SKA_ZEROSPEEDDETECTOR_H
#define SKA2000_OHBC_CONTROLLER_SKA_ZEROSPEEDDETECTOR_H

#include <stdint.h>
#include <stdbool.h>

#include "SKA_CircularBuffer.h"

typedef struct {
    double dTs;                             // 采样周期(s)
    double dTimeLen;                        // 检测时域长度(s)
    double dMaxSpeed;                       // 检测为零速度时允许的最大速度(m/s)
    SKA_CircularBuffer stPosCirBuf;         // 位置测量值的循环缓冲区
} SKA_ZeroSpeedDetector;

/**
 * @brief 创建零速度检测器
 * @param pZSD 零速度检测器的地址
 * @param dTs 采样周期(s)
 * @param dTimeLen 检测时域长度(s)
 * @param dMaxSpeed 检测为零速度时允许的最大速度(m/s)
 * @return 错误码 {0: 正常}
 */
int8_t SKA_ZSD_Create(SKA_ZeroSpeedDetector *pZSD, double dTs, double dTimeLen, double dMaxSpeed);

/**
 * @brief 运行零速度检测器
 * @param pZSD 零速度检测器的地址
 * @param dCurPos 当前位置(m)
 * @param pIsZeroSpeed 存储检测结果的地址
 * @return 错误码 {0: 正常}
 */
int8_t SKA_ZSD_Run(SKA_ZeroSpeedDetector *pZSD, double dCurPos, bool *pIsZeroSpeed);

/**
 * @brief 销毁零速度检测器
 * @param pZSD 零速度检测器的地址
 * @return 错误码 {0: 正常}
*/
int8_t SKA_ZSD_Destroy(SKA_ZeroSpeedDetector *pZSD);

#endif //SKA2000_OHBC_CONTROLLER_SKA_ZEROSPEEDDETECTOR_H