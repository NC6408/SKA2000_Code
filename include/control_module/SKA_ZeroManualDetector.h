/**
  ******************************************************************************
  * @file       : SKA_ZeroManualDetector.h
  * @brief      : 零手动输入检测器
  * @author     : ZhangYi
  * @version    : None
  * @date       : 2024/11/01
  ******************************************************************************
  */
//

#ifndef SKA2000_OHBC_CONTROLLER_SKA_ZEROMANUALDETECTOR_H
#define SKA2000_OHBC_CONTROLLER_SKA_ZEROMANUALDETECTOR_H

#include <stdint.h>
#include <stdbool.h>

#include "SKA_CircularBuffer.h"

typedef struct {
    double dTs;                             // 采样周期(s)
    double dTimeLen;                        // 检测时域长度(s)
    SKA_CircularBuffer stCmdCirBuf;         // 手动输入指令的循环缓冲区
} SKA_ZeroManualDetector;

/**
 * @brief 创建零手动输入检测器
 * @param pZMD 零手动输入检测器的地址
 * @param dTs 采样周期(s)
 * @param dTimeLen 检测时域长度(s)
 * @return 错误码 {0: 正常}
 */
int8_t SKA_ZMD_Create(SKA_ZeroManualDetector *pZMD, double dTs, double dTimeLen);

/**
 * @brief 运行零手动输入检测器
 * @param pZMD 零手动输入检测器的地址
 * @param dCurManualInput 当前手动输入
 * @param pIsZeroManual 存储检测结果的地址
 * @return 错误码 {0: 正常}
 */
int8_t SKA_ZMD_Run(SKA_ZeroManualDetector *pZMD, double dCurManualInput, bool *pIsZeroManual);

/**
 * @brief 销毁零手动输入检测器
 * @param pZMD 零手动输入检测器的地址
 * @return 错误码 {0: 正常}
*/
int8_t SKA_ZMD_Destroy(SKA_ZeroManualDetector *pZMD);

#endif //SKA2000_OHBC_CONTROLLER_SKA_ZEROMANUALDETECTOR_H