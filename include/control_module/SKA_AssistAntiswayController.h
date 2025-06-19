/**
  ******************************************************************************
  * @file       : SKA_AssistAntiswayController.h
  * @brief      : 辅助防摇控制器
  * @author     : ZhangYi
  * @version    : None
  * @date       : 2024/10/31
  ******************************************************************************
  */
//

#ifndef SKA2000_OHBC_CONTROLLER_SKA_ASSISTANTISWAYCONTROLLER_H
#define SKA2000_OHBC_CONTROLLER_SKA_ASSISTANTISWAYCONTROLLER_H

#include <stdint.h>
#include <stdbool.h>

#include "SKA_Saturation.h"
#include "SKA_RateLimiter.h"
#include "SKA_InertialFilter.h"
#include "SKA_CircularBuffer.h"
#include "SKA_InputShaper.h"

// 辅助防摇
typedef struct {
    bool bActiveSwayCtrl;                   // 激活防摇控制
    double dTs;                             // 采样周期(s)
    double dMaxVelocity;                    // 最大速度(m/s)
    SKA_RateLimiter stRL;                   // 速率限制器
    SKA_InertialFilter stIF;                // 惯性滤波器
    SKA_CircularBuffer stCB;                // 循环缓冲区
    SKA_InputShaper stIS;                   // 输入整形器
} SKA_AssistAntiswayController;

/** 
 * @brief 创建辅助防摇控制器
 * @param pAAC 辅助防摇控制器
 * @param bActiveSwayCtrl 激活防摇控制
 * @param dTs 采样周期(s)
 * @param dMaxVelocity 最大速度(m/s)
 * @param dMaxAcceleration 最大加速度(m/s^2)
 * @param dIneFilTc 惯性滤波器时间常数(s)
 * @param dInitialVelocity 初始速度(m/s)
 * @return 错误码{0: 正常}
 */
int8_t SKA_AAC_Create(SKA_AssistAntiswayController *pAAC, bool bActiveSwayCtrl, double dTs, 
                      double dMaxVelocity, double dMaxAcceleration,double dIneFilTc, double dInitialVelocity);

/**
 * @brief 运行辅助防摇控制器
 * @param pAAC 辅助防摇控制器
 * @param dManualVelocity 手动操作的速度(m/s)
 * @param eType 输入整形器类型
 * @param dNaturalFreq 自然频率(Hz)
 * @param dDampingRatio 阻尼比
 * @param pOutputVelocity 存储输出速度(m/s)的地址
 * @return 错误码{0: 正常}
 */
int8_t SKA_AAC_Run(SKA_AssistAntiswayController *pAAC, double dManualVelocity, 
                   SKA_InputShaperType eType, double dNaturalFreq, double dDampingRatio, 
                   double *pOutputVelocity);

/**
 * @brief 设置辅助防摇控制器的最大速度
 * @param pAAC 辅助防摇控制器
 * @param dMaxVelocity 最大速度(m/s)
 * @return 错误码{0: 正常}
 */
int8_t SKA_AAC_SetMaxVelocity(SKA_AssistAntiswayController *pAAC, double dMaxVelocity);

/**
 * @brief 销毁辅助防摇控制器
 * @param pAAC 辅助防摇控制器
 * @return 错误码{0: 正常}
 */
int8_t SKA_AAC_Destroy(SKA_AssistAntiswayController *pAAC);


#endif //SKA2000_OHBC_CONTROLLER_SKA_ASSISTANTISWAYCONTROLLER_H