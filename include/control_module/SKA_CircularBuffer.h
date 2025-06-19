/**
  ******************************************************************************
  * @file       : SKA_CircularBuffer.h
  * @brief      : 循环缓冲区
  * @author     : ZhangYi
  * @version    : None
  * @date       : 2024/10/24
  ******************************************************************************
  */
//

#ifndef SKA2000_OHBC_CONTROLLER_SKA_CIRCULARBUFFER_H
#define SKA2000_OHBC_CONTROLLER_SKA_CIRCULARBUFFER_H

#include <stdint.h>

// 循环缓冲区
typedef struct {
    uint16_t nCapacity;             // 缓冲区容量
    uint16_t nSize;                 // 缓冲区大小(实际存储元素数量)
    double *pBuffer;                // 缓冲区指针
    uint16_t nWriteIndex;           // 写指针
} SKA_CircularBuffer;

/**
 * @brief 创建循环缓冲区
 * @param pCirBuf 循环缓冲区
 * @param nCapacity 缓冲区容量
 * @return 错误码 {0: 正常}
 */
uint8_t SKA_CirBuf_Create(SKA_CircularBuffer *pCirBuf, uint16_t nCapacity);

/**
 * @brief 初始化循环缓冲区
 * @param pCirBuf 循环缓冲区
 * @param dInitValue 初始化值
 * @return 错误码 {0: 正常}
 */
uint8_t SKA_CirBuf_Init(SKA_CircularBuffer *pCirBuf, double dInitValue);

/**
 * @brief 添加数据到循环缓冲区
 * @param pCirBuf 循环缓冲区
 * @param dNewValue 新数据
 * @return 错误码 {0: 正常}
 */
uint8_t SKA_CirBuf_Add(SKA_CircularBuffer *pCirBuf, double dNewValue);

/**
 * @brief 读取循环缓冲区的历史数据
 * @param pCirBuf 循环缓冲区
 * @param nOffset 读取偏移, 0表示最新数据
 * @param pValue 读取值
 * @return 数据
 */
uint8_t SKA_CirBuf_Get(SKA_CircularBuffer *pCirBuf, uint16_t nOffset, double *pValue);

/**
 * @brief 销毁循环缓冲区
 * @param pCirBuf 循环缓冲区
 * @return 错误码 {0: 正常}
 */
uint8_t SKA_CirBuf_Destroy(SKA_CircularBuffer *pCirBuf);

#endif //SKA2000_OHBC_CONTROLLER_SKA_CIRCULARBUFFER_H
