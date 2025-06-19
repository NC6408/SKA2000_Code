/**
  ******************************************************************************
  * @file       : SKA_CircularBuffer.c
  * @brief      : 循环缓冲区
  * @author     : ZhangYi
  * @version    : None
  * @date       : 2024/10/24
  ******************************************************************************
  */
//

#include <stddef.h> 
#include <stdlib.h>
#include <stdio.h>

#include "SKA_CircularBuffer.h"

uint8_t SKA_CirBuf_Create(SKA_CircularBuffer *pCirBuf, uint16_t nCapacity)
{
    /******************** 函数参数合法性检验 ********************/
    if (pCirBuf == NULL || nCapacity == 0) {
        printf("Invalid parameters of SKA_CirBuf_Create().\n");
        return -1;
    }

    /******************** 动态申请内存 ********************/
    pCirBuf->nCapacity = nCapacity;
    pCirBuf->nSize = 0;
    pCirBuf->pBuffer = (double *) malloc(nCapacity * sizeof(double));
    pCirBuf->nWriteIndex = 0;

    return 0;
}

uint8_t SKA_CirBuf_Init(SKA_CircularBuffer *pCirBuf, double dInitValue) {
    /******************** 函数参数合法性检验 ********************/
    if (pCirBuf == NULL) {
        printf("Invalid parameters of SKA_CirBuf_Init().\n");
        return -1;
    }

    /******************** 初始化 ********************/
    for (int i = 0; i < pCirBuf->nCapacity; i++) {
        pCirBuf->pBuffer[i] = dInitValue;
    }
    pCirBuf->nSize = 0;
    pCirBuf->nWriteIndex = 0;

    return 0;
}

uint8_t SKA_CirBuf_Add(SKA_CircularBuffer *pCirBuf, double dNewValue) {
    /******************** 函数参数合法性检验 ********************/
    if (pCirBuf == NULL) {
        printf("Invalid parameters of SKA_CirBuf_Add().\n");
        return -1;
    }

    /******************** 添加数据 ********************/
    pCirBuf->pBuffer[pCirBuf->nWriteIndex] = dNewValue;
    pCirBuf->nWriteIndex = ((uint16_t)((int32_t)(pCirBuf->nWriteIndex) + 1)) % pCirBuf->nCapacity;
    if (pCirBuf->nSize < pCirBuf->nCapacity) {
        pCirBuf->nSize++;
    }

    return 0;
}

uint8_t SKA_CirBuf_Get(SKA_CircularBuffer *pCirBuf, uint16_t nOffset, double *pValue) {
    /******************** 函数参数合法性检验 ********************/
    if (pCirBuf == NULL || pValue == NULL) {
        printf("Invalid parameters of SKA_CirBuf_Get().\n");
        return -1;
    }

    /******************** 获取数据 ********************/
    *pValue = pCirBuf->pBuffer[
        ((uint16_t)((int32_t)(pCirBuf->nWriteIndex) - (int32_t)1 - (int32_t)nOffset + (int32_t)(pCirBuf->nCapacity))) % pCirBuf->nCapacity];

    return 0;
}

uint8_t SKA_CirBuf_Destroy(SKA_CircularBuffer *pCirBuf) {
    /******************** 函数参数合法性检验 ********************/
    if (pCirBuf == NULL) {
        printf("Invalid parameters of SKA_CirBuf_Destroy().\n");
        return -1;
    }

    /******************** 释放内存 ********************/
    free(pCirBuf->pBuffer);
    pCirBuf->pBuffer = NULL;

    return 0;
}
