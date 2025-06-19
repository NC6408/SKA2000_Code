/**
  ******************************************************************************
  * @file       : SKA_ZeroManualDetector.c
  * @brief      : 零手动输入检测器
  * @author     : ZhangYi
  * @version    : None
  * @date       : 2024/11/01
  ******************************************************************************
  */
//

#include <stddef.h> 
#include <math.h>
#include <stdio.h>

#include "SKA_ZeroManualDetector.h"
#include "SKA_Constants.h"

int8_t SKA_ZMD_Create(SKA_ZeroManualDetector *pZMD, double dTs, double dTimeLen) {
    /******************** 函数参数合法性检验 ********************/
    if (pZMD == NULL || dTs <= 0 || dTimeLen <= 0) {
        printf("Invalid parameters of SKA_ZMD_Create().\n");
        return -1;
    }

    /******************** 创建零手动输入检测器 ********************/
    pZMD->dTs = dTs;
    pZMD->dTimeLen = dTimeLen;
    uint16_t nCap = (uint16_t)ceil(dTimeLen / dTs) + 1;
    SKA_CirBuf_Create(&pZMD->stCmdCirBuf, nCap);

    return 0;
}

int8_t SKA_ZMD_Run(SKA_ZeroManualDetector *pZMD, double dCurManualInput, bool *pIsZeroManual) {
    /******************** 函数参数合法性检验 ********************/
    if (pZMD == NULL || pIsZeroManual == NULL) {
        printf("Invalid parameters of SKA_ZMD_Run().\n");
        return -1;
    }

    /******************** 运行零手动输入检测器 ********************/

    // 存储当前手动输入指令
    SKA_CirBuf_Add(&pZMD->stCmdCirBuf, dCurManualInput);

    // 判断检测时域内的手动输入指令是否符合零输入要求
    if (pZMD->stCmdCirBuf.nSize < pZMD->stCmdCirBuf.nCapacity) {
        // 数据量不足
        *pIsZeroManual = 0;
    } else {
        *pIsZeroManual = 1;
        for (uint16_t i = 0; i < pZMD->stCmdCirBuf.nCapacity; i++) {
            double dManual;
            SKA_CirBuf_Get(&(pZMD->stCmdCirBuf), i, &dManual);
            if (fabs(dManual) > SKA_FlOAT_ERROR) {
                *pIsZeroManual = 0;
                break;
            }
        }
    }

    return 0;
}

int8_t SKA_ZMD_Destroy(SKA_ZeroManualDetector *pZMD) {
    /******************** 函数参数合法性检验 ********************/
    if (pZMD == NULL) {
        printf("Invalid parameters of SKA_ZMD_Destroy().\n");
        return -1;
    }

    /******************** 销毁零手动输入检测器 ********************/
    SKA_CirBuf_Destroy(&pZMD->stCmdCirBuf);

    return 0;
}