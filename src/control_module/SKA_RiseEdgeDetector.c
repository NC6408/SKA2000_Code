/**
  ******************************************************************************
  * @file       : SKA_RiseEdgeDetector.c
  * @brief      : 上升沿检测器
  * @author     : ZhangYi
  * @version    : None
  * @date       : 2024/10/24
  ******************************************************************************
  */
//

#include <stddef.h> 
#include <stdio.h>

#include "SKA_RiseEdgeDetector.h"

int8_t SKA_RED_Init(SKA_RiseEdgeDetector *pRED) {
    /******************** 函数参数合法性检验 ********************/
    if (pRED == NULL) {
        printf("Invalid parameters of SKA_RED_Init().\n");
        return -1;
    }

    /******************** 初始化 ********************/
    pRED->bPrevInput = 1;

    return 0;
}


int8_t SKA_RED_Run(SKA_RiseEdgeDetector *pRED, bool bInput, bool *pResult) {
    /******************** 函数参数合法性检验 ********************/
    if (pRED == NULL) {
        printf("Invalid parameters of SKA_RED_Run().\n");
        return -1;
    }

    /******************** 上升沿检测 ********************/
    if (pRED->bPrevInput == 0 && bInput == 1) {
        *pResult = 1;
    } else {
        *pResult = 0;
    }
    pRED->bPrevInput = bInput;

    return 0;
}