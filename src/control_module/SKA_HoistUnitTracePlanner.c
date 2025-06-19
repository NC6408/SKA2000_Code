/**
  ******************************************************************************
  * @file       : SKA_HoistUnitTracePlanner.c
  * @brief      : 起升机构的轨迹规划器
  * @author     : ZhangYi
  * @version    : None
  * @date       : 2024/10/24
  ******************************************************************************
  */
//

#include <stddef.h> 
#include <math.h>
#include <stdio.h>

#include "SKA_HoistUnitTracePlanner.h"
#include "SKA_Constants.h"

int8_t SKA_HUTP_Create(SKA_HoistUnitTracePlanner *pHUTP, double dStartPos, double dTargetPos, 
                       double dMaxVelocity, double dMaxAcceleration, double dMaxJerk) {
    /******************** 函数参数合法性检验 ********************/
    if (pHUTP == NULL || dMaxVelocity <= 0 || dMaxAcceleration <= 0 || dMaxJerk <= 0) {
        printf("Invalid parameters of SKA_HUTP_Create().\n");
        return -1;
    }

    /******************** 创建运行机构轨迹规划器 ********************/
    SKA_UTP_Create(&pHUTP->stUnsPlanner, dStartPos, dTargetPos, dMaxVelocity, dMaxAcceleration, dMaxJerk, 0.0);

    return 0;
}

int8_t SKA_HUTP_Run(SKA_HoistUnitTracePlanner *pHUTP, double dCurTime,
                    double *pRefPos, double *pRefVel, double *pRefAcc) {
    /******************** 函数参数合法性检验 ********************/
    if (pHUTP == NULL || pRefPos == NULL || pRefVel == NULL || pRefAcc == NULL) {
        printf("Invalid parameters of SKA_HUTP_Run().\n");
        return -1;
    }

    /******************** 运行运行机构轨迹规划器 ********************/

    SKA_UTP_Run(&(pHUTP->stUnsPlanner), dCurTime, pRefPos, pRefVel, pRefAcc);

    return 0;
}

int8_t SKA_HUTP_GetTotalTime(SKA_HoistUnitTracePlanner *pHUTP, double *pTotalTime) {
    /******************** 函数参数合法性检验 ********************/
    if (pHUTP == NULL || pTotalTime == NULL) {
        printf("Invalid parameters of SKA_HUTP_GetTotalTime().\n");
        return -1;
    }

    /******************** 获取运行机构轨迹规划器总时间 ********************/
    SKA_UTP_GetTotalTime(&(pHUTP->stUnsPlanner), pTotalTime);

    return 0;
}

int8_t SKA_HUTP_ModifyTargetPos(SKA_HoistUnitTracePlanner *pHUTP, double dCurTime, double dNewTargetPos) {
    /******************** 函数参数合法性检验 ********************/
    if (pHUTP == NULL) {
        printf("Invalid parameters of SKA_HUTP_ModifyTargetPos().\n");
        return -1;
    }

    /******************** 目标位置切换 ********************/
    SKA_UTP_ModifyTargetPos(&(pHUTP->stUnsPlanner), dCurTime, dNewTargetPos);

    return 0;
}

int8_t SKA_HUTP_QuickStop(SKA_HoistUnitTracePlanner *pHUTP, double dCurTime) {
    /******************** 函数参数合法性检验 ********************/
    if (pHUTP == NULL) {
        printf("Invalid parameters of SKA_HUTP_QuickStop().\n");
        return -1;
    }

    /******************** 急停 ********************/
    SKA_UTP_UrgencyStop(&(pHUTP->stUnsPlanner), dCurTime);

    return 0;
}

int8_t SKA_HUTP_GetAscBrakeDistance(SKA_HoistUnitTracePlanner *pHUTP, double dCurTime, double *pAscBrakeDistance) {
    /******************** 函数参数合法性检验 ********************/
    if (pHUTP == NULL || pAscBrakeDistance == NULL) {
        printf("Invalid parameters of SKA_HUTP_GetAscBrakeDistance().\n");
        return -1;
    }

    /******************** 计算制动距离 ********************/

    // 当前参考位置
    double dCurPos, dCurVel, dCurAcc;
    SKA_HUTP_Run(pHUTP, dCurTime, &dCurPos, &dCurVel, &dCurAcc);
    // 制动目标位置
    double dBrakeTargetPos;
    if (pHUTP->stUnsPlanner.bIsUrgencyStopping) {
        // 已触发急停
        SKA_HUTP_GetRealTargetPos(pHUTP, &dBrakeTargetPos);
    } else {
        // 预估无防摇急停的制动距离
        // 当前目标位置
        double dCurTargetPos;
        SKA_UTP_GetRealTargetPos(&(pHUTP->stUnsPlanner), &dCurTargetPos);
        // 急停
        SKA_UTP_UrgencyStop(&(pHUTP->stUnsPlanner), dCurTime);
        SKA_UTP_GetRealTargetPos(&(pHUTP->stUnsPlanner), &dBrakeTargetPos);
        // 清除急停标志
        pHUTP->stUnsPlanner.bIsUrgencyStopping = 0;
        // 还原目标位置
        SKA_UTP_ModifyTargetPos(&(pHUTP->stUnsPlanner), dCurTime, dCurTargetPos);

    }
    *pAscBrakeDistance = fabs(dBrakeTargetPos - dCurPos);

    return 0;
}

int8_t SKA_HUTP_GetRealTargetPos(SKA_HoistUnitTracePlanner *pHUTP, double *pRealTargetPos) {
    /******************** 函数参数合法性检验 ********************/
    if (pHUTP == NULL || pRealTargetPos == NULL) {
        printf("Invalid parameters of SKA_HUTP_GetRealTargetPos().\n");
        return -1;
    }

    /******************** 获取实际目标位置 ********************/
    SKA_UTP_GetRealTargetPos(&(pHUTP->stUnsPlanner), pRealTargetPos);

    return 0;
}

int8_t SKA_HUTP_ModifyMaxVelocity(SKA_HoistUnitTracePlanner *pHUTP, double dCurTime, double dNewMaxVelocity) {
    /******************** 函数参数合法性检验 ********************/
    if (pHUTP == NULL || dNewMaxVelocity <= 0) {
        printf("Invalid parameters of SKA_HUTP_ModifyMaxVelocity().\n");
        return -1;
    }

    /******************** 修改最大速度 ********************/
    // TODO

    return 0;
}

int8_t SKA_HUTP_Destroy(SKA_HoistUnitTracePlanner *pHUTP) {
    /******************** 函数参数合法性检验 ********************/
    if (pHUTP == NULL) {
        printf("Invalid parameters of SKA_HUTP_Destroy().\n");
        return -1;
    }

    /******************** 销毁运行机构轨迹规划器 ********************/
    SKA_UTP_Destroy(&pHUTP->stUnsPlanner);
    
    return 0;
}