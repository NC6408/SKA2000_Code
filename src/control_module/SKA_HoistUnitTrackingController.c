/**
  ******************************************************************************
  * @file       : SKA_HoistUnitTrackingController.c
  * @brief      : 起升机构的跟踪控制器
  * @author     : ZhangYi
  * @version    : None
  * @date       : 2024/10/24
  ******************************************************************************
  */
//

#include <stddef.h> 
#include <math.h>
#include <stdio.h>

#include "SKA_Constants.h"
#include "SKA_HoistUnitTrackingController.h"

int8_t SKA_HUTC_Init(SKA_HoistUnitTrackingController *pHUTC, bool bActivePosCtrl, double dTs, double dDelayTime, double dStartPos,
                     double dMaxTrackErr, double dMaxTrackErrRate, double dTrackErrFilTc,
                     double dKp, double dKi, double dMaxVi, double dPosDeadZone,
                     double dMaxVelocity, double dMaxAcceleration, double dMaxFreq, double dMinFreq, double dFreqCompenSoft) {
    /******************** 函数参数合法性检验 ********************/
    if (pHUTC == NULL || dTs <= 0 || dDelayTime < 0 || dMaxTrackErr < 0 || dMaxTrackErrRate < 0 ||
        dTrackErrFilTc < 0 || dKp < 0 || dKi < 0 || dMaxVi < 0 || dPosDeadZone < 0 || 
        dMaxVelocity <= 0 || dMaxAcceleration <= 0 || dMaxFreq <= 0 || dMinFreq < 0 || dFreqCompenSoft < 0 || dFreqCompenSoft > 1) {
        printf("Invalid parameters of SKA_HUTC_Init().\n");
        return -1;
    }

    /******************** 初始化 ********************/

    pHUTC->bActivePosCtrl = bActivePosCtrl;
    pHUTC->dTs = dTs;

    SKA_IneFil_Init(&(pHUTC->stDelayer), dTs, dDelayTime, dStartPos);
    pHUTC->dMaxTrackErr = dMaxTrackErr;
    SKA_RatLim_Init(&(pHUTC->stTrackErrRL), dMaxTrackErrRate, -dMaxTrackErrRate, dTs, 0.0);
    SKA_IneFil_Init(&(pHUTC->stTrackErrIF), dTs, dTrackErrFilTc, 0.0);

    pHUTC->dKp = dKp;
    pHUTC->dKi = dKi;
    pHUTC->dMaxVi = dMaxVi;
    pHUTC->dPosDeadZone = dPosDeadZone;
    pHUTC->bOpenIntegration = 0;
    pHUTC->dTrackErrInteg = 0.0;

    pHUTC->dMaxVelocity = dMaxVelocity;
    SKA_RatLim_Init(&(pHUTC->stVelocityRL), dMaxAcceleration, -dMaxAcceleration, dTs, 0.0);

    pHUTC->dMaxFreq = dMaxFreq;
    pHUTC->dMinFreq = dMinFreq;
    pHUTC->dFreqCompenSoft = dFreqCompenSoft;

    return 0;
}

int8_t SKA_HUTC_Run(SKA_HoistUnitTrackingController *pHUTC,
              double dRefPos, double dRefVel, double dRefAcc, double dCurPos,
              double *pOutputVel, double *pOutputFreq, int8_t *pDirection) {
    /******************** 函数参数合法性检验 ********************/
    if (pHUTC == NULL || pOutputVel == NULL || pOutputFreq == NULL || pDirection == NULL) {
        printf("Invalid parameters of SKA_HUTC_Run().\n");
        return -1;
    }

    /******************** 运行跟踪控制器 ******************** */

    /** 前馈控制 **/

    // 前馈控制量
    double dFeedforward = dRefVel;

    /** 反馈控制 **/

    // 延迟时间补偿
    double dRefPosDc;
    SKA_IneFil_Run(&(pHUTC->stDelayer), dRefPos, &dRefPosDc);
    
    // 位置误差预处理
    // 原始位置误差
    pHUTC->dOriTrackError = dRefPosDc - dCurPos;
    // 限幅滤波
    double dSatTrackError;
    SKA_Sat_Run(pHUTC->dOriTrackError, -pHUTC->dMaxTrackErr, pHUTC->dMaxTrackErr, &dSatTrackError);
    // 限速滤波
    double fRatLimTrackErr;
    SKA_RatLim_Run(&(pHUTC->stTrackErrRL), dSatTrackError, &fRatLimTrackErr);
    // 惯性滤波
    SKA_IneFil_Run(&(pHUTC->stTrackErrIF), fRatLimTrackErr, &(pHUTC->dFilTrackError));

    double dFeedback;
    if (pHUTC->bActivePosCtrl){
        // 激活闭环定位
        
        // PI控制器
        // 比例环节
        pHUTC->dVp = pHUTC->dKp * pHUTC->dFilTrackError;
        // 积分环节
        // 积分分离
        if (pHUTC->bOpenIntegration == 1) {
            // 启动积分
            // 避免积分饱和
            if (fabs(pHUTC->dKi * pHUTC->dTrackErrInteg) < pHUTC->dMaxVi) {
                pHUTC->dTrackErrInteg += pHUTC->dTs * pHUTC->dFilTrackError;
            } else {
                if (pHUTC->dFilTrackError * pHUTC->dTrackErrInteg < 0) {
                    pHUTC->dTrackErrInteg += pHUTC->dTs * pHUTC->dFilTrackError;
                } else {
                    // 积分饱和
                }
            }
        } else {
            // 关闭积分
            pHUTC->dTrackErrInteg = 0.0;
        }
        pHUTC->dVi = pHUTC->dKi * pHUTC->dTrackErrInteg;

        // 反馈控制量
        if(fabs(pHUTC->dFilTrackError) < pHUTC->dPosDeadZone){
            // 反馈控制死区
            dFeedback = 0.0;
            pHUTC->dTrackErrInteg = 0.0;
        }else{
            dFeedback = pHUTC->dVp + pHUTC->dVi;
        }
    } else {
        // 未激活闭环定位
        dFeedback = 0.0;
    }

    /** 控制量处理 **/

    // 复合控制量
    double dVsum = dFeedforward + dFeedback;
    // 速度限制
    double dVsat;
    SKA_Sat_Run(dVsum, -pHUTC->dMaxVelocity, pHUTC->dMaxVelocity, &dVsat);
    // 加速度限制
    double dVrl;
    SKA_RatLim_Run(&(pHUTC->stVelocityRL), dVsat, &dVrl);
    // 速度控制量
    double dUv = fabs(dVrl) < SKA_FlOAT_ERROR ? 0.0 : dVrl;
    
    // 输出运动方向
    if (dUv == 0.0) {
        *pDirection = 0;
    } else if (dUv > 0) {
        *pDirection = 1;
    } else {
        *pDirection = -1;
    }
    // 输出速度和频率
    // 速度-频率比例系数
    double fRatio = pHUTC->dMaxVelocity / pHUTC->dMaxFreq;
    *pOutputVel = dUv;
    // 控制量死区补偿
    if (*pDirection != 0) {
        // 最小能动频率对应的速度
        double dMinVelocity = pHUTC->dMinFreq * fRatio;
        if(fabs(dUv) < dMinVelocity){
            *pOutputVel = *pDirection * dMinVelocity * pow(fabs(dUv) / dMinVelocity, pHUTC->dFreqCompenSoft);
        }
    }
    *pOutputFreq = *pOutputVel / fRatio;

    return 0;
}

int8_t SKA_HUTC_GetPosTrackError(SKA_HoistUnitTrackingController *pHUTC, double *pPosTrackError) {
    /******************** 函数参数合法性检验 ********************/
    if (pHUTC == NULL || pPosTrackError == NULL) {
        printf("Invalid parameters of SKA_HUTC_GetPosTrackError().\n");
        return -1;
    }

    /******************** 获取位置跟踪误差 ******************** */
    if (pHUTC->bActivePosCtrl) {
        // 激活闭环定位控制
        *pPosTrackError = pHUTC->dOriTrackError;
    } else {
        // 未激活闭环定位控制
        *pPosTrackError = 0.0;
    }

    return 0;
}